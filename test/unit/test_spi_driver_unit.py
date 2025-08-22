import types
import sys
import pytest


def _install_fake_pigpio(monkeypatch, *, ctrl3c_init=0x00, status_value=0x03, block_bytes=b""):
    """
    Install a fake 'pigpio' module in sys.modules that mimics the minimal SPI
    behavior used by hardware/spi_driver.SPIBus.

    - ctrl3c_init: initial value returned when reading CTRL3_C (0x12)
    - status_value: value returned when reading STATUS_REG (0x1E)
    - block_bytes: 12 bytes to return when doing a block read from 0x22..0x2D
    """
    class FakePi:
        def __init__(self):
            self.connected = True
            self._last_addr = None
            self._writes = []     # record [reg, val] pairs written
            self._closed = False
            self._stopped = False
            self._ctrl3c_init = ctrl3c_init
            self._status_value = status_value
            # default 12 bytes if not provided
            self._block_bytes = (
                block_bytes if block_bytes
                else bytes(range(0x22, 0x22 + 12))  # deterministic but arbitrary
            )

        # API expected by our driver
        def spi_open(self, channel, baud, mode):
            return 1  # handle

        def spi_write(self, handle, tx_bytes: bytes):
            # Writes happen in two ways:
            #  - write_reg: [reg, value]  -> capture as a register write
            #  - read(_block): [addr]     -> capture "last address" for the next spi_read
            if len(tx_bytes) == 2:
                # treat as register write
                self._writes.append(tuple(tx_bytes))
            elif len(tx_bytes) == 1:
                self._last_addr = tx_bytes[0]
            else:
                # shouldn't happen in our driver
                self._last_addr = tx_bytes[-1] if tx_bytes else None

        def spi_read(self, handle, nbytes: int):
            # Respond based on the last single-byte address written
            # Read flags: bit7=1 (read); auto-inc for block: bit6=1
            addr = self._last_addr or 0x00
            reg = addr & 0x3F  # low 6 bits (strip read/auto-inc flags)

            # CTRL3_C (0x12) single-byte read
            if reg == 0x12 and nbytes == 1:
                return (1, bytes([self._ctrl3c_init]))

            # STATUS_REG (0x1E) single-byte read
            if reg == 0x1E and nbytes == 1:
                return (1, bytes([self._status_value]))

            # OUTX_L_G block read (0x22) — expect 12 bytes
            if reg == 0x22 and nbytes == 12:
                return (12, bytes(self._block_bytes[:12]))

            # Fallback: zeros of requested length
            return (nbytes, bytes([0x00] * nbytes))

        def spi_close(self, handle):
            self._closed = True

        def stop(self):
            self._stopped = True

    fake_mod = types.ModuleType("pigpio")
    fake_mod.pi = FakePi

    monkeypatch.setitem(sys.modules, "pigpio", fake_mod)
    return fake_mod


@pytest.mark.unit
def test_spi_init_sets_bdu_when_unset(monkeypatch):
    """
    If CTRL3_C BDU bit (bit6) is not set, SPIBus.__init__ should set it.
    """
    # CTRL3_C starts at 0x00 (BDU clear); status ready; any block bytes
    _install_fake_pigpio(monkeypatch, ctrl3c_init=0x00, status_value=0x03)

    # Import after installing fake pigpio
    from hardware.spi_driver import SPIBus

    bus = SPIBus()
    # Access the fake pi to inspect writes
    fake_pi = bus._pi  # type: ignore[attr-defined]

    # Expect a write to CTRL3_C (0x12) with BDU bit set: old | 0x40
    writes = getattr(fake_pi, "_writes", [])
    assert (0x12, 0x40) in writes, f"Expected write to CTRL3_C setting BDU, got {writes}"

    bus.close()
    assert fake_pi._closed is True
    assert fake_pi._stopped is True


@pytest.mark.unit
def test_spi_init_skips_bdu_write_when_already_set(monkeypatch):
    """
    If CTRL3_C already has BDU set (bit6=1), __init__ should not write it again.
    """
    # CTRL3_C starts at 0x40 (BDU set)
    _install_fake_pigpio(monkeypatch, ctrl3c_init=0x40, status_value=0x03)

    from hardware.spi_driver import SPIBus

    bus = SPIBus()
    fake_pi = bus._pi  # type: ignore[attr-defined]

    writes = getattr(fake_pi, "_writes", [])
    # No write to CTRL3_C expected in this case
    assert all(not (reg == 0x12) for reg, _ in writes), f"Unexpected CTRL3_C write(s): {writes}"

    bus.close()
    assert fake_pi._closed is True
    assert fake_pi._stopped is True


@pytest.mark.unit
def test_imu_read_returns_6_bytes_reordered(monkeypatch):
    """
    imu_read() should poll STATUS (GDA|XLDA), perform one 12-byte burst from 0x22,
    and return 6 bytes in order: [AX_L, AX_H, AY_L, AY_H, GZ_L, GZ_H].
    """
    # Craft a deterministic 12-byte block:
    # [GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H, AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H]
    gx = (0x11, 0x11)
    gy = (0x22, 0x22)
    gz = (0x33, 0x33)
    ax = (0x44, 0x44)
    ay = (0x55, 0x55)
    az = (0x66, 0x66)
    block = bytes((*gx, *gy, *gz, *ax, *ay, *az))

    _install_fake_pigpio(monkeypatch, ctrl3c_init=0x00, status_value=0x03, block_bytes=block)

    from hardware.spi_driver import SPIBus

    bus = SPIBus()
    out = bus.imu_read()  # Option 1: SPI returns a new 6-byte buffer

    # Expect only [AX_L, AX_H, AY_L, AY_H, GZ_L, GZ_H]
    assert isinstance(out, (bytes, bytearray))
    assert len(out) == 6
    assert out == bytes([ax[0], ax[1], ay[0], ay[1], gz[0], gz[1]])

    # Close should call through to pigpio
    fake_pi = bus._pi  # type: ignore[attr-defined]
    bus.close()
    assert fake_pi._closed is True
    assert fake_pi._stopped is True

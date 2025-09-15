# test/unit/test_lsm6ds3tr_driver_mapping.py
import pytest

# Registers we care about (match driver)
CTRL3_C    = 0x12
STATUS_REG = 0x1E
OUTX_L_G   = 0x22

XLDA = 0x01
GDA  = 0x02


class FakePiHost:
    """
    Minimal pigpio.pi()-like shim:
      - spi_open / spi_close / spi_xfer
      - STATUS_REG returns XLDA|GDA
      - OUTX_L_G block read returns a supplied 12-byte block
      - CTRL3_C writes are captured (to confirm BDU set)
    """
    def __init__(self, *, block12: bytes, ctrl3c_init=0x00):
        assert len(block12) == 12
        self._open = False
        self._h = 1
        self._ctrl3c = ctrl3c_init
        self._writes = []
        self._block12 = block12

    def spi_open(self, channel: int, baud: int, flags: int):
        self._open = True
        return self._h

    def spi_close(self, handle: int):
        self._open = False

    def stop(self):
        self._open = False

    def spi_xfer(self, handle: int, tx: bytes):
        if not self._open:
            raise RuntimeError("SPI not open")
        if not tx:
            return 0, b""

        addr_raw = tx[0]
        is_read  = bool(addr_raw & 0x80)     # bit7
        auto_inc = bool(addr_raw & 0x40)     # bit6
        # For LSM6 series, bit6 is the autoinc flag, not part of the register
        reg      = (addr_raw & 0x3F) if auto_inc else (addr_raw & 0x7F)
        trailing = len(tx) - 1                # bytes requested on read

        if is_read:
            if reg == STATUS_REG and trailing >= 1:
                rx = bytes([0x00, XLDA | GDA])  # dummy + ready
                return len(rx), rx[: 1 + trailing]
            if reg == OUTX_L_G and auto_inc and trailing >= 1:
                rx = bytes([0x00]) + self._block12  # dummy + 12 bytes
                return len(rx), rx[: 1 + trailing]
            rx = bytes([0x00]) + (b"\x00" * trailing)
            return len(rx), rx

        # WRITE path (capture CTRL3_C to verify BDU set)
        payload = tx[1:]
        if payload:
            if auto_inc:
                for i, b in enumerate(payload):
                    self._writes.append(((reg + i) & 0xFF, b))
            else:
                self._writes.append((reg, payload[0]))
                if reg == CTRL3_C:
                    self._ctrl3c = payload[0]
        return len(tx), tx


def test_read_ax_ay_gz_bytes_maps_expected_order(monkeypatch):
    # Distinctive 12-byte block:
    # [GX_L,GX_H, GY_L,GY_H, GZ_L,GZ_H, AX_L,AX_H, AY_L,AY_H, AZ_L,AZ_H]
    block12 = bytes([
        0x11, 0x12,  # GX
        0x21, 0x22,  # GY
        0x31, 0x32,  # GZ
        0x41, 0x42,  # AX
        0x51, 0x52,  # AY
        0x61, 0x62,  # AZ
    ])

    # Driver should return [AX_L,AX_H, AY_L,AY_H, GZ_L,GZ_H]
    import hardware.spi_driver as spi_drv
    monkeypatch.setattr(
        spi_drv, "get_spi_host",
        lambda _params=None: FakePiHost(block12=block12),
        raising=True,
    )

    from hardware.LSM6DS3TR_driver import LSM6DS3TRDriver
    dev = LSM6DS3TRDriver(controller_params={})
    try:
        six = dev.read_ax_ay_gz_bytes()
    finally:
        dev.close()

    assert six == bytes([0x41, 0x42, 0x51, 0x52, 0x31, 0x32])


def test_driver_sets_bdu_once(monkeypatch):
    # Start with BDU clear (0x00); verify write to CTRL3_C sets bit6
    block12 = bytes([0] * 12)
    host = FakePiHost(block12=block12, ctrl3c_init=0x00)

    import hardware.spi_driver as spi_drv
    monkeypatch.setattr(spi_drv, "get_spi_host", lambda _p=None: host, raising=True)

    from hardware.LSM6DS3TR_driver import LSM6DS3TRDriver
    dev = LSM6DS3TRDriver(controller_params={})
    dev.close()

    assert any(reg == CTRL3_C and (val & 0x40) for reg, val in host._writes), \
        f"Expected BDU bit set in CTRL3_C write, got {host._writes}"

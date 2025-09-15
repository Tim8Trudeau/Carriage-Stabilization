# hardware/spi_driver.py
from __future__ import annotations

import logging
import os

spi_log = logging.getLogger("spi")

# LSM6 registers we need to recognize in mock xfers
STATUS_REG = 0x1E
OUTX_L_G   = 0x22

# --- public factory ----------------------------------------------------------
def get_spi_host(controller_params: dict | None = None):
    """
    Return a 'pi-like' object that implements:
        - spi_open(channel, baud, flags) -> handle
        - spi_close(handle) -> None
        - spi_xfer(handle, tx: bytes) -> (count:int, rx:bytes)
    Real HW: pigpio.pi()
    Mock   : shim backed by mock_spi.MockSPIBus (or test.mocks.mock_spi.MockSPIBus)
    """
    force_mock = os.getenv("CS_HW", "auto").lower() == "mock"

    if not force_mock:
        try:
            import pigpio  # type: ignore
            pi = pigpio.pi()
            if not pi.connected:
                raise RuntimeError("pigpio daemon not running")
            spi_log.info("spi_driver: using pigpio.pi() as SPI host.")
            return pi
        except Exception as e:
            spi_log.warning("spi_driver: pigpio unavailable (%s). Falling back to mock SPI host.", e)

    # --- Mock path: create a pi-like shim over MockSPIBus -------------------
    # Try test.mocks first (pytest), then project-local mock_spi.
    MockSPI = None
    try:
        from test.mocks.mock_spi import MockSPIBus as MockSPI  # type: ignore
    except Exception:
        try:
            from mock_spi import MockSPIBus as MockSPI  # type: ignore
        except Exception as e:
            spi_log.error("spi_driver: MockSPIBus unavailable: %s", e)
            raise

    class _MockPiHost:
        """
        Minimal pigpio.pi() shim using MockSPIBus data. Enough for LSM6DS3TRDriver:
          - We synthesize STATUS=0x03 (XLDA|GDA) on reads of STATUS_REG.
          - We synthesize 12B burst at OUTX_L_G using MockSPIBus().imu_read().
          - Other regs writes are accepted and ignored (BDU set, etc.).
        """
        def __init__(self):
            self._h = 1
            self._open = False
            self._regs = {}
            self._mock = MockSPI(controller_params or {})  # signature is tolerant in your mocks

        def spi_open(self, channel: int, baud: int, flags: int):
            self._open = True
            return self._h

        def spi_close(self, handle: int):
            self._open = False

        def spi_xfer(self, handle: int, tx: bytes):
            if not self._open:
                raise RuntimeError("SPI not open in mock host")
            if not tx:
                return 0, b""

            addr = tx[0] & 0x7F
            is_read = (tx[0] & 0x80) != 0
            auto_inc = (tx[0] & 0x40) != 0
            trailing = len(tx) - 1  # number of bytes requested in read

            # READ path
            if is_read:
                # STATUS register read (1 byte)
                if addr == STATUS_REG and trailing >= 1:
                    # Bit0=XLDA, Bit1=GDA
                    rx = bytes([0x00, 0x03])  # dummy + status
                    return len(rx), rx[: 1 + trailing]

                # Block read from OUTX_L_G with auto-increment → 12 bytes
                if addr == OUTX_L_G and auto_inc and trailing >= 1:
                    # Pull 6B from mock (AX, AY, GZ) and expand to 12B block:
                    # [GX_L,GX_H, GY_L,GY_H, GZ_L,GZ_H, AX_L,AX_H, AY_L,AY_H, AZ_L,AZ_H]
                    six = self._mock.imu_read()
                    ax_l, ax_h, ay_l, ay_h, gz_l, gz_h = six
                    block = bytes([
                        0x00, 0x00,  # GX
                        0x00, 0x00,  # GY
                        gz_l, gz_h,  # GZ
                        ax_l, ax_h,  # AX
                        ay_l, ay_h,  # AY
                        0x00, 0x00,  # AZ
                    ])
                    rx = bytes([0x00]) + block  # leading dummy/status byte
                    return len(rx), rx[: 1 + trailing]

                # Default: return zeros of requested size (+dummy)
                rx = bytes([0x00]) + (b"\x00" * trailing)
                return len(rx), rx

            # WRITE path: remember reg values (for BDU etc.), ignore otherwise
            payload = tx[1:]
            if payload:
                if auto_inc:
                    for i, b in enumerate(payload):
                        self._regs[(addr + i) & 0xFF] = b
                else:
                    self._regs[addr] = payload[0]
            # pigpio returns the bytes written as rx on write-xfer (we mimic by echoing tx)
            return len(tx), tx

        # Compatibility with LSM6DS3TRDriver.close()
        def stop(self):
            self._open = False

    spi_log.info("spi_driver: using MockSPIBus-backed SPI host shim.")
    return _MockPiHost()

# hardware/LSM6DS3TR_driver.py
"""
Thin, device-specific SPI driver for ST LSM6DS3TR-C.
This class owns a 'pi-like' SPI host obtained from hardware.spi_driver.get_spi_host().
It exposes register-level helpers and a convenience read for AX/AY/GZ.

Notes (LSM6 series over SPI):
- Read transactions set BIT7=1 on the register address.
- Multi-byte auto-increment sets BIT6=1 on the register address.
"""

from __future__ import annotations
from typing import Iterable
import time

# SPI command bits
_READ_BIT = 0x80
_AUTOINC  = 0x40

# Registers / bits
CTRL3_C      = 0x12
STATUS_REG   = 0x1E
OUTX_L_G     = 0x22   # start of 12B block through 0x2D
_BDU_BIT     = 0x40
_STATUS_XLDA = 0x01
_STATUS_GDA  = 0x02


class LSM6DS3TRDriver:
    """
    Owns an SPI handle from a 'pi-like' host (pigpio or mock shim) and implements:
      - readfrom_into(reg, buf)
      - read(reg, nbytes) -> bytes
      - write(reg, data: bytes) -> None
      - read_ax_ay_gz_bytes(timeout_s) -> bytes(6) [AX_L,AX_H, AY_L,AY_H, GZ_L,GZ_H]
    """

    def __init__(
        self,
        controller_params: dict | None = None,
        *,
        spi_channel: int = 0,
        spi_baud: int = 1_000_000,
        spi_flags: int = 0,
    ) -> None:
        # Acquire a host with pigpio-like SPI API from the factory
        from hardware.spi_driver import get_spi_host  # local import to avoid import-time pigpio on Windows
        self._pi = get_spi_host(controller_params or {})
        self._h = self._pi.spi_open(spi_channel, spi_baud, spi_flags)

        # Ensure BDU=1 (so low/high bytes are coherent across reads)
        cur = self.read(CTRL3_C, 1)[0]
        if not (cur & _BDU_BIT):
            self.write(CTRL3_C, bytes([cur | _BDU_BIT]))

    # --- basic reg ops -----------------------------------------------------

    def readfrom_into(self, reg: int, buf: bytearray) -> None:
        """Read len(buf) bytes from reg into buf (auto-increment)."""
        n = len(buf)
        addr = (reg | _READ_BIT | _AUTOINC) & 0xFF
        tx = bytes([addr]) + (b"\x00" * n)
        count, rx = self._pi.spi_xfer(self._h, tx)
        # rx[0] is dummy/status; data starts at rx[1]
        mv = memoryview(rx)[1 : 1 + n]
        buf[:n] = mv

    def read(self, reg: int, nbytes: int) -> bytes:
        """Read N bytes starting at reg (auto-increment)."""
        addr = (reg | _READ_BIT | _AUTOINC) & 0xFF
        tx = bytes([addr]) + (b"\x00" * nbytes)
        _, rx = self._pi.spi_xfer(self._h, tx)
        return rx[1 : 1 + nbytes]

    def write(self, reg: int, data: bytes | bytearray | Iterable[int]) -> None:
        """Write bytes starting at reg (auto-increment when len(data) > 1)."""
        payload = bytes(data)
        addr = (reg | (_AUTOINC if len(payload) > 1 else 0x00)) & 0xFF
        tx = bytes([addr]) + payload
        self._pi.spi_xfer(self._h, tx)

    # --- convenience --------------------------------------------------------

    def read_ax_ay_gz_bytes(self, timeout_s: float = 0.005) -> bytes:
        """
        Poll STATUS for accel+gyro ready, read 12B block 0x22..0x2D, and
        return 6 bytes in this order: [AX_L, AX_H, AY_L, AY_H, GZ_L, GZ_H].
        """
        deadline = time.perf_counter() + timeout_s
        while True:
            status = self.read(STATUS_REG, 1)[0]
            if (status & (_STATUS_XLDA | _STATUS_GDA)) == (_STATUS_XLDA | _STATUS_GDA):
                break
            if time.perf_counter() >= deadline:
                raise RuntimeError(f"data not ready (STATUS=0x{status:02X})")

        block = self.read(OUTX_L_G, 12)
        if len(block) != 12:
            raise RuntimeError(f"short read ({len(block)} bytes)")

        # 0x22..0x2D = [GX_L,GX_H, GY_L,GY_H, GZ_L,GZ_H, AX_L,AX_H, AY_L,AY_H, AZ_L,AZ_H]
        return bytes([block[6], block[7], block[8], block[9], block[4], block[5]])

    # --- lifecycle ----------------------------------------------------------

    def close(self) -> None:
        try:
            if getattr(self, "_h", None) is not None:
                self._pi.spi_close(self._h)
        finally:
            if hasattr(self, "_pi") and self._pi:
                # pigpio.pi() has .stop(); our shim also exposes .stop()
                stop = getattr(self._pi, "stop", None)
                if callable(stop):
                    stop()
            self._h = None
            self._pi = None

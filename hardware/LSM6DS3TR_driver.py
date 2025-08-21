# hardware/LSM6DS3TR_driver.py
"""
Thin, device-specific SPI driver for ST LSM6DS3TR-C using pigpio.
Provides register-level helpers compatible with a 'readfrom_into / read / write' API.

Notes (LSM6 series over SPI):
- Read transactions set BIT7=1 on the register address.
- Multi-byte auto-increment sets BIT6=1 on the register address.
"""

from __future__ import annotations
import pigpio
from typing import Optional, Iterable, Tuple

# LSM6DS3TR-C SPI command bits
_READ_BIT = 0x80
_AUTOINC  = 0x40


class LSM6DS3TRDriver:
    """
    Real hardware driver. Owns a pigpio SPI handle and implements:
      - readfrom_into(reg, buf)
      - read(reg, nbytes) -> bytes
      - write(reg, data: bytes) -> None
    """

    def __init__(
        self,
        pi: pigpio.pi,
        spi_channel: int = 0,
        spi_baud: int = 1_000_000,
        spi_flags: int = 0,
    ) -> None:
        """
        Args:
            pi: an active pigpio.pi() instance (caller controls lifecycle)
            spi_channel: hardware SPI channel (0 or 1 on Pi)
            spi_baud: SPI clock
            spi_flags: pigpio SPI flags (mode, chip select polarity, etc.)
        """
        if not isinstance(pi, pigpio.pi):
            raise TypeError("pi must be a pigpio.pi() instance")

        self._pi = pi
        self._h = self._pi.spi_open(spi_channel, spi_baud, spi_flags)

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

    # --- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        if getattr(self, "_h", None) is not None:
            try:
                self._pi.spi_close(self._h)
            finally:
                self._h = None

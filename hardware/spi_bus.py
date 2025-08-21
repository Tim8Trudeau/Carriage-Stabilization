# hardware/spi_bus.py
"""
Trivial SPIBus wrapper (no mocking). You may delete this if you use the device driver directly.
"""
from __future__ import annotations
import pigpio

class SPIBus:
    def __init__(self, pi: pigpio.pi, channel: int = 0, baud: int = 1_000_000, flags: int = 0) -> None:
        self._pi = pi
        self._h = self._pi.spi_open(channel, baud, flags)

    def xfer(self, tx: bytes) -> bytes:
        _, rx = self._pi.spi_xfer(self._h, tx)
        return rx

    def close(self) -> None:
        if getattr(self, "_h", None) is not None:
            try:
                self._pi.spi_close(self._h)
            finally:
                self._h = None

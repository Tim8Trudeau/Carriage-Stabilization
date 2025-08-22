# test/mocks/mock_spi.py
import logging
import math
import random

imu_log = logging.getLogger("imu_log")


class MockSPIBus:
    """
    Mock SPI for tests. Adds imu_read() to match spi_driver.SPIBus (Option 1):
      - imu_read() -> returns 6 bytes [xL,xH, yL,yH, omegaL,omegaH]
    Legacy helpers (readfrom_into/spi_xfer) still present.
    """

    def __init__(self, spi_channel=0, baud=1_000_000, io_mode=0,
                 omega_mode="constant", omega_raw_base=5000, noise_span=2000, theta_step=5):
        self.spi_channel = spi_channel
        self.baud = baud
        self.io_mode = io_mode
        self.omega_mode = (omega_mode or "constant").lower()
        self.omega_raw_base = int(omega_raw_base)
        self.noise_span = int(noise_span)
        self.step = 0
        self.direction = theta_step
        imu_log.info("Mock SPI bus initialized (mode=%s, ch=%d, baud=%d, spi_mode=%d)",
                     self.omega_mode, self.spi_channel, self.baud, self.io_mode)

    # --- sample synthesis (shared) ---
    def _synthesize(self):
        # accel in a circle (±16384)
        x_raw = int(16_384 * math.sin(math.radians(self.step)))
        y_raw = int(16_384 * math.cos(math.radians(self.step)))

        if self.omega_mode in ("none", "off", "0"):
            omega_raw = 0
        elif self.omega_mode == "constant":
            omega_raw = self.omega_raw_base
        elif self.omega_mode == "noisy":
            omega_raw = self.omega_raw_base + random.randint(-self.noise_span, self.noise_span)
        elif self.omega_mode == "random":
            omega_raw = random.randint(-self.noise_span, self.noise_span)
        else:
            omega_raw = 0

        omega_raw = max(-32768, min(32767, omega_raw))

        # step motion
        self.step += self.direction
        if self.step > 120 or self.step < -120:
            self.direction = -self.direction
        return x_raw, y_raw, omega_raw

    # NEW: Option-1 compatible API
    def imu_read(self, **_):
        x_raw, y_raw, omega_raw = self._synthesize()
        buf = bytearray(6)
        buf[0:2] = x_raw.to_bytes(2, "little", signed=True)
        buf[2:4] = y_raw.to_bytes(2, "little", signed=True)
        buf[4:6] = omega_raw.to_bytes(2, "little", signed=True)
        return buf

    # Legacy helpers kept for back-compat in older tests
    def spi_xfer(self):
        x_raw, y_raw, omega_raw = self._synthesize()
        # legacy order: [y, x, omega]
        buf = bytearray(6)
        buf[0:2] = y_raw.to_bytes(2, "little", signed=True)
        buf[2:4] = x_raw.to_bytes(2, "little", signed=True)
        buf[4:6] = omega_raw.to_bytes(2, "little", signed=True)
        return buf

    def readfrom_into(self, buffer: bytearray):
        return self.spi_xfer()

    def close(self):
        imu_log.info("Mock SPI bus closed.")


# Back-compat export
SPIBus = MockSPIBus
__all__ = ["MockSPIBus", "SPIBus"]

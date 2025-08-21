# test/mocks/mock_spi.py
import logging
import math
import random

imu_log = logging.getLogger("imu_log")


class MockSPIBus:
    """
    Mock pigpio-style SPIBus for testing without hardware.
    Simulates IMU returning x, y, omega readings.
    Mock SPI bus implementation for tests.
    The sensitivity of the IMU accelerometer
    with Linear acceleration measurement range set to +/- 2g
    is .000061g per LSb. The raw x,y values will be in the range
    of -16,384 to +16,384. Y axis pointing down will be 16384 when theta is 0.
    The sensitivity of the IMU gyro
    with angular measurement range set to +/- 250 degrees per second
    (dps) is .000875 dps per LSb.
    The raw omega values will be in the range
    of -32,768 to +32,768.
    This mock simulates the IMU SPI interface.
    The output of the readinto method will be a bytearray
    containing the x, y, and omega values in little-endian format.
    """

    def __init__(self, spi_channel=0, baud=1_000_000, io_mode=0, omega_mode="constant",
                 omega_raw_base=5000, noise_span=2000, theta_step=5):
        """
        Args:
            spi_channel (int): SPI channel (unused in mock)
            baud (int): baud rate (unused in mock)
            mode (int): SPI mode (unused in mock)
            omega_mode (str): constant, random, noisy, none
        """
        self.spi_channel = spi_channel
        self.baud = baud
        self.io_mode = io_mode
        self.omega_mode = (omega_mode or "constant").lower()
        self.omega_raw_base = int(omega_raw_base) # base raw value for constant mode (~38% of full scale)
        self.noise_span = int(noise_span)         # raw counts noise half-width
        self.step = 0                             # simulated initial carriage position motion in degrees
        self.direction = theta_step               # degrees change per iteration

        imu_log.info(
            "Mock SPI bus initialized (mode=%s, channel=%d, baud=%d, spi_mode=%d)",
            self.omega_mode, self.spi_channel, self.baud, self.io_mode
        )

    def spi_xfer(self):
        """
        Simulate pigpio.spi_xfer(): returns (count, rx_bytes). Simulation
        can ignore the read request that normally gets sent to the IMU.
        Generate raw IMU input, cycling x and y: range [-16,384, +16,384]
        and radial velocity: range[-32,768, +32,768] in bytearray format.
        The x and y values change with step, and simulates a path around a circle.
        sin/cos are reversed because step = 0 degrees is "up" (y=16384, x=0).
        Omega is simulated based on the omega_mode:
        - "constant": returns a constant raw value (e.g., 5000)
        - "noisy": returns a base value ± noise_span
        - "random": returns a random value in [-noise_span, +noise_span]
        - "none": returns 0 (no rotation)
        """
        buf = bytearray(6)

        # Simulated accelerometer raw output values
        x_raw = int(16_384 * math.sin(math.radians(self.step)))  # tangential accel
        y_raw = int(16_384 * math.cos(math.radians(self.step)))  # radial accel
        # Limit simulated data to 1g
        # Simulated omega raw values
        if self.omega_mode in ("none", "off", "0"):
            omega_raw = 0

        elif self.omega_mode == "constant":
            omega_raw = self.omega_raw_base

        elif self.omega_mode == "noisy":
            # noisy: base ± noise_span
            noise = random.randint(-self.noise_span, self.noise_span)
            omega_raw = self.omega_raw_base + noise

        elif self.omega_mode == "random":
            # NEW: uniform random in ±noise_span (ignores base)
            omega_raw = random.randint(-self.noise_span, self.noise_span)

        else:
            # unknown mode -> be safe
            omega_raw = 0

        # clip to int16 range just like hardware would
        omega_raw = max(-32768, min(32767, omega_raw))

        # Pack raw data (little endian, signed)
        buf[0:2] = y_raw.to_bytes(2, "little", signed=True)
        buf[2:4] = x_raw.to_bytes(2, "little", signed=True)
        buf[4:6] = omega_raw.to_bytes(2, "little", signed=True)

        # Simulate motion
        self.step += self.direction
        if self.step > 120 or self.step < -120:
            self.direction = -self.direction

        imu_log.debug(
            "MockSPI xfer: pos=%.1f, x=%d, y=%d, omega=%d",
            self.step, x_raw, y_raw, omega_raw
        )
        # First byte in array simulates SPI read register address
        return buf

    def readfrom_into(self, buffer: bytearray):
        return self.spi_xfer()

    def close(self):
        imu_log.info("Mock SPI bus closed.")


# ---- Back-compat exports ----
# Old tests may import SPIBus directly from this module.
SPIBus = MockSPIBus
__all__ = ["MockSPIBus", "SPIBus"]

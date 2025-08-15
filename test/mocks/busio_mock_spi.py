# test/mocks/mock_spi.py

import logging
import math

imu_log = logging.getLogger("imu_log")


class MockBoard:
    """Fake board with pin definitions."""

    SEL = "MOCK_SEL"
    SCK = "MOCK_SCK"
    MOSI = "MOCK_MOSI"
    MISO = "MOCK_MISO"


class MockDigitalInOut:
    """Simulates digital I/O pin."""

    def __init__(self, pin):
        self.pin = pin
        self.direction = None
        self.value = True  # True = High (not selected)

    def __repr__(self):
        return f"<MockDigitalInOut pin={self.pin} value={self.value}>"


class MockSPI:
    """Mock SPI bus implementation for tests.
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

    def __init__(self):
        self.configured = False
        self.locked = False
        self.read_log = []
        self.step = math.degrees(math.atan2(0, 1.57))  # Carriage at the top in degrees
        self.direction = 5  # Rotating CW or CCW

    def try_lock(self):
        self.locked = True
        return True

    def unlock(self):
        self.locked = False

    def configure(self, baudrate, phase, polarity):
        self.configured = True
        self.baudrate = baudrate
        self.phase = phase
        self.polarity = polarity

    def write(self, data):
        imu_log.debug("MockSPI write: %s", data)

    def readinto(self, buffer):
        """Simulate raw input cycling from -16,384 to +16,384 for x and y,
        -32,768 to +32,768 for omega.
        sin/cos are reversed because step = 0 degrees is "up" (y=16384, x=0).
        """
        x = int(16_384 * math.sin(math.radians(self.step)))  # tangential component
        y = int(16_384 * math.cos(math.radians(self.step)))  # radial component
        # omega = int(32_768 * math.cos(math.radians(self.step)))  # omega input
        omega = 0  # Simulate no rotation for simplicity

        buffer[0:2] = y.to_bytes(2, "little", signed=True)
        buffer[2:4] = x.to_bytes(2, "little", signed=True)
        buffer[4:6] = omega.to_bytes(2, "little", signed=True)

        # By convention if angle(theta) < 0 the rotation error is CCW
        # print(f"step: {self.step}, direction: {self.direction}")

        if (
            self.direction > 0
            and self.step < 120
            or self.direction < 0
            and self.step < -120
        ):
            self.direction = +5
        else:
            self.direction = -5
        self.step += self.direction
        imu_log.debug("MockSPI readinto: %s", list(buffer))
        return buffer


class SPIBus:
    """Mock SPIBus to match hardware.spi_driver.SPIBus."""

    def __init__(self):
        self.cs_pin = MockDigitalInOut(MockBoard.SEL)
        self.spi = MockSPI()
        imu_log.info("Mock SPI bus initialized.")

    def readfrom_into(self, register: int, buffer: bytearray):
        self.spi.try_lock()
        self.spi.configure(baudrate=1_000_000, phase=0, polarity=0)
        self.cs_pin.value = False
        self.spi.write(bytes([register | 0x80]))
        self.spi.readinto(buffer)
        self.cs_pin.value = True
        self.spi.unlock()

    def write(self, pin, value):
        # simulate setting pin state
        imu_log.debug("Mock write to pin %s: %d", pin, value)

    def set_mode(self, pin, mode):
        # simulate setting pin mode
        imu_log.debug("Mock set_mode for pin %s: %d", pin, mode)

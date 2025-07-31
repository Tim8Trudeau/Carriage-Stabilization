# test/mocks/mock_spi.py

import logging

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
    """Mock SPI bus implementation for tests."""

    def __init__(self):
        self.configured = False
        self.locked = False
        self.read_log = []
        self.step = 0  # simple time counter

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
        # Simulate raw input cycling from -16000 to +16000
        import math

        x = int(16000 * math.sin(self.step * 0.1))  # radial component
        y = int(16000 * math.cos(self.step * 0.1))  # tangential component
        omega = int(8000 * math.cos(self.step * 0.1))  # omega input

        buffer[0:2] = y.to_bytes(2, "little", signed=True)
        buffer[2:4] = x.to_bytes(2, "little", signed=True)
        buffer[4:6] = omega.to_bytes(2, "little", signed=True)

        self.step += 1
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

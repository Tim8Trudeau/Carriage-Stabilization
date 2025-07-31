# hardware/spi_driver.py

import logging

spi_log = logging.getLogger("spi")


class SPIBus:
    """
    SPI driver using busio.SPI for communication and pigpio to manually
    control the Chip Select (CS) line (default: GPIO 5).
    """

    def __init__(self, cs_pin=5):
        try:
            import pigpio  # noqa
            import board  # noqa
            import busio  # noqa

            self.spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
            self.cs_pin = cs_pin
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("Could not connect to pigpio daemon")

            self.pi.set_mode(self.cs_pin, pigpio.OUTPUT)
            self.pi.write(self.cs_pin, 1)  # deselect
            spi_log.info("SPI and CS initialized (CS GPIO %d)", self.cs_pin)

        except (ImportError, AttributeError, NotImplementedError) as e:
            # Fallback to mock if on unsupported platform (e.g., Windows)
            spi_log.warning("Using mock SPIBus due to: %s", e)
            try:
                from test.mocks.mock_spi import SPIBus as MockSPIBus

                mock_spi = MockSPIBus()
                self.spi = mock_spi.spi
                self.cs_pin = mock_spi.cs_pin
                self.pi = mock_spi  # <<== Fix: provide compatible mock object

            except ImportError as mock_error:
                spi_log.error("Mock SPIBus could not be loaded: %s", mock_error)
                self.spi = None
                self.pi = None

    def readfrom_into(self, register: int, buffer: bytearray):
        """
        Reads data from the device starting at the specified register.
        SPI Clock rate is 1MHz.

        Args:
            register (int): Register address to read from.
            buffer (bytearray): Buffer to fill with data read from SPI device.
        """
        if not self.spi or not self.pi:
            # from mock_spi we call self.cs_pin = MockDigitalInOut(MockBoard.SEL)
            raise RuntimeError("SPI bus or pigpio not initialized")

        # Manually control CS using pigpio
        try:
            while not self.spi.try_lock():
                pass

            self.spi.configure(baudrate=1_000_000, phase=0, polarity=0)
            self.pi.write(self.cs_pin, 0)  # CS low (select)
            self.spi.write(bytes([register | 0x80]))  # MSB set for read
            self.spi.readinto(buffer)
            self.pi.write(self.cs_pin, 1)  # CS high (deselect)

        finally:
            self.spi.unlock()

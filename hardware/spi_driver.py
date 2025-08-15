# hardware/spi_driver.py
import logging
import subprocess
import time

spi_log = logging.getLogger("spi")
# pylint: disable=no-member  # pigpio C-extension: members resolved at runtime

class SPIBus:
    """
    SPI driver that uses pigpio's hardware SPI interface when running on a
    Raspberry Pi, and automatically falls back to a mock SPI implementation
    when running on a platform without pigpio (e.g., Windows PC for pytest).

    This allows the same driver to work in both production (Pi hardware)
    and development/test environments (mock SPI).
    """

    def __init__(self, spi_channel=0, baud=1_000_000, mode=0):
        """
        Initialize the SPI bus.

        Args:
            spi_channel (int): SPI channel number.
                0 -> SPI0 CE0 (/dev/spidev0.0)
                1 -> SPI0 CE1 (/dev/spidev0.1)
            baud (int): SPI clock frequency in Hz.
            mode (int): SPI mode (0–3). Bits:
                - CPOL: clock polarity
                - CPHA: clock phase
        """
        self.spi_channel = spi_channel
        self.baud = baud
        self.mode = mode
        self.handle = None
        self.pi = None

        try:
            # Try importing pigpio locally so ImportError on non-Pi can be caught
            import pigpio
            flags = self.mode & 0b11  # pigpio: bits 0–1 = mode

            # Try connecting to pigpio daemon
            self.pi = pigpio.pi()
            if not self.pi.connected:
                spi_log.warning("pigpio daemon not running — attempting to start it...")
                self._start_pigpiod()
                # Retry connection after starting daemon
                self.pi = pigpio.pi()
                if not self.pi.connected:
                    raise RuntimeError("Unable to connect to pigpio daemon after start attempt")

            # Open the SPI device
            self.handle = self.pi.spi_open(self.spi_channel, self.baud, flags)
            spi_log.info(
                "pigpio SPI opened (channel=%d, baud=%d, mode=%d)",
                self.spi_channel, self.baud, self.mode
            )

            # Store the actual transfer method
            self._xfer = lambda tx: self.pi.spi_xfer(self.handle, tx)

        except Exception as e:
            # Any failure here (ImportError on Windows, no daemon, open error, etc.)
            # will result in falling back to the mock SPI.
            spi_log.warning("SPI falling back to mock: %s", e)

            from test.mocks.mock_spi import MockSPIBus
            mock = MockSPIBus(spi_channel=self.spi_channel,
                              baud=self.baud,
                              mode=self.mode)
            self._xfer = mock.spi_xfer
            self._mock = mock  # Keep a reference so GC doesn't remove it

    def _start_pigpiod(self):
        """
        Attempt to start the pigpio daemon using systemctl or direct launch.
        """
        try:
            # Try using systemctl (service manager)
            subprocess.run(
                ["sudo", "systemctl", "start", "pigpiod"],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(0.5)  # brief pause for daemon startup
            spi_log.info("pigpiod started via systemctl")
        except subprocess.CalledProcessError:
            spi_log.warning("systemctl start failed, trying manual pigpiod start...")
            try:
                subprocess.Popen(["sudo", "pigpiod"])
                time.sleep(0.5)
                spi_log.info("pigpiod started manually")
            except Exception as e:
                spi_log.error("Failed to start pigpiod manually: %s", e)
                raise

    def readfrom_into(self, register: int, buffer: bytearray):
        """
        Reads data from the device starting at the specified register.

        Args:
            register (int): The device register address to read from.
            buffer (bytearray): Buffer to fill with the received data.

        Notes:
            - The MSB of the register address is set (| 0x80) to indicate a read.
            - The first received byte is a dummy after sending the register,
              so it is skipped.
        """
        tx = bytes([register | 0x80]) + bytes(len(buffer))
        count, rx = self._xfer(tx)
        if count != len(tx):
            raise IOError("SPI transfer length mismatch")
        buffer[:] = rx[1:]  # skip the dummy byte

    def close(self):
        """
        Close the SPI interface and release resources.
        For pigpio: closes the SPI handle and stops the pigpio connection.
        For mock: does nothing.
        """
        if self.pi and self.handle is not None:
            try:
                self.pi.spi_close(self.handle)
            finally:
                self.pi.stop()
            self.handle = None
            self.pi = None

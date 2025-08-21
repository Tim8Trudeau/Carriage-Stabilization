# hardware/spi_driver.py
import logging
import subprocess
import time

spi_log = logging.getLogger("spi")
# pylint: disable=no-member  # pigpio C-extension: members resolved at runtime

class SPIBus:
    """
    SPI driver that uses pigpio on a Raspberry Pi and automatically falls back to a
    mock SPI implementation when pigpio isn't available. The caller can provide
    `mock_params` (e.g., from flc_config.toml) to parameterize the mock.
    """

    def __init__(self, controller_params):
        self.spi_channel = 0
        self.baud = 1_000_000  # 1 MHz
        self.io_mode = 0b00  # Mode 0: CPOL=0
        self.handle = None
        self.is_mock = False
        self._mock = None
        self.pi = None

        try:
            import pigpio
            flags = self.io_mode & 0b11  # pigpio: bits 0–1 = mode

            # Try connecting to pigpio daemon
            self.pi = pigpio.pi()
            if not self.pi.connected:
                spi_log.warning("pigpio daemon not running — attempting to start it...")
                self._start_pigpiod()
                self.pi = pigpio.pi()
                if not self.pi.connected:
                    raise RuntimeError("Unable to connect to pigpio daemon after start attempt")

            # Open SPI
            self.handle = self.pi.spi_open(self.spi_channel, self.baud, flags)
            spi_log.info(
                "pigpio SPI opened (channel=%d, baud=%d, io mode=%d)",
                self.spi_channel, self.baud, self.io_mode
            )
            # Define transfer method
            # pigpio.spi_xfer() returns (count, rx_bytes)
            self._xfer = lambda tx: self.pi.spi_xfer(self.handle, tx)

        except Exception as e:
            # Fallback to mock (Windows, missing pigpio, daemon down, etc.)
            spi_log.warning("SPI falling back to mock: %s", e)
            from test.mocks.mock_spi import MockSPIBus

            cfg_src = mock_params or {}
            cfg = cfg_src.get("MOCK_SPI", cfg_src)  # accept either nested or flat
                                                    # mock_params structure
            self._mock = MockSPIBus(
                # Pass through simulation knobs from config
                omega_mode=cfg.get("OMEGA_MODE", "none"),         # "constant" | "random" | "none"
                omega_raw_base=cfg.get("OMEGA_RAW_BASE", 5000),   # raw constant counts
                noise_span=cfg.get("NOISE_SPAN", 2000),           # raw noise ±span
                theta_step=cfg.get("THETA_STEP", 5),              # degrees/step for theta evolution
            )
            self._xfer = self._mock.spi_xfer
            self.is_mock = True

    def _start_pigpiod(self):
        try:
            subprocess.run(
                ["sudo", "systemctl", "start", "pigpiod"],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(0.5)
            spi_log.info("pigpiod started via systemctl")
        except subprocess.CalledProcessError:
            spi_log.warning("systemctl start failed, trying manual pigpiod start...")
            subprocess.Popen(["sudo", "pigpiod"])
            time.sleep(0.5)
            spi_log.info("pigpiod started manually")

    def readfrom_into(self, register: int, buffer: bytearray):
        tx = bytes([register | 0x80]) + bytes(len(buffer))
        count, rx = self._xfer(tx)
        if count != len(tx):
            raise IOError("SPI transfer length mismatch")
        buffer[:] = rx[1:]  # skip dummy byte. data passed in buffer[1:]

    def close(self):
        if self.pi and self.handle is not None:
            try:
                self.pi.spi_close(self.handle)
            finally:
                self.pi.stop()
            self.handle = None
            self.pi = None

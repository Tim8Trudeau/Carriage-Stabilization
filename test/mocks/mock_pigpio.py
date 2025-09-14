# test/mocks/mock_pigpio.py

OUTPUT = 1  # numeric like pigpio

class _MockPi:
    """
    Minimal pigpio 'pi' mock with SPI support sufficient for spi_driver.SPIBus:
      - connected (bool)
      - spi_open/spi_write/spi_read/spi_close
      - stop()
    It also records CTRL3_C writes so tests can assert BDU was set.
    """

    def __init__(self):
        # public-ish state for assertions in tests
        self.connected = True
        self._closed = False
        self._stopped = False
        self._writes = []          # list[(reg, val)] for register writes
        self._last_addr = None     # last address byte written for a subsequent read
        self._handle = 1
        self.pwm_states: dict[int, dict[str, int]] = {}
        # Register read defaults used by spi_driver:
        self._ctrl3c_val = 0x00    # CTRL3_C initial value (BDU clear by default)
        self._status_val = 0x03    # STATUS_REG: XLDA|GDA set by default
        # 12-byte block for 0x22..0x2D:
        # [GX_L,GX_H, GY_L,GY_H, GZ_L,GZ_H, AX_L,AX_H, AY_L,AY_H, AZ_L,AZ_H]
        self._block = bytes(range(0x22, 0x22 + 12))

    # --- configuration helpers for tests ---
    def set_mode(self, gpio, mode):
        # optional; lets pwm_driver call it without errors
        self._last_mode = (int(gpio), int(mode) if isinstance(mode, int) else mode)

    def hardware_PWM(self, gpio, frequency, dutycycle):
        self.pwm_states[int(gpio)] = {
            "frequency": int(frequency),
            "dutycycle": int(dutycycle),
        }
        return 0  # mimic pigpio success

    # --- SPI helpers for tests ---
    def set_ctrl3c(self, val: int):
        self._ctrl3c_val = val & 0xFF

    def set_status(self, val: int):
        self._status_val = val & 0xFF

    def set_block(self, bb: bytes):
        self._block = bytes(bb[:12]).ljust(12, b"\x00")

    # --- pigpio-like SPI API ---
    def spi_open(self, channel, baud, flags):
        return self._handle

    def spi_write(self, handle, tx_bytes: bytes):
        # two patterns:
        #   - write_reg: [reg, val]      -> treat as a register write
        #   - read(_block): [addr]       -> remember for the next spi_read()
        if isinstance(tx_bytes, (bytearray, bytes)):
            if len(tx_bytes) == 2:
                self._writes.append((tx_bytes[0] & 0x7F, tx_bytes[1] & 0xFF))
            elif len(tx_bytes) == 1:
                self._last_addr = tx_bytes[0] & 0xFF
            elif len(tx_bytes) > 1:
                self._last_addr = tx_bytes[-1] & 0xFF

    def spi_read(self, handle, nbytes: int):
        # Interpret the last single-byte address; pigpio SPI read is separate.
        addr = self._last_addr or 0x00
        reg = addr & 0x3F  # strip read/auto-inc (bit7/bit6)

        # CTRL3_C (0x12)
        if reg == 0x12 and nbytes == 1:
            return (1, bytes([self._ctrl3c_val]))

        # STATUS_REG (0x1E)
        if reg == 0x1E and nbytes == 1:
            return (1, bytes([self._status_val]))

        # OUTX_L_G block (0x22) — return 12 bytes
        if reg == 0x22 and nbytes == 12:
            return (12, self._block[:12])

        return (nbytes, bytes([0x00] * nbytes))

    def spi_close(self, handle):
        self._closed = True

    def stop(self):
        self._stopped = True


def pi():
    return _MockPi()

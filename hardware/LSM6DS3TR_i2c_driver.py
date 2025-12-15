"""
Corrected LSM6DS3TR-C I2C driver.
- Proper deterministic initialization (no broken RMW)
- Gyroscope correctly enabled (SLEEP_G=0)
- Accel + gyro ODR set to 52 Hz
- Full-scale = ±2g (accel), ±245 dps (gyro)
- Clean axis extraction: GX, GY, GZ, AX, AY, AZ
"""

from __future__ import annotations
from typing import Iterable
import time
import logging

_log = logging.getLogger("imu.i2c")

# Register Map --------------------------------------------------------------
FIFO_CTRL1      = 0x06
FIFO_CTRL2      = 0x07
FIFO_CTRL3      = 0x08
FIFO_CTRL4      = 0x09
FIFO_CTRL5      = 0x0A
INT1_CTRL       = 0x0D
INT2_CTRL       = 0x0E
WHO_AM_I        = 0x0F
CTRL1_XL        = 0x10
CTRL2_G         = 0x11
CTRL3_C         = 0x12
CTRL4_C         = 0x13
CTRL6_C         = 0x15
CTRL7_G         = 0x16
CTRL8_XL        = 0x17
CTRL9_XL        = 0x18
CTRL10_C        = 0x19
MASTER_CONFIG   = 0x1A
STATUS_REG      = 0x1E
OUTX_L_G        = 0x22     # 12-byte block: GX,GY,GZ,AX,AY,AZ

_STATUS_XLDA = 0x01
_STATUS_GDA  = 0x02


# ----------------------------------------------------------------------------
# Driver
# ----------------------------------------------------------------------------

class LSM6DS3TRDriver:
    def __init__(self, controller_params: dict | None = None,
                 *, i2c_bus: int | None = None,
                 i2c_addr: int | None = None,
                 i2c_flags: int = 0):

        from hardware.i2c_driver import get_i2c_host

        params = controller_params or {}
        bus  = i2c_bus  if i2c_bus  is not None else int(params.get("I2C_BUS", 1))
        addr = i2c_addr if i2c_addr is not None else int(params.get("I2C_ADDR", 0x6B))

        self._pi = get_i2c_host(params)
        self._h  = self._pi.i2c_open(bus, addr, i2c_flags)

        _log.info("LSM6DS3TR(I2C): bus=%d addr=0x%02X host=%s",
                  bus, addr,
                  "mock" if hasattr(self._pi, "set_motor_cmd") else "pigpio")

        self._init_device()


    # ----------------------------------------------------------------------------
    # Deterministic IMU Init (no read-modify-write)
    # ----------------------------------------------------------------------------
    def _init_device(self) -> None:
        # WHO_AM_I check
        who = self.read(WHO_AM_I, 1)[0]
        if who != 0x69:
            _log.warning("WHO_AM_I = 0x%02X (expected 0x69)", who)
        else:
            _log.info("WHO_AM_I = 0x%02X OK", who)

        # CTRL3_C – BDU = 1, IF_INC = 1 (0x44)
        self.write(CTRL3_C, b"\x44")

        # CTRL4_C – ensure gyro is awake, I2C enabled
        # SLEEP_G = 0, I2C_DISABLE = 0, others = default
        self.write(CTRL4_C, b"\x00")

        # Accel – 52 Hz, ±2g, BW ODR/2 → 0x30
        self.write(CTRL1_XL, b"\x30")

        # Gyro – 52 Hz, ±245 dps → 0x30
        self.write(CTRL2_G, b"\x30")

        # Disable filters
        self.write(CTRL6_C, b"\x00")   # FTYPE=0 → LPF at ODR/2
        self.write(CTRL7_G, b"\x00")   # HP filter disabled
        self.write(CTRL8_XL, b"\x00")  # Accel LPF off

        # Disable embedded functions
        self.write(CTRL10_C, b"\x07")   # enable GX, GY, GZ
        self.write(MASTER_CONFIG, b"\x00")

        # FIFO bypass
        self.write(FIFO_CTRL1, b"\x00")
        self.write(FIFO_CTRL2, b"\x00")
        self.write(FIFO_CTRL3, b"\x00")
        self.write(FIFO_CTRL4, b"\x00")
        self.write(FIFO_CTRL5, b"\x00")

        time.sleep(0.05)
        _log.info("IMU initialized: accel+gyro @52 Hz, FS=±2g/±245dps")


    # ----------------------------------------------------------------------------
    # Register I/O
    # ----------------------------------------------------------------------------
    def _read_block(self, reg: int, n: int) -> bytes:
        """
        Safe register block read for LSM6DS3TR-C using pigpio.

        pigpio.i2c_read_i2c_block_data() does NOT reliably auto-increment
        on this IMU, especially when reading gyro registers. This causes the
        gyro to never update (STATUS.GDA = 0) because the IMU will not latch
        new gyro samples unless the host reads the correct addresses.

        This version reads bytes one at a time with repeated starts, which is
        100% reliable for LSM6DS3TR-C.
        """
        out = bytearray()
        for offset in range(n):
            b = self._pi.i2c_read_byte_data(self._h, (reg + offset) & 0xFF)
            out.append(b & 0xFF)
        return bytes(out)


    def read(self, reg: int, nbytes: int) -> bytes:
        return self._read_block(reg, int(nbytes))

    def write(self, reg: int, data: bytes | bytearray | Iterable[int]) -> None:
        payload = bytes(data)
        if len(payload) == 1:
            self._pi.i2c_write_byte_data(self._h, reg & 0xFF, payload[0])
        else:
            for i, b in enumerate(payload):
                self._pi.i2c_write_byte_data(self._h, (reg + i) & 0xFF, b)


    # ----------------------------------------------------------------------------
    # Status-synchronized read of all six axes
    # ----------------------------------------------------------------------------
    def read_all_axes(self, timeout_s: float = 0.02):
        """Returns (AX, AY, AZ, GX, GY, GZ) raw signed 16-bit."""
        deadline = time.perf_counter() + timeout_s

        while True:
            status = self._pi.i2c_read_byte_data(self._h, STATUS_REG) & 0xFF
            if (status & (_STATUS_XLDA | _STATUS_GDA)) == (_STATUS_XLDA | _STATUS_GDA):
                break
            if time.perf_counter() > deadline:
                raise RuntimeError(f"IMU data not ready, STATUS=0x{status:02X}")

        block = self._read_block(OUTX_L_G, 12)

        gx = int.from_bytes(block[0:2],  "little", signed=True)
        gy = int.from_bytes(block[2:4],  "little", signed=True)
        gz = int.from_bytes(block[4:6],  "little", signed=True)
        ax = int.from_bytes(block[6:8],  "little", signed=True)
        ay = int.from_bytes(block[8:10], "little", signed=True)
        az = int.from_bytes(block[10:12],"little", signed=True)

        return ax, ay, az, gx, gy, gz


    # ----------------------------------------------------------------------------
    # Lifecycle
    # ----------------------------------------------------------------------------
    def close(self):
        try:
            self._pi.i2c_close(self._h)
        finally:
            stop = getattr(self._pi, "stop", None)
            if callable(stop):
                stop()
            self._h = None
            self._pi = None

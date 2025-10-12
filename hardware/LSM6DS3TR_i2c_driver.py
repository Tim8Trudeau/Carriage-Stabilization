# hardware/LSM6DS3TR_i2c_driver.py
"""
Thin, device-specific I2C driver for ST LSM6DS3TR-C (uses hardware.i2c_driver).
"""

from __future__ import annotations
from typing import Iterable
import time, logging

_log = logging.getLogger("imu.i2c")

# IMU Register addresses

FIFO_CTRL1  = 0x06
FIFO_CTRL2  = 0x07
FIFO_CTRL3  = 0x08
FIFO_CTRL4  = 0x09
FIFO_CTRL5  = 0x0A
INT1_CTRL   = 0x0D
INT2_CTRL   = 0x0E
WHO_AM_I    = 0x0F
CTRL1_XL    = 0x10
CTRL2_G     = 0x11
CTRL3_C     = 0x12
CTRL4_C     = 0x13
CTRL6_C     = 0x15
CTRL7_G     = 0x16
CTRL8_XL    = 0x17
CTRL9_XL    = 0x18
CTRL10_C    = 0x19
MASTER_CONFIG = 0x1A  # sensor hub/master (disable)
STATUS_REG  = 0x1E
OUTX_L_G    = 0x22
_BDU_BIT     = 0x40
_IFINC_BIT   = 0x04
_STATUS_XLDA = 0x01
_STATUS_GDA  = 0x02
_STATUS_TDA  = 0x04


class LSM6DS3TRDriver:
    # ---- Added this to hardware/LSM6DS3TR_i2c_driver.py (inside class LSM6DS3TRDriver) ----

    def _rmw(self, reg: int, *, set_bits: int = 0, clr_bits: int = 0) -> None:
        """Read-modify-write helper."""
        cur = self.read(reg, 1)[0]
        val = (cur | (set_bits & 0xFF)) & (~clr_bits & 0xFF)
        if val != cur:
            self.write(reg, bytes([val]))

    def init_low_power_52hz(self) -> None:
        """
        Configure LSM6DS3TR-C for:
        - ODR 52 Hz accel/gyro (low-power bucket)
        - FS_XL ±2g, FS_G ±245/250 dps
        - Block Data Update (BDU), auto-increment (IF_INC), little-endian
        - No interrupts, FIFO bypass, gyro sleep disabled
        - Gyro LPF1 = ODR/2, no LPF2/HPF/slope on accel
        - No pedometer/tilt/tap/sensor-hub features
        """
        # ---------------- Sanity ----------------
        who = self.read(WHO_AM_I, 1)[0]
        if who != 0x69:
            _log.warning("LSM6DS3TR WHO_AM_I=0x%02X (expected 0x69) — continuing", who)
        else:
            _log.info("LSM6DS3TR WHO_AM_I=0x%02X OK", who)
        # ---------------- Basic interface / endianness / sync ----------------
        # CTRL3_C bits: BDU=bit6, IF_INC=bit2, BLE=bit1 (0=little), SIM=bit0 (i2c; keep 0)
        self._rmw(CTRL3_C, set_bits=(1<<6) | (1<<2), clr_bits=(1<<1) | (1<<0))

        # Ensure I2C enabled and gyro sleep disabled (CTRL4_C: I2C_DISABLE=bit2, SLEEP_G=bit6)
        self._rmw(CTRL4_C, clr_bits=(1<<2) | (1<<6))

        # ---------------- Output Data Rates / Full-scales ----------------
        # ODR code for 52 Hz is 0b0011 in [7:4] → 0x30.
        ODR_52 = 0x30

        # CTRL1_XL: ODR_XL[7:4] | FS_XL[3:2]=00 (±2g) | BW_XL[1:0] choose ODR/2 AA bandwidth (01)
        FS_XL_2G = 0x00
        BW_XL_ODR_DIV_2 = 0x01
        self.write(CTRL1_XL, bytes([ODR_52 | (FS_XL_2G << 2) | BW_XL_ODR_DIV_2]))

        # CTRL2_G: ODR_G[7:4] | FS_G[3:2]=00 (±245 dps)
        FS_G_245DPS = 0x00
        self.write(CTRL2_G, bytes([ODR_52 | (FS_G_245DPS << 2)]))

        # ---------------- Filters ----------------
        # Gyro LPF1 cutoff via CTRL6_C.FTYPE[2:0]. 0b000 ≈ ODR/2.
        self._rmw(CTRL6_C, set_bits=0x00, clr_bits=0x07)   # clear FT bits → ODR/2

        # Disable gyro HPF (CTRL7_G: HP_EN_G=bit6 → 0)
        self._rmw(CTRL7_G, clr_bits=(1<<6))

        # Accel: disable LPF2 + HP/slope (CTRL8_XL: LPF2_XL_EN=bit7, HP_SLOPE_XL_EN=bit2)
        self._rmw(CTRL8_XL, clr_bits=(1<<7) | (1<<2))

        # Leave CTRL9_XL as default (no soft resets here)

        # ---------------- Interrupts off ----------------
        self.write(INT1_CTRL, b"\x00")
        self.write(INT2_CTRL, b"\x00")

        # ---------------- FIFO bypass ----------------
        # FIFO_CTRL5: FIFO_MODE[2:0]=000 (bypass), ODR selection cleared
        self.write(FIFO_CTRL1, b"\x00")
        self.write(FIFO_CTRL2, b"\x00")
        self.write(FIFO_CTRL3, b"\x00")
        self.write(FIFO_CTRL4, b"\x00")
        self.write(FIFO_CTRL5, b"\x00")

        # ---------------- Disable sensor-hub / fancy functions ----------------
        # MASTER_CONFIG (sensor hub) off
        try:
            self.write(MASTER_CONFIG, b"\x00")
        except Exception:
            pass  # Some variants alias/omit this; safe to ignore.

        # CTRL10_C hosts embedded function enables on some variants — clear them.
        # (No wrist tilt, pedometer, step, tap, timestamp, etc.)
        self.write(CTRL10_C, b"\x00")

        _log.info("LSM6DS3TR configured: 52Hz, ±2g, ±245dps, BDU+IF_INC, little-endian, "
                "gyro LPF1 ODR/2, no LPF2/HPF/slope, FIFO bypass, no interrupts.")


    def __init__(self, controller_params: dict | None = None, *, i2c_bus: int | None = None,
                 i2c_addr: int | None = None, i2c_flags: int = 0) -> None:
        from hardware.i2c_driver import get_i2c_host

        params = controller_params or {}
        bus  = i2c_bus  if i2c_bus  is not None else int(params.get("I2C_BUS", 1))
        addr = i2c_addr if i2c_addr is not None else int(params.get("I2C_ADDR", 0x6B))

        self._pi = get_i2c_host(params)       # <- selects pigpio vs mock
        self._h  = self._pi.i2c_open(bus, addr, i2c_flags)

        # 3-line log tweak (host kind)
        _log.info("LSM6DS3TR(I2C): bus=%d addr=0x%02X host=%s",
                  bus, addr, "mock" if hasattr(self._pi, "set_motor_cmd") else "pigpio")

        self.init_low_power_52hz()

    # --- internal: normalize pigpio/mock block reads to bytes of length n ---
    # If pigpio is opened, it has i2c_read_i2c_block_data() else its in mock_pigpio.
    def _read_block(self, reg: int, n: int) -> bytes:
        _readdata = self._pi.i2c_read_i2c_block_data(self._h, reg & 0xFF, int(n)) # type: ignore
        _log.info(f"called _read_block(readdata ={_readdata}, type={type(_readdata)})")
        ret = _readdata

        def _to_bytes_any(obj) -> bytes:
            # Fast paths
            if isinstance(obj, (bytes, bytearray, memoryview)):
                return bytes(obj)
            # Some pigpio builds or wrappers may hand back array('B') or lists/tuples
            try:
                return bytes(bytearray(int(x) & 0xFF for x in obj))
            except Exception:
                # Last resort: if it's a str (shouldn't happen, but be robust)
                if isinstance(obj, str):
                    # Treat as raw 8-bit data; latin-1 preserves byte values 0..255
                    return obj.encode("latin-1", errors="ignore")
                # Give up—empty bytes
                return b""

        # pigpio returns (count, data); mock returns bytes/bytearray
        if isinstance(ret, tuple) and len(ret) == 2:
            count, data = ret
            data_b = _to_bytes_any(data)[: int(count)]
        else:
            data_b = _to_bytes_any(ret)

        if len(data_b) < n:
            raise RuntimeError(f"short read from 0x{reg:02X}: got {len(data_b)} < {n}")
        return data_b[:n]

    # --- reg helpers ---
    def readfrom_into(self, reg: int, buf: bytearray) -> None:
        n = len(buf)
        buf[:n] = memoryview(self._read_block(reg, n))[:n]

    def read(self, reg: int, nbytes: int) -> bytes:
        return self._read_block(reg, int(nbytes))

    def write(self, reg: int, data: bytes | bytearray | Iterable[int]) -> None:
        payload = bytes(data)
        if len(payload) == 1:
            self._pi.i2c_write_byte_data(self._h, reg & 0xFF, payload[0]) # type: ignore
        else:
            for i, b in enumerate(payload):
                self._pi.i2c_write_byte_data(self._h, (reg + i) & 0xFF, b) # type: ignore

    # --- convenience ---
    def read_ax_ay_gz_bytes(self, timeout_s: float = 0.02) -> bytes:
        deadline = time.perf_counter() + timeout_s
        while True:
            status = self._pi.i2c_read_byte_data(self._h, STATUS_REG) & 0xFF # type: ignore
            _log.debug(f"LSM6DS3TR STATUS={status} time= {time.perf_counter()} deadline {deadline}")
            if (status & (_STATUS_XLDA | _STATUS_GDA)) == (_STATUS_XLDA | _STATUS_GDA):
                break
            if time.perf_counter() >= deadline:
                raise RuntimeError(f"data not ready (STATUS=0x{status:02X})")
        block = self._read_block(OUTX_L_G, 12)
        # Coordinate remap: AX->AZ AY->AY GZ->GX
        # [GX][GY][GZ][AX][AY][AZ] → return [AX, AY, GZ] (each 16-bit LE)
        # return bytes([block[6], block[7], block[8], block[9], block[4], block[5]])
        return bytes([block[10], block[11], block[8], block[9], block[0], block[1]])

    # --- lifecycle ---
    def close(self) -> None:
        try:
            if getattr(self, "_h", None) is not None:
                self._pi.i2c_close(self._h) # type: ignore
        finally:
            stop = getattr(self._pi, "stop", None)
            if callable(stop):
                stop()
            self._h = None
            self._pi = None

    # --- sim passthroughs (no-op on real pigpio) ---
    def set_motor_cmd(self, v: float) -> None:
        f = getattr(self._pi, "set_motor_cmd", None)
        if callable(f):
            f(v)

    def get_sim_theta(self):
        f = getattr(self._pi, "get_sim_theta", None)
        return float(f()) if callable(f) else None # type: ignore

# ---- Add this to hardware/LSM6DS3TR_i2c_driver.py (inside class LSM6DS3TRDriver) ----

# Register addresses (reuse any you already defined at top of file)
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
INT1_CTRL   = 0x0D
INT2_CTRL   = 0x0E
FIFO_CTRL1  = 0x06
FIFO_CTRL2  = 0x07
FIFO_CTRL3  = 0x08
FIFO_CTRL4  = 0x09
FIFO_CTRL5  = 0x0A

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

    # ---------------- Basic interface / endianness / sync ----------------
    # CTRL3_C bits: BDU=bit6, IF_INC=bit2, BLE=bit1 (0=little), SIM=bit0 (SPI 3-wire; keep 0)
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

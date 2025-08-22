from __future__ import annotations

import logging
import struct
import time

spi_log = logging.getLogger("spi")


class SPIBus:
    """
    Minimal SPI helper for LSM6DS3TR-C over pigpio.
    - Sets BDU (Block Data Update) on init
    - Polls STATUS_REG for accel+gyro ready
    - Reads a single block and returns 6 bytes reordered as (raw_x, raw_y, raw_omega)
      where raw_x, raw_y come from accelerometer X/Y and raw_omega from gyro Z.
    """

    # --- LSM6DS3TR-C registers (SPI) ---
    CTRL3_C      = 0x12
    STATUS_REG   = 0x1E
    OUTX_L_G     = 0x22   # block read start (gyro X L)
    OUTZ_H_XL    = 0x2D   # block read end   (accel Z H)

    # Bit masks
    _BDU_BIT     = 0x40   # CTRL3_C bit6
    _READ_FLAG   = 0x80   # bit7 = 1 (read)
    _AUTO_INC    = 0x40   # bit6 = 1 (auto-increment)
    _STATUS_XLDA = 0x01   # accel new data available
    _STATUS_GDA  = 0x02   # gyro  new data available

    def __init__(self, controller_params: dict | None = None, *, spi_channel=0, baud=1_000_000, mode=0):
        try:
            import pigpio
        except Exception as e:  # pragma: no cover (unit tests patch SPIBus)
            raise RuntimeError("pigpio not available for SPIBus") from e

        self._pigpio = pigpio
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("pigpio daemon not running")

        # Open SPI
        self._h = self._pi.spi_open(spi_channel, baud, mode)  # mode=0 (CPOL=0, CPHA=0)
        spi_log.info("SPI opened (ch=%d, baud=%d, mode=%d)", spi_channel, baud, mode)

        # Ensure BDU=1 (read-modify-write CTRL3_C)
        cur = self._read_reg(self.CTRL3_C, 1)[0]
        if not (cur & self._BDU_BIT):
            self._write_reg(self.CTRL3_C, cur | self._BDU_BIT)
            spi_log.debug("CTRL3_C set: BDU=1 (old=0x%02X -> new=0x%02X)", cur, cur | self._BDU_BIT)

    # ---- low-level helpers ----
    def _write_reg(self, reg: int, val: int) -> None:
        # SPI write: MSB=0 (write), no auto-inc for single byte
        tx = bytes((reg & 0x7F, val & 0xFF))
        self._pi.spi_write(self._h, tx)

    def _read_reg(self, reg: int, nbytes: int) -> bytes:
        # SPI read is a two-step: write addr, then read N bytes
        # Note: for single read, no need for auto-inc; for block, we use it.
        self._pi.spi_write(self._h, bytes((self._READ_FLAG | (reg & 0x7F),)))
        _count, rx = self._pi.spi_read(self._h, nbytes)
        return rx

    def _read_block(self, start_reg: int, nbytes: int) -> bytes:
        # Burst read with auto-increment: send (READ|AUTO_INC|start) then read nbytes
        self._pi.spi_write(self._h, bytes((self._READ_FLAG | self._AUTO_INC | (start_reg & 0x7F),)))
        _count, rx = self._pi.spi_read(self._h, nbytes)
        return rx

    # ---- public API ----
    def imu_read(self, *, timeout_s: float = 0.005) -> bytes:
        """
        Wait until both accel (XLDA) and gyro (GDA) have fresh data, then
        read the whole block 0x22..0x2D (12 bytes), and return ONLY the
        desired 6 bytes in the order expected by imu_driver:

            return: bytearray(6) as [raw_x L, raw_x H, raw_y L, raw_y H, raw_omega L, raw_omega H]

        raw_x, raw_y = accelerometer X, Y
        raw_omega    = gyro Z

        Raises RuntimeError on timeout.
        """
        # 1) poll STATUS_REG for both bits set
        deadline = time.perf_counter() + timeout_s
        while True:
            status = self._read_reg(self.STATUS_REG, 1)[0]
            if (status & (self._STATUS_XLDA | self._STATUS_GDA)) == (self._STATUS_XLDA | self._STATUS_GDA):
                break
            if time.perf_counter() >= deadline:
                raise RuntimeError("imu_read timeout: data not ready (STATUS=0x%02X)" % status)
            # small spin-wait; IMU data-ready is typically fast
            # no sleep to keep latency low; add time.sleep(0.0001) if needed

        # 2) read full 12-byte block once
        block = self._read_block(self.OUTX_L_G, 12)
        if len(block) != 12:
            raise RuntimeError(f"imu_read: short read ({len(block)} bytes)")

        # LSM6DS3TR-C block layout for 0x22..0x2D (little endian shorts):
        # [GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H, AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H]
        # We only keep: GZ (omega), AX (x), AY (y)
        gz_l, gz_h = block[4], block[5]
        ax_l, ax_h = block[6], block[7]
        ay_l, ay_h = block[8], block[9]

        # 3) return in the order imu_driver expects: (raw_x, raw_y, raw_omega) as int16 LE
        # i.e., [AX_L, AX_H, AY_L, AY_H, GZ_L, GZ_H]
        out = bytearray(6)
        out[0] = ax_l
        out[1] = ax_h
        out[2] = ay_l
        out[3] = ay_h
        out[4] = gz_l
        out[5] = gz_h
        return out

    def close(self) -> None:
        try:
            if hasattr(self, "_h") and self._h is not None:
                self._pi.spi_close(self._h)
        finally:
            if hasattr(self, "_pi") and self._pi is not None:
                self._pi.stop()

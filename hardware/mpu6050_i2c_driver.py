"""
MPU6050 I2C Driver
------------------

Drop-in replacement for LSM6DS3TR_i2c_driver.py for the Carriage Stabilization Project.

This module exposes:
    read_all_axes() -> (AX, AY, AZ, GX, GY, GZ)

All values are returned as 16-bit signed integers, matching the format expected
by IMU_Driver (see imu_driver.py).

Notes
-----
- This driver uses the project's I2C host abstraction (hardware.i2c_driver.get_i2c_host),
  which typically returns a pigpio host on the Pi (pigpiod must be running).
- Error handling is written to work both with pigpio and with mocks that don't expose
  pigpio.error.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from hardware.i2c_driver import get_i2c_host

_log = logging.getLogger("imu.mpu")

# Try to import pigpio for proper exception typing. Keep optional so mocks work.
try:
    import pigpio as pigpio  # type: ignore
    _PigpioError = pigpio.error  # pyright: ignore[reportAttributeAccessIssue]
except Exception:  # pragma: no cover
    pigpio = None  # type: ignore
    _PigpioError = Exception


# --------------------------------------------------------------------------
# MPU6050 Registers
# --------------------------------------------------------------------------
MPU_WHO_AM_I = 0x75
MPU_PWR_MGMT_1 = 0x6B
MPU_SMPLRT_DIV = 0x19
MPU_CONFIG = 0x1A
MPU_GYRO_CONFIG = 0x1B
MPU_ACCEL_CONFIG = 0x1C

MPU_ACCEL_XOUT_H = 0x3B  # 14 bytes from here
# [AXH, AXL, AYH, AYL, AZH, AZL, TEMP_H, TEMP_L, GXH, GXL, GYH, GYL, GZH, GZL]


class MPU6050Driver:
    def __init__(
        self,
        controller_params: dict | None = None,
        *,
        i2c_bus: int | None = None,
        i2c_addr: int | None = None,
        i2c_flags: int = 0,
    ) -> None:
        params = controller_params or {}
        self._bus = int(i2c_bus if i2c_bus is not None else params.get("I2C_BUS", 1))
        self._addr = int(i2c_addr if i2c_addr is not None else params.get("I2C_ADDR", 0x68))

        self._pi = get_i2c_host(params)
        self._h: Optional[int] = None

        # Open handle
        self._h = self._pi.i2c_open(self._bus, self._addr, i2c_flags)

        _log.info(
            "MPU6050(I2C): bus=%d addr=0x%02X host=%s",
            self._bus,
            self._addr,
            "mock" if hasattr(self._pi, "set_motor_cmd") else "pigpio",
        )

        self._init_device()

    # ----------------------------------------------------------------------
    def _write_byte(self, reg: int, value: int) -> None:
        if self._h is None:
            raise RuntimeError("I2C handle not open")
        try:
            self._pi.i2c_write_byte_data(self._h, reg & 0xFF, value & 0xFF) # pyright: ignore[reportOptionalMemberAccess]
        except _PigpioError as e:
            raise RuntimeError(
                f"MPU6050 I2C write failed: bus={self._bus} addr=0x{self._addr:02X} reg=0x{reg & 0xFF:02X} val=0x{value & 0xFF:02X} ({e})"
            ) from e

    def _read_byte(self, reg: int) -> int:
        if self._h is None:
            raise RuntimeError("I2C handle not open")
        try:
            return int(self._pi.i2c_read_byte_data(self._h, reg & 0xFF)) # pyright: ignore[reportOptionalMemberAccess]
        except _PigpioError as e:
            raise RuntimeError(
                f"MPU6050 I2C read failed: bus={self._bus} addr=0x{self._addr:02X} reg=0x{reg & 0xFF:02X} ({e})"
            ) from e

    def _read_block(self, reg: int, length: int) -> bytes:
        """Read bytes one-by-one for maximum reliability across adapters."""
        out = bytearray()
        for i in range(length):
            out.append(self._read_byte(reg + i) & 0xFF)
        return bytes(out)

    # ----------------------------------------------------------------------
    def _init_device(self) -> None:
        """Full MPU6050 init sequence."""

        who = self._read_byte(MPU_WHO_AM_I)
        if who != 0x68:
            _log.warning("WHO_AM_I=0x%02X (expected 0x68)", who)
        else:
            _log.info("WHO_AM_I=0x68 OK")

        # Wake up device (SLEEP=0)
        self._write_byte(MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.005)

        # DLPF (CONFIG): 0x03 -> ~44Hz accel/gyro bandwidth
        self._write_byte(MPU_CONFIG, 0x03)

        # Gyro FS = ±250 dps
        self._write_byte(MPU_GYRO_CONFIG, 0x00)

        # Accel FS = ±2g
        self._write_byte(MPU_ACCEL_CONFIG, 0x00)

        # Sample rate = 1 kHz / (1 + SMPLRT_DIV)
        # Here: 1 kHz / 8 = 125 Hz
        self._write_byte(MPU_SMPLRT_DIV, 0x07)

        time.sleep(0.05)
        _log.info("MPU6050 initialized: accel+gyro active, LPF~44Hz, 125Hz output")

    # ----------------------------------------------------------------------
    def read_all_axes(self) -> tuple[int, int, int, int, int, int]:
        """Return raw (AX, AY, AZ, GX, GY, GZ) as 16-bit signed integers."""
        block = self._read_block(MPU_ACCEL_XOUT_H, 14)

        ax = int.from_bytes(block[0:2], "big", signed=True)
        ay = int.from_bytes(block[2:4], "big", signed=True)
        az = int.from_bytes(block[4:6], "big", signed=True)

        gx = int.from_bytes(block[8:10], "big", signed=True)
        gy = int.from_bytes(block[10:12], "big", signed=True)
        gz = int.from_bytes(block[12:14], "big", signed=True)

        return ax, ay, az, gx, gy, gz

    # ----------------------------------------------------------------------
    def close(self) -> None:
        """Close I2C handle and stop the pigpio connection if applicable."""
        try:
            if self._pi is not None and self._h is not None:
                try:
                    self._pi.i2c_close(self._h)
                finally:
                    self._h = None
        finally:
            stop = getattr(self._pi, "stop", None)
            if callable(stop):
                stop()
            self._pi = None

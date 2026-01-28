"""
MPU6050 I2C Driver
------------------

Drop-in replacement for the prior IMU device driver in the Carriage Stabilization project.

This module exposes:
    read_all_axes() -> (AX, AY, AZ, GX, GY, GZ)

All values are returned as 16-bit signed integers, matching the format expected
by IMU_Driver (see imu_driver.py).

Notes
-----
- Uses the project's I2C host abstraction: hardware.i2c_driver.get_i2c_host().
- Designed to work with both a real pigpio-backed host (on a Raspberry Pi) and
  test/mocked hosts.
"""

from __future__ import annotations

import logging
import time
from typing import Optional, Protocol, cast

from hardware import i2c_driver

_log = logging.getLogger("imu.mpu")


# --------------------------------------------------------------------------
# Minimal I2C host interface for static type checkers (Pylance/Pyright)
# --------------------------------------------------------------------------
class I2CHost(Protocol):
    def i2c_open(self, bus: int, addr: int, flags: int = 0) -> int: ...
    def i2c_close(self, handle: int) -> None: ...
    def i2c_read_byte_data(self, handle: int, reg: int) -> int: ...
    def i2c_write_byte_data(self, handle: int, reg: int, value: int) -> None: ...
    def stop(self) -> None: ...


# --------------------------------------------------------------------------
# MPU6050 Registers
# --------------------------------------------------------------------------
MPU_WHO_AM_I = 0x75
MPU_PWR_MGMT_1 = 0x6B
MPU_SMPLRT_DIV = 0x19
MPU_CONFIG = 0x1A
MPU_GYRO_CONFIG = 0x1B
MPU_ACCEL_CONFIG = 0x1C

MPU_ACCEL_XOUT_H = 0x3B  # 14 bytes from here:
# [AXH, AXL, AYH, AYL, AZH, AZL, TEMP_H, TEMP_L, GXH, GXL, GYH, GYL, GZH, GZL]


class MPU6050Driver:
    """
    MPU6050 I2C driver using the project I2C-host abstraction.

    controller_params keys used:
      - I2C_BUS (int, default 1)
      - I2C_ADDR (int, default 0x68)
      - I2C_FLAGS (int, default 0)
    """

    def __init__(self, controller_params: Optional[dict] = None):
        params = controller_params or {}

        self._bus: int = int(params.get("I2C_BUS", 1))
        self._addr: int = int(params.get("I2C_ADDR", 0x68))
        self._flags: int = int(params.get("I2C_FLAGS", 0))

        # get_i2c_host() often returns a pigpio "pi" object; its type stubs may not
        # include I2C methods. We cast to our Protocol so Pylance is satisfied.
        self._pi: I2CHost = cast(I2CHost, i2c_driver.get_i2c_host(params))
        self._h: int = -1
        self._closed: bool = False

        # Open handle (support both pigpio signature and some mock signatures)
        try:
            self._h = self._pi.i2c_open(self._bus, self._addr, self._flags)
        except TypeError:
            self._h = self._pi.i2c_open(self._bus, self._addr)

        _log.info("MPU6050(I2C): bus=%d addr=0x%02X", self._bus, self._addr)

        self._init_device()

    # ----------------------------------------------------------------------
    def _require_open(self) -> None:
        if self._closed or self._h < 0:
            raise RuntimeError("MPU6050 I2C handle not open (device closed?)")

    def _write_byte(self, reg: int, value: int) -> None:
        self._require_open()
        try:
            self._pi.i2c_write_byte_data(self._h, reg & 0xFF, value & 0xFF)
        except Exception as e:
            raise RuntimeError(
                f"MPU6050 I2C write failed: bus={self._bus} addr=0x{self._addr:02X} "
                f"reg=0x{reg & 0xFF:02X} val=0x{value & 0xFF:02X} ({e})"
            ) from e

    def _read_byte(self, reg: int) -> int:
        self._require_open()
        try:
            return int(self._pi.i2c_read_byte_data(self._h, reg & 0xFF))
        except Exception as e:
            raise RuntimeError(
                f"MPU6050 I2C read failed: bus={self._bus} addr=0x{self._addr:02X} "
                f"reg=0x{reg & 0xFF:02X} ({e})"
            ) from e

    def _read_block(self, reg: int, length: int) -> bytes:
        """Read bytes one-by-one for maximum compatibility across adapters and mocks."""
        out = bytearray()
        for i in range(int(length)):
            out.append(self._read_byte(reg + i) & 0xFF)
        return bytes(out)

    # ----------------------------------------------------------------------
    def _init_device(self) -> None:
        """MPU6050 init sequence."""
        who = self._read_byte(MPU_WHO_AM_I)
        if who != 0x68:
            _log.warning("WHO_AM_I=0x%02X (expected 0x68)", who)
        else:
            _log.info("WHO_AM_I=0x68 OK")

        # Wake up device (SLEEP=0)
        self._write_byte(MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.005)

        # DLPF (CONFIG): 0x03 -> ~44Hz accel/gyro bandwidth (common baseline)
        self._write_byte(MPU_CONFIG, 0x03)

        # Gyro FS = ±250 dps
        self._write_byte(MPU_GYRO_CONFIG, 0x00)

        # Accel FS = ±2g
        self._write_byte(MPU_ACCEL_CONFIG, 0x00)

        # Sample rate = 1 kHz / (1 + SMPLRT_DIV); 0x07 => 125 Hz
        self._write_byte(MPU_SMPLRT_DIV, 0x07)

        time.sleep(0.05)
        _log.info("MPU6050 initialized: accel+gyro active, LPF~44Hz, 125Hz output")

    # ----------------------------------------------------------------------
    def read_all_axes(self) -> tuple[int, int, int, int, int, int]:
        """
        Return raw (AX, AY, AZ, GX, GY, GZ) as 16-bit signed integers.

        MPU6050 register stream is big-endian per axis.
        """
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
        """Close I2C handle and stop the host connection if applicable."""
        if self._closed:
            return
        self._closed = True

        try:
            if self._h >= 0:
                self._pi.i2c_close(self._h)
        finally:
            self._h = -1
            try:
                self._pi.stop()
            except Exception:
                pass

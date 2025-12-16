"""
MPU6050 I2C Driver
------------------

Drop-in replacement for LSM6DS3TR_i2c_driver.py for the Carriage Stabilization Project.

This module exposes:
    read_all_axes() → (AX, AY, AZ, GX, GY, GZ)

All values are returned as 16-bit signed integers, matching the format expected
by IMU_Driver (see imu_driver.py).

Initialization configures:
    - Accelerometer: ±2g, 1 kHz internal sample rate, DLPF enabled
    - Gyroscope: ±250 deg/sec, DLPF enabled
    - Sample rate divider: 7 → 1 kHz / (1+7) = 125 Hz output
"""

from __future__ import annotations
import time
import logging
from typing import Iterable
from hardware.i2c_driver import get_i2c_host

_log = logging.getLogger("imu.mpu")

# --------------------------------------------------------------------------
# MPU6050 Registers
# --------------------------------------------------------------------------
MPU_WHO_AM_I      = 0x75
MPU_PWR_MGMT_1    = 0x6B
MPU_SMPLRT_DIV    = 0x19
MPU_CONFIG        = 0x1A
MPU_GYRO_CONFIG   = 0x1B
MPU_ACCEL_CONFIG  = 0x1C

MPU_ACCEL_XOUT_H  = 0x3B  # 14 bytes from here
# [AXH, AXL, AYH, AYL, AZH, AZL, TEMP_H, TEMP_L, GXH, GXL, GYH, GYL, GZH, GZL]

# --------------------------------------------------------------------------
class MPU6050Driver:
    def __init__(
        self,
        controller_params: dict | None = None,
        *,
        i2c_bus: int | None = None,
        i2c_addr: int | None = None,
        i2c_flags: int = 0
    ) -> None:

        params = controller_params or {}
        bus = i2c_bus if i2c_bus is not None else int(params.get("I2C_BUS", 1))
        addr = i2c_addr if i2c_addr is not None else int(params.get("I2C_ADDR", 0x68))

        self._pi = get_i2c_host(params)
        self._h = self._pi.i2c_open(bus, addr, i2c_flags)

        _log.info(
            "MPU6050(I2C): bus=%d addr=0x%02X host=%s",
            bus,
            addr,
            "mock" if hasattr(self._pi, "set_motor_cmd") else "pigpio",
        )

        self._init_device()

    # ----------------------------------------------------------------------
    def _write_byte(self, reg: int, value: int) -> None:
        self._pi.i2c_write_byte_data(self._h, reg & 0xFF, value & 0xFF)

    def _read_byte(self, reg: int) -> int:
        return self._pi.i2c_read_byte_data(self._h, reg & 0xFF)

    def _read_block(self, reg: int, length: int) -> bytes:
        """Read bytes one-by-one for maximum reliability."""
        out = bytearray()
        for i in range(length):
            out.append(self._read_byte(reg + i))
        return bytes(out)

    # ----------------------------------------------------------------------
    def _init_device(self) -> None:
        """Full MPU6050 init sequence."""

        who = self._read_byte(MPU_WHO_AM_I)
        if who != 0x68:
            _log.warning(f"WHO_AM_I = 0x{who:02X} (expected 0x68)")
        else:
            _log.info("WHO_AM_I = 0x68 OK")

        # Wake up device (SLEEP=0)
        self._write_byte(MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.005)

        # Set DLPF to 44 Hz for both accel/gyro
        self._write_byte(MPU_CONFIG, 0x03)

        # Gyro FS = ±250 dps
        self._write_byte(MPU_GYRO_CONFIG, 0x00)

        # Accel FS = ±2g
        self._write_byte(MPU_ACCEL_CONFIG, 0x00)

        # Sample rate = 1 kHz / (1 + SMPLRT_DIV)
        # Here: 1 kHz / 8 = 125 Hz
        self._write_byte(MPU_SMPLRT_DIV, 0x07)

        time.sleep(0.05)
        _log.info("MPU6050 initialized: accel+gyro active, LPF=44Hz, 125Hz output")

    # ----------------------------------------------------------------------
    def read_all_axes(self):
        """
        Returns raw (AX, AY, AZ, GX, GY, GZ) with 16-bit signed integers.
        """

        block = self._read_block(MPU_ACCEL_XOUT_H, 14)

        ax = int.from_bytes(block[0:2],  "big", signed=True)
        ay = int.from_bytes(block[2:4],  "big", signed=True)
        az = int.from_bytes(block[4:6],  "big", signed=True)

        gx = int.from_bytes(block[8:10], "big", signed=True)
        gy = int.from_bytes(block[10:12],"big", signed=True)
        gz = int.from_bytes(block[12:14],"big", signed=True)

        return ax, ay, az, gx, gy, gz

    # ----------------------------------------------------------------------
    def close(self):
        try:
            self._pi.i2c_close(self._h)
        finally:
            stop = getattr(self._pi, "stop", None)
            if callable(stop):
                stop()
            self._pi = None
            self._h = None

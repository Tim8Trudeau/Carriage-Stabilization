# utils/imu_watch.py
# Simple Pi-only IMU stream for the MPU-6050:
# prints normalized AX, AY, AZ and ωy in [-1, 1], plus a quick θ estimate.
#
# Requires pigpio daemon running on the Pi:
#   sudo systemctl start pigpiod   # or: sudo pigpiod
#
# Notes (your current axis conventions):
#   - θ is derived from (AX, AZ): theta_rad = atan2(AX, -AZ)
#   - ω (carriage angular rate) is derived from gyro Y (GY), positive for CCW.
#
# Default I2C address is 0x68 (AD0 low). If AD0 is high, use 0x69.

from __future__ import annotations

import math
import os
import time

from hardware.mpu6050_i2c_driver import MPU6050Driver

# MPU-6050 registers used for quick sanity check
MPU_WHO_AM_I = 0x75  # expected 0x68

# Normalization constants (match the rest of your stack’s conventions)
ACCEL_RAW_1G = 16384  # raw counts for ~1 g at ±2g
OMEGA_FS = 32768      # map raw 16-bit gyro to [-1, 1] by full-scale counts

# Normalization range for theta. theta_norm = theta_rad/(pi/2) maps:
#   0 deg -> 0, +90 deg -> +1, -90 deg -> -1
THETA_RANGE_RAD = math.pi / 2


def _clamp_1(v: float) -> float:
    return -1.0 if v < -1.0 else (1.0 if v > 1.0 else v)


def main() -> None:
    bus = int(os.getenv("IMU_I2C_BUS", "1"))
    addr = int(os.getenv("IMU_I2C_ADDR", "0x68"), 0)

    dev = None
    try:
        dev = MPU6050Driver(controller_params={"I2C_BUS": bus, "I2C_ADDR": addr})

        # Helpful warning if we accidentally hit the mock host
        is_mock = hasattr(dev, "_pi") and getattr(dev._pi, "set_motor_cmd", None) is not None
        if is_mock:
            print("WARNING: mock I2C host in use — ensure pigpio daemon is running (sudo systemctl start pigpiod).")

        # WHO_AM_I sanity check (expected 0x68)
        who = dev._read_byte(MPU_WHO_AM_I)  # util use of a private helper is OK here
        print(f"WHO_AM_I=0x{who:02X} (expect 0x68)  bus={bus} addr=0x{addr:02X}")

        # Stream at a terminal-friendly rate
        hz = float(os.getenv("IMU_WATCH_HZ", "25"))
        dt = 1.0 / max(1.0, hz)

        while True:
            ax, ay, az, gx, gy, gz = dev.read_all_axes()

            ax_n = _clamp_1(ax / float(ACCEL_RAW_1G))
            ay_n = _clamp_1(ay / float(ACCEL_RAW_1G))
            az_n = _clamp_1(az / float(ACCEL_RAW_1G))

            wy_n = _clamp_1(gy / float(OMEGA_FS))

            theta_rad = math.atan2(float(ax), float(-az))
            theta_deg = theta_rad * 180.0 / math.pi
            theta_n = _clamp_1(theta_rad / THETA_RANGE_RAD)

            print(
                f"AX:{ax_n:+.3f}  AY:{ay_n:+.3f}  AZ:{az_n:+.3f}  "
                f"ωy:{wy_n:+.3f}  θ:{theta_deg:+6.1f}° (n={theta_n:+.3f})"
            )

            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Keep this message practical: in your last capture, i2cdetect found no devices.
        print(f"IMU watch failed: {e}")
        print(f"Quick checks:")
        print(f"  - Is I2C enabled? (sudo raspi-config -> Interface Options -> I2C)")
        print(f"  - Does the device show up? (sudo i2cdetect -y {bus})  expect 68 or 69")
        print(f"  - Wiring: SDA=GPIO2(pin3), SCL=GPIO3(pin5), GND common, correct VCC")
        raise
    finally:
        if dev is not None:
            dev.close()


if __name__ == "__main__":
    main()

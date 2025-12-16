#!/usr/bin/env python3
"""
Terminal oscilloscope for MPU6050 IMU.
Plots AX, AY, AZ, GX, GY, GZ as ASCII waveforms in real-time over SSH.
"""

import time
import os
import asciichartpy as ascii
from collections import deque

from hardware.mpu6050_i2c_driver import MPU6050Driver


WINDOW = 120
REFRESH = 0.05  # 20 FPS


def main():
    dev = MPU6050Driver(controller_params={"I2C_BUS":1, "I2C_ADDR":0x68})
    time.sleep(0.2)

    ax_buf = deque([0]*WINDOW, maxlen=WINDOW)
    ay_buf = deque([0]*WINDOW, maxlen=WINDOW)
    az_buf = deque([0]*WINDOW, maxlen=WINDOW)
    gx_buf = deque([0]*WINDOW, maxlen=WINDOW)
    gy_buf = deque([0]*WINDOW, maxlen=WINDOW)
    gz_buf = deque([0]*WINDOW, maxlen=WINDOW)

    try:
        while True:
            ax, ay, az, gx, gy, gz = dev.read_all_axes()

            ax_buf.append(ax)
            ay_buf.append(ay)
            az_buf.append(az)
            gx_buf.append(gx)
            gy_buf.append(gy)
            gz_buf.append(gz)

            os.system("clear")
            print("MPU6050 IMU Terminal Oscilloscope (CTRL-C to exit)\n")

            print("AX (motor-shaft direction — unused for tilt):")
            print(ascii.plot(list(ax_buf), {"height": 6}), "\n")

            print("AY (tangential — used in atan2):")
            print(ascii.plot(list(ay_buf), {"height": 6}), "\n")

            print("AZ (radial inward — used in atan2):")
            print(ascii.plot(list(az_buf), {"height": 6}), "\n")

            print("GX (rotation axis — wheel angular velocity):")
            print(ascii.plot(list(gx_buf), {"height": 6}), "\n")

            print("GY:")
            print(ascii.plot(list(gy_buf), {"height": 6}), "\n")

            print("GZ:")
            print(ascii.plot(list(gz_buf), {"height": 6}), "\n")

            time.sleep(REFRESH)

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        dev.close()


if __name__ == "__main__":
    main()

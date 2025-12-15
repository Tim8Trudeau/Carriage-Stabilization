#!/usr/bin/env python3
"""
Terminal oscilloscope for IMU signals (AX, AY, AZ, GX, GY, GZ).
Works over SSH / VSCode terminal. No GUI required.

Uses the corrected LSM6DS3TR_i2c_driver and displays live ASCII plots
for debugging axis orientation, tilt behavior, and gyro response.

Run:
    python -m utils.imu_scope
"""

import time
import os
import asciichartpy as ascii
from collections import deque

from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver


WINDOW = 120     # number of samples in rolling plot window
REFRESH = 0.05   # seconds between frames (≈20 FPS)


def main():
    dev = LSM6DS3TRDriver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x6B})
    time.sleep(0.2)

    # Rolling buffers
    ax_buf = deque([0]*WINDOW, maxlen=WINDOW)
    ay_buf = deque([0]*WINDOW, maxlen=WINDOW)
    az_buf = deque([0]*WINDOW, maxlen=WINDOW)
    gx_buf = deque([0]*WINDOW, maxlen=WINDOW)
    gy_buf = deque([0]*WINDOW, maxlen=WINDOW)
    gz_buf = deque([0]*WINDOW, maxlen=WINDOW)

    try:
        while True:
            # Read all 6 axes
            ax, ay, az, gx, gy, gz = dev.read_all_axes()

            # Append to buffers
            ax_buf.append(ax)
            ay_buf.append(ay)
            az_buf.append(az)
            gx_buf.append(gx)
            gy_buf.append(gy)
            gz_buf.append(gz)

            # Clear terminal
            os.system("clear")
            print("IMU Terminal Oscilloscope (CTRL-C to exit)\n")

            # ACCELEROMETER -----------------------------------------------------
            print("AX (motor-shaft direction — not used for tilt):")
            print(ascii.plot(list(ax_buf), {"height": 6}))
            print()

            print("AY (tangential — tilt numerator):")
            print(ascii.plot(list(ay_buf), {"height": 6}))
            print()

            print("AZ (radial inward — tilt denominator):")
            print(ascii.plot(list(az_buf), {"height": 6}))
            print()

            # GYROSCOPE ---------------------------------------------------------
            print("GX (wheel rotation axis — OMEGA):")
            print(ascii.plot(list(gx_buf), {"height": 6}))
            print()

            print("GY:")
            print(ascii.plot(list(gy_buf), {"height": 6}))
            print()

            print("GZ:")
            print(ascii.plot(list(gz_buf), {"height": 6}))
            print()

            time.sleep(REFRESH)

    except KeyboardInterrupt:
        print("\nExiting imu_scope.")
    finally:
        dev.close()


if __name__ == "__main__":
    main()

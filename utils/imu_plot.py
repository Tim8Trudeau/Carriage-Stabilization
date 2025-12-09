"""
Real-time IMU axis plotting tool.
Plots all 6 raw axes (AX, AY, AZ, GX, GY, GZ) in real time.
Use this to determine the correct axis mapping for the carriage orientation.

Run:
    python -m utils.imu_plot
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver

WHO_AM_I  = 0x0F
CTRL1_XL  = 0x10
CTRL2_G   = 0x11

WINDOW_SEC = 5.0         # rolling window of 5 seconds
FPS = 52                 # IMU ODR (set by driver)

def extract_all_axes(block):
    """
    Given a 12-byte block read starting at OUTX_L_G, extract:
    GX, GY, GZ, AX, AY, AZ  (each 16-bit signed)
    """
    gx = int.from_bytes(block[0:2],  "little", signed=True)
    gy = int.from_bytes(block[2:4],  "little", signed=True)
    gz = int.from_bytes(block[4:6],  "little", signed=True)
    ax = int.from_bytes(block[6:8],  "little", signed=True)
    ay = int.from_bytes(block[8:10], "little", signed=True)
    az = int.from_bytes(block[10:12],"little", signed=True)
    return ax, ay, az, gx, gy, gz


def main():
    dev = LSM6DS3TRDriver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x6B})
    time.sleep(0.3)

    # WHO_AM_I
    who = dev.read(WHO_AM_I, 1)[0]
    print(f"WHO_AM_I=0x{who:02X} (expect 0x69)")

    # Configure ODR = 52 Hz for accel + gyro
    dev.write(CTRL1_XL, bytes([0x30]))
    dev.write(CTRL2_G,  bytes([0x30]))

    print("Configured accel+gyro ODR=52Hz, FS=default. Starting live plot...")

    # rolling buffers
    N = int(WINDOW_SEC * FPS)
    tbuf = deque(maxlen=N)
    axbuf = deque(maxlen=N)
    aybuf = deque(maxlen=N)
    azbuf = deque(maxlen=N)
    gxbuf = deque(maxlen=N)
    gybuf = deque(maxlen=N)
    gzbuf = deque(maxlen=N)

    # Matplotlib setup
    plt.ion()
    fig, (ax_accel, ax_gyro) = plt.subplots(2, 1, figsize=(12, 8))

    ax_accel.set_title("Accelerometer (raw LSBs)")
    ax_accel.set_ylim([-20000, 20000])
    ax_accel.grid(True)

    ax_gyro.set_title("Gyroscope (raw LSBs)")
    ax_gyro.set_ylim([-20000, 20000])
    ax_gyro.grid(True)

    # plot handles
    (l_ax,) = ax_accel.plot([], [], label="AX")
    (l_ay,) = ax_accel.plot([], [], label="AY")
    (l_az,) = ax_accel.plot([], [], label="AZ")
    ax_accel.legend(loc="upper right")

    (l_gx,) = ax_gyro.plot([], [], label="GX")
    (l_gy,) = ax_gyro.plot([], [], label="GY")
    (l_gz,) = ax_gyro.plot([], [], label="GZ")
    ax_gyro.legend(loc="upper right")

    start = time.perf_counter()

    try:
        while True:
            block = dev._read_block(0x22, 12)  # OUTX_L_G
            ax, ay, az, gx, gy, gz = extract_all_axes(block)

            now = time.perf_counter() - start

            tbuf.append(now)
            axbuf.append(ax)
            aybuf.append(ay)
            azbuf.append(az)
            gxbuf.append(gx)
            gybuf.append(gy)
            gzbuf.append(gz)

            # update accel plot
            l_ax.set_data(tbuf, axbuf)
            l_ay.set_data(tbuf, aybuf)
            l_az.set_data(tbuf, azbuf)
            ax_accel.set_xlim([now - WINDOW_SEC, now])

            # update gyro plot
            l_gx.set_data(tbuf, gxbuf)
            l_gy.set_data(tbuf, gybuf)
            l_gz.set_data(tbuf, gzbuf)
            ax_gyro.set_xlim([now - WINDOW_SEC, now])

            plt.pause(0.001)

    except KeyboardInterrupt:
        print("Stopping IMU plot.")
    finally:
        dev.close()


if __name__ == "__main__":
    main()

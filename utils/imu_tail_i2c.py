# utils/imu_tail_i2c.py
# Simple real-time IMU tail: print normalized X, Y, omega in [-1, 1] @ ~20 Hz.

from __future__ import annotations
import time
import math

from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver

# --- LSM6DS3TR-C registers we touch (minimal) ---
WHO_AM_I   = 0x0F
CTRL1_XL   = 0x10  # accel ODR/FS
CTRL2_G    = 0x11  # gyro  ODR/FS
# CTRL3_C is set by the driver to BDU=1, IF_INC=1

# --- Config (edit if your wiring differs) ---
I2C_BUS  = 1
I2C_ADDR = 0x6B          # 0x6A if SDO/SA0 is tied low
ACCEL_RAW_FS = 16384     # ≈ 1 g in raw LSB for ±2g (project convention)
OMEGA_FS_LSB = 32768     # full-scale mapping for normalized gyro (project convention)

# Choose modest ODR (52 Hz) and default full-scales (±2g, default gyro FS):
# For LSM6DS3TR family, ODR field is in bits [7:4]. 0x40 selects ~52 Hz.
_ODR_52HZ = 0x40
_ACCEL_FS_BITS = 0x00    # ±2g
_GYRO_FS_BITS  = 0x00    # lowest FS (typical default)

def _clamp1(x: float) -> float:
    return -1.0 if x < -1.0 else 1.0 if x > 1.0 else x

def main() -> None:
    dev = LSM6DS3TRDriver(
        controller_params={"I2C_BUS": I2C_BUS, "I2C_ADDR": I2C_ADDR}
    )
    try:
        # Sanity: WHO_AM_I should be 0x69 for LSM6DS3TR-C
        who = dev.read(WHO_AM_I, 1)[0]
        print(f"WHO_AM_I: 0x{who:02X}")
        if who != 0x69:
            print("Warning: unexpected WHO_AM_I (expect 0x69). Continuing anyway...")

        # Enable sensors (ODR ~52 Hz). Keep default full-scale ranges.
        dev.write(CTRL1_XL, bytes([_ODR_52HZ | _ACCEL_FS_BITS]))
        dev.write(CTRL2_G,  bytes([_ODR_52HZ | _GYRO_FS_BITS]))

        print("Reading IMU... (Ctrl+C to stop)")
        period_s = 0.050  # 50 ms ≈ 20 Hz

        # Simple warm-up read (discard once to allow first data to settle)
        try:
            dev.read_ax_ay_gz_bytes(timeout_s=0.010)
        except Exception:
            pass

        while True:
            t0 = time.perf_counter()

            # 6 bytes: [AX_L,AX_H, AY_L,AY_H, GZ_L,GZ_H]
            six = dev.read_ax_ay_gz_bytes(timeout_s=0.010)
            ax = int.from_bytes(six[0:2], "little", signed=True)
            ay = int.from_bytes(six[2:4], "little", signed=True)
            gz = int.from_bytes(six[4:6], "little", signed=True)

            # Normalize to [-1, 1] (linear + clamp)
            x_norm = _clamp1(ax / float(ACCEL_RAW_FS))
            y_norm = _clamp1(ay / float(ACCEL_RAW_FS))
            omega_norm = _clamp1(gz / float(OMEGA_FS_LSB))

            # Print one line, overwrite-friendly
            print(f"X={x_norm:+.3f}  Y={y_norm:+.3f}  ω={omega_norm:+.3f}", flush=True)

            # Sleep to maintain ~50 ms loop
            dt = time.perf_counter() - t0
            # Avoid negative sleep if read was slow
            time.sleep(period_s - dt if dt < period_s else 0.0)

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        dev.close()

if __name__ == "__main__":
    main()

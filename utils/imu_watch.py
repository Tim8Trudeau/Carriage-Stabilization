# utils/imu_watch.py
# Simple Pi-only IMU stream: prints normalized X, Y, and ωz in [-1, 1].
# Requires pigpio daemon running (sudo pigpiod).

from __future__ import annotations

from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver  # uses i2c_driver under the hood

# LSM6DS3TR-C registers we touch here
WHO_AM_I  = 0x0F
CTRL1_XL  = 0x10   # accel ODR/FS
CTRL2_G   = 0x11   # gyro  ODR/FS
# CTRL3_C is handled by the driver (__init__ sets BDU=1, IF_INC=1)

# Normalization constants (match the rest of your stack)
ACCEL_RAW_FS = 16384   # ≈ raw counts for 1 g at ±2g
OMEGA_FS     = 32768   # raw LSBs that map to ±full-scale for ω-norm

def _clamp_1(v: float) -> float:
    return -1.0 if v < -1.0 else (1.0 if v > 1.0 else v)

def main():
    # Bus/address for SDO/SA0 pulled high → 0x6B (low would be 0x6A)
    dev = LSM6DS3TRDriver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x6B})
    try:
        # Helpful warning if we accidentally hit the mock host
        is_mock = hasattr(dev, "_pi") and hasattr(dev._pi, "set_motor_cmd")
        if is_mock:
            print("WARNING: mock I2C host in use — ensure pigpio daemon is running (sudo pigpiod).")

        # WHO_AM_I sanity check (expected 0x69)
        who = dev.read(WHO_AM_I, 1)[0]
        print(f"WHO_AM_I=0x{who:02X} (expect 0x69)")

        # Configure **52 Hz ODR** for accel and gyro, keep default full-scale
        # For LSM6DS3TR series, ODR code 0b0011 (<<4) → 52 Hz → value 0x30 in CTRL1_XL/CTRL2_G.
        dev.write(CTRL1_XL, bytes([0x30]))   # accel 52 Hz, ±2g default
        dev.write(CTRL2_G,  bytes([0x30]))   # gyro  52 Hz, ±245 dps default
        print("Configured: ODR=52 Hz (XL,G). BDU+IF_INC set by driver.")

        # Loop: the driver's helper polls STATUS (XLDA|GDA) and returns 6B: [AX, AY, GZ]
        while True:
            six = dev.read_ax_ay_gz_bytes(timeout_s=0.2)  # polls STATUS_XLDA|GDA for ~52 Hz pacing

            ax = int.from_bytes(six[0:2], "little", signed=True)
            ay = int.from_bytes(six[2:4], "little", signed=True)
            gz = int.from_bytes(six[4:6], "little", signed=True)

            ax_n = _clamp_1(ax / float(ACCEL_RAW_FS))
            ay_n = _clamp_1(ay / float(ACCEL_RAW_FS))
            wz_n = _clamp_1(gz / float(OMEGA_FS))

            # Single-line, human-friendly stream
            print(f"X:{ax_n:+.3f}  Y:{ay_n:+.3f}  ωz:{wz_n:+.3f}")

            # No sleep needed; STATUS polling in read_ax_ay_gz_bytes gates us near 52 Hz

    except KeyboardInterrupt:
        pass
    finally:
        dev.close()

if __name__ == "__main__":
    main()

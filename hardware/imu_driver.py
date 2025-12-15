"""
IMU_Driver
==========

High-level IMU processing layer for the Carriage Stabilization Project.

This driver converts raw inertial measurements from the LSM6DS3TR-C
(through `LSM6DS3TR_i2c_driver`) into **normalized tilt angle (theta_norm)**
and **normalized angular velocity (omega_norm)** suitable for the fuzzy logic
controller.

Coordinate System & Physical Mounting
-------------------------------------

The IMU is mounted on the carriage with the following axes:

    • aZ : points radially inward toward the center of the large wheel
           (+1 g when carriage is upright at the top of the wheel)

    • aY : points tangential to the wheel (in the direction of CCW rotation)
           (0 g when upright; +1 g when rotated 90° CCW)

    • aX : parallel to the motor shafts (front/back direction, not used for tilt)

The carriage behaves like an inverted pendulum whose tilt lies in the **YZ plane**.
The wheel’s rotation axis is parallel to the motor shafts and therefore matches
the IMU’s **gyro X (GX)** axis.

Computed Quantities
-------------------

1. Tilt angle (theta)
   ------------------
   Tilt is computed from the gravity vector projection in the YZ plane:

       theta_rads = atan2(aY_filtered, aZ_filtered)

   This definition ensures:
       • theta = 0      when upright at the top of the wheel
       • theta = +π/2   at 90° CCW rotation
       • theta = -π/2   at 90° CW rotation

   The value is clamped and then normalized to:

       theta_norm = theta_rads / THETA_RANGE_RAD    ∈ [-1, +1]


2. Angular velocity (omega)
   -------------------------
   The carriage rotates around the wheel axis, which aligns with the IMU’s GX:

       ω_raw = GX_filtered

   After first-order low-pass filtering, it is normalized to:

       omega_norm = ω_filtered / GYRO_FULL_SCALE_RAW    ∈ [-1, +1]


Filtering & Soft-Clipping
-------------------------

To suppress noise and handle vibration:

    • Accelerometer channels (aY, aZ) use a first-order IIR LPF
      with coefficient derived from ACCEL_CUTOFF_HZ and SAMPLE_RATE_HZ.

    • Gyro X (GX) uses a similar IIR LPF based on OMEGA_CUTOFF_HZ.

    • Raw accelerometer readings are “soft-clipped” using tanh
      to avoid hard saturation while preserving dynamics.

Configuration Parameters
------------------------

controller_params:
    THETA_RANGE_RAD      – maximum physical angle range for normalization
    GYRO_FULL_SCALE_RADS_S – full-scale rotation rate for normalization
    ACCEL_RAW_FS         – expected ±1 g raw magnitude (for soft clipping)

iir_params:
    SAMPLE_RATE_HZ       – IMU sample rate (typically 52 Hz)
    ACCEL_CUTOFF_HZ      – LPF cutoff for accelerometer filtering
    OMEGA_CUTOFF_HZ      – LPF cutoff for gyro filtering

Data Flow
---------

    LSM6DS3TR_i2c_driver.read_all_axes()
        → AX, AY, AZ, GX, GY, GZ (signed 16-bit)
        → LPF & soft-clipping (AY, AZ, GX)
        → theta_rads = atan2(AY_lp, AZ_lp)
        → theta_norm, omega_norm
        → returned to controller

Returned Values
---------------

read_normalized() → (theta_norm, omega_norm)

Both values lie strictly within [-1.0, +1.0] and are suitable for direct
input into the fuzzy logic controller.

Lifecycle
---------

IMU_Driver owns the underlying LSM6DS3TR driver. Calling `close()` will shut
down the hardware interface cleanly.

"""


import math
import logging
from typing import Tuple, Optional, Any

from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver

imu_log = logging.getLogger("imu")


# --------------------------------------------------------------------------
# Math helpers
# --------------------------------------------------------------------------

def _iir_alpha(sample_rate_hz: float, cutoff_hz: float) -> float:
    """Return 1st-order low-pass EWMA alpha."""
    cutoff_hz = max(1e-6, float(cutoff_hz))
    dt = 1.0 / max(1e-6, float(sample_rate_hz))
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (rc + dt)


def _soft_clip_tanh(v: int, fs: int) -> int:
    """Soft clamp toward ±fs using tanh."""
    if fs <= 0:
        return int(v)
    return int(fs * math.tanh(float(v) / float(fs)))


# --------------------------------------------------------------------------
# IMU Driver
# --------------------------------------------------------------------------

class IMU_Driver:
    """
    Produces normalized (theta_norm, omega_norm) where:
      theta_norm = atan2(AX_lp, AZ_lp) / THETA_RANGE_RAD
      omega_norm = LPF(GX_raw) / GYRO_FULL_SCALE_RAW

    Raw inputs come from corrected LSM6DS3TR driver:
        AX, AY, AZ, GX, GY, GZ = read_all_axes()
    """

    def __init__(self, iir_params: dict, controller_params: dict) -> None:

        # Store params
        self.controller_params = dict(controller_params) if controller_params else {}
        self.iir_params = dict(iir_params) if iir_params else {}

        # Device driver (corrected, gyro-awake version)
        self._dev = LSM6DS3TRDriver(controller_params=self.controller_params)

        # Normalization limits
        self.theta_range_rad = float(self.controller_params.get("THETA_RANGE_RAD", math.pi))
        self.gyro_full_scale_rps = float(self.controller_params.get("GYRO_FULL_SCALE_RADS_S", 4.363))
        self.accel_raw_fs = int(self.controller_params.get("ACCEL_RAW_FS", 16384))  # ≈1 g raw

        # Filtering parameters
        self.sample_rate_hz = float(self.iir_params.get("SAMPLE_RATE_HZ", 52.0))
        self.accel_cutoff_hz = float(self.iir_params.get("ACCEL_CUTOFF_HZ", 4.0))
        self.omega_cutoff_hz = float(self.iir_params.get("OMEGA_CUTOFF_HZ", 5.0))

        # LPF coefficients
        self.alpha_acc = _iir_alpha(self.sample_rate_hz, self.accel_cutoff_hz)
        self.alpha_omega = _iir_alpha(self.sample_rate_hz, self.omega_cutoff_hz)

        # Filter state
        self._ay_lp = 0.0
        self._az_lp = 0.0
        self._omega_filt = 0.0

        # Gyro raw full-scale LSB for ±1 normalization
        self._OMEGA_FS = 32768.0

        imu_log.info(
            f"IMU_Driver init | sample={self.sample_rate_hz}Hz | "
            f"a_cut={self.accel_cutoff_hz}Hz α_acc={self.alpha_acc:.4f} | "
            f"ω_cut={self.omega_cutoff_hz}Hz α_ω={self.alpha_omega:.4f} | "
            f"θ_range={self.theta_range_rad} rad"
        )

    # ----------------------------------------------------------------------
    # Read & normalize
    # ----------------------------------------------------------------------

    def read_normalized(self):
        # Read: AX, AY, AZ, GX, GY, GZ
        ax, ay, az, gx, gy, gz = self._dev.read_all_axes()

        # Soft-clip accel
        ay_sc = _soft_clip_tanh(ay, self.accel_raw_fs)
        az_sc = _soft_clip_tanh(az, self.accel_raw_fs)

        # LP filter accel
        self._ay_lp += self.alpha_acc * (float(ay_sc) - self._ay_lp)
        self._az_lp += self.alpha_acc * (float(az_sc) - self._az_lp)

        # Correct tilt angle in YZ plane
        theta_rads = math.atan2(self._ay_lp, self._az_lp)
        theta_norm = max(-1.0, min(1.0, theta_rads / self.theta_range_rad))

        # Gyro X is wheel rotation axis
        self._omega_filt += self.alpha_omega * (float(gx) - self._omega_filt)
        omega_norm = max(-1.0, min(1.0, self._omega_filt / self._OMEGA_FS))

        imu_log.debug(
            "raw=(AY=%d AZ=%d GX=%d) | lp=(AY=%.2f AZ=%.2f) | θ=%.3f rad (norm=%.4f) | ω_norm=%.4f",
            ay, az, gx, self._ay_lp, self._az_lp, theta_rads, theta_norm, omega_norm
        )
        return theta_norm, omega_norm

    # ----------------------------------------------------------------------
    def close(self) -> None:
        try:
            if getattr(self, "_dev", None) is not None:
                self._dev.close()
        finally:
            self._dev = None

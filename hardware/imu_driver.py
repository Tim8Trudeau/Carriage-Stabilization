"""
IMU_Driver
==========

High-level IMU processing layer for the Carriage Stabilization Project.

This module converts raw inertial measurements from the active IMU hardware
driver into the two normalized signals used by the fuzzy logic controller:

    • theta_norm  – normalized carriage tilt angle
    • omega_norm  – normalized carriage angular velocity

It is intentionally 1-axis (no DMP/quaternions). It estimates tilt from the
gravity projection in a single plane and rate from the gyro axis aligned with
the wheel/motor-shaft axis.

IMU Hardware Driver Interface
-----------------------------

This module expects the underlying IMU driver to provide:

    read_all_axes() -> (AX, AY, AZ, GX, GY, GZ)

where each value is a signed 16-bit integer in the IMU’s native units.
(For MPU-6050: big-endian register reads, returned as Python ints.)

To stay “drop-in” across IMUs, this module attempts to import a preferred
MPU-6050 driver first, and falls back to the LSM6DS3TR driver if needed.

Project Axis Conventions (Current MPU-6050 Mounting)
----------------------------------------------------

You have empirically verified the new IMU orientation:

Accelerometer:
    • Tilt angle theta is computed in the X–Z plane.
    • At theta=0 (carriage at the top): aZ is maximally negative (≈ -1 g),
      and aX is near zero.
    • At theta=+90° CCW: aX is near +1 g and aZ near 0.
    • aX goes positive for CCW rotation around the big wheel.

Therefore:
    theta_acc = atan2(aX_lp, -aZ_lp)

Gyroscope:
    • Gyro Y is aligned with the wheel rotation axis (parallel to motor shafts).
    • Gyro Y is positive for CCW rotation.
Therefore:
    omega_raw = GY

Computed Quantities
-------------------

1) Tilt angle (theta)
   From filtered accelerometer values:

       theta_acc = atan2(aX_lp, -aZ_lp)

   Normalization:

       theta_norm = clamp(theta / THETA_RANGE_RAD, -1, +1)

2) Angular velocity (omega)
   From filtered gyro Y (with optional bias correction):

       omega_raw  = (GY - gyro_bias_y)
       omega_norm = clamp(omega_lp / OMEGA_FS_RAW, -1, +1)

Filtering, Bias Calibration, and Robustness
-------------------------------------------

A) Gyro bias calibration at startup
   Optionally averages N samples of gyro Y while stationary and subtracts the
   mean bias from subsequent samples. This reduces steady offset and improves
   omega behavior.

B) Complementary tilt estimator (optional)
   When enabled, blends:
       • integrated gyro rate (smooth, drift-prone)
       • accelerometer tilt (absolute, noise-prone)

   Typical form:
       theta_gyro = theta_est + omega_rad_s * dt
       theta_est  = alpha * theta_gyro + (1-alpha) * theta_acc

C) Detect accel “non-gravity” moments
   When the acceleration magnitude deviates significantly from 1 g (vibration,
   shocks, tangential acceleration), the accelerometer tilt is less trustworthy.
   The driver computes |a| and, when outside tolerance, temporarily relies more
   on the gyro path (alpha -> 1.0) to prevent bad accel corrections.

Configuration Parameters
------------------------

controller_params:
    THETA_RANGE_RAD       – normalization range for theta (radians)
    ACCEL_RAW_FS          – soft-clip scale (raw counts)
    ACCEL_1G_RAW          – raw counts corresponding to 1 g (default 16384)
    GYRO_LSB_PER_DPS      – gyro sensitivity (default 131 for ±250 dps)
    DO_GYRO_BIAS_CAL      – enable startup gyro bias calibration (default True)
    GYRO_BIAS_SAMPLES     – number of samples for bias calibration (default 200)
    USE_COMPLEMENTARY     – enable complementary tilt estimator (default False)
    COMP_ALPHA            – base blend factor (e.g. 0.98) (default 0.98)
    ACCEL_MAG_TOL_G       – accel magnitude tolerance around 1 g (default 0.15)

iir_params:
    SAMPLE_RATE_HZ        – nominal sample rate for filtering (default 52 Hz)
    ACCEL_CUTOFF_HZ       – LPF cutoff for accel channels
    OMEGA_CUTOFF_HZ       – LPF cutoff for gyro channel

Returned Values
---------------

read_normalized() -> (theta_norm, omega_norm)

Both outputs are clamped to [-1.0, +1.0] and intended for direct controller input.

Lifecycle
---------

IMU_Driver owns the underlying IMU driver instance. Call close() to release the
I²C handle cleanly.

"""

import math
import time
import logging
from typing import Tuple


# Prefer MPU-6050 if present; otherwise fall back to LSM6DS3TR.
try:
    from hardware.MPU6050_driver import MPU6050Driver as _IMUDevice
except Exception:  # pragma: no cover
    from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver as _IMUDevice


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
    """Soft clamp toward ±fs using tanh (avoids hard saturation)."""
    if fs <= 0:
        return int(v)
    return int(fs * math.tanh(float(v) / float(fs)))


# --------------------------------------------------------------------------
# IMU Driver
# --------------------------------------------------------------------------

class IMU_Driver:
    """
    Produces normalized (theta_norm, omega_norm) using the project’s axis conventions.

    Current conventions (new MPU orientation):
        theta = atan2(aX, -aZ)
        omega = GY
    """

    def __init__(self, iir_params: dict, controller_params: dict) -> None:
        self.controller_params = dict(controller_params) if controller_params else {}
        self.iir_params = dict(iir_params) if iir_params else {}

        # Underlying device driver (MPU6050 preferred if installed)
        self._dev = _IMUDevice(controller_params=self.controller_params)

        # Normalization limits
        self.theta_range_rad = float(self.controller_params.get("THETA_RANGE_RAD", math.pi))

        # Soft-clip scale (raw counts)
        self.accel_raw_fs = int(self.controller_params.get("ACCEL_RAW_FS", 16384))

        # Filtering parameters
        self.sample_rate_hz = float(self.iir_params.get("SAMPLE_RATE_HZ", 52.0))
        self.accel_cutoff_hz = float(self.iir_params.get("ACCEL_CUTOFF_HZ", 4.0))
        self.omega_cutoff_hz = float(self.iir_params.get("OMEGA_CUTOFF_HZ", 5.0))

        self.alpha_acc = _iir_alpha(self.sample_rate_hz, self.accel_cutoff_hz)
        self.alpha_omega = _iir_alpha(self.sample_rate_hz, self.omega_cutoff_hz)

        # --- MPU6050 sensitivity defaults (±250 dps -> 131 LSB/(deg/s))
        self.gyro_lsb_per_dps = float(
            self.controller_params.get("GYRO_LSB_PER_DPS",
            self.iir_params.get("GYRO_LSB_PER_DPS", 131.0))
        )

        # Raw counts that represent ~1g magnitude (MPU6050 at ±2g is 16384 LSB/g)
        self.accel_1g_raw = float(
            self.controller_params.get("ACCEL_1G_RAW",
            self.iir_params.get("ACCEL_1G_RAW", 16384.0))
        )

        # --- Optional features
        self.do_gyro_bias_cal = bool(
            self.controller_params.get("DO_GYRO_BIAS_CAL",
            self.iir_params.get("DO_GYRO_BIAS_CAL", True))
        )
        self.gyro_bias_samples = int(
            self.controller_params.get("GYRO_BIAS_SAMPLES",
            self.iir_params.get("GYRO_BIAS_SAMPLES", 200))
        )

        self.use_complementary = bool(
            self.controller_params.get("USE_COMPLEMENTARY",
            self.iir_params.get("USE_COMPLEMENTARY", False))
        )
        self.comp_alpha = float(
            self.controller_params.get("COMP_ALPHA",
            self.iir_params.get("COMP_ALPHA", 0.98))
        )
        self.accel_mag_tol_g = float(
            self.controller_params.get("ACCEL_MAG_TOL_G",
            self.iir_params.get("ACCEL_MAG_TOL_G", 0.15))
        )

        # Filter state
        self._ax_lp = 0.0
        self._az_lp = 0.0
        self._omega_filt = 0.0

        # Complementary estimator state
        self._gyro_bias_y = 0.0
        self._theta_est = 0.0
        self._last_t = None

        # Gyro raw full-scale LSB for normalization; 16-bit signed range
        # (keeps omega_norm in [-1,1] regardless of configured dps range)
        self._OMEGA_FS_RAW = 32768.0

        imu_log.info(
            "IMU_Driver init | sample=%.2fHz | a_cut=%.2fHz α_acc=%.4f | ω_cut=%.2fHz α_ω=%.4f | "
            "θ_range=%.3frad | complementary=%s | bias_cal=%s",
            self.sample_rate_hz,
            self.accel_cutoff_hz, self.alpha_acc,
            self.omega_cutoff_hz, self.alpha_omega,
            self.theta_range_rad,
            self.use_complementary,
            self.do_gyro_bias_cal,
        )

        # Optional gyro bias calibration (keep carriage still during startup)
        if self.do_gyro_bias_cal:
            self._gyro_bias_y = self._calibrate_gyro_bias_y(self.gyro_bias_samples)
            imu_log.info("Gyro bias calibration complete: bias_y=%.2f raw", self._gyro_bias_y)

    # ----------------------------------------------------------------------
    def _calibrate_gyro_bias_y(self, n: int) -> float:
        """Average gyro Y over n samples. Assumes the carriage is stationary."""
        n = max(10, int(n))
        acc = 0.0
        dt = 1.0 / max(1e-6, self.sample_rate_hz)

        for _ in range(n):
            ax, ay, az, gx, gy, gz = self._dev.read_all_axes()
            acc += float(gy)
            time.sleep(dt)

        return acc / float(n)

    # ----------------------------------------------------------------------
    def read_normalized(self) -> Tuple[float, float]:
        """
        Returns:
            theta_norm ∈ [-1, +1]
            omega_norm ∈ [-1, +1]
        """

        # Read: AX, AY, AZ, GX, GY, GZ
        ax, ay, az, gx, gy, gz = self._dev.read_all_axes()

        # Time step
        now = time.perf_counter()
        if self._last_t is None:
            dt = 1.0 / max(1e-6, self.sample_rate_hz)
        else:
            dt = max(1e-4, now - self._last_t)
        self._last_t = now

        # Soft-clip + LPF accel (use AX and AZ)
        ax_sc = _soft_clip_tanh(ax, self.accel_raw_fs)
        az_sc = _soft_clip_tanh(az, self.accel_raw_fs)

        self._ax_lp += self.alpha_acc * (float(ax_sc) - self._ax_lp)
        self._az_lp += self.alpha_acc * (float(az_sc) - self._az_lp)

        # Accel tilt: theta = atan2(aX, -aZ)
        theta_acc = math.atan2(self._ax_lp, -self._az_lp)

        # Gyro Y bias correction + LPF (omega axis is GY)
        gy_corr = float(gy) - self._gyro_bias_y
        self._omega_filt += self.alpha_omega * (gy_corr - self._omega_filt)

        omega_norm = self._omega_filt / self._OMEGA_FS_RAW
        omega_norm = max(-1.0, min(1.0, omega_norm))

        # Detect non-gravity moments: |a| should be near 1g
        amag = math.sqrt(float(ax) * float(ax) + float(ay) * float(ay) + float(az) * float(az))
        amag_g = amag / max(1e-6, self.accel_1g_raw)
        accel_trust = abs(amag_g - 1.0) <= self.accel_mag_tol_g

        # Optional complementary filter for theta
        if self.use_complementary:
            # Convert filtered omega (raw) -> deg/s -> rad/s
            omega_dps = self._omega_filt / max(1e-9, self.gyro_lsb_per_dps)
            omega_rad_s = omega_dps * (math.pi / 180.0)

            theta_gyro = self._theta_est + omega_rad_s * dt

            # If accel isn't trustworthy, lean entirely on gyro this sample
            alpha = self.comp_alpha if accel_trust else 1.0

            self._theta_est = alpha * theta_gyro + (1.0 - alpha) * theta_acc
            theta_rads = self._theta_est
        else:
            theta_rads = theta_acc

        theta_norm = theta_rads / self.theta_range_rad
        theta_norm = max(-1.0, min(1.0, theta_norm))

        imu_log.debug(
            "raw=(AX=%d AY=%d AZ=%d GY=%d) | lp=(AX=%.1f AZ=%.1f) | "
            "θ_acc=%.3f θ=%.3f (norm=%.4f) | ω_norm=%.4f | |a|=%.2fg trust=%s",
            ax, ay, az, gy,
            self._ax_lp, self._az_lp,
            theta_acc, theta_rads, theta_norm,
            omega_norm,
            amag_g, accel_trust,
        )

        return theta_norm, omega_norm

    # ----------------------------------------------------------------------
    def close(self) -> None:
        try:
            if getattr(self, "_dev", None) is not None:
                self._dev.close()
        finally:
            self._dev = None

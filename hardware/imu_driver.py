# hardware/imu_driver.py
import math
import logging
from typing import Tuple

# Switched to I2C-backed device driver
from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver

imu_log = logging.getLogger("imu")


def _iir_alpha(sample_rate_hz: float, cutoff_hz: float) -> float:
    """Return 1st-order low-pass EWMA alpha: y += alpha * (x - y)."""
    cutoff_hz = max(1e-6, float(cutoff_hz))
    dt = 1.0 / max(1e-6, float(sample_rate_hz))
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (rc + dt)


def _soft_clip_tanh(v: int, fs: int) -> int:
    """
    Soft clamp toward ±fs using tanh. Linear near zero, asymptotic near limits.
    Prevents flat-top bias in atan2 that hard clip would introduce.
    """
    if fs <= 0:
        return int(v)
    return int(fs * math.tanh(float(v) / float(fs)))


class IMU_Driver:
    """
    Reads 6 bytes from the IMU:
        [AX_L, AX_H, AY_L, AY_H, GZ_L, GZ_H]  (all int16 LE)

    Processing:
        - Soft clamp accel (tanh) then LP filter (pre-atan2)
        - theta_rads = atan2(ax_lp, ay_lp) → normalize by THETA_RANGE_RAD
        - LP filter omega (raw LSB) → normalize by ±OMEGA_FS (32768)
    """

    def __init__(self, iir_params: dict, controller_params: dict) -> None:
        # Save params early
        self.controller_params = dict(controller_params) if controller_params else {}
        self.iir_params = dict(iir_params) if iir_params else {}

        # Device driver (it owns the I2C bus)
        self._dev = LSM6DS3TRDriver(controller_params=self.controller_params)
        self._get6 = lambda: self._dev.read_ax_ay_gz_bytes(timeout_s=0.2)        # Normalization ranges
        self.theta_range_rad = float(self.controller_params.get("THETA_RANGE_RAD", math.pi))
        self.gyro_full_scale_rps = float(self.controller_params.get("GYRO_FULL_SCALE_RADS_S", 4.363))

        # Accel model
        self.accel_raw_fs = int(self.controller_params.get("ACCEL_RAW_FS", 16384))  # ≈1 g in raw counts

        # Filtering setup
        self.sample_rate_hz = float(self.iir_params.get("SAMPLE_RATE_HZ", 50.0))
        self.omega_cutoff_hz = float(self.iir_params.get("CUTOFF_FREQ_HZ", 5.0))
        self.accel_cutoff_hz = float(self.iir_params.get("ACCEL_CUTOFF_HZ", 4.0))
        self.alpha_omega = _iir_alpha(self.sample_rate_hz, self.omega_cutoff_hz)
        self.alpha_acc = _iir_alpha(self.sample_rate_hz, self.accel_cutoff_hz)

        # States
        self._omega_raw_filt = 0.0
        self._ax_lp = 0.0
        self._ay_lp = 0.0
        self._OMEGA_FS = 32768  # ± full-scale LSB for ω-norm mapping

        imu_log.info(
            "IMU_Driver init: fs= %.3f Hz | ω_cut= %.3f Hz (α= %.6f) | a_cut= %.3f Hz (α= %.6f) "
            "| θ_rng= %.4f rad | gyro_fs= %.4f rad/s | accel_fs= %d",
            self.sample_rate_hz, self.omega_cutoff_hz, self.alpha_omega,
            self.accel_cutoff_hz, self.alpha_acc,
            self.theta_range_rad, self.gyro_full_scale_rps, self.accel_raw_fs
        )

    def read_normalized(self) -> Tuple[float, float]:
        """
        Returns:
            theta_norm (float): normalized theta in [-1.0, +1.0]
            omega_norm (float): low-pass filtered normalized omega in [-1.0, +1.0]
        """
        buf = self._get6()  # [AX_L,AX_H, AY_L,AY_H, GZ_L,GZ_H]

        raw_x = int.from_bytes(buf[0:2], "little", signed=True)
        raw_y = int.from_bytes(buf[2:4], "little", signed=True)
        raw_omega = int.from_bytes(buf[4:6], "little", signed=True)

        ax_sc = _soft_clip_tanh(raw_x, self.accel_raw_fs)
        ay_sc = _soft_clip_tanh(raw_y, self.accel_raw_fs)

        self._ax_lp += self.alpha_acc * (float(ax_sc) - self._ax_lp)
        self._ay_lp += self.alpha_acc * (float(ay_sc) - self._ay_lp)

        theta_rads = math.atan2(self._ax_lp, self._ay_lp)
        theta_norm = max(-1.0, min(1.0, theta_rads / self.theta_range_rad))

        self._omega_raw_filt += self.alpha_omega * (float(raw_omega) - self._omega_raw_filt)
        omega_norm = float(self._omega_raw_filt / self._OMEGA_FS)
        omega_norm = max(-1.0, min(1.0, omega_norm))

        imu_log.debug(
            "raw_x=%d raw_y=%d raw_ω=%d | ax_lp=%8.2f ay_lp=%8.2f | θ=%.3f rad (norm=%.4f) | ω_norm=%.4f",
            raw_x, raw_y, raw_omega, self._ax_lp, self._ay_lp, theta_rads, theta_norm, omega_norm
        )
        return theta_norm, omega_norm

    def close(self) -> None:
        try:
            if getattr(self, "_dev", None) is not None:
                self._dev.close()
        finally:
            self._dev = None

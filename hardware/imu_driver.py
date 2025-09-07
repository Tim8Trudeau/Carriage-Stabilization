# hardware/imu_driver.py

import math
import logging
from typing import Tuple

import math
import logging
from typing import Tuple

from hardware.spi_driver import SPIBus

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
    Reads 6 bytes from the IMU via SPI (little-endian, signed 16-bit):
        raw_x (accel), raw_y (accel), raw_omega (gyro)

    Processing:
        - Soft clamp raw_x/raw_y to ±ACCEL_RAW_FS using tanh
        - 1st-order low-pass filter on accel axes (pre-atan2)
        - theta_rads = atan2(ax_lp, ay_lp)
        - theta_norm ∈ [-1, +1] via division by THETA_RANGE_RAD (default π)
        - 1st-order low-pass on raw_omega (in LSB), then scale to norm
        - omega_norm ∈ [-1, +1] via division by OMEGA_FS (32768 default)
        - omega_rps = omega_norm * GYRO_FULL_SCALE_RADS_S (for logging)
    """

    def __init__(self, iir_params: dict, controller_params: dict) -> None:
        # Save params early (fixes prior order-of-use issues)
        self.controller_params = dict(controller_params) if controller_params else {}
        self.iir_params = dict(iir_params) if iir_params else {}

        # Registers and SPI wrapper
        self.spi = SPIBus(self.controller_params)

        # Normalization ranges
        self.theta_range_rad = float(self.controller_params.get("THETA_RANGE_RAD", math.pi))
        self.gyro_full_scale_rps = float(self.controller_params.get("GYRO_FULL_SCALE_RADS_S", 4.363))

        # Accel model
        self.accel_raw_fs = int(self.controller_params.get("ACCEL_RAW_FS", 16384))  # ~1 g in raw counts

        # Filtering setup
        self.sample_rate_hz = float(self.iir_params.get("SAMPLE_RATE_HZ", 50.0))
        # Omega cutoff (existing behavior)
        self.omega_cutoff_hz = float(self.iir_params.get("CUTOFF_FREQ_HZ", 5.0))
        self.alpha_omega = _iir_alpha(self.sample_rate_hz, self.omega_cutoff_hz)

        # Accel cutoff (for x/y pre-atan2 LP). If not supplied, use a conservative 4 Hz.
        self.accel_cutoff_hz = float(self.iir_params.get("ACCEL_CUTOFF_HZ", 4.0))
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
            omega_norm_filt (float): low-pass filtered normalized omega in [-1.0, +1.0]
        """
        # Read 6 bytes starting at the IMU data register.
        buf = self.spi.imu_read()  # x, y, omega (int16 LE each)

        # Unpack little-endian signed 16-bit values
        raw_x = int.from_bytes(buf[0:2], "little", signed=True)
        raw_y = int.from_bytes(buf[2:4], "little", signed=True)
        raw_omega = int.from_bytes(buf[4:6], "little", signed=True)

        # --- Accelerometer soft clamp (smooth, avoids hard clip bias) ---
        ax_sc = _soft_clip_tanh(raw_x, self.accel_raw_fs)
        ay_sc = _soft_clip_tanh(raw_y, self.accel_raw_fs)

        # --- Low-pass filter accel axes (pre-atan2) ---
        # EWMA form: y += alpha * (x - y)
        self._ax_lp += self.alpha_acc * (float(ax_sc) - self._ax_lp)
        self._ay_lp += self.alpha_acc * (float(ay_sc) - self._ay_lp)

        # --- Theta from filtered axes ---
        theta_rads = math.atan2(self._ax_lp, self._ay_lp)
        theta_norm = max(-1.0, min(1.0, theta_rads / self.theta_range_rad))

        # --- Omega IIR (filter in raw LSB, then scale) ---
        self._omega_raw_filt += self.alpha_omega * (float(raw_omega) - self._omega_raw_filt)

        omega_norm = float(self._omega_raw_filt / self._OMEGA_FS)
        omega_norm = max(-1.0, min(1.0, omega_norm))
        omega_rps = omega_norm * self.gyro_full_scale_rps

        # Optional diagnostics: accel magnitude vs 1 g proxy
        accel_norm_g = math.sqrt(float(raw_x) ** 2 + float(raw_y) ** 2) / max(1.0, float(self.accel_raw_fs))
        saturated = (abs(raw_x) > self.accel_raw_fs) or (abs(raw_y) > self.accel_raw_fs)

        if callable (get_sim_theta):
            theta_sim = float(get_sim_theta())
            imu_log.debug(
                "θ_compare | θ_imu=%.3f rad | θ_sim=%.3f rad | dθ=%.3f",
                theta_rads, theta_sim, (theta_rads - theta_sim)
            )

        imu_log.debug(
            "raw_x=%.3f raw_y=%.3f raw_ω=%.3f | ax_sc=%.3f ay_sc=%.3f | "
            "ax_lp=%8.2f ay_lp=%8.2f | θ=%.3f rad (norm=%.4f) | "
            "ω_raw_filt=%8.2f ω_norm=%.4f ω_rps=%.4f | |a|/1g=%.3f sat=%s",
            raw_x, raw_y, raw_omega,
            ax_sc, ay_sc,
            self._ax_lp, self._ay_lp,
            theta_rads, theta_norm,
            self._omega_raw_filt, omega_norm, omega_rps,
            accel_norm_g, saturated,
        )
        return theta_norm, omega_norm

    def close(self) -> None:
        """Release SPI resources."""
        try:
            if getattr(self, "spi", None) is not None:
                self.spi.close()
        finally:
            self.spi = None

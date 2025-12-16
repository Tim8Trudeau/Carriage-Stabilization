# test/test_imu_driver_filters.py
import math
from pytest import approx
INT16_MAX = 32767
INT16_MIN = -32768
ACC_FS = 16384
PI = math.pi


def _cap_int16(v: int) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(v)))


def test_soft_clamp_extreme_accel_yields_finite_theta(make_imu):
    """
    With extreme X-accel samples, theta should remain finite (tanh soft clamp),
    not NaN/inf, and stay within [-1, 1] after normalization.
    """
    # Drive close to ±π/2 without overflowing int16
    extreme_x = _cap_int16(10 * ACC_FS)         # caps at 32767
    samples = [(extreme_x, 0, 0)] * 8           # (raw_x, raw_y, raw_omega)

    ctrl = {"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": PI, "GYRO_FULL_SCALE_RADS_S": 4.363}
    iir = {"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 4.0, "CUTOFF_FREQ_HZ": 5.0}

    imu, _fake = make_imu(samples, iir, ctrl)

    try:
        theta_norm = omega_norm = None
        for _ in range(len(samples)):
            theta_norm, omega_norm = imu.read_normalized()
    finally:
        imu.close()

    # Basic sanity checks
    assert theta_norm is not None and omega_norm is not None
    assert math.isfinite(theta_norm)
    assert -1.0 <= theta_norm <= 1.0
    assert omega_norm == approx(0.0, abs=1e-6)

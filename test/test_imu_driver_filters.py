# tests/test_imu_driver_filters.py
import math

# Raw full-scales
ACC_FS = 16384
PI = math.pi

# Signed 16-bit raw limits for IMU bytes
INT16_MAX = 32767
INT16_MIN = -32768


def _cap_int16(v: int) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(v)))


def _run_series(imu, samples):
    """Consume all samples via imu.read_normalized(); return (thetas, omegas)."""
    thetas, omegas = [], []
    for _ in samples:
        th, w = imu.read_normalized()
        thetas.append(th)
        omegas.append(w)
    return thetas, omegas


def test_soft_clamp_extreme_accel_yields_finite_theta(make_imu):
    # Extreme accel on x: drive close to ±π/2 without overflowing int16
    extreme_x = _cap_int16(10 * ACC_FS)   # will cap at 32767
    samples = [(extreme_x, 0, 0)] * 8

    ctrl = {"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": PI, "GYRO_FULL_SCALE_RADS_S": 4.363}
    iir = {"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 4.0, "CUTOFF_FREQ_HZ": 5.0}
    imu, _ = make_imu(samples, iir, ctrl)

    thetas, omegas = _run_series(imu, samples)
    # Near ±π/2 normalized by π ≈ ±0.5 (filtered so not exact)
    assert 0.30 < abs(thetas[-1]) < 0.90
    # Gyro input is zero, filtered omega should be near zero
    assert abs(omegas[-1]) < 1e-6


def test_accel_lowpass_step_response_is_gradual(make_imu):
    # Step on x from 0 -> +1g while y goes +1g -> 0 to move θ toward +π/2
    pre = [(0, ACC_FS, 0)] * 2
    step = [(_cap_int16(ACC_FS), 0, 0)] * 24
    samples = pre + step

    ctrl = {"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": PI, "GYRO_FULL_SCALE_RADS_S": 4.363}
    iir = {"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 4.0, "CUTOFF_FREQ_HZ": 5.0}
    imu, _ = make_imu(samples, iir, ctrl)

    thetas, _ = _run_series(imu, samples)

    # After the step, theta_norm should increase over several samples (not instantaneous)
    post = thetas[2:]  # after step
    assert post[0] < post[3] < post[6]  # monotonic-ish rise
    # Should not immediately hit the asymptote (~0.5)
    assert post[0] < 0.4


def test_accel_impulse_is_suppressed_by_lp(make_imu):
    # Impulse: one large spike in x, else zeros; y stays at +1g
    spike_x = _cap_int16(6 * ACC_FS)  # cap to stay within int16
    samples = [(0, ACC_FS, 0)] * 5 + [(spike_x, ACC_FS, 0)] + [(0, ACC_FS, 0)] * 20

    ctrl = {"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": PI, "GYRO_FULL_SCALE_RADS_S": 4.363}
    iir = {"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 4.0, "CUTOFF_FREQ_HZ": 5.0}
    imu, _ = make_imu(samples, iir, ctrl)

    thetas, _ = _run_series(imu, samples)

    # The spike should cause a small, brief bump—well below the hard limit (~0.5),
    # and it should decay back quickly.
    peak = max(abs(v) for v in thetas)
    assert peak < 0.49
    # ensure it returns near baseline
    assert abs(thetas[-1]) < 0.1


def test_omega_lowpass_matches_first_order_iir(make_imu):
    # Step raw_omega from 0 -> +INT16_MAX so omega_norm input ≈ +INT16_MAX / 32768 ≈ 0.99997
    # (Use INT16_MAX, not 32768, to avoid overflow in FakeSPI.)
    pre = [(0, ACC_FS, 0)] * 2
    step = [(0, ACC_FS, INT16_MAX)] * 20
    samples = pre + step

    # Choose clean parameters
    sr = 50.0
    fc = 5.0
    ctrl = {"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": PI, "GYRO_FULL_SCALE_RADS_S": 4.363}
    iir = {"SAMPLE_RATE_HZ": sr, "ACCEL_CUTOFF_HZ": 4.0, "CUTOFF_FREQ_HZ": fc}
    imu, _ = make_imu(samples, iir, ctrl)

    # Expected α for first-order EWMA: alpha = dt / (RC + dt)
    dt = 1.0 / sr
    rc = 1.0 / (2.0 * math.pi * fc)
    alpha = dt / (rc + dt)

    _, omegas = _run_series(imu, samples)

    # Skip pre-roll; compare a few samples of the step response
    expected = 0.0
    u = INT16_MAX / 32768.0  # slightly less than 1 due to int16 cap
    for k in range(2, 12):
        expected = expected + alpha * (u - expected)  # EWMA toward u
        # Allow generous tolerance for numerical/quantization differences
        assert abs(omegas[k] - expected) < 0.10


def test_accel_cutoff_changes_response_speed(make_imu):
    # Same step input, compare low vs high cutoff
    samples = [(0, ACC_FS, 0)] * 2 + [(_cap_int16(ACC_FS), 0, 0)] * 30
    ctrl = {"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": PI, "GYRO_FULL_SCALE_RADS_S": 4.363}

    imu_slow, _ = make_imu(samples, {"SAMPLE_RATE_HZ": 50.0,
                                     "ACCEL_CUTOFF_HZ": 1.0,
                                     "CUTOFF_FREQ_HZ": 5.0}, ctrl)
    imu_fast, _ = make_imu(samples, {"SAMPLE_RATE_HZ": 50.0,
                                     "ACCEL_CUTOFF_HZ": 20.0,
                                     "CUTOFF_FREQ_HZ": 5.0}, ctrl)

    th_slow, _ = _run_series(imu_slow, samples)
    th_fast, _ = _run_series(imu_fast, samples)

    # Use an EARLIER index so rise difference is more pronounced
    idx = 6  # ~0.12 s after the step at 50 Hz
    assert abs(th_fast[idx]) > abs(th_slow[idx]) + 0.02
    # Both eventually converge near 0.5
    assert abs(th_slow[-1] - 0.5) < 0.1
    assert abs(th_fast[-1] - 0.5) < 0.1


def test_accel_cutoff_sweep_monotonic_response_standalone(make_imu):
    """
    With identical accel step input, higher ACCEL_CUTOFF_HZ should yield a faster rise:
        theta(1 Hz) < theta(4 Hz) < theta(20 Hz)
    Measured at a fixed sample index after the step.
    """
    samples = [(0, ACC_FS, 0)] * 2 + [(_cap_int16(ACC_FS), 0, 0)] * 30
    ctrl = {"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": PI, "GYRO_FULL_SCALE_RADS_S": 4.363}

    def run(cutoff_hz: float) -> float:
        iir = {"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": cutoff_hz, "CUTOFF_FREQ_HZ": 5.0}
        imu, _ = make_imu(samples, iir, ctrl)
        thetas, _ = _run_series(imu, samples)
        return thetas[6]  # earlier snapshot for better separation

    slow = run(1.0)
    mid = run(4.0)
    fast = run(20.0)

    # Monotonic with small slack
    assert slow < mid - 0.01
    assert mid < fast - 0.01
    # still below final (~0.5) at that index
    assert fast < 0.6


def test_no_legacy_mock_driver_is_used_or_imported():
    """
    Ensure imu_driver does not expose or import the legacy mock_LSM6DS3TR driver.
    """
    import sys
    import hardware.imu_driver as imu_mod

    # No IMUDev symbol anymore
    assert not hasattr(imu_mod, "IMUDev")

    # And the legacy module shouldn't be imported as a side-effect
    assert "test.mocks.mock_LSM6DS3TR_driver" not in sys.modules

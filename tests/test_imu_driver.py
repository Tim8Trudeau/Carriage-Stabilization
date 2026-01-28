import importlib
import math
import types
import pytest


class FakeIMUDevice:
    """
    Fake device that returns a programmable sequence of (AX, AY, AZ, GX, GY, GZ).
    """
    def __init__(self, seq):
        self._seq = list(seq)
        self.closed = False

    def read_all_axes(self):
        if not self._seq:
            # keep returning last value if sequence exhausted
            return self._last
        self._last = self._seq.pop(0)
        return self._last

    def close(self):
        self.closed = True


def _make_driver(monkeypatch, seq, *, iir=None, ctrl=None, perf_counter_seq=None, sleep_noop=True):
    import hardware.imu_driver as imu

    # Replace underlying device factory in module namespace
    monkeypatch.setattr(imu, "_IMUDevice", lambda controller_params=None: FakeIMUDevice(seq))

    # Make time deterministic
    if perf_counter_seq is not None:
        it = iter(perf_counter_seq)
        monkeypatch.setattr(imu.time, "perf_counter", lambda: next(it))
    if sleep_noop:
        monkeypatch.setattr(imu.time, "sleep", lambda _dt: None)

    iir_params = {
        "SAMPLE_RATE_HZ": 100.0,
        "ACCEL_CUTOFF_HZ": 1e9,   # alpha ~1
        "OMEGA_CUTOFF_HZ": 1e9,   # alpha ~1
    }
    if iir:
        iir_params.update(iir)

    ctrl_params = {
        "THETA_RANGE_RAD": math.pi,
        "THETA_GAIN": 1.0,
        "DO_GYRO_BIAS_CAL": False,
        "USE_COMPLEMENTARY": False,
    }
    if ctrl:
        ctrl_params.update(ctrl)

    return imu.IMU_Driver(iir_params=iir_params, controller_params=ctrl_params)


def test_theta_norm_basic_zero(monkeypatch):
    # ax=0, az=-1g => theta=0
    seq = [
        (0, 0, -16384, 0, 0, 0),
    ]
    d = _make_driver(monkeypatch, seq)
    theta, omega = d.read_normalized()
    assert abs(theta) < 1e-6
    assert abs(omega) < 1e-6
    d.close()


def test_theta_gain_scales_linearly(monkeypatch):
    # Choose ax=+10000, az=-10000 => atan2(10000, 10000)=pi/4
    seq = [
        (10000, 0, -10000, 0, 0, 0),
    ]
    d = _make_driver(monkeypatch, seq, ctrl={"THETA_GAIN": 2.0})
    theta, _ = d.read_normalized()
    # 2*(pi/4)/pi = 0.5
    assert theta == pytest.approx(0.5, abs=1e-3)
    d.close()


def test_theta_norm_clamps_to_one(monkeypatch):
    # Near +90° already gives theta=~pi/2; with gain=3 -> 1.5*pi/2/pi=1.5 -> clamp to 1
    seq = [
        (16384, 0, 0, 0, 0, 0),
    ]
    d = _make_driver(monkeypatch, seq, ctrl={"THETA_GAIN": 3.0})
    theta, _ = d.read_normalized()
    assert theta == 1.0
    d.close()


def test_omega_norm_from_gy(monkeypatch):
    seq = [
        (0, 0, -16384, 0, 16384, 0),
    ]
    d = _make_driver(monkeypatch, seq)
    _, omega = d.read_normalized()
    assert omega == pytest.approx(16384/32768.0, abs=1e-6)
    d.close()


def test_gyro_bias_calibration_zeroes_constant_rate(monkeypatch):
    # During calibration, gy=100 for N samples. After that, also 100 => omega_norm ~0.
    n = 20
    seq = [(0, 0, -16384, 0, 100, 0)] * (n + 1)
    d = _make_driver(
        monkeypatch,
        seq,
        ctrl={"DO_GYRO_BIAS_CAL": True, "GYRO_BIAS_SAMPLES": n},
        perf_counter_seq=[0.0, 0.01],  # only used after init
    )
    _, omega = d.read_normalized()
    assert abs(omega) < 1e-6
    d.close()


def test_complementary_filter_ignores_bad_accel_when_mag_off(monkeypatch):
    import hardware.imu_driver as imu

    # accel magnitude far from 1g -> accel_trust False => alpha forced to 1.0
    # Provide gyro rate that integrates theta upward. Keep accel tilt at 0 to prove we don't pull back to 0.
    # Use two samples with dt=0.1s.
    seq = [
        (0, 0, -16384*4, 0, 1310, 0),  # huge accel magnitude (4g), gyro=1310 raw => 10 dps
        (0, 0, -16384*4, 0, 1310, 0),
    ]
    d = _make_driver(
        monkeypatch,
        seq,
        ctrl={
            "USE_COMPLEMENTARY": True,
            "COMP_ALPHA": 0.0,         # would fully trust accel if accel_trust True
            "ACCEL_MAG_TOL_G": 0.15,
            "DO_GYRO_BIAS_CAL": False,
            "GYRO_LSB_PER_DPS": 131.0,
        },
        perf_counter_seq=[0.0, 0.1, 0.2],
    )

    # First call sets _theta_est from integrated gyro and alpha=1.0 (because accel_trust False)
    theta1, _ = d.read_normalized()
    theta2, _ = d.read_normalized()

    assert theta2 > theta1  # keeps integrating
    d.close()


def test_math_helpers_iir_alpha_monotonic():
    import hardware.imu_driver as imu
    a1 = imu._iir_alpha(100.0, 1.0)
    a2 = imu._iir_alpha(100.0, 10.0)
    assert 0.0 < a1 < a2 < 1.0


def test_soft_clip_tanh_bounds():
    import hardware.imu_driver as imu
    assert imu._soft_clip_tanh(0, 1000) == 0
    assert abs(imu._soft_clip_tanh(10_000_000, 1000)) <= 1000
    assert abs(imu._soft_clip_tanh(-10_000_000, 1000)) <= 1000

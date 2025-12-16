import math
import sys
import types
import pytest


def _install_fake_mpu6050_driver_module():
    '''
    Ensure importing hardware.imu_driver doesn't fall back to LSM6DS3TR on systems
    where hardware.MPU6050_driver isn't present (e.g., Windows test runs).
    '''
    mod = types.ModuleType("hardware.MPU6050_driver")

    class _PlaceholderMPU6050Driver:
        def __init__(self, controller_params=None, **kwargs):
            self.controller_params = controller_params or {}

        def read_all_axes(self):
            return (0, 0, 0, 0, 0, 0)

        def close(self):
            return None

    mod.MPU6050Driver = _PlaceholderMPU6050Driver
    sys.modules["hardware.MPU6050_driver"] = mod


@pytest.fixture(autouse=True)
def _ensure_fake_mpu_module():
    _install_fake_mpu6050_driver_module()


class StubIMUDevice:
    '''
    Stub IMU device used to drive IMU_Driver unit tests.

    It returns a fixed (ax, ay, az, gx, gy, gz) tuple each call, or uses a generator.
    '''
    def __init__(self, controller_params=None, sequence=None):
        self._seq = iter(sequence) if sequence is not None else None
        self.last = (0, 0, 0, 0, 0, 0)

    def read_all_axes(self):
        if self._seq is None:
            return self.last
        self.last = next(self._seq)
        return self.last

    def close(self):
        return None


def _mk_driver(
    monkeypatch,
    *,
    sample_rate_hz=100.0,
    accel_cut_hz=4.0,
    omega_cut_hz=8.0,
    theta_range=math.pi / 2,
    accel_fs=16384,
    accel_1g=16384.0,
    do_bias=False,
    bias_samples=10,
    use_comp=False,
    comp_alpha=0.98,
    accel_mag_tol_g=0.15,
    stub_sequence=None,
):
    '''
    Construct IMU_Driver with a StubIMUDevice and deterministic timing.
    '''
    import hardware.imu_driver as imu_mod

    imu_mod._IMUDevice = lambda controller_params=None: StubIMUDevice(sequence=stub_sequence)

    # Avoid real sleeps during bias calibration
    monkeypatch.setattr(imu_mod.time, "sleep", lambda _dt: None)

    # Deterministic perf_counter (only matters for complementary filter dt)
    t = {"v": 0.0}

    def _pc():
        t["v"] += 1.0 / sample_rate_hz
        return t["v"]

    monkeypatch.setattr(imu_mod.time, "perf_counter", _pc)

    iir_params = {
        "SAMPLE_RATE_HZ": sample_rate_hz,
        "ACCEL_CUTOFF_HZ": accel_cut_hz,
        "OMEGA_CUTOFF_HZ": omega_cut_hz,
    }
    controller_params = {
        "THETA_RANGE_RAD": theta_range,
        "ACCEL_RAW_FS": accel_fs,
        "ACCEL_1G_RAW": accel_1g,
        "DO_GYRO_BIAS_CAL": do_bias,
        "GYRO_BIAS_SAMPLES": bias_samples,
        "USE_COMPLEMENTARY": use_comp,
        "COMP_ALPHA": comp_alpha,
        "ACCEL_MAG_TOL_G": accel_mag_tol_g,
        "GYRO_LSB_PER_DPS": 131.0,
    }

    return imu_mod.IMU_Driver(iir_params, controller_params)


def test_theta_zero_at_top(monkeypatch):
    '''
    At theta=0 (top): aX≈0, aZ≈-1g so atan2(aX, -aZ) ≈ 0.
    '''
    seq = [
        (0, 0, -16384, 0, 0, 0),
    ]
    d = _mk_driver(monkeypatch, stub_sequence=seq, do_bias=False, use_comp=False)
    theta_n, omega_n = d.read_normalized()

    assert abs(theta_n) < 1e-3
    assert abs(omega_n) < 1e-6


def test_theta_plus_90_ccw(monkeypatch):
    '''
    At +90° CCW: aX≈+1g, aZ≈0 so atan2(aX, -aZ) ≈ +pi/2 -> theta_norm≈+1.
    '''
    seq = [
        (16384, 0, 0, 0, 0, 0),
    ]
    d = _mk_driver(monkeypatch, stub_sequence=seq, do_bias=False, use_comp=False, theta_range=math.pi / 2)
    theta_n, _ = d.read_normalized()
    assert theta_n > 0.90


def test_omega_norm_saturates(monkeypatch):
    '''
    omega_norm is derived from filtered gyro Y divided by 32768 and then clamped.
    A large positive GY should clamp to +1.0.
    '''
    seq = [
        (0, 0, -16384, 0, 40000, 0),
    ]
    d = _mk_driver(monkeypatch, stub_sequence=seq, do_bias=False, use_comp=False)
    _, omega_n = d.read_normalized()
    assert omega_n == 1.0


def test_gyro_bias_calibration_zeroes_omega(monkeypatch):
    '''
    If gyro bias calibration runs on constant GY=+100, subsequent readings of the same
    GY should result in omega near 0 after bias subtraction.
    '''
    seq = [(0, 0, -16384, 0, 100, 0)] * 11  # 10 for calibration + 1 for first read
    d = _mk_driver(monkeypatch, stub_sequence=seq, do_bias=True, bias_samples=10, use_comp=False)
    _, omega_n = d.read_normalized()
    assert abs(omega_n) < 1e-6


def test_complementary_filter_downweights_accel_when_non_gravity(monkeypatch):
    '''
    When complementary filter is enabled and |a| deviates from ~1g, IMU_Driver forces
    alpha=1.0 for that sample (trust gyro only).

    We test this by:
    - seeding theta_est near 0 at the top,
    - injecting a bogus accel magnitude on the second sample,
    - providing a known gyro rate that integrates to a small positive angle.
    '''
    seq = [
        (0, 0, -16384, 0, 0, 0),
        (32768, 0, -32768, 0, 1000, 0),
    ]

    d = _mk_driver(
        monkeypatch,
        stub_sequence=seq,
        do_bias=False,
        use_comp=True,
        comp_alpha=0.5,
        accel_mag_tol_g=0.05,
        sample_rate_hz=100.0,
        theta_range=math.pi / 2,
    )

    d.read_normalized()  # seed
    theta_n, _ = d.read_normalized()

    omega_rad_s = (1000 / 131.0) * (math.pi / 180.0)
    expected_theta = omega_rad_s * (1.0 / 100.0)
    expected_norm = expected_theta / (math.pi / 2)

    assert theta_n < 0.2
    assert abs(theta_n - expected_norm) < 0.05


def test_omega_norm_computes_correctly(monkeypatch):
    '''
    Test omega_norm computation:
    - filtered_gyro_y = 40000 (raw) should clamp to +1.0
    - filtered_gyro_y = -40000 (raw) should clamp to -1.0
    - filtered_gyro_y = 0 (raw) should result in omega_norm = 0.0
    - filtered_gyro_y = 32768 (raw) should result in omega_norm = 1.0
    - filtered_gyro_y = -32768 (raw) should result in omega_norm = -1.0
    '''
    seq = [
        (0, 0, -16384, 0, 40000, 0),
        (0, 0, -16384, 0, -40000, 0),
        (0, 0, -16384, 0, 0, 0),
        (0, 0, -16384, 0, 32768, 0),
        (0, 0, -16384, 0, -32768, 0),
    ]
    d = _mk_driver(monkeypatch, stub_sequence=seq, do_bias=False, use_comp=False)
    for _i in range(5):
        _, omega_n = d.read_normalized()

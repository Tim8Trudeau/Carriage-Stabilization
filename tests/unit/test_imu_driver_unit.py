# tests/unit/test_imu_driver_unit.py
import math
import pytest


class _StubDevSeq:
    """Stub IMU device returning a sequence of (AX, AY, AZ, GX, GY, GZ)."""

    def __init__(self, controller_params=None, seq=None, **_kw):
        self._seq = list(seq or [(0, 0, -16384, 0, 0, 0)])
        self._i = 0

    def read_all_axes(self):
        if self._i < len(self._seq):
            v = self._seq[self._i]
            self._i += 1
        else:
            v = self._seq[-1]
        return v

    def close(self):
        return None


def _patch_underlying_device(monkeypatch, imu_mod, seq):
    # Preferred seam: module-level factory/class _IMUDevice
    if hasattr(imu_mod, "_IMUDevice"):
        monkeypatch.setattr(imu_mod, "_IMUDevice", lambda controller_params=None: _StubDevSeq(seq=seq))
        return
    # Fallback: IMU_Driver may directly construct MPU6050Driver
    if hasattr(imu_mod, "MPU6050Driver"):
        monkeypatch.setattr(imu_mod, "MPU6050Driver", lambda *a, **k: _StubDevSeq(seq=seq), raising=True)
        return
    raise RuntimeError("Cannot patch underlying IMU device constructor in imu_driver.")


@pytest.mark.unit
def test_theta_zero_at_top(monkeypatch):
    """At the top: AX≈0, AZ≈-1g => atan2(AX, -AZ) ≈ 0 => theta_norm≈0."""
    import hardware.imu_driver as imu_mod

    seq = [(0, 0, -16384, 0, 0, 0)]
    _patch_underlying_device(monkeypatch, imu_mod, seq)
    monkeypatch.setattr(imu_mod.time, "sleep", lambda _dt: None, raising=False)

    imu = imu_mod.IMU_Driver(
        iir_params={"SAMPLE_RATE_HZ": 100.0, "ACCEL_CUTOFF_HZ": 50.0, "OMEGA_CUTOFF_HZ": 50.0},
        controller_params={"THETA_RANGE_RAD": math.pi / 2, "DO_GYRO_BIAS_CAL": False},
    )

    theta_n, omega_n = imu.read_normalized()
    imu.close()

    assert abs(theta_n) < 1e-3
    assert abs(omega_n) <= 1.0


@pytest.mark.unit
def test_theta_plus_90_ccw(monkeypatch):
    """At +90° CCW: AX≈+1g, AZ≈0 => theta_norm should be near +1."""
    import hardware.imu_driver as imu_mod

    seq = [(16384, 0, 0, 0, 0, 0)]
    _patch_underlying_device(monkeypatch, imu_mod, seq)

    imu = imu_mod.IMU_Driver(
        iir_params={"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 50.0, "OMEGA_CUTOFF_HZ": 50.0},
        controller_params={"THETA_RANGE_RAD": math.pi / 2, "DO_GYRO_BIAS_CAL": False},
    )

    theta_n, _ = imu.read_normalized()
    imu.close()

    assert theta_n > 0.90


@pytest.mark.unit
def test_omega_sign_and_clamp(monkeypatch):
    """Large +GY should clamp omega_norm to +1.0; large -GY => -1.0."""
    import hardware.imu_driver as imu_mod

    seq = [
        (0, 0, -16384, 0, 50000, 0),
        (0, 0, -16384, 0, -50000, 0),
    ]
    _patch_underlying_device(monkeypatch, imu_mod, seq)

    imu = imu_mod.IMU_Driver(
        iir_params={"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 50.0, "OMEGA_CUTOFF_HZ": 50.0},
        controller_params={"THETA_RANGE_RAD": math.pi / 2, "DO_GYRO_BIAS_CAL": False},
    )

    _, om1 = imu.read_normalized()
    _, om2 = imu.read_normalized()
    imu.close()

    assert om1 == 1.0
    assert om2 == -1.0


@pytest.mark.unit
def test_gyro_bias_calibration_zeroes_constant_gyro(monkeypatch):
    """If bias calibration is enabled, constant gyro during calibration should subtract out."""
    import hardware.imu_driver as imu_mod

    seq = [(0, 0, -16384, 0, 100, 0)] * 21  # 20 for calibration + 1 for read
    _patch_underlying_device(monkeypatch, imu_mod, seq)
    monkeypatch.setattr(imu_mod.time, "sleep", lambda _dt: None, raising=False)

    imu = imu_mod.IMU_Driver(
        iir_params={"SAMPLE_RATE_HZ": 100.0, "ACCEL_CUTOFF_HZ": 50.0, "OMEGA_CUTOFF_HZ": 50.0},
        controller_params={"THETA_RANGE_RAD": math.pi / 2, "DO_GYRO_BIAS_CAL": True, "GYRO_BIAS_SAMPLES": 20},
    )

    _, omega_n = imu.read_normalized()
    imu.close()

    assert abs(omega_n) < 0.02

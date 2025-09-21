# test/unit/test_imu_driver_unit.py
import math
import pytest

INT16_MAX = 32767
INT16_MIN = -32768
ACC_FS = 16384


def _cap_int16(v: int) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(v)))


def _pack3(x, y, g) -> bytes:
    """Little-endian int16 packing for (x_raw, y_raw, omega_raw)."""
    def p(v):
        return int(_cap_int16(v)).to_bytes(2, "little", signed=True)
    return p(x) + p(y) + p(g)


class _FakeDevOnce:
    """Device driver stub that always returns one fixed 6-byte sample."""
    def __init__(self, *a, sample_bytes: bytes, **k):
        self._buf = sample_bytes
        self.closed = False
    def read_ax_ay_gz_bytes(self, *a, **k) -> bytes:
        return self._buf
    def close(self):
        self.closed = True


class _FakeDevSeq:
    """Device driver stub that plays back a sequence of (x,y,g) samples."""
    def __init__(self, *a, seq=None, **k):
        self._seq = [_pack3(x, y, g) for (x, y, g) in (seq or [(0, 0, 0)])]
        self._i = 0
        self.closed = False
    def read_ax_ay_gz_bytes(self, *a, **k) -> bytes:
        if self._i < len(self._seq):
            buf = self._seq[self._i]
            self._i += 1
        else:
            buf = self._seq[-1]
        return buf
    def close(self):
        self.closed = True


@pytest.mark.unit
@pytest.mark.parametrize(
    "raw_y, raw_x, raw_omega",
    [
        (16384, 0, 0),
        (16135, 2845, 0),
        (0, 16384, 0),
        (11585, 11585, 1000),
        (-11585, 11585, 1000),
        (0, 16384, -1000),
        (0, -16384, 0),
        (-8191, 14188, -1000),
        (-8191, -14188, 1000),
        (-11585, 11585, 0),
        (-16384, 0, 0),
        (8192, -14188, 500),
    ],
)
def test_read_normalized_from_inputs(monkeypatch, raw_y, raw_x, raw_omega):
    """IMU_Driver consumes 6 bytes from LSM6DS3TRDriver and normalizes them."""
    import hardware.imu_driver as imu_mod

    buf = _pack3(raw_x, raw_y, raw_omega)
    monkeypatch.setattr(
        imu_mod, "LSM6DS3TRDriver",
        lambda *a, **k: _FakeDevOnce(sample_bytes=buf),
        raising=True,
    )

    iir_params = {"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 50.0, "CUTOFF_FREQ_HZ": 50.0}
    controller_params = {"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": math.pi, "GYRO_FULL_SCALE_RADS_S": 4.363}

    imu = imu_mod.IMU_Driver(iir_params, controller_params)
    try:
        for _ in range(3):
            theta_norm, omega_norm = imu.read_normalized()
    finally:
        imu.close()

    theta_expected = math.atan2(_cap_int16(raw_x), _cap_int16(raw_y)) / math.pi
    omega_expected = _cap_int16(raw_omega) / 32768.0

    assert abs(theta_norm - theta_expected) < 0.05
    assert abs(omega_norm - omega_expected) < 0.02
    if raw_omega != 0:
        assert (omega_norm > 0) == (raw_omega > 0)


@pytest.mark.unit
def test_imu_driver_uses_driver_for_reads(monkeypatch):
    """Smoke test: IMU_Driver reads from the device driver and returns sane norms."""
    import hardware.imu_driver as imu_mod

    monkeypatch.setattr(
        imu_mod, "LSM6DS3TRDriver",
        lambda *a, **k: _FakeDevOnce(sample_bytes=_pack3(0, 0, 0)),
        raising=True,
    )

    imu = imu_mod.IMU_Driver(iir_params={}, controller_params={})
    try:
        theta_norm, omega_norm = imu.read_normalized()
    finally:
        imu.close()

    assert -1.0 <= theta_norm <= 1.0
    assert -1.0 <= omega_norm <= 1.0


@pytest.mark.unit
def test_imu_driver_reads_sequence(monkeypatch):
    """With a step in AX/AY, theta_norm should respond (after LP settling)."""
    import hardware.imu_driver as imu_mod

    seq = [(0, ACC_FS, 0)] * 2 + [(ACC_FS, 0, 0)] * 4
    monkeypatch.setattr(
        imu_mod, "LSM6DS3TRDriver",
        lambda *a, **k: _FakeDevSeq(seq=seq),
        raising=True,
    )

    imu = imu_mod.IMU_Driver(
        iir_params={"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 4.0, "CUTOFF_FREQ_HZ": 5.0},
        controller_params={"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": math.pi, "GYRO_FULL_SCALE_RADS_S": 4.363},
    )
    try:
        vals = [imu.read_normalized()[0] for _ in seq]
    finally:
        imu.close()

    assert vals[-1] > vals[1]

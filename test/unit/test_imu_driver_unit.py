# test/test_imu_driver_unit.py
import math
import pytest

INT16_MAX = 32767
INT16_MIN = -32768
ACC_FS = 16384  # ≈ 1 g in raw counts


def _cap_int16(v: int) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(v)))


def _pack3(x, y, g) -> bytes:
    """Little-endian int16 packing for (x_raw, y_raw, omega_raw)."""
    def p(v):
        return int(_cap_int16(v)).to_bytes(2, "little", signed=True)
    return p(x) + p(y) + p(g)


class NoopSPIBus:
    """Returns a constant zero sample."""
    def __init__(self, *a, **k):
        self._buf = _pack3(0, 0, 0)
        self.closed = False
    def imu_read(self, **_):
        return self._buf
    def close(self):
        self.closed = True


class FakeSPIBus:
    """Plays back a fixed sequence of (x,y,omega) samples."""
    def __init__(self, seq):
        self._seq = list(seq)
        self._i = 0
        self.closed = False
    def imu_read(self, **_):
        if self._i < len(self._seq):
            x, y, g = self._seq[self._i]
            self._i += 1
        else:
            x, y, g = self._seq[-1] if self._seq else (0, 0, 0)
        return _pack3(x, y, g)
    def close(self):
        self.closed = True


class _FakeSPIOnce:
    """Always returns the same one sample (x,y,omega)."""
    def __init__(self, sample_bytes: bytes):
        self._buf = sample_bytes
    def imu_read(self, **_):
        return self._buf
    def close(self):
        pass


@pytest.mark.unit
@pytest.mark.parametrize(
    "raw_y, raw_x, raw_omega",
    [
        (16384, 0, 0),              # Top of circle y=max, x=0, Omega=0
        (16135, 2845, 0),           # ~10° CW,   Omega=0
        (0, 16384, 0),              # 90° CW,    Omega=0
        (11585, 11585, 1000),       # 45° CW,    positive omega (CW)
        (-11585, 11585, 1000),      # ~135° CW
        (0, 16384, -1000),          # 90° CW,    negative omega (CCW)
        (0, -16384, 0),             # 90° CCW,   Omega=0
        (-8191, 14188, -1000),      # ~120° CW,  negative omega
        (-8191, -14188, 1000),      # ~120° CCW, positive omega
        (-11585, 11585, 0),         # ~135° CW
        (-16384, 0, 0),             # Bottom (180°)
        (8192, -14188, 500),        # ~-60°,     positive omega
    ],
)
def test_read_normalized_from_inputs(monkeypatch, raw_y, raw_x, raw_omega):
    """
    IMU_Driver calls SPIBus.imu_read() -> 6 bytes:
    [raw_x L, raw_x H, raw_y L, raw_y H, raw_omega L, raw_omega H]
    Feed fixed raw samples and check theta/omega normalization.
    """
    import hardware.imu_driver as imu_mod

    # Build the single sample buffer (note order: x, y, omega)
    buf = _pack3(raw_x, raw_y, raw_omega)

    # Patch the SPIBus symbol inside imu_driver to our constant fake
    monkeypatch.setattr(imu_mod, "SPIBus", lambda *_a, **_k: _FakeSPIOnce(buf), raising=False)

    # High cutoffs so LP filters have minimal lag; read a few frames to settle
    iir_params = {
        "SAMPLE_RATE_HZ": 50.0,
        "ACCEL_CUTOFF_HZ": 50.0,   # accel LP ~ passthrough
        "CUTOFF_FREQ_HZ": 50.0,    # omega LP  ~ passthrough
    }
    controller_params = {
        "ACCEL_RAW_FS": ACC_FS,
        "THETA_RANGE_RAD": math.pi,
        "GYRO_FULL_SCALE_RADS_S": 4.363,
    }

    imu = imu_mod.IMU_Driver(iir_params, controller_params)

    # Let 1st-order filters converge toward steady input
    for _ in range(3):
        theta_norm, omega_norm = imu.read_normalized()

    # Expected from the same convention the driver uses: theta = atan2(x, y)
    theta_expected = math.atan2(_cap_int16(raw_x), _cap_int16(raw_y)) / math.pi
    omega_expected = _cap_int16(raw_omega) / 32768.0

    # Reasonable tolerances for EWMA filters with high cutoff
    assert abs(theta_norm - theta_expected) < 0.05
    assert abs(omega_norm - omega_expected) < 0.02

    # Sign sanity for omega
    if raw_omega != 0:
        assert (omega_norm > 0) == (raw_omega > 0)


def test_imu_driver_uses_spi_for_reads(monkeypatch):
    from hardware import imu_driver as imu_mod

    # Patch SPIBus symbol only (SPI-based design; no IMUDev anymore)
    monkeypatch.setattr(imu_mod, "SPIBus", lambda *_a, **_k: NoopSPIBus(), raising=False)

    imu = imu_mod.IMU_Driver(iir_params={}, controller_params={})
    theta_norm, omega_norm = imu.read_normalized()

    assert -1.0 <= theta_norm <= 1.0
    assert -1.0 <= omega_norm <= 1.0


def test_imu_driver_reads_sequence(monkeypatch):
    from hardware import imu_driver as imu_mod

    seq = [(0, ACC_FS, 0)] * 2 + [(ACC_FS, 0, 0)] * 4
    monkeypatch.setattr(imu_mod, "SPIBus", lambda *_a, **_k: FakeSPIBus(seq), raising=False)

    imu = imu_mod.IMU_Driver(
        iir_params={"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 4.0, "CUTOFF_FREQ_HZ": 5.0},
        controller_params={"ACCEL_RAW_FS": ACC_FS, "THETA_RANGE_RAD": math.pi, "GYRO_FULL_SCALE_RADS_S": 4.363},
    )

    vals = [imu.read_normalized()[0] for _ in seq]
    # should rise after the step
    assert vals[-1] > vals[1]

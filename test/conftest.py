# test/conftest.py
import pytest

INT16_MAX = 32767
INT16_MIN = -32768


def _cap_int16(v: int) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(v)))


def _pack3(x: int, y: int, g: int) -> bytes:
    """Little-endian int16 packing for (x_raw, y_raw, omega_raw)."""
    def p(val: int) -> bytes:
        return int(_cap_int16(val)).to_bytes(2, "little", signed=True)
    return p(x) + p(y) + p(g)


class FakeDevSeq:
    """
    Device-driver stub compatible with LSM6DS3TRDriver's public API.
    Plays back a sequence of (x, y, omega) raw int16 triples.
    """
    def __init__(self, *, seq=None):
        triples = seq or [(0, 0, 0)]
        self._seq = [_pack3(x, y, g) for (x, y, g) in triples]
        self._i = 0
        self.closed = False

    def read_ax_ay_gz_bytes(self, *_, **__) -> bytes:
        if self._i < len(self._seq):
            buf = self._seq[self._i]
            self._i += 1
        else:
            buf = self._seq[-1]
        return buf

    def close(self):
        self.closed = True


@pytest.fixture
def make_imu(monkeypatch):
    """
    Build an IMU_Driver wired to a FakeDevSeq (new architecture).
    Usage:
        imu, fake = make_imu(samples=[(x,y,ω), ...], iir_params=..., ctrl_params=...)
    """
    def _builder(samples, iir_params=None, ctrl_params=None):
        import hardware.imu_driver as imu_mod

        dev_ref = {}

        def _driver_factory(*_a, **_k):
            dev = FakeDevSeq(seq=samples)
            dev_ref["dev"] = dev
            return dev

        # Patch the device driver class used by imu_driver
        monkeypatch.setattr(imu_mod, "LSM6DS3TRDriver", _driver_factory, raising=True)

        imu = imu_mod.IMU_Driver(iir_params or {}, ctrl_params or {})
        return imu, dev_ref["dev"]

    return _builder

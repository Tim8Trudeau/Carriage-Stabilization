# test/conftest.py
import pytest

def _pack3(x, y, g):
    def p(v):
        return int(v).to_bytes(2, "little", signed=True)
    return p(x) + p(y) + p(g)

class FakeSPI:
    """
    Minimal SPI replacement with scripted outputs.
    Provide an iterable of (x_raw, y_raw, omega_raw).
    """
    def __init__(self, samples):
        self._seq = list(samples)
        self._i = 0
        self.closed = False

    def load_samples(self, samples):
        self._seq = list(samples)
        self._i = 0

    def imu_read(self, **_):
        if not self._seq:
            x, y, g = 0, 0, 0
        elif self._i < len(self._seq):
            x, y, g = self._seq[self._i]
            self._i += 1
        else:
            x, y, g = self._seq[-1]
        return _pack3(x, y, g)

    def close(self):
        self.closed = True


@pytest.fixture
def make_imu(monkeypatch):
    """
    Factory that builds an IMU_Driver instance with:
      - SPI replaced by FakeSPI (scripted samples)
      - CRITICAL: patch the SPIBus SYMBOL inside imu_driver before construction
    Returns a builder: make_imu(samples, iir_params=None, ctrl_params=None)
    """
    def _builder(samples, iir_params=None, ctrl_params=None):
        import hardware.imu_driver as imu_mod

        # Patch the imported SPIBus symbol inside imu_driver
        monkeypatch.setattr(
            imu_mod,
            "SPIBus",
            lambda *_a, **_k: FakeSPI(samples),
            raising=False,
        )

        imu = imu_mod.IMU_Driver(iir_params or {}, ctrl_params or {})
        fake = imu.spi  # expose the FakeSPI instance
        return imu, fake

    return _builder

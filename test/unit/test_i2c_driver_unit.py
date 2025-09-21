# test/unit/test_i2c_driver_unit.py
import types
import sys
import pytest


def _install_fake_pigpio(monkeypatch):
    """Install a minimal fake 'pigpio' with pi() exposing i2c_* methods."""
    class FakePi:
        def __init__(self):
            self.connected = True
            self._open = {}
            self._next_h = 1
        def i2c_open(self, bus, addr, flags=0):
            h = self._next_h
            self._next_h += 1
            self._open[h] = (bus, addr, flags)
            return h
        def i2c_close(self, h):
            self._open.pop(h, None)
        def i2c_read_byte_data(self, h, reg):
            return 0
        def i2c_read_i2c_block_data(self, h, reg, count):
            return bytes([0] * count)
        def i2c_write_byte_data(self, h, reg, val):
            return 0
        def stop(self):
            self._open.clear()

    fake_mod = types.ModuleType("pigpio")
    fake_mod.pi = FakePi
    monkeypatch.setitem(sys.modules, "pigpio", fake_mod)
    return fake_mod


def _install_stub_mock_i2c(monkeypatch):
    """
    Ensure imports of 'test.mocks.mock_i2c' or 'mock_i2c' succeed with a tiny stub
    that implements MockI2CBus.imu_read() -> 6 bytes.
    """
    class StubMockI2CBus:
        def __init__(self, *a, **k): pass
        def imu_read(self, **_):
            # AX=+1, AY=-2, GZ=+3 (little-endian int16)
            return (1).to_bytes(2, "little", signed=True) + \
                   (-2).to_bytes(2, "little", signed=True) + \
                   (3).to_bytes(2, "little", signed=True)
        def close(self): pass

    for name in ("test.mocks.mock_i2c", "mock_i2c"):
        mod = types.ModuleType(name)
        mod.MockI2CBus = StubMockI2CBus
        monkeypatch.setitem(sys.modules, name, mod)


@pytest.mark.unit
def test_get_i2c_host_returns_pigpio_when_available(monkeypatch):
    _install_stub_mock_i2c(monkeypatch)   # in case code falls back
    _install_fake_pigpio(monkeypatch)

    import hardware.i2c_driver as i2c_drv
    host = i2c_drv.get_i2c_host({})

    # pigpio path should give us a FakePi instance
    assert hasattr(host, "i2c_open") and hasattr(host, "i2c_close")
    assert hasattr(host, "i2c_read_byte_data") and hasattr(host, "i2c_read_i2c_block_data")
    assert hasattr(host, "i2c_write_byte_data") and hasattr(host, "stop")


@pytest.mark.unit
def test_get_i2c_host_returns_mock_when_forced(monkeypatch):
    _install_stub_mock_i2c(monkeypatch)
    _install_fake_pigpio(monkeypatch)  # present, but we force mock

    monkeypatch.setenv("CS_HW", "mock")
    import importlib
    import hardware.i2c_driver as i2c_drv
    importlib.reload(i2c_drv)  # ensure env is read fresh

    host = i2c_drv.get_i2c_host({})
    # Should still be pi-like
    assert hasattr(host, "i2c_open") and hasattr(host, "i2c_close")
    assert hasattr(host, "i2c_read_byte_data") and hasattr(host, "i2c_read_i2c_block_data")
    assert hasattr(host, "i2c_write_byte_data") and hasattr(host, "stop")

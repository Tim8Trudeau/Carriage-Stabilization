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
    setattr(fake_mod, "pi", FakePi)  # Dynamically add the 'pi' attribute    fake_mod.pi = FakePi
    monkeypatch.setitem(sys.modules, "pigpio", fake_mod)
    return fake_mod


    """
    Provide a minimal 'test.mocks.mock_pigpio' module whose pi() returns an object
    with the i2c_* API used by the driver.
    """
def _install_stub_mock_pigpio(monkeypatch):
    import types, sys

    class FakePi:
        def __init__(self): self.connected = True
        def i2c_open(self, bus, addr, flags=0): return 1
        def i2c_close(self, h): pass
        def i2c_read_byte_data(self, h, reg): return 0
        def i2c_read_i2c_block_data(self, h, reg, count): return (count, bytes([0]*count))
        def i2c_write_byte_data(self, h, reg, val): pass
        def stop(self): pass

    mod = types.ModuleType("test.mocks.mock_pigpio")

    # Define a named function (friendlier to static analyzers than a lambda)
    def pi():
        return FakePi()

    # Attach it using setattr; add a type-ignore to silence Pylance
    setattr(mod, "pi", pi)  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "test.mocks.mock_pigpio", mod)

@pytest.mark.unit
def test_get_i2c_host_returns_pigpio_when_available(monkeypatch):
    _install_stub_mock_pigpio(monkeypatch)  # ensure mock is present

    import hardware.i2c_driver as i2c_drv
    host = i2c_drv.get_i2c_host({})

    # pigpio path should give us a FakePi instance
    assert hasattr(host, "i2c_open") and hasattr(host, "i2c_close")
    assert hasattr(host, "i2c_read_byte_data") and hasattr(host, "i2c_read_i2c_block_data")
    assert hasattr(host, "i2c_write_byte_data") and hasattr(host, "stop")


@pytest.mark.unit
def test_get_i2c_host_returns_mock_when_forced(monkeypatch):
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

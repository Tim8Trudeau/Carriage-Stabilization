# tests/unit/test_i2c_driver_unit.py
import importlib
import sys
import types
import pytest


def _install_fake_pigpio(monkeypatch):
    """Install a minimal fake 'pigpio' module exposing pigpio.pi()."""

    class FakePi:
        def __init__(self):
            self.connected = True
            self._origin = "pigpio"

        def i2c_open(self, bus, addr, flags=0): return 1
        def i2c_close(self, h): return None
        def i2c_read_byte_data(self, h, reg): return 0
        def i2c_read_i2c_block_data(self, h, reg, count): return (count, bytes([0] * count))
        def i2c_write_byte_data(self, h, reg, val): return None
        def stop(self): return None

    fake_mod = types.ModuleType("pigpio")
    fake_mod.pi = lambda: FakePi()  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "pigpio", fake_mod)


def _install_fake_mock_pigpio(monkeypatch):
    """Install fake mock_pigpio under BOTH import paths to match i2c_driver fallbacks."""

    class FakePi:
        def __init__(self):
            self.connected = True
            self._origin = "mock_pigpio"

        def i2c_open(self, bus, addr, flags=0): return 1
        def i2c_close(self, h): return None
        def i2c_read_byte_data(self, h, reg): return 0
        def i2c_read_i2c_block_data(self, h, reg, count): return (count, bytes([0] * count))
        def i2c_write_byte_data(self, h, reg, val): return None
        def stop(self): return None

    def _pi():
        return FakePi()

    # Create package modules for both: test.mocks.mock_pigpio and tests.mocks.mock_pigpio
    pkg_test = types.ModuleType("test")
    pkg_test_mocks = types.ModuleType("test.mocks")
    mod_test_mock = types.ModuleType("test.mocks.mock_pigpio")
    mod_test_mock.pi = _pi  # type: ignore[attr-defined]

    pkg_tests = types.ModuleType("tests")
    pkg_tests_mocks = types.ModuleType("tests.mocks")
    mod_tests_mock = types.ModuleType("tests.mocks.mock_pigpio")
    mod_tests_mock.pi = _pi  # type: ignore[attr-defined]

    monkeypatch.setitem(sys.modules, "test", pkg_test)
    monkeypatch.setitem(sys.modules, "test.mocks", pkg_test_mocks)
    monkeypatch.setitem(sys.modules, "test.mocks.mock_pigpio", mod_test_mock)

    monkeypatch.setitem(sys.modules, "tests", pkg_tests)
    monkeypatch.setitem(sys.modules, "tests.mocks", pkg_tests_mocks)
    monkeypatch.setitem(sys.modules, "tests.mocks.mock_pigpio", mod_tests_mock)


def _reload_i2c_driver():
    # Remove any existing module entry so the next import picks up current sys.modules
    # (monkeypatched fake modules) and produces a fresh module object for tests.
    sys.modules.pop("hardware.i2c_driver", None)
    import hardware.i2c_driver as i2c_drv
    return i2c_drv


@pytest.mark.unit
def test_get_i2c_host_sim_mode_true_returns_mock(monkeypatch):
    _install_fake_pigpio(monkeypatch)
    _install_fake_mock_pigpio(monkeypatch)

    i2c_drv = _reload_i2c_driver()
    host = i2c_drv.get_i2c_host({"SIM_MODE": True})

    assert hasattr(host, "i2c_open") and hasattr(host, "i2c_close")
    assert hasattr(host, "i2c_read_byte_data") and hasattr(host, "i2c_read_i2c_block_data")
    assert hasattr(host, "i2c_write_byte_data") and hasattr(host, "stop")
    assert getattr(host, "_origin", "") == "mock_pigpio"


@pytest.mark.unit
def test_get_i2c_host_sim_mode_false_prefers_pigpio(monkeypatch):
    _install_fake_pigpio(monkeypatch)
    _install_fake_mock_pigpio(monkeypatch)

    i2c_drv = _reload_i2c_driver()
    host = i2c_drv.get_i2c_host({"SIM_MODE": False})

    assert hasattr(host, "i2c_open") and hasattr(host, "i2c_close")
    assert getattr(host, "_origin", "") == "pigpio"

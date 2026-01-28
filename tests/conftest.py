import sys
import types
import pytest


class FakeI2CHost:
    """
    Minimal I2C host that emulates the subset of pigpio-style APIs used by the project.

    Provides:
      - i2c_open(bus, addr, flags) -> handle int
      - i2c_close(handle)
      - i2c_read_byte_data(handle, reg) -> int 0..255
      - i2c_write_byte_data(handle, reg, value)
      - stop()

    A simple register map is stored per (bus, addr).
    """

    class Error(RuntimeError):
        """Used as a stand-in for pigpio.error in tests."""

    def __init__(self):
        self._next_h = 1
        self._handles = {}          # handle -> (bus, addr)
        self.regs = {}              # (bus, addr) -> {reg:int}
        self.writes = []            # list of (handle, reg, val)
        self.reads = []             # list of (handle, reg)
        self.closed = []
        self.stopped = False

    def _regmap(self, bus, addr):
        return self.regs.setdefault((int(bus), int(addr)), {})

    def i2c_open(self, bus, addr, flags=0):
        h = self._next_h
        self._next_h += 1
        self._handles[h] = (int(bus), int(addr))
        return h

    def i2c_close(self, h):
        self.closed.append(h)
        self._handles.pop(h, None)

    def i2c_write_byte_data(self, h, reg, value):
        if h not in self._handles:
            raise self.Error("bad handle")
        bus, addr = self._handles[h]
        self.writes.append((h, reg & 0xFF, value & 0xFF))
        self._regmap(bus, addr)[reg & 0xFF] = value & 0xFF

    def i2c_read_byte_data(self, h, reg):
        if h not in self._handles:
            raise self.Error("bad handle")
        self.reads.append((h, reg & 0xFF))
        bus, addr = self._handles[h]
        return int(self._regmap(bus, addr).get(reg & 0xFF, 0x00))

    def stop(self):
        self.stopped = True


@pytest.fixture
def fake_i2c_host():
    return FakeI2CHost()


@pytest.fixture
def install_fake_i2c_host(monkeypatch, fake_i2c_host):
    """
    Patch the MPU6050 driver to use FakeI2CHost.

    The driver previously did:
        from hardware.i2c_driver import get_i2c_host
    and newer versions do:
        from hardware import i2c_driver
        i2c_driver.get_i2c_host(...)

    This fixture supports both by patching the correct attribute depending on what's present.
    """
    import importlib

    mod = importlib.import_module("hardware.mpu6050_i2c_driver")

    # Old style: get_i2c_host imported into module namespace
    if hasattr(mod, "get_i2c_host"):
        monkeypatch.setattr(mod, "get_i2c_host", lambda params=None: fake_i2c_host, raising=True)
    else:
        # New style: module imports hardware.i2c_driver as i2c_driver
        # Patch the function on that imported module object.
        monkeypatch.setattr(mod.i2c_driver, "get_i2c_host", lambda params=None: fake_i2c_host, raising=True)

    return fake_i2c_host


@pytest.fixture
def install_fake_pigpio_module(monkeypatch):
    """
    Some modules optionally import pigpio. Provide a lightweight module in sys.modules.
    NOTE: pwm_driver prefers its own tests.mocks.mock_pigpio when PIGPIO_FORCE_MOCK=1.
    """
    m = types.SimpleNamespace()

    class _Err(Exception):
        pass

    m.error = _Err
    monkeypatch.setitem(sys.modules, "pigpio", m)
    return m

# test/unit/test_spi_driver_unit.py
import types
import sys
import pytest


def _install_fake_pigpio(monkeypatch):
    """Install a minimal fake 'pigpio' with pi() exposing spi_open/close/xfer."""
    class FakePi:
        def __init__(self):
            self.connected = True
            self._open = False
        def spi_open(self, ch, baud, flags):
            self._open = True
            return 1
        def spi_close(self, h):
            self._open = False
        def spi_xfer(self, h, tx: bytes):
            # Return a dummy/status byte + zeros of the requested size
            trailing = max(0, len(tx) - 1)
            return len(tx), bytes([0x00]) + (b"\x00" * trailing)
        def stop(self):
            self._open = False

    fake_mod = types.ModuleType("pigpio")
    fake_mod.pi = FakePi
    monkeypatch.setitem(sys.modules, "pigpio", fake_mod)
    return fake_mod


def _install_stub_mock_spi(monkeypatch):
    """
    Ensure imports of 'test.mocks.mock_spi' or 'mock_spi' succeed with a tiny stub
    that implements MockSPIBus.imu_read() -> 6 bytes.
    """
    class StubMockSPIBus:
        def __init__(self, *a, **k): pass
        def imu_read(self, **_):
            # AX=+1, AY=-2, GZ=+3 (little-endian int16)
            return (1).to_bytes(2, "little", signed=True) + \
                   (-2).to_bytes(2, "little", signed=True) + \
                   (3).to_bytes(2, "little", signed=True)
        def close(self): pass

    for name in ("test.mocks.mock_spi", "mock_spi"):
        mod = types.ModuleType(name)
        mod.MockSPIBus = StubMockSPIBus
        monkeypatch.setitem(sys.modules, name, mod)


@pytest.mark.unit
def test_get_spi_host_returns_pigpio_when_available(monkeypatch):
    _install_stub_mock_spi(monkeypatch)   # in case code falls back
    _install_fake_pigpio(monkeypatch)

    import hardware.spi_driver as spi_drv
    host = spi_drv.get_spi_host({})

    # pigpio path should give us a FakePi instance
    assert hasattr(host, "spi_open") and hasattr(host, "spi_close") and hasattr(host, "spi_xfer")
    assert hasattr(host, "stop")


@pytest.mark.unit
def test_get_spi_host_returns_mock_when_forced(monkeypatch):
    _install_stub_mock_spi(monkeypatch)
    _install_fake_pigpio(monkeypatch)  # present, but we force mock

    monkeypatch.setenv("CS_HW", "mock")
    import importlib
    import hardware.spi_driver as spi_drv
    importlib.reload(spi_drv)  # ensure env is read fresh

    host = spi_drv.get_spi_host({})
    # Should still be pi-like
    assert hasattr(host, "spi_open") and hasattr(host, "spi_close") and hasattr(host, "spi_xfer")
    assert hasattr(host, "stop")

# tests/unit/test_pwm_driver_unit.py
# These tests run during unit testing under Windows while real hardware is missing.

import types
import importlib
import pytest
import sys

# Mirror constants from hardware.pwm_driver
MIN_PWM = 57_000
MAX_PWM = 1_000_000

def expected_pwm(value: float) -> int:
    v = abs(float(value))
    if v < 1e-6:
        return 0
    if v > 1.0:
        v = 1.0
    return int(MIN_PWM + v * (MAX_PWM - MIN_PWM))


@pytest.fixture
def patched_pwm_driver(monkeypatch):
    """Reload hardware.pwm_driver with tests.mocks.mock_pigpio wired in as pigpio."""
    from tests.mocks import mock_pigpio as mp
    monkeypatch.setitem(sys.modules, "pigpio", mp)   # isolate per test
    import hardware.pwm_driver as pwm_mod
    importlib.reload(pwm_mod)
    return pwm_mod


@pytest.mark.unit
def test_daemon_not_connected(monkeypatch):
    from tests.mocks import mock_pigpio as mp

    class UnconnectedPi(getattr(mp, "_MockPi")):
        def __init__(self):
            super().__init__()
            self.connected = False

    fake_mod = types.SimpleNamespace(
        pi=lambda: UnconnectedPi(),
        OUTPUT=getattr(mp, "OUTPUT", "OUTPUT"),
    )

    monkeypatch.setitem(sys.modules, "pigpio", fake_mod)

    import hardware.pwm_driver as pwm_mod
    importlib.reload(pwm_mod)

    with pytest.raises(RuntimeError, match="Could not connect to pigpio daemon"):
        pwm_mod.DualPWMController()


@pytest.mark.unit
def test_positive_set_speed(patched_pwm_driver):
    DualPWMController = patched_pwm_driver.DualPWMController
    pwm = DualPWMController()
    pi = pwm.pi  # mock pigpio instance with pwm_states

    pwm.set_speed(0.5)
    assert pi.pwm_states[13]["frequency"] == pwm.freq
    assert pi.pwm_states[13]["dutycycle"] == expected_pwm(0.5)
    assert pi.pwm_states[12]["frequency"] == pwm.freq
    assert pi.pwm_states[12]["dutycycle"] == 0

    pwm.set_speed(-0.5)
    assert pi.pwm_states[13]["dutycycle"] == 0
    assert pi.pwm_states[12]["dutycycle"] == expected_pwm(0.5)

    pwm.set_speed(0.0)
    assert pi.pwm_states[13]["dutycycle"] == 0
    assert pi.pwm_states[12]["dutycycle"] == 0

    pwm.set_speed(-1.0)
    assert pi.pwm_states[13]["dutycycle"] == 0
    assert pi.pwm_states[12]["dutycycle"] == expected_pwm(1.0)

    pwm.stop()
    assert pi.pwm_states[13]["dutycycle"] == 0
    assert pi.pwm_states[12]["dutycycle"] == 0
    assert getattr(pi, "_stopped", False) is True


@pytest.mark.unit
def test_clamping(patched_pwm_driver):
    DualPWMController = patched_pwm_driver.DualPWMController
    pwm = DualPWMController()
    pi = pwm.pi

    pwm.set_speed(99999)
    assert pi.pwm_states[13]["dutycycle"] == expected_pwm(1.0)
    assert pi.pwm_states[12]["dutycycle"] == 0

    pwm.set_speed(-99999)
    assert pi.pwm_states[13]["dutycycle"] == 0
    assert pi.pwm_states[12]["dutycycle"] == expected_pwm(1.0)

    pwm.set_speed(0.01)
    assert pi.pwm_states[13]["dutycycle"] == expected_pwm(0.01)
    assert pi.pwm_states[12]["dutycycle"] == 0

    pwm.set_speed(-0.01)
    assert pi.pwm_states[13]["dutycycle"] == 0
    assert pi.pwm_states[12]["dutycycle"] == expected_pwm(0.01)

    pwm.stop()
    assert pi.pwm_states[13]["dutycycle"] == 0
    assert pi.pwm_states[12]["dutycycle"] == 0


@pytest.mark.unit
def test_zero_input(patched_pwm_driver):
    DualPWMController = patched_pwm_driver.DualPWMController
    pwm = DualPWMController()
    pi = pwm.pi

    pwm.set_speed(0)
    assert pi.pwm_states[13]["dutycycle"] == 0
    assert pi.pwm_states[12]["dutycycle"] == 0
    pwm.stop()

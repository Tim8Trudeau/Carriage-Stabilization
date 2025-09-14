# tests/unit/test_pwm_driver_unit.py
# These tests during unit testing under windows while real hardware is missing.

import types
import importlib
import pytest
import sys

@pytest.fixture
def patched_pwm_driver(monkeypatch):
    """
    For each test, ensure 'pigpio' resolves to the connected mock module,
    then reload hardware.pwm_driver so it binds to that mock.
    """
    from test.mocks import mock_pigpio as mp
    monkeypatch.setitem(sys.modules, "pigpio", mp)   # isolate per test
    import hardware.pwm_driver as pwm_mod
    importlib.reload(pwm_mod)
    return pwm_mod

@pytest.mark.unit
def test_daemon_not_connected(monkeypatch):
    # Base mock pigpio
    from test.mocks import mock_pigpio as mp

    # Force a pi() whose 'connected' is False (boolean attribute)
    class UnconnectedPi(getattr(mp, "_MockPi")):
        def __init__(self):
            super().__init__()
            self.connected = False

    # Minimal pigpio facade
    fake_mod = types.SimpleNamespace(
        pi=lambda: UnconnectedPi(),
        OUTPUT=getattr(mp, "OUTPUT", "OUTPUT"),
    )

    # Make sure *imports* of 'pigpio' get our fake (overrides any earlier mp binding)
    monkeypatch.setitem(sys.modules, "pigpio", fake_mod)

    # Reload the driver to bind to our fake pigpio
    import hardware.pwm_driver as pwm_mod
    importlib.reload(pwm_mod)

    with pytest.raises(RuntimeError, match="Could not connect to pigpio daemon"):
        pwm_mod.DualPWMController()

@pytest.mark.unit
def test_positive_set_speed(patched_pwm_driver):
    DualPWMController = patched_pwm_driver.DualPWMController
    pwm = DualPWMController()
    pi = pwm.pi  # mock pigpio instance with pwm_states

    # +50% forward: PWM0=50%, PWM1=0%
    pwm.set_speed(0.5)
    assert pi.pwm_states[12]["frequency"] == pwm.freq
    assert pi.pwm_states[12]["dutycycle"] == 500_000
    assert pi.pwm_states[13]["frequency"] == pwm.freq
    assert pi.pwm_states[13]["dutycycle"] == 0

    # -50% reverse: PWM0=50%, PWM1=0%
    pwm.set_speed(-0.5)
    assert pi.pwm_states[12]["frequency"] == pwm.freq
    assert pi.pwm_states[12]["dutycycle"] == 0
    assert pi.pwm_states[13]["frequency"] == pwm.freq
    assert pi.pwm_states[13]["dutycycle"] == 500_000

    # stop: both 0%
    pwm.set_speed(0.0)
    assert pi.pwm_states[12]["dutycycle"] == 0
    assert pi.pwm_states[13]["dutycycle"] == 0

    # -100% reverse: PWM0=0%, PWM1=100%
    pwm.set_speed(-1.0)
    assert pi.pwm_states[12]["dutycycle"] == 0
    assert pi.pwm_states[13]["dutycycle"] == 1_000_000

    # stop() also zeros and stops pigpio
    pwm.stop()
    assert pi.pwm_states[12]["dutycycle"] == 0
    assert pi.pwm_states[13]["dutycycle"] == 0
    assert getattr(pi, "_stopped", False) is True


@pytest.mark.unit
def test_clamping(patched_pwm_driver):
    DualPWMController = patched_pwm_driver.DualPWMController
    pwm = DualPWMController()
    pi = pwm.pi  # mock pigpio instance with pwm_states

    # absurdly large positive clamps to 100%
    pwm.set_speed(99999)
    assert pi.pwm_states[12]["dutycycle"] == 1_000_000
    assert pi.pwm_states[13]["dutycycle"] == 0

    # absurdly large negative clamps to -100%
    pwm.set_speed(-99999)
    assert pi.pwm_states[12]["dutycycle"] == 0
    assert pi.pwm_states[13]["dutycycle"] == 1_000_000

    # tiny positive maps to small duty on channel 0
    pwm.set_speed(0.01)
    assert 0 < pi.pwm_states[12]["dutycycle"] <= 10_000
    assert pi.pwm_states[13]["dutycycle"] == 0

    # Test clamping of small negative speed
    pwm.set_speed(-0.01)
    assert 0 < pwm.pi.pwm_states[13]["dutycycle"] <= 10_000

    #Test Stop
    pwm.stop()
    assert pwm.pi.pwm_states[12]["dutycycle"] == 0
    assert pwm.pi.pwm_states[13]["dutycycle"] == 0

@pytest.mark.unit
def test_zero_input(patched_pwm_driver):
    DualPWMController = patched_pwm_driver.DualPWMController
    pwm = DualPWMController()
    pi = pwm.pi  # mock pigpio instance with pwm_states

    pwm.set_speed(0)
    assert pwm.pi.pwm_states[12]["dutycycle"] == 0
    assert pwm.pi.pwm_states[13]["dutycycle"] == 0
    pwm.stop()

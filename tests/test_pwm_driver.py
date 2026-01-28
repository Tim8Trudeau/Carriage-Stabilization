import importlib
import os
import pytest


def _reload_pwm_driver(monkeypatch):
    # Force pwm_driver to use tests.mocks.mock_pigpio
    monkeypatch.setenv("PIGPIO_FORCE_MOCK", "1")
    import hardware.pwm_driver as pwm
    return importlib.reload(pwm)


def test_map_with_deadzone(monkeypatch):
    pwm = _reload_pwm_driver(monkeypatch)

    c = pwm.DualPWMController(frequency=250)
    assert c._map_with_deadzone(0.0) == 0
    assert c._map_with_deadzone(1e-9) == 0  # below threshold
    assert c._map_with_deadzone(0.1) >= pwm.MIN_PWM
    assert c._map_with_deadzone(1.0) == pwm.MAX_PWM
    assert c._map_with_deadzone(2.0) == pwm.MAX_PWM  # clamps magnitude
    # sign should not change magnitude mapping
    assert c._map_with_deadzone(-0.5) == c._map_with_deadzone(0.5)


def test_set_speed_positive(monkeypatch):
    pwm = _reload_pwm_driver(monkeypatch)
    c = pwm.DualPWMController(frequency=123)

    c.set_speed(0.5)

    # duty_cycle_0 should be set, duty_cycle_1 should be zero
    assert c.duty_cycle_0 > 0
    assert c.duty_cycle_1 == 0

    # Verify last two hardware_PWM calls target both pins at correct frequency
    calls = [x for x in c.pi.calls if x[0] == "hardware_PWM"]
    assert calls[-2][1:] == (c.gpio_pwm0, 123, c.duty_cycle_0)
    assert calls[-1][1:] == (c.gpio_pwm1, 123, 0)


def test_set_speed_negative(monkeypatch):
    pwm = _reload_pwm_driver(monkeypatch)
    c = pwm.DualPWMController(frequency=500)

    c.set_speed(-0.25)
    assert c.duty_cycle_0 == 0
    assert c.duty_cycle_1 > 0

    calls = [x for x in c.pi.calls if x[0] == "hardware_PWM"]
    assert calls[-2][1:] == (c.gpio_pwm0, 500, 0)
    assert calls[-1][1:] == (c.gpio_pwm1, 500, c.duty_cycle_1)


def test_set_speed_zero(monkeypatch):
    pwm = _reload_pwm_driver(monkeypatch)
    c = pwm.DualPWMController(frequency=250)

    c.set_speed(0.0)
    assert c.duty_cycle_0 == 0
    assert c.duty_cycle_1 == 0

    calls = [x for x in c.pi.calls if x[0] == "hardware_PWM"]
    assert calls[-2][1:] == (c.gpio_pwm0, 250, 0)
    assert calls[-1][1:] == (c.gpio_pwm1, 250, 0)


def test_stop_releases(monkeypatch):
    pwm = _reload_pwm_driver(monkeypatch)
    c = pwm.DualPWMController(frequency=250)

    c.set_speed(0.7)
    c.stop()

    assert c.duty_cycle_0 == 0
    assert c.duty_cycle_1 == 0
    assert ("stop",) in c.pi.calls

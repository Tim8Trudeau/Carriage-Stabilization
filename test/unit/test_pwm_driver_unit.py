# tests/unit/test_pwm_driver_unit.py
# These tests during unit testing under windows while real hardware is missing.

import pytest
from test.mocks.mock_pigpio import MockPigpio, MockPi
from hardware.pwm_driver import DualPWMController

@pytest.fixture
def patched_pwm_driver(monkeypatch):
    monkeypatch.setattr("hardware.pwm_driver.pigpio", MockPigpio)
    return DualPWMController()

@pytest.mark.unit
def test_positive_command(patched_pwm_driver):
    pwm = patched_pwm_driver
    pwm.command(16384)  # 50%
    pwm.command(0)      # Stop
    pwm.command(-32768) # 100% reverse
    pwm.stop()

    pi = pwm.pi
    assert pi.calls[4] == ('hardware_PWM', 18, 30, 500000)  # 50%
    assert pi.calls[9] == ('hardware_PWM', 19, 30, 1000000) # 100%

@pytest.mark.unit
def test_clamping(monkeypatch):
    monkeypatch.setattr("hardware.pwm_driver.pigpio", MockPigpio)
    pwm = DualPWMController()
    pwm.command(99999)
    assert pwm.pi.pwm_states[18]['dutycycle'] == 1_000_000
    pwm.command(-99999)
    assert pwm.pi.pwm_states[19]['dutycycle'] == 1_000_000
    pwm.command(1)
    assert 0 < pwm.pi.pwm_states[18]['dutycycle'] < 40000
    pwm.stop()

@pytest.mark.unit
def test_zero_input(monkeypatch):
    monkeypatch.setattr("hardware.pwm_driver.pigpio", MockPigpio)
    pwm = DualPWMController()
    pwm.command(0)
    assert pwm.pi.pwm_states[18]['dutycycle'] == 0
    assert pwm.pi.pwm_states[19]['dutycycle'] == 0
    pwm.stop()

@pytest.mark.unit
def test_daemon_not_connected(monkeypatch):
    class UnconnectedPi(MockPi):
        def connected(self):
            return False

    class FaultyPigpio(MockPigpio):
        @staticmethod
        def pi():
            return UnconnectedPi()

    monkeypatch.setattr("hardware.pwm_driver.pigpio", FaultyPigpio)
    with pytest.raises(RuntimeError, match="Could not connect to pigpio daemon"):
        DualPWMController()

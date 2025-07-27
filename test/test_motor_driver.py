import pytest
from hardware.pwm_driver import DualPWMController

max_int = 32_768

@pytest.fixture
def motor_driver():
    return DualPWMController()

def test_motor_driver_init(motor_driver: DualPWMController):
    assert motor_driver is not None
    assert motor_driver.duty_cycle_0 == 0
    assert motor_driver.duty_cycle_1 == 0

def test_set_speed_zero(motor_driver: DualPWMController):
    motor_driver.set_speed(0)
    assert motor_driver.duty_cycle_0 == 0
    assert motor_driver.duty_cycle_1 == 0

def test_set_speed_max_positive(motor_driver: DualPWMController):
    motor_driver.set_speed(max_int)
    assert motor_driver.duty_cycle_0 == 1_000_000
    assert motor_driver.duty_cycle_1 == 0

def test_set_speed_max_negative(motor_driver: DualPWMController):
    motor_driver.set_speed(-max_int)
    assert motor_driver.duty_cycle_0 == 0
    assert motor_driver.duty_cycle_1 == 1_000_000

def test_set_speed_half_positive(motor_driver: DualPWMController):
    motor_driver.set_speed(max_int/2)
    assert motor_driver.duty_cycle_0 == 500_000
    assert motor_driver.duty_cycle_1 == 0

def test_set_speed_half_negative(motor_driver: DualPWMController):
    motor_driver.set_speed(-max_int/2)
    assert motor_driver.duty_cycle_0 == 0
    assert motor_driver.duty_cycle_1 == 500_000

def test_set_speed_clamping_high(motor_driver: DualPWMController):
    motor_driver.set_speed(40_000)
    assert motor_driver.duty_cycle_0 == 1_000_000
    assert motor_driver.duty_cycle_1 == 0

def test_set_speed_clamping_low(motor_driver: DualPWMController):
    motor_driver.set_speed(-40_000)
    assert motor_driver.duty_cycle_0 == 0
    assert motor_driver.duty_cycle_1 == 1_000_000

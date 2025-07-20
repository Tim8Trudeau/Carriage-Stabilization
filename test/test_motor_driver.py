import pytest
from hardware.motor_driver import MotorDriver

@pytest.fixture
def motor_driver():
    return MotorDriver()

def test_motor_driver_init(motor_driver: MotorDriver):
    assert motor_driver is not None
    assert motor_driver.i2c is not None

def test_set_speed_zero(motor_driver: MotorDriver):
    value = motor_driver.set_speed(0.0)
    # Check the value captured by the mock I2C object
    assert motor_driver.i2c._last_val == 0

def test_set_speed_max_positive(motor_driver: MotorDriver):
    motor_driver.set_speed(1.0)
    assert motor_driver.i2c._last_val == 4095

def test_set_speed_max_negative(motor_driver: MotorDriver):
    motor_driver.set_speed(-1.0)
    assert motor_driver.i2c._last_val == -4096

def test_set_speed_half_positive(motor_driver: MotorDriver):
    motor_driver.set_speed(0.5)
    assert motor_driver.i2c._last_val == int(4095 * 0.5) # 2047

def test_set_speed_half_negative(motor_driver: MotorDriver):
    motor_driver.set_speed(-0.5)
    assert motor_driver.i2c._last_val == int(-4096 * 0.5) # -2048

def test_set_speed_clamping_high(motor_driver: MotorDriver):
    motor_driver.set_speed(1.5)
    assert motor_driver.i2c._last_val == 4095

def test_set_speed_clamping_low(motor_driver: MotorDriver):
    motor_driver.set_speed(-1.5)
    assert motor_driver.i2c._last_val == -4096

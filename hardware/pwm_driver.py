# hardware/pwm_driver.py
import logging
import os

motor_log = logging.getLogger("motor")

# Prefer real pigpio unless an override requests the mock
_USE_MOCK = os.getenv("PIGPIO_FORCE_MOCK", "0") == "1"

if not _USE_MOCK:
    try:
        import pigpio as _pigpio  # real library
    except Exception:
        _USE_MOCK = True

if _USE_MOCK:
    try:
        from test.mocks import mock_pigpio as _pigpio  # module-style mock
    except Exception as e:
        _pigpio = None
        motor_log.error("Neither pigpio nor mock_pigpio available: %s", e)


class DualPWMController:
    """
    DualPWMController generates two hardware PWM signals using pigpio (or a test mock).
    GPIO 12 (PWM0) and GPIO 13 (PWM1) are driven at `frequency` (default 200 Hz).
    Use set_speed(value: float) with value in [-1.0, +1.0]:
      value > 0 -> PWM0 active; value < 0 -> PWM1 active; 0 -> both off.
    Attributes:
        pi: pigpio.pi() or mock
        freq: int
        gpio_pwm0: int (default 12)
        gpio_pwm1: int (default 13)
    """

    def __init__(self, frequency: int = 200):
        """
        Initialize the PWM controller with given frequency (Hz).
        Both PWM channels start at 0% duty.
        Args:
            frequency (int): PWM signal frequency in Hz (default: 200).
        """
        if _pigpio is None:
            raise ImportError("pigpio not available and test.mocks.mock_pigpio not found")

        self.freq = frequency
        self.duty_cycle_0 = 0
        self.duty_cycle_1 = 0
        # self.gpio_pwm0 = 12
        # self.gpio_pwm1 = 13
        self.gpio_pwm0 = 13 # Reverse motor direction
        self.gpio_pwm1 = 12


        self.pi = _pigpio.pi()

        # Real pigpio: `connected` is a bool attribute (not a function)
        if hasattr(self.pi, "connected") and not self.pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon")

        # Some mocks may not implement set_mode; guard it
        if hasattr(self.pi, "set_mode") and hasattr(_pigpio, "OUTPUT"):
            self.pi.set_mode(self.gpio_pwm0, _pigpio.OUTPUT)
            self.pi.set_mode(self.gpio_pwm1, _pigpio.OUTPUT)

        # Initialize both channels at 0% duty
        if hasattr(self.pi, "hardware_PWM"):
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)

        motor_log.info("PWM_0 PIN %d, PWM_1 PIN %d @ %d Hz", self.gpio_pwm0, self.gpio_pwm1, self.freq)

    def set_speed(self, value: float):
        """
        Set the PWM output based on the signed float input.
        In pigpio, hardware_PWM(gpio, frequency, dutycycle) expects dutycycle 0..1_000_000.
        Args:
            value (float): range [-1, +1]
        """
        # Negative value means reverse motor direction
        if value < 0.0:
            duty_val = min(-value, 1.0)  # Clamp to max 1.0
            self.duty_cycle_0 = 0
            self.duty_cycle_1 = int(duty_val * 1_000_000)
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)

        elif value > 0.0:
            duty_val = min(value, 1.0)  # Clamp to max 1.0
            self.duty_cycle_0 = int(duty_val * 1_000_000)
            self.duty_cycle_1 = 0
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)

        else:  # value == 0.0 Stop motor
            self.duty_cycle_0 = 0
            self.duty_cycle_1 = 0
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, 0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, 0)

        motor_log.debug(
            "CW PWM_0 Dutycycle %d,  CCW PWM_1 Dutycycle %d",
            self.duty_cycle_0,
            self.duty_cycle_1,
        )

    def stop(self):
        """Stop both PWM outputs and release the pigpio interface."""
        self.duty_cycle_0 = 0
        self.duty_cycle_1 = 0
        self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
        self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)
        self.pi.stop()
        motor_log.debug("----Motor STOPPED----")

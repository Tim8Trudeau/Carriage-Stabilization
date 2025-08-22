# hardware/pwm_driver.py

import logging

motor_log = logging.getLogger("motor")

# Try real pigpio, else fall back to our test mock as a module
try:
    import pigpio as _pigpio
except Exception:
    try:
        # module-style mock that provides pi() and constants (e.g., OUTPUT)
        from test.mocks import mock_pigpio as _pigpio  # noqa: F401
    except Exception as e:
        _pigpio = None
        motor_log.error("Neither pigpio nor mock_pigpio available: %s", e)



class DualPWMController:
    """
    DualPWMController generates two hardware PWM signals using the pigpio library on a Raspberry Pi.
    GPIO 18 (PWM0) and GPIO 19 (PWM1) are used as outputs. Both are initialized with 0% duty cycle
    at a frequency of 50 Hz (default). The `command()` method accepts an integer to dynamically control
    which output is active and at what duty cycle, simulating directional control (e.g., for motor drivers).
    Positive input:
        - Activates PWM0 (GPIO 18) with duty cycle proportional to value [1..32768]
        - Sets PWM1 (GPIO 19) low (0% duty)
    Negative input:
        - Activates PWM1 (GPIO 19) with duty cycle proportional to |value| [1..32768]
        - Sets PWM0 (GPIO 18) low
    Zero input:
        - Sets both outputs to 0% duty (low)
    Attributes:
        pi (pigpio.pi): The pigpio interface instance.
        freq (int): PWM frequency in Hz (default: 50).
        gpio_pwm0 (int): GPIO pin for PWM0 (default: 18).
        gpio_pwm1 (int): GPIO pin for PWM1 (default: 19).
    """

    def __init__(self, frequency: int = 50):
        """
        Initialize the PWM controller with given frequency (Hz).
        Both PWM channels start at 0% duty.
        Args:
            frequency (int): PWM signal frequency in Hz (default: 50).
        """
        if _pigpio is None:
            raise ImportError("pigpio not available and test.mocks.mock_pigpio not found")

        self.freq = frequency
        self.duty_cycle_0 = 0
        self.duty_cycle_1 = 0
        self.gpio_pwm0 = 18
        self.gpio_pwm1 = 19

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
        Set the PWM output based on the signed integer input.
        Value must get rescaled for pigpio.
        In the pigpio library, hardware_PWM(gpio, frequency, dutycycle)
           expects a dutycycle between 0 and 1,000,000, or duty cycle in uSec
        Args:
            value (int): float in range [-1, +1]
                         - Positive value: PWM0 active on GPIO 18
                         - Negative value: PWM1 active on GPIO 19
                         - Zero: both outputs inactive (low)
        Returns:
            None

        """
        value = value  # FLC output from -1.0 to +1.0
        # Negative value means reverse motor direction
        if value < 0.0:
            # Convert negative input to a positive duty_val and clamp
            duty_val = min(-value, 1.0)  # Clamp to max 1.0
            self.duty_cycle_0 = 0
            # pigpio.PWM expects a duty cycle in the range [0, 1_000_000]usec
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
            "CW PWM_0 Dutycyle %d,  CCW PWM_1 Dutycyle %d",
            self.duty_cycle_0,
            self.duty_cycle_1,
        )

    def stop(self):
        """
        Stop both PWM outputs and release the pigpio interface.
        Returns:
            None
        """
        self.duty_cycle_0 = 0
        self.duty_cycle_1 = 0
        self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
        self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)
        self.pi.stop()
        motor_log.debug("----Motor STOPPED----")

try:
    import pigpio
except ImportError:
    pigpio = None  # Will be replaced by mock in tests/mocks

import logging

motor_log = logging.getLogger("motor")


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

    def __init__(self, frequency=50):
        """
        Initialize the PWM controller with given frequency (Hz).
        Both PWM channels start at 0% duty.
        Args:
            frequency (int): PWM signal frequency in Hz (default: 50).
        """
        if pigpio is None:
            from test.mocks.mock_pigpio import MockPigpio

            self.pi = MockPigpio.pi()
            self.freq = frequency
            self.gpio_pwm0 = 18
            self.gpio_pwm1 = 19
        else:
            self.pi = pigpio.pi()

            if not self.pi.connected():
                raise RuntimeError("Could not connect to pigpio daemon")
            self.freq = frequency
            self.gpio_pwm0 = 18
            self.gpio_pwm1 = 19
            self.pi.set_mode(self.gpio_pwm0, pigpio.OUTPUT)
            self.pi.set_mode(self.gpio_pwm1, pigpio.OUTPUT)
            self.duty_cycle_0 = 0
            self.duty_cycle_1 = 0
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)
        print("Leaving DualPWMController.__init__")
        motor_log.info("PWM_0 PIN %d,  PWM_1 PIN %d", self.gpio_pwm0, self.gpio_pwm1)

    def set_speed(self, value: float):
        """
        Set the PWM output based on the signed integer input.
        Value must get rescaled for pigpio.
        In the pigpio library, hardware_PWM(gpio, frequency, dutycycle)
           expects a dutycycle between 0 and 1,000,000, or duty cycle in uSec
        Args:
            value (int): float in range [-1, +1]
                         - Positive: PWM0 active on GPIO 18
                         - Negative: PWM1 active on GPIO 19
                         - Zero: both outputs inactive (low)
        Returns:
            None

        """
        value = int(value * 32768)  # FLC output from float to 16bit signed int for PWM
        if value == 0:
            self.duty_cycle_0 = 0
            self.duty_cycle_1 = 0
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, 0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, 0)
        elif value > 0:
            duty_val = min(value, 32768)
            # rescale again for pigpio.PWM
            self.duty_cycle_0 = int(duty_val * 1_000_000 / 32768)
            self.duty_cycle_1 = 0
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)
        else:
            # Convert negative input to a positive duty_val and clamp it to the range [1, 32768]
            duty_val = min(max(-value, 1), 32768)
            self.duty_cycle_0 = 0
            # rescale again for pigpio.PWM
            self.duty_cycle_1 = int(duty_val * 1_000_000 / 32768)
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)

        motor_log.debug(
            "PWM_0 Dutycyle %d,  PWM_1 Dutycyle %d",
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

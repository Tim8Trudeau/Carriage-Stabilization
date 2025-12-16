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
        from tests.mocks import mock_pigpio as _pigpio  # module-style mock
    except Exception as e:
        _pigpio = None
        motor_log.error("Neither pigpio nor mock_pigpio available: %s", e)


# Deadzone/offset so motors don't stall at low duty.
# Dutycycle units are pigpio "hardware_PWM" units: 0..1_000_000.
MIN_PWM = 57_000
MAX_PWM = 1_000_000


class DualPWMController:
    """DualPWMController generates two hardware PWM signals using pigpio (or a test mock)."""

    def __init__(self, frequency: int = 250):
        if _pigpio is None:
            raise ImportError("pigpio not available and tests.mocks.mock_pigpio not found")

        self.freq = frequency
        self.duty_cycle_0 = 0
        self.duty_cycle_1 = 0

        # GPIO mapping (reversed to change motor direction)
        self.gpio_pwm0 = 13  # CW
        self.gpio_pwm1 = 12  # CCW

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

    def _map_with_deadzone(self, value: float) -> int:
        """Map value in [-1,+1] to pigpio dutycycle, with offset deadzone."""
        v = abs(float(value))
        if v < 1e-6:
            return 0
        # Clamp magnitude to 1.0 so absurd inputs don't overflow dutycycle
        if v > 1.0:
            v = 1.0
        return int(MIN_PWM + v * (MAX_PWM - MIN_PWM))

    def set_speed(self, value: float):
        """Set signed speed in [-1,+1]."""
        if value < 0.0:
            pwm = self._map_with_deadzone(value)
            self.duty_cycle_0 = 0
            self.duty_cycle_1 = pwm
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)

        elif value > 0.0:
            pwm = self._map_with_deadzone(value)
            self.duty_cycle_0 = pwm
            self.duty_cycle_1 = 0
            self.pi.hardware_PWM(self.gpio_pwm0, self.freq, self.duty_cycle_0)
            self.pi.hardware_PWM(self.gpio_pwm1, self.freq, self.duty_cycle_1)

        else:
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

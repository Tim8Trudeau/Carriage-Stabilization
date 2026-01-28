"""
Module-style pigpio mock for pwm_driver.py.

pwm_driver imports:
    from tests.mocks import mock_pigpio as _pigpio

So this file must exist at tests/mocks/mock_pigpio.py in your repo.
"""

OUTPUT = 1


class _Pi:
    def __init__(self):
        self.connected = True
        self.calls = []  # ("set_mode"| "hardware_PWM" | "stop", ...)

    def set_mode(self, gpio, mode):
        self.calls.append(("set_mode", int(gpio), int(mode)))

    def hardware_PWM(self, gpio, frequency, dutycycle):
        self.calls.append(("hardware_PWM", int(gpio), int(frequency), int(dutycycle)))

    def stop(self):
        self.calls.append(("stop",))


def pi():
    return _Pi()

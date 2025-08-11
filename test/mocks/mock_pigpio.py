# mock_pigpio.py


class MockPi:
    def __init__(self):
        self.calls = []
        self.pwm_states = {}
        pass

    def set_mode(self, gpio, mode):
        self.calls.append(("set_mode", gpio, mode))

    def hardware_PWM(self, gpio, frequency, dutycycle):
        self.calls.append(("hardware_PWM", gpio, frequency, dutycycle))
        self.pwm_states[gpio] = {"frequency": frequency, "dutycycle": dutycycle}

    def write(self, pin, value):
        pass

    def stop(self):
        self.calls.append(("stop",))

    def connected(self):
        return True


class MockPigpio:
    OUTPUT = "OUTPUT"

    @staticmethod
    def pi():
        return MockPi()

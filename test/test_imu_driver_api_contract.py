class FakeSPI:
    """
    Minimal SPI replacement with scripted outputs.
    Provide an iterable of (x_raw, y_raw, omega_raw).
    """
    def __init__(self, samples):
        self._seq = list(samples)
        self._i = 0
        self.closed = False

    def load_samples(self, samples):
        """Replace the remaining sequence with new samples and reset index."""
        self._seq = list(samples)
        self._i = 0

    def imu_read(self, **_):
        if not self._seq:
            x, y, g = 0, 0, 0
        elif self._i < len(self._seq):
            x, y, g = self._seq[self._i]
            self._i += 1
        else:
            # hold last value if sequence ends
            x, y, g = self._seq[-1]
        return _pack3(x, y, g)

    def close(self):
        self.closed = True

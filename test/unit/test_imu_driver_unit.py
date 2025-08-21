# tests/test_imu_driver_unit.py

import math
import pytest

@pytest.mark.unit
@pytest.mark.parametrize(
    "raw_y, raw_x, raw_omega",
    [
        (16384, 0, 0),              # Top of circle y=max, x=0, Omega=0
        (16135, 2845, 0),           # 10° CW,   Omega=0
        (0, 16384, 0),              # 90° CW,   Omega=0
        (11585, 11585, 1000),       # 45° CW,   positive omega (CW)
        (-11585, 11585, 1000),      # 135° CW? (as given)
        (0, 16384, -1000),          # 90° CW,   negative omega (CCW)
        (0, -16384, 0),             # 90° CCW,  Omega=0
        (-8191, 14188, -1000),      # 120° CW,  negative omega
        (-8191, -14188, 1000),      # 120° CCW, positive omega
        (-11585, 11585, 0),         # 135° CW
        (-16384, 0, 0),             # Bottom (180°)
        (8192, -14188, 500),        # -60°,     positive omega
    ],
)
def test_read_normalized_from_inputs(monkeypatch, raw_y, raw_x, raw_omega):
    """
    Adapts to refactor where IMU_Driver reads via LSM6DS3TR driver:
    - Monkeypatch hardware.LSM6DS3TR_driver.LSM6DS3TRDriver with a fake
    - Provide 6 bytes as (x, y, omega) int16 LE
    """
    # Build buffer in NEW order expected by imu_driver: raw_x, raw_y, raw_omega
    sample = bytearray(6)
    sample[0:2] = int(raw_x).to_bytes(2, "little", signed=True)
    sample[2:4] = int(raw_y).to_bytes(2, "little", signed=True)
    sample[4:6] = int(raw_omega).to_bytes(2, "little", signed=True)

    # Fake device driver with same ctor + API shape as real one
    class FakeLSM6DS3TRDriver:
        def __init__(self, *_, **__):
            self._sample = sample
        def readfrom_into(self, _reg, buf):  # pylint: disable=unused-argument
            buf[:] = self._sample
        def read(self, _reg, nbytes):        # pylint: disable=unused-argument
            return bytes(self._sample[:nbytes])
        def write(self, _reg, _data):        # pylint: disable=unused-argument
            pass
        def close(self):
            pass

    # No-op SPI so pigpio isn’t touched on Windows
    class NoopSPIBus:
        # accept arbitrary args to match real ctor signature
        def __init__(self, *_, **__):  # pylint: disable=unused-argument
            pass
        def close(self):
            pass

    # Import the module, then patch the symbols it uses
    from hardware import imu_driver as imu_mod
    monkeypatch.setattr(imu_mod, "IMUDev", FakeLSM6DS3TRDriver, raising=False)
    monkeypatch.setattr(imu_mod, "SPIBus", NoopSPIBus, raising=False)

    # Define params BEFORE using them
    iir_params = {"SAMPLE_RATE_HZ": 100.0, "CUTOFF_FREQ_HZ": 10.0}
    controller_params = {
        "THETA_RANGE_RAD": math.pi,
        "GYRO_FULL_SCALE_RADS_S": 250.0 * math.pi / 180.0,
    }

    # Instantiate via the module (don’t use a bare IMU_Driver symbol)
    imu = imu_mod.IMU_Driver(iir_params, controller_params)

    theta_norm, omega_norm = imu.read_normalized()
    assert isinstance(theta_norm, float)
    assert isinstance(omega_norm, float)
    assert -1.0 <= theta_norm <= 1.0
    assert -1.0 <= omega_norm <= 1.0

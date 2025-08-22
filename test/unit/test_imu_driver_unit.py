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
    IMU_Driver now calls SPIBus.imu_read() which returns a 6-byte buffer in order:
    [raw_x L, raw_x H, raw_y L, raw_y H, raw_omega L, raw_omega H]
    """
    # Build sample buffer exactly as imu_driver expects
    sample = bytearray(6)
    sample[0:2] = int(raw_x).to_bytes(2, "little", signed=True)
    sample[2:4] = int(raw_y).to_bytes(2, "little", signed=True)
    sample[4:6] = int(raw_omega).to_bytes(2, "little", signed=True)

    # Fake IMU device class (not used directly by this test, but patched to avoid real imports)
    class FakeLSM6DS3TRDriver:
        def __init__(self, *_, **__):  # pylint: disable=unused-argument
            pass
        def readfrom_into(self, _reg, _buf):  # pylint: disable=unused-argument
            pass
        def read(self, _reg, _n):  # pylint: disable=unused-argument
            return b""
        def write(self, _reg, _data):  # pylint: disable=unused-argument
            pass
        def close(self):
            pass

    # No-op SPI that returns the prepared 6-byte buffer
    class NoopSPIBus:
        def __init__(self, *_, **__):  # pylint: disable=unused-argument
            pass
        def imu_read(self, **_):  # pylint: disable=unused-argument
            # Option 1: SPI owns/returns the buffer
            return sample
        def close(self):
            pass

    # Patch the symbols used by imu_driver BEFORE constructing IMU_Driver
    from hardware import imu_driver as imu_mod
    monkeypatch.setattr(imu_mod, "IMUDev", FakeLSM6DS3TRDriver, raising=False)
    monkeypatch.setattr(imu_mod, "SPIBus", NoopSPIBus, raising=False)

    # Reasonable params
    iir_params = {"SAMPLE_RATE_HZ": 100.0, "CUTOFF_FREQ_HZ": 10.0}
    controller_params = {
        "THETA_RANGE_RAD": math.pi,
        "GYRO_FULL_SCALE_RADS_S": 250.0 * math.pi / 180.0,
    }

    imu = imu_mod.IMU_Driver(iir_params, controller_params)
    theta_norm, omega_norm = imu.read_normalized()

    assert isinstance(theta_norm, float)
    assert isinstance(omega_norm, float)
    assert -1.0 <= theta_norm <= 1.0
    assert -1.0 <= omega_norm <= 1.0

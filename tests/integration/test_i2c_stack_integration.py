# test/integration/test_i2c_stack_integration.py
import math

def _is_int16(x: int) -> bool:
    return -32768 <= x <= 32767


def test_end_to_end_mock_stack_produces_finite_values(monkeypatch):
    # Force mock I2C host (no pigpio required)
    monkeypatch.setenv("CS_HW", "mock")

    # Import the top-level IMU (uses the I2C-backed device driver under the hood)
    from hardware.imu_driver import IMU_Driver

    imu = IMU_Driver(
        iir_params={"SAMPLE_RATE_HZ": 50.0, "ACCEL_CUTOFF_HZ": 4.0, "CUTOFF_FREQ_HZ": 5.0},
        controller_params={"ACCEL_RAW_FS": 16384, "THETA_RANGE_RAD": math.pi, "GYRO_FULL_SCALE_RADS_S": 4.363},
    )
    try:
        vals = [imu.read_normalized() for _ in range(10)]
    finally:
        imu.close()

    # Each sample is (theta_norm, omega_norm); ensure finite and in bounds
    for theta_norm, omega_norm in vals:
        assert math.isfinite(theta_norm) and -1.0 <= theta_norm <= 1.0
        assert math.isfinite(omega_norm) and -1.0 <= omega_norm <= 1.0

    # The mock source may not advance enough to change rounded values; ensure there's at least
    # a small change between successive theta samples rather than relying on rounding set size.
    thetas = [t for (t, _) in vals]
    if len(thetas) > 1:
        assert any(abs(a - b) > 1e-6 for a, b in zip(thetas, thetas[1:])), "theta_norm did not change across samples"


def test_device_driver_returns_six_bytes_in_int16_range(monkeypatch):
    # Force mock I2C host
    monkeypatch.setenv("CS_HW", "mock")

    from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver

    dev = LSM6DS3TRDriver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x6B})
    try:
        six = dev.read_ax_ay_gz_bytes()
    finally:
        dev.close()

    assert isinstance(six, (bytes, bytearray)) and len(six) == 6

    ax = int.from_bytes(six[0:2], "little", signed=True)
    ay = int.from_bytes(six[2:4], "little", signed=True)
    gz = int.from_bytes(six[4:6], "little", signed=True)

    assert _is_int16(ax) and _is_int16(ay) and _is_int16(gz)

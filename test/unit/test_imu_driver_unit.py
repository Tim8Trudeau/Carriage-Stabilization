import pytest
from hardware.imu_driver import IMU_Driver
from test.mocks import mock_spi


@pytest.mark.parametrize(
    "raw_y, raw_x, raw_omega",
    [
        (16384, 0, 0),  # Top of circle y=max, x=0, Omaga=0
        (16135, 2845, 0),  # 10° CW,   Omaga=0
        (0, 16384, 0),  # 10° CCW,  Omaga=0
        (11585, 11585, 1000),  # 45° CW,   positive omega moving CW
        (-11585, 11585, 1000),  # 45° CCW,  positive omega moving CW
        (0, 16384, -1000),  # 90° CW,   negative omega moving CCW
        (0, -16384, 0),  # 90° CCW,  positive omega moving CW
        (-8191, 14188, -1000),  # 120° CW,  positive omega moving CCW
        (-8191, -14188, 1000),  # 120° CCW, positive omega moving CW
        (-11585, 11585, 0),  # 135° CW
        (-16384, 0, 0),  # Bottom (180°)
        (8192, -14188, 500),  # -60°,     positive omega moving CW
    ],
)
def test_read_normalized_from_inputs(monkeypatch, raw_y, raw_x, raw_omega):
    # Encode raw values
    buffer = bytearray(6)
    buffer[0:2] = raw_y.to_bytes(2, "little", signed=True)
    buffer[2:4] = raw_x.to_bytes(2, "little", signed=True)
    buffer[4:6] = raw_omega.to_bytes(2, "little", signed=True)

    # Patch SPIBus
    class TestSPIBus(mock_spi.SPIBus):
        def __init__(self):
            super().__init__()
            self.test_buffer = buffer

        def readfrom_into(self, addr, buf):
            buf[:] = self.test_buffer

    monkeypatch.setattr("hardware.spi_driver.SPIBus", TestSPIBus)

    # Init IMU
    iir_params = {"SAMPLE_RATE_HZ": 100.0, "CUTOFF_FREQ_HZ": 10.0}
    controller_params = {"THETA_RANGE_RAD": 3.14, "OMEGA_RANGE_RAD_S": 2.0}
    imu = IMU_Driver(iir_params, controller_params)

    # Test output range
    theta, omega = imu.read_normalized()
    assert isinstance(theta, float)
    assert isinstance(omega, float)
    assert -1.5 <= theta <= 1.5
    assert -1.0 <= omega <= 1.0

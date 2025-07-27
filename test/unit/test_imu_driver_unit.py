# test/unit/test_imu_driver_unit.py

import pytest
from hardware.imu_driver import IMU_Driver
from test.mocks import mock_spi


@pytest.fixture
def imu_buffer():
    # Default test buffer: Y=0x0001, X=0xFFFE, Omega=0x0002
    buf = bytearray([0x01, 0x00, 0xFE, 0xFF, 0x02, 0x00])
    return buf


@pytest.fixture
def mock_imu(monkeypatch, imu_buffer):
    # Patch SPIBus with a mock that uses the provided imu_buffer
    class SPIBusWithTestBuffer(mock_spi.SPIBus):
        def __init__(self):
            super().__init__()
            self.test_buffer = imu_buffer

        def readfrom_into(self, address, buffer):
            buffer[:] = self.test_buffer

    monkeypatch.setattr("hardware.spi_driver.SPIBus", SPIBusWithTestBuffer)
    iir_params = {"SAMPLE_RATE_HZ": 100.0, "CUTOFF_FREQ_HZ": 10.0}
    controller_params = {"THETA_RANGE_RAD": 3.14, "OMEGA_RANGE_RAD_S": 2.0}
    return IMU_Driver(iir_params, controller_params)


def test_mock_read(mock_imu, imu_buffer):
    # SPI mock injects imu_buffer into read
    theta, omega = mock_imu.read_normalized()
    assert isinstance(theta, float)
    assert isinstance(omega, float)


def test_filter_initialization(mock_imu):
    assert hasattr(mock_imu, 'b_iir')
    assert hasattr(mock_imu, 'a_iir')
    assert hasattr(mock_imu, 'zi')
    assert len(mock_imu.b_iir) > 0
    assert len(mock_imu.zi) == max(len(mock_imu.a_iir), len(mock_imu.b_iir)) - 1


def test_read_normalized(mock_imu):
    theta, omega = mock_imu.read_normalized()
    assert isinstance(theta, float)
    assert isinstance(omega, float)
    assert -1.0 <= theta <= 1.0
    assert -1.0 <= omega <= 1.0
    

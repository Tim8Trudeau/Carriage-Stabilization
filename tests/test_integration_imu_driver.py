"""
Optional integration-style test: IMU_Driver with a fake underlying device.
This does NOT require pigpio.

If you already have test_i2c_stack_integration.py in your repo, you can keep it.
This file is named differently to avoid collisions.
"""
import math
import pytest


class FakeIMUDevice:
    def __init__(self, controller_params=None):
        self._k = 0

    def read_all_axes(self):
        # produce a slowly changing theta and omega
        # theta via ax changes, omega via gy changes
        self._k += 1
        ax = int(2000 * math.sin(self._k * 0.1))
        az = -16384
        gy = int(5000 * math.cos(self._k * 0.07))
        return (ax, 0, az, 0, gy, 0)

    def close(self):
        pass


def test_imu_driver_outputs_change_over_time(monkeypatch):
    import hardware.imu_driver as imu
    monkeypatch.setattr(imu, "_IMUDevice", lambda controller_params=None: FakeIMUDevice(controller_params))

    d = imu.IMU_Driver(
        iir_params={"SAMPLE_RATE_HZ": 100.0, "ACCEL_CUTOFF_HZ": 5.0, "OMEGA_CUTOFF_HZ": 5.0},
        controller_params={"THETA_RANGE_RAD": math.pi, "DO_GYRO_BIAS_CAL": False},
    )

    vals = [d.read_normalized() for _ in range(20)]
    thetas = [v[0] for v in vals]
    omegas = [v[1] for v in vals]

    assert any(abs(a - b) > 1e-6 for a, b in zip(thetas, thetas[1:]))
    assert any(abs(a - b) > 1e-6 for a, b in zip(omegas, omegas[1:]))

    d.close()

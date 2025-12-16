# test/mocks/mock_pigpio.py
"""
Unified mock pigpio module for I2C- and PWM-based testing.

Implements the subset of pigpio.pi() used by the hardware drivers:
  - i2c_open / i2c_close
  - i2c_read_byte_data / i2c_read_i2c_block_data / i2c_write_byte_data
  - hardware_PWM / set_mode / stop
Also includes a lightweight internal simulation of the LSM6DS3TR IMU
with simple carriage dynamics so integration tests behave realistically.

Usage:
    from test.mocks import mock_pigpio as _pigpio
    pi = _pigpio.pi()
"""

from __future__ import annotations
import math, os, sys
from typing import Optional, Dict

OUTPUT = 1  # pigpio-style constant

# --- Register constants used by IMU driver ---
_CTRL3_C = 0x12
_STATUS_REG = 0x1E
_OUTX_L_G = 0x22
_STATUS_XLDA = 0x01
_STATUS_GDA = 0x02

# --- Helpers ---
_INT16_MIN, _INT16_MAX = -32768, 32767
def _i16(x: float) -> int:
    x = int(round(x))
    if x < _INT16_MIN:
        return _INT16_MIN
    if x > _INT16_MAX:
        return _INT16_MAX
    return x

def _under_pytest() -> bool:
    return "PYTEST_CURRENT_TEST" in os.environ or "pytest" in sys.modules


class _MockPi:
    """
    Unified pigpio 'pi' mock with I2C + PWM support and built-in plant simulation.
    """

    def __init__(self, controller_params: Optional[Dict] = None,
                 sim_config_path: Optional[str] = None, sim_mode: bool = True):
        self.connected = True
        self._closed = False
        self._stopped = False
        self._handle = 1
        self._writes = []
        self.pwm_states: dict[int, dict[str, int]] = {}

        # --- IMU register state ---
        self._ctrl3c_val = 0x00
        self._status_val = _STATUS_XLDA | _STATUS_GDA

        # --- Simulation parameters ---
        p = controller_params or {}
        self._acc_fs = int(p.get("ACCEL_RAW_FS", 16384))
        self._gyro_fs_rads = float(p.get("GYRO_FULL_SCALE_RADS_S", 4.363))
        sr, theta0, omega0 = 50.0, 0.0, 0.0

        # Read sim config if available (skip under pytest)
        try:
            import tomllib  # py3.11+
        except Exception:
            tomllib = None
        if sim_mode and sim_config_path and tomllib and not _under_pytest():
            try:
                with open(sim_config_path, "rb") as f:
                    cfg = tomllib.load(f)
                sim = cfg.get("simulation", {})
                timing = sim.get("timing", {})
                init = sim.get("initial_conditions", {})
                imu = sim.get("imu_model", {})
                sr = float(timing.get("SAMPLE_RATE_HZ", sr))
                theta0 = float(init.get("THETA_INITIAL_RAD", theta0))
                omega0 = float(init.get("OMEGA_INITIAL_RAD_S", omega0))
                self._acc_fs = int(imu.get("ACCEL_RAW_FS", self._acc_fs))
                self._gyro_fs_rads = float(imu.get("GYRO_FS_RAD_S", self._gyro_fs_rads))
            except Exception:
                pass

        self._dt = 1.0 / max(1e-6, sr)
        self._theta, self._omega, self._cmd = theta0, omega0, 0.0
        self._k_u, self._damp = 5.0, 0.5

    # --- Simulation hooks (motor + state) ---
    def set_motor_cmd(self, v: float) -> None:
        self._cmd = max(-1.0, min(1.0, float(v)))

    def get_sim_theta(self) -> float:
        return float(self._theta)

    def _step(self) -> None:
        ddth = self._k_u * self._cmd - self._damp * self._omega
        self._omega += ddth * self._dt
        self._theta += self._omega * self._dt

    # ---------------- PWM mock ----------------
    def set_mode(self, gpio: int, mode: int):
        self._last_mode = (int(gpio), int(mode))

    def hardware_PWM(self, gpio: int, frequency: int, dutycycle: int):
        self.pwm_states[int(gpio)] = {
            "frequency": int(frequency),
            "dutycycle": int(dutycycle),
        }
        return 0  # pigpio success

    # ---------------- I2C mock ----------------
    def i2c_open(self, bus: int, addr: int, flags: int = 0):
        self._bus, self._addr, self._flags = bus, addr, flags
        return self._handle

    def i2c_close(self, handle: int):
        self._closed = True

    def i2c_write_byte_data(self, handle: int, reg: int, val: int):
        self._writes.append((reg & 0xFF, val & 0xFF))
        if reg == _CTRL3_C:
            self._ctrl3c_val = val & 0xFF

    def i2c_read_byte_data(self, handle: int, reg: int) -> int:
        if reg == _CTRL3_C:
            return self._ctrl3c_val
        if reg == _STATUS_REG:
            return self._status_val
        return 0x00

    def i2c_read_i2c_block_data(self, handle: int, reg: int, count: int):
        if reg == _OUTX_L_G:
            self._step()
            ax = _i16(math.sin(self._theta) * self._acc_fs)
            ay = _i16(math.cos(self._theta) * self._acc_fs)
            gz = _i16((self._omega / max(1e-9, self._gyro_fs_rads)) * 32768.0)
            # Compose block: [GX][GY][GZ][AX][AY][AZ]
            block = (
                gz.to_bytes(2, "little", signed=True) +
                gz.to_bytes(2, "little", signed=True) +
                gz.to_bytes(2, "little", signed=True) +
                ax.to_bytes(2, "little", signed=True) +
                ay.to_bytes(2, "little", signed=True) +
                b"\x00\x00"
            )
            return (12, block[:12])
        return (count, bytes([0x00] * count))

    # ---------------- Utility ----------------
    def stop(self):
        self._stopped = True


def pi():
    """Factory mimicking pigpio.pi() constructor."""
    return _MockPi()
# End of mock_pigpio.py
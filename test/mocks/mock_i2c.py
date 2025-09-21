# mock_i2c.py
"""
Project I2C mock with tiny closed-loop plant and optional TOML initial conditions.
Reads TOML only when sim_mode=True and not under pytest.
"""

from __future__ import annotations
import math, os, sys
from typing import Optional, Dict

try:
    import tomllib  # py3.11+
except Exception:
    tomllib = None

_INT16_MIN, _INT16_MAX = -32768, 32767
def _i16(x: float) -> int:
    x = int(round(x));  x = _INT16_MIN if x < _INT16_MIN else _INT16_MAX if x > _INT16_MAX else x;  return x
def _under_pytest() -> bool: return "PYTEST_CURRENT_TEST" in os.environ or "pytest" in sys.modules

class MockI2CBus:
    def __init__(self, controller_params: Optional[Dict] = None,
                 sim_config_path: Optional[str] = None, sim_mode: bool = False):
        p = controller_params or {}
        self._acc_fs = int(p.get("ACCEL_RAW_FS", 16384))
        self._gyro_fs_rads = float(p.get("GYRO_FULL_SCALE_RADS_S", 4.363))
        sr, theta0, omega0 = 50.0, 0.0, 0.0

        if sim_mode and sim_config_path and tomllib and not _under_pytest():
            try:
                with open(sim_config_path, "rb") as f:
                    cfg = tomllib.load(f)
                sim = cfg.get("simulation", {})
                timing = sim.get("timing", {})
                init   = sim.get("initial_conditions", {})
                imu    = sim.get("imu_model", {})
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

    # hooks used by sim harness
    def set_motor_cmd(self, v: float) -> None:
        if v < -1.0: v = -1.0
        if v >  1.0: v =  1.0
        self._cmd = float(v)

    def get_sim_theta(self) -> float:
        return float(self._theta)

    # IMU synthesis
    def _step(self) -> None:
        ddth = self._k_u * self._cmd - self._damp * self._omega
        self._omega += ddth * self._dt
        self._theta += self._omega * self._dt

    def imu_read(self) -> bytes:
        self._step()
        ax = _i16(math.sin(self._theta) * self._acc_fs)
        ay = _i16(math.cos(self._theta) * self._acc_fs)
        gz = _i16((self._omega / max(1e-9, self._gyro_fs_rads)) * 32768.0)
        return (
            ax.to_bytes(2, "little", signed=True) +
            ay.to_bytes(2, "little", signed=True) +
            gz.to_bytes(2, "little", signed=True)
        )
# End of mock_i2c.py

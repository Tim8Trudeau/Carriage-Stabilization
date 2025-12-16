"""Updated simulation runner that routes controller inputs through imu_driver.py
so simulation and hardware share the same IMU processing logic.
"""
from __future__ import annotations

import math
import os
import tomllib
from dataclasses import dataclass

from simulation.central_config import load_simulation_config
from simulation.carriage_simulator import CarriageSimulator
from simulation.plot_sim_results import plot_sim_results
from flc.controller import FLCController


def _load_toml(path: str) -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)


@dataclass
class _SimState:
    theta: float = 0.0
    omega: float = 0.0
    t: float = 0.0


_STATE = _SimState()


class SimIMUDevice:
    def __init__(self, controller_params=None):
        p = controller_params or {}
        self.accel_1g = float(p.get("ACCEL_1G_RAW", 16384.0))
        self.omega_max = float(p.get("OMEGA_MAX_RAD_S", 1.0))

    def read_all_axes(self):
        ax = int(math.sin(_STATE.theta) * self.accel_1g)
        az = int(-math.cos(_STATE.theta) * self.accel_1g)
        ay = 0

        gy = int(max(-1.0, min(1.0, _STATE.omega / self.omega_max)) * 32768.0)
        return ax, ay, az, 0, gy, 0

    def close(self):
        pass


def main():
    plant, motor, sim_cfg, duration, _ctrl, ic = load_simulation_config()

    cfg_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
    sim_cfg_toml = _load_toml(os.path.join(cfg_dir, "sim_config.toml"))

    ctrl_cfg = sim_cfg_toml.get("controller", {})
    flc_path = ctrl_cfg.get("FLC_CONFIG_PATH", "config/flc_config.toml")
    if not os.path.isabs(flc_path):
        flc_path = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", flc_path))

    flc_cfg = _load_toml(flc_path)
    flc = FLCController(flc_cfg)

    scale = flc_cfg.get("scaling", {})
    theta_max = float(scale.get("THETA_MAX_RAD", math.pi / 2))
    omega_max = float(scale.get("OMEGA_MAX_RAD_S", 10.0))

    imu_cfg_path = os.path.join(cfg_dir, "imu_config.toml")
    imu_cfg = _load_toml(imu_cfg_path) if os.path.exists(imu_cfg_path) else {}
    ctrl_params = dict(imu_cfg.get("controller_params", {}) or {})
    iir_params = dict(imu_cfg.get("iir_params", {}) or {})

    ctrl_params.setdefault("THETA_RANGE_RAD", theta_max)
    ctrl_params["OMEGA_MAX_RAD_S"] = omega_max
    ctrl_params["DO_GYRO_BIAS_CAL"] = False

    iir_params.setdefault("SAMPLE_RATE_HZ", 1.0 / max(sim_cfg.dt, 1e-6))

    import hardware.imu_driver as imu_mod
    imu_mod._IMUDevice = SimIMUDevice
    imu_mod.time.perf_counter = lambda: _STATE.t

    imu = imu_mod.IMU_Driver(iir_params, ctrl_params)

    def controller(theta, omega, t):
        _STATE.theta = theta
        _STATE.omega = omega
        _STATE.t = t
        th_n, om_n = imu.read_normalized()
        return flc.calculate_motor_cmd(th_n, om_n)

    sim = CarriageSimulator(plant, motor, sim_cfg, controller)

    theta0, omega0, t0 = ic
    _STATE.theta, _STATE.omega, _STATE.t = theta0, omega0, t0

    sim.reset(theta0, omega0, t0)
    sim.run(duration)
    plot_sim_results(sim)


if __name__ == "__main__":
    main()

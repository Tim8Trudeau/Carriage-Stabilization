# simulation/central_config.py
"""
==================
Unified configuration loader for the Carriage Stabilization project.

This module provides a single, centralized interface for loading all
configuration inputs needed to run the physics-based carriage simulator.
It consolidates the parsing of TOML configuration files and constructs
all required dataclasses and controller objects so that the rest of the
project remains independent of file formats and configuration layout.

Responsibilities
----------------
• Load simulation parameters from:
      config/sim_config.toml

• Load fuzzy-logic controller (Sugeno FLC) configuration from:
      config/flc_config.toml

• Construct the following dataclasses:
      - PlantParams     (mechanical inertia model)
      - MotorParams     (DC motor + gearbox model)
      - SimConfig       (numerical integration and logging settings)

• Read initial conditions:
      (theta0, omega0, start_time)

• Provide simulation duration (seconds)

• Select and build the controller:
      - PD controller      (Kp, Kd from sim_config.toml)
      - Sugeno FLC         (loaded from flc_config.toml)
      - NONE               (open-loop, motor torque = 0)

This design ensures that the simulation engine (`CarriageSimulator`)
remains free of configuration concerns. It accepts only prebuilt
dataclasses and a controller callback, making it fully reusable in tests,
batch simulations, or alternative experiment frameworks.

Returned Values
---------------
load_simulation_config() returns a 6-tuple:

    plant       : PlantParams
    motor       : MotorParams
    sim_cfg     : SimConfig
    duration    : float
    controller  : Callable[[theta, omega, t], motor_cmd]
    ic_tuple    : (theta0, omega0, t0)

Architecture Notes
------------------
• No part of this module depends on plotting, hardware drivers, or
  the simulator itself. This keeps configuration logic isolated.

• FLCController is imported only for building fuzzy controllers.
  Raw TOML data is converted into the controller's internal format via
  FLCController(flc_cfg).

• TOML parsing is done via Python's built-in `tomllib` module.

Typical Usage
-------------
    from simulation.central_config import load_simulation_config
    from simulation.carriage_simulator import CarriageSimulator

    plant, motor, sim_cfg, duration, controller, ic = load_simulation_config()

    sim = CarriageSimulator(plant, motor, sim_cfg, controller)
    sim.reset(*ic)
    sim.run(duration)

    # results available in sim.log_theta, sim.log_cmd, etc.

This loader allows high-level scripts (main.py, test harnesses,
batch sweep tools) to construct simulations cleanly and uniformly.
"""
import tomllib
from typing import Callable, Optional

from simulation.carriage_simulator import PlantParams, MotorParams, SimConfig
from simulation.perturbations import Perturbation
from flc.controller import FLCController


# ------------------------------------------------------------
# Load TOML
# ------------------------------------------------------------
def _load_toml(path: str) -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)


# ------------------------------------------------------------
# Load FLC controller
# ------------------------------------------------------------
def load_flc_from_file(path: str) -> FLCController:
    flc_cfg = _load_toml(path)
    return FLCController(flc_cfg)


# ------------------------------------------------------------
# Main loader
# ------------------------------------------------------------
def load_simulation_config(
    sim_cfg_path: str = "config/sim_config.toml",
):
    """
    Builds and returns the full simulation configuration:

        plant       : PlantParams
        motor       : MotorParams
        sim_cfg     : SimConfig
        duration    : float
        controller  : Callable[theta,omega,t] → cmd
        ic_tuple    : (theta0, omega0, t0)
    """
    cfg = _load_toml(sim_cfg_path)

    # ------------------------------------------------------------
    # Mechanical plant
    # ------------------------------------------------------------
    plant = PlantParams(
        I=float(cfg["plant"]["I"]),
        m=float(cfg["plant"]["m"]),
        r=float(cfg["plant"]["r"]),
        b=float(cfg["plant"].get("b", 0.0)),
        g=float(cfg["plant"].get("g", 9.80665)),
    )

    # ------------------------------------------------------------
    # Motor parameters
    # ------------------------------------------------------------
    motor = MotorParams(
        tau_motor_one=float(cfg["motor"]["tau_motor_one"]),
        n_rollers=int(cfg["motor"]["n_rollers"]),
        r_roller=float(cfg["motor"]["r_roller"]),
        r_wheel=float(cfg["motor"]["r_wheel"]),
    )

    # ------------------------------------------------------------
    # Simulation configuration
    # ------------------------------------------------------------
    sim_cfg = SimConfig(
        dt=float(cfg["simulation"]["dt"]),
        steps_per_log=int(cfg["simulation"]["steps_per_log"]),
    )

    duration = float(cfg["simulation"]["DURATION_S"])

    # ------------------------------------------------------------
    # Initial conditions
    # ------------------------------------------------------------
    icfg = cfg["initial_conditions"]
    ic_tuple = (
        float(icfg["THETA_INITIAL_RAD"]),
        float(icfg["OMEGA_INITIAL_RAD_S"]),
        float(icfg.get("START_TIME_S", 0.0)),
    )

    # ------------------------------------------------------------
    # Disturbance configuration
    # ------------------------------------------------------------
    sim_cfg.perturb = Perturbation()

    # --- Impulses ---
    if "impulse" in cfg and cfg["impulse"].get("enable", False):
        for ev in cfg["impulse"].get("events", []):
            sim_cfg.perturb.add_impulse(
                float(ev["t0"]),
                float(ev["magnitude"]),
            )

    # --- Step disturbances ---
    if "step" in cfg and cfg["step"].get("enable", False):
        for ev in cfg["step"].get("events", []):
            sim_cfg.perturb.add_step(
                float(ev["t0"]),
                float(ev["t1"]),
                float(ev["magnitude"]),
            )

    # --- Deterministic sine wave ---
    if "sine" in cfg and cfg["sine"].get("enable", False):
        s = cfg["sine"]
        sim_cfg.perturb.add_sine(
            amplitude=float(s["amplitude"]),
            freq=float(s["frequency"]),
            phase=float(s.get("phase", 0.0)),
            t_start=float(s.get("t_start", 0.0)),
            t_end=float(s.get("t_end", float("inf"))),
        )

    # --- Random-phase sine wave ---
    if "rnd_sine" in cfg and cfg["rnd_sine"].get("enable", False):
        r = cfg["rnd_sine"]
        sim_cfg.perturb.add_random_phase_sine(
            amplitude=float(r["amplitude"]),
            freq=float(r["frequency"]),
            t_start=float(s.get("t_start", 0.0)),
            t_end=float(s.get("t_end", float("inf"))),
        )

    # --- Multiple sine harmonics ---
    if "multi_sine" in cfg:
        for ev in cfg["multi_sine"]:
            sim_cfg.perturb.add_sine(
                amplitude=float(ev["amplitude"]),
                freq=float(ev["frequency"]),
                phase=float(ev.get("phase", 0.0)),
                t_start=float(ev.get("t_start", 0.0)),
                t_end=float(ev.get("t_end", float("inf"))),
            )

    if "noise" in cfg and cfg["noise"].get("enable", False):
        sim_cfg.perturb.add_noise(
            float(cfg["noise"]["std"])
        )

    # ------------------------------------------------------------
    # Controller selection
    # ------------------------------------------------------------
    ctrl_cfg = cfg["controller"]
    ctrl_type = ctrl_cfg["type"].upper()

    controller: Optional[Callable] = None

    if ctrl_type == "PD":
        Kp = float(ctrl_cfg["Kp"])
        Kd = float(ctrl_cfg["Kd"])

        def pd_controller(theta, omega, t):
            u = Kp * theta + Kd * omega
            return max(-1.0, min(1.0, u))

        controller = pd_controller

    elif ctrl_type == "FLC":
        print(">>> Controller type = FLC")
        flc_path = ctrl_cfg["FLC_CONFIG_PATH"]
        print(">>> Loading FLC config from:", flc_path)

        flc_cfg = _load_toml(flc_path)
        print(">>> Loaded keys:", flc_cfg.keys())

        flc = FLCController(flc_cfg)

        # Input scaling
        scale_cfg = flc_cfg.get("scaling", {})
        theta_max = float(scale_cfg.get("THETA_MAX_RAD", 1.0))
        omega_max = float(scale_cfg.get("OMEGA_MAX_RAD_S", 1.0))

        def flc_controller(theta, omega, t):
            theta_n = max(-1.0, min(1.0, theta / theta_max))
            omega_n = max(-1.0, min(1.0, omega / omega_max))
            return flc.calculate_motor_cmd(theta_n, omega_n)

        controller = flc_controller

    elif ctrl_type == "NONE":
        controller = None

    else:
        raise ValueError(f"Unknown controller type: {ctrl_type}")

    return plant, motor, sim_cfg, duration, controller, ic_tuple

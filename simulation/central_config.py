# central_config.py
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

The controller returned is always a callable mapping simulator state
to a normalized motor command in [-1, +1].

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
from flc.controller import FLCController


# ------------------------------------------------------------
# Load TOML file into Python dict
# ------------------------------------------------------------
def _load_toml(path: str) -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)


# ------------------------------------------------------------
# Load FLC (Sugeno) controller
# ------------------------------------------------------------
def load_flc_from_file(path: str) -> FLCController:
    """
    Load 'config/flc_config.toml' and build an FLCController.

    The FLC expects a Python dict with membership definitions,
    rule table, and defuzzification parameters.
    """
    flc_cfg = _load_toml(path)
    return FLCController(flc_cfg)


# ------------------------------------------------------------
# Load full simulation configuration
# ------------------------------------------------------------
def load_simulation_config(
    sim_cfg_path: str = "config/sim_config.toml"
):
    """
    Loads the complete simulation configuration and returns:

        plant       : PlantParams
        motor       : MotorParams
        sim_cfg     : SimConfig
        duration    : float (seconds)
        controller  : Callable[theta, omega, t] → motor_cmd
        ic_tuple    : (theta0, omega0, t0)

    This supports:
        - PD controller
        - Sugeno FLC controller
        - No controller (open loop)
    """
    cfg = _load_toml(sim_cfg_path)

    # ------------------------------------------------------------
    # Plant parameters
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
        R=float(cfg["motor"]["R"]),
        Kt=float(cfg["motor"]["Kt"]),
        Kv=float(cfg["motor"]["Kv"]),
        V_max=float(cfg["motor"]["V_max"]),
        I_max=float(cfg["motor"]["I_max"]),
        tau_max=float(cfg["motor"]["tau_max"]),
        gear_ratio=float(cfg["motor"]["gear_ratio"]),
        eta=float(cfg["motor"]["eta"]),
    )

    # ------------------------------------------------------------
    # Simulation settings
    # ------------------------------------------------------------
    sim_cfg = SimConfig(
        dt=float(cfg["simulation"]["dt"]),
        steps_per_log=int(cfg["simulation"]["steps_per_log"]),
        tau_ext=float(cfg["simulation"]["tau_ext"]),
    )

    duration = float(cfg["simulation"]["DURATION_S"])

    # ------------------------------------------------------------
    # Initial conditions (theta, omega, t)
    # ------------------------------------------------------------
    icfg = cfg["initial_conditions"]
    ic_tuple = (
        float(icfg["THETA_INITIAL_RAD"]),
        float(icfg["OMEGA_INITIAL_RAD_S"]),
        float(icfg.get("START_TIME_S", 0.0)),
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
        flc_path = ctrl_cfg["FLC_CONFIG_PATH"]

        # Load fuzzy logic config
        flc_cfg = _load_toml(flc_path)
        flc = FLCController(flc_cfg)

        # ------------------------------------------------------------
        # Load scaling parameters for normalization
        # ------------------------------------------------------------
        scale_cfg = flc_cfg.get("scaling", {})
        theta_max = float(scale_cfg.get("THETA_MAX_RAD", 1.0))
        omega_max = float(scale_cfg.get("OMEGA_MAX_RAD_S", 1.0))

        # Create scaling wrapper so simulator inputs are normalized
        def flc_controller(theta, omega, t):
            # Normalize to [-1, +1]
            theta_norm = max(-1.0, min(1.0, theta / theta_max))
            omega_norm = max(-1.0, min(1.0, omega / omega_max))

            # Pass normalized values to Sugeno FLC
            return flc.calculate_motor_cmd(theta_norm, omega_norm)

        controller = flc_controller

    elif ctrl_type == "NONE":
        controller = None

    else:
        raise ValueError(f"Unknown controller type: {ctrl_type}")

    return plant, motor, sim_cfg, duration, controller, ic_tuple

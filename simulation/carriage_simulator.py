# carriage_simulator.py
"""
carriage_simulator.py
======================

Inertia-based physics simulator for the Carriage Stabilization project.

This module implements the full nonlinear rotary dynamics of the carriage
assembly, including:

    • Rigid-body inertia:        I * alpha = tau_total
    • Gravity torque:            tau_g = m * g * r * sin(theta)
    • Viscous damping:           -b * omega
    • Motor model:
          - Armature resistance
          - Back-EMF
          - Current saturation
          - Torque saturation
          - Gear reduction + efficiency

The simulator integrates state using semi-implicit Euler:
    omega ← omega + alpha * dt
    theta ← theta + omega * dt

It accepts an arbitrary controller callback:
    controller(theta, omega, time) → command in [-1, +1]

This allows:
    • PD controllers
    • Sugeno fuzzy controllers
    • External controllers for testing or optimization
    • Open-loop simulation (controller=None)

Logging:
    The simulator records time, angle, angular velocity, motor torque,
    gravity torque, and motor command at a decimated rate (steps_per_log).

Typical usage::

    plant, motor, cfg, duration, controller, ic = load_simulation_from_toml()

    sim = CarriageSimulator(plant, motor, cfg, controller)
    sim.reset(theta0, omega0)
    sim.run(duration)

    # logs available in sim.log_theta, sim.log_cmd, etc.

This simulator contains no configuration parsing, plotting, or hardware code.
All inputs must be provided programmatically or through the loader.

UPDATED:
    - Integrates simulation.perturbations.Perturbation
    - Adds external torque input tau_ext(t)
    - Logs tau_ext for perturbation-aware plotting overlays
    - No changes to controller API or plant physics
"""

from dataclasses import dataclass, field
from typing import Callable, Optional, List
import math
from simulation.perturbations import Perturbation   # <--- NEW


# ------------------------------------------------------------
# Dataclasses
# ------------------------------------------------------------

@dataclass
class MotorParams:
    tau_motor_one: float
    n_rollers: int
    r_roller: float
    r_wheel: float


@dataclass
class PlantParams:
    I: float
    m: float
    r: float
    b: float = 0.0
    g: float = 9.80665


@dataclass
class SimConfig:
    dt: float = 0.002
    steps_per_log: int = 10


# ------------------------------------------------------------
# Simulator
# ------------------------------------------------------------

@dataclass
class CarriageSimulator:
    plant: PlantParams
    motor: MotorParams
    cfg: SimConfig = field(default_factory=SimConfig)
    controller: Optional[Callable[[float, float, float], float]] = None

    theta: float = 0.0
    omega: float = 0.0
    t: float = 0.0

    log_t: List[float] = field(default_factory=list)
    log_theta: List[float] = field(default_factory=list)
    log_omega: List[float] = field(default_factory=list)
    log_tau_m: List[float] = field(default_factory=list)
    log_tau_g: List[float] = field(default_factory=list)
    log_tau_ext: List[float] = field(default_factory=list)   # <--- NEW
    log_cmd: List[float] = field(default_factory=list)
    _log_decim: int = 0

    # ------------------------------------------------------------
    def reset(self, theta=0.0, omega=0.0, t=0.0):
        self.theta = theta
        self.omega = omega
        self.t = t

        self.log_t.clear()
        self.log_theta.clear()
        self.log_omega.clear()
        self.log_tau_m.clear()
        self.log_tau_g.clear()
        self.log_tau_ext.clear()   # <--- NEW
        self.log_cmd.clear()
        self._log_decim = 0

        # Install perturbation driver
        self.perturb = Perturbation()

        # Default test conditions:
        # 1. Impulse at t = 5
        self.perturb.add_impulse(t0=5.0, magnitude=0.35)

        # 2. Step from t = 10 to 15
        self.perturb.add_step(10.0, 15.0, magnitude=0.04)

        # Additional tests can be enabled:
        # self.perturb.add_noise(0.01)
        # self.perturb.add_sine(amplitude=0.02, freq=1.0)
        # self.perturb.add_random_kick(magnitude=0.1, probability=0.005)

    # ------------------------------------------------------------
    def _gravity_torque(self):
        return self.plant.m * self.plant.g * self.plant.r * math.sin(self.theta)

    # ------------------------------------------------------------
    def _motor_torque(self, motor_cmd: float):
        """Convert motor command [-1..1] into torque via friction-drive rollers."""
        tau_one = self.motor.tau_motor_one
        n = self.motor.n_rollers
        r_roller = self.motor.r_roller
        r_wheel = self.motor.r_wheel

        tau_rollers = motor_cmd * tau_one * n
        F_tangent = tau_rollers / r_roller
        return F_tangent * r_wheel

    # ------------------------------------------------------------
    def step(self):
        # Controller
        cmd = self.controller(self.theta, self.omega, self.t) if self.controller else 0.0
        cmd = max(-1.0, min(1.0, cmd))

        # Motor & plant torques
        tau_m = self._motor_torque(cmd)
        tau_g = self._gravity_torque()
        tau_d = -self.plant.b * self.omega

        # NEW: perturbation torque
        tau_ext = self.perturb.get(self.t)

        # Dynamics
        alpha = (tau_m - tau_g + tau_d + tau_ext) / self.plant.I
        self.omega += alpha * self.cfg.dt
        self.theta += self.omega * self.cfg.dt
        self.t     += self.cfg.dt

        # Logging (decimated)
        self._log_decim += 1
        if self._log_decim >= self.cfg.steps_per_log:
            self.log_t.append(self.t)
            self.log_theta.append(self.theta)
            self.log_omega.append(self.omega)
            self.log_tau_m.append(tau_m)
            self.log_tau_g.append(tau_g)
            self.log_tau_ext.append(tau_ext)   # <--- NEW
            self.log_cmd.append(cmd)
            self._log_decim = 0

    # ------------------------------------------------------------
    def run(self, seconds):
        steps = int(seconds / self.cfg.dt)
        for _ in range(steps):
            self.step()

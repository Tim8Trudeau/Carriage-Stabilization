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
"""

from dataclasses import dataclass, field
from typing import Callable, Optional, List, Tuple
import math

# ------------------------------------------------------------
# Dataclasses for mechanical and electrical parameters
# ------------------------------------------------------------

@dataclass
class MotorParams:
    R: float = 3.0
    Kt: float = 0.05
    Kv: float = 0.05
    V_max: float = 12.0
    I_max: float = 5.0
    tau_max: float = 0.6
    gear_ratio: float = 20
    eta: float = 0.85


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
    tau_ext: float = 0.0


# ------------------------------------------------------------
# Main simulation engine
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
        self.log_cmd.clear()
        self._log_decim = 0

    # ------------------------------------------------------------

    def _gravity_torque(self, theta):
        return self.plant.m * self.plant.g * self.plant.r * math.sin(theta)

    def _motor_torque(self, cmd, omega):
        c = max(-1.0, min(1.0, cmd))
        V = c * self.motor.V_max

        omega_m = self.motor.gear_ratio * omega
        i = (V - self.motor.Kv * omega_m) / self.motor.R
        i = max(-self.motor.I_max, min(self.motor.I_max, i))

        tau_shaft = max(-self.motor.tau_max, min(self.motor.tau_max, self.motor.Kt * i))

        return tau_shaft * self.motor.gear_ratio * self.motor.eta

    # ------------------------------------------------------------

    def step(self):
        cmd = self.controller(self.theta, self.omega, self.t) if self.controller else 0.0
        cmd = max(-1.0, min(1.0, cmd))

        tau_m = self._motor_torque(cmd, self.omega)
        tau_g = self._gravity_torque(self.theta)
        tau_d = -self.plant.b * self.omega

        alpha = (tau_m + tau_g + tau_d + self.cfg.tau_ext) / self.plant.I

        self.omega += alpha * self.cfg.dt
        self.theta += self.omega * self.cfg.dt
        self.t     += self.cfg.dt

        self._log_decim += 1
        if self._log_decim >= self.cfg.steps_per_log:
            self.log_t.append(self.t)
            self.log_theta.append(self.theta)
            self.log_omega.append(self.omega)
            self.log_tau_m.append(tau_m)
            self.log_tau_g.append(tau_g)
            self.log_cmd.append(cmd)
            self._log_decim = 0

    # ------------------------------------------------------------

    def run(self, seconds):
        steps = int(seconds / self.cfg.dt)
        for _ in range(steps):
            self.step()

    def last_sample(self):
        if not self.log_t:
            return (self.t, self.theta, self.omega, 0.0, 0.0, 0.0)
        return (
            self.log_t[-1],
            self.log_theta[-1],
            self.log_omega[-1],
            self.log_tau_m[-1],
            self.log_tau_g[-1],
            self.log_cmd[-1],
        )

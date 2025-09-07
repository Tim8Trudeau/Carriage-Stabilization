from __future__ import annotations
from dataclasses import dataclass, asdict
import logging
import math
import os
import random
from typing import Tuple

# simulation.py
import logging, os

class _EnsureI(logging.Filter):
    def filter(self, record):
        if not hasattr(record, "i"):
            record.i = -1
        return True

def setup_sim_logging(log_path: str = "logs/simulation.log") -> None:
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    log = logging.getLogger("simulation")
    if log.handlers:
        return
    log.setLevel(logging.DEBUG)
    fh = logging.FileHandler(log_path, mode="w", encoding="utf-8")
    fmt = logging.Formatter("%(i)06d | %(message)s")
    fh.setFormatter(fmt)
    fh.addFilter(_EnsureI())
    log.addHandler(fh)
    # (optional console handler; add filter too if you include it)



@dataclass
class SimParams:
    # timing
    sample_rate_hz: float
    duration_s: float
    # initial conditions
    theta0_rad: float
    omega0_rad_s: float
    # mechanics
    wheel_radius_m: float
    carriage_mass_kg: float
    motor_force_n: float   # max tangential force at rim for |motor_cmd| = 1
    gravity_m_s2: float
    # imu model
    gyro_fs_rad_s: float   # maps to ±RAW_FS
    accel_raw_fs: int      # RAW full-scale (±)
    noise_span: int        # randint(-noise_span, +noise_span) for accel
    # derived
    dt: float = 0.02       # populated in __post_init__
    steps: int = 0         # populated in __post_init__
    raw_limit: int = 16384 # convenience, clamp target

    def __post_init__(self) -> None:
        self.sample_rate_hz = float(self.sample_rate_hz or 50.0)
        self.duration_s = float(self.duration_s or 10.0)
        self.dt = 1.0 / self.sample_rate_hz
        self.steps = int(self.duration_s * self.sample_rate_hz)
        self.accel_raw_fs = int(self.accel_raw_fs or 16384)
        self.raw_limit = int(self.accel_raw_fs)


class CarriageSimulator:
    """
    Simulates a motorized carriage constrained on the outer rim of a fixed wheel.

    # How the simulation inputs work (what they should look like)
    - `motor_cmd` is a dimensionless control effort in the range [-1, +1].
      - `+1` requests maximum available *tangential* CW force at the wheel rim (`motor_force_n`).
      - `-1` requests the same magnitude in the opposite CCW direction.
    - The motor produces a linear tangential acceleration component: `(motor_force_n * motor_cmd) / carriage_mass_kg`.
    - Gravity introduces a component along the tangential direction: `gravity_m_s2 * sin(theta)`.
      - Positive `theta` means the carriage has moved CW from “top dead center”.
    - Net angular *tangential* acceleration (treated here as linear along the rim) is:
         `alpha = motor_accel + grav_accel`
      and then integrated into `omega` (rad/s) and `theta` (rad) using `dt`.

    # IMU synthesis (raw-like signals)
    - Accelerometer “raw” channels approximate the unit-circle projection with optional noise and then get clamped:
        ```
        x_raw = int(ACCEL_RAW_FS * sin(theta)) + randint(-NOISE_SPAN, +NOISE_SPAN)
        y_raw = int(ACCEL_RAW_FS * cos(theta)) + randint(-NOISE_SPAN, +NOISE_SPAN)
        ```
    - Gyro “raw” is proportional to angular rate and clamped to ±ACCEL_RAW_FS for convenience:
        ```
        omega_raw = int(ACCEL_RAW_FS * clamp(omega / GYRO_FS_RAD_S, -1, +1))
        ```
      Here `GYRO_FS_RAD_S` is the angular-rate full-scale used to normalize ω.

    Dynamics (simple form):
        alpha = (motor_force_n * motor_cmd) / carriage_mass_kg  +  gravity_m_s2 * sin(theta)
        omega <- omega + alpha * dt
        theta <- theta + omega * dt

    IMU synthesis (raw-like):
        x_raw = int(ACCEL_RAW_FS * sin(theta)) + randint(-NOISE_SPAN, +NOISE_SPAN)
        y_raw = int(ACCEL_RAW_FS * cos(theta)) + randint(-NOISE_SPAN, +NOISE_SPAN)
        omega_raw = int(ACCEL_RAW_FS * clamp(omega / GYRO_FS_RAD_S, -1, +1))
    """
    def __init__(self, p: SimParams, *, log_path: str = "logs/simulation.log"):
        setup_sim_logging(log_path)
        self.log = logging.getLogger("simulation")

        self.p = p
        self.theta = float(p.theta0_rad)
        self.omega = float(p.omega0_rad_s)
        self.motor_cmd = 0.0

        # Header: dump all params once at the very top of the log
        self.log.info("===== Simulation Parameters =====")
        for k, v in asdict(self.p).items():
            self.log.info("PARAM %-18s = %s", k, v)
        self.log.info("=================================")

        self.log.info("Initial state: theta= %.3f rad, omega= %.3f rad/s", self.theta, self.omega)

    def set_motor_cmd(self, u: float) -> None:
        # motor_cmd ∈ [-1, +1]
        mc = max(-1.0, min(1.0, float(u)))
        if mc != self.motor_cmd:
            self.log.debug("motor_cmd updated: %.3f -> %.3f", self.motor_cmd, mc)
        self.motor_cmd = mc

    def step(self) -> Tuple[int, int, int, float, float]:
        theta_lo, theta_hi = -2.0,  2.0

        # --- dynamics ---
        mass = max(1e-9, self.p.carriage_mass_kg)
        motor_accel = (self.p.motor_force_n * self.motor_cmd) / mass
        grav_accel  = self.p.gravity_m_s2 * math.sin(self.theta)
        alpha       = motor_accel + grav_accel

        self.omega += alpha * self.p.dt
        self.theta += self.omega * self.p.dt

        # --- limit θ with “hold at limit” behavior ---
        if self.theta > theta_hi:
            self.theta = theta_hi
            # If acceleration is pushing farther from 0 at +limit, don't allow outward motion
            if alpha > 0 or self.omega > 0:
                self.omega = 0.0
        elif self.theta < theta_lo:
            self.theta = theta_lo
            # If acceleration is pushing farther from 0 at -limit, don't allow outward motion
            if alpha < 0 or self.omega < 0:
                self.omega = 0.0

        # --- IMU synthesis ---
        x_raw = int(self.p.accel_raw_fs * math.sin(self.theta)) + random.randint(-self.p.noise_span, self.p.noise_span)
        y_raw = int(self.p.accel_raw_fs * math.cos(self.theta)) + random.randint(-self.p.noise_span, self.p.noise_span)

        if self.p.gyro_fs_rad_s > 0:
            omega_norm = max(-1.0, min(1.0, self.omega / self.p.gyro_fs_rad_s))
        else:
            omega_norm = 0.0
        omega_raw = int(self.p.accel_raw_fs * omega_norm)

        # clamp to ±RAW_FS
        lim = self.p.raw_limit
        x_raw = max(-lim, min(lim, x_raw))
        y_raw = max(-lim, min(lim, y_raw))
        omega_raw = max(-lim, min(lim, omega_raw))

        # Per-step log (make this DEBUG if file sizes get too big)
        self.log.info(
            "x_raw= %6d y_raw= %6d omega_raw= %6d | theta= %.3f rad, omega= %.3f rad/s | "
            "motor_accel= %.3f m/s^2 grav_accel= %.3f m/s^2 alpha= %.3f",
            x_raw, y_raw, omega_raw, self.theta, self.omega, motor_accel, grav_accel, alpha
        )

        return x_raw, y_raw, omega_raw, self.theta, self.omega

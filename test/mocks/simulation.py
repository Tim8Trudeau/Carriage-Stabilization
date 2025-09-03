# simulation.py
from __future__ import annotations
from dataclasses import dataclass
import math
import random

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
    Dynamics (simple form):
        alpha = (motor_force_n * motor_cmd) / carriage_mass_kg  +  gravity_m_s2 * sin(theta)
        omega <- omega + alpha * dt
        theta <- theta + omega * dt

    IMU synthesis (raw-like):
        x_raw = int(ACCEL_RAW_FS * sin(theta)) + randint(-NOISE_SPAN, +NOISE_SPAN)
        y_raw = int(ACCEL_RAW_FS * cos(theta)) + randint(-NOISE_SPAN, +NOISE_SPAN)
        omega_raw = int(ACCEL_RAW_FS * clamp(omega / GYRO_FS_RAD_S, -1, +1))
    """
    def __init__(self, p: SimParams):
        self.p = p
        self.theta = float(p.theta0_rad)
        self.omega = float(p.omega0_rad_s)
        self.motor_cmd = 0.0

    def set_motor_cmd(self, u: float) -> None:
        # motor_cmd ∈ [-1, +1]
        self.motor_cmd = max(-1.0, min(1.0, float(u)))

    def step(self) -> tuple[int, int, int, float, float]:
        # --- dynamics ---
        mass = max(1e-9, self.p.carriage_mass_kg)
        motor_accel = (self.p.motor_force_n * self.motor_cmd) / mass
        grav_accel = self.p.gravity_m_s2 * math.sin(self.theta)
        alpha = motor_accel + grav_accel

        self.omega += alpha * self.p.dt
        self.theta += self.omega * self.p.dt

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

        return x_raw, y_raw, omega_raw, self.theta, self.omega

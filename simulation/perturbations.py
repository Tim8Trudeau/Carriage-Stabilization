"""
perturbations.py
================

Defines the Perturbation class used to inject external disturbances
into the carriage dynamics during simulation. This includes:

    • Impulses
    • Step disturbances
    • Deterministic sine waves (with activation windows)
    • Random-phase sine waves (with activation windows)
    • Multi-sine harmonics
    • Gaussian noise
    • Random torque "kicks" with probability

All disturbance components sum into a single external torque τ_ext
returned by Perturbation.get(t).
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Perturbation:
    # (t0, magnitude)
    impulses: List[Tuple[float, float]] = field(default_factory=list)

    # (t0, t1, magnitude)
    steps: List[Tuple[float, float, float]] = field(default_factory=list)

    # (amplitude, freq, phase, t_start, t_end)
    sine_waves: List[Tuple[float, float, float, float, float]] = field(default_factory=list)

    # (amplitude, freq, phase_random, t_start, t_end)
    rnd_phase_sine: List[Tuple[float, float, float, float, float]] = field(default_factory=list)

    # Noise model
    noise_enable: bool = False
    noise_std: float = 0.0

    # Random kicks: (magnitude, probability)
    random_kicks: List[Tuple[float, float]] = field(default_factory=list)

    # ------------------------------------------------------------------
    # Impulses
    # ------------------------------------------------------------------
    def add_impulse(self, t0: float, magnitude: float):
        self.impulses.append((t0, magnitude))

    # ------------------------------------------------------------------
    # Steps
    # ------------------------------------------------------------------
    def add_step(self, t0: float, t1: float, magnitude: float):
        self.steps.append((t0, t1, magnitude))

    # ------------------------------------------------------------------
    # Deterministic sine wave
    # ------------------------------------------------------------------
    def add_sine(
        self,
        amplitude: float,
        freq: float,
        phase: float = 0.0,
        t_start: float = 0.0,
        t_end: float = float("inf"),
    ):
        self.sine_waves.append((amplitude, freq, phase, t_start, t_end))

    # ------------------------------------------------------------------
    # Random-phase sine wave
    # ------------------------------------------------------------------
    def add_random_phase_sine(
        self,
        amplitude: float,
        freq: float,
        t_start: float = 0.0,
        t_end: float = float("inf"),
    ):
        phase = random.uniform(0, 2 * math.pi)
        self.rnd_phase_sine.append((amplitude, freq, phase, t_start, t_end))

    # ------------------------------------------------------------------
    # Gaussian noise
    # ------------------------------------------------------------------
    def add_noise(self, std: float):
        self.noise_enable = True
        self.noise_std = std

    # ------------------------------------------------------------------
    # Random kicks
    # ------------------------------------------------------------------
    def add_random_kick(self, magnitude: float, probability: float):
        self.random_kicks.append((magnitude, probability))

    # ------------------------------------------------------------------
    # Compute total external disturbance τ_ext(t)
    # ------------------------------------------------------------------
    def get(self, t: float) -> float:
        tau = 0.0

        # Impulses
        for t0, mag in self.impulses:
            if abs(t - t0) < 1e-6:  # single-step pulse
                tau += mag

        # Steps
        for t0, t1, mag in self.steps:
            if t0 <= t <= t1:
                tau += mag

        # Deterministic sine waves
        for amp, freq, phase, t0, t1 in self.sine_waves:
            if t0 <= t <= t1:
                tau += amp * math.sin(2 * math.pi * freq * t + phase)

        # Random-phase sine waves
        for amp, freq, phase, t0, t1 in self.rnd_phase_sine:
            if t0 <= t <= t1:
                tau += amp * math.sin(2 * math.pi * freq * t + phase)

        # Gaussian noise
        if self.noise_enable and self.noise_std > 0:
            tau += random.gauss(0, self.noise_std)

        # Random kicks
        for mag, prob in self.random_kicks:
            if random.random() < prob:
                tau += mag

        return tau

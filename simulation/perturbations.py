"""
Perturbation models for carriage simulation.

This module provides functions that generate time-dependent disturbances
(impulse, step, sinusoidal, noise, random kicks). The main simulator calls
get_disturbance(t) each timestep to retrieve an external torque input.

This class encapsulates ALL possible disturbance sources:
    • Impulse torque disturbances
    • Step torque disturbances
    • Sine-wave disturbances
    • Gaussian noise disturbances
    • Random kick disturbances

All disturbances return an external torque tau_ext(t) added to the plant.
"""

import math
import random

class Perturbation:
    def __init__(self):
        # These MUST exist for plot_sim_results.py
        self.impulses = []        # list of (t0, magnitude)
        self.steps = []           # list of (t1, t2, magnitude)
        self.sine_params = []     # list of (amplitude, frequency)
        self.random_kicks = []    # list of (magnitude, probability)
        self.noise_std = 0.0      # Gaussian noise σ

    # ----------------------------------------------------------
    # Disturbance registration methods
    # ----------------------------------------------------------

    def add_impulse(self, t0: float, magnitude: float):
        """Adds a one-shot impulse torque at time t0."""
        self.impulses.append((t0, magnitude))

    def add_step(self, t1: float, t2: float, magnitude: float):
        """Adds a constant torque between t1 and t2."""
        self.steps.append((t1, t2, magnitude))

    def add_sine(self, amplitude: float, freq: float):
        """Adds a sinusoidal external torque."""
        self.sine_params.append((amplitude, freq))

    def add_noise(self, std: float):
        """Adds Gaussian noise torque."""
        self.noise_std = float(std)

    def add_random_kick(self, magnitude: float, probability: float):
        """Adds randomly occurring torque spikes."""
        self.random_kicks.append((magnitude, probability))

    # ----------------------------------------------------------
    # Disturbance output
    # ----------------------------------------------------------

    def get(self, t: float) -> float:
        """Compute total disturbance torque τ_ext(t)."""
        tau = 0.0

        # Impulse disturbances
        for t0, mag in self.impulses:
            if abs(t - t0) < 1e-3:
                tau += mag

        # Step disturbances
        for t1, t2, mag in self.steps:
            if t1 <= t <= t2:
                tau += mag

        # Sine disturbances
        for amp, freq in self.sine_params:
            tau += amp * math.sin(2 * math.pi * freq * t)

        # Gaussian noise
        if self.noise_std > 0:
            tau += random.gauss(0, self.noise_std)

        # Random kicks
        for mag, prob in self.random_kicks:
            if random.random() < prob:
                tau += mag

        return tau

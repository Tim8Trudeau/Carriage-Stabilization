# plot_sim_results.py
"""
plot_sim_results.py
====================

Plotting utilities for the Carriage Stabilization physics simulator.

This module provides high-level visualization of simulation logs produced by
CarriageSimulator. The primary function `plot_sim_results()` accepts a completed
simulation instance and generates a multi-axis Matplotlib plot showing:

    • Angle (theta) vs time
    • Angular velocity (omega) vs time
    • Motor torque vs time
    • Gravity torque vs time
    • Motor command (−1 to +1)

All plots are saved to a PNG file and optionally displayed interactively.

Typical usage::

    from plot_sim_results import plot_sim_results

    sim.run(10.0)
    plot_sim_results(sim, save_path="plots/sim.png", show=True)

This module contains no physics, configuration, or simulation code and is
safe to modify independently (styling, labels, colors, scaling, etc.).
"""

import matplotlib.pyplot as plt
import os


def plot_sim_results(sim, save_path="plots/sim_output.png", show=True):
    t    = sim.log_t
    th   = sim.log_theta
    om   = sim.log_omega
    tauM = sim.log_tau_m
    tauG = sim.log_tau_g
    cmd  = sim.log_cmd

    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax1.set_title("Carriage Simulation Results")
    ax1.set_xlabel("Time (s)")
    ax1.grid(True, alpha=0.3)

    ax1.plot(t, th, label="theta (rad)")
    ax1.plot(t, om, label="omega (rad/s)")
    ax1.plot(t, tauM, label="motor torque (Nm)")
    ax1.plot(t, tauG, label="gravity torque (Nm)")

    ax2 = ax1.twinx()
    ax2.plot(t, cmd, "r--", alpha=0.6, label="command (-1..+1)")
    ax2.set_ylabel("Motor Command")

    ax1.legend(loc="upper left")

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    if show:
        plt.show()

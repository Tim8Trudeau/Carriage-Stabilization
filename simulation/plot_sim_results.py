# plot_sim_results.py
"""
plot_sim_results.py
====================

Analysis & Plotting utilities for the Carriage Stabilization physics simulator.

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

    sim.run(20.0)
    plot_sim_results(sim, save_path="plots/sim.png", show=True)

This module contains no physics, configuration, or simulation code and is
safe to modify independently (styling, labels, colors, scaling, etc.).

FEATURES:
    • Perturbation-aware plotting: overlay impulse / step / noise disturbances
    • Performance metrics:
         - overshoot
         - settling time (2% band)
         - logarithmic decrement damping ratio estimation
         - rise time
    • Performance metrics
    • Monte-Carlo perturbation testing

Compatible with:
    - CarriageSimulator
    - Perturbation
    - run_simulation.py

Fixes:
    • Clean separation of axes:
          - Left axis: theta, omega, motor torque, gravity torque
          - Right axis: motor command (dimensionless)
          - Right-outer axis: external perturbation torque
    • Gravity torque and external torque are now visually distinct
    • All dashed lines now have unambiguous labels
    • Perturbation regions properly shaded
"""
from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict
from simulation.carriage_simulator import CarriageSimulator


# ============================================================
# PERFORMANCE METRICS
# ============================================================

def compute_overshoot(theta: np.ndarray) -> float:
    final = theta[-1]
    peak = np.max(theta)
    return max(0.0, peak - final)


def compute_settling_time(t: np.ndarray, theta: np.ndarray, band: float = 0.02) -> float:
    final = theta[-1]
    upper = final * (1 + band)
    lower = final * (1 - band)
    for i in range(len(theta)):
        window = theta[i:]
        if np.all((window >= lower) & (window <= upper)):
            return t[i]
    return t[-1]


def estimate_damping_ratio(theta: np.ndarray, t: np.ndarray) -> float:
    peaks = []
    for i in range(1, len(theta) - 1):
        if theta[i] > theta[i-1] and theta[i] > theta[i+1]:
            peaks.append(theta[i])

    if len(peaks) < 2:
        return 1.0

    x1, x2 = peaks[0], peaks[1]
    if x1 <= 0 or x2 <= 0:
        return 1.0

    delta = np.log(x1 / x2)
    return delta / np.sqrt((2*np.pi)**2 + delta**2)


def compute_rise_time(t: np.ndarray, theta: np.ndarray,
                      start_frac=0.1, end_frac=0.9) -> float:
    peak = np.max(theta)
    th_start = peak * start_frac
    th_end = peak * end_frac

    t_start = None
    t_end = None

    for i, th in enumerate(theta):
        if t_start is None and th >= th_start:
            t_start = t[i]
        if t_start is not None and th >= th_end:
            t_end = t[i]
            break

    if t_start is not None and t_end is not None:
        return t_end - t_start

    return np.nan


# ============================================================
# PERTURBATION OVERLAY + ANNOTATION
# ============================================================

def overlay_perturbations(ax, ax_ext, sim: CarriageSimulator):
    t = np.array(sim.log_t)
    tau_ext = np.array(sim.log_tau_ext)

    if np.all(tau_ext == 0):
        return

    ax_ext.plot(t, tau_ext, 'r-.', linewidth=1.8, label="external torque (Nm)")

    # Impulse = sharp sudden change
    for i in range(1, len(tau_ext)):
        if abs(tau_ext[i] - tau_ext[i-1]) > 0.15:
            ax.axvline(t[i], color='red', linestyle='--', alpha=0.5)

    # Step region = sustained nonzero interval
    nz = np.where(np.abs(tau_ext) > 1e-4)[0]
    if len(nz) > 5:
        t1, t2 = t[nz[0]], t[nz[-1]]
        ax.axvspan(t1, t2, color='orange', alpha=0.18)


def annotate_perturbations(ax, sim: CarriageSimulator):
    pert = sim.perturb
    lines = []

    for t0, mag in pert.impulses:
        lines.append(f"Impulse: {mag:+.3f} Nm @ {t0:.2f}s")

    for t1, t2, mag in pert.steps:
        lines.append(f"Step: {mag:+.3f} Nm from {t1:.2f}–{t2:.2f}s")

    for amp, freq in pert.sine_params:
        lines.append(f"Sine: {amp:.3f} Nm @ {freq:.2f} Hz")

    if pert.noise_std > 0:
        lines.append(f"Noise σ = {pert.noise_std:.4f} Nm")

    for mag, prob in pert.random_kicks:
        lines.append(f"Kicks: {mag:+.3f} Nm, p={prob:.3f}")

    if not lines:
        return

    ax.text(
        0.02, 0.98, "\n".join(lines),
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment='top',
        bbox=dict(boxstyle="round,pad=0.4", facecolor="lightyellow", alpha=0.8),
    )


# ============================================================
# MAIN PLOTTING FUNCTION
# ============================================================

def plot_sim_results(sim: CarriageSimulator, title="Carriage Simulation Results"):
    t = np.array(sim.log_t)
    theta = np.array(sim.log_theta)
    omega = np.array(sim.log_omega)
    tau_m = np.array(sim.log_tau_m)
    tau_g = np.array(sim.log_tau_g)
    cmd = np.array(sim.log_cmd)

    fig, ax = plt.subplots(figsize=(11, 6))

    # Left axis signals
    ax.plot(t, theta, label="theta (rad)")
    ax.plot(t, omega, label="omega (rad/s)")
    ax.plot(t, tau_m, label="motor torque (Nm)")
    ax.plot(t, tau_g, 'r--', label="gravity torque (Nm)")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle / Motor / Gravity Torque")
    ax.set_title(title)

    # Right-axis for motor command
    ax_cmd = ax.twinx()
    ax_cmd.plot(t, cmd, color='gray', alpha=0.5, label="motor command")
    ax_cmd.set_ylabel("Motor Command", color='gray')
    ax_cmd.tick_params(axis='y', labelcolor='gray')

    # Right-outer-axis for external torque
    ax_ext = ax.twinx()
    ax_ext.spines["right"].set_position(("axes", 1.12))
    ax_ext.set_ylabel("External Torque (Nm)", color='red')
    ax_ext.tick_params(axis='y', labelcolor='red')

    overlay_perturbations(ax, ax_ext, sim)
    annotate_perturbations(ax, sim)

    # Build combined legend
    lines = ax.get_lines() + ax_cmd.get_lines() + ax_ext.get_lines()
    labels = [ln.get_label() for ln in lines]
    ax.legend(lines, labels, loc="upper right")

    plt.tight_layout()
    plt.show()

    # Performance metrics
    print("\n=== PERFORMANCE METRICS ===")
    overshoot = compute_overshoot(theta)
    settling = compute_settling_time(t, theta)
    zeta = estimate_damping_ratio(theta, t)
    rise = compute_rise_time(t, theta)

    print(f"Overshoot:     {overshoot:.5f} rad")
    print(f"Settling time: {settling:.3f} s")
    print(f"ζ estimate:    {zeta:.3f}")
    print(f"Rise time:     {rise:.3f} s")
    print("====================================\n")


# ============================================================
# MONTE CARLO TESTING
# ============================================================

def monte_carlo_test(sim_factory, N=40) -> Dict[str, float]:
    results = {"zeta": [], "overshoot": [], "settling": []}

    for _ in range(N):
        sim = sim_factory()

        # Randomized disturbances
        sim.perturb.add_impulse(5.0, np.random.uniform(0.2, 0.45))
        sim.perturb.add_step(10.0, 15.0, np.random.uniform(0.025, 0.075))
        sim.perturb.add_noise(np.random.uniform(0.0, 0.01))

        sim.run(20.0)

        t = np.array(sim.log_t)
        theta = np.array(sim.log_theta)

        results["zeta"].append(estimate_damping_ratio(theta, t))
        results["overshoot"].append(compute_overshoot(theta))
        results["settling"].append(compute_settling_time(t, theta))

    print(f"\n===== MONTE CARLO RESULTS (N={N}) =====")
    print(f"ζ mean  = {np.mean(results['zeta']):.3f}")
    print(f"ζ std   = {np.std(results['zeta']):.3f}")
    print(f"Overshoot mean = {np.mean(results['overshoot']):.4f}")
    print(f"Settling mean  = {np.mean(results['settling']):.3f}")
    print("==========================================")

    return results
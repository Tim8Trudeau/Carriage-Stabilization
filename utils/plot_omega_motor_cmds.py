#!/usr/bin/env python3
"""
Omega membership plot with motor_cmd overlay.

This mirrors the theta plot script but targets omega:
- Left axis: membership functions for omega (from config)
- Right axis: motor_cmd (FLC output) for a sweep of omega with a fixed theta

Notes:
- The FLC is called as flc.calculate_motor_cmd(theta, omega)
- theta is held at theta_fixed so we can see how motor_cmd varies with omega alone
"""

import os
import tomllib
import matplotlib.pyplot as plt
import numpy as np


def plot_omega_membership_with_motor_cmd_dual_axis(config_path, omega_sweep, motor_cmds):
    # --- Load membership functions from config ---
    with open(config_path, "rb") as f:
        config = tomllib.load(f)
    omega_mfs = config["membership_functions"]["omega"]

    # --- Get full x-range for shapes ---
    all_points = []
    for points in omega_mfs.values():
        all_points.extend(points)
    min_omega = min(all_points)
    max_omega = max(all_points)

    # --- Plot membership functions on the left axis ---
    fig, ax1 = plt.subplots(figsize=(12, 6))
    x_dense = np.linspace(min_omega, max_omega, 1000)

    for label, shape_points in omega_mfs.items():
        pts = np.array(shape_points)
        if len(pts) == 3:
            # triangular: 0 -> 1 -> 0
            y = np.interp(x_dense, pts, [0, 1, 0], left=0, right=0)
        elif len(pts) == 4:
            # trapezoidal: 0 -> 1 == 1 -> 0
            y = np.interp(x_dense, pts, [0, 1, 1, 0], left=0, right=0)
        else:
            print(f"Warning: Unsupported shape for {label}: {pts}")
            continue
        ax1.plot(x_dense, y, label=label)
        ax1.fill_between(x_dense, y, alpha=0.13)

    ax1.set_xlabel("Omega (normalized value)")
    ax1.set_ylabel("Membership Degree")
    ax1.set_xlim(min_omega, max_omega)
    ax1.set_ylim(0, 1)
    ax1.grid(True)
    ax1.legend(loc="upper left")

    # --- Overlay swept motor_cmd data on the right axis ---
    if (
        omega_sweep is not None
        and motor_cmds is not None
        and len(omega_sweep) > 0
        and len(motor_cmds) > 0
    ):
        ax2 = ax1.twinx()
        ax2.scatter(
            omega_sweep,
            motor_cmds,
            color="red",
            s=20,
            edgecolors="black",
            linewidths=0.8,
            label="motor_cmd (FLC sweep)",
            zorder=10,
        )
        ax2.set_ylabel("Motor Command", color="red")
        ax2.set_ylim(-1, 1)
        ax2.legend(loc="upper right")
        ax2.tick_params(axis="y", colors="red")

    plt.title(
        "Omega Membership Functions (left axis)\nMotor Command (right axis, red dots)"
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # --- Load config and initialize FLCController ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "..", "config", "flc_config.toml")
    config_path = os.path.normpath(config_path)

    # Import your FLCController (adjust this import if your structure is different)
    from flc.controller import FLCController

    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    flc = FLCController(config)

    # --- Choose fixed theta and omega sweep range ---
    theta_fixed = 0.0
    # Omega normalized range is typically [-1.0, 1.0]; if your config defines
    # a wider/narrower set of points, we’ll sweep the actual shape bounds:
    omega_mfs = config["membership_functions"]["omega"]
    omega_points = np.array([p for pts in omega_mfs.values() for p in pts])
    wmin, wmax = float(np.min(omega_points)), float(np.max(omega_points))

    # Generate omega sweep and corresponding motor_cmds
    omega_sweep = np.arange(wmin, wmax + 1e-9, 0.01)
    motor_cmds = [flc.calculate_motor_cmd(theta_fixed, w) for w in omega_sweep]

    # --- Plot everything ---
    plot_omega_membership_with_motor_cmd_dual_axis(config_path, omega_sweep, motor_cmds)

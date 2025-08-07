import os
import tomllib
import matplotlib.pyplot as plt
import numpy as np


def plot_membership_with_motor_cmd_dual_axis(config_path, theta_sweep, motor_cmds):
    # --- Load membership functions from config ---
    with open(config_path, "rb") as f:
        config = tomllib.load(f)
    theta_mfs = config["membership_functions"]["theta"]

    # --- Get full x-range for shapes ---
    all_points = []
    for points in theta_mfs.values():
        all_points.extend(points)
    min_theta = min(all_points)
    max_theta = max(all_points)

    # --- Plot membership functions on the left axis ---
    fig, ax1 = plt.subplots(figsize=(12, 6))
    x_dense = np.linspace(min_theta, max_theta, 1000)

    for label, shape_points in theta_mfs.items():
        points = np.array(shape_points)
        if len(points) == 3:
            y = np.interp(x_dense, points, [0, 1, 0], left=0, right=0)
        elif len(points) == 4:
            y = np.interp(x_dense, points, [0, 1, 1, 0], left=0, right=0)
        else:
            print(f"Warning: Unsupported shape for {label}: {points}")
            continue
        ax1.plot(x_dense, y, label=label)
        ax1.fill_between(x_dense, y, alpha=0.13)

    ax1.set_xlabel("Theta (normalized value)")
    ax1.set_ylabel("Membership Degree")
    ax1.set_xlim(min_theta, max_theta)
    ax1.set_ylim(0, 1)
    ax1.grid(True)
    ax1.legend(loc="upper left")

    # --- Overlay swept motor_cmd data on the right axis ---
    if (
        theta_sweep is not None
        and motor_cmds is not None
        and len(theta_sweep) > 0
        and len(motor_cmds) > 0
    ):
        ax2 = ax1.twinx()
        ax2.scatter(
            theta_sweep,
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
        "Theta Membership Functions (left axis)\nMotor Command (right axis, red dots)"
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # --- Load config and initialize FLCController ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "..", "config", "flc_config.toml")
    config_path = os.path.normpath(config_path)

    # Import your FLCController (adjust this import if your structure is different!)
    from flc.controller import FLCController

    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    flc = FLCController(config)

    # --- Generate theta sweep and motor_cmds ---
    theta_sweep = np.arange(-1.5, 1.5 + 0.001, 0.01)
    omega_sweep = np.zeros_like(theta_sweep)
    # omega_sweep = theta_sweep
    motor_cmds = [
        flc.calculate_motor_cmd(theta, omega)
        for theta, omega in zip(theta_sweep, omega_sweep)
    ]

    # --- Plot everything ---
    plot_membership_with_motor_cmd_dual_axis(config_path, theta_sweep, motor_cmds)

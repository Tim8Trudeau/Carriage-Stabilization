import os
import tomllib
import matplotlib.pyplot as plt
import re
import numpy as np


def plot_membership_with_motor_cmd_dual_axis(config_path, log_path):
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

    # --- Load (theta, motor_cmd) pairs from controller.log ---
    thetas = []
    motor_cmds = []
    if os.path.exists(log_path):
        with open(log_path, "r") as f:
            current_theta = None
            for line in f:
                m = re.search(r"FLC Cycle Start \(theta=\s*([-\d.]+)", line)
                if m:
                    current_theta = float(m.group(1))
                n = re.search(r"FLC Cycle End \(motor_cmd=\s*([-\d.]+)", line)
                if n and current_theta is not None:
                    motor_cmd = float(n.group(1))
                    thetas.append(current_theta)
                    motor_cmds.append(motor_cmd)
                    current_theta = None
    else:
        print(f"controller.log not found at: {log_path}")

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

    # --- Overlay motor_cmd data on the right axis ---
    if thetas and motor_cmds:
        ax2 = ax1.twinx()
        ax2.scatter(
            thetas,
            motor_cmds,
            color="red",
            s=20,
            edgecolors="black",
            linewidths=0.8,
            label="motor_cmd data",
            zorder=10,
        )
        ax2.set_ylabel("Motor Command", color="red")
        ax2.set_ylim(-1, 1)
        ax2.legend(loc="upper right")
        ax2.tick_params(axis="y", colors="red")
    else:
        ax2 = None

    plt.title(
        "Theta Membership Functions (left axis)\nMotor Command (right axis, red dots)"
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "..", "config", "flc_config.toml")
    log_path = os.path.join(script_dir, "..", "logs", "controller.log")
    config_path = os.path.normpath(config_path)
    log_path = os.path.normpath(log_path)
    plot_membership_with_motor_cmd_dual_axis(config_path, log_path)

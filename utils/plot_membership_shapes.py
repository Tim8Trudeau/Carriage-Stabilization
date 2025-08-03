# utilities/plot_membership_shapes.py

import json5
import matplotlib.pyplot as plt
import os


def plot_membership_functions(mf_data, title, save=False, output_dir="plots"):
    """
    Plot triangular and trapezoidal membership functions from config.

    Args:
        mf_data (dict): Dictionary of label -> list of shape points
        title (str): Title of the plot
        save (bool): Whether to save the plot as a PNG
        output_dir (str): Directory to save the plot
    """
    plt.figure(figsize=(8, 4))
    for label, points in mf_data.items():
        if len(points) == 3:
            y = [0, 1, 0]  # Triangle
        elif len(points) == 4:
            y = [0, 1, 1, 0]  # Trapezoid
        else:
            print(f"Warning: Unsupported shape for {label}: {points}")
            continue

        plt.plot(points, y, label=label)
        plt.fill_between(points, y, alpha=0.1)

    plt.title(f"Membership Functions – {title}")
    plt.xlabel("Normalized Value")
    plt.ylabel("Membership Degree")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    if save:
        os.makedirs(output_dir, exist_ok=True)
        filename = os.path.join(output_dir, f"{title.lower()}_membership_functions.png")
        plt.savefig(filename)
        print(f"Saved plot to: {filename}")

    plt.show()


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Plot fuzzy membership function shapes."
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save plots as PNG files in the 'plots/' directory.",
    )
    args = parser.parse_args()

    config_path = os.path.join("config", "flc_config.json5")
    if not os.path.exists(config_path):
        print(f"Error: Config file not found at: {config_path}")
        return

    with open(config_path, "r") as f:
        config = json5.load(f)

    mf_section = config.get("membership_functions", {})
    theta_mfs = mf_section.get("theta", {})
    omega_mfs = mf_section.get("omega", {})

    if not theta_mfs or not omega_mfs:
        print("Error: Could not find theta or omega membership function definitions.")
        return

    plot_membership_functions(theta_mfs, "Theta", save=args.save)
    plot_membership_functions(omega_mfs, "Omega", save=args.save)


if __name__ == "__main__":
    main()

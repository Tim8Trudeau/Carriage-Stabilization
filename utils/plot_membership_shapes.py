import tomllib
import matplotlib.pyplot as plt
import os
import re


def plot_membership_functions(
    mf_data, title, points=None, save=False, output_dir="plots"
):
    """
    Plot triangular and trapezoidal membership functions from config.
    Optionally overlay points as red dots.
    Args:
        mf_data (dict): Dictionary of label -> list of shape points
        title (str): Title of the plot
        points (list of (x, y)): Points to overlay (optional)
        save (bool): Whether to save the plot as a PNG
        output_dir (str): Directory to save the plot
    """
    plt.figure(figsize=(8, 4))
    for label, shape_points in mf_data.items():
        if len(shape_points) == 3:
            y = [0, 1, 0]  # Triangle
        elif len(shape_points) == 4:
            y = [0, 1, 1, 0]  # Trapezoid
        else:
            print(f"Warning: Unsupported shape for {label}: {shape_points}")
            continue

        plt.plot(shape_points, y, label=label)
        plt.fill_between(shape_points, y, alpha=0.1)

    # Overlay points if given
    if points is not None and len(points) > 0:
        x, y = zip(*points)
        print("Overlay points:", len(x))
        plt.scatter(
            x,
            y,
            color="red",
            s=30,
            marker="o",
            edgecolors="black",
            linewidths=0.8,
            label="(Input, Z) from log",
            zorder=10,
        )

    plt.title(f"Membership Functions – {title}")
    plt.xlabel("Normalized Value")
    plt.ylabel("Membership Degree / Z")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    if save:
        os.makedirs(output_dir, exist_ok=True)
        filename = os.path.join(output_dir, f"{title.lower()}_membership_functions.png")
        plt.savefig(filename)
        print(f"Saved plot to: {filename}")

    plt.show()


def parse_rule_engine_log(log_path):
    """
    Parse log for Theta, Omega, W, Z sequences.
    Returns list of dicts: [{'theta': val, 'omega': val, 'W': [...], 'Z': [...]}]
    """
    step_data = []
    theta = None
    omega = None
    current = None

    with open(log_path, "r") as f:
        lines = f.readlines()

    for line in lines:
        # Match the Theta line
        m_theta = re.match(r"\s*Theta:\s*([-\d.]+),", line)
        if m_theta:
            theta = float(m_theta.group(1))
            continue  # Next line will have Omega

        # Match the Omega line (assume it's always right after Theta)
        m_omega = re.match(r"\s*Omega:\s*([-\d.]+),", line)
        if m_omega and theta is not None:
            omega = float(m_omega.group(1))
            # Prepare a new step entry, add after W/Z line is found
            current = {"theta": theta, "omega": omega, "W": [], "Z": []}
            theta = None  # Reset for next block
            continue

        # Match Rule fire lines (W and Z values)
        m_rule = re.search(r"W=([-\d.]+), Z=([-\d.]+)", line)
        if m_rule and current:
            w = float(m_rule.group(1))
            z = float(m_rule.group(2))
            current["W"].append(w)
            current["Z"].append(z)
            step_data.append(current)
            current = None  # Reset for next step

    return step_data


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
    config_path = os.path.join("config", "flc_config.toml")
    if not os.path.exists(config_path):
        print(f"Error: Config file not found at: {config_path}")
        return

    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    mf_section = config.get("membership_functions", {})
    theta_mfs = mf_section.get("theta", {})
    omega_mfs = mf_section.get("omega", {})

    # Attempt to find and parse the log
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(SCRIPT_DIR, "..", "logs", "rule_engine.log")
    log_path = os.path.normpath(log_path)
    print(f"Looking for rule_engine.log at: {log_path}")
    overlay_theta = None
    overlay_omega = None
    if os.path.exists(log_path):
        print(f"Overlaying with values from: {log_path}")
        steps = parse_rule_engine_log(log_path)
        # Points: (theta, Z), using weighted average Z for each step
        overlay_theta = [
            (
                s["theta"],
                sum(w * z for w, z in zip(s["W"], s["Z"])) / sum(s["W"])
                if s["W"]
                else 0.0,
            )
            for s in steps
        ]
        overlay_omega = [
            (
                s["omega"],
                sum(w * z for w, z in zip(s["W"], s["Z"])) / sum(s["W"])
                if s["W"]
                else 0.0,
            )
            for s in steps
        ]
    else:
        print(
            f"No rule_engine.log found at {log_path}, plotting membership functions only."
        )

    plot_membership_functions(theta_mfs, "Theta", points=overlay_theta, save=args.save)
    plot_membership_functions(omega_mfs, "Omega", points=overlay_omega, save=args.save)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback

        print("CRITICAL ERROR:", e)
        traceback.print_exc()
        input("Press Enter to exit...")

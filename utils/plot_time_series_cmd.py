# utils/plot_time_series_cmd.py
import os
import argparse
import tomllib
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from flc.controller import FLCController
from test.mocks.mock_spi import MockSPIBus  # same source as plot_omega_motor_cmd.py


# ----------------------------- Mock collection ----------------------------- #
def collect_mock_samples(n_samples: int, theta_step: float, omega_mode: str):
    """
    Collect (raw_y, raw_x, raw_omega) triplets from the mock SPI.
    - theta_step: degrees/iteration for mock motion
    - omega_mode: one of {"constant", "noisy", "random", "none"} (case-insensitive)
    """
    spi = MockSPIBus(omega_mode=omega_mode, theta_step=theta_step)
    buf = bytearray(6)
    samples = []
    try:
        for _ in range(n_samples):
            # Fills buf = [y0,y1, x0,x1, om0,om1] (int16 LE)
            spi.readfrom_into(0x00, buf)
            raw_y     = int.from_bytes(buf[0:2], "little", signed=True)
            raw_x     = int.from_bytes(buf[2:4], "little", signed=True)
            raw_omega = int.from_bytes(buf[4:6], "little", signed=True)
            samples.append((raw_y, raw_x, raw_omega))
    finally:
        # Optional but tidy
        if hasattr(spi, "close"):
            try:
                spi.close()
            except Exception:
                pass
    return samples


# ----------------------------- Time-series run ----------------------------- #
def run_time_series(config_path: str,
                    n_samples: int,
                    theta_step: float,
                    omega_mode: str,
                    sample_interval_ms: int = 10,
                    save: bool = False):

    # Load config
    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    # Controller
    flc = FLCController(config)

    # Controller scaling params (from your TOML)
    ACCEL_FS_G  = float(config["controller_params"]["ACCEL_FULL_SCALE_G"])      # e.g., 2.0
    GYRO_FS_DPS = float(config["controller_params"]["GYRO_FULL_SCALE_DPS"])     # e.g., 250.0

    # Acquire mock raw samples
    raw_samples = collect_mock_samples(n_samples, theta_step, omega_mode)

    # Convert + evaluate FLC
    theta_deg_series   = []
    omega_deg_s_series = []
    motor_cmds         = []

    for raw_y, raw_x, raw_omega in raw_samples:
        # 1) Raw accel -> g (assuming int16 full scale -> +/- 32768)
        ay_g = (raw_y / 32768.0) * ACCEL_FS_G
        ax_g = (raw_x / 32768.0) * ACCEL_FS_G

        # 2) Theta from gravity vector (x,y plane). Adjust axes if your convention differs.
        theta_rad = np.arctan2(ax_g, ay_g)
        theta_deg = np.degrees(theta_rad)

        # 3) Raw gyro -> deg/s
        omega_deg_s = (raw_omega / 32768.0) * GYRO_FS_DPS
        omega_rad_s = np.radians(omega_deg_s)

        # 4) FLC (expects radians, rad/s)
        motor_cmd = flc.calculate_motor_cmd(theta_rad, omega_rad_s)

        theta_deg_series.append(theta_deg)
        omega_deg_s_series.append(omega_deg_s)
        motor_cmds.append(motor_cmd)

    # Time vector (seconds)
    dt = sample_interval_ms / 1000.0
    time = np.arange(n_samples) * dt

    # ------------------------------- Plotting ------------------------------- #
    fig, ax1 = plt.subplots(figsize=(12, 6))

    # Left axis: theta (deg) and omega (deg/s)
    ax1.plot(time, theta_deg_series, label="Theta (deg)", color="tab:blue")
    ax1.plot(time, omega_deg_s_series, label="Omega (deg/s)", color="tab:green", alpha=0.75)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Theta (deg) / Omega (deg/s)")
    ax1.grid(True)
    # X axis: minor ticks every 1s
    ax1.xaxis.set_minor_locator(MultipleLocator(1.0))

    # Left Y axis (degrees/deg/s): minor ticks every 2°
    ax1.yaxis.set_minor_locator(MultipleLocator(2.0))

    # Make minor grid visible (dotted), keep major grid solid
    ax1.grid(True, which="major", linewidth=0.8)
    ax1.grid(True, which="minor", linestyle=":", alpha=0.4)

    # Right axis: motor command [-1, 1]
    ax2 = ax1.twinx()
    ax2.plot(time, motor_cmds, label="Motor Cmd", color="tab:red")
    ax2.set_ylabel("Motor Cmd [-1, 1]", color="tab:red")
    ax2.set_ylim(-1.1, 1.1)
    ax2.tick_params(axis="y", colors="tab:red")

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    plt.title(f"Theta, Omega, and Motor Command vs Time  "
              f"(samples={n_samples}, θ_step={theta_step}, ω_mode={omega_mode})")
    plt.tight_layout()

    if save:
        # Save to <project_root>/plots/
        root_dir = os.path.abspath(os.path.join(os.path.dirname(config_path), ".."))
        out_dir = os.path.join(root_dir, "plots")
        os.makedirs(out_dir, exist_ok=True)
        out_file = os.path.join(out_dir, "theta_omega_motor_cmd_timeseries.png")
        plt.savefig(out_file, dpi=150)
        print(f"Saved plot to {out_file}")
    else:
        plt.show()


# --------------------------------- CLI ------------------------------------ #
if __name__ == "__main__":
    # Resolve config relative to this script
    script_dir  = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.normpath(os.path.join(script_dir, "..", "config", "flc_config.toml"))

    # Read config (for defaults)
    with open(config_path, "rb") as f:
        _cfg = tomllib.load(f)

    default_hz = float(_cfg["controller_params"].get("TARGET_HZ", 50.0))  # if you want to derive samples
    default_samples = int(default_hz * 2)  # ~2 seconds by default

    parser = argparse.ArgumentParser(description="Plot Theta (deg), Omega (deg/s), and Motor Cmd vs Time")
    parser.add_argument("--samples", type=int, help="number of samples (default ~2s of TARGET_HZ)")
    parser.add_argument("--theta-step", type=float, help="theta increment per sample (degrees/iter)")
    parser.add_argument("--omega-mode", choices=["constant", "noisy", "random", "none"], default="none")
    parser.add_argument("--interval", type=int, default=10, help="sample interval in ms (default 10ms)")
    parser.add_argument("--save", action="store_true", help="save PNG to plots/ instead of showing")

    args = parser.parse_args()

    n_samples = args.samples if args.samples is not None else default_samples
    theta_step = args.theta_step if args.theta_step is not None else 1

    # Run
    run_time_series(
        config_path=config_path,
        n_samples=n_samples,
        theta_step=theta_step,
        omega_mode=args.omega_mode,
        sample_interval_ms=args.interval,
        save=args.save,
    )

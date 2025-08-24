# utils/plot_time_series_cmd.py
import os
import logging
import argparse
import math
from tkinter import N
import tomllib
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from utils.logger import setup_logging

# Runtime controller + IMU
from flc.controller import FLCController
from hardware import imu_driver as imu_mod

# Mock SPI (we’ll inject this into imu_driver.SPIBus)
from test.mocks.mock_spi import MockSPIBus


def run_time_series(config_path: str,
                    n_samples: int,
                    theta_step_deg: float,
                    omega_mode: str,
                    omega_slope: float,
                    sample_interval_ms: int = 10,
                    save: bool = False):
    """
    Generate a time-series plot where synthetic IMU data flows through the same
    pipeline as production:
      MockSPIBus -> IMU_Driver.read_normalized() -> FLCController.calculate_motor_cmd()
    """

    # ---- Load config (same as runtime) ----
    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    controller_params = dict(config.get("controller_params", {}))
    iir_params = dict(config.get("iir_filter", {})) if "iir_filter" in config else {}

    # Pull runtime gyro scale (rad/s full-scale)
    gyro_fs_rps = float(controller_params.get("GYRO_FULL_SCALE_RADS_S", 4.363323129985823))
    # IMU_Driver normalizes theta to [-1, +1] using pi, so we need pi to convert back

    # ---- Inject Mock SPI into imu_driver so IMU_Driver uses it ----
    # Allow TOML defaults for the mock, but let CLI override
    mock_cfg = dict(controller_params.get("MOCK_SPI", {}))
    omega_mode = (omega_mode or mock_cfg.get("OMEGA_MODE") or "none").lower()
    omega_slope = float(omega_slope if omega_slope is not None else mock_cfg.get("OMEGA_SLOPE", 0.0))

    theta_step_deg = float(theta_step_deg if theta_step_deg is not None else mock_cfg.get("THETA_STEP_DEG", 1.0))

    class _InjectedSPIBus(MockSPIBus):
        """Factory matching imu_driver.SPIBus(controller_params) signature."""
        def __init__(self, _controller_params):
            # You can honor more mock knobs here if you add them to TOML
            super().__init__(omega_mode=omega_mode, omega_slope=omega_slope, theta_step=theta_step_deg)

    # Monkey-patch the symbol imu_driver uses to create its SPI
    imu_mod.SPIBus = _InjectedSPIBus  # type: ignore[attr-defined]

    # ---- Build runtime objects ----
    flc = FLCController(config)
    imu = imu_mod.IMU_Driver(iir_params, controller_params)

    # ---- Collect through the REAL pipeline ----
    theta_deg_series = []
    omega_deg_s_series = []
    motor_cmds = []

    try:
        for _ in range(n_samples):
            # IMU_Driver returns normalized values:
            #   theta_norm in [-1, +1] where 1 maps to +pi radians
            #   omega_norm in [-1, +1] where 1 maps to gyro_fs_rps rad/s
            theta_norm, omega_norm = imu.read_normalized()

            theta_rad = theta_norm * math.pi
            omega_rps = omega_norm * gyro_fs_rps

            motor_cmd = flc.calculate_motor_cmd(theta_norm, omega_norm)

            theta_deg_series.append(math.degrees(theta_rad))
            omega_deg_s_series.append(math.degrees(omega_rps))
            motor_cmds.append(float(motor_cmd))
    finally:
        try:
            imu.close()
        except Exception:
            pass

    # ---- Time base ----
    dt = sample_interval_ms / 1000.0
    time = np.arange(n_samples) * dt

    # ---- Plot ----
    fig, ax1 = plt.subplots(figsize=(12, 6))

    ax1.plot(time, theta_deg_series, label="Theta (deg)", color="tab:blue")
    ax1.plot(time, omega_deg_s_series, label="Omega (deg/s)", color="tab:green", alpha=0.75)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Theta (deg) / Omega (deg/s)")
    ax1.xaxis.set_minor_locator(MultipleLocator(1.0))
    ax1.yaxis.set_minor_locator(MultipleLocator(10.0))
    ax1.grid(True, which="major", linewidth=0.8)
    ax1.grid(True, which="minor", linestyle=":", alpha=0.4)

    ax2 = ax1.twinx()
    ax2.plot(time, motor_cmds, label="Motor Cmd", color="tab:red")
    ax2.set_ylabel("Motor Cmd [-1, 1]", color="tab:red")
    ax2.set_ylim(-1.1, 1.1)
    ax2.tick_params(axis="y", colors="tab:red")
# --- Add dashed zero line for motor_cmd ---
    ax2.axhline(0, color="tab:red", linestyle="--", linewidth=1, alpha=0.7)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    plt.title(
        f"Theta, Omega, and Motor Command vs Time  "
        f"(samples={n_samples}, θ_step={theta_step_deg}°/sample, ω_mode={omega_mode})"
    )
    plt.tight_layout()

    if save:
        root_dir = os.path.abspath(os.path.join(os.path.dirname(config_path), ".."))
        out_dir = os.path.join(root_dir, "plots")
        os.makedirs(out_dir, exist_ok=True)
        out_file = os.path.join(out_dir, "theta_omega_motor_cmd_timeseries.png")
        plt.savefig(out_file, dpi=150)
        print(f"Saved plot to {out_file}")
    else:
        plt.show()


if __name__ == "__main__":
    # Resolve config relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.normpath(os.path.join(script_dir, "..", "config", "flc_config.toml"))

    setup_logging()

    # Read config to derive defaults
    with open(config_path, "rb") as f:
        _cfg = tomllib.load(f)
    default_hz = float(_cfg["controller_params"].get("TARGET_HZ", 50.0))
    default_samples = int(default_hz * 2)  # ~2 seconds
    default_theta_step = float(_cfg["controller_params"].get("TARGET_HZ", 50.0))
    default_theta_step = float(_cfg["controller_params"].get("MOCK_SPI", {}).get("THETA_STEP", 1.0))
    default_samples = float(_cfg["controller_params"].get("MOCK_SPI", {}).get("SAMPLES", default_samples))

    parser = argparse.ArgumentParser(description="Plot Theta/Omega/Motor Cmd via real runtime pipeline")
    parser.add_argument("--samples", type=int, default=default_samples, help="number of samples")
    parser.add_argument("--theta-step", type=float, default=default_theta_step, help="theta increment per sample (deg/sample)")
    parser.add_argument("--omega-mode", choices=["constant", "noisy", "random", "slope", "none"], default=None,
                        help="override MOCK_SPI.OMEGA_MODE")
    parser.add_argument("--omega-slope", type=float, default=None,
                        help="Set omega slope (deg/s per sample) for 'slope' mode")
    parser.add_argument("--interval", type=int, default=10, help="sample interval in ms")
    parser.add_argument("--save", action="store_true", help="save PNG to plots/ instead of showing")
    args = parser.parse_args()
    n_samples=int(args.samples)

    run_time_series(
        config_path=config_path,
        n_samples=n_samples,
        theta_step_deg=args.theta_step,
        omega_mode=args.omega_mode,
        omega_slope=args.omega_slope,
        sample_interval_ms=args.interval,
        save=args.save,
    )

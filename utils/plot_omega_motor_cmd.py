#!/usr/bin/env python3
"""
Omega membership plot with motor_cmd overlay using MockSPIBus.

Run:
    python -m utils.plot_omega_motor_cmd
Options:
    --samples N        number of mock samples (default: 600)
    --save             save figure to plots/omega_motor_cmd.png
    --omega-mode M     override MOCK_SPI OMEGA_MODE (constant|noisy|random|none)
    --random           Generate random omega samples (shorthand for --omega-mode random)
    --noisy            Generate noisy omega samples (shorthand for --omega-mode noisy)
    --constant         Generate constant omega samples (shorthand for --omega-mode constant)
"""

from __future__ import annotations
import argparse
import math
import os
import sys
import random
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np

# Python 3.11+ has tomllib in stdlib
try:
    import tomllib  # py311+
except Exception:  # pragma: no cover
    import tomli as tomllib  # fallback if needed


def load_config(repo_root: str) -> Dict:
    cfg_path = os.path.join(repo_root, "config", "flc_config.toml")
    with open(cfg_path, "rb") as f:
        return tomllib.load(f)


def build_mock_from_config(cfg: Dict, override_omega_mode: str | None = None):
    """
    Construct test.mocks.mock_spi.MockSPIBus using controller_params.MOCK_SPI.
    Accepts nested or flat dictionaries.
    """
    # Lazy import to avoid polluting other contexts
    from test.mocks.mock_spi import MockSPIBus

    ctrl = cfg.get("controller_params", {})
    spi_flat = ctrl.get("MOCK_SPI", ctrl.get("mock_spi", {}))

    spi_channel = int(ctrl.get("SPI_CHANNEL", 0))
    spi_baud    = int(ctrl.get("SPI_BAUD", 1_000_000))
    spi_mode    = int(ctrl.get("SPI_MODE", 0))

    omega_mode = (override_omega_mode or spi_flat.get("OMEGA_MODE", "constant") or "constant")
    omega_raw_base  = int(spi_flat.get("OMEGA_RAW_BASE", 5000))
    noise_span      = int(spi_flat.get("NOISE_SPAN", 2000))
    theta_step      = int(spi_flat.get("THETA_STEP", 5))

    return MockSPIBus(
        spi_channel=spi_channel,
        baud=spi_baud,
        io_mode=spi_mode,
        omega_mode=omega_mode,
        omega_raw_base=omega_raw_base,
        noise_span=noise_span,
        theta_step=theta_step,
    )


def compute_alpha(sample_rate_hz: float, cutoff_freq_hz: float) -> float:
    dt = 1.0 / float(sample_rate_hz)
    rc = 1.0 / (2.0 * math.pi * float(cutoff_freq_hz))
    return dt / (rc + dt)


def membership_points_from_config(cfg: Dict) -> Dict[str, List[float]]:
    """
    Expects:
      [membership_functions.omega]
      NEG_BIG  = [-1.0, -1.0, -0.7]      # tri
      NEG_MED  = [-1.0, -0.7, -0.4, 0.0] # trap
      ...
    Returns {label: [x1,x2,x3,(x4)]} in normalized omega units.
    """
    mf = cfg.get("membership_functions", {})
    omega_mf = mf.get("omega", {})
    if not omega_mf:
        raise KeyError("membership_functions.omega not found in config.")
    return omega_mf


def plot_memberships(ax, mf_map: Dict[str, List[float]]):
    # Determine global x-range
    xs = [x for pts in mf_map.values() for x in pts]
    xmin, xmax = min(xs), max(xs)
    x_dense = np.linspace(xmin, xmax, 1000)

    for label, pts in mf_map.items():
        pts_arr = np.array(pts, dtype=float)
        if len(pts_arr) == 3:
            # triangular
            y = np.interp(x_dense, pts_arr, [0.0, 1.0, 0.0], left=0.0, right=0.0)
        elif len(pts_arr) == 4:
            # trapezoidal
            y = np.interp(x_dense, pts_arr, [0.0, 1.0, 1.0, 0.0], left=0.0, right=0.0)
        else:
            print(f"Warning: unsupported membership shape for '{label}': {pts}", file=sys.stderr)
            continue
        ax.plot(x_dense, y, label=label)
        ax.fill_between(x_dense, y, alpha=0.12)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(0.0, 1.0)
    ax.set_xlabel("Omega (normalized)")
    ax.set_ylabel("Membership degree")
    ax.grid(True)
    ax.legend(loc="upper left")

    return xmin, xmax


def collect_mock_samples(cfg: Dict, samples: int, override_omega_mode: str | None = None):
    """
    Use MockSPIBus to produce raw y/x/omega.
    Convert to:
      theta_norm = atan2(x, y) / THETA_RANGE_RAD
      omega_norm = (IIR-filtered omega_rads) / OMEGA_RANGE_RAD_S
    """
    mock = build_mock_from_config(cfg, override_omega_mode)
    print("USING:", mock.omega_mode, "NOISE_SPAN:", mock.noise_span)

    # IMU / filter params (match your IMU_Driver / config semantics)
    iir = cfg.get("iir_params", {})
    sample_rate_hz = float(iir.get("SAMPLE_RATE_HZ", 100.0))
    cutoff_freq_hz = float(iir.get("CUTOFF_FREQ_HZ", 10.0))
    alpha = compute_alpha(sample_rate_hz, cutoff_freq_hz)

    ctrl = cfg.get("controller_params", {})
    theta_range = float(ctrl.get("THETA_RANGE_RAD", 1.5))
    omega_range = float(ctrl.get("OMEGA_RANGE_RAD_S", math.pi))
    gyro_full_scale_dps = float(ctrl.get("GYRO_FULL_SCALE_DPS", 250.0))

    filtered_omega = 0.0
    thetas_norm, omegas_norm = [], []

    buf = bytearray(6)
    for _ in range(int(samples)):
        # Read raw bytes from mock
        # NOTE: In real code, you would use mock.readfrom_into(0x00, buf)
#        mock.readfrom_into(0x00, buf)
        buf = mock.imu_read()
        raw_y = int.from_bytes(buf[0:2], "little", signed=True)
        raw_x = int.from_bytes(buf[2:4], "little", signed=True)
        raw_omega = int.from_bytes(buf[4:6], "little", signed=True)

        theta_rads = math.atan2(raw_x, raw_y)
        theta_norm = theta_rads / theta_range

        omega_dps_instant = (raw_omega * gyro_full_scale_dps) / 32768.0
        omega_rps_instant = omega_dps_instant * (math.pi / 180.0)

        # 1st-order IIR
        filtered_omega += alpha * (omega_rps_instant - filtered_omega)
        omega_norm = filtered_omega / omega_range

        thetas_norm.append(theta_norm)
        omegas_norm.append(omega_norm)

    return np.array(thetas_norm), np.array(omegas_norm)


def main():
    parser = argparse.ArgumentParser(
        description="Plot omega membership functions with motor_cmd overlay (MockSPIBus-driven)"
    )
    parser.add_argument("--samples", type=int, default=600,
                        help="number of sim samples (default: 600)")
    parser.add_argument("--save", action="store_true",
                        help="save figure to plots/omega_motor_cmd.png")

    # Put ALL mode options in one mutually-exclusive group (prevents conflicts)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--omega-mode", choices=["constant", "noisy", "random", "slope", "none"],
                    help="override MOCK_SPI OMEGA_MODE for this run")
    mode.add_argument("--random",   action="store_true",
                    help="shorthand for --omega-mode random")
    mode.add_argument("--noisy",    action="store_true",
                    help="shorthand for --omega-mode noisy")
    mode.add_argument("--constant", action="store_true",
                    help="shorthand for --omega-mode constant")
    mode.add_argument("--slope", action="store_true",
                    help="shorthand for --omega-mode constant")
    mode.add_argument("--none",     action="store_true",
                    help="shorthand for --omega-mode none")

    args = parser.parse_args()

    # Compute the override (shorthands take precedence)
    override = args.omega_mode
    if args.random:   override = "random"
    elif args.noisy:  override = "noisy"
    elif args.constant: override = "constant"
    elif args.slope: override = "slope"
    elif args.none:   override = "none"


    # Repo root = parent of utils/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.normpath(os.path.join(script_dir, ".."))

    # Load config and controller
    cfg = load_config(repo_root)

    # Build FLC
    from flc.controller import FLCController
    flc = FLCController(cfg)

    # Collect θ, ω from MockSPIBus (reflecting current config/mock behavior)
    theta_norm, omega_norm = collect_mock_samples(cfg, args.samples, override)

    # Compute motor_cmd for each sample at the measured (θ, ω)
    motor_cmds = [flc.calculate_motor_cmd(float(t), float(w)) for t, w in zip(theta_norm, omega_norm)]

    # Plot
    fig, ax1 = plt.subplots(figsize=(12, 6))
    mf_map = membership_points_from_config(cfg)
    xmin, xmax = plot_memberships(ax1, mf_map)

    # Overlay motor_cmd vs. omega
    ax2 = ax1.twinx()
    ax2.scatter(
        omega_norm,
        motor_cmds,
        s=14,
        edgecolors="black",
        linewidths=0.6,
        label="motor_cmd (MockSPIBus samples)",
        zorder=10,
    )
    ax2.set_ylabel("Motor Command")
    ax2.set_ylim(-1.0, 1.0)
    ax2.legend(loc="upper right")
    ax2.tick_params(axis="y")

    plt.title("Omega Memberships (left) with motor_cmd overlay (right) — MockSPIBus-driven")
    plt.tight_layout()

    if args.save:
        out_dir = os.path.join(repo_root, "plots")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, "omega_motor_cmd.png")
        plt.savefig(out_path, dpi=150)
        print(f"Saved: {os.path.relpath(out_path, repo_root)}")

    plt.show()


if __name__ == "__main__":
    main()

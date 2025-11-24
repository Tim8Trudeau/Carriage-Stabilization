"""
plot_flc_alignment.py
======================

Visualize the FLC (Sugeno) membership functions aligned to their
corresponding physical input ranges (theta in radians, omega in rad/s).

This tool is used to verify that the the triangular and trapezoidal
fuzzy membership functions defined in `config/flc_config.toml`
are properly scaled relative to the physical values coming from the
CarriageSimulator or IMU_Driver.
It helps ensure that membership functions cover the expected operating range and that
the fuzzy controller will respond correctly to real-world data.
Plot fuzzy membership functions aligned to physical theta/omega ranges.



It reads:
    - membership_functions → from Fuzzifier
    - scaling.THETA_MAX_RAD and scaling.OMEGA_MAX_RAD_S

Usage:
    python -m utils.plot_flc_alignment

The script plots:

    1. Theta membership functions vs. physical angle (rad)
    2. Omega membership functions vs. physical angular velocity (rad/s)

Scaling parameters are read from:

    [scaling]
    THETA_MAX_RAD     = ...
    OMEGA_MAX_RAD_S   = ...

These are used to convert physical units into normalized values
in [-1, +1], which are then passed to each membership function.

When to Use
-----------
Run this script whenever:

    • You adjust THETA_MAX_RAD or OMEGA_MAX_RAD_S
    • You change membership function definitions
    • You want to confirm alignment between fuzzy inputs and real dynamics
    • You tune the FLC for new mechanical configurations or motors

How to Run
----------
From your project root:

    python -m utils.plot_flc_alignment

Or directly:

    python utils/plot_flc_alignment.py

Dependencies
------------
Requires:
    - numpy
    - matplotlib
    - flc.controller.FLCController
    - config/flc_config.toml

Outputs
-------
Two interactive Matplotlib figures showing the fuzzy membership curves
mapped onto the *actual physical range* of theta and omega.

Interpretation Guide
--------------------
• The leftmost and rightmost MF curves should reach μ ≈ 1 at the physical extremes.
• Overlapping regions should appear centered around angles/velocities meaningful to the controller.
• If curves appear compressed or stretched unnaturally, adjust the scaling block:
      THETA_MAX_RAD or OMEGA_MAX_RAD_S
in `flc_config.toml`.

This tool is essential for validating that the fuzzy logic controller
perceives physical state values correctly and responds appropriately.
"""

import numpy as np
import matplotlib.pyplot as plt
import tomllib
from flc.controller import FLCController


def plot_flc_alignment(flc_config_path="config/flc_config.toml"):
    # Load FLC config TOML
    with open(flc_config_path, "rb") as f:
        cfg = tomllib.load(f)

    # Build FLC
    flc = FLCController(cfg)

    # Scaling values
    scale_cfg = cfg.get("scaling", {})
    theta_max = float(scale_cfg.get("THETA_MAX_RAD", 1.0))
    omega_max = float(scale_cfg.get("OMEGA_MAX_RAD_S", 1.0))

    # Access membership functions correctly
    theta_mfs = flc.fuzzifier.membership_functions.get("theta", {})
    omega_mfs = flc.fuzzifier.membership_functions.get("omega", {})

    # Prepare ranges
    theta_vals = np.linspace(-theta_max, theta_max, 400)
    omega_vals = np.linspace(-omega_max, omega_max, 400)

    # Normalize for fuzzy sets
    theta_norm = theta_vals / theta_max
    omega_norm = omega_vals / omega_max

    # -------------- Plot Theta Membership Functions --------------
    plt.figure(figsize=(10, 4))
    for name, params in theta_mfs.items():
        mf_vals = []
        for x in theta_norm:
            if len(params) == 3:
                # triangle
                a, b, c = params
                if x <= a or x >= c:
                    y = 0.0
                elif a < x <= b:
                    y = (x - a) / (b - a) if b != a else 1.0
                else:
                    y = (c - x) / (c - b) if c != b else 1.0
                mf_vals.append(y)
            elif len(params) == 4:
                # trapezoid
                a, b, c, d = params
                if x <= a or x >= d:
                    y = 0.0
                elif b <= x <= c:
                    y = 1.0
                elif a < x < b:
                    y = (x - a) / (b - a)
                else:
                    y = (d - x) / (d - c)
                mf_vals.append(y)
        plt.plot(theta_vals, mf_vals, label=name)

    plt.title("Theta Membership Functions (physical range)")
    plt.xlabel("theta (rad)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    # -------------- Plot Omega Membership Functions --------------
    plt.figure(figsize=(10, 4))
    for name, params in omega_mfs.items():
        mf_vals = []
        for x in omega_norm:
            if len(params) == 3:
                a, b, c = params
                if x <= a or x >= c:
                    y = 0.0
                elif a < x <= b:
                    y = (x - a) / (b - a)
                else:
                    y = (c - x) / (c - b)
                mf_vals.append(y)
            elif len(params) == 4:
                a, b, c, d = params
                if x <= a or x >= d:
                    y = 0.0
                elif b <= x <= c:
                    y = 1.0
                elif a < x < b:
                    y = (x - a) / (b - a)
                else:
                    y = (d - x) / (d - c)
                mf_vals.append(y)
        plt.plot(omega_vals, mf_vals, label=name)

    plt.title("Omega Membership Functions (physical range)")
    plt.xlabel("omega (rad/s)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.show()


if __name__ == "__main__":
    plot_flc_alignment()

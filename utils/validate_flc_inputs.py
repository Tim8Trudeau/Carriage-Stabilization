"""
validate_flc_inputs.py
=======================

Runtime validation tool for ensuring that physical simulation or IMU data
(theta and omega logs) remain within the normalized input ranges expected
by the FLC scaling block.

This script checks whether your logged simulator values exceed:

    THETA_MAX_RAD
    OMEGA_MAX_RAD_S

as defined in:

    config/flc_config.toml  →  [scaling] block

Its primary purpose is to verify that your chosen scaling parameters
correctly cover the full operating range of the inverted pendulum. If the
real system produces values that exceed the scaling thresholds, the FLC
will saturate, rules will fire incorrectly, and stability may degrade.

The validator computes:

    • Maximum |theta| observed
    • Maximum |omega| observed
    • Number of samples exceeding limits
    • Recommendations for scaling adjustments

When to Use
-----------
Run this after simulation or real IMU-based runs:

    • to validate scaling blocks
    • to tune THETA_MAX_RAD and OMEGA_MAX_RAD_S
    • after modifying mechanical or electrical parameters
    • after changing controller behavior

How to Run
----------
Inside a simulation script:

    from utils.validate_flc_inputs import validate_flc_inputs
    validate_flc_inputs(sim.log_theta, sim.log_omega)

Or from an interactive shell:

    python -m utils.validate_flc_inputs

Inputs
------
theta_log : list[float]
    Logged theta values in radians from the simulator or IMU.

omega_log : list[float]
    Logged omega values in rad/s.

flc_config_path : str, optional
    Path to FLC TOML config (default: "config/flc_config.toml").

Output
------
Prints a human-readable diagnostic report:

    === FLC Input Range Validation ===
    THETA_MAX_RAD = 0.50
    OMEGA_MAX_RAD_S = 4.00

    Theta max observed: 0.412
    Omega max observed: 3.582

    Theta samples exceeding limit: 0/500
    Omega samples exceeding limit: 2/500

With recommendations if scaling is insufficient.

Recommendations
---------------
If exceeding limits is frequent:

    • Increase THETA_MAX_RAD or OMEGA_MAX_RAD_S by ~20–50%.
    • Or improve controller authority (raise motor torque / gear ratio).
    • Or reduce simulation disturbance torque (tau_ext).

This script ensures that your FLC input scaling corresponds accurately to
the true physical dynamics of the system, enabling safe and reliable
fuzzy control both in simulation and on hardware.
"""
import tomllib
import statistics

def validate_flc_inputs(theta_log, omega_log, flc_config_path="config/flc_config.toml"):
    with open(flc_config_path, "rb") as f:
        cfg = tomllib.load(f)

    scale = cfg["scaling"]
    theta_max = scale["THETA_MAX_RAD"]
    omega_max = scale["OMEGA_MAX_RAD_S"]

    # Compute statistics
    t_abs = [abs(x) for x in theta_log]
    w_abs = [abs(x) for x in omega_log]

    theta_over = sum(1 for x in t_abs if x > theta_max)
    omega_over = sum(1 for x in w_abs if x > omega_max)

    print("=== FLC Input Range Validation ===")
    print(f"THETA_MAX_RAD = {theta_max}")
    print(f"OMEGA_MAX_RAD_S = {omega_max}")

    print(f"\nTheta max observed: {max(t_abs):.3f}")
    print(f"Omega max observed: {max(w_abs):.3f}")

    print(f"\nTheta samples exceeding limit: {theta_over}/{len(theta_log)}")
    print(f"Omega samples exceeding limit: {omega_over}/{len(omega_log)}")

    if theta_over == 0 and omega_over == 0:
        print("\n✔ FLC scaling is appropriate.")
    else:
        print("\n⚠ Consider increasing THETA_MAX_RAD or OMEGA_MAX_RAD_S.")
        print("  Your system is exceeding normalization bounds.")


# Example usage:
# from validate_flc_inputs import validate_flc_inputs
# validate_flc_inputs(sim.log_theta, sim.log_omega)

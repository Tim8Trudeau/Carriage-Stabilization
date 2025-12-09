import tomllib
import statistics
import re
import os


def _extract_from_logfile(path):
    """
    Extract theta and omega from any existing log file.
    Supports lines like:
        theta=..., omega=...
        norm_theta=..., norm_omega=...
    Returns (theta_list, omega_list).
    """
    theta = []
    omega = []

    if not os.path.exists(path):
        return theta, omega

    rx = re.compile(
        r"(?:theta|norm_theta)\s*=\s*([-+0-9.eE]+).*?"
        r"(?:omega|norm_omega)\s*=\s*([-+0-9.eE]+)"
    )

    with open(path, "r") as f:
        for line in f:
            m = rx.search(line)
            if m:
                try:
                    t = float(m.group(1))
                    w = float(m.group(2))
                    theta.append(t)
                    omega.append(w)
                except ValueError:
                    pass

    return theta, omega


def validate_flc_inputs(theta_log=None,
                        omega_log=None,
                        flc_config_path="config/flc_config.toml",
                        log_paths=None):
    """
    theta_log, omega_log:
        Optional direct lists (e.g., from simulation)

    log_paths:
        Optional list of log files to extract theta/omega from.
        If None, will search:
            - imu.log
            - main.log
            - sim.log
    """

    # If no lists provided, try extracting from log files
    if (theta_log is None or omega_log is None):
        if log_paths is None:
            log_paths = ["imu.log", "main.log", "sim.log"]

        theta = []
        omega = []
        for p in log_paths:
            t, w = _extract_from_logfile(p)
            theta.extend(t)
            omega.extend(w)

        if not theta or not omega:
            raise RuntimeError(
                "No theta/omega data found in logs. "
                "Provide lists or supply log_paths=[...]."
            )

        theta_log = theta
        omega_log = omega

    # Continue with existing logic
    with open(flc_config_path, "rb") as f:
        cfg = tomllib.load(f)

    scale = cfg["scaling"]
    theta_max = scale["THETA_MAX_RAD"]
    omega_max = scale["OMEGA_MAX_RAD_S"]

    t_abs = [abs(x) for x in theta_log]
    w_abs = [abs(x) for x in omega_log]

    theta_over = sum(1 for x in t_abs if x > theta_max)
    omega_over = sum(1 for x in w_abs if x > omega_max)

    print("=== FLC Input Range Validation ===")
    print(f"THETA_MAX_RAD = {theta_max}")
    print(f"OMEGA_MAX_RAD_S = {omega_max}\n")

    print(f"Theta max observed: {max(t_abs):.3f}")
    print(f"Omega  max observed: {max(w_abs):.3f}\n")

    print(f"Theta samples exceeding limit: {theta_over}/{len(theta_log)}")
    print(f"Omega  samples exceeding limit: {omega_over}/{len(omega_log)}\n")

    if theta_over == 0 and omega_over == 0:
        print("✔ FLC scaling is appropriate.")
    else:
        print("⚠ FLC input scaling too small.")
        print("  Increase THETA_MAX_RAD or OMEGA_MAX_RAD_S.")


if __name__ == "__main__":
    import argparse
    import os

    parser = argparse.ArgumentParser(
        description="Validate FLC input scaling using logs or explicit theta/omega files."
    )

    parser.add_argument(
        "--logs",
        nargs="*",
        default=None,
        help="Log files to scan (default: auto-detect: imu.log, main.log, sim.log)."
    )

    parser.add_argument(
        "--config",
        default="config/flc_config.toml",
        help="Path to flc_config.toml (default: config/flc_config.toml)"
    )

    args = parser.parse_args()

    # Determine log paths
    if args.logs is None or len(args.logs) == 0:
        # Auto-detection
        log_dir = "/logs"
        default_candidates = [
        os.path.join(log_dir, "imu.log"),
        os.path.join(log_dir, "main.log"),
        os.path.join(log_dir, "sim.log"),
        ]
        log_paths = [p for p in default_candidates if os.path.exists(p)]

        if not log_paths:
            print("No log files found. Looked for imu.log, main.log, sim.log.")
            print("Use --logs to specify paths.")
            exit(1)

        print(f"Using detected logs: {log_paths}")
    else:
        # User-supplied logs
        log_paths = args.logs
        print(f"Using logs: {log_paths}")

    # Call validator
    validate_flc_inputs(
        theta_log=None,
        omega_log=None,
        flc_config_path=args.config,
        log_paths=log_paths
    )

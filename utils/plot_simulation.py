from __future__ import annotations
from utils.logger import setup_logging
import math
import logging, os
import tomllib
import matplotlib.pyplot as plt

from test.mocks.mock_spi import MockSPIBus, set_motor_cmd
from utils.logger import setup_logging, set_loop_index

# Import your actual FLC controller here:
try:
    from controller import FLCController
except Exception:
    from flc.controller import FLCController  # type: ignore


def load_sim_config(path="config/sim_config.toml") -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)

def load_flc_config(path="config/flc_config.toml") -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def _setup_loop_logger():
    os.makedirs("logs", exist_ok=True)
    log = logging.getLogger("simloop")
    if not log.handlers:
        fh = logging.FileHandler("logs/simloop.log", mode="w", encoding="utf-8")
        fmt = logging.Formatter("%(relativeCreated)d | %(levelname)s | %(name)s | %(message)s")
        fh.setFormatter(fmt)
        log.addHandler(fh)
        log.setLevel(logging.INFO)
    return log


def main():
    setup_logging()
    sim_cfg = load_sim_config()
    flc_cfg = load_flc_config()
    log_sim = _setup_loop_logger()
    logs = setup_logging()
    timing = sim_cfg["simulation"]["timing"]
    imu = sim_cfg["simulation"]["imu_model"]

    hz = float(timing.get("SAMPLE_RATE_HZ", 50.0))
    dt = 1.0 / hz
    steps = int(float(timing.get("DURATION_S", 10.0)) * hz)

    gyro_fs = float(imu.get("GYRO_FS_RAD_S", 4.363))
    raw_fs = int(imu.get("ACCEL_RAW_FS", 16384))

    # FLC normalization ranges (kept in flc_config.toml)
    ctrl_params = flc_cfg.get("controller_params", {})
    theta_range_rad = float(ctrl_params.get("THETA_RANGE_RAD", math.pi))
    # omega is normalized by gyro_fs_rad_s (unit-consistent with simulator)

    # Build controller and SPI (sim)
    flc = FLCController(flc_cfg)
    spi = MockSPIBus(controller_params=ctrl_params, sim_config_path="config/sim_config.toml")

    # logging buffers
    t = [0.0] * steps
    theta = [0.0] * steps
    omega = [0.0] * steps
    u_cmd = [0.0] * steps
    theta_sim = [0.0] * steps
    theta_imu = [0.0] * steps

    # loop
    time_acc = 0.0
    for i in range(steps):
        set_loop_index(i)
        # IMU read (raw)
        buf = spi.imu_read()
        x_raw = int.from_bytes(buf[0:2], "little", signed=True)
        y_raw = int.from_bytes(buf[2:4], "little", signed=True)
        omega_raw = int.from_bytes(buf[4:6], "little", signed=True)

        # Convert to θ, ω (match runtime conventions)
        theta_rad = math.atan2(x_raw, y_raw)  # consistent with CW positive
        omega_rad_s = (omega_raw / raw_fs) * gyro_fs

        # Normalize for FLC
        theta_norm = clamp(theta_rad / theta_range_rad, -1.0, 1.0)
        omega_norm = clamp(omega_rad_s / gyro_fs, -1.0, 1.0)

        # Controller output
        motor_cmd = flc.calculate_motor_cmd(theta_norm, omega_norm)
        set_motor_cmd(motor_cmd)

        # NEW: pull simulator θ from the mock and log both
        th_sim = getattr(spi, "get_sim_theta", lambda: float("nan"))()
        log_sim.info(
            "t= %.3f | th_imu= %.3f rad | th_norm= %.3f rad | om_imu= %.3f rad/s | om_norm= %.3f rad/s | cmd= %.3f",
            time_acc, theta_rad, theta_norm, omega_rad_s, omega_norm, motor_cmd
        )

        time_acc += dt

        # Store per-step values
        t[i] = time_acc
        theta_imu[i] = theta_rad
        omega[i] = omega_rad_s
        u_cmd[i] = motor_cmd

        # Grab simulator theta from the SPI wrapper each step
        get_sim_theta = getattr(spi, "get_sim_theta", None)
        theta_sim[i] = float(get_sim_theta()) if callable(get_sim_theta) else float("nan")


    # --- plot (shared time axis) ---
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.set_title("Carriage Simulation — θ, ω, motor_cmd vs time")
    ax1.set_xlabel("Time (s)")
    ax1.grid(True, which="both", alpha=0.25)
    ax1.axhline(0.0, linestyle="--", linewidth=0.8)  # left-axis zero line

    # Left y-axis: theta & omega
    lh0, = ax1.plot(t, theta_sim, linewidth=1.0, alpha=0.4, label="theta_sim (rad)")
    lh1, = ax1.plot(t, theta_imu, linewidth=1.5,            label="theta_imu (rad)")
    lh2, = ax1.plot(t, omega, label="omega (rad/s)")

    # Right y-axis: motor_cmd
    ax2 = ax1.twinx()
    ax2.set_ylabel("motor_cmd")
    ax2.set_ylim(-1.5, 1.5)

    # Plot motor_cmd
    rh1, = ax2.plot(t, u_cmd, color="red", label="motor_cmd (−1..+1)")

    # Add zero line to right axis too
    ax2.axhline(0.0, color="red", linestyle="--", linewidth=0.8)

    # Build a joint legend
    lines = [lh0, lh1, lh2, rh1]
    labels = [str(l.get_label()) for l in lines]
    ax1.legend(lines, labels, loc="best")
    plt.tight_layout()

    # Save and display
    out_dir = "plots"
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "sim_output.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.show()

if __name__ == "__main__":
    main()

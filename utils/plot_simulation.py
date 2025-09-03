# plot_simulation.py
from __future__ import annotations
import math
import tomllib
import matplotlib.pyplot as plt

from test.mocks.mock_spi import MockSPIBus, set_motor_cmd

# Import your actual FLC controller here:
# Adjust the import to match your repo layout, e.g.:
# from controller import FLCController
try:
    from controller import FLCController
except Exception:
    # Fallback path (adjust as needed)
    from flc.controller import FLCController  # type: ignore

def load_sim_config(path="config/sim_config.toml") -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)

def load_flc_config(path="config/flc_config.toml") -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def main():
    sim_cfg = load_sim_config()
    flc_cfg = load_flc_config()

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

    # loop
    time_acc = 0.0
    for i in range(steps):
        # IMU read (raw)
        buf = spi.imu_read()
        x_raw = int.from_bytes(buf[0:2], "little", signed=True)
        y_raw = int.from_bytes(buf[2:4], "little", signed=True)
        omega_raw = int.from_bytes(buf[4:6], "little", signed=True)

        # Convert to θ, ω (match runtime conventions)
        theta_rad = math.atan2(x_raw, y_raw)  # consistent with prior usage
        omega_rad_s = (omega_raw / raw_fs) * gyro_fs

        # Normalize for FLC
        theta_norm = clamp(theta_rad / theta_range_rad, -1.0, 1.0)
        omega_norm = clamp(omega_rad_s / gyro_fs, -1.0, 1.0)

        # Controller output
        motor_cmd = flc.calculate_motor_cmd(theta_norm, omega_norm)
        set_motor_cmd(motor_cmd)

        # record
        t[i] = time_acc
        theta[i] = theta_rad
        omega[i] = omega_rad_s
        u_cmd[i] = motor_cmd
        time_acc += dt

    # --- plot (shared time axis) ---
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.set_title("Carriage Simulation — θ, ω, motor_cmd vs time")
    ax1.set_xlabel("Time (s)")
    ax1.grid(True, which="both", alpha=0.25)
    ax1.axhline(0.0, linestyle="--", linewidth=0.8)

    ax1.plot(t, theta, label="theta (rad)")
    ax1.plot(t, omega, label="omega (rad/s)")
    ax1.plot(t, u_cmd, label="motor_cmd (–1..+1)")
    ax1.legend(loc="best")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

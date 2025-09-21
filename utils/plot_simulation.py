# utils/plot_simulation.py
from __future__ import annotations
import math, logging, os, tomllib, matplotlib.pyplot as plt
from utils.logger import setup_logging, set_loop_index
from flc.controller import FLCController
from hardware.imu_driver import IMU_Driver

def load_sim_config(path="config/sim_config.toml") -> dict:
    with open(path, "rb") as f: return tomllib.load(f)
def load_flc_config(path="config/flc_config.toml") -> dict:
    with open(path, "rb") as f: return tomllib.load(f)
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def _setup_loop_logger():
    os.makedirs("logs", exist_ok=True)
    log = logging.getLogger("simloop")
    if not log.handlers:
        fh = logging.FileHandler("logs/simloop.log", mode="w", encoding="utf-8")
        fmt = logging.Formatter("%(relativeCreated)d | %(levelname)s | %(name)s | %(message)s")
        fh.setFormatter(fmt); log.addHandler(fh); log.setLevel(logging.INFO)
    return log

def main():
    setup_logging()
    sim_cfg = load_sim_config()
    flc_cfg = load_flc_config()
    log_sim = _setup_loop_logger()

    timing  = sim_cfg["simulation"]["timing"]
    imu_cfg = sim_cfg["simulation"]["imu_model"]
    init    = sim_cfg["simulation"].get("initial_conditions", {})

    hz = float(timing.get("SAMPLE_RATE_HZ", 50.0)); dt = 1.0 / hz
    steps = int(float(timing.get("DURATION_S", 10.0)) * hz)
    gyro_fs = float(imu_cfg.get("GYRO_FS_RAD_S", 4.363))
    raw_fs  = int(imu_cfg.get("ACCEL_RAW_FS", 16384))

    ctrl_params = dict(flc_cfg.get("controller_params", {}))
    theta_range_rad = float(ctrl_params.get("THETA_RANGE_RAD", math.pi))

    # Tell hardware stack we're simulating
    ctrl_params["SIM_MODE"] = True

    flc = FLCController(flc_cfg)
    imu = IMU_Driver(
        iir_params={
            "SAMPLE_RATE_HZ": hz,
            "ACCEL_CUTOFF_HZ": float(flc_cfg.get("iir_params", {}).get("ACCEL_CUTOFF_HZ", 4.0)),
            "CUTOFF_FREQ_HZ": float(flc_cfg.get("iir_params", {}).get("CUTOFF_FREQ_HZ", 5.0)),
        },
        controller_params={
            **ctrl_params,
            "ACCEL_RAW_FS": raw_fs,
            "GYRO_FULL_SCALE_RADS_S": gyro_fs,
            "THETA_RANGE_RAD": theta_range_rad,
        },
    )

    log_sim.info("Simulation ICs: THETA_INITIAL_RAD=%s OMEGA_INITIAL_RAD_S=%s",
                 init.get("THETA_INITIAL_RAD"), init.get("OMEGA_INITIAL_RAD_S"))

    # Buffers
    t = [0.0] * steps; theta_sim = [float("nan")] * steps
    theta_imu = [0.0] * steps; omega = [0.0] * steps; u_cmd = [0.0] * steps

    # Helpers using the driver pass-throughs (no mock imports)
    def _set_motor_cmd(cmd: float) -> None:
        f = getattr(getattr(imu, "_dev", None), "set_motor_cmd", None)
        if callable(f): f(cmd)
    def _get_sim_theta():
        f = getattr(getattr(imu, "_dev", None), "get_sim_theta", None)
        try: return float(f()) if callable(f) else float("nan")
        except Exception: return float("nan")

    # Loop
    time_acc = 0.0
    for i in range(steps):
        set_loop_index(i)

        theta_norm, omega_norm = imu.read_normalized()
        theta_rad = clamp(theta_norm, -1.0, 1.0) * theta_range_rad
        omega_rad_s = clamp(omega_norm, -1.0, 1.0) * gyro_fs

        motor_cmd = flc.calculate_motor_cmd(theta_norm, omega_norm)
        _set_motor_cmd(motor_cmd)

        th_sim = _get_sim_theta()
        log_sim.info(
            "t= %.3f | th_imu= %.3f rad | th_norm= %.3f | om_imu= %.3f rad/s | om_norm= %.3f | cmd= %.3f",
            time_acc, theta_rad, theta_norm, omega_rad_s, omega_norm, motor_cmd
        )

        time_acc += dt
        t[i] = time_acc; theta_sim[i] = th_sim
        theta_imu[i] = theta_rad; omega[i] = omega_rad_s; u_cmd[i] = motor_cmd

    # Plot
    import matplotlib.pyplot as plt
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.set_title("Carriage Simulation — θ, ω, motor_cmd vs time")
    ax1.set_xlabel("Time (s)"); ax1.grid(True, which="both", alpha=0.25); ax1.axhline(0.0, linestyle="--", linewidth=0.8)
    lh0, = ax1.plot(t, theta_sim, linewidth=1.0, alpha=0.4, label="theta_sim (rad)")
    lh1, = ax1.plot(t, theta_imu, linewidth=1.5,            label="theta_imu (rad)")
    lh2, = ax1.plot(t, omega, label="omega (rad/s)")
    ax2 = ax1.twinx(); ax2.set_ylabel("motor_cmd"); ax2.set_ylim(-1.5, 1.5)
    rh1, = ax2.plot(t, u_cmd, label="motor_cmd (−1..+1)")
    ax1.legend([lh0, lh1, lh2, rh1], [l.get_label() for l in [lh0, lh1, lh2, rh1]], loc="best")
    plt.tight_layout(); os.makedirs("plots", exist_ok=True)
    plt.savefig(os.path.join("plots", "sim_output.png"), dpi=150, bbox_inches="tight"); plt.show()

if __name__ == "__main__":
    main()

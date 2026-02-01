# main.py
"""
Hardware-only entrypoint for the Carriage Stabilization project.

Design constraints:
- Hardware code runs ONLY on the Raspberry Pi.
- Simulation runs are executed via `python -m simulation.run_simulation`.
- This file must be safe to import on Windows for unit testing.

This file intentionally does NOT support simulation execution.
"""
from __future__ import annotations

import logging
import os
import signal
import threading
import time
import tomllib
from typing import Dict, Any, Tuple

from utils.logger import setup_logging
from utils.profiler import CodeProfiler
from flc.controller import FLCController

shutdown = threading.Event()


def _on_signal(_sig, _frm):
    shutdown.set()


signal.signal(signal.SIGINT, _on_signal)
if os.name != "nt":
    signal.signal(signal.SIGTERM, _on_signal)


def _load_toml(path: str, *, optional: bool = False) -> Dict[str, Any]:
    try:
        with open(path, "rb") as f:
            return tomllib.load(f)
    except FileNotFoundError:
        if optional:
            return {}
        raise


def load_config() -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    cfg_dir = os.path.join(os.path.dirname(__file__), "config")

    flc_cfg = _load_toml(os.path.join(cfg_dir, "flc_config.toml"))
    imu_cfg = _load_toml(os.path.join(cfg_dir, "imu_config.toml"), optional=True)

    imu_controller_params = dict(imu_cfg.get("controller_params", {}) or {})
    iir_params = dict(imu_cfg.get("iir_params", {}) or {})

    if not iir_params:
        legacy = flc_cfg.get("iir_filter", {})
        if isinstance(legacy, dict):
            iir_params = dict(legacy)

    if not imu_controller_params:
        imu_controller_params = flc_cfg

    return flc_cfg, imu_controller_params, iir_params




def make_stiction_booster(flc_cfg: Dict[str, Any]):
    """Return a function that applies a stiction/low-speed torque boost.

    Uses normalized inputs (theta_n, omega_n) and returns a clamped motor command.

    Configuration is read from [stiction_boost] in flc_config.toml.
    """
    sb = dict(flc_cfg.get("stiction_boost", {}) or {})
    enabled = bool(sb.get("ENABLED", False))
    if not enabled:
        def _passthrough(theta_n: float, omega_n: float, u_flc: float, dt: float) -> float:
            return max(-1.0, min(1.0, float(u_flc)))
        return _passthrough

    scaling = dict(flc_cfg.get("scaling", {}) or {})
    theta_max_rad = float(scaling.get("THETA_MAX_RAD", 1.0))
    theta_on_deg = float(sb.get("THETA_ON_DEG", 7.0))
    theta_off_deg = float(sb.get("THETA_OFF_DEG", 6.0))
    theta_on_n = (math.radians(theta_on_deg) / theta_max_rad) if theta_max_rad > 0 else 0.0
    theta_off_n = (math.radians(theta_off_deg) / theta_max_rad) if theta_max_rad > 0 else 0.0

    omega_stuck = float(sb.get("OMEGA_STUCK_NORM", 0.012))
    hold_s = float(sb.get("HOLD_S", 0.20))
    ramp_per_s = float(sb.get("BOOST_RAMP_PER_S", 1.0))
    decay_per_s = float(sb.get("BOOST_DECAY_PER_S", 2.0))
    boost_max = float(sb.get("BOOST_MAX", 0.35))

    stuck_time_s = 0.0
    boost = 0.0
    latched = False

    def apply(theta_n: float, omega_n: float, u_flc: float, dt: float) -> float:
        nonlocal stuck_time_s, boost, latched

        th = abs(float(theta_n))
        om = abs(float(omega_n))

        if latched:
            theta_active = th >= theta_off_n
        else:
            theta_active = th >= theta_on_n

        omega_zero = om <= omega_stuck
        stuck_now = theta_active and omega_zero

        if stuck_now:
            stuck_time_s += max(0.0, float(dt))
            if stuck_time_s >= hold_s:
                latched = True
                boost = min(boost + ramp_per_s * dt, boost_max)
        else:
            stuck_time_s = 0.0
            latched = False
            boost = max(boost - decay_per_s * dt, 0.0)

        # push "downhill" (same sign as theta)
        u = float(u_flc) + (boost if theta_n > 0 else -boost)
        if u > 1.0:
            return 1.0
        if u < -1.0:
            return -1.0
        return u

    return apply


def main_control_loop() -> None:
    from hardware.imu_driver import IMU_Driver
    from hardware.pwm_driver import DualPWMController

    setup_logging()
    log = logging.getLogger("main")
    log.info("Starting hardware control loop")

    flc_cfg, imu_controller_params, iir_params = load_config()

    stiction = make_stiction_booster(flc_cfg)

    loop_hz = float(flc_cfg.get("LOOP_FREQ_HZ", 50.0))
    loop_period = 1.0 / max(loop_hz, 1e-6)

    imu = IMU_Driver(iir_params, imu_controller_params)
    time.sleep(0.5)

    flc = FLCController(flc_cfg)
    motor = DualPWMController(frequency=int(flc_cfg.get("PWM_FREQ_HZ", 250)))

    last_tick = None

    try:
        while not shutdown.is_set():
            now = time.perf_counter()
            dt_loop = loop_period if last_tick is None else (now - last_tick)
            last_tick = now
            t0 = now

            with CodeProfiler("Control Loop"):
                theta_n, omega_n = imu.read_normalized()
                u_flc = flc.calculate_motor_cmd(theta_n, omega_n)
                u_cmd = stiction(theta_n, omega_n, u_flc, dt_loop)
                motor.set_speed(u_cmd)

            dt = time.perf_counter() - t0
            sleep = loop_period - dt
            if sleep > 0:
                end = time.perf_counter() + sleep
                while not shutdown.is_set() and time.perf_counter() < end:
                    time.sleep(0.002)

    finally:
        try:
            motor.stop()
        except Exception:
            log.exception("motor.stop failed")


if __name__ == "__main__":
    if os.name == "nt":
        print("Hardware main cannot run on Windows. Use simulation instead.")
        raise SystemExit(0)

    if os.getenv("CS_TARGET_MODE", "0") == "1":
        logging.disable(logging.CRITICAL)

    main_control_loop()

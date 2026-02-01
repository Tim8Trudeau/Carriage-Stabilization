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


def main_control_loop() -> None:
    from hardware.imu_driver import IMU_Driver
    from hardware.pwm_driver import DualPWMController

    setup_logging()
    log = logging.getLogger("main")
    log.info("Starting hardware control loop")

    flc_cfg, imu_controller_params, iir_params = load_config()

    loop_hz = float(flc_cfg.get("LOOP_FREQ_HZ", 100.0))
    loop_period = 1.0 / max(loop_hz, 1e-6)

    #   Create FLC    #
    imu = IMU_Driver(iir_params, imu_controller_params)
    flc = FLCController(flc_cfg)
    motor = DualPWMController(frequency=int(flc_cfg.get("PWM_FREQ_HZ", 250)))

    try:
        while not shutdown.is_set():
            t0 = time.perf_counter()

            with CodeProfiler("Control Loop"):
                ############
                # FLC Loop #
                ############
                theta_n, omega_n = imu.read_normalized()
                motor.set_speed(flc.calculate_motor_cmd(theta_n, omega_n))

            dt = time.perf_counter() - t0
            sleep = loop_period - dt
            if sleep > 0:
                end = time.perf_counter() + sleep
                while not shutdown.is_set() and time.perf_counter() < end:
                    time.sleep(0.001)

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

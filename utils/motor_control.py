#!/usr/bin/env python3
"""
Auto-sweep PWM for Raspberry Pi using DualPWMController.

Behavior:
  Starts at 0.0, increments by +step every second up to +1.0,
  then decrements by -step every second down to -1.0,
  and repeats forever.

Flags:
  --step <float>   Step size per second (0 < step <= 1). Default: 0.10
  --freq <int>     PWM frequency in Hz. Default: 200
"""

import os
import sys
import time
import signal
import subprocess
import logging
import argparse

from utils.logger import setup_logging
from hardware.pwm_driver import DualPWMController  # uses GPIO 12/13, default 200 Hz

LOG = logging.getLogger("pwm_auto_sweep")


def _pigpiod_running() -> bool:
    try:
        out = subprocess.run(["pgrep", "pigpiod"], capture_output=True, text=True, check=False)
        return out.returncode == 0
    except Exception:
        return False


def _start_pigpiod():
    if os.name == "nt":
        LOG.info("Windows run: skipping pigpiod; mock pigpio will be used if available.")
        return
    if _pigpiod_running():
        LOG.info("pigpio daemon already running.")
        return
    LOG.info("Starting pigpio daemon (pigpiod)…")
    for cmd in (["sudo", "pigpiod"], ["pigpiod"], ["sudo", "systemctl", "start", "pigpiod"]):
        try:
            subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.3)
            if _pigpiod_running():
                LOG.info("pigpio daemon is running.")
                return
        except Exception:
            pass
    raise RuntimeError("Failed to start pigpio daemon. Try `sudo pigpiod` then re-run.")


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def main():
    setup_logging()
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=0.10, help="step size each second (0<step<=1); default 0.10")
    ap.add_argument("--freq", type=int, default=200, help="PWM frequency in Hz; default 200")
    args = ap.parse_args()

    step = args.step
    if not (0.0 < step <= 1.0):
        LOG.warning("Invalid --step %.3f; clamping to 0.10", step)
        step = 0.10
    freq = int(args.freq) if args.freq and args.freq > 0 else 200

    _start_pigpiod()

    ctrl = DualPWMController(frequency=freq)  # set_speed expects [-1.0, +1.0]
    speed = 0.0
    direction = +1  # +1 going up to +1.0, -1 going down to -1.0
    ctrl.set_speed(speed)
    LOG.info("Auto sweep started: step=%.2f, freq=%d Hz", step, freq)

    def graceful_exit(_sig=None, _frm=None):
        LOG.info("Exiting… stopping PWM.")
        try:
            ctrl.stop()
        finally:
            sys.exit(0)

    signal.signal(signal.SIGINT, graceful_exit)
    signal.signal(signal.SIGTERM, graceful_exit)

    # Main loop: tick once per second
    while True:
        # Advance
        speed = _clamp(round(speed + direction * step, 3), -1.0, 1.0)
        ctrl.set_speed(speed)
        LOG.info("speed=%.2f", speed)
        print(f"motor speed = {speed}")

        # Flip direction at endpoints
        if speed >= 1.0:
            direction = -1
        elif speed <= -1.0:
            direction = +1

        time.sleep(1.0)


if __name__ == "__main__":
    main()

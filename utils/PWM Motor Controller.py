#!/usr/bin/env python3
"""
Manual PWM drive for Raspberry Pi using the project's DualPWMController.

Keys:
  q -> increase command by +0.1
  z -> decrease command by -0.1
  0 -> stop (command = 0.0)
  x or Ctrl-C -> exit
"""

import sys
import time
import signal
import select
import subprocess
import termios
import tty
import logging

from utils.logger import setup_logging
from hardware.pwm_driver import DualPWMController  # GPIO18/19, default 50 Hz :contentReference[oaicite:1]{index=1}

LOG = logging.getLogger("pwm_manual")

def _pigpiod_running() -> bool:
    try:
        out = subprocess.run(["pgrep", "pigpiod"], capture_output=True, text=True, check=False)
        return out.returncode == 0
    except Exception:
        return False

def _start_pigpiod():
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

class _RawTTY:
    """Put stdin into cbreak mode so single keypresses are read without Enter."""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def main():
    # Project-standard logging (writes to logs/* per utils.logger setup)
    setup_logging()

    # Ensure pigpio daemon is available before creating the controller
    _start_pigpiod()

    # Default 50 Hz; set_speed expects [-1.0, +1.0] and maps to GPIO18/19. :contentReference[oaicite:2]{index=2} :contentReference[oaicite:3]{index=3}
    ctrl = DualPWMController()

    speed = 0.0
    ctrl.set_speed(speed)
    LOG.info("Ready. q:+10%%, z:-10%%, 0:stop, x/CTRL-C:exit | speed=%.1f", speed)

    def graceful_exit(_sig=None, _frm=None):
        LOG.info("Exiting… stopping PWM.")
        try:
            ctrl.stop()
        finally:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, graceful_exit)
    signal.signal(signal.SIGTERM, graceful_exit)

    with _RawTTY():
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.25)
            if r:
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                if ch.lower() == "q":
                    speed = _clamp(round(speed + 0.1, 3), -1.0, 1.0)
                    ctrl.set_speed(speed)
                    LOG.info("speed=%.1f", speed)
                elif ch.lower() == "z":
                    speed = _clamp(round(speed - 0.1, 3), -1.0, 1.0)
                    ctrl.set_speed(speed)
                    LOG.info("speed=%.1f", speed)
                elif ch == "0":
                    speed = 0.0
                    ctrl.set_speed(speed)
                    LOG.info("speed=%.1f (STOP)", speed)
                elif ch.lower() == "x":
                    graceful_exit()

if __name__ == "__main__":
    main()

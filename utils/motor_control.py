#!/usr/bin/env python3
"""
Manual PWM drive for Raspberry Pi using the project's DualPWMController.

Keys:
  q -> increase command by +0.1
  z -> decrease command by -0.1
  0 -> stop (command = 0.0)
  x -> exit
"""

import os
import sys
import time
import signal
import select
import subprocess
import logging
import argparse

from utils.logger import setup_logging
from hardware.pwm_driver import DualPWMController  # GPIO12/13, default 50 Hz

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
        print("pigpio daemon already running.")
        return
    LOG.info("Starting pigpio daemon (pigpiod)…")
    print("Starting pigpio daemon (pigpiod)…")
    # Try a few common commands to start pigpiod:
    for cmd in (["sudo", "pigpiod"], ["pigpiod"], ["sudo", "systemctl", "start", "pigpiod"]):
        LOG.info("pigpio daemon cmd %s", cmd)
        print("pigpio daemon cmd %s", cmd)

        try:
            subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.3)
            if _pigpiod_running():
                LOG.info("pigpio daemon is running.")
                print("pigpio daemon is running.")
                return
        except Exception:
            pass
    raise RuntimeError("Failed to start pigpio daemon. Try `sudo pigpiod` then re-run.")


# --- POSIX raw-tty helper (define first) -------------------------------------
class _RawTTY:
    """Put stdin into cbreak mode so single keypresses are read without Enter."""
    def __enter__(self):
        print("Entering raw TTY mode (press 'x' to exit)…")
        import termios, tty  # imported only on POSIX
        self._termios = termios
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)

    def read_key(self, timeout: float = 0.25):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None


# --- Cross-platform shim -----------------------------------------------------
if os.name != "nt":
    # On Linux/Pi/macOS: use the POSIX helper directly
    KeyReader = _RawTTY
else:
    # On Windows: compatible minimal reader using msvcrt
    import msvcrt
    class KeyReader:
        def __enter__(self): return self
        def __exit__(self, *exc): pass
        def read_key(self, timeout: float = 0.25):
            end = time.time() + timeout
            while time.time() < end:
                if msvcrt.kbhit():
                    return msvcrt.getwch()
                time.sleep(0.01)
            return None


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def main():
    setup_logging()

    # Only start pigpio daemon on Linux (Windows uses mock)
    if os.name != "nt":
        _start_pigpiod()
    else:
        LOG.info("Windows run: skipping pigpiod; mock pigpio will be used if available.")

    ctrl = DualPWMController()  # set_speed expects [-1.0, +1.0]
    speed = 0.0
    ctrl.set_speed(speed)
    LOG.info("Ready. q:+10%%, z:-10%%, 0:stop, x:exit | speed=%.1f", speed)

    def graceful_exit(_sig=None, _frm=None):
        LOG.info("Exiting… stopping PWM.")
        try:
            ctrl.stop()
        finally:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, graceful_exit)
    signal.signal(signal.SIGTERM, graceful_exit)

    with KeyReader() as keys:
        while True:
            ch = keys.read_key(0.25)
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

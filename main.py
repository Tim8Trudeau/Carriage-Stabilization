"""
Main entry point for the Carriage Stabilization application.

This script initializes all components of the Fuzzy Logic Controller system,
including the imu_sensor driver, the FLC itself, and the motor driver. It then
runs a continuous control loop at a fixed frequency, orchestrating the flow
of data from imu_sensor to motor to stabilize the carriage.
"""

import time
import tomllib
import logging
import os
import signal
import threading

from utils.logger import setup_logging
from utils.profiler import CodeProfiler
from hardware.imu_driver import IMU_Driver
from flc.controller import FLCController
from hardware.pwm_driver import DualPWMController

# -----------------------------------------------------------------------------
# Cross-platform shutdown handling:
# - SIGINT works on Windows and Linux (Ctrl-C).
# - SIGTERM is installed only on non-Windows (sent by `systemctl stop`).
# - SIGBREAK is tried on Windows consoles but safely ignored elsewhere.
# -----------------------------------------------------------------------------
shutdown = threading.Event()


def _on_signal(_sig, _frm):
    shutdown.set()


signal.signal(signal.SIGINT, _on_signal)  # Ctrl-C everywhere
try:
    signal.signal(signal.SIGBREAK, _on_signal)  # Windows console Break
except (AttributeError, OSError):
    pass
if os.name != "nt":
    signal.signal(signal.SIGTERM, _on_signal)  # systemd stop on Pi


def load_config():
    """
    Loads configuration from config/flc_config.toml located relative to this script.
    """
    cfg_path = os.path.join(os.path.dirname(__file__), "config", "flc_config.toml")
    with open(cfg_path, "rb") as f:
        return tomllib.load(f)


def main_control_loop():
    """
    Main control loop: read IMU -> compute FLC output -> command motor at fixed rate.
    """

    # Initialize logging
    setup_logging()
    main_log = logging.getLogger("main")
    main_log.info("Logging system initialized.")
    main_log.info("Application starting...")

    # Load configuration
    controller_params = load_config()
    main_log.info("Configuration file 'flc_config.toml' loaded.")

    # Read loop frequency (Hz) with a default of 50 Hz
    loop_hz = float(controller_params.get("LOOP_FREQ_HZ", 50.0))
    loop_period = 1.0 / loop_hz

    # Create components
    # IMU_Driver expects controller_params (pass cfg); its underlying
    # LSM6DS3TRDriver.__init__ now performs the device's ODR/BDU/IF_INC init.
    iir_filter = controller_params.get("iir_filter", {})
    imu_sensor = IMU_Driver(iir_filter, controller_params)
    time.sleep(0.1)  # Allow some time for the IMU to stabilize
    # Fuzzy logic controller
    flc = FLCController(controller_params)

    # Motor PWM controller (GPIO 12/13 by default; 200 Hz unless overridden in cfg)
    motor_freq = int(controller_params.get("PWM_FREQ_HZ", 200))
    motor = DualPWMController(frequency=motor_freq)

    main_log.info("All components initialized successfully.")
    main_log.info("Starting control loop at %.1f Hz (%.1f ms period)...", loop_hz, loop_period * 1000.0)

    # Track start time (optional: can be used for warm-up logic if ever needed)
    start_time = time.perf_counter()

    try:
        # Main loop — exit cleanly when 'shutdown' is set by a signal
        while not shutdown.is_set():
            loop_start_time = time.perf_counter()

            with CodeProfiler("Control Loop"):
                # Read normalized inputs from the IMU (theta_norm, omega_norm)
                norm_theta, norm_omega = imu_sensor.read_normalized()

                # Compute motor command via the fuzzy logic controller
                motor_cmd = flc.calculate_motor_cmd(norm_theta, norm_omega)

                # Send the command to the motor driver
                motor.set_speed(motor_cmd)

            # Maintain the loop period (sleep only the remainder of the tick)
            processing_time = time.perf_counter() - loop_start_time
            sleep_time = loop_period - processing_time
            if sleep_time > 0:
                # Sleep in small chunks so we respond quickly to shutdown
                end = time.perf_counter() + sleep_time
                while not shutdown.is_set() and time.perf_counter() < end:
                    time.sleep(0.002)

    except KeyboardInterrupt:
        main_log.info("Keyboard interrupt received. Shutting down.")
    except Exception as e:
        # Keep this critical log to match existing behavior and aid diagnosis
        main_log.critical(
            "An unhandled exception occurred in the main loop: %s", e, exc_info=True
        )
        # Exit non-zero only if you want systemd to auto-restart on failure.
        # sys.exit(1)
    finally:
        # Ensure the motor is stopped on exit
        main_log.info("Setting motor speed to 0 and shutting down.")
        try:
            motor.stop()
            main_log.info("Application finished.")
        except Exception:
            main_log.exception("motor.stop() failed")


if __name__ == "__main__":
    main_control_loop()

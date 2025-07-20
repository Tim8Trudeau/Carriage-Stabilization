"""
Main entry point for the Carriage Stabilization application.

This script initializes all components of the Fuzzy Logic Controller system,
including the sensor driver, the FLC itself, and the motor driver. It then
runs a continuous control loop at a fixed frequency, orchestrating the flow
of data from sensor to motor to stabilize the carriage.
"""

import time
import json
import logging

from utils.logger import setup_logging
from utils.profiler import CodeProfiler
from sensor.accel_driver import AccelDriver
from flc.controller import FLCController
from hardware.motor_driver import MotorDriver

# Setup logging first
setup_logging()
main_log = logging.getLogger('main')

def main_control_loop():
    """
    Executes the primary, continuous control loop of the application.
    """
    main_log.info("Application starting...")

    # --- 1. Load Configuration ---
    try:
        with open('config/flc_config.json', 'r') as f:
            config = json.load(f)
        main_log.info("Configuration file 'flc_config.json' loaded.")
    except FileNotFoundError:
        main_log.error("FATAL: Configuration file not found. Exiting.")
        return
    except json.JSONDecodeError:
        main_log.error("FATAL: Configuration file is not valid JSON. Exiting.")
        return

    # --- 2. Initialize Components ---
    target_hz = config['controller_params']['TARGET_HZ']
    loop_period = 1.0 / target_hz

    try:
        sensor = AccelDriver(config['iir_filter'], config['controller_params'])
        flc = FLCController(config)
        motor = MotorDriver()
        main_log.info("All components initialized successfully.")
    except Exception as e:
        main_log.error("FATAL: Failed to initialize components: %s", e, exc_info=True)
        return

    # --- 3. Run Control Loop ---
    main_log.info("Starting control loop at %.1f Hz (%.1f ms period)...",
                  target_hz, loop_period * 1000)
    
    try:
        while True:
            loop_start_time = time.perf_counter()

            with CodeProfiler("Control Loop"):
                # a. Read and process sensor data
                norm_theta, norm_omega = sensor.get_processed_inputs()

                # b. Calculate motor command with FLC
                motor_cmd = flc.calculate_motor_cmd(norm_theta, norm_omega)

                # c. Send command to motor
                motor.set_speed(motor_cmd)

            # d. Maintain control rate
            processing_time = time.perf_counter() - loop_start_time
            sleep_time = loop_period - processing_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                main_log.warning("Loop overrun: Processing time (%.2fms) exceeded period (%.2fms)",
                                 processing_time * 1000, loop_period * 1000)

    except KeyboardInterrupt:
        main_log.info("Keyboard interrupt received. Shutting down.")
    except Exception as e:
        main_log.critical("An unhandled exception occurred in the main loop: %s",
                          e, exc_info=True)
    finally:
        # Ensure the motor is stopped on exit
        main_log.info("Setting motor speed to 0.")
        motor.set_speed(0.0)
        main_log.info("Application finished.")

if __name__ == "__main__":
    main_control_loop()
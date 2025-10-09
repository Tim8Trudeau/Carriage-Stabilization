"""
Main entry point for the Carriage Stabilization application.

This script initializes all components of the Fuzzy Logic Controller system,
including the imu_sensor driver, the FLC itself, and the motor driver. It then
runs a continuous control loop at a fixed frequency, orchestrating the flow
of data from imu_sensor to motor to stabilize the carriage.
"""

import time, sys, logging
import tomllib
from utils.logger import setup_logging
from utils.profiler import CodeProfiler
from hardware.imu_driver import IMU_Driver
from flc.controller import FLCController
from hardware.pwm_driver import DualPWMController

# Setup logging
main_log = logging.getLogger("main")

def main_control_loop():
    """
    Executes the primary, continuous control loop of the application.
    """
    main_log.info("Application starting...")

    # --- 1. Load Configuration ---
    try:
        with open("config/flc_config.toml", "rb") as f:
            config = tomllib.load(f)
        main_log.info("Configuration file 'flc_config.toml' loaded.")
    except FileNotFoundError:
        main_log.error("FATAL: Configuration file not found. Exiting.")
        return
    except Exception as e:
        main_log.error(
            "FATAL: Configuration file is not valid toml. Exiting. Error: %s", e
        )
        return

    # --- 2. Initialize Components ---
    sample_hz = config["controller_params"]["SAMPLE_RATE_HZ"]
    loop_period = 1.0 / sample_hz

    try:
        iir_filter = config.get("iir_filter", {})
        controller_params = config.get("controller_params", {})
        # init the accel and gyro
        imu_sensor = IMU_Driver(iir_filter, controller_params)
        # get FLC configurations
        flc = FLCController(config)
        # init the motor PWM driver
        motor = DualPWMController()
        main_log.info("All components initialized successfully.")
    except Exception as e:
        main_log.error("FATAL: Failed to initialize components: %s", e, exc_info=True)
        return

    # --- 3. Run Control Loop ---
    main_log.info(
        "Starting control loop at %.1f Hz (%.1f ms period)...",
        sample_hz,
        loop_period * 1000,
        )

    try:
        while True:
            loop_start_time = time.perf_counter()

            with CodeProfiler("Control Loop"):
                # a. Read and process imu_sensor data
                norm_theta, norm_omega = imu_sensor.read_normalized()

                # b. Calculate motor command with FLC. Range +1 to -1
                motor_cmd = flc.calculate_motor_cmd(norm_theta, norm_omega)

                # c. Send PWM command to motor
                motor.set_speed(motor_cmd)

            # d. Maintain control rate
            processing_time = time.perf_counter() - loop_start_time
            sleep_time = loop_period - processing_time
            if sleep_time > 0:
                time.sleep(sleep_time)

            else:
                #pass
                main_log.warning(
                    "Loop overrun: Processing time (%.2fms) exceeded period (%.2fms)",
                    processing_time * 1000,
                    loop_period * 1000,
                )

    except KeyboardInterrupt:
        main_log.info("Keyboard interrupt received. Shutting down.")
    except Exception as e:
        main_log.critical(
            "An unhandled exception occurred in the main loop: %s", e, exc_info=True
        )
        # sysexit(1)  # non-zero so systemd can retry only on Pi
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

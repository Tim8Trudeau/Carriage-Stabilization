"""
Configures the application-wide logging system.

This module sets up multiple loggers for different parts of the application
(e.g., fuzzifier, rule_engine, defuzzifier) to write to separate, structured
log files. This aids in debugging and later analysis.
"""

import os
import logging
import logging.handlers


def setup_logging(log_level=logging.DEBUG):
    """
    Configures loggers for all FLC components.

    Creates a main application logger and specific loggers for FLC modules.
    Each logger writes to its own file in the 'logs' directory.

    Args:
        log_level (int): The logging level to set for all handlers (e.g.,
            logging.INFO, logging.DEBUG).
    """
    os.makedirs("logs", exist_ok=True)

    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03d | %(message)s",
        datefmt="%m-%d %H:%M:%S.%f"[:-3],  # Exclude year from date format,
    )

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    console_handler.setLevel(logging.INFO)

    # Define logger names used throughout the project
    loggers = [
        "main",
        "controller",
        "fuzzifier",
        "rule_engine",
        "defuzzifier",
        "spi",
        "imu",
        "motor",
        "profiler",
    ]

    for name in loggers:
        logger = logging.getLogger(name)
        logger.setLevel(log_level)

        # Clear handlers to prevent duplication during repeated setup
        if logger.hasHandlers():
            logger.handlers.clear()

        file_handler = logging.handlers.RotatingFileHandler(
            f"logs/{name}.log", maxBytes=1024 * 1024, backupCount=3, encoding="utf-8"
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        # Send main logger output to console too
        if name == "main":
            logger.addHandler(console_handler)

    logging.getLogger("main").info("Logging system initialized.")

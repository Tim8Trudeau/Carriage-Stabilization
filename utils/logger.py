"""
Configures the application-wide logging system.

This module sets up multiple loggers for different parts of the application
(e.g., fuzzifier, rule_engine, defuzzifier) to write to separate, structured
log files. This aids in debugging and later analysis.
"""
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
    import os
    if not os.path.exists('logs'):
        os.makedirs('logs')

    # Define a consistent format
    formatter = logging.Formatter(
        '%(asctime)s.%(msecs)03d|%(name)-12s|%(levelname)-8s| %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    # Setup console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    console_handler.setLevel(logging.INFO) # Keep console output clean

    # List of loggers to configure
    loggers = [
        'main', 'controller', 'fuzzifier', 'rule_engine', 'defuzzifier',
        'spi', 'imu', 'motor', 'profiler'
    ]

    for name in loggers:
        logger = logging.getLogger(name)
        logger.setLevel(log_level)
        
        # Prevent duplicate handlers if this function is called multiple times
        if logger.hasHandlers():
            logger.handlers.clear()

        # Add file handler for detailed logs
        file_handler = logging.handlers.RotatingFileHandler(
            f'logs/{name}.log', maxBytes=1024*1024, backupCount=3)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        # The main logger also gets console output
        if name == 'main':
            logger.addHandler(console_handler)

    logging.getLogger('main').info("Logging system initialized.")


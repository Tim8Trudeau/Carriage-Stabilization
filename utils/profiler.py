"""
A simple context manager for code profiling.

This utility measures the execution time of a block of code, which is useful
for verifying that the controller meets its real-time performance requirements.
"""
import time
import logging

profiler_log = logging.getLogger('profiler')

class CodeProfiler:
    """
    A context manager to time the execution of a code block.

    Example:
        with CodeProfiler("My Function"):
            # code to time goes here

    Attributes:
        name (str): The name of the code block being timed.
        start_time (float): The time when the block was entered.
    """
    def __init__(self, name=""):
        self.name = name

    def __enter__(self):
        self.start_time = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        elapsed_time_ms = (time.perf_counter() - self.start_time) * 1000
        profiler_log.info("'%s' execution time: %.3f ms", self.name, elapsed_time_ms)
        if elapsed_time_ms > 10.0:
            profiler_log.warning("'%s' exceeded 10ms latency requirement.", self.name)


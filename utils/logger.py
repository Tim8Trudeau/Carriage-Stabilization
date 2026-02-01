# utils/logger.py
#
# Central logging setup for the Carriage Stabilization project.
#
# This version supports selectively enabling loggers and disabling everything else
# to keep log volume (and disk usage) under control.

from __future__ import annotations

import glob
import logging
import os
from contextvars import ContextVar
from typing import Iterable, Optional, Set


_LOOP_I: ContextVar[int] = ContextVar("loop_i", default=-1)


def set_loop_index(i: int) -> None:
    _LOOP_I.set(int(i))


class LoopIndexFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        # ensure every record has .i
        record.i = _LOOP_I.get()
        return True


def setup_logging(
    log_dir: str = "logs",
    overwrite: bool = True,
    log_level: int = logging.DEBUG,
    console_level: int = logging.INFO,
    cleanup_rotated: bool = True,
    *,
    enabled_loggers: Optional[Iterable[str]] = None,
) -> None:
    """Configure project logging.

    If enabled_loggers is provided (or left as default), only those loggers will
    write logs. All other known project loggers will be disabled and will not
    propagate to root.

    Default behavior: enable only the IMU logger ('imu').

    Args:
        log_dir: Directory for log files.
        overwrite: If True, truncate existing logs on startup.
        log_level: File log level for enabled loggers.
        console_level: (Unused unless you later choose to attach a console handler.)
        cleanup_rotated: Remove any '*.log.*' rotated remnants at startup.
        enabled_loggers: Names of loggers to keep enabled. Default {'imu'}.
    """

    os.makedirs(log_dir, exist_ok=True)

    if cleanup_rotated:
        for path in glob.glob(os.path.join(log_dir, "*.log.*")):
            try:
                os.remove(path)
            except OSError:
                pass

    # Enable only imu and motor logs by default
    enabled: Set[str] = set(enabled_loggers) if enabled_loggers is not None else {"imu", "motor"}

    fmt = logging.Formatter("%(i)06d | %(levelname)s | %(name)s | %(message)s")
    loop_filter = LoopIndexFilter()

    # Ensure root does not emit anything (prevents accidental propagation noise).
    root = logging.getLogger()
    for h in list(root.handlers):
        root.removeHandler(h)
    root.addHandler(logging.NullHandler())
    root.setLevel(logging.CRITICAL)
    root.propagate = False

    # Known project loggers (kept for consistency / explicit disabling).
    logger_names = [
        "main",
        "controller",
        "rule_engine",
        "WZ_engine",
        "fuzzifier",
        "defuzzifier",
        "simulation",
        "simloop",
        "imu",
        "i2c",
        "motor",
        "profiler",
    ]

    mode = "w" if overwrite else "a"

    for name in logger_names:
        log = logging.getLogger(name)

        # Always strip any handlers previously attached (prevents duplicates across runs/tests).
        for h in list(log.handlers):
            log.removeHandler(h)

        log.propagate = False

        if name in enabled:
            log.disabled = False
            log.setLevel(log_level)

            fh = logging.FileHandler(
                os.path.join(log_dir, f"{name}.log"),
                mode=mode,
                encoding="utf-8",
            )
            fh.setFormatter(fmt)
            fh.setLevel(log_level)
            fh.addFilter(loop_filter)
            log.addHandler(fh)
        else:
            # Hard-disable everything else.
            log.disabled = True
            log.setLevel(logging.CRITICAL)

    # If you ever want a console handler, attach it ONLY to an enabled logger.
    # (User request: disable everything except imu, so we do not attach console by default.)

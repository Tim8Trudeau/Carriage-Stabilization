# logger.py (patched)
import os, glob, logging
from contextvars import ContextVar

_LOOP_I = ContextVar("loop_i", default=-1)

def set_loop_index(i: int) -> None:
    _LOOP_I.set(int(i))

class LoopIndexFilter(logging.Filter):
    def filter(self, record):
        # ensure every record has .i
        record.i = _LOOP_I.get()
        return True

def setup_logging(
    log_dir: str = "logs",
    overwrite: bool = True,
    log_level: int = logging.DEBUG,
    console_level: int = logging.INFO,
    cleanup_rotated: bool = True,
) -> None:
    os.makedirs(log_dir, exist_ok=True)
    if cleanup_rotated:
        for path in glob.glob(os.path.join(log_dir, "*.log.*")):
            try: os.remove(path)
            except OSError: pass

    fmt = logging.Formatter("%(i)06d | %(levelname)s | %(name)s | %(message)s")

    # console for "main"
    console = logging.StreamHandler()
    console.setFormatter(fmt)
    console.setLevel(console_level)
    console.addFilter(LoopIndexFilter())   # <<< ADD THIS

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

    for name in logger_names:
        log = logging.getLogger(name)
        log.setLevel(log_level)
        log.propagate = False
        for h in list(log.handlers):
            log.removeHandler(h)

        mode = "w" if overwrite else "a"
        fh = logging.FileHandler(os.path.join(log_dir, f"{name}.log"), mode=mode, encoding="utf-8")
        fh.setFormatter(fmt)
        fh.setLevel(log_level)
        fh.addFilter(LoopIndexFilter())    # <<< AND THIS
        log.addHandler(fh)

    # attach console to "main" after filters exist
    logging.getLogger("main").addHandler(console)
    logging.getLogger("main").info("Logging system initialized.")

# test/mock/mock_LSM6DS3TR_driver.py
"""
Mock driver for LSM6DS3TR-C with the SAME API as the real driver:
  - readfrom_into(reg, buf)
  - read(reg, nbytes) -> bytes
  - write(reg, data: bytes) -> None

It *self-loads* MOCK_SPI settings from flc_config.toml:
  [controller_params.MOCK_SPI]
  OMEGA_MODE = "sine" | "constant" | "noisy" | "none"
  STEP_RAD   = 0.05          # increment per sample
  OMEGA_SCALE = 1.0          # scale for gyro synthesis in [-1, +1]
  CONST_OMEGA = 0.0          # constant normalized omega if mode="constant"
  NOISE_RMS   = 0.03         # normalized RMS noise on omega if mode="noisy"
  # (All optional with sensible defaults)

If the file is missing or the section isn't present, sane defaults are used.
"""

from __future__ import annotations
import math
import os
import random
from typing import Dict, Iterable, Optional

try:
    import tomllib  # Py3.11+
except ModuleNotFoundError:
    # Py3.10 backport if needed (project may already depend on tomli)
    import tomli as tomllib  # type: ignore


# Default register addresses commonly used for data reads (you can adjust)
OUTX_L_G = 0x22  # Starting reg in your current code path for 6 bytes (X_L, X_H, Y_L, Y_H, G_L, G_H)


class MockLSM6DS3TRDriver:
    """
    Deterministic synthetic IMU with register map and data streaming.
    """

    def __init__(self, config_path: Optional[str] = None) -> None:
        self._regs: Dict[int, int] = {}  # simple byte-addressable map for init writes

        # Load config on our own (no params passed down)
        cfg_file = config_path or os.environ.get("FLC_CONFIG", "flc_config.toml")
        self._cfg = self._load_mock_cfg(cfg_file)

        # Synthesis state
        self._step = 0.0
        self._rng = random.Random(self._cfg.get("SEED", 12345))

        # Constants for raw scaling
        self._ACC_FS = 32_768  # ±1g → ±16384 LSB (per your project notes)
        self._GYR_FS = 32_768  # ±250 dps → ±32768 LSB (per your project notes)

    # --- config ------------------------------------------------------------

    def _load_mock_cfg(self, path: str) -> Dict[str, float | str]:
        dflt = {
            "OMEGA_MODE": "sine",   # "sine" | "constant" | "noisy" | "none"
            "STEP_RAD": 0.05,
            "OMEGA_SCALE": 1.0,
            "CONST_OMEGA": 0.0,
            "NOISE_RMS": 0.03,
            "SEED": 12345,
            "DATA_START_REG": OUTX_L_G,
        }

        try:
            with open(path, "rb") as f:
                root = tomllib.load(f)
            ms = (
                root.get("controller_params", {})
                    .get("MOCK_SPI", {})
                if isinstance(root, dict) else {}
            )
            if isinstance(ms, dict):
                dflt.update({k: ms[k] for k in ms.keys() & dflt.keys()})
                # allow overriding DATA_START_REG if your code uses a different base
                if "DATA_START_REG" in ms:
                    dflt["DATA_START_REG"] = int(ms["DATA_START_REG"])
        except FileNotFoundError:
            pass
        except Exception:
            # If parsing fails, just keep defaults
            pass
        return dflt

    # --- API parity with real driver --------------------------------------

    def readfrom_into(self, reg: int, buf: bytearray) -> None:
        """If reg == configured data start reg, synthesize 6 bytes (x, y, gyro)."""
        n = len(buf)
        if n == 6 and reg == int(self._cfg["DATA_START_REG"]):
            raw_x, raw_y, raw_g = self._next_sample()
            buf[0:2] = int.to_bytes(raw_x, 2, "little", signed=True)
            buf[2:4] = int.to_bytes(raw_y, 2, "little", signed=True)
            buf[4:6] = int.to_bytes(raw_g, 2, "little", signed=True)
        else:
            # Generic register reads from shadow map
            for i in range(n):
                buf[i] = self._regs.get((reg + i) & 0xFF, 0)

    def read(self, reg: int, nbytes: int) -> bytes:
        ba = bytearray(nbytes)
        self.readfrom_into(reg, ba)
        return bytes(ba)

    def write(self, reg: int, data: bytes | bytearray | Iterable[int]) -> None:
        # Shadow register map to support init sequences (ODR, range, etc.)
        for i, b in enumerate(bytes(data)):
            self._regs[(reg + i) & 0xFF] = int(b) & 0xFF

    def close(self) -> None:
        pass

    # --- sample synthesis --------------------------------------------------

    def _next_sample(self) -> tuple[int, int, int]:
        """Synthesize (raw_x, raw_y, raw_omega) as little-endian signed 16-bit."""
        mode = str(self._cfg["OMEGA_MODE"]).lower()
        step_rad = float(self._cfg["STEP_RAD"])
        omega_scale = float(self._cfg["OMEGA_SCALE"])
        const_omega = float(self._cfg["CONST_OMEGA"])
        noise_rms = float(self._cfg["NOISE_RMS"])

        # position on a unit circle gives accel X/Y (scaled to ±16384)
        x = math.sin(self._step)
        y = math.cos(self._step)
        raw_x = max(-self._ACC_FS, min(self._ACC_FS, int(round(self._ACC_FS * x))))
        raw_y = max(-self._ACC_FS, min(self._ACC_FS, int(round(self._ACC_FS * y))))

        # gyro synthesis
        if mode == "sine":
            base = math.sin(self._step)
        elif mode == "constant":
            base = const_omega  # already in normalized [-1, +1]
        elif mode == "noisy":
            base = math.sin(self._step) + self._rng.gauss(0.0, noise_rms)
        else:
            base = 0.0

        base = max(-1.0, min(1.0, base * omega_scale))
        raw_g = max(-self._GYR_FS, min(self._GYR_FS, int(round(self._GYR_FS * base))))

        # advance
        self._step += step_rad
        # keep step bounded (avoid large floats)
        if self._step > 1e6:
            self._step = math.fmod(self._step, 2.0 * math.pi)

        return raw_x, raw_y, raw_g

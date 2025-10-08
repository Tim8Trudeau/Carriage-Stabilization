# hardware/LSM6DS3TR_i2c_driver.py
"""
Thin, device-specific I2C driver for ST LSM6DS3TR-C (uses hardware.i2c_driver).
"""

from __future__ import annotations
from typing import Iterable
import time, logging

_log = logging.getLogger("imu.i2c")

CTRL3_C      = 0x12
STATUS_REG   = 0x1E
OUTX_L_G     = 0x22
_BDU_BIT     = 0x40
_IFINC_BIT   = 0x04
_STATUS_XLDA = 0x01
_STATUS_GDA  = 0x02

class LSM6DS3TRDriver:
    def __init__(self, controller_params: dict | None = None, *, i2c_bus: int | None = None,
                 i2c_addr: int | None = None, i2c_flags: int = 0) -> None:
        from hardware.i2c_driver import get_i2c_host

        params = controller_params or {}
        bus  = i2c_bus  if i2c_bus  is not None else int(params.get("I2C_BUS", 1))
        addr = i2c_addr if i2c_addr is not None else int(params.get("I2C_ADDR", 0x6B))

        self._pi = get_i2c_host(params)       # <- selects pigpio vs mock
        self._h  = self._pi.i2c_open(bus, addr, i2c_flags)

        # 3-line log tweak (host kind)
        _log.info("LSM6DS3TR(I2C): bus=%d addr=0x%02X host=%s",
                  bus, addr, "mock" if hasattr(self._pi, "set_motor_cmd") else "pigpio")

        # BDU=1, IF_INC=1 (coherent multi-byte reads + auto-increment)
        cur = self.read(CTRL3_C, 1)[0]
        want = (cur | _BDU_BIT | _IFINC_BIT) & 0xFF
        if want != cur:
            self.write(CTRL3_C, bytes([want]))

    # --- internal: normalize pigpio/mock block reads to bytes of length n ---
    def _read_block(self, reg: int, n: int) -> bytes:
        ret = self._pi.i2c_read_i2c_block_data(self._h, reg & 0xFF, int(n))

        def _to_bytes_any(obj) -> bytes:
            # Fast paths
            if isinstance(obj, (bytes, bytearray, memoryview)):
                return bytes(obj)
            # Some pigpio builds or wrappers may hand back array('B') or lists/tuples
            try:
                return bytes(bytearray(int(x) & 0xFF for x in obj))
            except Exception:
                # Last resort: if it's a str (shouldn't happen, but be robust)
                if isinstance(obj, str):
                    # Treat as raw 8-bit data; latin-1 preserves byte values 0..255
                    return obj.encode("latin-1", errors="ignore")
                # Give up—empty bytes
                return b""

        # pigpio returns (count, data); mock returns bytes/bytearray
        if isinstance(ret, tuple) and len(ret) == 2:
            count, data = ret
            data_b = _to_bytes_any(data)[: int(count)]
        else:
            data_b = _to_bytes_any(ret)

        if len(data_b) < n:
            raise RuntimeError(f"short read from 0x{reg:02X}: got {len(data_b)} < {n}")
        return data_b[:n]

    # --- reg helpers ---
    def readfrom_into(self, reg: int, buf: bytearray) -> None:
        n = len(buf)
        buf[:n] = memoryview(self._read_block(reg, n))[:n]

    def read(self, reg: int, nbytes: int) -> bytes:
        return self._read_block(reg, int(nbytes))

    def write(self, reg: int, data: bytes | bytearray | Iterable[int]) -> None:
        payload = bytes(data)
        if len(payload) == 1:
            self._pi.i2c_write_byte_data(self._h, reg & 0xFF, payload[0])
        else:
            for i, b in enumerate(payload):
                self._pi.i2c_write_byte_data(self._h, (reg + i) & 0xFF, b)

    # --- convenience ---
    def read_ax_ay_gz_bytes(self, timeout_s: float = 0.02) -> bytes:
        deadline = time.perf_counter() + timeout_s
        while True:
            status = self._pi.i2c_read_byte_data(self._h, STATUS_REG) & 0xFF
            if (status & (_STATUS_XLDA | _STATUS_GDA)) == (_STATUS_XLDA | _STATUS_GDA):
                break
            if time.perf_counter() >= deadline:
                raise RuntimeError(f"data not ready (STATUS=0x{status:02X})")
        block = self._read_block(OUTX_L_G, 12)
        # Coordinate remap: AX->AZ AY->AY GZ->GX
        # [GX][GY][GZ][AX][AY][AZ] → return [AX, AY, GZ] (each 16-bit LE)
        # return bytes([block[6], block[7], block[8], block[9], block[4], block[5]])
        return bytes([block[10], block[11], block[8], block[9], block[0], block[1]])

    # --- lifecycle ---
    def close(self) -> None:
        try:
            if getattr(self, "_h", None) is not None:
                self._pi.i2c_close(self._h)
        finally:
            stop = getattr(self._pi, "stop", None)
            if callable(stop): stop()
            self._h = None; self._pi = None

    # --- sim passthroughs (no-op on real pigpio) ---
    def set_motor_cmd(self, v: float) -> None:
        f = getattr(self._pi, "set_motor_cmd", None)
        if callable(f): f(v)

    def get_sim_theta(self):
        f = getattr(self._pi, "get_sim_theta", None)
        return float(f()) if callable(f) else None

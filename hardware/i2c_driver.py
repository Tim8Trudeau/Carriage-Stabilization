# hardware/i2c_driver.py
"""
Unified I2C host factory for real hardware (pigpio) and simulation (mock).

- Pass SIM_MODE via controller_params["SIM_MODE"] (or sim_mode arg).
- If SIM_MODE=True → return mock host that can read sim_config.toml and apply
  [simulation.initial_conditions].
- Else try pigpio; if unavailable, fall back to mock host (TOML disabled).
"""

from __future__ import annotations
import logging, os, sys
from typing import Optional, Dict

_i2c_log = logging.getLogger("i2c")

# Minimal regs used by the IMU driver
CTRL3_C    = 0x12
STATUS_REG = 0x1E
OUTX_L_G   = 0x22
_XLDA = 0x01
_GDA  = 0x02

_SIM_TOML_PATH = "config/sim_config.toml"

def _running_under_pytest() -> bool:
    return "PYTEST_CURRENT_TEST" in os.environ or "pytest" in sys.modules

def get_i2c_host(controller_params: Optional[Dict] = None,
                 sim_mode: Optional[bool] = None):
    """
    Returns a pigpio-like object implementing:
      i2c_open, i2c_close, i2c_read_byte_data, i2c_read_i2c_block_data,
      i2c_write_byte_data, stop
    """
    params = controller_params or {}
    if sim_mode is None:
        sim_mode = bool(params.get("SIM_MODE", False))

    if sim_mode:
        _i2c_log.info("i2c_driver: SIM_MODE=True → using mock I2C host with sim_config.toml")
        return _make_mock_host(params, apply_initial_conditions=True)

    # Try real pigpio first
    try:
        import pigpio  # type: ignore
        pi = pigpio.pi()
        if not pi.connected:
            _i2c_log.debug("******* pigpio did not start.**********")
            raise RuntimeError("pigpio daemon not running (try: sudo pigpiod)")
        _i2c_log.info("i2c_driver: using pigpio.pi() as I2C host.")
        return pi
    except Exception as e:
        _i2c_log.warning("i2c_driver: pigpio unavailable (%s). Falling back to mock I2C host.", e)
        return _make_mock_host(params, apply_initial_conditions=False)

def _make_mock_host(controller_params: Dict, apply_initial_conditions: bool):
    """
    Create a pigpio-like mock host and wire it to mock_i2c.MockI2CBus.
    """
    try:
        from test.mocks.mock_i2c import MockI2CBus  # project mock
    except Exception as e:
        raise ImportError("mock_i2c.MockI2CBus not found at repo root") from e

    class _MockPiHost:
        def __init__(self):
            self._handles = {}
            self._next_h = 1
            self._regs: Dict[int, int] = {CTRL3_C: 0x00}
            use_toml = bool(apply_initial_conditions and not _running_under_pytest())
            self._bus = MockI2CBus(
                controller_params=controller_params,
                sim_config_path=_SIM_TOML_PATH if use_toml else None,
                sim_mode=use_toml,
            )

        # pigpio-like API
        def i2c_open(self, bus: int, addr: int, flags: int = 0):
            h = self._next_h; self._next_h += 1
            self._handles[h] = (bus, addr, flags)
            return h

        def i2c_close(self, handle: int):
            self._handles.pop(handle, None)

        def i2c_write_byte_data(self, handle, reg, byte):
            # self.i2c_write_i2c_block_data(handle, reg, bytes([byte]))
            return

        def i2c_write_i2c_block_data(self, handle, reg, data):
            return

        def i2c_read_byte_data(self, handle: int, reg: int) -> int:
            if reg == STATUS_REG: return _XLDA | _GDA
            return int(self._regs.get(reg, 0)) & 0xFF

        # These calls are only for mocking of the i2c bus
        def i2c_read_i2c_block_data(self, handle: int, reg: int, count: int):
            if reg == OUTX_L_G and count >= 12:
                # six = [AX, AY, GZ] (each 16-bit LE) in logical coordinates
                six = self._bus.imu_read()
                ax_l, ax_h, ay_l, ay_h, gz_l, gz_h = six

                # Device block must satisfy:
                #   logical AX = AZ_device
                #   logical AY = AY_device
                #   logical GZ = GX_device
                block = bytes([
                    gz_l, gz_h,       # GX_device  ← logical GZ
                    0x00, 0x00,       # GY_device  (unused)
                    0x00, 0x00,       # GZ_device  (unused under this mapping)
                    0x00, 0x00,       # AX_device  (unused)
                    ay_l, ay_h,       # AY_device  ← logical AY
                    ax_l, ax_h,       # AZ_device  ← logical AX
                ])
                return block[:count] if count < 12 else block
            return bytes([0] * count)

        def stop(self): self._handles.clear()

        # sim conveniences (no-ops on real HW)
        def set_motor_cmd(self, v: float) -> None:
            f = getattr(self._bus, "set_motor_cmd", None)
            if callable(f): f(v)

        def get_sim_theta(self):
            f = getattr(self._bus, "get_sim_theta", None)
            return float(f()) if callable(f) else None

    _i2c_log.info("i2c_driver: using MockI2CBus-backed I2C host shim.")
    return _MockPiHost()

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

_i2c_log = logging.getLogger("imu.i2c")

# Minimal regs used by the IMU driver
CTRL3_C    = 0x12
STATUS_REG = 0x1E
OUTX_L_G   = 0x22

_XLDA = 0x01
_GDA  = 0x02

_SIM_TOML_PATH = "config/sim_config.toml"

def _running_under_pytest() -> bool:
    pt = "PYTEST_CURRENT_TEST" in os.environ or "pytest" in sys.modules
    _i2c_log.info("i2c_driver: SIM_MODE=Running under pytest? %s", pt)
    return pt

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
    Create a pigpio-like mock host from the unified mock_pigpio.
    """
    try:
        from test.mocks import mock_pigpio
    except Exception as e:
        raise ImportError("test.mocks.mock_pigpio not found") from e

    use_toml = bool(apply_initial_conditions and not _running_under_pytest())
    pi = mock_pigpio.pi()
    _i2c_log.info(
        "i2c_driver: using unified mock_pigpio as I2C host (SIM_MODE=%s, toml=%s)",
        bool(controller_params.get("SIM_MODE", False)), use_toml
    )
    return pi

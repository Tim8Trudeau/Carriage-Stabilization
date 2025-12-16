# hardware/i2c_driver.py
"""
Unified I2C host factory for real hardware (pigpio) and simulation (mock).

Selection:
  - Pass SIM_MODE via controller_params["SIM_MODE"] (preferred), or override with sim_mode=...
  - If SIM_MODE is True: return the mock I2C host.
  - Else: try pigpio; if unavailable or not connected, fall back to the mock host
    (with TOML/initial-conditions disabled to keep tests deterministic).

The returned host is "pigpio-like" and implements at least:
  i2c_open, i2c_close, i2c_read_byte_data, i2c_read_i2c_block_data,
  i2c_write_byte_data, stop
"""

from __future__ import annotations

import logging
import os
from typing import Any, Dict, Optional

_i2c_log = logging.getLogger("imu.i2c")


def _running_under_pytest() -> bool:
    return "PYTEST_CURRENT_TEST" in os.environ


def _import_mock_pigpio():
    """Import mock pigpio from the project's mocks package.

    Your project historically used `test.mocks.mock_pigpio`.
    If you later move it under `tests.mocks.mock_pigpio`, this also supports that.
    """
    try:
        from test.mocks import mock_pigpio  # legacy/real in your repo
        return mock_pigpio
    except Exception:
        from tests.mocks import mock_pigpio  # fallback if migrated to tests/
        return mock_pigpio


def _make_mock_host(
    controller_params: Dict[str, Any],
    *,
    apply_initial_conditions: bool = True,
):
    """Create a pigpio-like mock host."""
    mock_pigpio = _import_mock_pigpio()

    # Keep the knob for future mocks; current mock_pigpio.pi() takes no args.
    use_toml = bool(apply_initial_conditions and not _running_under_pytest())

    pi = mock_pigpio.pi()

    _i2c_log.info(
        "i2c_driver: using mock_pigpio I2C host (SIM_MODE=%s, toml=%s)",
        bool(controller_params.get("SIM_MODE", False)),
        use_toml,
    )
    return pi


def get_i2c_host(
    controller_params: Optional[Dict[str, Any]] = None,
    *,
    sim_mode: Optional[bool] = None,
    apply_initial_conditions: bool = True,
):
    """Return an I2C host object (real pigpio or mock)."""
    params: Dict[str, Any] = controller_params or {}

    if sim_mode is None:
        sim_mode = bool(params.get("SIM_MODE", False))

    if sim_mode:
        return _make_mock_host(params, apply_initial_conditions=apply_initial_conditions)

    # Try real pigpio first.
    try:
        import pigpio  # type: ignore
        pi = pigpio.pi()
        if getattr(pi, "connected", True):
            _i2c_log.info("i2c_driver: using real pigpio host")
            return pi
        _i2c_log.warning("i2c_driver: pigpio host not connected; falling back to mock")
    except Exception as e:
        _i2c_log.warning("i2c_driver: pigpio unavailable (%s); falling back to mock", e)

    # Fallback mock with TOML disabled.
    return _make_mock_host(params, apply_initial_conditions=False)

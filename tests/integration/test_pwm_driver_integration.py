# tests/integration/test_pwm_driver_integration.py
# These tests must be run on the target processor during integration.

import platform
import pytest


@pytest.mark.integration
@pytest.mark.skipif(
    platform.system() == "Windows",
    reason="Integration tests require Linux (preferably Raspberry Pi)",
)
def test_real_pwm_connection():
    if platform.system() != "Linux" or "arm" not in platform.machine():
        pytest.skip("Not running on a Raspberry Pi")

    try:
        import pigpio
    except ImportError:
        pytest.skip("pigpio not installed")

    pi = pigpio.pi()
    if not pi.connected:
        pytest.skip("pigpio daemon not running")

    from pwm_driver import DualPWMController

    pwm = DualPWMController()

    pwm.command(8192)  # 25% duty cycle
    pwm.command(0)  # Stop
    pwm.command(-16384)  # 50% reverse
    pwm.stop()

    assert pi.connected

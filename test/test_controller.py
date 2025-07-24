import pytest
import json5
from flc.controller import FLCController
# Theta and Omega must be normilized to range -1 to +1

@pytest.fixture
def full_config():
    """Loads the actual project config file."""
    with open('config/flc_config.json5', 'r') as f:
        return json5.load(f)

@pytest.fixture
def flc_controller(full_config):
    """Returns an FLCController instance with the full configuration."""
    return FLCController(full_config)

def test_controller_initialization(flc_controller):
    assert flc_controller is not None
    assert flc_controller.fuzzifier is not None
    assert flc_controller.rule_engine is not None
    assert flc_controller.defuzzifier is not None

def test_controller_cycle_zero_inputs(flc_controller):
    """Test with zero error, expecting zero motor command."""
    motor_cmd = flc_controller.calculate_motor_cmd(theta=0.0, omega=0.0)
    # With zero inputs, only the 'NE' and 'NV' rule should be fully active.
    # The output for that rule is bias=0.0, so the result should be 0.
    assert motor_cmd == pytest.approx(0.0, abs=1e-6)

def test_controller_cycle_position_error(flc_controller):
    """Test with only a positive position error (SECW)."""
    # theta=0.25 is the peak of SECW and also on the edge of NE and MECW.
    # omega=0.0 is the peak of NV.
    # We expect a negative motor command to correct the CW error.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=0.25, omega=0.0)
    assert motor_cmd < 0.0

def test_controller_cycle_velocity_error(flc_controller):
    """Test with only a positive velocity error (SVCW)."""
    # theta=0.0 is the peak of NE.
    # omega=0.5 is the peak of SVCW and also on the edge of NV and LVCW.
    # We expect a negative motor command to counteract the CW velocity.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=0.0, omega=0.5)
    assert motor_cmd < 0.0

def test_controller_cycle_combined_error(flc_controller):
    """Test with both position and velocity error."""
    # Carriage is slightly counter-clockwise and moving counter-clockwise.
    # We expect a strong positive (CW) motor command to correct this.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=-0.3, omega=-0.6)
    assert motor_cmd > 0.0

def test_controller_cycle_large_error(flc_controller):
    """Test with position near pi and a velocity error."""
    # Carriage is at -90 degrees (CCW) and moving counter-clockwise.
    # We expect a max positive (CW) motor command to correct this big error.
    # The rules produce a small negitive response!?
    motor_cmd = flc_controller.calculate_motor_cmd(theta=-1.0, omega=-.6)
    assert motor_cmd == pytest.approx(1.0, abs=1e-2)


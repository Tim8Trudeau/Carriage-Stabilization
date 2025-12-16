from typing import Any
import pytest
import json5
from flc.controller import FLCController
# Theta and Omega must be normilized to range -1 to +1


@pytest.fixture
def full_config():
    """Loads the actual project config file."""
    with open("config/flc_config.json5", "r") as f:
        return json5.load(f)


@pytest.fixture
def flc_controller(full_config: Any):
    """Returns an FLCController instance with the full configuration."""
    return FLCController(full_config)


def test_controller_initialization(flc_controller: FLCController):
    assert flc_controller is not None
    assert flc_controller.fuzzifier is not None
    assert flc_controller.rule_engine is not None
    assert flc_controller.defuzzifier is not None


def test_controller_cycle_zero_inputs(flc_controller: FLCController):
    """Test with zero error, expecting zero motor command."""
    motor_cmd = flc_controller.calculate_motor_cmd(theta=0.0, omega=0.0)
    # With zero inputs, only the 'NE' and 'NV' rule should be fully active.
    # The output for that rule is bias=0.0, so the result should be 0.
    assert motor_cmd == pytest.approx(0.0, abs=1e-6)


def test_controller_cycle_position_error(flc_controller: FLCController):
    """Test with only a positive position error (SECW)."""
    # theta=0.25 is the peak of SECW and also on the edge of NE and MECW.
    # omega=0.0 is the peak of NV.
    # We expect a negative motor command to correct the CW error.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=0.25, omega=0.0)
    assert motor_cmd < 0.0


def test_controller_cycle_velocity_error(flc_controller: FLCController):
    """Test with only a positive velocity error (SVCW)."""
    # theta=0.0 is the peak of NE.
    # omega=0.5 is the peak of SVCW and also on the edge of NV and LVCW.
    # Carrage has over-shot and needs to change direction
    # We expect a small negative motor command to counteract the CW inertia.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=0.0, omega=0.45)
    assert motor_cmd == pytest.approx(-0.1, abs=1e-1)


def test_controller_cycle_combined_error(flc_controller: FLCController):
    """Test with both position and velocity error."""
    # Carriage is counter-clockwise and moving counter-clockwise.
    # We expect a strong positive (CW) motor command to correct this.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=-0.32, omega=-0.15)
    assert motor_cmd > 0.5  # ***Fails with -0.18


def test_controller_cycle_large_combined_error(flc_controller: FLCController):
    """Test with position near pi and a velocity error."""
    # Carriage is -80 degrees (CCW) and moving counter-clockwise.
    # We expect a max positive (CW) motor command to correct this big error.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=-0.9, omega=-0.4)
    assert motor_cmd == pytest.approx(-1.0, abs=1e-2)


def test_controller_cycle_correcting(flc_controller: FLCController):
    """Test with position near pi and a velocity error."""
    # Carriage is at -45 degrees (CCW) and moving clockwise.
    # We expect an error correcting positive (CW) motor command.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=-0.5, omega=0.4)
    assert motor_cmd == pytest.approx(0.7, abs=1e-1)  # ***Fails with -0.4


def test_controller_cycle_oppose_inertia(flc_controller: FLCController):
    # Carriage is at +10 degrees (CW) and moving CCW too fast.
    # We expect a small negitive (CCW) motor command to slow coorction.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=0.05, omega=-0.9)
    assert motor_cmd == pytest.approx(-0.2, abs=1e-1)


def test_controller_cycle_near_zero_error(flc_controller: FLCController):
    # Carriage is at +4.5 degrees (CW) and moving slowly CCW.
    # We expect a very small negitive (CCW) motor command approaching zero error.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=0.05, omega=-0.1)
    assert motor_cmd == pytest.approx(-0.01, abs=1e-2)


def test_controller_cycle_near_zero_error2(flc_controller: FLCController):
    # Carriage is at -4.5 degrees (CCW) and not moving.
    # We expect a very small positive (CW) motor command approaching zero error.
    motor_cmd = flc_controller.calculate_motor_cmd(theta=-0.5, omega=0.0)
    assert motor_cmd == pytest.approx(0.01, abs=1e-2)

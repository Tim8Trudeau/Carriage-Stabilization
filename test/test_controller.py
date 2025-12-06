# tests/test_controller.py

import pytest
from flc.controller import FLCController
import tomllib

# ------------------------------------------------------------
# Fixture: Load your REAL FLC (from config/flc_config.toml)
# ------------------------------------------------------------
@pytest.fixture
def flc_controller():
    with open("config/flc_config.toml", "rb") as f:
        config = tomllib.load(f)
    return FLCController(config)


# ------------------------------------------------------------
# Parametrized tests for motor command range
# These tests verify:
#   1. Output is always clamped to [-1, 1]
#   2. Increasing positive error (theta > 0) produces larger negative command
#   3. Increasing negative error (theta < 0) produces larger positive command
#   4. Omega contributes consistently with negative feedback
# ------------------------------------------------------------
@pytest.mark.parametrize(
    "theta, omega",
    [
        (0.0, 0.0),
        (0.1, 0.0),
        (-0.1, 0.0),
        (0.0, 0.2),
        (0.0, -0.2),
        (0.3, 0.3),
        (-0.3, -0.3),
        (0.5, -0.4),
        (-0.6, 0.1),
        (1.0, 0.5),
        (-1.0, -0.5),
    ],
)
def test_motor_cmd_within_bounds(flc_controller, theta, omega):
    """Motor command must always be in [-1, 1] regardless of input."""
    cmd = flc_controller.calculate_motor_cmd(theta, omega)
    assert -1.0 <= cmd <= 1.0


# ------------------------------------------------------------
# Check sign behavior: positive theta → negative correction
# ------------------------------------------------------------
@pytest.mark.parametrize(
    "theta, omega",
    [
        (0.05, 0.0),
        (0.1, 0.0),
        (0.3, 0.1),
        (0.5, -0.2),
        (1.0, 0.0),
    ],
)
def test_positive_theta_produces_negative_cmd(flc_controller, theta, omega):
    """FLC should apply negative feedback for positive tilt."""
    cmd = flc_controller.calculate_motor_cmd(theta, omega)
    assert cmd <= 0.0


# ------------------------------------------------------------
# Check sign behavior: negative theta → positive correction
# ------------------------------------------------------------
@pytest.mark.parametrize(
    "theta, omega",
    [
        (-0.05, 0.0),
        (-0.1, -0.1),
        (-0.3, 0.03),
        (-0.7, -0.2),
        (-1.0, 0.0),
    ],
)
def test_negative_theta_produces_positive_cmd(flc_controller, theta, omega):
    """FLC should apply positive feedback for negative tilt."""
    cmd = flc_controller.calculate_motor_cmd(theta, omega)
    assert cmd >= 0.0


# ------------------------------------------------------------
# Omega-only tests: angular velocity contributes same-sign braking
# ------------------------------------------------------------
@pytest.mark.parametrize(
    "theta, omega, expectation",
    [
        (0.0, 0.2, -1),   # CW rotation → apply CCW torque (negative cmd)
        (0.0, -0.2, +1),  # CCW rotation → apply CW torque (positive cmd)
    ],
)
def test_omega_drives_correct_direction(flc_controller, theta, omega, expectation):
    """Omega alone should produce braking torque."""
    cmd = flc_controller.calculate_motor_cmd(theta, omega)
    if expectation < 0:
        assert cmd <= 0.0
    else:
        assert cmd >= 0.0


# ------------------------------------------------------------
# Monotonicity: larger magnitude errors → larger magnitude outputs
# ------------------------------------------------------------
@pytest.mark.parametrize(
    "small, large",
    [
        ((0.1, 0.0), (0.4, 0.0)),
        ((-0.1, 0.0), (-0.4, 0.0)),
        ((0.0, 0.1), (0.0, 0.4)),
        ((0.2, -0.2), (0.6, -0.5)),
    ],
)
def test_magnitude_increases_with_error(flc_controller, small, large):
    """Ensure controller output magnitude increases with larger errors."""
    s_th, s_om = small
    l_th, l_om = large

    cmd_small = abs(flc_controller.calculate_motor_cmd(s_th, s_om))
    cmd_large = abs(flc_controller.calculate_motor_cmd(l_th, l_om))

    assert cmd_large >= cmd_small

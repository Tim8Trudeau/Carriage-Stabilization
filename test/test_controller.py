import pytest
from flc.controller import FLCController
import json5


@pytest.fixture
def flc_controller():
    with open("config/flc_config.json5", "r") as f:
        config = json5.load(f)
    return FLCController(config)


@pytest.mark.parametrize(
    "theta, omega, expected_range",
    [
        (0.0, 0.0, (-0.01, 0.01)),  # No error
        (0.25, 0.0, (-1.0, 0.0)),  # Positive theta only
        (0.0, 0.5, (-0.3, 0.0)),  # Positive omega only
        (-0.9, -0.4, (-1.0, -0.9)),  # Large CCW error
        (-0.5, 0.4, (0.5, 1.0)),  # Mixed correction
        (0.05, -0.9, (-0.3, 0.0)),  # Small pos theta, fast CCW
        (-0.1, 0.1, (-0.1, 0.1)),  # Small error both ways
    ],
)
def test_motor_cmd_range(flc_controller, theta, omega, expected_range):
    motor_cmd = flc_controller.calculate_motor_cmd(theta, omega)
    assert expected_range[0] <= motor_cmd <= expected_range[1]

import pytest
from flc.controller import FLCController
import tomllib

# fmt: off #noqa
@pytest.fixture
def flc_controller():
    with open("config/flc_config.toml", "rb") as f:
        config = tomllib.load(f)
    return FLCController(config)


@pytest.mark.parametrize(
    "theta, omega, expected_range",
    [
        (0.0, 0.0, (-0.01, 0.01)),  # No control error
        (0.25, 0.0, (-0.5, 0.0)),  # Positive theta only
        (0.0, 0.5, (-0.3, 0.0)),  # Positive omega only
        (-0.9, -0.4, (0.9, 1.0)),  # Large CCW error
        (-0.5, 0.4, (0.5, 1.0)),  # Mixed correction
        (0.05, -0.9, (-0.2, 0.3)),  # Small pos theta, fast CCW - THis motor command should be close to zero!!!
        (-0.1, 0.1, (0.0, 0.05)),  # Small CCW error moving CW
    ],
)
def test_motor_cmd_range(flc_controller, theta, omega, expected_range):
    motor_cmd = flc_controller.calculate_motor_cmd(theta, omega)
    assert expected_range[0] <= motor_cmd <= expected_range[1]

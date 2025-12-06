# tests/test_central_config.py

import types
from simulation.central_config import load_simulation_config

def test_central_config_loads():
    plant, motor, sim_cfg, duration, controller, ic = load_simulation_config()

    assert plant.I > 0
    assert motor.n_rollers >= 1
    assert sim_cfg.dt > 0
    assert duration > 0
    assert isinstance(ic, tuple)
    assert callable(controller) or controller is None

def test_controller_callable():
    _, _, _, _, controller, ic = load_simulation_config()
    if controller is not None:
        assert controller(0.1, 0.0, 0.0) is not None

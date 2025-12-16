# tests/test_simulator_perturbations.py

import numpy as np
from simulation.carriage_simulator import (
    CarriageSimulator, PlantParams, MotorParams, SimConfig
)

def make_sim():
    plant = PlantParams(I=0.12, m=1.088, r=0.622)
    motor = MotorParams(0.16, 2, 0.012, 0.6096)
    cfg = SimConfig(dt=0.002, steps_per_log=1)
    sim = CarriageSimulator(plant, motor, cfg, lambda th, om, t: 0)
    return sim

def test_impulse_is_applied():
    sim = make_sim()
    sim.reset()
    sim.perturb.impulses.clear()
    sim.perturb.add_impulse(0.1, 0.5)

    sim.run(0.2)
    tau_ext = np.array(sim.log_tau_ext)

    assert np.any(np.abs(tau_ext) > 0.1)

def test_step_applied_region():
    sim = make_sim()
    sim.reset()
    sim.perturb.steps.clear()
    sim.perturb.add_step(0.2, 0.4, 0.03)

    sim.run(0.5)

    t = np.array(sim.log_t)
    tau = np.array(sim.log_tau_ext)

    region = (t >= 0.2) & (t <= 0.4)
    # Require step to be applied on all but perhaps the first dt interval
    assert np.allclose(tau[region][1:], 0.03, atol=1e-4)

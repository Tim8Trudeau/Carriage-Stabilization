# tests/test_simulator_perturbations.py

import numpy as np
from simulation.carriage_simulator import (
    CarriageSimulator, PlantParams, MotorParams
)

def make_sim():
    plant = PlantParams(I=0.12, m=1.088, r=0.622)
    motor = MotorParams(0.16, 2, 0.012, 0.6096)
    return CarriageSimulator(plant, motor)

def test_impulse_is_applied():
    sim = make_sim()
    sim.reset()
    sim.perturb.impulses.clear()
    sim.perturb.add_impulse(0.1, 0.5)

    sim.run(0.2)

    tau_ext = np.array(sim.log_tau_ext)
    assert any(abs(t) > 0.4 for t in tau_ext)

def test_step_applied_region():
    sim = make_sim()
    sim.reset()
    sim.perturb.steps.clear()
    sim.perturb.add_step(0.2, 0.4, 0.03)

    sim.run(0.5)

    t = np.array(sim.log_t)
    tau_ext = np.array(sim.log_tau_ext)

    assert np.all(tau_ext[(t >= 0.2) & (t <= 0.4)] > 0.0)

# tests/test_simulator_physics.py

import numpy as np
from simulation.carriage_simulator import (
    CarriageSimulator, PlantParams, MotorParams, SimConfig
)

def test_motor_torque_scaling():
    motor = MotorParams(tau_motor_one=0.16, n_rollers=2,
                        r_roller=0.012, r_wheel=0.6096)
    plant = PlantParams(I=0.12, m=1.088, r=0.622)

    sim = CarriageSimulator(plant, motor)
    tau_half = sim._motor_torque(0.5)
    tau_full = sim._motor_torque(1.0)

    assert np.isclose(tau_full, 2 * tau_half, rtol=1e-6)

def test_gravity_torque_sign():
    plant = PlantParams(I=0.12, m=1.088, r=0.622)
    motor = MotorParams(0.16, 2, 0.012, 0.6096)
    sim = CarriageSimulator(plant, motor)

    # Small positive angle → positive sin(theta) → positive τ_g
    sim.reset(theta=0.1)
    tg = sim._gravity_torque()
    assert tg > 0

def test_no_nan_outputs():
    plant = PlantParams(I=0.12, m=1.088, r=0.622)
    motor = MotorParams(0.16, 2, 0.012, 0.6096)
    sim = CarriageSimulator(plant, motor)

    sim.reset(0.1, 0.0)
    sim.run(2.0)

    assert not any(np.isnan(sim.log_theta))
    assert not any(np.isnan(sim.log_omega))

import numpy as np
from simulation.carriage_simulator import CarriageSimulator
from simulation.central_config import load_simulation_config

def test_motor_torque_limits():
    plant, motor, sim_cfg, duration, controller, ic = load_simulation_config()
    sim = CarriageSimulator(plant, motor, sim_cfg, controller)

    # Full forward
    tau_plus = sim._motor_torque(1.0)
    # Full reverse
    tau_minus = sim._motor_torque(-1.0)

    assert tau_plus > 0
    assert tau_minus < 0

    # Should be symmetric
    assert abs(tau_plus + tau_minus) < 1e-6

    # Should be near 16.27 N*m
    assert abs(abs(tau_plus) - 16.27) < 0.5

def test_gravity_torque_symmetry():
    plant, motor, sim_cfg, duration, controller, ic = load_simulation_config()
    sim = CarriageSimulator(plant, motor, sim_cfg, controller)

    sim.theta = 0
    assert abs(sim._gravity_torque()) < 1e-9

    sim.theta = np.pi/2
    tau1 = sim._gravity_torque()

    sim.theta = -np.pi/2
    tau2 = sim._gravity_torque()

    assert abs(tau1 + tau2) < 1e-9

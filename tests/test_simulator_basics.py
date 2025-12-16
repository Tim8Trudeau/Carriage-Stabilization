# tests/test_simulator_basic.py

import math
from simulation.carriage_simulator import (
    CarriageSimulator, PlantParams, MotorParams, SimConfig
)

def test_sim_run_basic():
    plant = PlantParams(I=0.12, m=1.088, r=0.622)
    motor = MotorParams(tau_motor_one=0.16, n_rollers=2,
                        r_roller=0.012, r_wheel=0.6096)
    cfg = SimConfig(dt=0.002, steps_per_log=10)

    def zero_ctrl(theta, omega, t):
        return 0.0

    sim = CarriageSimulator(plant, motor, cfg, zero_ctrl)
    sim.reset(theta=0.1, omega=0.0)
    sim.run(1.0)

    assert len(sim.log_t) > 0
    assert len(sim.log_theta) == len(sim.log_t)
    assert len(sim.log_omega) == len(sim.log_t)
    assert len(sim.log_cmd) == len(sim.log_t)

    # Gravity torque should be nonzero at nonzero theta
    assert any(abs(tg) > 0 for tg in sim.log_tau_g)

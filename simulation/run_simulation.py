"""
run_simulation.py
=================

Driver script for running the carriage physics simulation.

Loads configuration from config/sim_config.toml,
builds the CarriageSimulator instance, runs the simulation,
and optionally plots results.
"""

from simulation.central_config import load_simulation_config
from simulation.carriage_simulator import CarriageSimulator
from simulation.plot_sim_results import plot_sim_results


def main():
    plant, motor, sim_cfg, duration, controller, ic = load_simulation_config()

    sim = CarriageSimulator(plant, motor, sim_cfg, controller)

    theta0, omega0, t0 = ic
    sim.reset(theta0, omega0, t0)

    sim.run(duration)

    plot_sim_results(sim)


if __name__ == "__main__":
    main()
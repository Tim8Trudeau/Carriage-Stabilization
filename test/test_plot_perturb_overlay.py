# tests/test_plot_perturb_overlay.py

from simulation.plot_sim_results import overlay_perturbations, annotate_perturbations
from simulation.carriage_simulator import PlantParams, MotorParams, CarriageSimulator
import matplotlib.pyplot as plt

def test_overlay_and_annotations_do_not_crash():
    plant = PlantParams(I=0.12, m=1.088, r=0.622)
    motor = MotorParams(0.16, 2, 0.012, 0.6096)

    sim = CarriageSimulator(plant, motor)
    sim.reset()
    sim.run(1.0)

    fig, ax = plt.subplots()
    ax_ext = ax.twinx()

    overlay_perturbations(ax, ax_ext, sim)
    annotate_perturbations(ax, sim)

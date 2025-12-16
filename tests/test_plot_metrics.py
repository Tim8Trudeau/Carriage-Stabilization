# tests/test_plot_metrics.py

import numpy as np
from simulation.plot_sim_results import (
    compute_overshoot, compute_settling_time, estimate_damping_ratio, compute_rise_time
)

def test_metric_functions():
    t = np.linspace(0, 5, 500)
    theta = 0.3 * np.exp(-0.4 * t) * np.sin(4 * t)

    assert compute_overshoot(theta) >= 0
    assert compute_settling_time(t, theta) >= 0
    assert 0.0 <= estimate_damping_ratio(theta, t) <= 1.0
    assert compute_rise_time(t, theta) >= 0

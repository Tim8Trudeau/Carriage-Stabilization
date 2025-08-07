# rule_trace.py

from typing import List, Dict, Any
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def trace_rule_firing(
    rules: List[Dict[str, Any]],
    fuzzified_theta: Dict[str, float],
    fuzzified_omega: Dict[str, float],
    crisp_theta: float,
    crisp_omega: float,
    plot: bool = False,
) -> None:
    """
    Evaluate each rule and return detailed trace information per rule,
    including membership degrees, firing strength, and Z output.

    Args:
        rules: List of fuzzy rules (from config).
        fuzzified_theta: Dict of theta membership degrees.
        fuzzified_omega: Dict of omega membership degrees.
        crisp_theta: Raw normalized theta input.
        crisp_omega: Raw normalized omega input.

    Returns:
        A list of dictionaries with detailed rule evaluation traces.
    """
    if not plot:
        return
    else:
        traces = []

        for i, rule in enumerate(rules):
            antecedent = rule["rule"]
            theta_set = antecedent[0]
            omega_set = antecedent[1]

            degree_theta = fuzzified_theta.get(theta_set, 0.0)
            degree_omega = fuzzified_omega.get(omega_set, 0.0)

            firing_strength = max(degree_theta, degree_omega)
            if firing_strength == 0:
                continue
            consequent = rule["output"]
            z = (
                consequent["theta_coeff"] * crisp_theta
                + consequent["omega_coeff"] * crisp_omega
                + consequent["bias"]
            )

            traces.append(
                {
                    "rule_index": i,
                    "theta_set": theta_set,
                    "omega_set": omega_set,
                    "degree_theta": degree_theta,
                    "degree_omega": degree_omega,
                    "firing_strength": firing_strength,
                    "z": z,
                }
            )

            plot_rule_contributions(traces, crisp_theta, crisp_omega)


import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def plot_rule_contributions(trace_data, crisp_theta, crisp_omega):
    plt.title(
        f"Rule Contributions: Firing Strength and Z Output\n"
        f"crisp_theta = {crisp_theta:.3f}, crisp_omega = {crisp_omega:.3f}"
    )
    labels = [f"{t['theta_set']}/{t['omega_set']}" for t in trace_data]
    ws = [t["firing_strength"] for t in trace_data]
    zs = [t["z"] for t in trace_data]

    source_flags = [
        "theta" if t["degree_theta"] >= t["degree_omega"] else "omega"
        for t in trace_data
    ]
    colors = ["blue" if src == "theta" else "green" for src in source_flags]

    fig, ax1 = plt.subplots(figsize=(12, 6))

    bars = ax1.bar(range(len(labels)), ws, color=colors, alpha=0.7)

    ax2 = ax1.twinx()
    ax2.plot(range(len(labels)), zs, label="Z Output", color="red", marker="o")

    ax1.set_ylabel("Firing Strength")
    ax2.set_ylabel("Z Output")

    ax1.set_xticks(range(len(labels)))
    ax1.set_xticklabels(labels, rotation=45, ha="right")
    ax1.text(
        0.01,
        0.99,
        f"crisp_theta = {crisp_theta:.3f}\ncrisp_omega = {crisp_omega:.3f}",
        transform=ax1.transAxes,
        fontsize=11,
        verticalalignment="top",
        bbox=dict(facecolor="white", alpha=0.7, edgecolor="gray"),
    )

    # Annotate W values on top of bars
    for i, bar in enumerate(bars):
        height = bar.get_height()
        if height > 0:
            ax1.text(
                bar.get_x() + bar.get_width() / 2,
                height + 0.01,
                f"{height:.2f}",
                ha="center",
                va="bottom",
                fontsize=8,
                color="black",
            )

    # Legend
    theta_patch = mpatches.Patch(color="blue", label="Firing from degree_theta")
    omega_patch = mpatches.Patch(color="green", label="Firing from degree_omega")
    ax1.legend(handles=[theta_patch, omega_patch], loc="upper left")

    plt.title("Rule Contributions: Firing Strength and Z Output")
    plt.tight_layout()
    plt.show()

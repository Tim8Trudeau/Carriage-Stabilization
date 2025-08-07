"""
Orchestrates the Fuzzy Logic Controller (FLC) operations.

This module integrates the Fuzzifier, Rule Engine, and Defuzzifier to process
sensor inputs and compute a final motor command. It serves as the main
interface to the FLC system. It does not instanciate hardware.imu_driver or
hardware.pwm_driver.
"""

import logging
from typing import Dict, Any

from flc.fuzzifier import Fuzzifier
from flc.rule_engine import RuleEngine
from flc.defuzzifier import Defuzzifier

controller_log = logging.getLogger("controller")


class FLCController:
    """
    The main Fuzzy Logic Controller class.

    This class coordinates the entire fuzzy inference process, from receiving
    crisp inputs to producing a final crisp output.

    Attributes:
        fuzzifier (Fuzzifier): The fuzzifier instance.
        rule_engine (RuleEngine): The rule engine instance.
        defuzzifier (Defuzzifier): The defuzzifier instance.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initializes the FLC by setting up its components.

        Args:
            config (Dict[str, Any]): The full configuration dictionary, which
                contains parameters for all sub-modules.
        """
        mf_params = config.get("membership_functions", {})
        rule_base = config.get("rule_base", [])

        self.fuzzifier = Fuzzifier(mf_params)
        self.rule_engine = RuleEngine(rule_base)
        self.defuzzifier = Defuzzifier()
        controller_log.info("FLC Controller initialized and ready.")

    def calculate_motor_cmd(
        self, theta: float, omega: float, plot: bool = False
    ) -> float:
        """
        Executes one full cycle of the fuzzy inference system.

        Args:
            theta (float): The normalized angular position error.
            omega (float): The normalized angular velocity error.

        Returns:
            float: The calculated normalized motor command, in the range [-1.0, 1.0].
        """
        controller_log.debug(
            "--- FLC Cycle Start (theta=  %.3f, omega=  %.3f) ---", theta, omega
        )

        # 1. Fuzzification - Range for theta is [-1.5, +1.5] and omega is [-1.0, +1.0]
        fuzzified_theta = self.fuzzifier.fuzzify("theta", theta)
        fuzzified_omega = self.fuzzifier.fuzzify("omega", omega)

        # 2. Rule Evaluation
        rule_outputs = self.rule_engine.evaluate(
            fuzzified_theta, fuzzified_omega, theta, omega, plot=plot
        )

        # 3. Defuzzification
        motor_cmd = self.defuzzifier.defuzzify(rule_outputs)
        # print(f"motor_cmd=  {motor_cmd:.4f} ")
        controller_log.debug("--- FLC Cycle End (motor_cmd=  %.4f) ---", motor_cmd)
        return motor_cmd


import numpy as np
import matplotlib.pyplot as plt
import logging
from utils.logger import setup_logging


def plot_motor_cmd_vs_theta(controller, theta_range=(-1.5, 1.5), omega=0.0, num=200):
    """
    Sweeps theta across the specified range with fixed omega,
    calls controller.calculate_motor_cmd, and plots motor_cmd vs theta
    (theta on vertical axis).
    """
    thetas = np.linspace(theta_range[0], theta_range[1], num)
    motor_cmds = []
    for theta in thetas:
        cmd = controller.calculate_motor_cmd(theta, omega)
        motor_cmds.append(cmd)

    plt.figure(figsize=(8, 6))
    plt.plot(thetas, motor_cmds, color="blue")
    plt.xlabel("Theta")
    plt.ylabel("Motor Command")
    plt.title("Motor Command vs Theta (Omega = 0.0)")

    plt.grid(True)
    plt.tight_layout()
    plt.show()


# Example usage:
if __name__ == "__main__":
    import tomllib
    import os

    # Setup logging first
    setup_logging()
    main_log = logging.getLogger("main")
    # Load TOML config
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "..", "config", "flc_config.toml")
    config_path = os.path.normpath(config_path)

    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    controller = FLCController(config)
    plot_motor_cmd_vs_theta(controller)

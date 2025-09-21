
"""
Orchestrates the Fuzzy Logic Controller (FLC) operations.

This module integrates the Fuzzifier, Rule Engine, and Defuzzifier to process
sensor inputs and compute a final motor command. It serves as the main
interface to the FLC system. It does not instantiate hardware.imu_driver or
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
        rule_scaling = config.get("flc_scaling", {})
        mf_params = config.get("membership_functions", {})
        rule_base = config.get("rule_base", [])

        self.fuzzifier = Fuzzifier(mf_params)
        self.rule_engine = RuleEngine(rule_base, rule_scaling)
        self.defuzzifier = Defuzzifier()
        controller_log.info("FLC Controller initialized and ready.")

    def calculate_motor_cmd(self, theta: float, omega: float, plot: bool = False) -> float:
        """
        Executes one full cycle of the fuzzy inference system.

        Args:
            theta (float): The angular position error in radians (normalized to controller range).
            omega (float): The normalized angular velocity error [-1.0, +1.0].
            plot (bool): Optional passthrough for debugging visuals inside the rule engine.

        Returns:
            float: The calculated normalized motor command, in the range [-1.0, 1.0].
        """
        controller_log.debug(
            "--- FLC Cycle Start (theta= %.3f, omega= %.3f) ---", theta, omega
        )

        # 1) Fuzzification
        fuzzified_theta = self.fuzzifier.fuzzify("theta", theta)
        fuzzified_omega = self.fuzzifier.fuzzify("omega", omega)

        # 2) Rule Evaluation
        rule_outputs = self.rule_engine.evaluate(
            fuzzified_theta, fuzzified_omega, theta, omega, plot=plot
        )

        # 3) Defuzzification
        motor_cmd = self.defuzzifier.defuzzify(rule_outputs)
        controller_log.debug("--- FLC Cycle End (motor_cmd= %.4f) ---", motor_cmd)
        return motor_cmd

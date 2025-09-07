"""
Evaluates the fuzzy rule base to determine rule activation and output.

This module takes the fuzzified inputs (membership degrees) and applies them to a
set of Sugeno-type rules. For each rule, it calculates the firing strength (activation
level) and the crisp output value based on the rule's consequent function.
"""

import logging
from math import e
from typing import Dict, List, Tuple

rule_engine_log = logging.getLogger("rule_engine")
WZ_log = logging.getLogger("WZ_engine")


class RuleEngine:
    """
    Evaluates a Sugeno-type fuzzy rule base.
    Attributes:
        rules (List[Dict]): The list of rule definitions, where each rule has an
            'IF' part (antecedent) and a 'THEN' part (consequent).
    """

    def __init__(self, rule_base: List[Dict], rule_scaling: Dict):
        """
        Initializes the RuleEngine with a specific rule base.

        Args:
            rule_base (List[Dict]): A list of rules loaded from the config.
            rule_scaling (Dict): Parameters from config (flc_config.toml).
        """
        self.rules = rule_base
        scaling = rule_scaling or {}
        # Safe numeric defaults
        self.theta_scale_factor = float(scaling.get("THETA_SCALE_FACTOR", 1.0))
        self.omega_scale_factor = float(scaling.get("OMEGA_SCALE_FACTOR", 1.0))

        rule_engine_log.info("Rule Engine initialized with %d rules.", len(self.rules))
        rule_engine_log.info(
            "Using THETA_SCALE_FACTOR=%.2f, OMEGA_SCALE_FACTOR=%.2f",
            self.theta_scale_factor, self.omega_scale_factor)
        rule_engine_log.info(
            "W is rule firing strength and Z is the crisp output for the rule."
        )

    def evaluate(
        self,
        fuzzified_theta: Dict[str, float],
        fuzzified_omega: Dict[str, float],
        crisp_theta: float,
        crisp_omega: float,
        plot: bool = True,
    ) -> List[Tuple[float, float]]:
        """
        Evaluates all rules in the rule base.

        For each rule, it computes:
            - The firing strength (W), taken as the maximum of the membership
            degrees of the antecedents (fuzzy OR).
            - The crisp output (Z), calculated from a Sugeno-style linear equation.

        Z represents the corrective motor command contributed by a rule. The consequent
        is defined as a linear function of the crisp inputs (theta, omega), parameterized by:

            - theta_coeff: coefficient applied to theta (must be negative)
            - omega_coeff: coefficient applied to omega (must be negative)
            - bias: constant offset term

        Additional rule-level scale factors (THETA_SCALE_FACTOR, OMEGA_SCALE_FACTOR)
        are applied when calculating Z1 and Z2.

        Design constraint:
            Both theta_coeff and omega_coeff must remain negative to enforce
            negative feedback, ensuring the motor command acts to reduce
            positional error and restore the system toward the setpoint.

        Args:
            fuzzified_theta (Dict[str, float]): Membership degrees for theta.
            fuzzified_omega (Dict[str, float]): Membership degrees for omega.
            crisp_theta (float): The original normalized theta value.
            crisp_omega (float): The original normalized omega value.
            plot (bool, optional): If True, generates a plot of the rule firing
                strengths and outputs. Defaults to False.

        Returns:
            List[Tuple[float, float]]: A list of (W, Z) tuples, where W is the
            firing strength and Z is the crisp output for each active rule.
        """
        # Optionally plot the rule firing details.
        plot = False
        if plot:
            from utils.rule_trace import trace_rule_firing

            # Get the detailed trace of rule firing
            # This will include membership degrees, firing strengths, and Z outputs and
            # plots the results.
            trace_rule_firing(
                self.rules,
                fuzzified_theta,
                fuzzified_omega,
                crisp_theta,
                crisp_omega,
                plot=False,
            )

        # Log the rule firing details
        # Log the input values and their fuzzified membership degrees
        # This helps in debugging and understanding the rule evaluation process.
        rounded = {k: round(v, 3) for k, v in fuzzified_theta.items()}
        rule_engine_log.info("Theta: %.3f, fuzzy_Theta=%s", crisp_theta, rounded)

        rounded = {k: round(v, 3) for k, v in fuzzified_omega.items()}
        rule_engine_log.info("Omega: %.3f, fuzzy_Omega=%s", crisp_omega, rounded)


        # Initialize a list to hold the outputs of contributing rules.
        # An contributing rule is one that has a non-zero firing strength.
        active_rules_output = []
        for i, rule in enumerate(self.rules):
            antecedent = rule["rule"] # Rule antecedent: [theta_set, omega_set]
            theta_set = antecedent[0]
            omega_set = antecedent[1]

            # Get membership degrees, default to 0 if rule does not contribute.
            degree_theta = fuzzified_theta.get(theta_set, 0.0)
            degree_omega = fuzzified_omega.get(omega_set, 0.0)

            # Firing strength (W) is the fuzzy OR (max) of the rules membership degrees.
            firing_strength = max(degree_theta, degree_omega)

            if firing_strength > 0:
                consequent = rule["output"]
                z1 = (
                    consequent["theta_coeff"]
                    * self.theta_scale_factor
                    * degree_theta
                    * ((crisp_theta > 0) - (crisp_theta < 0)) #get sign of crisp_theta
                )
                z2 = (
                    consequent["omega_coeff"]
                    * self.omega_scale_factor
                    * degree_omega
                    * ((crisp_omega > 0) - (crisp_omega < 0)) #get sign of crisp_omega
                )
                z3 = consequent["bias"]
                z = z1 + z2 + z3

                active_rules_output.append((firing_strength, z))
                if degree_theta > degree_omega:
                    conseq = "theta_coeff"
                else:
                    conseq = "omega_coeff"

                WZ_log.debug(
                    "Rule# %d (theta_member %s) (omega_member %s) "
                    "Z=%.2f , Z1_theta=%.2f, Z2_omega=%.2f ",
                    i,
                    theta_set, omega_set,
                    z, z1, z2,
                )
                WZ_log.debug(
                    "crisp_theta= %.2f, degree_theta= %.2f, "
                    "degree_omega= %.2f, "
                    "W= %.2f |%s rule_consq= %.2f",
                    crisp_theta, degree_theta,
                    degree_omega,
                    firing_strength, conseq, consequent[conseq],
                )
            else:
                WZ_log.debug("Rule# %d W= %.2f", i, firing_strength)

        return active_rules_output

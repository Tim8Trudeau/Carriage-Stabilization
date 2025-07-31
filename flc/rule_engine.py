"""
Evaluates the fuzzy rule base to determine rule activation and output.

This module takes the fuzzified inputs (membership degrees) and applies them to a
set of Sugeno-type rules. For each rule, it calculates the firing strength (activation
level) and the crisp output value based on the rule's consequent function.
"""

import logging
from typing import Dict, List, Tuple

from _pytest.stash import T

rule_engine_log = logging.getLogger("rule_engine")


class RuleEngine:
    """
    Evaluates a Sugeno-type fuzzy rule base.

    Attributes:
        rules (List[Dict]): The list of rule definitions, where each rule has an
            'IF' part (antecedent) and a 'THEN' part (consequent).
    """

    def __init__(self, rule_base: List[Dict]):
        """
        Initializes the RuleEngine with a specific rule base.

        Args:
            rule_base (List[Dict]): A list of rules loaded from the JSON config.
        """
        self.rules = rule_base
        rule_engine_log.info("Rule Engine initialized with %d rules.", len(self.rules))

    def evaluate(
        self,
        fuzzified_theta: Dict[str, float],
        fuzzified_omega: Dict[str, float],
        crisp_theta: float,
        crisp_omega: float,
    ) -> List[Tuple[float, float]]:
        """
        Evaluates all rules in the rule base.

        For each rule, it computes the firing strength (W) and the crisp output (Z).
        The firing strength is the maximum of the membership degrees of the
        antecedents (fuzzy AND). The crisp output is calculated from the
        Sugeno-style linear equation.

        Args:
            fuzzified_theta (Dict[str, float]): Membership degrees for theta.
            fuzzified_omega (Dict[str, float]): Membership degrees for omega.
            crisp_theta (float): The original normalized theta value.
            crisp_omega (float): The original normalized omega value.

        Returns:
            List[Tuple[float, float]]: A list of (W, Z) tuples, where W is the
                firing strength and Z is the crisp output for each active rule.
        Options:
            plot (bool): If True, generates a plot of the rule firing strengths
                and outputs. Defaults to False.
        """
        plot = True
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
                plot=True,
            )

        # Log the rule firing details
        rule_engine_log.info(
            "W is rule firing strength and Z is the crisp output for the rule."
        )
        # Log the input values and their fuzzified membership degrees
        # This helps in debugging and understanding the rule evaluation process.
        rule_engine_log.info(
            "\nTheta: %.2f, fuzzy_Theta=%s\n Omega: %.2f, fuzzy_Omega=%s",
            crisp_theta,
            fuzzified_theta,
            crisp_omega,
            fuzzified_omega,
        )
        # Initialize a list to hold the outputs of active rules
        # An active rule is one that has a non-zero firing strength.
        active_rules_output = []
        for i, rule in enumerate(self.rules):
            antecedent = rule["rule"]
            theta_set = antecedent[0]
            omega_set = antecedent[1]

            # Get membership degrees, default to 0 if not active
            degree_theta = fuzzified_theta.get(theta_set, 0.0)
            degree_omega = fuzzified_omega.get(omega_set, 0.0)

            # Wrong- Firing strength (W) is the fuzzy AND (min) of the degrees
            # **** Need to change the rules to fuzzy OR (max) - done
            firing_strength = max(degree_theta, degree_omega)

            if firing_strength > 0:
                # Compute the crisp output (Z) for this rule using its Sugeno-style consequent.
                # The output Z represents the corrective motor command contributed by this rule.
                # The consequence is modeled as a linear function of the crisp inputs (theta and omega),
                # scaled by the rule-specific coefficients: theta_coeff, omega_coeff, and a constant bias term.
                # Typically, theta_coeff and omega_coeff have signs opposite to the input errors
                # so that the resulting output acts to reduce deviation and restore the system toward the setpoint.

                consequent = rule["output"]
                z1 = consequent["theta_coeff"] * degree_theta * crisp_theta
                z2 = consequent["omega_coeff"] * degree_omega * crisp_omega
                z3 = consequent["bias"]
                z = z1 + z2 + z3

                active_rules_output.append((firing_strength, z))
                rule_engine_log.debug(
                    "Rule %d fired: (IF theta is %s OR omega is %s) W=%.3f, Z=%.3f ",
                    i,
                    theta_set,
                    omega_set,
                    firing_strength,
                    z,
                )

        return active_rules_output

"""
Evaluates the fuzzy rule base to determine rule activation and output.

This module takes the fuzzified inputs (membership degrees) and applies them to a
set of Sugeno-type rules. For each rule, it calculates the firing strength (activation
level) and the crisp output value based on the rule's consequent function.
"""

import logging
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
            - The firing strength (W), taken as the minimum of the membership
              degrees of the antecedents (fuzzy AND).
            - The crisp output (Z), calculated from a Sugeno-style linear equation:

                    Z = a*theta + b*omega + c

              where:
                a = theta_coeff  (must be negative)
                b = omega_coeff  (must be negative)
                c = bias         (usually 0)

        IMPORTANT DESIGN NOTE:
            The original implementation incorrectly multiplied membership degrees
            and discrete sign() functions, which caused discontinuities and
            high-frequency "barcode" oscillations. This patched version correctly
            uses the crisp normalized inputs (already ∈ [-1,1]) directly in the
            Sugeno consequent.

        Args:
            fuzzified_theta (Dict[str, float]): Membership degrees for theta.
            fuzzified_omega (Dict[str, float]): Membership degrees for omega.
            crisp_theta (float): The original normalized theta value in [-1, 1].
            crisp_omega (float): The original normalized omega value in [-1, 1].
            plot (bool, optional): If True, generates a plot of the rule firing
                strengths and outputs. Defaults to False.

        Returns:
            List[Tuple[float, float]]: A list of (W, Z) tuples, where W is the
            firing strength and Z is the crisp output for each active rule.
        """

        # Optionally invoke detailed tracing (disabled by default for speed)
        plot = False
        if plot:
            from utils.rule_trace import trace_rule_firing
            trace_rule_firing(
                self.rules,
                fuzzified_theta,
                fuzzified_omega,
                crisp_theta,
                crisp_omega,
                plot=False,
            )

        # Log fuzzified values for debugging
        rounded = {k: round(v, 3) for k, v in fuzzified_theta.items()}
        rule_engine_log.info("Theta: %.3f, fuzzy_Theta=%s", crisp_theta, rounded)

        rounded = {k: round(v, 3) for k, v in fuzzified_omega.items()}
        rule_engine_log.info("Omega: %.3f, fuzzy_Omega=%s", crisp_omega, rounded)

        # Collect active rules
        active_rules_output = []

        for i, rule in enumerate(self.rules):
            antecedent = rule["rule"]     # [theta_set, omega_set]
            theta_set = antecedent[0]
            omega_set = antecedent[1]

            # Membership degrees (0 if not applicable)
            degree_theta = fuzzified_theta.get(theta_set, 0.0)
            degree_omega = fuzzified_omega.get(omega_set, 0.0)

            # Firing strength is the fuzzy AND (min) of the rules membership degrees.
            firing_strength = min(degree_theta, degree_omega)

            if firing_strength > 0:
                consequent = rule["output"]

                # Correct Sugeno linear consequent
                # Use crisp theta/omega (already normalized to [-1,1])
                z = (
                    consequent["theta_coeff"] * self.theta_scale_factor * crisp_theta
                    + consequent["omega_coeff"] * self.omega_scale_factor * crisp_omega
                    + consequent["bias"]
                )

                active_rules_output.append((firing_strength, z))

                # Detailed rule logging
                WZ_log.debug(
                    "Rule# %d (theta_set=%s, omega_set=%s) "
                    "Z=%.3f | theta_term=%.3f | omega_term=%.3f | bias=%.3f",
                    i,
                    theta_set,
                    omega_set,
                    z,
                    consequent["theta_coeff"] * crisp_theta,
                    consequent["omega_coeff"] * crisp_omega,
                    consequent["bias"],
                )
                WZ_log.debug(
                    "crisp_theta= %.3f, degree_theta= %.3f, degree_omega= %.3f, W= %.3f",
                    crisp_theta,
                    degree_theta,
                    degree_omega,
                    firing_strength,
                )
            else:
                WZ_log.debug("Rule# %d W= %.3f", i, firing_strength)

        return active_rules_output
# End of rule_engine.py

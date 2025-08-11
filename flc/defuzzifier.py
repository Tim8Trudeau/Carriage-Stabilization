"""
Computes the final crisp output from the aggregated fuzzy rule outputs.

This module implements the defuzzification process for a Sugeno-type system,
which calculates the weighted average of the outputs of all active fuzzy rules.
"""

import logging
from typing import List, Tuple

defuzzifier_log = logging.getLogger("defuzzifier")


class Defuzzifier:
    """Performs Sugeno-style weighted average defuzzification."""

    def __init__(self):
        """Initializes the Defuzzifier."""
        defuzzifier_log.info("Defuzzifier initialized.")

    def defuzzify(self, rule_outputs: List[Tuple[float, float]]) -> float:
        """
        Calculates the final crisp output value.

        The output is the weighted average of all rule outputs, calculated as:
        motor_cmd = (Σ(Wi * Zi)) / (Σ Wi)
        where Wi is the firing strength and Zi is the crisp output of rule i.

        Args:
            rule_outputs (List[Tuple[float, float]]): A list of (W, Z) tuples
                from the RuleEngine, where W is firing strength and Z is output.

        Returns:
            float: The final, crisp, normalized motor command value. Returns 0
                if no rules were activated.
        """
        numerator = 0.0
        denominator = 0.0001  # Avoid division by zero

        if not rule_outputs:
            defuzzifier_log.warning("No active rules to defuzzify. Outputting 0.")
            return 0.0

        for w, z in rule_outputs:
            numerator += w * z
            denominator += w
            # print(f"w=  {w:.2f}, z=  {z:.2f}, w*z=  {w * z:.2f}")
        final_output = numerator / denominator

        # print(f"numerator=  {numerator:.2f}, denominator=  {denominator:.2f}")
        if denominator == 0:
            defuzzifier_log.warning("Sum of firing strengths is zero. Outputting 0.")
            return 0.0

        # Clamp output to the normalized range [-1.0, 1.0] as a safety measure
        final_output_clamped = max(-1.0, min(1.0, final_output))

        if final_output != final_output_clamped:
            defuzzifier_log.warning(
                "Defuzzified output %.4f was outside range and clamped to %.4f.",
                final_output,
                final_output_clamped,
            )
            final_output = final_output_clamped

        defuzzifier_log.debug(
            "Defuzzified output: %.4f (from %d active rules)",
            final_output,
            len(rule_outputs),
        )
        return final_output

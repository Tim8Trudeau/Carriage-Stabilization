"""
Fuzzifies crisp input values into fuzzy sets using triangular membership functions.

This module takes normalized, crisp sensor readings (position and velocity) and
determines their degree of membership across a predefined set of fuzzy linguistic
variables (e.g., 'Small Error ClockWise').
"""

import logging
from typing import Dict, List

fuzzifier_log = logging.getLogger("fuzzifier")


class Fuzzifier:
    """
    Calculates membership degrees for crisp inputs.

    Attributes:
        membership_functions (Dict[str, Dict[str, List[float]]]): A dictionary
            holding the parameters (left, peak, right) for the triangular
            membership functions for each input variable ('theta', 'omega').
    """

    def __init__(self, mf_params: Dict[str, Dict[str, List[float]]]) -> None:
        """
        Initializes the Fuzzifier with membership function parameters.

        Args:
            mf_params (Dict[str, Dict[str, List[float]]]): Configuration for
                membership functions loaded from the JSON config file.
        """
        self.membership_functions = mf_params
        fuzzifier_log.info(
            "Fuzzifier initialized with %d theta and %d omega functions.",
            len(self.membership_functions.get("theta", {})),
            len(self.membership_functions.get("omega", {})),
        )

    def _triangle(self, x: float, params: List[float]) -> float:
        """
        Calculates the membership degree for a triangular function.

        Args:
            x (float): The crisp input value.
            params (List[float]): A list [a, b, c] where 'a' is the left foot,
                'b' is the peak, and 'c' is the right foot of the triangle.

        Returns:
            float: The degree of membership, from 0.0 to 1.0.
        """
        a, b, c = params
        if not a <= b <= c:
            raise ValueError(f"Invalid triangle params [{a}, {b}, {c}]")

        if x <= a or x >= c:
            return 0.0
        # left half rt triangle
        elif a < x <= b:
            return (x - a) / (b - a) if b - a > 0 else 1.0
        # right half rt triangle
        elif b < x < c:
            return (c - x) / (c - b) if c - b > 0 else 1.0
        return 0.0

    def _trapezoid(self, x: float, params: List[float]) -> float:
        """
        Calculates the membership degree for a trapezoidal function.

        Args:
            x (float): The crisp input value.
            params (List[float]): A list [a, b, c, d] where:
                - 'a' and 'd' are the bases (zero membership),
                - 'b' to 'c' is the top (membership = 1.0)

        Returns:
            float: Degree of membership (0.0 to 1.0)
        """
        a, b, c, d = params
        if not (a <= b <= c <= d):
            raise ValueError(f"Invalid trapezoid params [{a}, {b}, {c}, {d}]")

        if x <= a or x >= d:
            return 0.0
        elif b <= x <= c:
            return 1.0
        elif a < x < b:
            return (x - a) / (b - a)
        elif c < x < d:
            return (d - x) / (d - c)
        return 0.0

    def fuzzify(self, input_name: str, crisp_value: float) -> Dict[str, float]:
        """
        Fuzzifies a single crisp input value.

        Args:
            input_name (str): The name of the input variable ('theta' or 'omega').
            crisp_value (float): The normalized crisp value to fuzzify.

        Returns:
            Dict[str, float]: A dictionary mapping each fuzzy set name to its
                calculated membership degree. Only sets with a degree > 0
                are included.
        """
        if input_name not in self.membership_functions:
            raise KeyError(f"No membership functions defined for input '{input_name}'")

        fuzzified_output = {}
        for set_name, params in self.membership_functions[input_name].items():
            if len(params) == 3:
                degree = self._triangle(crisp_value, params)
            elif len(params) == 4:
                degree = self._trapezoid(crisp_value, params)
            else:
                raise ValueError(
                    f"Invalid membership function shape for '{set_name}': {params}"
                )
            if degree > 0:
                fuzzified_output[set_name] = degree

        # Format and log after collecting all outputs
        formatted_output = {k: f"{v:.3f}" for k, v in fuzzified_output.items()}
        if input_name == "omega":
            input_name = "O"
        fuzzifier_log.debug(
            "Fuzzified %s=  %.3f -> %s", input_name, crisp_value, formatted_output
        )
        return fuzzified_output

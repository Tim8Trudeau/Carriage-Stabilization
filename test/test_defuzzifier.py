import pytest
from flc.defuzzifier import Defuzzifier


@pytest.fixture
def defuzzifier():
    return Defuzzifier()


def test_defuzzifier_init(defuzzifier):
    assert defuzzifier is not None


def test_defuzzify_normal_case(defuzzifier):
    # Sum(W*Z) = (0.8 * 0.5) + (0.2 * -0.3) = 0.4 - 0.06 = 0.34
    # Sum(W) = 0.8 + 0.2 = 1.0
    # Result = 0.34 / 1.0 = 0.34
    rule_outputs = [(0.8, 0.5), (0.2, -0.3)]
    result = defuzzifier.defuzzify(rule_outputs)
    assert result == pytest.approx(0.34, abs=1e-3)


def test_defuzzify_no_rules(defuzzifier):
    rule_outputs = []
    result = defuzzifier.defuzzify(rule_outputs)
    assert result == 0.0


def test_defuzzify_zero_firing_strength(defuzzifier):
    # Sum(W*Z) = (0.0 * 2.5) = 0.0
    # Sum(W) = 0.0
    rule_outputs = [(0.0, 0.5), (0.0, -0.3)]
    result = defuzzifier.defuzzify(rule_outputs)
    assert result == 0.0


def test_defuzzify_output_clamping_high(defuzzifier):
    # Sum(W*Z) = (1.0 * 2.5) = 2.5
    # Sum(W) = 1.0
    # Result = 2.5, which should be clamped to 1.0
    rule_outputs = [(1.0, 2.5)]
    result = defuzzifier.defuzzify(rule_outputs)
    assert result == pytest.approx(1.0)


def test_defuzzify_output_clamping_low(defuzzifier):
    # Sum(W*Z) = (0.5 * -3.0) = -1.5
    # Sum(W) = 0.5
    # Result = -3.0, which should be clamped to -1.0
    rule_outputs = [(0.5, -3.0)]
    result = defuzzifier.defuzzify(rule_outputs)
    assert result == pytest.approx(-1.0)

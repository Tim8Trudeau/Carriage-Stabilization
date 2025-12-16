# tests/test_rule_engine.py

import math
import pytest
from flc.rule_engine import RuleEngine

# A minimal rule base for testing
RULES = [
    {"rule": ["NZ", "NZ"], "output": {"theta_coeff": -1, "omega_coeff": -1, "bias": 0}},
    {"rule": ["PZ", "PZ"], "output": {"theta_coeff": -2, "omega_coeff": -1, "bias": 0}},
]

SCALING = {
    "THETA_SCALE_FACTOR": 1.0,
    "OMEGA_SCALE_FACTOR": 1.0,
}

@pytest.fixture
def engine():
    return RuleEngine(RULES, SCALING)

def test_w_is_min_membership(engine):
    fuzz_th = {"NZ": 0.2}
    fuzz_om = {"NZ": 0.9}
    result = engine.evaluate(fuzz_th, fuzz_om, crisp_theta=0.5, crisp_omega=0.2)
    W, Z = result[0]
    assert W == pytest.approx(min(0.2, 0.9))

def test_missing_membership_defaults_to_zero(engine):
    fuzz_th = {"NZ": 0.3}
    fuzz_om = {}       # missing membership → default 0
    result = engine.evaluate(fuzz_th, fuzz_om, 0.5, 0.1)
    # NZ/NZ rule requires both memberships > 0 → should not fire
    assert result == []

def test_single_rule_output(engine):
    fuzz_th = {"NZ": 0.4}
    fuzz_om = {"NZ": 0.4}
    out = engine.evaluate(fuzz_th, fuzz_om, crisp_theta=-0.5, crisp_omega=-0.2)
    W, Z = out[0]
    # Z = a*theta + b*omega + c
    assert Z == pytest.approx((-1)*(-0.5) + (-1)*(-0.2))

def test_multiple_rules(engine):
    fuzz_th = {"NZ": 0.8, "PZ": 0.8}
    fuzz_om = {"NZ": 0.8, "PZ": 0.8}
    out = engine.evaluate(fuzz_th, fuzz_om, 0.3, 0.4)
    assert len(out) == 2     # both rules fire
    assert out[0][0] == pytest.approx(0.8)
    assert out[1][0] == pytest.approx(0.8)

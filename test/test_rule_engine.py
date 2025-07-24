import pytest
from flc.rule_engine import RuleEngine

@pytest.fixture
def basic_rule_engine():
    """Returns a RuleEngine with two simulated rules."""
    rule_base = [
        {
            "rule": ["ZERO_TH", "ZERO_OM"],
            "output": {"theta_coeff": 1.0, "omega_coeff": 0.5, "bias": 0.1}
        },
        {
            "rule": ["POS_TH", "ZERO_OM"],
            "output": {"theta_coeff": -1.0, "omega_coeff": 0.0, "bias": -0.5}
        }
    ]
    return RuleEngine(rule_base)

def test_rule_engine_init(basic_rule_engine):
    assert basic_rule_engine is not None
    assert len(basic_rule_engine.rules) == 2

def test_evaluate_no_active_rules(basic_rule_engine):
    # NEG_TH & NEG_OM are simulated member fuctions
    fuzz_theta = {"NEG_TH": 1.0}
    fuzz_omega = {"NEG_OM": 1.0}
    outputs = basic_rule_engine.evaluate(fuzz_theta, fuzz_omega, -0.5, -0.5)
    assert len(outputs) == 0

def test_evaluate_one_active_rule(basic_rule_engine):
    fuzz_theta = {"ZERO_TH": 0.8}
    fuzz_omega = {"ZERO_OM": 0.0} # was .6
    crisp_theta = 0.1
    crisp_omega = -0.1
    outputs = basic_rule_engine.evaluate(fuzz_theta, fuzz_omega, crisp_theta, crisp_omega)
    assert len(outputs) == 1
    
    # Firing strength W = max(0.8, 0.6) = 0.6
    w, z = outputs[0]
    assert w == pytest.approx(0.8)
    
    # Crisp output Z = 1.0 * 0.1 + 0.5 * (-0.1) + 0.1 = 0.1 - 0.05 + 0.1 = 0.15
    assert z == pytest.approx(0.15)

def test_evaluate_multiple_active_rules(basic_rule_engine):
    fuzz_theta = {"ZERO_TH": 0.3, "POS_TH": 0.2}
    fuzz_omega = {"ZERO_OM": 0.9}
    crisp_theta = 0.1
    crisp_omega = 0.0
    
    outputs = basic_rule_engine.evaluate(fuzz_theta, fuzz_omega, crisp_theta, crisp_omega)
    
    assert len(outputs) == 2
    
    # Rule 1
    w1 = max(0.8, 0.9)
    z1 = 1.0 * crisp_theta + 0.5 * crisp_omega + 0.1 # 0.1 + 0 + 0.1 = 0.2
    assert outputs[0] == pytest.approx((w1, z1))
    
    # Rule 2
    w2 = max(0.2, 0.9)
    z2 = -1.0 * crisp_theta + 0.0 * crisp_omega - 0.5 # -0.1 + 0 - 0.5 = -0.6
    assert outputs[1] == pytest.approx((w2, z2))


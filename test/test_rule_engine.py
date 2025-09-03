# tests/test_rule_engine.py
import pytest

# Use your real package import path:
from flc.rule_engine import RuleEngine


@pytest.fixture
def controller_params():
    # Exercise non-default scaling to verify scale factors are applied in Z.
    return {
        "THETA_SCALE_FACTOR": 1.0,
        "OMEGA_SCALE_FACTOR": 2.0,
    }


@pytest.fixture
def rule_base():
    # Two rules; coefficients are negative to conform with the design goal
    # of negative feedback described in the engine's docs.
    #
    # R1: ZERO_TH & ZERO_OM
    # R2: POS_TH  & ZERO_OM
    return [
        {
            "rule": ["ZERO_TH", "ZERO_OM"],
            "output": {"theta_coeff": -1.0, "omega_coeff": -0.5, "bias": 0.00},
        },
        {
            "rule": ["POS_TH", "ZERO_OM"],
            "output": {"theta_coeff": -0.3, "omega_coeff": -0.1, "bias": 0.00},
        },
    ]


@pytest.fixture
def engine(rule_base, controller_params):
    return RuleEngine(rule_base, controller_params)


def test_init_sets_rules_and_scales(engine, controller_params, rule_base):
    # Rules loaded
    assert hasattr(engine, "rules")
    assert len(engine.rules) == len(rule_base)

    # Scale factors attached from controller_params
    assert getattr(engine, "theta_scale_factor") == pytest.approx(
        controller_params["THETA_SCALE_FACTOR"]
    )
    assert getattr(engine, "omega_scale_factor") == pytest.approx(
        controller_params["OMEGA_SCALE_FACTOR"]
    )


def test_no_active_rules_returns_empty(engine):
    # Fuzzy labels do not match any antecedents in rule base -> no activations.
    fuzz_theta = {"NEG_TH": 1.0}
    fuzz_omega = {"NEG_OM": 0.7}
    outputs = engine.evaluate(fuzz_theta, fuzz_omega, crisp_theta=-0.5, crisp_omega=-0.3)
    assert outputs == []


def test_single_active_rule_values(engine):
    # ZERO_TH & ZERO_OM should fire R1 only
    fuzz_theta = {"ZERO_TH": 0.8}
    fuzz_omega = {"ZERO_OM": 0.6}
    crisp_theta = 0.2
    crisp_omega = -0.1

    outputs = engine.evaluate(fuzz_theta, fuzz_omega, crisp_theta, crisp_omega)
    assert len(outputs) == 2

    w, z = outputs[0]

    # W is fuzzy OR (max)
    assert w == pytest.approx(max(0.8, 0.6))

    # Z = (th_coeff * THETA_SF * deg_th * sign_of(crisp_th))
    #   + (om_coeff * OMEGA_SF * deg_om * sign_of(crisp_om))
    #   + bias
    # With R1 values: -1.0, -0.5, bias=+0.00; THETA_SF=1.0, OMEGA_SF=2.0
    z_expected = (-1.0 * 1.0 * 0.8 *(1)) + (-0.5 * 2.0 * 0.6 * (-1)) + 0.00
    # = (-0.8) + (+0.6) + 0.00 = -0.20
    assert z == pytest.approx(z_expected, abs=1e-6)


def test_multiple_active_rules_order_and_values(engine):
    # ZERO_TH (0.3) and POS_TH (0.7) with ZERO_OM (0.9) fires both rules.
    fuzz_theta = {"ZERO_TH": 0.3, "POS_TH": 0.7}
    fuzz_omega = {"ZERO_OM": 0.9}
    crisp_theta = 0.9
    crisp_omega = 0.1
    outputs = engine.evaluate(fuzz_theta, fuzz_omega, crisp_theta, crisp_omega)
    assert len(outputs) == 2

    # Rule 1 (ZERO_TH, ZERO_OM)
    w1_expected = max(0.3, 0.9)  # = 0.9
    # Z = (th_coeff * THETA_SF * deg_th * sign_of(crisp_th))
    #   + (om_coeff * OMEGA_SF * deg_om * sign_of(crisp_omega)
    z1_expected = (-1.0 * 1.0 * 0.3 * 1) + (-0.5 * 2.0 * 0.9 * 1) + 0.00
    # = -0.3 + 0.1 = -0.2
    assert outputs[0][0] == pytest.approx(w1_expected, abs=1e-12)
    assert outputs[0][1] == pytest.approx(z1_expected, abs=1e-12)

    # Rule 2 (POS_TH, ZERO_OM)
    w2_expected = max(0.7, 0.9)  # = 0.9
    z2_expected = (-0.3 * 1.0 * 0.7 * 1) + (-0.1 * 2.0 * 0.9 * 1) - 0.00
    # = -0.21 + 0.0 - 0.0 = -0.021
    assert outputs[1][0] == pytest.approx(w2_expected, abs=1e-12)
    assert outputs[1][1] == pytest.approx(z2_expected, abs=1e-12)


@pytest.mark.parametrize(
    "deg_th,deg_om,expected_w",
    [
        (0.2, 0.9, 0.9),   # omega dominates
        (0.95, 0.1, 0.95), # theta dominates
        (0.4, 0.4, 0.4),   # equal
    ],
)
def test_w_is_max_of_memberships(engine, deg_th, deg_om, expected_w):
    fuzz_theta = {"ZERO_TH": deg_th}
    fuzz_omega = {"ZERO_OM": deg_om}
    outputs = engine.evaluate(fuzz_theta, fuzz_omega, crisp_theta=0.0, crisp_omega=0.0)
    assert len(outputs) == 2
    w, _ = outputs[0]
    assert w == pytest.approx(expected_w, abs=1e-12)


def test_scale_factors_affect_z(engine, rule_base):
    # Compare two engines differing only in OMEGA scale factor.
    engine_a = RuleEngine(rule_base, {"THETA_SCALE_FACTOR": 1.0, "OMEGA_SCALE_FACTOR": 1.0})
    engine_b = RuleEngine(rule_base, {"THETA_SCALE_FACTOR": 1.0, "OMEGA_SCALE_FACTOR": 2.0})

    fuzz_theta = {"ZERO_TH": 0.0}
    fuzz_omega = {"ZERO_OM": 0.75}
    crisp_theta = 0.9
    crisp_omega = 0.4
    bias = rule_base[0]["output"]["bias"]

    out_a = engine_a.evaluate(fuzz_theta, fuzz_omega, crisp_theta, crisp_omega)
    out_b = engine_b.evaluate(fuzz_theta, fuzz_omega, crisp_theta, crisp_omega)

    assert len(out_a) == 2 and len(out_b) == 2

    # Only R1 fires; W unaffected by scale factor change.
    assert out_a[0][0] == pytest.approx(out_b[0][0])

    # The difference in Z should equal the change in the omega term:
    # ΔZ = om_coeff * (SF_b - SF_a) * deg_om * sign_of(crisp_om) + bias
    om_coeff = -0.5 # From rule
    delta_expected = om_coeff * (2.0 - 1.0) * 0.75 * 1 + bias # = -0.355
    omega_a = out_a[0][1]
    omega_b = out_b[0][1]
    assert (omega_b - omega_a) == pytest.approx(delta_expected, abs=1e-12)


def test_missing_membership_defaults_to_zero(engine):
    # Only theta membership provided for the R1 antecedent; omega degree defaults to 0.
    fuzz_theta = {"ZERO_TH": 0.6}
    fuzz_omega = {}
    crisp_theta = 0.2
    crisp_omega = -0.2

    outputs = engine.evaluate(fuzz_theta, fuzz_omega, crisp_theta, crisp_omega)
    assert len(outputs) == 1

    w, z = outputs[0]
    assert w == pytest.approx(0.6)  # max(0.6, 0.0)

    # z = (-1.0 * 1.0 * 0.6 * 1) + (-0.5 * 2.0 * 0.0 * -1) + 0.00
    #   = (-0.12) + 0 + 0.02 = -0.60
    assert z == pytest.approx(-0.60, abs=1e-12)


def test_zero_degrees_yield_no_output(engine):
    # Antecedent labels exist but both degrees are exactly zero -> rule does not contribute.
    fuzz_theta = {"ZERO_TH": 0.0}
    fuzz_omega = {"ZERO_OM": 0.0}
    outputs = engine.evaluate(fuzz_theta, fuzz_omega, crisp_theta=0.3, crisp_omega=0.4)
    assert outputs == []

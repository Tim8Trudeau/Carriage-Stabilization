import pytest
from flc.fuzzifier import Fuzzifier

@pytest.fixture
def basic_fuzzifier():
    """Returns a Fuzzifier instance with simple, clear membership functions."""
    mf_params = {
        'theta': {
            'ZERO': [-0.5, 0.0, 0.5],
            'POS': [0.0, 0.5, 1.0]
        }
    }
    return Fuzzifier(mf_params)

def test_fuzzifier_init(basic_fuzzifier):
    assert basic_fuzzifier is not None
    assert 'theta' in basic_fuzzifier.membership_functions

def test_triangle_membership_function(basic_fuzzifier):
    params = [-0.5, 0.0, 0.5] # ZERO set
    assert basic_fuzzifier._triangle(0.0, params) == pytest.approx(1.0)
    assert basic_fuzzifier._triangle(-0.25, params) == pytest.approx(0.5)
    results = basic_fuzzifier._triangle(-0.25, params)
    assert results == pytest.approx(0.5)
    assert basic_fuzzifier._triangle(-0.5, params) == pytest.approx(0.0)
    assert basic_fuzzifier._triangle(0.5, params) == pytest.approx(0.0)
    assert basic_fuzzifier._triangle(-1.0, params) == pytest.approx(0.0)
    assert basic_fuzzifier._triangle(1.0, params) == pytest.approx(0.0)

def test_fuzzify_single_activation(basic_fuzzifier):
    result = basic_fuzzifier.fuzzify('theta', -0.25)
    assert 'ZERO' in result
    assert result['ZERO'] == pytest.approx(0.5)
    assert 'POS' not in result

def test_fuzzify_multiple_activation(basic_fuzzifier):
    result = basic_fuzzifier.fuzzify('theta', 0.25)
    assert 'ZERO' in result
    assert result['ZERO'] == pytest.approx(0.5)
    assert 'POS' in result
    assert result['POS'] == pytest.approx(0.5)

def test_fuzzify_peak_activation(basic_fuzzifier):
    result = basic_fuzzifier.fuzzify('theta', 0.5)
    assert 'ZERO' not in result # Should be exactly 0
    assert 'POS' in result
    assert result['POS'] == pytest.approx(1.0)

def test_fuzzify_no_activation(basic_fuzzifier):
    result = basic_fuzzifier.fuzzify('theta', 0.8)
    assert 'ZERO' not in result
    assert 'POS' in result
    assert result['POS'] == pytest.approx(0.4) # (1.0 - 0.8) / (1.0 - 0.5)

def test_fuzzify_invalid_input_name(basic_fuzzifier):
    with pytest.raises(KeyError):
        basic_fuzzifier.fuzzify('nonexistent_input', 0.0)

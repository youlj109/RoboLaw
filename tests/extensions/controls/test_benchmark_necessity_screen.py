"""Evidence aggregation must not promote empty or safe references to necessity."""
import pytest
from scripts.extensions.controls.screen_benchmark_necessity import summarize


def sample(valid, gap, time):
    return dict(valid=valid, distance_m=gap, time_s=time)


def test_empty_evidence_is_not_success():
    with pytest.raises(ValueError, match='No counterfactual samples'):
        summarize([])


def test_safe_reference_is_not_effective_avoidance_evidence():
    result = summarize([sample(True, .08, 0), sample(True, .07, 1)])
    assert not result['requires_adjustment']
    assert result['first_violation'] is None


def test_first_event_is_not_confused_with_worst_event():
    rows = [sample(True, .08, 0), sample(False, .01, 1), sample(False, 0, 2)]
    result = summarize(rows)
    assert result['requires_adjustment']
    assert result['first_violation']['time_s'] == 1
    assert result['minimum']['time_s'] == 2
    assert result['violations'] == 2

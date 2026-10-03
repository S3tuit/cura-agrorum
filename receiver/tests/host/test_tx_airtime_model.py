"""Independent rational-time and integer-time airtime recovery checks.

These are design evidence. Production tests separately exercise the actual
ledger, codecs, SQLite owner, RTC receipts and scheduler.
"""
import pytest
from tests.support.models import tx_airtime as rational
from tests.support.models import tx_airtime_adversarial as adversarial


def test_finite_recovery_envelope_and_exact_clock_bounds():
    assert rational.recovery_envelopes()['comparisons'] == 235543
    assert rational.clock_edges()['exact_rational_checks'] == 10500


def test_directed_receipts_expiration_and_refunds():
    rational.directed()


def test_broken_design_controls_are_detected():
    assert len(rational.mutations()) == 3


def test_lost_save_restart_trace():
    result = rational.lost_save_restart_trace()
    assert result['old_admissions'] == 609
    assert result['old_nominal_rf_us'] > 36_000_000


@pytest.mark.parametrize('mode', ['mixed', 'restart', 'fallback', 'idle', 'variable'])
@pytest.mark.parametrize('seed', range(4))
def test_rational_generated_histories(mode, seed):
    rational.random_trial(4004 + seed, 200, mode)


@pytest.mark.parametrize('scenario', list(adversarial.SCENARIOS))
def test_independent_integer_adversarial_scenarios(scenario):
    result = adversarial.attempt(adversarial.SCENARIOS[scenario])
    assert result['violation'] is None, result
    assert result['max_rolling_rf_us'] <= 36_000_000


@pytest.mark.parametrize('seed', range(4))
def test_integer_generated_histories(seed):
    result = adversarial.attempt(adversarial.random_run, 1_000_003 + seed, 2)
    assert result['violation'] is None, result


@pytest.mark.parametrize('control', list(adversarial.CONTROLS))
def test_adversarial_negative_controls(control):
    ctl = adversarial.CONTROLS[control]
    hits = [adversarial.attempt(fn, ctl)['violation'] for fn in adversarial.SCENARIOS.values()]
    if not any(hits):
        hits = [adversarial.attempt(adversarial.random_run, 7+seed, 4, ctl)['violation']
                for seed in range(20)]
    assert any(hits), control

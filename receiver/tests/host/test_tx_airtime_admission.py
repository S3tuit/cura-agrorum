"""Production admission boundaries, conservative refunds and correlated saves."""
import pytest
from cura_receiver.tx_airtime import AirtimeReason as R, TxCertainty
from cura_receiver.generated.receiver_enums_generated import RtcHealth
from tests.support.builders.persistence_control import state


def ready(component):
    policy, _, _, clock, _ = component(initial_state=state())
    assert policy.recover(deadline_monotonic_us=5_000_000).reason is R.STATE_READY
    return policy, clock


def test_snapshot_descheduling_cannot_mix_correlations(airtime_component, monkeypatch):
    policy, clock = ready(airtime_component)
    policy.report_tx(policy.try_spend().token, TxCertainty.STARTED)
    deadlines = policy._ledger.deadlines.copy()
    encode = policy._ledger.snapshot_entries
    def delayed(now):
        clock.advance_elapsed_us(30_000_000)
        return encode(now)
    monkeypatch.setattr(policy._ledger, 'snapshot_entries', delayed)
    assert policy.save(deadline_monotonic_us=100_000_000).reason is R.STATE_READY
    assert policy.state.airtime_snapshot.utc_us == 0
    assert policy._ledger.deadlines == deadlines


def test_equality_refund_and_duplicate_token(airtime_component):
    policy, _ = ready(airtime_component)
    policy.longest_packet_charge_us = 100_000
    for _ in range(18):
        policy.report_tx(policy.try_spend(100_000).token, TxCertainty.STARTED)
    final = policy.try_spend(100_000)
    assert policy.used_since_save_us == 1_900_000 and policy.save_required
    assert policy.try_spend().reason is R.SAVE_REQUIRED
    policy.report_tx(final.token, TxCertainty.NOT_STARTED)
    assert not policy.save_required and policy.available_charge_us == 200_000
    with pytest.raises(ValueError):
        policy.report_tx(final.token, TxCertainty.NOT_STARTED)


def test_submission_deadline_leaves_room_for_packet_completion(airtime_component):
    policy, clock = ready(airtime_component)
    before = clock.now_monotonic_us()
    spent = policy.try_spend()
    # Slow-clock physical bound plus the conservative charge fits the envelope.
    assert (spent.submission_deadline_monotonic_us-before)*1_000_000 <= (250_000-67_866)*996_300


def test_time_trust_loss_does_not_rebase_or_revoke_live_accounting(airtime_component):
    policy, _ = ready(airtime_component)
    policy.report_tx(policy.try_spend().token, TxCertainty.STARTED)
    deadlines, generation = policy._ledger.deadlines.copy(), policy.state.generation
    policy.update_time(None, rtc_health=RtcHealth.MISSING)
    assert policy.available_charge_us == 2_000_000-67_866
    assert policy._ledger.deadlines == deadlines and policy.state.generation == generation


def test_late_refund_cannot_reopen_saved_group(airtime_component):
    policy, _ = ready(airtime_component)
    spent = policy.try_spend()
    assert policy.save(deadline_monotonic_us=5_000_000).reason is R.STATE_READY
    with pytest.raises(ValueError):
        policy.report_tx(spent.token, TxCertainty.NOT_STARTED)
    assert not policy.group_outstanding


def test_generation_exhaustion_fails_closed(airtime_component):
    policy, _, _, _, _ = airtime_component(initial_state=state(generation=(1 << 63)-1))
    assert policy.recover(deadline_monotonic_us=5_000_000).reason is R.INVALID_STATE
    assert policy.available_charge_us == 0

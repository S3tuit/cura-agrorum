"""Exact covering receipts close groups; failures cannot create fresh allowance."""
from dataclasses import replace
import pytest
from cura_receiver.tx_airtime import AirtimeReason as R, TxCertainty
from cura_receiver.generated.receiver_enums_generated import PersistenceControlPurpose as Purpose
from tests.support.builders.persistence_control import state
from tests.support.coordination.state_commit import LostStateReply


def ready(component):
    policy, worker, database, clock, loaded = component(initial_state=state())
    assert policy.recover(deadline_monotonic_us=5_000_000).reason is R.STATE_READY
    return policy, worker, clock


def spend(policy, count):
    for _ in range(count):
        result = policy.try_spend()
        assert result.reason is R.ALLOWED
        policy.report_tx(result.token, TxCertainty.STARTED)


def test_threshold_28_29_30_and_group_closure(airtime_component):
    policy, _, clock = ready(airtime_component)
    spend(policy, 28)
    assert not policy.save_required
    spend(policy, 1)
    assert policy.used_since_save_us == 1_968_114 and policy.save_required
    assert policy.try_spend().reason is R.SAVE_REQUIRED
    old = policy._ledger.current_entry
    deadline = policy._ledger.deadlines[old]
    assert policy.save(deadline_monotonic_us=5_000_000).reason is R.STATE_READY
    assert policy.used_since_save_us == 0 and not policy.group_outstanding
    assert policy._ledger.deadlines[old] == deadline
    spend(policy, 1)
    assert policy._ledger.current_entry != old


def test_failed_save_preserves_counter_and_barrier(airtime_component):
    policy, _, _ = ready(airtime_component)
    spend(policy, 29)
    assert policy.save(deadline_monotonic_us=0).reason is R.PERSISTENCE_FAILED
    assert policy.used_since_save_us == 1_968_114
    assert policy.try_spend().reason is R.SAVE_REQUIRED
    assert policy.save(deadline_monotonic_us=5_000_000).reason is R.STATE_READY


@pytest.mark.parametrize('installed', [False, True])
def test_unknown_save_blocks_tx_until_exact_receipt(airtime_component, installed):
    policy, worker, clock = ready(airtime_component)
    spend(policy, 29)
    channel = LostStateReply(worker.control, installed=installed)
    policy.owner._control = channel
    assert policy.save(deadline_monotonic_us=5_000_000).reason is R.PERSISTENCE_PENDING
    frozen = policy.owner.pending.requested
    assert policy.try_spend().reason is R.PERSISTENCE_PENDING
    channel.fail_load = True
    assert policy.maintain(deadline_monotonic_us=5_000_000).reason is R.PERSISTENCE_PENDING
    assert policy.owner.pending.requested is frozen
    channel.fail_load = False
    update = policy.maintain(deadline_monotonic_us=5_000_000)
    assert update.reason is (R.STATE_READY if installed else R.PERSISTENCE_FAILED)
    assert policy.used_since_save_us == (0 if installed else 1_968_114)
    assert policy.save_required is (not installed)


@pytest.mark.parametrize('partial', [False, True])
def test_rtc_receipt_closes_full_or_partial_usage_exactly_once(airtime_component, partial):
    policy, _, clock = ready(airtime_component)
    # Inject a larger allowed accounting group to check the normative 0.8/1.2-s
    # receipt example independently of today's fixed-size ACK packet.
    policy._ledger.reserve(800_000, clock.now_monotonic_us())
    old = policy._ledger.current_entry
    requested = policy.snapshot(provenance=None, snapshot_monotonic_us=100,
                                snapshot_utc_us=0, previous_state=policy.state)
    if partial:
        # Extra uncovered usage before submission; no TX occurs during commit.
        policy._ledger.reserve(400_000, 100)
    receipt = policy.owner.commit(requested, deadline_monotonic_us=5_000_000,
                                  purpose=Purpose.RTC_PROVENANCE)
    policy.snapshot_receipt(receipt)
    assert policy.used_since_save_us == (400_000 if partial else 0)
    assert policy._ledger.current_entry is None
    saved_deadline = policy._ledger.deadlines[old]
    policy.snapshot_receipt(receipt)
    assert policy.used_since_save_us == (400_000 if partial else 0)
    spend(policy, 1)
    assert policy._ledger.current_entry != old
    assert policy._ledger.deadlines[old] == saved_deadline


def test_abandoned_rtc_snapshot_does_not_credit_or_permanently_block(airtime_component):
    policy, _, _ = ready(airtime_component)
    spend(policy, 1)
    policy.snapshot(provenance=None, snapshot_monotonic_us=100,
                    snapshot_utc_us=0, previous_state=policy.state)
    assert policy.available_charge_us == 0
    policy.snapshot_receipt(None)
    assert policy.used_since_save_us == 67_866
    spend(policy, 1)


def test_foreign_receipt_rejected(airtime_component):
    policy, _, _ = ready(airtime_component)
    from cura_receiver.communicator_state_owner import StateCommitReceipt
    requested = policy.snapshot(provenance=None, snapshot_monotonic_us=100,
                                snapshot_utc_us=0, previous_state=policy.state)
    with pytest.raises(ValueError):
        policy.snapshot_receipt(StateCommitReceipt(replace(requested)))


def test_full_rtc_save_closes_zero_usage_group_left_by_refund(airtime_component):
    policy, _, _ = ready(airtime_component)
    result = policy.try_spend()
    policy.report_tx(result.token, TxCertainty.NOT_STARTED)
    assert policy.used_since_save_us == 0 and policy.group_outstanding
    old = policy._ledger.current_entry
    requested = policy.snapshot(provenance=None, snapshot_monotonic_us=100,
                                snapshot_utc_us=0, previous_state=policy.state)
    receipt = policy.owner.commit(requested, deadline_monotonic_us=5_000_000,
                                  purpose=Purpose.RTC_PROVENANCE)
    policy.snapshot_receipt(receipt)
    assert not policy.group_outstanding
    spend(policy, 1)
    assert policy._ledger.current_entry != old

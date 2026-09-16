"""Tentative charge, certainty, settlement and correlation changes."""

from dataclasses import replace

import pytest

from cura_receiver.airtime_ledger import AirtimeCorrelation
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.generated.receiver_entities_generated import (
    TxAirtimeBucketV1 as Bucket,
)
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth as RH,
    SystemTimeQuality as Q,
)
from cura_receiver.time_observations import TrustedTimeSample
from cura_receiver.tx_airtime import AirtimeReason as R, TxCertainty as C
from tests.support.builders.persistence_control import state
from tests.support.coordination.state_commit import LostStateReply


def acquired(airtime_component, baseline=0):
    original = (
        state()
        if not baseline
        else state(buckets=(Bucket(baseline, 3_780_000_000),) + (Bucket(0, 0),) * 63)
    )
    policy, worker, path, clock, _ = airtime_component(initial_state=original)
    assert policy.acquire_grant(deadline_monotonic_us=5_000_100).reason is R.ALLOWED
    return policy, worker, path, clock


def settle(policy, clock, *, precharge=False, deadline=None):
    return policy.settle(
        precharge=precharge,
        deadline_monotonic_us=(
            clock.now_monotonic_us() + 5_000_000 if deadline is None else deadline
        ),
    )


# Only definite non-start reclaims a tentative spend; loaded baseline is unchanged in every outcome.
@pytest.mark.parametrize("certainty", [C.NOT_STARTED, C.STARTED, C.UNCERTAIN, None])
@pytest.mark.parametrize("baseline", [0, 3_000_000])
def test_certainty_and_exact_settlement(airtime_component, certainty, baseline):
    policy, _, _, clock = acquired(airtime_component, baseline)
    token = policy.try_spend().token
    assert token is not None
    assert policy.available_charge_us == 8_000_000 - baseline - 67_866
    assert (
        policy.total_used == 8_000_000
    )  # The durable ceiling remains charged until settlement.
    if certainty is not None:
        policy.report_tx(token, certainty)
    expected = baseline + (0 if certainty is C.NOT_STARTED else 67_866)
    assert settle(policy, clock).reason is R.STATE_READY
    assert policy.total_used == expected and policy.available_charge_us == 0
    assert policy.state.generation == 3
    assert sum(b.charged_airtime_us for b in policy.state.buckets) == expected
    assert policy.state.buckets[0] == (
        Bucket(expected, 3_780_000_000) if expected else Bucket(0, 0)
    )


# At a bucket edge one atomic generation keeps exact possible use and precharges the next minute.
def test_atomic_settlement_and_next_bucket_grant(airtime_component):
    policy, _, _, clock = acquired(airtime_component, 3_000_000)
    policy.report_tx(policy.try_spend().token, C.STARTED)
    policy.report_tx(policy.try_spend().token, C.UNCERTAIN)
    policy.try_spend()  # No terminal outcome is also a possible transmission.
    clock.advance_elapsed_us(60_000_000)
    assert policy.try_spend().reason is R.GRANT_EXPIRED
    assert settle(policy, clock, precharge=True).reason is R.ALLOWED
    assert policy.state.buckets[:2] == (
        Bucket(3_203_598, 3_780_000_000),
        Bucket(8_000_000, 3_840_000_000),
    )
    assert policy.total_used == 11_203_598 and policy.available_charge_us == 8_000_000
    assert policy.state.generation == 3


# Foreign, duplicate and settled tokens cannot manufacture allowance or change frozen canonical bytes.
def test_spend_token_ownership_and_late_certainty(airtime_component):
    policy, _, _, clock = acquired(airtime_component)
    other, _, _, _ = acquired(airtime_component)
    token = policy.try_spend().token
    with pytest.raises(ValueError):
        other.report_tx(token, C.NOT_STARTED)
    policy.report_tx(token, C.NOT_STARTED)
    assert policy.available_charge_us == 8_000_000
    with pytest.raises(ValueError):
        policy.report_tx(token, C.NOT_STARTED)
    token = policy.try_spend().token
    assert settle(policy, clock).reason is R.STATE_READY
    with pytest.raises(ValueError):
        policy.report_tx(token, C.NOT_STARTED)
    assert policy.total_used == 67_866 and policy.available_charge_us == 0


# Definite settlement failure retains the full authoritative precharge; resumption uses the original deadline.
@pytest.mark.parametrize("expired", [False, True])
def test_definite_settlement_failure_resumption(airtime_component, expired):
    policy, _, _, clock = acquired(airtime_component)
    policy.report_tx(policy.try_spend().token, C.STARTED)
    original = policy.state
    if expired:
        clock.advance_elapsed_us(60_000_000)
    assert (
        settle(policy, clock, precharge=True, deadline=0).reason is R.PERSISTENCE_FAILED
    )
    assert policy.state is original and policy.total_used == 8_000_000
    assert policy.available_charge_us == (0 if expired else 7_932_134)


# Unknown settlement blocks TX until exact old/new bytes resolve which generation is authoritative.
@pytest.mark.parametrize("installed", [False, True])
@pytest.mark.parametrize("precharge", [False, True])
def test_unknown_settlement_exact_resolution(airtime_component, installed, precharge):
    policy, worker, _, clock = acquired(airtime_component)
    policy.report_tx(policy.try_spend().token, C.UNCERTAIN)
    original = policy.state
    channel = LostStateReply(worker.control, installed=installed)
    policy.owner = CommunicatorStateOwner(control=channel, initial_state=original)
    assert settle(policy, clock, precharge=precharge).reason is R.PERSISTENCE_PENDING
    assert policy.available_charge_us == 0 and policy.try_spend().token is None
    pending = policy.owner.pending
    channel.fail_load = True
    assert (
        policy.reconcile(deadline_monotonic_us=5_000_100).reason
        is R.PERSISTENCE_PENDING
    )
    assert policy.owner.pending is pending
    channel.fail_load = False
    result = policy.reconcile(deadline_monotonic_us=5_000_100)
    assert result.reason is (
        R.STATE_READY if installed and not precharge else R.ALLOWED
    )
    assert policy.state.generation == (3 if installed else 2)
    assert policy.total_used == (67_866 if installed and not precharge else 8_000_000)
    assert policy.available_charge_us == (
        0 if installed and not precharge else 7_932_134
    )


# A lost-trust episode freezes even an unchanged UTC offset until a new durable state transition.
def test_trust_loss_cannot_reopen_grant_via_recovery(airtime_component):
    policy, _, _, clock = acquired(airtime_component)
    policy.try_spend()
    policy.update_time(None, rtc_health=RH.MISSING)
    assert policy.try_spend().reason is R.UNTRUSTED_TIME
    clock.advance_elapsed_us(10_000_000)
    policy.update_time(
        AirtimeCorrelation(
            TrustedTimeSample(
                clock.now_monotonic_us(), 10_000_000, 1, Q.NETWORK_SYNCED, 2
            ),
            2,
            20_000_100,
        ),
        rtc_health=RH.MISSING,
    )
    assert policy.available_charge_us == 0
    assert policy.recover(deadline_monotonic_us=20_000_100).reason is R.GRANT_REQUIRED
    assert policy.reconcile(deadline_monotonic_us=20_000_100).reason is R.GRANT_REQUIRED
    assert policy.available_charge_us == 0
    assert settle(policy, clock, precharge=True).reason is R.ALLOWED
    assert policy.available_charge_us == 7_932_134 and policy.state.generation == 3


# Forward/backward trusted-offset changes freeze the old grant and preserve its original charge identity.
@pytest.mark.parametrize("offset_change", [-30_000_000, 30_000_000])
def test_new_offset_requires_durable_transition(airtime_component, offset_change):
    policy, _, _, clock = acquired(airtime_component)
    policy.try_spend()
    clock.advance_elapsed_us(10_000_000)
    policy.update_time(
        AirtimeCorrelation(
            TrustedTimeSample(
                clock.now_monotonic_us(),
                10_000_000 + offset_change,
                31_000_000,
                Q.RTC_HOLDOVER,
                2,
            ),
            2,
            20_000_100,
        ),
        rtc_health=RH.PRESENT,
    )
    assert policy.available_charge_us == 0
    assert settle(policy, clock, precharge=True).reason is R.ALLOWED
    charges = {
        b.expires_at_utc_us: b.charged_airtime_us
        for b in policy.state.buckets
        if b.charged_airtime_us
    }
    assert 3_780_000_000 in charges
    if offset_change < 0:
        assert charges == {3_720_000_000: 8_000_000, 3_780_000_000: 67_866}
    else:
        assert charges == {3_780_000_000: 8_000_000}


# Exhaustion never creates another spend; an exact same-bucket settlement cannot reopen already used airtime.
def test_exhausted_allowance_stays_exhausted(airtime_component):
    policy, _, _, clock = acquired(airtime_component)
    for _ in range(117):
        policy.report_tx(policy.try_spend().token, C.STARTED)
    assert policy.try_spend().reason is R.BUDGET_EXHAUSTED
    assert settle(policy, clock, precharge=True).reason is R.BUDGET_EXHAUSTED
    assert policy.total_used == 7_940_322 and policy.available_charge_us == 0
    assert policy.state.buckets[0] == Bucket(7_940_322, 3_780_000_000)
    assert (
        policy.acquire_grant(deadline_monotonic_us=5_000_100).reason
        is R.BUDGET_EXHAUSTED
    )

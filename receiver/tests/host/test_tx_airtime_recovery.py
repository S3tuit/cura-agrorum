"""Generation-zero policy and actual SQLite recovery transactions."""

from dataclasses import replace
import hashlib
import sqlite3

import pytest

from cura_receiver.airtime_ledger import AirtimeCorrelation
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.generated.receiver_entities_generated import (
    communicator_state_v1_parameters,
    AirtimeSnapshotV1,
)
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth as RH,
    SystemTimeQuality as Q,
)
from cura_receiver.persistence_control_values import CommunicatorStateCondition as C
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.time_observations import TrustedTimeSample
from cura_receiver.tx_airtime import TxAirtimePolicy, AirtimeReason as R
from tests.support.builders.persistence import INSTANCE
from tests.support.builders.persistence_control import state, synthetic
from tests.support.coordination.persistence_worker import (
    CheckedPersistenceWorker,
    prepare_worker_files,
)
from tests.support.coordination.state_commit import LostStateReply
from tests.support.fakes.os_clock import FakeOsClock


def recover(policy, clock):
    return policy.recover(deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000)


# Missing/corrupt history starts without allowance and installs the literal worst-case ledger atomically.
@pytest.mark.parametrize("condition", [C.MISSING, C.CORRUPT])
def test_synthetic_recovery_exact_durable_history(airtime_component, condition):
    policy, _, database, clock, _ = airtime_component(condition)
    assert policy.state is None and policy.total_used is None
    assert policy.recover(deadline_monotonic_us=0).reason is R.PERSISTENCE_FAILED
    assert policy.state is None
    assert recover(policy, clock).reason is R.STATE_READY
    assert policy.state == replace(synthetic(), airtime_snapshot=AirtimeSnapshotV1(0, 1))
    assert policy.total_used == 36_000_000
    with sqlite3.connect(database) as connection:
        assert connection.execute(
            "SELECT * FROM communicator_state"
        ).fetchone() == communicator_state_v1_parameters(replace(synthetic(), airtime_snapshot=AirtimeSnapshotV1(0, 1)))
        archived = connection.execute(
            "SELECT observed_singleton_id, observed_state_format_version, observed_generation, observed_state_blob, observed_state_sha256 FROM quarantined_communicator_states ORDER BY quarantined_state_id"
        ).fetchall()
        assert archived == (
            [(None, "bad", 1.5, 42, None), (2, 1, 0, b"bad", b"bad")]
            if condition is C.CORRUPT
            else []
        )
        assert connection.execute("PRAGMA integrity_check").fetchone() == ("ok",)


# Incompatible history waits a full conservative hour beginning only after confirmed TX inability.
@pytest.mark.parametrize("condition", [C.UNSUPPORTED_VERSION, C.POLICY_MISMATCH])
def test_incompatible_wait_start_boundary_and_atomic_archive(
    airtime_component, condition
):
    policy, _, database, clock, _ = airtime_component(condition)
    clock.advance_elapsed_us(10_000_000)
    assert recover(policy, clock).reason is R.RECOVERY_WAIT
    policy.confirm_transmitter_disabled()
    clock.advance_elapsed_us(3_613_319_999)
    assert recover(policy, clock).reason is R.RECOVERY_WAIT
    policy.confirm_transmitter_disabled()  # Duplicate confirmation does not restart the proof.
    clock.advance_elapsed_us(1)
    assert recover(policy, clock).reason is R.STATE_READY
    assert policy.state.generation == 1 and policy.total_used == 0
    assert all(b.charged_airtime_us == 0 for b in policy.state.buckets)
    with sqlite3.connect(database) as connection:
        assert connection.execute(
            "SELECT count(*) FROM quarantined_communicator_states"
        ).fetchone() == (1,)
        assert connection.execute(
            "SELECT generation FROM communicator_state"
        ).fetchone() == (1,)


# A replacement process loses the no-TX interval proof even when the Linux monotonic clock continues.
def test_process_restart_restarts_entire_incompatible_wait(airtime_component):
    policy, worker, _, clock, loaded = airtime_component(C.UNSUPPORTED_VERSION)
    policy.confirm_transmitter_disabled()
    clock.advance_elapsed_us(3_613_320_000)
    replacement = TxAirtimePolicy(
        state_owner=CommunicatorStateOwner.from_load(
            control=worker.control, loaded=loaded
        ),
        clock=clock,
    )
    replacement.update_time(
        AirtimeCorrelation(
            TrustedTimeSample(
                clock.now_monotonic_us(), 3_613_320_000, 1, Q.RTC_HOLDOVER, 2
            ),
            2,
            10_000_000_100,
        ),
        rtc_health=RH.PRESENT,
    )
    assert recover(replacement, clock).reason is R.RECOVERY_WAIT
    replacement.confirm_transmitter_disabled()
    clock.advance_elapsed_us(3_613_319_999)
    assert recover(replacement, clock).reason is R.RECOVERY_WAIT
    clock.advance_elapsed_us(1)
    assert recover(replacement, clock).reason is R.STATE_READY


# Missing/corrupt and waited incompatible state can recover without UTC.
@pytest.mark.parametrize("condition", [C.MISSING, C.CORRUPT, C.UNSUPPORTED_VERSION])
def test_recovery_without_live_trust(airtime_component, condition):
    policy, _, _, clock, _ = airtime_component(condition)
    policy.confirm_transmitter_disabled()
    clock.advance_elapsed_us(3_613_320_000)
    policy.update_time(None, rtc_health=RH.MISSING)
    assert recover(policy, clock).reason is R.STATE_READY
    assert policy.state.airtime_snapshot is None
    assert policy.state.last_observed_system_time_quality is Q.UNTRUSTED
    assert policy.state.rtc_provenance is None
    assert policy.state.last_observed_rtc_health is RH.MISSING
    assert policy.total_used == (0 if condition is C.UNSUPPORTED_VERSION else 36_000_000)


# Lost recovery replies retain exact requested bytes, including while snapshot time advances before retry.
@pytest.mark.parametrize("installed", [False, True])
def test_unknown_recovery_reconciles_exact_request(airtime_component, installed):
    policy, worker, database, clock, loaded = airtime_component(C.CORRUPT)
    channel = LostStateReply(worker.control, installed=installed)
    policy.owner = CommunicatorStateOwner.from_load(control=channel, loaded=loaded)
    assert recover(policy, clock).reason is R.PERSISTENCE_PENDING
    requested = policy.owner.pending.requested
    channel.fail_load = True
    clock.advance_elapsed_us(100)
    assert recover(policy, clock).reason is R.PERSISTENCE_PENDING
    assert policy.state is None and policy.owner.pending.requested is requested
    channel.fail_load = False
    assert recover(policy, clock).reason is R.STATE_READY
    assert policy.state is requested
    assert all(value is requested for value in channel.requests)
    with sqlite3.connect(database) as connection:
        assert connection.execute(
            "SELECT count(*) FROM quarantined_communicator_states"
        ).fetchone() == (2,)
        assert connection.execute(
            "SELECT * FROM communicator_state"
        ).fetchone() == communicator_state_v1_parameters(requested)


# Airtime no longer adds retention durations to UTC, even at the signed UTC limit.
def test_recovery_at_utc_limit(airtime_component):
    policy, _, _, clock, _ = airtime_component(utc=(1 << 63) - 1)
    assert recover(policy, clock).reason is R.STATE_READY
    assert policy.total_used == 36_000_000
    assert policy.state.airtime_snapshot.utc_us == (1 << 63) - 1

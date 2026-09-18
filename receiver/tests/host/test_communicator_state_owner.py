"""Complete-state authority through an actual worker and file-backed SQLite."""

from dataclasses import replace
import sqlite3

import pytest

from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.generated.receiver_entities_generated import (
    communicator_state_v1_parameters,
)
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as CD,
    CommunicatorStateCondition as C,
    CommunicatorStateLoadStatus as LS,
)
from cura_receiver.receiver_startup import ReceiverInstanceStart
from tests.support.builders.persistence import INSTANCE
from tests.support.builders.persistence_control import state, synthetic
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from tests.support.fakes.os_clock import FakeOsClock


@pytest.fixture
def running(worker_files):
    database, configuration, boot = worker_files
    clock = FakeOsClock(monotonic_us=100)
    worker = CheckedPersistenceWorker(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=database,
        configuration_path=configuration,
        boot_id_path=boot,
        clock=clock,
    )
    worker.start()
    started = worker.wait_started(deadline_monotonic_us=5_000_100)
    assert started is not None and started.database_failure is None
    try:
        yield worker, database, started.state_load
    finally:
        worker.finish_test()


from tests.support.coordination.state_commit import LostStateReply as LostReply


# Recovery and ordinary snapshots share one authority; failure never changes the acknowledged value.
def test_recovery_then_ordinary_generation_authority(running):
    worker, _, loaded = running
    owner = CommunicatorStateOwner.from_load(control=worker.control, loaded=loaded)
    assert owner.condition is C.MISSING and owner.state is None
    assert (
        owner.commit(synthetic(), deadline_monotonic_us=0).disposition
        is CD.NOT_INSTALLED
    )
    assert owner.condition is C.MISSING and owner.pending is None
    assert (
        owner.commit(synthetic(), deadline_monotonic_us=5_000_100).disposition
        is CD.COMMITTED
    )
    initial = owner.state
    assert owner.condition is C.NONE and initial == synthetic()
    candidate = replace(initial, generation=2, airtime_snapshot_utc_us=1)
    assert (
        owner.commit(candidate, deadline_monotonic_us=0).disposition is CD.NOT_INSTALLED
    )
    assert owner.state is initial
    assert (
        owner.commit(candidate, deadline_monotonic_us=5_000_100).disposition
        is CD.COMMITTED
    )
    assert owner.state is candidate
    with pytest.raises(ValueError):
        owner.commit(replace(candidate, generation=4), deadline_monotonic_us=5_000_100)


# An unavailable load is never treated as an exact preceding-state match; only the same request may retry.
@pytest.mark.parametrize("installed", [False, True])
def test_unknown_recovery_exact_reconciliation_and_retry(running, installed):
    worker, database, loaded = running
    channel = LostReply(worker.control, installed=installed)
    owner = CommunicatorStateOwner.from_load(control=channel, loaded=loaded)
    requested = synthetic()
    assert (
        owner.commit(requested, deadline_monotonic_us=5_000_100).disposition
        is CD.OUTCOME_UNKNOWN
    )
    pending = owner.pending
    assert pending.preceding is None and pending.requested is requested
    assert owner.state is None
    with pytest.raises(RuntimeError):
        owner.commit(
            replace(requested, airtime_snapshot_utc_us=1),
            deadline_monotonic_us=5_000_100,
        )
    with pytest.raises(RuntimeError):
        owner.retry_pending_recovery(deadline_monotonic_us=5_000_100)
    channel.fail_load = True
    for _ in range(2):
        assert (
            owner.reconcile(deadline_monotonic_us=5_000_100).status
            is LS.DEADLINE_EXCEEDED
        )
        assert owner.pending is pending
    channel.fail_load = False
    result = owner.reconcile(deadline_monotonic_us=5_000_100)
    if not installed:
        assert result.state_condition is C.MISSING and owner.pending is pending
        assert (
            owner.retry_pending_recovery(deadline_monotonic_us=5_000_100).disposition
            is CD.COMMITTED
        )
        assert channel.requests == [requested, requested]
        assert channel.requests[0] is channel.requests[1]
    assert owner.state is requested and owner.pending is None
    with sqlite3.connect(database) as observer:
        assert observer.execute(
            "SELECT * FROM communicator_state"
        ).fetchone() == communicator_state_v1_parameters(requested)


# Unexpected generation-one bytes remain a conflict even when the generation matches the requested one.
def test_recovery_reconciliation_conflict_keeps_original_evidence(running):
    worker, _, loaded = running
    channel = LostReply(worker.control, installed=False)
    owner = CommunicatorStateOwner.from_load(control=channel, loaded=loaded)
    requested = synthetic()
    owner.commit(requested, deadline_monotonic_us=5_000_100)
    # A second caller is deliberately an invariant violation used to expose conflicting durable evidence.
    other = replace(
        requested,
        last_observed_rtc_health=state().last_observed_rtc_health,
        rtc_provenance=None,
        airtime_snapshot_utc_us=10,
        buckets=tuple(
            (
                replace(b, expires_at_utc_us=b.expires_at_utc_us + 10)
                if b.charged_airtime_us
                else b
            )
            for b in requested.buckets
        ),
    )
    assert (
        worker.control.commit_communicator_state(
            other, deadline_monotonic_us=5_000_100
        ).disposition
        is CD.COMMITTED
    )
    pending = owner.pending
    assert owner.reconcile(deadline_monotonic_us=5_000_100).state == other
    assert (
        owner.reconciliation_conflict
        and owner.state is None
        and owner.pending is pending
    )
    with pytest.raises(RuntimeError):
        owner.retry_pending_recovery(deadline_monotonic_us=5_000_100)


# Generation overflow and an unestablished baseline cannot submit a request to persistence.
def test_owner_requires_baseline_and_finite_next_generation(running):
    worker, _, _ = running
    owner = CommunicatorStateOwner(control=worker.control)
    with pytest.raises(RuntimeError):
        owner.commit(synthetic(), deadline_monotonic_us=5_000_100)
    owner = CommunicatorStateOwner(
        control=worker.control, initial_state=state(generation=(1 << 63) - 1)
    )
    with pytest.raises(ValueError):
        owner.commit(state(generation=1 << 63), deadline_monotonic_us=5_000_100)
    assert owner.pending is None


# One lost control reply plus exact reconciliation yields one original diagnostic root.
def test_observed_unknown_commit_is_one_episode(running):
    from cura_receiver.control_diagnostics import ControlEpisodeTracker
    from cura_receiver.generated import receiver_enums_generated as E
    worker, _, loaded = running
    tracker = ControlEpisodeTracker()
    events = []
    def observe(event):
        events.append(event)
        tracker(event)
    owner = CommunicatorStateOwner.from_load(control=LostReply(worker.control, installed=True),
        loaded=loaded, clock=worker._clock, observer=observe)
    owner.commit(synthetic(), deadline_monotonic_us=5_000_100,
                 purpose=E.PersistenceControlPurpose.AIRTIME_HISTORY_RECOVERY)
    assert tracker.take_ready() == ()
    assert owner.pending is not None
    owner.reconcile(deadline_monotonic_us=5_000_100)
    episodes = tracker.take_ready()
    assert len(episodes) == 1 and owner.pending is None
    assert episodes[0].context.disposition is E.PersistenceControlDisposition.OUTCOME_UNKNOWN
    assert episodes[0].context.requested_generation == 1
    assert episodes[0].context.authoritative_generation_before == 0
    assert episodes[0].context.purpose is E.PersistenceControlPurpose.AIRTIME_HISTORY_RECOVERY
    assert [e.command for e in events] == [E.PersistenceControlCommand.COMMIT_COMMUNICATOR_STATE,
                                         E.PersistenceControlCommand.LOAD_COMMUNICATOR_STATE]
    assert all(e.finished >= e.started for e in events)
    assert tracker.take_ready() == ()

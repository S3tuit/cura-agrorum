import sqlite3
from dataclasses import replace
from queue import Queue

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    PersistenceAdmissionState as State,
)
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, ProfileOnlyUnitV1
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as D,
    CommunicatorStateCommitFailureKind as F,
    CommunicatorStateCondition as Condition,
    CommunicatorStateLoadStatus as L,
)
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_database import DatabaseFailure
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import INSTANCE, _profile
from tests.support.builders.persistence_control import synthetic
from tests.support.fakes.os_clock import FakeOsClock


class ObservedWorker(CheckedPersistenceWorker):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.waits = Queue()

    def _wait_for_work(self, timeout):
        self.waits.put(timeout)
        assert self._wake.wait(5), "manual-clock recovery wait was not woken"


@pytest.fixture
def owner(worker_files):
    workers = []

    def create(transactions):
        path, config, boot = worker_files
        worker = ObservedWorker(
            instance=ReceiverInstanceStart(INSTANCE, 0),
            database_path=path,
            configuration_path=config,
            boot_id_path=boot,
            clock=FakeOsClock(monotonic_us=100),
            transactions=transactions,
        )
        workers.append(worker)
        worker.start()
        assert worker.wait_started(deadline_monotonic_us=5_000_100)
        assert worker.waits.get(timeout=5) is None
        return worker

    yield create
    for worker in workers:
        worker.finish_test()


def fault(code):
    error = sqlite3.OperationalError("concrete storage boundary fault")
    error.sqlite_errorcode = code
    return error


def due(worker):
    deadline = worker._recovery.retry_deadline_monotonic_us
    worker._clock.advance_elapsed_us(deadline - worker._clock.now_monotonic_us())
    worker._wake.set()
    return worker.waits.get(timeout=5)


# Empty-queue control errors establish real recovery work without replaying a failed mutation.
def test_control_only_recovery_preserves_deadlines_and_counts(owner, worker_files):
    class Faults(SqliteTransactions):
        begins = 0
        checkpoints = 0

        def begin(self, connection):
            self.begins += 1
            raise fault(sqlite3.SQLITE_BUSY)

        def checkpoint(self, connection):
            self.checkpoints += 1
            if self.checkpoints == 1:
                raise fault(sqlite3.SQLITE_IOERR_FSYNC)
            return super().checkpoint(connection)

    backend = Faults()
    worker = owner(backend)
    result = worker.control.commit_communicator_state(
        synthetic(), deadline_monotonic_us=5_000_100
    )
    assert (result.disposition, result.failure_kind) == (
        D.NOT_INSTALLED,
        F.DATABASE_ERROR,
    )
    assert worker.waits.get(timeout=5) == 0.250925
    recovery = worker._recovery
    assert recovery is worker._ordinary.recovery
    assert recovery.checkpoint_pending and worker._ordinary.pending_entities == 0
    assert recovery.counters.wal_checkpoint_attempts == 0
    assert worker.queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
    worker._clock.advance_elapsed_us(100_000)
    assert (
        worker.control.load_communicator_state(
            deadline_monotonic_us=5_000_100
        ).state_condition
        is Condition.MISSING
    )
    assert worker.waits.get(timeout=5) == 0.150925
    assert (
        worker.control.commit_communicator_state(
            synthetic(), deadline_monotonic_us=5_000_100
        ).disposition
        is D.NOT_INSTALLED
    )
    assert worker.waits.get(timeout=5) == 0.150925
    assert recovery.retry_deadline_monotonic_us == 251025
    assert due(worker) == 0.501850
    assert recovery.retry_deadline_monotonic_us == 752875
    assert due(worker) is None
    assert worker.queue.snapshot().admission_snapshot.state is State.AVAILABLE
    assert (backend.begins, backend.checkpoints) == (2, 2)
    assert (
        recovery.counters.wal_checkpoint_attempts,
        recovery.counters.wal_checkpoint_failures,
        recovery.counters.wal_checkpoint_successes,
    ) == (2, 1, 1)
    with sqlite3.connect(worker_files[0]) as db:
        assert db.execute("SELECT * FROM communicator_state").fetchall() == []
        assert db.execute("SELECT count(*) FROM receiver_instances").fetchone() == (1,)


# Exact serialized loads resolve unknown control commits without clearing independent storage recovery.
@pytest.mark.parametrize("committed", [False, True])
def test_control_unknown_is_caller_reconciled(owner, committed):
    class Unknown(SqliteTransactions):
        commits = 0

        def commit(self, connection):
            self.commits += 1
            if committed:
                super().commit(connection)
            raise fault(sqlite3.SQLITE_IOERR)

    backend = Unknown()
    worker = owner(backend)
    result = worker.control.commit_communicator_state(
        synthetic(), deadline_monotonic_us=5_000_100
    )
    assert result.disposition is D.OUTCOME_UNKNOWN
    worker.waits.get(timeout=5)
    loaded = worker.control.load_communicator_state(deadline_monotonic_us=5_000_100)
    assert loaded.state == (synthetic() if committed else None)
    assert loaded.state_condition is (
        Condition.NONE if committed else Condition.MISSING
    )
    worker.waits.get(timeout=5)
    assert worker.queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
    assert due(worker) is None
    assert backend.commits == 1


# A discarded connection reopens only the original bound file at the due recovery boundary.
@pytest.mark.parametrize("replacement", [False, True])
def test_control_recovery_same_file_binding(owner, worker_files, replacement):
    class Discard(SqliteTransactions):
        def begin(self, connection):
            raise fault(sqlite3.SQLITE_IOERR)

        def rollback(self, connection):
            raise fault(sqlite3.SQLITE_IOERR)

    worker = owner(Discard())
    worker.control.commit_communicator_state(
        synthetic(), deadline_monotonic_us=5_000_100
    )
    worker.waits.get(timeout=5)
    path = worker_files[0]
    if replacement:
        saved = path.with_suffix(".original")
        path.rename(saved)
        path.write_bytes(saved.read_bytes())
        before = path.read_bytes()
    assert due(worker) is None
    assert worker.queue.snapshot().admission_snapshot.state is (
        State.UNAVAILABLE_INCOMPATIBLE_SCHEMA if replacement else State.AVAILABLE
    )
    if replacement:
        assert path.read_bytes() == before
    else:
        result = worker.control.load_communicator_state(deadline_monotonic_us=5_000_100)
        assert result.state_condition is Condition.MISSING


# Corrupt/incompatible controls close admission and later controls cannot bypass the operator gate.
@pytest.mark.parametrize(
    "code,state",
    [
        (sqlite3.SQLITE_CORRUPT, State.UNAVAILABLE_CORRUPT),
        (sqlite3.SQLITE_SCHEMA, State.UNAVAILABLE_INCOMPATIBLE_SCHEMA),
        (sqlite3.SQLITE_FULL, State.UNAVAILABLE_DISK_FULL),
    ],
)
def test_control_failure_closed_classification(owner, code, state):
    class Fail(SqliteTransactions):
        def begin(self, connection):
            raise fault(code)

    worker = owner(Fail())
    worker.control.commit_communicator_state(
        synthetic(), deadline_monotonic_us=5_000_100
    )
    worker.waits.get(timeout=5)
    assert worker.queue.snapshot().admission_snapshot.state is state
    if state is State.UNAVAILABLE_DISK_FULL:
        assert due(worker) is None
        assert worker.queue.snapshot().admission_snapshot.state is State.AVAILABLE
    else:
        worker._clock.advance_elapsed_us(100_000_000)
        loaded = worker.control.load_communicator_state(
            deadline_monotonic_us=105_000_100
        )
        assert loaded.status is L.DATABASE_ERROR
        assert loaded.sqlite_extended_code == code
        assert worker.waits.get(timeout=5) is None
        assert worker._recovery.counters.wal_checkpoint_attempts == 0
        assert worker.queue.snapshot().admission_snapshot.state is state


# A control recovery obligation cannot delay immediate prefix removal during poison isolation.
def test_control_requirement_preserves_immediate_prefix(setup):
    _, db, queue, clock, create = setup
    ordinary = create()
    ordinary.enable_admission()
    for sequence in range(1, 4):
        profile = _profile(sequence=sequence)
        if sequence == 2:
            profile = replace(profile, busy_wait_count=-1)
        queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(profile)
        )
    ordinary.attempt(
        max_entities=3
    )  # Concrete SQL/validation failure starts isolation.
    ordinary.recovery.fail(DatabaseFailure(State.UNAVAILABLE_IO), control=True)
    clock.advance_elapsed_us(250925)
    first = ordinary.attempt(max_entities=3)
    assert first.acknowledged_entities == 1
    assert queue.snapshot().published_entities == 2
    assert queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
    assert db.execute(
        "SELECT occurrence_sequence FROM message_profiles"
    ).fetchall() == [(1,)]
    for _ in range(4):
        if not queue.snapshot().published_entities:
            break
        ordinary.attempt(max_entities=3)
    assert queue.snapshot().published_entities == 0
    assert ordinary.checkpoint_pending
    assert queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
    assert ordinary.checkpoint().failure is None
    assert queue.snapshot().admission_snapshot.state is State.AVAILABLE
    assert db.execute(
        "SELECT occurrence_sequence FROM message_profiles ORDER BY occurrence_sequence"
    ).fetchall() == [(1,), (3,)]
    assert db.execute("SELECT count(*) FROM quarantined_entities").fetchone() == (1,)


# The agreed control recovery criterion accepts real partial and no-fresh-work PASSIVE results.
@pytest.mark.parametrize("reader_limited", [False, True])
def test_control_recovery_checkpoint_progress(setup, reader_limited):
    path, _, queue, clock, create = setup
    ordinary = create()
    ordinary.enable_admission()
    reader = sqlite3.connect(path, isolation_level=None)
    try:
        queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(_profile())
        )
        ordinary.attempt(max_entities=1)
        ordinary.checkpoint()
        if reader_limited:
            reader.execute("BEGIN")
            reader.execute("SELECT * FROM message_profiles").fetchall()
            queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
                ProfileOnlyUnitV1(_profile(sequence=2))
            )
            ordinary.attempt(max_entities=1)
        ordinary.recovery.fail(DatabaseFailure(State.UNAVAILABLE_IO), control=True)
        assert ordinary.counters.wal_checkpoint_failures == 0
        clock.advance_elapsed_us(250925)
        result = ordinary.checkpoint()
        assert result.failure is None
        if reader_limited:
            assert result.checkpointed_frames < result.wal_frames
        else:
            assert result.checkpointed_frames == result.wal_frames
        assert ordinary.counters.wal_checkpoint_attempts == 2
        assert ordinary.counters.wal_checkpoint_failures == 0
        assert queue.snapshot().admission_snapshot.state is State.AVAILABLE
    finally:
        reader.close()


# Connection cleanup failure closes admission without changing a confirmed durable control result.
def test_control_post_commit_connection_failure(owner):
    class ClosedAfterCommit(SqliteTransactions):
        def commit(self, connection):
            super().commit(connection)
            connection.close()

    worker = owner(ClosedAfterCommit())
    result = worker.control.commit_communicator_state(
        synthetic(), deadline_monotonic_us=5_000_100
    )
    assert result.disposition is D.COMMITTED
    worker.waits.get(timeout=5)
    assert worker.queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
    assert due(worker) is None
    assert (
        worker.control.load_communicator_state(deadline_monotonic_us=5_000_100).state
        == synthetic()
    )


# A control failure added to unknown ordinary work preserves its exact frozen health snapshot.
def test_shared_recovery_keeps_frozen_unknown_health(setup):
    from cura_receiver.persist_queue_entities import RECEIVER_HEALTH_REQUEST_V1_SPEC
    from cura_receiver.ports.host_observations import HostObservations
    from tests.support.builders.persistence import _health_request

    _, db, queue, clock, create = setup

    class Host:
        calls = 0

        def sample(self):
            self.calls += 1
            return HostObservations(memory_available_bytes=self.calls)

    class UnknownOnce(SqliteTransactions):
        failed = False

        def commit(self, connection):
            super().commit(connection)
            if not self.failed:
                self.failed = True
                raise fault(sqlite3.SQLITE_IOERR)

    host = Host()
    ordinary = create(UnknownOnce(), host_observations=host)
    ordinary.enable_admission()
    queue.try_reserve_one(RECEIVER_HEALTH_REQUEST_V1_SPEC).reservation.publish(
        _health_request()
    )
    ordinary.attempt(max_entities=1)
    before = db.execute("SELECT * FROM receiver_health").fetchall()
    assert len(before) == 1 and queue.snapshot().claimed_entities == 1
    deadline = ordinary.retry_deadline_monotonic_us
    clock.advance_elapsed_us(100_000)
    ordinary.recovery.fail(DatabaseFailure(State.UNAVAILABLE_IO), control=True)
    assert ordinary.retry_deadline_monotonic_us == deadline
    clock.advance_elapsed_us(150925)
    assert ordinary.attempt(max_entities=1).acknowledged_entities == 1
    assert host.calls == 1
    assert db.execute("SELECT * FROM receiver_health").fetchall() == before
    assert queue.snapshot().published_entities == 0
    assert queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
    assert ordinary.checkpoint().failure is None
    assert queue.snapshot().admission_snapshot.state is State.AVAILABLE

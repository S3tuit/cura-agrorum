from __future__ import annotations

import sqlite3
from dataclasses import replace

import pytest
from cura_receiver.generated.receiver_enums_generated import (
    AdmissionResult,
    PersistenceAdmissionState as State,
)
from cura_receiver.persist_queue_entities import (
    CLOCK_OBSERVATION_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    ProfileOnlyUnitV1,
)
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import _observation, _profile


def _publish(queue, entity, spec=CLOCK_OBSERVATION_V1_SPEC):
    reservation = queue.try_reserve_one(spec)
    assert reservation.status is AdmissionResult.RESERVED
    reservation.reservation.publish(entity)


# WAL copying is explicit, synchronous and leaves all retained application rows intact.
def test_explicit_checkpoint_and_duration(setup):
    path, connection, queue, clock, create = setup

    class TimedCheckpoint(SqliteTransactions):
        def checkpoint(self, connection):
            result = super().checkpoint(connection)
            clock.advance_elapsed_us(71)
            return result

    persistence = create(TimedCheckpoint())
    assert persistence.checkpoint() is None
    persistence.enable_admission()
    for sequence in range(1, 41):
        _publish(queue, _observation(sequence=sequence))
    assert persistence.attempt(max_entities=40).acknowledged_entities == 40
    assert persistence.counters.wal_checkpoint_attempts == 0
    before = connection.execute("SELECT * FROM clock_observations").fetchall()
    result = persistence.checkpoint()
    assert result.failure is None and result.duration_us == 71
    assert result.wal_frames == result.checkpointed_frames > 0
    assert connection.execute("SELECT * FROM clock_observations").fetchall() == before
    assert persistence.counters.wal_checkpoint_attempts == 1
    assert persistence.counters.wal_checkpoint_successes == 1
    assert persistence.counters.wal_checkpoint_failures == 0


# A real reader keeps older WAL frames necessary without making PASSIVE an I/O error.
def test_passive_reader_partial_progress(setup):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(queue, _observation())
    persistence.attempt(max_entities=1)
    reader = sqlite3.connect(path, isolation_level=None)
    try:
        reader.execute("BEGIN")
        assert reader.execute("SELECT count(*) FROM clock_observations").fetchone() == (
            1,
        )
        _publish(queue, _observation(sequence=2))
        persistence.attempt(max_entities=1)
        result = persistence.checkpoint()
        assert result.failure is None
        assert 0 <= result.checkpointed_frames < result.wal_frames
        assert not persistence.checkpoint_pending
        assert queue.snapshot().admission_snapshot.state is State.AVAILABLE
        assert reader.execute("SELECT count(*) FROM clock_observations").fetchone() == (
            1,
        )
        assert path.with_name(path.name + "-wal").stat().st_size > 0
    finally:
        reader.close()
    complete = persistence.checkpoint()
    assert complete.failure is None
    assert complete.checkpointed_frames == complete.wal_frames
    assert connection.execute("SELECT count(*) FROM clock_observations").fetchone() == (
        2,
    )


# Checkpoint faults unavailable through portable real SQLite keep their distinct closed states.
@pytest.mark.parametrize(
    "code,state",
    [
        (sqlite3.SQLITE_IOERR_FSYNC, State.UNAVAILABLE_IO),
        (sqlite3.SQLITE_FULL, State.UNAVAILABLE_DISK_FULL),
        (sqlite3.SQLITE_CORRUPT, State.UNAVAILABLE_CORRUPT),
        (sqlite3.SQLITE_SCHEMA, State.UNAVAILABLE_INCOMPATIBLE_SCHEMA),
    ],
)
@pytest.mark.parametrize("after_real_checkpoint", [False, True])
def test_empty_queue_checkpoint_recovery(setup, code, state, after_real_checkpoint):
    path, connection, queue, clock, create = setup

    class FaultCheckpoint(SqliteTransactions):
        fail = True

        def checkpoint(self, connection):
            if not self.fail:
                return super().checkpoint(connection)
            if after_real_checkpoint:
                super().checkpoint(connection)
            error = sqlite3.OperationalError("checkpoint boundary fault")
            error.sqlite_errorcode = code
            raise error

    fault = FaultCheckpoint()
    persistence = create(fault)
    persistence.enable_admission()
    result = persistence.checkpoint()
    assert result.failure.admission_state is state
    assert result.failure.sqlite_extended_code == code
    assert persistence.checkpoint_pending
    assert queue.snapshot().published_entities == 0
    assert (
        queue.try_reserve_one(CLOCK_OBSERVATION_V1_SPEC).status
        is AdmissionResult.PERSISTENCE_UNAVAILABLE
    )
    fault.fail = False
    assert persistence.checkpoint() is None
    assert persistence.attempt(max_entities=1) is None
    if state in (State.UNAVAILABLE_CORRUPT, State.UNAVAILABLE_INCOMPATIBLE_SCHEMA):
        clock.advance_elapsed_us(100_000_000)
        assert persistence.checkpoint() is None
        persistence.request_operator_recovery()
        # An idle ordinary attempt must not consume maintenance authorization.
        assert persistence.attempt(max_entities=1) is None
    else:
        clock.advance_elapsed_us(250924)
        assert persistence.checkpoint() is None
        clock.advance_elapsed_us(1)
    result = persistence.checkpoint()
    assert result.failure is None
    assert not persistence.checkpoint_pending
    assert persistence.retry_deadline_monotonic_us is None
    assert queue.snapshot().admission_snapshot.state is State.AVAILABLE
    assert persistence.counters.batch_transaction_attempts == 0
    assert persistence.counters.wal_checkpoint_attempts == 2
    assert persistence.counters.wal_checkpoint_failures == 1
    with sqlite3.connect(path) as observer:
        assert observer.execute(
            "SELECT count(*) FROM clock_observations"
        ).fetchone() == (0,)
        assert observer.execute("PRAGMA integrity_check").fetchone() == ("ok",)


# Empty-queue checkpoint retries obey the same exact backoff and have no attempt limit.
def test_checkpoint_capped_backoff(setup):
    path, connection, queue, clock, create = setup

    class BusyCheckpoint(SqliteTransactions):
        def checkpoint(self, connection):
            return (1, -1, -1)

    persistence = create(BusyCheckpoint())
    persistence.enable_admission()
    waits = [250925, 501850, 1003700, 2007400, 4014800] + [5018500] * 20
    for wait in waits:
        result = persistence.checkpoint()
        assert result.failure.sqlite_primary_code == sqlite3.SQLITE_BUSY
        assert (
            persistence.retry_deadline_monotonic_us == clock.now_monotonic_us() + wait
        )
        clock.advance_elapsed_us(wait - 1)
        assert persistence.checkpoint() is None
        clock.advance_elapsed_us(1)
    assert persistence.counters.wal_checkpoint_failures == 25
    assert persistence.checkpoint_pending


# Completing ordinary or quarantined work cannot clear an outstanding checkpoint failure.
@pytest.mark.parametrize("poison", [False, True])
def test_pending_checkpoint_survives_batch_success(setup, poison):
    path, connection, queue, clock, create = setup

    class OnceCheckpoint(SqliteTransactions):
        fail = True

        def checkpoint(self, connection):
            if self.fail:
                self.fail = False
                raise OSError(5, "I/O fault")
            return super().checkpoint(connection)

    persistence = create(OnceCheckpoint())
    persistence.enable_admission()
    if poison:
        _publish(
            queue,
            ProfileOnlyUnitV1(replace(_profile(), busy_wait_count=-1)),
            PROFILE_ONLY_V1_SPEC,
        )
    else:
        _publish(queue, _observation())
    assert persistence.checkpoint().failure is not None
    clock.advance_elapsed_us(250925)
    result = persistence.attempt(max_entities=1)
    if poison:
        assert persistence.checkpoint() is None
        persistence.attempt(max_entities=1)
        assert persistence.checkpoint() is None
        result = persistence.attempt(max_entities=1)
    assert result.acknowledged_entities == 1
    assert persistence.checkpoint_pending
    assert queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
    assert persistence.checkpoint().failure is None
    assert queue.snapshot().admission_snapshot.state is State.AVAILABLE


# Recovery must still validate identity after other work resolved the original batch failure.
def test_checkpoint_revalidates_after_ordinary_success(setup):
    path, connection, queue, clock, create = setup

    class OnceCheckpoint(SqliteTransactions):
        fail = True

        def checkpoint(self, connection):
            if self.fail:
                self.fail = False
                raise OSError(5, "I/O fault")
            return super().checkpoint(connection)

    persistence = create(OnceCheckpoint())
    persistence.enable_admission()
    _publish(queue, _observation())
    persistence.checkpoint()
    clock.advance_elapsed_us(250925)
    persistence.attempt(max_entities=1)
    connection.execute("PRAGMA application_id=7")
    result = persistence.checkpoint()
    assert result.failure.admission_state is State.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    assert persistence.checkpoint_pending


# A real preventive free-space check also retains checkpoint work until storage is usable.
def test_checkpoint_low_space(setup):
    path, connection, queue, clock, create = setup
    persistence = create(minimum_free_bytes=1 << 63)
    persistence.enable_admission()
    result = persistence.checkpoint()
    assert result.failure.admission_state is State.UNAVAILABLE_LOW_SPACE
    assert persistence.checkpoint_pending
    persistence._minimum_free_bytes = 0
    clock.advance_elapsed_us(250925)
    assert persistence.checkpoint().failure is None
    assert queue.snapshot().admission_snapshot.state is State.AVAILABLE


# Ordinary commits, exact retries, quarantine and checkpointing preserve all prior retained rows.
def test_all_ordinary_history_retained(setup):
    from cura_receiver.persist_queue_entities import (
        DIAGNOSTIC_V1_SPEC,
        MEASUREMENT_PROFILE_V1_SPEC,
        RECEIVER_HEALTH_REQUEST_V1_SPEC,
    )
    from tests.support.builders.persistence import (
        _diagnostic,
        _health_request,
        _measurement,
    )

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    units = (
        (_observation(), CLOCK_OBSERVATION_V1_SPEC),
        (_measurement(), MEASUREMENT_PROFILE_V1_SPEC),
        (ProfileOnlyUnitV1(_profile(sequence=2)), PROFILE_ONLY_V1_SPEC),
        (_diagnostic(), DIAGNOSTIC_V1_SPEC),
        (_health_request(), RECEIVER_HEALTH_REQUEST_V1_SPEC),
        (
            ProfileOnlyUnitV1(replace(_profile(sequence=3), busy_wait_count=-1)),
            PROFILE_ONLY_V1_SPEC,
        ),
    )
    for entity, spec in units:
        _publish(queue, entity, spec)
    for _ in range(10):
        if queue.snapshot().published_entities == 0:
            break
        persistence.attempt(max_entities=6)
    assert queue.snapshot().published_entities == 0
    tables = (
        "receiver_instances",
        "clock_observations",
        "reading_messages",
        "message_profiles",
        "diagnostics",
        "receiver_health",
        "quarantined_entities",
    )
    before = {
        table: connection.execute(f"SELECT * FROM {table}").fetchall()
        for table in tables
    }
    assert all(before.values())
    # Replay the exact immutable non-health successes. Health replay is separately
    # covered with frozen enrichment because a new request samples a new health row.
    for entity, spec in units[:4]:
        _publish(queue, entity, spec)
    assert persistence.attempt(max_entities=4).acknowledged_entities == 4
    assert persistence.checkpoint().failure is None
    assert {
        table: connection.execute(f"SELECT * FROM {table}").fetchall()
        for table in tables
    } == before
    assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
    assert connection.execute("PRAGMA foreign_key_check").fetchall() == []

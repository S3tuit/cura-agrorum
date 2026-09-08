import sqlite3
from dataclasses import replace

import pytest
from cura_receiver.generated import receiver_entities_generated as row
from cura_receiver.generated import receiver_enums_generated as enum
from cura_receiver.ordinary_persistence import OrdinaryBatchCommitOutcome as Outcome
from cura_receiver.persist_queue_entities import (
    CLOCK_OBSERVATION_V1_SPEC,
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    ProfileOnlyUnitV1,
)
from cura_receiver.quarantine_evidence import (
    encode_quarantine_evidence_v1,
    quarantine_evidence_sha256,
)
from cura_receiver.sqlite_transactions import SqliteTransactions

from tests.support.builders.persistence import (
    INSTANCE,
    _health_request,
    _measurement,
    _observation,
    _profile,
)


def _publish(queue, entity, spec):
    result = queue.try_reserve_one(spec)
    assert result.status is enum.AdmissionResult.RESERVED
    result.reservation.publish(entity)


def _drain(persistence, queue, clock, bound=20):
    outcomes = []
    for _ in range(bound):
        if queue.snapshot().published_entities == 0:
            return outcomes
        deadline = persistence.retry_deadline_monotonic_us
        if deadline is not None and deadline > clock.now_monotonic_us():
            clock.advance_elapsed_us(deadline - clock.now_monotonic_us())
        result = persistence.attempt(max_entities=500)
        outcomes.append(result)
        if result is None:
            return outcomes
    pytest.fail("bounded isolation did not finish")


# Each valid unit or exact quarantine row durably completes its own FIFO slot.
def test_mixed_batch_quarantine_and_remaining_lease(setup):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    poison = ProfileOnlyUnitV1(replace(_profile(sequence=2), busy_wait_count=-1))
    _publish(queue, _measurement(), MEASUREMENT_PROFILE_V1_SPEC)
    _publish(queue, poison, PROFILE_ONLY_V1_SPEC)
    _publish(
        queue,
        _measurement(sequence=3, message=101, domain=2),
        MEASUREMENT_PROFILE_V1_SPEC,
    )
    assert persistence.attempt(max_entities=3).outcome is Outcome.NOT_COMMITTED
    assert connection.execute("SELECT count(*) FROM message_profiles").fetchone() == (
        0,
    )
    assert queue.snapshot().claimed_entities == 3
    # The valid prefix leaves the queue while the remaining lease stays owned.
    assert persistence.attempt(max_entities=3).acknowledged_entities == 1
    assert connection.execute("SELECT count(*) FROM message_profiles").fetchone() == (
        1,
    )
    assert queue.snapshot().published_entities == 2
    assert queue.snapshot().claimed_entities == 2
    assert persistence.attempt(max_entities=3).outcome is Outcome.NOT_COMMITTED
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (0,)
    assert persistence.attempt(max_entities=3).acknowledged_entities == 1
    assert queue.snapshot().published_entities == 1
    assert queue.snapshot().claimed_entities == 1
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (1,)
    assert persistence.attempt(max_entities=3).acknowledged_entities == 1
    evidence = encode_quarantine_evidence_v1(poison, spec=PROFILE_ONLY_V1_SPEC)
    assert connection.execute("SELECT * FROM quarantined_entities").fetchone() == (
        quarantine_evidence_sha256(evidence),
        2,
        1,
        len(evidence),
        evidence,
        INSTANCE,
        100,
        enum.DATABASE_SCHEMA_VERSION,
        5,
        5,
        19,
        sqlite3.SQLITE_CONSTRAINT_CHECK,
        None,
        1,
    )
    assert connection.execute(
        "SELECT occurrence_sequence,persistence_classification_id FROM message_profiles ORDER BY occurrence_sequence"
    ).fetchall() == [(1, 1), (3, 3)]
    assert connection.execute("SELECT count(*) FROM diagnostics").fetchone() == (0,)
    assert persistence.counters.batch_entities_committed == 2
    assert persistence.counters.durable_quarantine_successes == 1


# Every reachable entity-defect reason preserves the entire logical queue unit as quarantine evidence.
@pytest.mark.parametrize(
    "kind,reason",
    [
        ("decoding", 2),
        ("binding", 3),
        ("range", 4),
        ("constraint", 5),
        ("derivation", 6),
    ],
)
def test_reachable_poison_reasons(setup, kind, reason):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    if kind == "decoding":
        valid = _measurement()
        entity = replace(valid, candidate=replace(valid.candidate, reading_body=b"bad"))
        spec = MEASUREMENT_PROFILE_V1_SPEC
    elif kind == "binding":
        entity = ProfileOnlyUnitV1(replace(_profile(), busy_wait_count="wrong type"))
        spec = PROFILE_ONLY_V1_SPEC
    elif kind == "range":
        entity = ProfileOnlyUnitV1(replace(_profile(), busy_wait_total_us=1 << 63))
        spec = PROFILE_ONLY_V1_SPEC
    elif kind == "constraint":
        entity = ProfileOnlyUnitV1(replace(_profile(), busy_wait_count=-1))
        spec = PROFILE_ONLY_V1_SPEC
    else:
        entity = replace(_health_request(), radio_recovery_attempts_by_reason=(0,))
        spec = RECEIVER_HEALTH_REQUEST_V1_SPEC
    _publish(queue, entity, spec)
    _drain(persistence, queue, clock)
    assert queue.snapshot().published_entities == 0
    actual = connection.execute(
        "SELECT failure_reason_id,entity_bytes FROM quarantined_entities"
    ).fetchone()
    assert actual == (reason, encode_quarantine_evidence_v1(entity, spec=spec))
    assert persistence.counters.batch_entities_committed == 0


# An isolated clock defect retains its complete batch and prevents all following FIFO work.
def test_nonquarantinable_clock_boundary(setup):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(
        queue,
        replace(_observation(), sampled_at_monotonic_us=1 << 63),
        CLOCK_OBSERVATION_V1_SPEC,
    )
    _publish(queue, ProfileOnlyUnitV1(_profile()), PROFILE_ONLY_V1_SPEC)
    _drain(persistence, queue, clock)
    assert queue.snapshot().claimed_entities == 2
    assert (
        queue.snapshot().admission_snapshot.state
        is enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    )
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (0,)
    assert connection.execute("SELECT count(*) FROM message_profiles").fetchone() == (
        0,
    )


# Unsupported evidence types cannot be substituted, dropped or acknowledged after isolated reproduction.
def test_evidence_encoding_failure_retains_batch(setup):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(
        queue,
        ProfileOnlyUnitV1(replace(_profile(), busy_wait_count=1.5)),
        PROFILE_ONLY_V1_SPEC,
    )
    _drain(persistence, queue, clock)
    assert queue.snapshot().claimed_entities == 1
    assert (
        queue.snapshot().admission_snapshot.state
        is enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    )
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (0,)


# An entity-specific failure that does not reproduce is processed normally without quarantine.
def test_nonreproducing_constraint_is_not_poison(setup):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(queue, ProfileOnlyUnitV1(_profile()), PROFILE_ONLY_V1_SPEC)
    connection.execute(
        "CREATE TEMP TRIGGER one_failure BEFORE INSERT ON message_profiles BEGIN SELECT RAISE(ABORT,'test'); END"
    )
    assert persistence.attempt(max_entities=1).outcome is Outcome.NOT_COMMITTED
    connection.execute("DROP TRIGGER one_failure")
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (0,)


# Quarantine failure and lost replies retain the exact frozen evidence and provenance until reconciliation.
@pytest.mark.parametrize("mode", ["before", "after", "absent_unknown"])
def test_quarantine_commit_recovery(setup, mode):
    path, connection, queue, clock, create = setup

    class QuarantineFault(SqliteTransactions):
        failed = False

        def begin(self, connection):
            if (
                mode == "before"
                and not self.failed
                and connection.execute(
                    "SELECT count(*) FROM quarantined_entities"
                ).fetchone()
                == (0,)
                and self.ready
            ):
                self.failed = True
                raise OSError("quarantine begin failed")
            super().begin(connection)

        def commit(self, connection):
            if not self.failed and self.ready:
                self.failed = True
                if mode == "after":
                    super().commit(connection)
                raise OSError("quarantine commit reply unavailable")
            super().commit(connection)

        ready = False

    transactions = QuarantineFault()
    persistence = create(transactions)
    persistence.enable_admission()
    poison = ProfileOnlyUnitV1(replace(_profile(), busy_wait_count=-1))
    _publish(queue, poison, PROFILE_ONLY_V1_SPEC)
    persistence.attempt(max_entities=1)
    persistence.attempt(max_entities=1)
    transactions.ready = True
    first = persistence.attempt(max_entities=1)
    assert first.outcome is (
        Outcome.NOT_COMMITTED if mode == "before" else Outcome.OUTCOME_UNKNOWN
    )
    assert queue.snapshot().claimed_entities == 1
    assert (
        queue.snapshot().admission_snapshot.state
        is enum.PersistenceAdmissionState.UNAVAILABLE_IO
    )
    assert persistence.attempt(max_entities=1) is None
    clock.advance_elapsed_us(1_000_000)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert connection.execute(
        "SELECT quarantined_at_monotonic_us,isolation_attempt_count FROM quarantined_entities"
    ).fetchone() == (100, 1)
    assert persistence.counters.durable_quarantine_successes == 1
    assert persistence.counters.durable_quarantine_failures == 1


# A poisoned first reading cannot give a later valid occurrence a canonical relation that never committed.
def test_poisoned_reading_precedes_valid_same_sample(setup):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    poison = _measurement()
    poison = replace(poison, profile=replace(poison.profile, busy_wait_count=-1))
    _publish(queue, poison, MEASUREMENT_PROFILE_V1_SPEC)
    _publish(queue, _measurement(sequence=2, message=101), MEASUREMENT_PROFILE_V1_SPEC)
    _drain(persistence, queue, clock)
    assert queue.snapshot().published_entities == 0
    assert connection.execute(
        "SELECT message_id,is_canonical_for_sample,first_occurrence_sequence FROM reading_messages"
    ).fetchall() == [(101, 1, 2)]
    assert connection.execute(
        "SELECT entity_bytes FROM quarantined_entities"
    ).fetchone() == (
        encode_quarantine_evidence_v1(poison, spec=MEASUREMENT_PROFILE_V1_SPEC),
    )


# Quarantine code/provenance collisions never overwrite the row or acknowledge its queued evidence.
@pytest.mark.parametrize("column", row.QUARANTINED_ENTITY_ROW_V1_COLUMNS[1:])
def test_quarantine_exact_row_collision(setup, column):
    path, connection, queue, clock, create = setup

    class LoseQuarantineReply(SqliteTransactions):
        lost = False

        def commit(self, connection):
            super().commit(connection)
            if not self.lost:
                self.lost = True
                raise OSError("lost quarantine reply")

    persistence = create(LoseQuarantineReply())
    persistence.enable_admission()
    _publish(
        queue,
        ProfileOnlyUnitV1(replace(_profile(), busy_wait_count=-1)),
        PROFILE_ONLY_V1_SPEC,
    )
    persistence.attempt(max_entities=1)
    persistence.attempt(max_entities=1)
    assert persistence.attempt(max_entities=1).outcome is Outcome.OUTCOME_UNKNOWN
    if column == "receiver_instance_id":
        from cura_receiver.receiver_startup import (
            ReceiverInstanceStart,
            insert_receiver_instance_start,
        )

        changed = bytes.fromhex("11112233445546778899aabbccddeeff")
        insert_receiver_instance_start(
            connection, ReceiverInstanceStart(changed, 1), b"b" * 16
        )
    else:
        original = connection.execute(
            f"SELECT {column} FROM quarantined_entities"
        ).fetchone()[0]
        if type(original) is bytes:
            changed = bytes([original[0] ^ 1]) + original[1:]
        elif original is None:
            changed = 0
        else:
            changed = original + 1
    # The length column has an exact SQL relation: change both values, still
    # requiring the frozen comparison to catch each tested provenance defect.
    trigger = connection.execute(
        "SELECT sql FROM sqlite_master WHERE name='quarantined_entities_no_update'"
    ).fetchone()[0]
    connection.execute("DROP TRIGGER quarantined_entities_no_update")
    if column == "entity_length":
        connection.execute(
            "UPDATE quarantined_entities SET entity_length=entity_length+1,entity_bytes=CAST(entity_bytes || ? AS BLOB)",
            (b" ",),
        )
    else:
        connection.execute(f"UPDATE quarantined_entities SET {column}=?", (changed,))
    connection.execute(trigger)
    before = connection.execute("SELECT * FROM quarantined_entities").fetchall()
    clock.advance_elapsed_us(1_000_000)
    result = persistence.attempt(max_entities=1)
    assert (
        result.failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    )
    assert queue.snapshot().claimed_entities == 1
    assert connection.execute("SELECT * FROM quarantined_entities").fetchall() == before


# Capacity and corruption during quarantine retain their distinct classifier states and active lease.
@pytest.mark.parametrize(
    "code,expected",
    [
        (sqlite3.SQLITE_FULL, enum.PersistenceAdmissionState.UNAVAILABLE_DISK_FULL),
        (
            sqlite3.SQLITE_IOERR_CORRUPTFS,
            enum.PersistenceAdmissionState.UNAVAILABLE_CORRUPT,
        ),
    ],
)
def test_quarantine_storage_failure_precedence(setup, code, expected):
    path, connection, queue, clock, create = setup

    class Failure(SqliteTransactions):
        def commit(self, connection):
            error = sqlite3.OperationalError("injected backend outcome")
            error.sqlite_errorcode = code
            raise error

    persistence = create(Failure())
    persistence.enable_admission()
    _publish(
        queue,
        ProfileOnlyUnitV1(replace(_profile(), busy_wait_count=-1)),
        PROFILE_ONLY_V1_SPEC,
    )
    persistence.attempt(max_entities=1)
    persistence.attempt(max_entities=1)
    result = persistence.attempt(max_entities=1)
    assert result.outcome is Outcome.OUTCOME_UNKNOWN
    assert result.failure.admission_state is expected
    assert queue.snapshot().claimed_entities == 1
    assert queue.snapshot().published_entities == 1


# A global failure during isolated reproduction cannot quarantine a unit that has not reproduced alone.
def test_global_failure_interrupts_isolation(setup):
    path, connection, queue, clock, create = setup

    class FailBegin(SqliteTransactions):
        calls = 0

        def begin(self, connection):
            self.calls += 1
            if self.calls == 2:
                raise OSError("transient isolation access failure")
            super().begin(connection)

    persistence = create(FailBegin())
    persistence.enable_admission()
    _publish(
        queue,
        ProfileOnlyUnitV1(replace(_profile(), busy_wait_count=-1)),
        PROFILE_ONLY_V1_SPEC,
    )
    persistence.attempt(max_entities=1)
    result = persistence.attempt(max_entities=1)
    assert (
        result.failure.admission_state is enum.PersistenceAdmissionState.UNAVAILABLE_IO
    )
    assert queue.snapshot().claimed_entities == 1
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (0,)
    _drain(persistence, queue, clock)
    assert queue.snapshot().published_entities == 0
    assert connection.execute(
        "SELECT isolation_attempt_count FROM quarantined_entities"
    ).fetchone() == (2,)


# Failure of the optional sampler cannot make a valid communicator request into entity poison.
def test_unavailable_host_source_is_absent(setup):
    path, connection, queue, clock, create = setup

    class MissingHost:
        def sample(self):
            raise OSError("host sampling unavailable")

    persistence = create(host_observations=MissingHost())
    persistence.enable_admission()
    _publish(queue, _health_request(), RECEIVER_HEALTH_REQUEST_V1_SPEC)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert connection.execute(
        "SELECT linux_load_1m_milli,cpu_temperature_milli_c,memory_available_bytes FROM receiver_health"
    ).fetchone() == (None, None, None)
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (0,)


# Ordinary attempts count isolation, while quarantine commits and durations keep their separate counters.
def test_counter_ownership_through_isolation(setup):
    path, connection, queue, clock, create = setup

    class TimedCommit(SqliteTransactions):
        def commit(self, connection):
            super().commit(connection)
            clock.advance_elapsed_us(7)

    persistence = create(TimedCommit())
    persistence.enable_admission()
    _publish(queue, ProfileOnlyUnitV1(_profile()), PROFILE_ONLY_V1_SPEC)
    _publish(
        queue,
        ProfileOnlyUnitV1(replace(_profile(sequence=2), busy_wait_count=-1)),
        PROFILE_ONLY_V1_SPEC,
    )
    _publish(queue, ProfileOnlyUnitV1(_profile(sequence=3)), PROFILE_ONLY_V1_SPEC)
    _drain(persistence, queue, clock)
    counts = persistence.counters
    assert (
        counts.batch_transaction_attempts,
        counts.batch_transaction_commits,
        counts.batch_transaction_failures,
    ) == (4, 2, 2)
    assert counts.batch_entities_committed == 2
    assert (
        counts.batch_commit_duration_total_us,
        counts.batch_commit_duration_max_us,
    ) == (14, 7)
    assert (
        counts.durable_quarantine_successes,
        counts.durable_quarantine_failures,
    ) == (1, 0)
    assert (
        counts.wal_checkpoint_attempts,
        counts.wal_checkpoint_successes,
        counts.wal_checkpoint_failures,
    ) == (0, 0, 0)

from __future__ import annotations

import sqlite3
from dataclasses import replace
from pathlib import Path

import pytest
from cura_receiver.generated import receiver_entities_generated as row
from cura_receiver.generated import receiver_enums_generated as enum
from cura_receiver.ordinary_persistence import OrdinaryBatchCommitOutcome as Outcome
from cura_receiver.ordinary_persistence import OrdinaryPersistence
from cura_receiver.persist_queue_entities import CLOCK_OBSERVATION_V1_SPEC
from cura_receiver.receiver_startup import (
    ReceiverInstanceStart,
    insert_receiver_instance_start,
)
from cura_receiver.sqlite_transactions import SqliteTransactions

from tests.support.builders.persistence import (
    GROUP,
    INSTANCE,
    _diagnostic,
    _health_request,
    _measurement,
    _observation,
    _profile,
)


def _publish(queue, entity, spec=CLOCK_OBSERVATION_V1_SPEC):
    result = queue.try_reserve_one(spec)
    assert result.status is enum.AdmissionResult.RESERVED
    result.reservation.publish(entity)


# Startup handoff and commit keep the original queue objects until durable removal.
def test_commit_before_acknowledgement(setup):
    path, connection, queue, clock, create = setup

    class InspectCommit(SqliteTransactions):
        def commit(self, connection):
            assert queue.snapshot().published_entities == 2
            assert queue.snapshot().claimed_entities == 2
            super().commit(connection)
            with sqlite3.connect(path) as observer:
                assert observer.execute(
                    "SELECT count(*) FROM clock_observations"
                ).fetchone() == (2,)
            assert queue.snapshot().published_entities == 2

    persistence = create(InspectCommit())
    assert (
        queue.snapshot().admission_snapshot.state
        is enum.PersistenceAdmissionState.UNAVAILABLE_STARTING
    )
    persistence.enable_admission()
    first = _observation()
    second = _observation(sequence=2)
    _publish(queue, first)
    _publish(queue, second)
    result = persistence.attempt(max_entities=2)
    assert result.outcome is Outcome.COMMITTED
    assert result.acknowledged_entities == 2
    assert queue.snapshot().published_entities == 0
    assert first == _observation() and second == _observation(sequence=2)
    assert connection.execute(
        "SELECT * FROM clock_observations ORDER BY observation_sequence"
    ).fetchall() == [
        (INSTANCE, 1, 0, 10, None, 0, 0, 1),
        (INSTANCE, 2, 0, 10, None, 0, 0, 1),
    ]


# A real entity constraint rolls the batch back and retains its exact FIFO lease for isolation.
def test_precommit_failure_retains_fifo(setup, monkeypatch):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    first, second = _observation(), _observation(sequence=2)
    _publish(queue, first)
    _publish(queue, second)
    connection.execute(
        "CREATE TEMP TRIGGER reject_second BEFORE INSERT ON clock_observations WHEN NEW.observation_sequence = 2 BEGIN SELECT RAISE(ABORT, 'test'); END"
    )
    claims = []
    real_claim = queue.claim_batch

    def capture_claim(**kwargs):
        lease = real_claim(**kwargs)
        claims.append(lease)
        return lease

    monkeypatch.setattr(queue, "claim_batch", capture_claim)
    result = persistence.attempt(max_entities=2)
    assert result.outcome is Outcome.NOT_COMMITTED
    assert connection.execute("SELECT count(*) FROM clock_observations").fetchone() == (
        0,
    )
    assert queue.snapshot().published_entities == 2
    assert queue.snapshot().claimed_entities == 2
    lease = claims[0]
    assert lease.entries[0].entity is first
    assert lease.entries[1].entity is second


# Losing the COMMIT reply retains the active lease even though real SQLite made the row durable.
def test_unknown_commit_keeps_active_lease(setup):
    path, connection, queue, clock, create = setup

    class LostReply(SqliteTransactions):
        def commit(self, connection):
            super().commit(connection)
            raise OSError("lost reply")

    persistence = create(LostReply())
    persistence.enable_admission()
    _publish(queue, _observation())
    result = persistence.attempt(max_entities=1)
    assert result.outcome is Outcome.OUTCOME_UNKNOWN
    assert queue.snapshot().claimed_entities == 1
    assert queue.snapshot().published_entities == 1
    assert connection.execute("SELECT count(*) FROM clock_observations").fetchone() == (
        1,
    )


# Construction rejects a missing lifecycle row and never silently enables admission.
def test_requires_durable_instance(setup):
    path, connection, queue, clock, create = setup
    with pytest.raises(ValueError, match="durable active instance"):
        OrdinaryPersistence(
            connection,
            queue,
            instance=ReceiverInstanceStart(
                bytes.fromhex("11112233445546778899aabbccddeeff"), 0
            ),
            database_path=path,
            group_id=GROUP,
            clock=clock,
        )
    assert queue.snapshot().admission_snapshot.generation == 0


# Ordinary profile and diagnostic rows retain all queued columns and receive no UTC enrichment.
def test_nonreading_rows_are_exact(setup):
    from cura_receiver.persist_queue_entities import (
        DIAGNOSTIC_V1_SPEC,
        PROFILE_ONLY_V1_SPEC,
        ProfileOnlyUnitV1,
    )

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(queue, ProfileOnlyUnitV1(_profile()), PROFILE_ONLY_V1_SPEC)
    _publish(queue, _diagnostic(), DIAGNOSTIC_V1_SPEC)
    assert persistence.attempt(max_entities=5).acknowledged_entities == 2
    assert connection.execute("SELECT * FROM message_profiles").fetchone() == (
        INSTANCE,
        1,
        10,
        0,
        bytes(255),
        None,
        None,
        None,
        None,
        0,
        None,
        None,
        None,
        2,
        0,
        7,
        0,
        1,
        None,
        0,
        0,
        0,
        0,
        None,
        11,
        12,
        None,
        None,
        None,
        14,
        0,
    )
    assert connection.execute("SELECT * FROM diagnostics").fetchone() == (
        INSTANCE,
        1,
        10,
        3,
        4,
        2,
        1,
        1,
        64,
        bytes.fromhex("00000101") + bytes(124),
    )


# Health is sampled even before a failing BEGIN, then the exact row survives elapsed time and retry.
def test_health_frozen_before_first_transaction(setup):
    from cura_receiver.persist_queue_entities import RECEIVER_HEALTH_REQUEST_V1_SPEC
    from cura_receiver.ports.host_observations import HostObservations

    path, connection, queue, clock, create = setup

    class Host:
        calls = 0

        def sample(self):
            self.calls += 1
            assert not connection.in_transaction
            return HostObservations(linux_load_1m_milli=1250, sqlite_wal_size_bytes=0)

    class FailBeginOnce(SqliteTransactions):
        failed = False

        def begin(self, connection):
            if not self.failed:
                self.failed = True
                raise OSError("temporary access")
            super().begin(connection)

    host = Host()
    persistence = create(FailBeginOnce(), host_observations=host)
    persistence.enable_admission()
    request = _health_request()
    _publish(queue, request, RECEIVER_HEALTH_REQUEST_V1_SPEC)
    assert persistence.attempt(max_entities=1).outcome is Outcome.NOT_COMMITTED
    assert host.calls == 1
    clock.advance_elapsed_us(1_000_000)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert host.calls == 1
    actual = connection.execute("SELECT * FROM receiver_health").fetchone()
    expected = (
        INSTANCE,
        1,
        15,
        enum.RadioState.RX_SINGLE.value,
        0,
        0,
        0,
        *([0] * 8),
        0,
        1,
        0,
        0,
        None,
        None,
        *([0] * 23),
        100,
        1,
        1,
        100,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        1250,
        None,
        None,
        None,
        None,
        0,
        None,
    )
    assert actual == expected
    assert request == _health_request()


# Every clock u64 remains an exact SQLite INTEGER at INT64_MAX, with overflow retaining work.
@pytest.mark.parametrize(
    "field",
    ["observation_sequence", "clock_state_generation", "sampled_at_monotonic_us"],
)
def test_component_integer_boundary(setup, field):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(queue, replace(_observation(), **{field: (1 << 63) - 1}))
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert connection.execute(
        f"SELECT {field}, typeof({field}) FROM clock_observations"
    ).fetchone() == ((1 << 63) - 1, "integer")
    _publish(queue, replace(_observation(sequence=2), **{field: 1 << 63}))
    assert persistence.attempt(max_entities=1).outcome is Outcome.NOT_COMMITTED
    assert queue.snapshot().published_entities == 1
    assert connection.execute("SELECT count(*) FROM clock_observations").fetchone() == (
        1,
    )


# Reviewed FIFO cases cover every classification, both conflict forms and current/backlog conversion.
@pytest.mark.parametrize("batch_size", [1, 6])
def test_reading_classifications_and_immutable_evidence(setup, batch_size):
    from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    units = [
        _measurement(),
        _measurement(sequence=2),
        _measurement(sequence=3, message=101, domain=2),
        _measurement(sequence=4, message=102, soil=1100),
        _measurement(sequence=5, domain=2),
        _measurement(sequence=6, sample=201),
    ]
    for unit in units:
        _publish(queue, unit, MEASUREMENT_PROFILE_V1_SPEC)
    while queue.snapshot().published_entities:
        assert persistence.attempt(max_entities=batch_size).outcome is Outcome.COMMITTED
    assert connection.execute(
        "SELECT occurrence_sequence, persistence_classification_id FROM message_profiles ORDER BY occurrence_sequence"
    ).fetchall() == [(1, 1), (2, 2), (3, 3), (4, 4), (5, 4), (6, 5)]
    expected = []
    for message, soil, canonical, first in [
        (100, 1000, 1, 1),
        (101, 1000, 0, 3),
        (102, 1100, 0, 4),
    ]:
        body = (
            (200).to_bytes(4, "little")
            + bytes.fromhex("3412")
            + soil.to_bytes(2, "little")
            + bytes.fromhex("d00785ffc801ebfca0860100381508033075c4090702ff03")
        )
        expected.append(
            (
                b"n" * 8,
                message,
                200,
                body,
                canonical,
                4660,
                soil,
                2000,
                -123,
                456,
                -789,
                100000,
                5432,
                8,
                3,
                30000,
                2500,
                7,
                2,
                1023,
                INSTANCE,
                first,
            )
        )
    assert (
        connection.execute(
            "SELECT * FROM reading_messages ORDER BY message_id"
        ).fetchall()
        == expected
    )
    assert connection.execute(
        "SELECT occurrence_sequence, received_frame FROM message_profiles ORDER BY occurrence_sequence"
    ).fetchall() == [(i, u.profile.received_frame) for i, u in enumerate(units, 1)]
    assert connection.execute("SELECT count(*) FROM diagnostics").fetchone() == (0,)


# Transport and sample identities are node-scoped, and earlier instances retain canonical ownership.
def test_classification_node_and_instance_scope(setup):
    from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC

    path, connection, queue, clock, create = setup
    other = bytes.fromhex("11112233445546778899aabbccddeeff")
    insert_receiver_instance_start(
        connection, ReceiverInstanceStart(other, 1), b"b" * 16
    )
    persistence = create()
    persistence.enable_admission()
    for unit in [
        _measurement(),
        _measurement(instance=other),
        _measurement(sequence=2, node=b"x" * 8, instance=other),
    ]:
        _publish(queue, unit, MEASUREMENT_PROFILE_V1_SPEC)
    assert persistence.attempt(max_entities=3).acknowledged_entities == 3
    assert connection.execute(
        "SELECT first_receiver_instance_id FROM reading_messages WHERE node_id=?",
        (b"n" * 8,),
    ).fetchone() == (INSTANCE,)
    assert connection.execute(
        "SELECT persistence_classification_id FROM message_profiles WHERE receiver_instance_id=? ORDER BY occurrence_sequence",
        (other,),
    ).fetchall() == [(2,), (1,)]


# Real constraints at either write boundary cannot leave half of an accepted pair durable.
@pytest.mark.parametrize("table", ["message_profiles", "reading_messages"])
def test_pair_write_failure_is_atomic(setup, table):
    from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(queue, _measurement(), MEASUREMENT_PROFILE_V1_SPEC)
    connection.execute(
        f"CREATE TEMP TRIGGER deny_pair BEFORE INSERT ON {table} BEGIN SELECT RAISE(ABORT, 'test'); END"
    )
    assert persistence.attempt(max_entities=1).outcome is Outcome.NOT_COMMITTED
    assert connection.execute("SELECT count(*) FROM reading_messages").fetchone() == (
        0,
    )
    assert connection.execute("SELECT count(*) FROM message_profiles").fetchone() == (
        0,
    )
    assert queue.snapshot().published_entities == 1


class _LoseOneCommitReply(SqliteTransactions):
    lost = False

    def commit(self, connection):
        super().commit(connection)
        if not self.lost:
            self.lost = True
            raise OSError("lost commit reply")


def _kind_case(kind):
    from cura_receiver import persist_queue_entities as entities

    if kind == "clock":
        return (
            _observation(),
            entities.CLOCK_OBSERVATION_V1_SPEC,
            "clock_observations",
            row.CLOCK_OBSERVATION_V1_COLUMNS,
        )
    if kind == "diagnostic":
        return (
            _diagnostic(),
            entities.DIAGNOSTIC_V1_SPEC,
            "diagnostics",
            row.DIAGNOSTIC_V1_COLUMNS,
        )
    if kind == "profile":
        return (
            entities.ProfileOnlyUnitV1(_profile()),
            entities.PROFILE_ONLY_V1_SPEC,
            "message_profiles",
            row.MESSAGE_PROFILE_ROW_V1_COLUMNS,
        )
    if kind == "health":
        return (
            _health_request(),
            entities.RECEIVER_HEALTH_REQUEST_V1_SPEC,
            "receiver_health",
            row.RECEIVER_HEALTH_V1_COLUMNS,
        )
    return (
        _measurement(),
        entities.MEASUREMENT_PROFILE_V1_SPEC,
        "message_profiles",
        row.MESSAGE_PROFILE_ROW_V1_COLUMNS,
    )


# Every identity reconciles an actual durable lost-reply commit without altering or resampling it.
@pytest.mark.parametrize(
    "kind", ["clock", "diagnostic", "profile", "health", "measurement"]
)
def test_exact_reconciliation_for_every_identity(setup, kind):
    from cura_receiver.ports.host_observations import HostObservations

    path, connection, queue, clock, create = setup

    class Host:
        calls = 0

        def sample(self):
            self.calls += 1
            return HostObservations(memory_available_bytes=self.calls)

    host = Host()
    persistence = create(_LoseOneCommitReply(), host_observations=host)
    persistence.enable_admission()
    entity, spec, table, columns = _kind_case(kind)
    _publish(queue, entity, spec)
    assert persistence.attempt(max_entities=1).outcome is Outcome.OUTCOME_UNKNOWN
    before = connection.execute(f"SELECT * FROM {table}").fetchall()
    clock.advance_elapsed_us(1_000_000)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert connection.execute(f"SELECT * FROM {table}").fetchall() == before
    assert host.calls == (1 if kind == "health" else 0)


def _collision_columns():
    for kind in ("clock", "diagnostic", "profile", "health", "measurement"):
        _, _, _, columns = _kind_case(kind)
        for column in columns[2:]:
            yield pytest.param(kind, column, id=f"{kind}-{column}")


# A differing value in any non-key column fails exact reconciliation and preserves conflicting evidence.
@pytest.mark.parametrize("kind,column", list(_collision_columns()))
def test_every_replay_column_is_compared(setup, kind, column):
    from cura_receiver.ports.host_observations import HostObservations

    path, connection, queue, clock, create = setup

    class Host:
        def sample(self):
            return HostObservations()

    persistence = create(_LoseOneCommitReply(), host_observations=Host())
    persistence.enable_admission()
    entity, spec, table, columns = _kind_case(kind)
    _publish(queue, entity, spec)
    assert persistence.attempt(max_entities=1).outcome is Outcome.OUTCOME_UNKNOWN
    original = connection.execute(f"SELECT {column} FROM {table}").fetchone()[0]
    # This is a copied test database invariant defect, made with real SQL. The
    # row still satisfies STRICT, CHECK and FK constraints; replay must detect it.
    if type(original) is bytes:
        replacement = bytes([original[0] ^ 1]) + original[1:]
    elif original is None:
        replacement = {
            "claimed_node_id": b"z" * 8,
            "ack_frame": bytes(23),
            "received_frame": bytes(255),
        }.get(column, 0)
    elif column in ("header_authenticated", "step_discontinuity_boundary"):
        replacement = 1 - original
    elif column in ("error_domain_id", "severity_id", "context_schema_id"):
        replacement = 1 if original != 1 else 0
    elif column == "ack_selection_id":
        replacement = 3 if original != 3 else 0
    elif column == "ack_selected_id":
        replacement = 3 if original != 3 else 0
    elif column == "processing_result_id" and original == 11:
        replacement = 1
    elif column == "persistence_classification_id":
        replacement = 0 if original else 1
    else:
        replacement = original + 1
    trigger_sql = connection.execute(
        "SELECT sql FROM sqlite_master WHERE name=?", (table + "_no_update",)
    ).fetchone()[0]
    connection.execute(f"DROP TRIGGER {table}_no_update")
    connection.execute(f"UPDATE {table} SET {column}=?", (replacement,))
    connection.execute(trigger_sql)
    conflict = connection.execute(f"SELECT * FROM {table}").fetchall()
    assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
    assert connection.execute("PRAGMA foreign_key_check").fetchall() == []
    clock.advance_elapsed_us(1_000_000)
    result = persistence.attempt(max_entities=1)
    assert result.acknowledged_entities == 0
    assert (
        result.failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    )
    assert queue.snapshot().claimed_entities == 1
    assert connection.execute(f"SELECT * FROM {table}").fetchall() == conflict
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (0,)


# Stored reading classifications remain authoritative when their occurrences are re-admitted in a later batch.
def test_measurement_replay_keeps_all_stored_classifications(setup):
    from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    units = [
        _measurement(),
        _measurement(sequence=2),
        _measurement(sequence=3, message=101, domain=2),
        _measurement(sequence=4, message=102, soil=1100),
        _measurement(sequence=5, domain=2),
        _measurement(sequence=6, sample=201),
    ]
    for unit in units:
        _publish(queue, unit, MEASUREMENT_PROFILE_V1_SPEC)
    assert persistence.attempt(max_entities=6).acknowledged_entities == 6
    before_profiles = connection.execute("SELECT * FROM message_profiles").fetchall()
    before_readings = connection.execute("SELECT * FROM reading_messages").fetchall()
    for unit in units:
        _publish(queue, unit, MEASUREMENT_PROFILE_V1_SPEC)
    assert persistence.attempt(max_entities=6).acknowledged_entities == 6
    assert (
        connection.execute("SELECT * FROM message_profiles").fetchall()
        == before_profiles
    )
    assert (
        connection.execute("SELECT * FROM reading_messages").fetchall()
        == before_readings
    )


# A frozen retry may no-op a matching identity and insert a still-absent FIFO identity atomically.
def test_mixed_existing_and_absent_replay(setup):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(queue, _observation())
    persistence.attempt(max_entities=1)
    _publish(queue, _observation())
    _publish(queue, _observation(sequence=2))
    assert persistence.attempt(max_entities=2).acknowledged_entities == 2
    assert connection.execute(
        "SELECT observation_sequence FROM clock_observations ORDER BY observation_sequence"
    ).fetchall() == [(1,), (2,)]


# Missing current reading effects are inserted from the authoritative profile; the reverse partial effect fails closed.
@pytest.mark.parametrize("missing", ["reading", "profile"])
def test_measurement_partial_effects(setup, missing):
    from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC

    path, connection, queue, clock, create = setup
    persistence = create(_LoseOneCommitReply())
    persistence.enable_admission()
    _publish(queue, _measurement(), MEASUREMENT_PROFILE_V1_SPEC)
    assert persistence.attempt(max_entities=1).outcome is Outcome.OUTCOME_UNKNOWN
    table = "reading_messages" if missing == "reading" else "message_profiles"
    connection.execute("PRAGMA foreign_keys=OFF")
    trigger = connection.execute(
        "SELECT sql FROM sqlite_master WHERE name=?", (table + "_no_delete",)
    ).fetchone()[0]
    connection.execute(f"DROP TRIGGER {table}_no_delete")
    connection.execute(f"DELETE FROM {table}")
    connection.execute(trigger)
    connection.execute("PRAGMA foreign_keys=ON")
    clock.advance_elapsed_us(1_000_000)
    result = persistence.attempt(max_entities=1)
    if missing == "reading":
        assert result.acknowledged_entities == 1
        assert connection.execute(
            "SELECT count(*) FROM reading_messages"
        ).fetchone() == (1,)
    else:
        # Recovery validation sees the broken FK before semantic replay can run.
        assert result.acknowledged_entities == 0
        assert (
            result.failure.admission_state
            is enum.PersistenceAdmissionState.UNAVAILABLE_CORRUPT
        )
        assert queue.snapshot().claimed_entities == 1


# Every current reading effect column participates in reconciliation, including decoded and owner evidence.
@pytest.mark.parametrize(
    "column", row.READING_MESSAGE_ROW_V1_COLUMNS[2:-2] + ("first_occurrence_sequence",)
)
def test_reading_side_effect_collision(setup, column):
    from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC

    path, connection, queue, clock, create = setup
    persistence = create(_LoseOneCommitReply())
    persistence.enable_admission()
    _publish(queue, _measurement(), MEASUREMENT_PROFILE_V1_SPEC)
    _publish(queue, _measurement(sequence=2), MEASUREMENT_PROFILE_V1_SPEC)
    assert persistence.attempt(max_entities=2).outcome is Outcome.OUTCOME_UNKNOWN
    original = connection.execute(f"SELECT {column} FROM reading_messages").fetchone()[
        0
    ]
    if type(original) is bytes:
        altered = bytes([original[0] ^ 1]) + original[1:]
    elif column == "is_canonical_for_sample":
        altered = 0
    elif column == "flags":
        altered = 511
    else:
        altered = original + 1
    trigger = connection.execute(
        "SELECT sql FROM sqlite_master WHERE name='reading_messages_no_update'"
    ).fetchone()[0]
    connection.execute("DROP TRIGGER reading_messages_no_update")
    connection.execute(f"UPDATE reading_messages SET {column}=?", (altered,))
    connection.execute(trigger)
    before = connection.execute("SELECT * FROM reading_messages").fetchall()
    clock.advance_elapsed_us(1_000_000)
    result = persistence.attempt(max_entities=2)
    assert (
        result.failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    )
    assert queue.snapshot().published_entities == 2
    assert queue.snapshot().claimed_entities == 2
    assert connection.execute("SELECT * FROM reading_messages").fetchall() == before


# Even an unknown outcome in which COMMIT did not run must reconcile the original frozen classification.
def test_unknown_commit_absent_rows_reconcile(setup):
    from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC

    path, connection, queue, clock, create = setup

    class NoCommitReply(SqliteTransactions):
        failed = False

        def commit(self, connection):
            if not self.failed:
                self.failed = True
                raise OSError("commit did not reach SQLite")
            super().commit(connection)

    persistence = create(NoCommitReply())
    persistence.enable_admission()
    _publish(queue, _measurement(), MEASUREMENT_PROFILE_V1_SPEC)
    _publish(queue, _measurement(sequence=2), MEASUREMENT_PROFILE_V1_SPEC)
    assert persistence.attempt(max_entities=2).outcome is Outcome.OUTCOME_UNKNOWN
    assert connection.execute("SELECT count(*) FROM message_profiles").fetchone() == (
        0,
    )
    clock.advance_elapsed_us(1_000_000)
    assert persistence.attempt(max_entities=2).acknowledged_entities == 2
    assert connection.execute(
        "SELECT persistence_classification_id FROM message_profiles ORDER BY occurrence_sequence"
    ).fetchall() == [(1,), (2,)]


# A real competing writer produces a bounded BUSY failure without releasing the original FIFO values.
def test_real_sqlite_contention_retains_work(setup):
    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(queue, _observation())
    other = sqlite3.connect(path, isolation_level=None)
    try:
        other.execute("BEGIN IMMEDIATE")
        result = persistence.attempt(max_entities=1)
        assert result.failure.sqlite_primary_code == sqlite3.SQLITE_BUSY
        assert (
            result.failure.admission_state
            is enum.PersistenceAdmissionState.UNAVAILABLE_IO
        )
        assert queue.snapshot().published_entities == 1
        assert queue.snapshot().claimed_entities == 0
        assert connection.execute("PRAGMA busy_timeout").fetchone() == (250,)
    finally:
        other.execute("ROLLBACK")
        other.close()


# Preventive free-space refusal retains queued data and cannot be classified as poison.
def test_real_low_space_inspection(setup):
    path, connection, queue, clock, create = setup
    persistence = create(minimum_free_bytes=1 << 63)
    persistence.enable_admission()
    _publish(queue, _observation())
    result = persistence.attempt(max_entities=1)
    assert (
        result.failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_LOW_SPACE
    )
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (0,)
    assert queue.snapshot().published_entities == 1


# SQLite page exhaustion produces a real FULL rollback while the entire mixed-size batch stays queued.
def test_real_sqlite_full_retains_batch(setup):
    from cura_receiver.persist_queue_entities import (
        PROFILE_ONLY_V1_SPEC,
        ProfileOnlyUnitV1,
    )

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    pages = connection.execute("PRAGMA page_count").fetchone()[0]
    connection.execute(f"PRAGMA max_page_count={pages}")
    for sequence in range(1, 501):
        _publish(
            queue, ProfileOnlyUnitV1(_profile(sequence=sequence)), PROFILE_ONLY_V1_SPEC
        )
    result = persistence.attempt(max_entities=500)
    assert (
        result.failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_DISK_FULL
    )
    assert result.failure.sqlite_primary_code == sqlite3.SQLITE_FULL
    assert queue.snapshot().published_entities == 500
    assert connection.execute("SELECT count(*) FROM message_profiles").fetchone() == (
        0,
    )
    assert connection.execute(
        "SELECT count(*) FROM quarantined_entities"
    ).fetchone() == (0,)


# A corruption result during COMMIT closes the owner without checkpointing or deleting its WAL evidence.
def test_corruption_closes_and_preserves_artifacts(setup):
    path, connection, queue, clock, create = setup

    class CorruptCommit(SqliteTransactions):
        def commit(self, connection):
            super().commit(connection)
            self.artifacts = {
                p: (p.stat().st_ino, p.stat().st_size, p.read_bytes())
                for p in (path, Path(str(path) + "-wal"), Path(str(path) + "-shm"))
            }
            error = sqlite3.OperationalError("corruption")
            error.sqlite_errorcode = sqlite3.SQLITE_IOERR_CORRUPTFS
            raise error

    transactions = CorruptCommit()
    persistence = create(transactions)
    persistence.enable_admission()
    _publish(queue, _observation())
    result = persistence.attempt(max_entities=1)
    assert result.outcome is Outcome.OUTCOME_UNKNOWN
    assert (
        result.failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_CORRUPT
    )
    assert queue.snapshot().claimed_entities == 1
    for artifact, (inode, size, content) in transactions.artifacts.items():
        assert (artifact.stat().st_ino, artifact.stat().st_size) == (inode, size)
        if not str(artifact).endswith("-shm"):
            assert artifact.read_bytes() == content
    with pytest.raises(sqlite3.ProgrammingError):
        connection.execute("SELECT 1")


# Revalidation rejects changed durability settings and missing required relations before recovery.
@pytest.mark.parametrize(
    "change,expected",
    [
        ("PRAGMA synchronous=NORMAL", enum.PersistenceAdmissionState.UNAVAILABLE_IO),
        (
            "DROP TABLE diagnostics",
            enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA,
        ),
    ],
)
def test_connection_revalidation(setup, change, expected):
    from cura_receiver.sqlite_database import validate_receiver_connection

    path, connection, queue, clock, create = setup
    assert validate_receiver_connection(connection, GROUP) is None
    connection.execute(change)
    assert validate_receiver_connection(connection, GROUP).admission_state is expected


# Semantic replay discovers impossible current ownership before any recovery integrity check has run.
def test_impossible_partial_ownership_is_incompatible(setup):
    from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    _publish(queue, _measurement(), MEASUREMENT_PROFILE_V1_SPEC)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    connection.execute("PRAGMA foreign_keys=OFF")
    connection.execute("DROP TRIGGER message_profiles_no_delete")
    connection.execute("DELETE FROM message_profiles")
    connection.execute("PRAGMA foreign_keys=ON")
    _publish(queue, _measurement(), MEASUREMENT_PROFILE_V1_SPEC)
    result = persistence.attempt(max_entities=1)
    assert (
        result.failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    )
    assert queue.snapshot().claimed_entities == 1
    assert connection.execute("SELECT count(*) FROM reading_messages").fetchone() == (
        1,
    )


# Exact virtual boundaries enforce the initial wait, exponential cap and arbitrarily many recovery attempts.
def test_recovery_deadlines_and_cap(setup):
    path, connection, queue, clock, create = setup

    class BlockBegin(SqliteTransactions):
        blocked = True
        calls = 0

        def begin(self, connection):
            self.calls += 1
            if self.blocked:
                raise OSError("temporary access")
            super().begin(connection)

    transactions = BlockBegin()
    persistence = create(transactions)
    persistence.enable_admission()
    _publish(queue, _observation())
    assert persistence.attempt(max_entities=1).outcome is Outcome.NOT_COMMITTED
    for wait in [250925, 501850, 1003700, 2007400, 4014800] + [5018500] * 20:
        deadline = persistence.retry_deadline_monotonic_us
        assert deadline == clock.now_monotonic_us() + wait
        count = transactions.calls
        # Repeated caller wakeups and virtual realtime changes cannot dispatch early.
        clock.step_realtime_us(-100)
        assert persistence.attempt(max_entities=1) is None
        clock.advance_elapsed_us(wait - 1)
        assert persistence.attempt(max_entities=1) is None
        assert transactions.calls == count
        clock.advance_elapsed_us(1)
        assert persistence.attempt(max_entities=1).outcome is Outcome.NOT_COMMITTED
    transactions.blocked = False
    clock.advance_elapsed_us(
        persistence.retry_deadline_monotonic_us - clock.now_monotonic_us()
    )
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert persistence.retry_deadline_monotonic_us is None
    assert (
        queue.snapshot().admission_snapshot.state
        is enum.PersistenceAdmissionState.AVAILABLE
    )
    _publish(queue, _observation(sequence=2))
    transactions.blocked = True
    persistence.attempt(max_entities=1)
    assert persistence.retry_deadline_monotonic_us == clock.now_monotonic_us() + 250925


# Recovery validates again before commit, so an operator-changed incompatible database cannot reopen admission.
def test_recovery_requires_validation_then_commit(setup):
    path, connection, queue, clock, create = setup
    persistence = create(_LoseOneCommitReply())
    persistence.enable_admission()
    _publish(queue, _observation())
    persistence.attempt(max_entities=1)
    connection.execute("PRAGMA application_id=1")
    clock.advance_elapsed_us(1_000_000)
    result = persistence.attempt(max_entities=1)
    assert (
        result.failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    )
    assert queue.snapshot().claimed_entities == 1
    connection.execute(f"PRAGMA application_id={enum.SQLITE_APPLICATION_ID}")
    assert persistence.attempt(max_entities=1) is None
    persistence.request_operator_recovery()
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert (
        queue.snapshot().admission_snapshot.state
        is enum.PersistenceAdmissionState.AVAILABLE
    )


# A failed rollback discards the connection and preserves original work for validated reopening.
def test_failed_rollback_reopens_without_reconstruction(setup):
    path, connection, queue, clock, create = setup

    class FailedRollback(SqliteTransactions):
        failed = False

        def begin(self, connection):
            super().begin(connection)
            if not self.failed:
                self.failed = True
                raise OSError("begin reply lost before any writes")

        def rollback(self, connection):
            raise OSError("rollback unavailable")

    persistence = create(FailedRollback())
    persistence.enable_admission()
    _publish(queue, _observation())
    result = persistence.attempt(max_entities=1)
    assert result.outcome is Outcome.NOT_COMMITTED
    assert queue.snapshot().published_entities == 1
    assert queue.snapshot().claimed_entities == 0
    clock.advance_elapsed_us(1_000_000)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    with sqlite3.connect(path) as read:
        assert read.execute("SELECT * FROM clock_observations").fetchone() == (
            INSTANCE,
            1,
            0,
            10,
            None,
            0,
            0,
            1,
        )


# A restored page limit still requires a due, validated commit before full-storage admission recovers.
def test_full_recovery_commits_original_batch(setup):
    from cura_receiver.persist_queue_entities import (
        PROFILE_ONLY_V1_SPEC,
        ProfileOnlyUnitV1,
    )

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    pages = connection.execute("PRAGMA page_count").fetchone()[0]
    connection.execute(f"PRAGMA max_page_count={pages}")
    for sequence in range(1, 101):
        _publish(
            queue, ProfileOnlyUnitV1(_profile(sequence=sequence)), PROFILE_ONLY_V1_SPEC
        )
    assert (
        persistence.attempt(max_entities=100).failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_DISK_FULL
    )
    connection.execute(f"PRAGMA max_page_count={pages+100}")
    assert persistence.attempt(max_entities=100) is None
    assert (
        queue.snapshot().admission_snapshot.state
        is enum.PersistenceAdmissionState.UNAVAILABLE_DISK_FULL
    )
    clock.advance_elapsed_us(250925)
    assert persistence.attempt(max_entities=100).acknowledged_entities == 100
    assert connection.execute("SELECT count(*) FROM message_profiles").fetchone() == (
        100,
    )
    assert (
        queue.snapshot().admission_snapshot.state
        is enum.PersistenceAdmissionState.AVAILABLE
    )


# Replay of a noncanonical transport still requires the canonical evidence behind its first classification.
def test_retransmission_requires_prior_canonical_relation(setup):
    from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    units = [
        _measurement(),
        _measurement(sequence=2, message=101),
        _measurement(sequence=3, message=101),
    ]
    for unit in units:
        _publish(queue, unit, MEASUREMENT_PROFILE_V1_SPEC)
    assert persistence.attempt(max_entities=3).acknowledged_entities == 3
    connection.execute("DROP TRIGGER reading_messages_no_delete")
    connection.execute("DELETE FROM reading_messages WHERE is_canonical_for_sample=1")
    _publish(queue, units[2], MEASUREMENT_PROFILE_V1_SPEC)
    result = persistence.attempt(max_entities=1)
    assert (
        result.failure.admission_state
        is enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    )
    assert queue.snapshot().claimed_entities == 1


# A lost COMMIT reply counts one failed attempt and one successful reconciliation, without double-counting units.
def test_reconciliation_counters_do_not_double_count(setup):
    path, connection, queue, clock, create = setup
    persistence = create(_LoseOneCommitReply())
    persistence.enable_admission()
    _publish(queue, _observation())
    _publish(queue, _observation(sequence=2))
    persistence.attempt(max_entities=2)
    assert persistence.counters.batch_entities_committed == 0
    clock.advance_elapsed_us(250925)
    persistence.attempt(max_entities=2)
    counts = persistence.counters
    assert (
        counts.batch_transaction_attempts,
        counts.batch_transaction_commits,
        counts.batch_transaction_failures,
    ) == (2, 1, 1)
    assert counts.batch_entities_committed == 2


# Cumulative persistence counters saturate at SQLite's INT64_MAX instead of wrapping or poisoning later health.
def test_persistence_counters_saturate(setup):
    from dataclasses import fields

    path, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    maximum = (1 << 63) - 1
    persistence._counters = replace(
        persistence.counters,
        **{field.name: maximum - 1 for field in fields(persistence.counters)},
    )
    _publish(queue, _observation())
    _publish(queue, _observation(sequence=2))
    assert persistence.attempt(max_entities=2).acknowledged_entities == 2
    assert persistence.counters.batch_entities_committed == maximum
    assert persistence.counters.batch_transaction_attempts == maximum
    assert persistence.counters.batch_transaction_commits == maximum
    _publish(queue, _observation(sequence=3))
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert persistence.counters.batch_entities_committed == maximum
    assert all(
        0 <= getattr(persistence.counters, field.name) <= maximum
        for field in fields(persistence.counters)
    )


# The ordinary owner verifies the supplied group/durability handoff before it can publish admission.
@pytest.mark.parametrize("change", ["group", "durability"])
def test_constructor_checks_database_handoff(setup, change):
    from cura_receiver.sqlite_database import StorageUnavailable

    path, connection, queue, clock, create = setup
    if change == "durability":
        connection.execute("PRAGMA synchronous=NORMAL")
    with pytest.raises(StorageUnavailable) as failure:
        OrdinaryPersistence(
            connection,
            queue,
            instance=ReceiverInstanceStart(INSTANCE, 0),
            database_path=path,
            group_id=b"x" * 8 if change == "group" else GROUP,
            clock=clock,
        )
    assert failure.value.failure.admission_state is (
        enum.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
        if change == "group"
        else enum.PersistenceAdmissionState.UNAVAILABLE_IO
    )
    assert queue.snapshot().admission_snapshot.generation == 0

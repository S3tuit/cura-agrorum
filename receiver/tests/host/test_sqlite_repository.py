from __future__ import annotations

import itertools
import json
import sqlite3
from dataclasses import replace
from pathlib import Path

import pytest

from cura_receiver.database_initializer import initialize_database
from cura_receiver.generated import receiver_entities_generated as row
from cura_receiver.generated import receiver_enums_generated as enum
from cura_receiver.receiver_startup import (
    ReceiverInstanceStart,
    insert_receiver_instance_start,
)
from cura_receiver.sqlite_database import open_receiver_database
from cura_receiver.sqlite_repository import SqliteRepository
from tests.support.builders.protocol_ingress import (
    REVIEWED_CURRENT_FRAME,
    REVIEWED_ACCEPTED_ACK,
    REVIEWED_READING_BODY,
)

INSTANCE = bytes.fromhex("00112233445546778899aabbccddeeff")
NODE = bytes.fromhex("0102030405060708")
FRAME = REVIEWED_CURRENT_FRAME + bytes(255 - len(REVIEWED_CURRENT_FRAME))


def _profile() -> row.MessageProfileRowV1:
    return row.MessageProfileRowV1(
        row.MessageProfilingV1(
            receiver_instance_id=INSTANCE,
            occurrence_sequence=1,
            received_at_monotonic_us=10,
            received_frame_length=len(REVIEWED_CURRENT_FRAME),
            received_frame=FRAME,
            claimed_control=0x20,
            claimed_domain=1,
            claimed_node_id=NODE,
            claimed_message_id=0x11223344,
            header_authenticated=True,
            decoded_sample_id=0x55667788,
            rssi_dbm_x2=-140,
            snr_db_x4=12,
            irq_status=2,
            device_errors=0,
            processing_result=enum.ProcessingResult.ACCEPTED,
            ack_selected=enum.AckSelection.ACCEPTED,
            ack_tx_result=enum.AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
            ack_frame=REVIEWED_ACCEPTED_ACK,
            busy_wait_total_us=0,
            busy_wait_max_us=0,
            busy_wait_count=0,
            busy_timeout_count=0,
            last_busy_timeout_opcode=None,
            t1_handler_started_monotonic_us=11,
            t2_packet_copied_monotonic_us=12,
            t3_authentication_completed_monotonic_us=13,
            t4_set_tx_attempted_monotonic_us=None,
            t5_tx_done_monotonic_us=None,
            t6_set_rx_issued_monotonic_us=14,
        ),
        enum.PersistenceClassification.FIRST_SEEN,
    )


def _reading() -> row.ReadingMessageRowV1:
    return row.ReadingMessageRowV1(
        node_id=NODE,
        message_id=0x11223344,
        sample_id=0x55667788,
        reading_body=REVIEWED_READING_BODY,
        is_canonical_for_sample=True,
        run_ms=4660,
        soil_0_mv=1000,
        soil_1_mv=2000,
        soil_temp_0_centi_c=-123,
        soil_temp_1_centi_c=456,
        enclosure_centi_c=-789,
        enclosure_pressure_pa=100000,
        enclosure_humidity_centi_pct=5432,
        reset_reason=8,
        previous_current_tx_attempts=3,
        previous_awake_ms=30000,
        previous_current_delivery_ms=2500,
        previous_cycle_tx_attempts=7,
        previous_cycle_accepted_readings=2,
        flags=1023,
        first_receiver_instance_id=INSTANCE,
        first_occurrence_sequence=1,
    )


def _health() -> row.ReceiverHealthV1:
    return row.ReceiverHealthV1(
        receiver_instance_id=INSTANCE,
        health_sequence=1,
        communicator_sampled_at_monotonic_us=10,
        radio_state=enum.RadioState.RX_SINGLE,
        radio_recovery_attempts=8,
        radio_recovery_successes=7,
        radio_recovery_failures=1,
        radio_recovery_attempts_by_reason=(1, 1, 1, 1, 1, 1, 1, 1),
        system_time_quality=enum.SystemTimeQuality.UNTRUSTED,
        rtc_health=enum.RtcHealth.PRESENT,
        time_quality_transition_count=0,
        rtc_health_transition_count=0,
        last_time_quality_transition_monotonic_us=None,
        last_rtc_health_transition_monotonic_us=None,
        chrony_step_command_results=(1, 2, 3),
        rtc_write_results=(4, 5, 6),
        rtc_write_readback_verified_count=3,
        rtc_write_trust_invalidated_count=2,
        persist_queue_admission_counts=(
            (1, 2, 3),
            (4, 5, 6),
            (7, 8, 9),
            (10, 11, 12),
            (13, 14, 15),
        ),
        persistence_sampled_at_monotonic_us=11,
        persistence_admission_generation=1,
        persistence_admission_state=enum.PersistenceAdmissionState.AVAILABLE,
        persistence_admission_changed_at_monotonic_us=0,
        persistence_admission_transition_counts=(0, 1, 0, 0, 0, 0, 0),
        durable_quarantine_successes=0,
        durable_quarantine_failures=0,
        batch_transaction_attempts=1,
        batch_transaction_commits=1,
        batch_transaction_failures=0,
        batch_entities_committed=1,
        batch_commit_duration_total_us=10,
        batch_commit_duration_max_us=10,
        wal_checkpoint_attempts=0,
        wal_checkpoint_successes=0,
        wal_checkpoint_failures=0,
        linux_load_1m_milli=None,
        cpu_temperature_milli_c=-100,
        memory_available_bytes=1000,
        sqlite_filesystem_available_bytes=2000,
        sqlite_database_size_bytes=3000,
        sqlite_wal_size_bytes=0,
        ntp_offset_us=None,
    )


def _examples():
    # Explicit SQL expectations; no generated binder/codec computes an oracle.
    return (
        (
            "clock_observation",
            row.ClockObservationV1(
                INSTANCE,
                1,
                0,
                10,
                None,
                False,
                enum.SystemTimeQuality.UNTRUSTED,
                enum.RtcHealth.PRESENT,
            ),
            (INSTANCE, 1, 0, 10, None, 0, 0, 1),
            (INSTANCE, 1),
        ),
        (
            "diagnostic",
            row.DiagnosticV1(
                INSTANCE,
                1,
                10,
                enum.DiagnosticSeverity.ERROR,
                enum.DiagnosticErrorDomain.RADIO,
                enum.DiagnosticOperation.VALIDATE,
                1,
                1,
                0,
                bytes(128),
            ),
            (INSTANCE, 1, 10, 2, 1, 2, 1, 1, 0, bytes(128)),
            (INSTANCE, 1),
        ),
        (
            "quarantined_entity",
            row.QuarantinedEntityRowV1(
                bytes(32),
                enum.PersistQueueEntityKind.PROFILE_ONLY,
                1,
                2,
                b"{}",
                INSTANCE,
                10,
                enum.DATABASE_SCHEMA_VERSION,
                enum.QuarantineFailureReason.ENTITY_DECODING_INVARIANT,
                enum.DiagnosticOperation.VALIDATE,
                None,
                None,
                None,
                1,
            ),
            (
                bytes(32),
                2,
                1,
                2,
                b"{}",
                INSTANCE,
                10,
                enum.DATABASE_SCHEMA_VERSION,
                2,
                2,
                None,
                None,
                None,
                1,
            ),
            (bytes(32),),
        ),
        (
            "message_profile",
            _profile(),
            (
                INSTANCE,
                1,
                10,
                len(REVIEWED_CURRENT_FRAME),
                FRAME,
                32,
                1,
                NODE,
                0x11223344,
                1,
                0x55667788,
                -140,
                12,
                2,
                0,
                11,
                3,
                2,
                REVIEWED_ACCEPTED_ACK,
                0,
                0,
                0,
                0,
                None,
                11,
                12,
                13,
                None,
                None,
                14,
                1,
            ),
            (INSTANCE, 1),
        ),
        (
            "reading_message",
            _reading(),
            (
                NODE,
                0x11223344,
                0x55667788,
                REVIEWED_READING_BODY,
                1,
                4660,
                1000,
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
                1,
            ),
            (NODE, 0x11223344),
        ),
        (
            "receiver_health",
            _health(),
            (INSTANCE, 1, 10, 2, 8, 7, 1)
            + (1,) * 8
            + (0, 1, 0, 0, None, None, 1, 2, 3, 4, 5, 6, 3, 2)
            + tuple(range(1, 16))
            + (
                11,
                1,
                1,
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
                1,
                1,
                0,
                1,
                10,
                10,
                0,
                0,
                0,
                None,
                -100,
                1000,
                2000,
                3000,
                0,
                None,
            ),
            (INSTANCE, 1),
        ),
    )


@pytest.fixture
def database(tmp_path: Path):
    path = tmp_path / "receiver.db"
    initialize_database(path, NODE)
    result = open_receiver_database(path, NODE, minimum_free_bytes=0)
    assert result.failure is None
    connection = result.connection
    insert_receiver_instance_start(
        connection, ReceiverInstanceStart(INSTANCE, 0), bytes(16)
    )
    try:
        yield connection, SqliteRepository(connection), path
    finally:
        connection.close()


# Each concrete method persists a complete exact row, leaves commit to its caller, and rejects collision.
@pytest.mark.parametrize(
    ("kind", "entity", "expected", "key"),
    _examples(),
    ids=[case[0] for case in _examples()],
)
def test_complete_row_insert_lookup_and_collision(
    database, kind: str, entity: object, expected: tuple, key: tuple
) -> None:
    connection, repository, path = database
    insert = getattr(repository, "insert_" + kind)
    find = getattr(repository, "find_" + kind)
    assert find(*key) is None
    connection.execute("BEGIN IMMEDIATE")
    if kind == "reading_message":
        repository.insert_message_profile(_profile())
    insert(entity)
    assert find(*key) == expected
    observer = sqlite3.connect(path)
    try:
        assert getattr(SqliteRepository(observer), "find_" + kind)(*key) is None
        assert connection.in_transaction
        with pytest.raises(sqlite3.IntegrityError, match="append-only"):
            insert(entity)
        assert find(*key) == expected
        connection.execute("COMMIT")
        assert getattr(SqliteRepository(observer), "find_" + kind)(*key) == expected
    finally:
        observer.close()


# No repository insertion can silently become an independent autocommit transaction.
@pytest.mark.parametrize(
    ("kind", "entity", "expected", "key"),
    _examples(),
    ids=[case[0] for case in _examples()],
)
def test_insert_requires_transaction(
    database, kind: str, entity: object, expected: tuple, key: tuple
) -> None:
    _, repository, _ = database
    with pytest.raises(RuntimeError, match="caller-owned transaction"):
        getattr(repository, "insert_" + kind)(entity)
    assert getattr(repository, "find_" + kind)(*key) is None


# A caller's rollback removes both newly inserted effects without changing canonical rows.
def test_reading_profile_rollback_and_canonical_lookup(database) -> None:
    connection, repository, _ = database
    assert repository.find_canonical_sample(NODE, 0x55667788) is None
    assert repository.find_first_profile_frame(NODE, 0x11223344) is None
    connection.execute("BEGIN")
    repository.insert_message_profile(_profile())
    repository.insert_reading_message(_reading())
    assert repository.find_first_profile_frame(NODE, 0x11223344) == (
        len(REVIEWED_CURRENT_FRAME),
        FRAME,
    )
    canonical = repository.find_reading_message(NODE, 0x11223344)
    assert repository.find_canonical_sample(NODE, 0x55667788) == canonical
    with pytest.raises(sqlite3.IntegrityError, match="append-only"):
        repository.insert_reading_message(replace(_reading(), message_id=7))
    connection.execute("ROLLBACK")
    assert repository.find_message_profile(INSTANCE, 1) is None
    assert repository.find_reading_message(NODE, 0x11223344) is None


# Existing lifecycle rows are read completely, in database ordinal order, without updates.
def test_lifecycle_lookups(database) -> None:
    connection, repository, _ = database
    assert repository.find_receiver_instance(INSTANCE) == (
        1,
        INSTANCE,
        bytes(16),
        0,
        None,
        None,
    )
    assert repository.find_previous_receiver_instance(1) is None
    assert repository.find_previous_receiver_instance(
        2
    ) == repository.find_receiver_instance(INSTANCE)
    for sql in (
        "DELETE FROM receiver_instances",
        "UPDATE receiver_instances SET started_at_monotonic_us = 1",
    ):
        with pytest.raises(sqlite3.IntegrityError):
            connection.execute(sql)


# Raw state reads retain every SQL storage class and malformed row for later validation.
def test_state_envelope_is_not_decoded_or_filtered(database) -> None:
    connection, repository, _ = database
    assert repository.read_communicator_state_rows() == ()
    connection.execute("DROP TABLE communicator_state")
    connection.execute(
        "CREATE TABLE communicator_state (singleton_id, state_format_version, generation, state_blob, state_sha256)"
    )
    observed = ((1, 99, 1, b"unknown", bytes(32)), (2, "invalid", None, 7.5, None))
    connection.executemany(
        "INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)", observed
    )
    assert repository.read_communicator_state_rows() == observed


# Real foreign keys forbid a reading's missing profile and a profile's missing instance.
def test_referential_closure(database) -> None:
    connection, repository, _ = database
    connection.execute("BEGIN")
    with pytest.raises(sqlite3.IntegrityError, match="FOREIGN KEY"):
        repository.insert_reading_message(_reading())
    with pytest.raises(sqlite3.IntegrityError, match="FOREIGN KEY"):
        repository.insert_message_profile(
            replace(
                _profile(),
                profile=replace(_profile().profile, receiver_instance_id=bytes(16)),
            )
        )
    connection.execute("ROLLBACK")


# SQLite integer overflow is rejected before execution rather than being cast or stored as REAL.
def test_repository_rejects_integer_overflow_before_sql(database) -> None:
    connection, repository, _ = database
    observation = row.ClockObservationV1(
        INSTANCE,
        1 << 63,
        0,
        10,
        None,
        False,
        enum.SystemTimeQuality.UNTRUSTED,
        enum.RtcHealth.PRESENT,
    )
    connection.execute("BEGIN")
    executed = []
    connection.set_trace_callback(executed.append)
    with pytest.raises(OverflowError, match="observation_sequence"):
        repository.insert_clock_observation(observation)
    assert executed == []


def _u64_cases():
    # Enumerate the normative manifest, independently of the binders/checker.
    manifest = json.loads(
        (
            Path(__file__).resolve().parents[2] / "schemas/receiver_entities.json"
        ).read_text()
    )
    axes = {axis["name"]: axis["members"] for axis in manifest["array_axes"]}
    definitions = list(manifest["logical_records"])
    for definition in manifest["entities"]:
        definitions.append(definition)
        definitions.extend(definition["persistence"].get("targets", ()))
    for kind, entity, _, _ in _examples():
        target = entity.profile if kind == "message_profile" else entity
        definition = next(
            item
            for item in definitions
            if item.get("python_name") == type(target).__name__
        )
        for field in definition["fields"]:
            if field["type"] != "u64":
                continue
            field_axes = field.get("array_axes", [])
            for members in itertools.product(*(axes[axis] for axis in field_axes)):
                indices = tuple(member["source_index"] for member in members)
                column = field.get("column_pattern", field.get("column", field["name"]))
                column = column.format(
                    **{
                        axis: member["name"].lower()
                        for axis, member in zip(field_axes, members)
                    }
                )
                yield pytest.param(
                    kind, entity, field["name"], indices, column, id=f"{kind}.{column}"
                )


def _replace_index(value: tuple, indices: tuple[int, ...], replacement: int):
    if not indices:
        return replacement
    items = list(value)
    items[indices[0]] = _replace_index(items[indices[0]], indices[1:], replacement)
    return tuple(items)


# Every scalar/array u64 column stores INT64_MAX exactly and rejects the next value before SQL.
@pytest.mark.parametrize(
    ("kind", "entity", "field", "indices", "column"), list(_u64_cases())
)
@pytest.mark.parametrize("value", ((1 << 63) - 1, 1 << 63), ids=("maximum", "overflow"))
def test_each_bound_u64(
    database,
    kind: str,
    entity: object,
    field: str,
    indices: tuple,
    column: str,
    value: int,
) -> None:
    connection, repository, _ = database
    target = entity.profile if kind == "message_profile" else entity
    replacement = _replace_index(getattr(target, field), indices, value)
    target = replace(target, **{field: replacement})
    updated = replace(entity, profile=target) if kind == "message_profile" else target
    connection.execute("BEGIN")
    if kind == "reading_message":
        profile = _profile()
        if value < 1 << 63:
            profile = replace(
                profile, profile=replace(profile.profile, occurrence_sequence=value)
            )
        repository.insert_message_profile(profile)
    executed = []
    connection.set_trace_callback(executed.append)
    if value == 1 << 63:
        with pytest.raises(OverflowError, match=column):
            getattr(repository, "insert_" + kind)(updated)
        assert executed == []
    else:
        getattr(repository, "insert_" + kind)(updated)
        table = {
            "clock_observation": "clock_observations",
            "diagnostic": "diagnostics",
            "quarantined_entity": "quarantined_entities",
            "receiver_health": "receiver_health",
            "message_profile": "message_profiles",
            "reading_message": "reading_messages",
        }[kind]
        assert connection.execute(
            f"SELECT {column}, typeof({column}) FROM {table}"
        ).fetchall() == [(value, "integer")]


# Narrow protocol integers retain their schema ranges instead of inheriting the u64 ceiling.
@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("message_id", 1 << 32),
        ("soil_0_mv", 1 << 16),
        ("soil_temp_0_centi_c", -(1 << 15) - 1),
        ("reset_reason", 256),
    ),
)
def test_narrow_integer_bounds(database, field: str, value: int) -> None:
    connection, repository, _ = database
    connection.execute("BEGIN")
    repository.insert_message_profile(_profile())
    with pytest.raises(sqlite3.IntegrityError, match="CHECK"):
        repository.insert_reading_message(replace(_reading(), **{field: value}))
    assert repository.find_reading_message(NODE, 0x11223344) is None


# Numeric strings/floats and Boolean counters cannot exploit SQLite's lossless coercions.
@pytest.mark.parametrize("value", (1.0, "1", True))
def test_integer_coercions_rejected_before_sql(database, value: object) -> None:
    connection, repository, _ = database
    connection.execute("BEGIN")
    executed = []
    connection.set_trace_callback(executed.append)
    with pytest.raises(TypeError):
        repository.insert_receiver_health(replace(_health(), health_sequence=value))
    assert executed == []

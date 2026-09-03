from __future__ import annotations

import copy
import hashlib
import json
import sqlite3
import struct
import subprocess
import sys
from dataclasses import fields, replace
from enum import Enum
from pathlib import Path

import pytest


RECEIVER_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = RECEIVER_ROOT.parent
GENERATOR = RECEIVER_ROOT / "tools" / "generate.py"
MANIFEST = RECEIVER_ROOT / "schemas" / "receiver_enums.json"
ENTITY_MANIFEST = RECEIVER_ROOT / "schemas" / "receiver_entities.json"
SCHEMA = RECEIVER_ROOT / "db" / "schema.sql"
HANDWRITTEN_TABLES = {
    "database_metadata",
    "receiver_instances",
    "quarantined_communicator_states",
}

from cura_receiver.generated import (
    receiver_entities_generated as generated_entities,
)
from cura_receiver.generated import (
    receiver_enums_generated as generated,
)


def pascal_case(name: str) -> str:
    return "".join(part.lower().capitalize() for part in name.split("_"))


def load_manifest() -> dict[str, object]:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def load_entity_manifest() -> dict[str, object]:
    return json.loads(ENTITY_MANIFEST.read_text(encoding="utf-8"))


def validate_entity_manifest_fixture(
    tmp_path: Path,
    entity_manifest: dict[str, object],
) -> subprocess.CompletedProcess[str]:
    fixture_directory = tmp_path / "manifest"
    fixture_directory.mkdir()
    fixture = fixture_directory / "receiver_entities.json"
    fixture.write_text(json.dumps(entity_manifest), encoding="utf-8")
    return subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            "--entities",
            str(fixture),
            "--validate-only",
        ],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def validate_schema_source_fixture(
    tmp_path: Path,
    schema_source: str,
) -> subprocess.CompletedProcess[str]:
    fixture_directory = tmp_path / "schema"
    fixture_directory.mkdir()
    fixture = fixture_directory / "schema_source.sql"
    fixture.write_text(schema_source, encoding="utf-8")
    return subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            "--schema-source",
            str(fixture),
            "--validate-only",
        ],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def open_schema() -> sqlite3.Connection:
    connection = sqlite3.connect(":memory:")
    connection.execute("PRAGMA foreign_keys = ON")
    connection.executescript(SCHEMA.read_text(encoding="utf-8"))
    return connection


def communicator_state(
    *,
    rtc_provenance: generated_entities.RtcProvenanceV1 | None = None,
) -> generated_entities.CommunicatorStateV1:
    empty_bucket = generated_entities.TxAirtimeBucketV1(
        charged_airtime_us=0,
        expires_at_utc_us=0,
    )
    return generated_entities.CommunicatorStateV1(
        generation=1,
        last_observed_system_time_quality=generated.SystemTimeQuality.UNTRUSTED,
        last_observed_rtc_health=generated.RtcHealth.PRESENT,
        rtc_provenance=rtc_provenance,
        rolling_window_us=3_600_000_000,
        tx_airtime_budget_us=36_000_000,
        bucket_width_us=60_000_000,
        bucket_charge_limit_us=8_000_000,
        bucket_expiration_guard_us=1_000_000,
        airtime_snapshot_utc_us=0,
        buckets=(empty_bucket,) * 62,
    )


# Requires every checked-in generated receiver artifact to be current.
def test_generated_outputs_are_current() -> None:
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--check"],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr


# Validates all checked-in receiver generator inputs together.
def test_generator_inputs_validate() -> None:
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--validate-only"],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr


# Matches generated enum values exactly without cross-domain integer equality.
def test_generated_enums_match_manifest_and_are_not_int_subclasses() -> None:
    manifest = load_manifest()
    for enum_spec in manifest["enums"]:  # type: ignore[index]
        enum_class = getattr(generated, pascal_case(enum_spec["name"]))
        assert issubclass(enum_class, Enum)
        assert not issubclass(enum_class, int)
        assert {member.name: member.value for member in enum_class} == {
            value["name"]: value["value"] for value in enum_spec["values"]
        }

    assert generated.SystemTimeQuality.UNTRUSTED != generated.AdmissionResult.RESERVED


# Pins every generated time-diagnostic status assignment.
def test_time_diagnostic_status_assignments_are_exact() -> None:
    expected = {
        generated.ChronyQueryStatus: {
            "OK": 1,
            "UNAVAILABLE": 2,
            "DEADLINE_EXCEEDED": 3,
            "INVALID_RESPONSE": 4,
        },
        generated.ChronyStepDisposition: {
            "SUBMITTED": 1,
            "NOT_SUBMITTED": 2,
            "OUTCOME_UNKNOWN": 3,
        },
        generated.Ds3231ReadStatus: {
            "OK": 1,
            "MISSING": 2,
            "INVALID": 3,
            "IO_ERROR": 4,
            "DEADLINE_EXCEEDED": 5,
        },
        generated.Ds3231WriteDisposition: {
            "COMPLETED": 1,
            "NOT_APPLIED": 2,
            "OUTCOME_UNKNOWN": 3,
        },
        generated.Ds3231Failure: {
            "NONE": 0,
            "MISSING": 1,
            "IO_ERROR": 2,
            "DEADLINE_EXCEEDED": 3,
        },
        generated.AdjtimexReturn: {
            "TIME_OK": 0,
            "TIME_INS": 1,
            "TIME_DEL": 2,
            "TIME_OOP": 3,
            "TIME_WAIT": 4,
            "TIME_ERROR": 5,
        },
    }
    for enum_class, assignments in expected.items():
        assert {member.name: member.value for member in enum_class} == assignments
    assert generated.AdjtimexReturn.TIME_OK.value == 0


# Pins the generated schema fingerprint and schema version to packaged SQL.
def test_schema_fingerprint_is_exact_schema_sql_sha256() -> None:
    schema_bytes = SCHEMA.read_bytes()
    assert hashlib.sha256(schema_bytes).hexdigest() == generated.DATABASE_SCHEMA_SHA256
    assert hashlib.sha256(schema_bytes).digest() == generated.DATABASE_SCHEMA_FINGERPRINT
    assert generated.SQLITE_APPLICATION_ID == 0x43555252
    assert generated.DATABASE_SCHEMA_VERSION == 7


# Requires every declared catalogue, entity table, and trigger in assembled SQL.
def test_schema_contains_declared_catalogues_and_entity_tables() -> None:
    manifest = load_manifest()
    entity_manifest = json.loads(ENTITY_MANIFEST.read_text(encoding="utf-8"))
    expected_catalogues = {
        enum_spec["persistence"]["table"]
        for enum_spec in manifest["enums"]  # type: ignore[index]
        if enum_spec["persistence"]["mode"] != "encoded_only"
    }
    expected_entity_tables: set[str] = set()
    expected_append_only_tables: set[str] = set()
    expected_table_constants: dict[str, str] = {}
    for entity in entity_manifest["entities"]:
        persistence = entity["persistence"]
        if persistence["mode"] == "multi_table_transaction":
            expected_entity_tables.update(
                target["table"] for target in persistence["targets"]
            )
            expected_table_constants.update(
                {
                    target["name"]: target["table"]
                    for target in persistence["targets"]
                }
            )
            expected_append_only_tables.update(
                target["table"]
                for target in persistence["targets"]
                if target["write_policy"] == "append_only"
            )
        else:
            expected_entity_tables.add(persistence["table"])
            expected_table_constants[entity["name"]] = persistence["table"]
            if persistence["write_policy"] == "append_only":
                expected_append_only_tables.add(persistence["table"])
    for entity_name, table in expected_table_constants.items():
        assert getattr(generated_entities, entity_name + "_TABLE") == table

    connection = open_schema()
    actual_tables = {
        row[0]
        for row in connection.execute(
            "SELECT name FROM sqlite_schema "
            "WHERE type = 'table' AND name NOT LIKE 'sqlite_%'"
        )
    }
    assert actual_tables == expected_catalogues | expected_entity_tables | HANDWRITTEN_TABLES

    strict_by_table = {
        row[1]: row[5]
        for row in connection.execute("PRAGMA table_list")
        if row[1] in actual_tables
    }
    assert strict_by_table == {table: 1 for table in actual_tables}
    actual_triggers = {
        row[0]
        for row in connection.execute(
            "SELECT name FROM sqlite_schema WHERE type = 'trigger'"
        )
    }
    assert {
        table + "_no_replace" for table in expected_append_only_tables
    } <= actual_triggers


# Keeps generated entity binding limited to relational projection.
def test_generated_entity_binding_is_projection_only() -> None:
    observation = generated_entities.ClockObservationV1(
        receiver_instance_id=bytes(16),
        observation_sequence=-1,
        clock_state_generation=0,
        sampled_at_monotonic_us=0,
        sampled_at_utc_us=None,
        step_discontinuity_boundary=True,
        system_time_quality=generated.SystemTimeQuality.UNTRUSTED,
        rtc_health=generated.RtcHealth.PRESENT,
    )
    values = generated_entities.clock_observation_v1_parameters(observation)
    assert values[1] == -1
    assert values[4] is None
    assert values[5] is True
    assert values[6:] == (0, 1)

    connection = open_schema()
    connection.execute("PRAGMA foreign_keys = OFF")
    columns = generated_entities.CLOCK_OBSERVATION_V1_COLUMNS
    insert = (
        f"INSERT INTO clock_observations ({', '.join(columns)}) VALUES "
        f"({', '.join('?' for _ in columns)})"
    )
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(insert, values)


# Flattens a logical message-profile record into its exact SQL parameters.
def test_message_profile_logical_record_is_flattened_for_sql() -> None:
    assert not hasattr(generated_entities, "MESSAGE_PROFILING_V1_TABLE")
    assert not hasattr(generated_entities, "MESSAGE_PROFILING_V1_COLUMNS")
    assert not hasattr(generated_entities, "message_profiling_v1_parameters")
    profile = generated_entities.MessageProfilingV1(
        receiver_instance_id=bytes(range(16)),
        occurrence_sequence=7,
        received_at_monotonic_us=11,
        persist_queue_used_bytes_before_admission=13,
        persist_queue_capacity_bytes=17,
        received_frame_length=None,
        received_frame=None,
        claimed_control=None,
        claimed_domain=None,
        claimed_node_id=None,
        claimed_message_id=None,
        header_authenticated=False,
        decoded_sample_id=None,
        rssi_dbm_x2=None,
        snr_db_x4=None,
        irq_status=None,
        device_errors=None,
        processing_result=generated.ProcessingResult.RADIO_ERROR,
        ack_selected=generated.AckSelection.NONE,
        ack_tx_result=generated.AckTxResult.NOT_APPLICABLE,
        ack_frame=None,
        busy_wait_total_us=19,
        busy_wait_max_us=23,
        busy_wait_count=29,
        busy_timeout_count=31,
        last_busy_timeout_opcode=None,
        t1_handler_started_monotonic_us=37,
        t2_packet_copied_monotonic_us=None,
        t3_authentication_completed_monotonic_us=None,
        t4_set_tx_attempted_monotonic_us=None,
        t5_tx_done_monotonic_us=None,
        t6_set_rx_issued_monotonic_us=None,
    )
    row = generated_entities.MessageProfileRowV1(
        profile=profile,
        persistence_classification=generated.PersistenceClassification.NOT_APPLICABLE,
    )

    parameters = generated_entities.message_profile_row_v1_parameters(row)
    columns = generated_entities.MESSAGE_PROFILE_ROW_V1_COLUMNS
    assert len(parameters) == len(columns) == 33
    assert columns[:3] == (
        "receiver_instance_id",
        "occurrence_sequence",
        "received_at_monotonic_us",
    )
    assert parameters[:3] == (bytes(range(16)), 7, 11)
    assert columns[-1] == "persistence_classification_id"
    assert parameters[-1] == generated.PersistenceClassification.NOT_APPLICABLE.value


# Rejects invalid logical-record declarations in the entity manifest.
@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        ("unknown_record", "no logical record UNKNOWN_RECORD_V1"),
        ("missing_flatten", "missing sql"),
        ("nested_record", "logical records cannot be nested"),
        ("duplicate_column", "repeats SQL column receiver_instance_id"),
    ),
)
def test_logical_record_manifest_validation(
    tmp_path: Path,
    mutation: str,
    message: str,
) -> None:
    manifest = copy.deepcopy(load_entity_manifest())
    transaction = next(
        entity
        for entity in manifest["entities"]  # type: ignore[index]
        if entity["name"] == "MESSAGE_PERSISTENCE_TRANSACTION_V1"
    )
    profile_target = transaction["persistence"]["targets"][0]
    profile_field = profile_target["fields"][0]
    if mutation == "unknown_record":
        profile_field["type"] = "logical_record:UNKNOWN_RECORD_V1"
    elif mutation == "missing_flatten":
        del profile_field["sql"]
    elif mutation == "nested_record":
        manifest["logical_records"][0]["fields"][0]["type"] = (
            "logical_record:MESSAGE_PROFILING_V1"
        )
    elif mutation == "duplicate_column":
        profile_target["fields"].append(
            {"name": "receiver_instance_id", "type": "u64"}
        )
    else:  # pragma: no cover - parametrization is closed above.
        raise AssertionError(mutation)

    result = validate_entity_manifest_fixture(tmp_path, manifest)
    assert result.returncode == 2
    assert message in result.stderr


# Rejects generated Python names that collide or are not valid identifiers.
@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        ("enum_class_collision", "repeats Python class"),
        ("python_keyword", "must not be a Python keyword"),
        ("global_target_collision", "repeats entity or target"),
    ),
)
def test_generated_python_names_are_globally_valid(
    tmp_path: Path,
    mutation: str,
    message: str,
) -> None:
    manifest = copy.deepcopy(load_entity_manifest())
    if mutation == "enum_class_collision":
        manifest["logical_records"][0]["python_name"] = "ProcessingResult"  # type: ignore[index]
    elif mutation == "python_keyword":
        manifest["logical_records"].append(  # type: ignore[union-attr]
            {
                "name": "UNUSED_RECORD_V1",
                "python_name": "UnusedRecordV1",
                "fields": [{"name": "class", "type": "u8"}],
            }
        )
    elif mutation == "global_target_collision":
        transaction = next(
            entity
            for entity in manifest["entities"]  # type: ignore[index]
            if entity["name"] == "MESSAGE_PERSISTENCE_TRANSACTION_V1"
        )
        repeated_target = copy.deepcopy(transaction["persistence"]["targets"][0])
        repeated_target["python_name"] = "OtherMessageProfileRowV1"
        repeated_target["table"] = "other_message_profiles"
        manifest["entities"].append(  # type: ignore[union-attr]
            {
                "name": "OTHER_TRANSACTION_V1",
                "persistence": {
                    "mode": "multi_table_transaction",
                    "atomic": True,
                    "targets": [repeated_target],
                },
            }
        )
    else:  # pragma: no cover - parametrization is closed above.
        raise AssertionError(mutation)

    result = validate_entity_manifest_fixture(tmp_path, manifest)
    assert result.returncode == 2
    assert message in result.stderr


# Rejects every transaction boundary in the handwritten schema source.
@pytest.mark.parametrize(
    "transaction_statement",
    (
        "BEGIN;",
        "BEGIN TRANSACTION;",
        "BEGIN DEFERRED TRANSACTION;",
        "BEGIN IMMEDIATE TRANSACTION;",
        "BEGIN EXCLUSIVE TRANSACTION;",
        "COMMIT;",
        "END TRANSACTION;",
        "ROLLBACK;",
        "SAVEPOINT nested;",
    ),
)
def test_schema_source_rejects_every_transaction_boundary(
    tmp_path: Path,
    transaction_statement: str,
) -> None:
    source = (RECEIVER_ROOT / "db" / "schema_source.sql").read_text(
        encoding="utf-8"
    )
    result = validate_schema_source_fixture(
        tmp_path,
        source + "\n" + transaction_statement + "\n",
    )
    assert result.returncode == 2
    assert "transaction boundary" in result.stderr


# Rejects invalid variable-length field relationships in the entity manifest.
@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        ("unknown", "references unknown field missing_length"),
        ("non_integer", "must reference an integer"),
        ("nullable", "cannot be nullable"),
        ("self", "cannot reference itself"),
        ("fixed_bytes", "fixed byte field cannot have length constraints"),
    ),
)
def test_length_field_manifest_validation(
    tmp_path: Path,
    mutation: str,
    message: str,
) -> None:
    manifest = copy.deepcopy(load_entity_manifest())
    quarantine = next(
        entity
        for entity in manifest["entities"]  # type: ignore[index]
        if entity["name"] == "QUARANTINED_ENTITY_ROW_V1"
    )
    fields_by_name = {field["name"]: field for field in quarantine["fields"]}
    entity_bytes = fields_by_name["entity_bytes"]
    if mutation == "unknown":
        entity_bytes["length_field"] = "missing_length"
    elif mutation == "non_integer":
        entity_bytes["length_field"] = "quarantine_id"
    elif mutation == "nullable":
        fields_by_name["entity_length"]["nullable"] = True
    elif mutation == "self":
        entity_bytes["length_field"] = "entity_bytes"
    elif mutation == "fixed_bytes":
        fields_by_name["quarantine_id"]["length_field"] = "entity_length"
    else:  # pragma: no cover - parametrization is closed above.
        raise AssertionError(mutation)

    result = validate_entity_manifest_fixture(tmp_path, manifest)
    assert result.returncode == 2
    assert message in result.stderr


# Enforces equality between quarantined entity bytes and declared length.
def test_quarantined_entity_bytes_must_match_declared_length() -> None:
    row = generated_entities.QuarantinedEntityRowV1(
        quarantine_id=bytes(32),
        entity_kind=generated.PersistQueueEntityKind.PROFILE_ONLY,
        entity_schema_version=1,
        entity_length=2,
        entity_bytes=b"\x02\x01",
        receiver_instance_id=bytes(16),
        quarantined_at_monotonic_us=0,
        database_schema_version=generated.DATABASE_SCHEMA_VERSION,
        failure_reason=generated.QuarantineFailureReason.ENTITY_DECODING_INVARIANT,
        failure_operation=generated.DiagnosticOperation.VALIDATE,
        sqlite_primary_code=None,
        sqlite_extended_code=None,
        os_errno=None,
        isolation_attempt_count=1,
    )
    columns = generated_entities.QUARANTINED_ENTITY_ROW_V1_COLUMNS
    insert = (
        f"INSERT INTO quarantined_entities ({', '.join(columns)}) VALUES "
        f"({', '.join('?' for _ in columns)})"
    )
    connection = open_schema()
    connection.execute("PRAGMA foreign_keys = OFF")
    connection.execute(
        insert,
        generated_entities.quarantined_entity_row_v1_parameters(row),
    )
    replacement = replace(
        row,
        entity_schema_version=2,
        entity_bytes=b"\x03\x01",
    )
    with pytest.raises(sqlite3.IntegrityError, match="append-only"):
        connection.execute(
            insert.replace("INSERT", "INSERT OR REPLACE", 1),
            generated_entities.quarantined_entity_row_v1_parameters(replacement),
        )
    assert connection.execute(
        "SELECT entity_schema_version, entity_bytes FROM quarantined_entities"
    ).fetchone() == (1, b"\x02\x01")
    maximum_measurement = replace(
        row,
        quarantine_id=bytes([2]) * 32,
        entity_kind=generated.PersistQueueEntityKind.MEASUREMENT_PROFILE,
        entity_length=490,
        entity_bytes=bytes(range(245)) * 2,
    )
    connection.execute(
        insert,
        generated_entities.quarantined_entity_row_v1_parameters(
            maximum_measurement
        ),
    )
    assert connection.execute(
        "SELECT length(entity_bytes) FROM quarantined_entities "
        "WHERE quarantine_id = ?",
        (maximum_measurement.quarantine_id,),
    ).fetchone() == (490,)
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            insert,
            generated_entities.quarantined_entity_row_v1_parameters(
                replace(
                    row,
                    quarantine_id=bytes([1]) * 32,
                    entity_length=3,
                )
            ),
        )


# Enforces append-only receiver instances with exactly one valid clean stop.
def test_receiver_instance_lifecycle_is_append_only_with_one_clean_stop() -> None:
    connection = open_schema()
    first_id = bytes.fromhex("00112233445566778899aabbccddeeff")
    second_id = bytes.fromhex("102132435465768798a9bacbdcedfe0f")
    boot_id = bytes.fromhex("ffeeddccbbaa99887766554433221100")
    insert = (
        "INSERT INTO receiver_instances "
        "(receiver_instance_id, linux_boot_id, started_at_monotonic_us) "
        "VALUES (?, ?, ?)"
    )
    with pytest.raises(sqlite3.IntegrityError, match="database ordered"):
        connection.execute(
            "INSERT INTO receiver_instances "
            "(instance_ordinal, receiver_instance_id, linux_boot_id, "
            "started_at_monotonic_us) VALUES (?, ?, ?, ?)",
            (100, bytes([3]) * 16, boot_id, 50),
        )
    assert connection.execute(
        "SELECT count(*) FROM receiver_instances"
    ).fetchone() == (0,)
    connection.execute(insert, (first_id, boot_id, 100))
    connection.execute(insert, (second_id, boot_id, 300))
    assert connection.execute(
        "SELECT instance_ordinal, receiver_instance_id "
        "FROM receiver_instances ORDER BY instance_ordinal"
    ).fetchall() == [(1, first_id), (2, second_id)]

    with pytest.raises(sqlite3.IntegrityError, match="database ordered"):
        connection.execute(
            "INSERT INTO receiver_instances "
            "(instance_ordinal, receiver_instance_id, linux_boot_id, "
            "started_at_monotonic_us) VALUES (?, ?, ?, ?)",
            (100, bytes([5]) * 16, boot_id, 400),
        )
    with pytest.raises(sqlite3.IntegrityError, match="append-only"):
        connection.execute(
            "INSERT OR REPLACE INTO receiver_instances "
            "(instance_ordinal, receiver_instance_id, linux_boot_id, "
            "started_at_monotonic_us) VALUES (?, ?, ?, ?)",
            (1, bytes([4]) * 16, boot_id, 500),
        )
    assert connection.execute(
        "SELECT receiver_instance_id FROM receiver_instances "
        "WHERE instance_ordinal = 1"
    ).fetchone() == (first_id,)

    connection.execute(
        "UPDATE receiver_instances "
        "SET clean_stopped_at_monotonic_us = ?, clean_stop_state_generation = ? "
        "WHERE receiver_instance_id = ?",
        (200, 0, first_id),
    )
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            "UPDATE receiver_instances SET clean_stop_state_generation = 1 "
            "WHERE receiver_instance_id = ?",
            (first_id,),
        )
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            "UPDATE receiver_instances SET clean_stopped_at_monotonic_us = ? "
            "WHERE receiver_instance_id = ?",
            (400, second_id),
        )
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            "UPDATE receiver_instances "
            "SET receiver_instance_id = ?, "
            "clean_stopped_at_monotonic_us = ?, "
            "clean_stop_state_generation = ? "
            "WHERE receiver_instance_id = ?",
            (bytes(16), 400, 1, second_id),
        )
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            "DELETE FROM receiver_instances WHERE receiver_instance_id = ?",
            (first_id,),
        )
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(insert, (first_id, boot_id, 500))
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            "INSERT INTO receiver_instances "
            "(receiver_instance_id, linux_boot_id, started_at_monotonic_us, "
            "clean_stopped_at_monotonic_us, clean_stop_state_generation) "
            "VALUES (?, ?, ?, ?, ?)",
            (bytes([2]) * 16, boot_id, 500, 600, 1),
        )


# Preserves exact SQLite storage classes for rejected communicator state.
def test_quarantined_communicator_state_preserves_storage_classes() -> None:
    connection = open_schema()
    receiver_instance_id = bytes.fromhex("00112233445566778899aabbccddeeff")
    connection.execute(
        "INSERT INTO receiver_instances "
        "(receiver_instance_id, linux_boot_id, started_at_monotonic_us) "
        "VALUES (?, ?, ?)",
        (receiver_instance_id, bytes(16), 0),
    )
    insert = (
        "INSERT INTO quarantined_communicator_states "
        "(observed_singleton_id, observed_state_format_version, "
        "observed_generation, observed_state_blob, observed_state_sha256, "
        "calculated_blob_sha256, preserved_by_receiver_instance_id, "
        "preserved_at_monotonic_us, database_schema_version) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)"
    )
    connection.execute(
        insert,
        (
            None,
            1,
            1.25,
            "not-a-blob",
            b"not-a-digest",
            bytes(32),
            receiver_instance_id,
            10,
            generated.DATABASE_SCHEMA_VERSION,
        ),
    )
    assert connection.execute(
        "SELECT typeof(observed_singleton_id), "
        "typeof(observed_state_format_version), typeof(observed_generation), "
        "typeof(observed_state_blob), typeof(observed_state_sha256) "
        "FROM quarantined_communicator_states"
    ).fetchone() == ("null", "integer", "real", "text", "blob")

    with pytest.raises(sqlite3.IntegrityError, match="append-only"):
        connection.execute(
            "INSERT OR REPLACE INTO quarantined_communicator_states "
            "(quarantined_state_id, observed_generation, "
            "preserved_by_receiver_instance_id, preserved_at_monotonic_us, "
            "database_schema_version) VALUES (?, ?, ?, ?, ?)",
            (1, 99, receiver_instance_id, 20, generated.DATABASE_SCHEMA_VERSION),
        )
    assert connection.execute(
        "SELECT observed_generation FROM quarantined_communicator_states "
        "WHERE quarantined_state_id = 1"
    ).fetchone() == (1.25,)

    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            "UPDATE quarantined_communicator_states "
            "SET observed_generation = 2"
        )
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute("DELETE FROM quarantined_communicator_states")
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            insert,
            (None, None, None, None, None, bytes(31), receiver_instance_id, 11, 1),
        )
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            insert,
            (None, None, None, None, None, None, bytes([1]) * 16, 11, 1),
        )


# Rejects generated foreign keys without a complete valid parent key.
@pytest.mark.parametrize(
    ("parent_table", "parent_column", "message"),
    (
        ("missing_receiver_instances", "receiver_instance_id", "missing table"),
        ("receiver_instances", "missing_receiver_instance_id", "missing columns"),
        ("receiver_instances", "linux_boot_id", "not a complete PRIMARY KEY or UNIQUE key"),
    ),
)
def test_assembled_schema_foreign_key_targets_must_be_valid(
    tmp_path: Path,
    parent_table: str,
    parent_column: str,
    message: str,
) -> None:
    manifest = copy.deepcopy(load_entity_manifest())
    observation = next(
        entity
        for entity in manifest["entities"]  # type: ignore[index]
        if entity["name"] == "CLOCK_OBSERVATION_V1"
    )
    references = observation["foreign_keys"][0]["references"]
    references["table"] = parent_table
    references["columns"] = [parent_column]

    result = validate_entity_manifest_fixture(tmp_path, manifest)
    assert result.returncode == 2
    assert message in result.stderr


# Requires lifecycle parents for all generated receiver-scoped rows.
def test_receiver_scoped_generated_rows_require_lifecycle_parent() -> None:
    receiver_instance_id = bytes.fromhex("00112233445566778899aabbccddeeff")
    profile = generated_entities.MessageProfilingV1(
        receiver_instance_id=receiver_instance_id,
        occurrence_sequence=1,
        received_at_monotonic_us=1,
        persist_queue_used_bytes_before_admission=0,
        persist_queue_capacity_bytes=490,
        received_frame_length=None,
        received_frame=None,
        claimed_control=None,
        claimed_domain=None,
        claimed_node_id=None,
        claimed_message_id=None,
        header_authenticated=False,
        decoded_sample_id=None,
        rssi_dbm_x2=None,
        snr_db_x4=None,
        irq_status=None,
        device_errors=None,
        processing_result=generated.ProcessingResult.RADIO_ERROR,
        ack_selected=generated.AckSelection.NONE,
        ack_tx_result=generated.AckTxResult.NOT_APPLICABLE,
        ack_frame=None,
        busy_wait_total_us=0,
        busy_wait_max_us=0,
        busy_wait_count=0,
        busy_timeout_count=0,
        last_busy_timeout_opcode=None,
        t1_handler_started_monotonic_us=1,
        t2_packet_copied_monotonic_us=None,
        t3_authentication_completed_monotonic_us=None,
        t4_set_tx_attempted_monotonic_us=None,
        t5_tx_done_monotonic_us=None,
        t6_set_rx_issued_monotonic_us=None,
    )
    profile_row = generated_entities.MessageProfileRowV1(
        profile=profile,
        persistence_classification=generated.PersistenceClassification.NOT_APPLICABLE,
    )
    health_values = {
        field.name: 0 for field in fields(generated_entities.ReceiverHealthV1)
    }
    health_values.update(
        receiver_instance_id=receiver_instance_id,
        radio_state=generated.RadioState.INITIALIZING,
        radio_recovery_attempts_by_reason=(0,) * 8,
        system_time_quality=generated.SystemTimeQuality.UNTRUSTED,
        rtc_health=generated.RtcHealth.PRESENT,
        chrony_step_command_results=(0,) * 3,
        rtc_write_results=(0,) * 3,
        persist_queue_admission_counts=((0,) * 3,) * 5,
        persistence_admission_state=(
            generated.PersistenceAdmissionState.UNAVAILABLE_STARTING
        ),
        persistence_admission_transition_counts=(0,) * 7,
    )
    rows = (
        (
            generated_entities.CLOCK_OBSERVATION_V1_TABLE,
            generated_entities.CLOCK_OBSERVATION_V1_COLUMNS,
            generated_entities.clock_observation_v1_parameters(
                generated_entities.ClockObservationV1(
                    receiver_instance_id=receiver_instance_id,
                    observation_sequence=1,
                    clock_state_generation=0,
                    sampled_at_monotonic_us=1,
                    sampled_at_utc_us=None,
                    step_discontinuity_boundary=False,
                    system_time_quality=generated.SystemTimeQuality.UNTRUSTED,
                    rtc_health=generated.RtcHealth.PRESENT,
                )
            ),
        ),
        (
            generated_entities.DIAGNOSTIC_V1_TABLE,
            generated_entities.DIAGNOSTIC_V1_COLUMNS,
            generated_entities.diagnostic_v1_parameters(
                generated_entities.DiagnosticV1(
                    receiver_instance_id=receiver_instance_id,
                    diagnostic_sequence=1,
                    sampled_at_monotonic_us=1,
                    severity=generated.DiagnosticSeverity.ERROR,
                    error_domain=generated.DiagnosticErrorDomain.RADIO,
                    operation=generated.DiagnosticOperation.VALIDATE,
                    error_code=1,
                    context_schema=0,
                    context_length=0,
                    context=bytes(128),
                )
            ),
        ),
        (
            generated_entities.QUARANTINED_ENTITY_ROW_V1_TABLE,
            generated_entities.QUARANTINED_ENTITY_ROW_V1_COLUMNS,
            generated_entities.quarantined_entity_row_v1_parameters(
                generated_entities.QuarantinedEntityRowV1(
                    quarantine_id=bytes(32),
                    entity_kind=generated.PersistQueueEntityKind.PROFILE_ONLY,
                    entity_schema_version=1,
                    entity_length=2,
                    entity_bytes=b"\x02\x01",
                    receiver_instance_id=receiver_instance_id,
                    quarantined_at_monotonic_us=1,
                    database_schema_version=generated.DATABASE_SCHEMA_VERSION,
                    failure_reason=(
                        generated.QuarantineFailureReason.ENTITY_DECODING_INVARIANT
                    ),
                    failure_operation=generated.DiagnosticOperation.VALIDATE,
                    sqlite_primary_code=None,
                    sqlite_extended_code=None,
                    os_errno=None,
                    isolation_attempt_count=1,
                )
            ),
        ),
        (
            generated_entities.RECEIVER_HEALTH_V1_TABLE,
            generated_entities.RECEIVER_HEALTH_V1_COLUMNS,
            generated_entities.receiver_health_v1_parameters(
                generated_entities.ReceiverHealthV1(**health_values)
            ),
        ),
        (
            generated_entities.MESSAGE_PROFILE_ROW_V1_TABLE,
            generated_entities.MESSAGE_PROFILE_ROW_V1_COLUMNS,
            generated_entities.message_profile_row_v1_parameters(profile_row),
        ),
    )
    connection = open_schema()
    for table, columns, parameters in rows:
        insert = (
            f"INSERT INTO {table} ({', '.join(columns)}) VALUES "
            f"({', '.join('?' for _ in columns)})"
        )
        with pytest.raises(sqlite3.IntegrityError, match="FOREIGN KEY"):
            connection.execute(insert, parameters)

    connection.execute(
        "INSERT INTO receiver_instances "
        "(receiver_instance_id, linux_boot_id, started_at_monotonic_us) "
        "VALUES (?, ?, ?)",
        (receiver_instance_id, bytes(16), 0),
    )
    for table, columns, parameters in rows:
        connection.execute(
            f"INSERT INTO {table} ({', '.join(columns)}) VALUES "
            f"({', '.join('?' for _ in columns)})",
            parameters,
        )

    reading = generated_entities.ReadingMessageRowV1(
        node_id=bytes(8),
        message_id=1,
        sample_id=1,
        reading_body=bytes(32),
        is_canonical_for_sample=True,
        run_ms=0,
        soil_0_mv=0,
        soil_1_mv=0,
        soil_temp_0_centi_c=0,
        soil_temp_1_centi_c=0,
        enclosure_centi_c=0,
        enclosure_pressure_pa=0,
        enclosure_humidity_centi_pct=0,
        reset_reason=0,
        previous_current_tx_attempts=0,
        previous_awake_ms=0,
        previous_current_delivery_ms=0,
        previous_cycle_tx_attempts=0,
        previous_cycle_accepted_readings=0,
        flags=0,
        first_receiver_instance_id=receiver_instance_id,
        first_occurrence_sequence=1,
    )
    reading_columns = generated_entities.READING_MESSAGE_ROW_V1_COLUMNS
    connection.execute(
        f"INSERT INTO reading_messages ({', '.join(reading_columns)}) VALUES "
        f"({', '.join('?' for _ in reading_columns)})",
        generated_entities.reading_message_row_v1_parameters(reading),
    )
    assert connection.execute("PRAGMA foreign_key_check").fetchall() == []


# Expands receiver-health arrays only when every generated shape is exact.
def test_receiver_health_binding_expands_only_exact_array_shapes() -> None:
    values = {
        field.name: 0 for field in fields(generated_entities.ReceiverHealthV1)
    }
    values.update(
        receiver_instance_id=bytes(16),
        radio_state=generated.RadioState.INITIALIZING,
        radio_recovery_attempts_by_reason=(0,) * 8,
        system_time_quality=generated.SystemTimeQuality.UNTRUSTED,
        rtc_health=generated.RtcHealth.PRESENT,
        chrony_step_command_results=(0,) * 3,
        rtc_write_results=(0,) * 3,
        persist_queue_admission_counts=((0,) * 3,) * 5,
        persistence_admission_state=(
            generated.PersistenceAdmissionState.UNAVAILABLE_STARTING
        ),
        persistence_admission_transition_counts=(0,) * 7,
    )
    health = generated_entities.ReceiverHealthV1(**values)
    parameters = generated_entities.receiver_health_v1_parameters(health)
    assert len(parameters) == 74
    assert len(generated_entities.RECEIVER_HEALTH_V1_COLUMNS) == 74

    wrong_shape = replace(health, chrony_step_command_results=(0,) * 2)
    with pytest.raises(ValueError, match="chrony_step_command_results"):
        generated_entities.receiver_health_v1_parameters(wrong_shape)


# Round-trips and binds the exact canonical communicator-state blob.
def test_communicator_state_canonical_blob_round_trip_and_binding() -> None:
    provenance = generated_entities.RtcProvenanceV1(
        verified_by_receiver_instance_id=bytes(range(16)),
        network_utc_at_verification_us=1_000_000,
        rtc_readback_utc_us=999_000,
        verification_uncertainty_us=2_000,
        drift_bound_ppm=20,
    )
    state = communicator_state(rtc_provenance=provenance)
    blob = generated_entities.encode_communicator_state_v1(state)
    assert len(blob) == 1120
    assert generated_entities.decode_communicator_state_v1(blob) == state

    parameters = generated_entities.communicator_state_v1_parameters(state)
    assert parameters[:3] == (1, 1, 1)
    assert parameters[3] == blob
    assert parameters[4] == hashlib.sha256(blob).digest()

    connection = open_schema()
    columns = generated_entities.COMMUNICATOR_STATE_V1_COLUMNS
    connection.execute(
        f"INSERT INTO communicator_state ({', '.join(columns)}) VALUES "
        f"({', '.join('?' for _ in columns)})",
        parameters,
    )
    assert connection.execute(
        "SELECT generation, state_blob, state_sha256 FROM communicator_state"
    ).fetchone() == (1, blob, hashlib.sha256(blob).digest())


# Rejects noncanonical communicator-state structure at the binary boundary.
def test_communicator_state_codec_enforces_only_canonical_structure() -> None:
    state = communicator_state()
    blob = generated_entities.encode_communicator_state_v1(state)

    with pytest.raises(ValueError, match="buckets.*length 62"):
        generated_entities.encode_communicator_state_v1(
            replace(state, buckets=state.buckets[:-1])
        )

    wrong_bucket_count = bytearray(blob)
    struct.pack_into("<H", wrong_bucket_count, 116, 61)
    with pytest.raises(ValueError, match="invalid fixed length"):
        generated_entities.decode_communicator_state_v1(bytes(wrong_bucket_count))

    noncanonical_absence = bytearray(blob)
    noncanonical_absence[20] = 1
    with pytest.raises(ValueError, match="absent representation is not zero"):
        generated_entities.decode_communicator_state_v1(bytes(noncanonical_absence))

    reserved_presence_bit = bytearray(blob)
    struct.pack_into("<H", reserved_presence_bit, 14, 2)
    with pytest.raises(ValueError, match="reserved bits set"):
        generated_entities.decode_communicator_state_v1(bytes(reserved_presence_bit))


# Keeps representation-only validity masks out of relational tables.
def test_relational_tables_do_not_persist_validity_masks() -> None:
    connection = open_schema()
    for table in ("clock_observations", "receiver_health", "message_profiles"):
        columns = {row[1] for row in connection.execute(f"PRAGMA table_info({table})")}
        assert "validity_mask" not in columns


# Allows catalogue identifiers to repeat only within separate domains.
def test_scoped_catalogues_reuse_ids_only_inside_their_domain() -> None:
    connection = open_schema()
    assert connection.execute(
        "SELECT code FROM diagnostic_error_codes "
        "WHERE error_domain_id = 1 AND id = 1"
    ).fetchone() == ("INVALID_ARGUMENT",)
    assert connection.execute(
        "SELECT code FROM diagnostic_error_codes "
        "WHERE error_domain_id = 2 AND id = 1"
    ).fetchone() == ("IO",)
    assert connection.execute(
        "SELECT count(*) FROM diagnostic_context_schemas"
    ).fetchone() == (8,)


# Allows exactly one canonical reading row for each node and sample.
def test_reading_messages_allow_one_canonical_row_per_sample() -> None:
    connection = open_schema()
    connection.execute("PRAGMA foreign_keys = OFF")
    assert connection.execute("PRAGMA foreign_keys").fetchone() == (0,)

    canonical = generated_entities.ReadingMessageRowV1(
        node_id=bytes.fromhex("0102030405060708"),
        message_id=10,
        sample_id=50,
        reading_body=bytes(32),
        is_canonical_for_sample=True,
        run_ms=0,
        soil_0_mv=0,
        soil_1_mv=0,
        soil_temp_0_centi_c=0,
        soil_temp_1_centi_c=0,
        enclosure_centi_c=0,
        enclosure_pressure_pa=0,
        enclosure_humidity_centi_pct=0,
        reset_reason=0,
        previous_current_tx_attempts=0,
        previous_awake_ms=0,
        previous_current_delivery_ms=0,
        previous_cycle_tx_attempts=0,
        previous_cycle_accepted_readings=0,
        flags=0,
        first_receiver_instance_id=bytes(16),
        first_occurrence_sequence=1,
    )
    columns = generated_entities.READING_MESSAGE_ROW_V1_COLUMNS
    insert = (
        f"INSERT INTO reading_messages ({', '.join(columns)}) VALUES "
        f"({', '.join('?' for _ in columns)})"
    )
    connection.execute(
        insert,
        generated_entities.reading_message_row_v1_parameters(canonical),
    )

    noncanonical = replace(
        canonical,
        message_id=11,
        is_canonical_for_sample=False,
        first_occurrence_sequence=2,
    )
    connection.execute(
        insert,
        generated_entities.reading_message_row_v1_parameters(noncanonical),
    )

    second_canonical = replace(
        canonical,
        message_id=12,
        first_occurrence_sequence=3,
    )
    with pytest.raises(sqlite3.IntegrityError):
        connection.execute(
            insert.replace("INSERT", "INSERT OR REPLACE", 1),
            generated_entities.reading_message_row_v1_parameters(second_canonical),
        )

    assert connection.execute(
        "SELECT message_id, is_canonical_for_sample FROM reading_messages "
        "ORDER BY message_id"
    ).fetchall() == [(10, 1), (11, 0)]


# Rejects mutation of installed metadata and generated catalogues.
def test_catalogues_and_installed_metadata_are_immutable() -> None:
    connection = open_schema()
    connection.execute(
        "INSERT INTO database_metadata "
        "(singleton_id, group_id, database_schema_version, "
        "database_schema_fingerprint) VALUES (?, ?, ?, ?)",
        (
            1,
            bytes.fromhex("0102030405060708"),
            generated.DATABASE_SCHEMA_VERSION,
            generated.DATABASE_SCHEMA_FINGERPRINT,
        ),
    )

    statements = (
        "INSERT INTO rtc_health_codes (id, code) VALUES (4, 'OTHER')",
        "UPDATE rtc_health_codes SET code = 'OTHER' WHERE id = 1",
        "DELETE FROM rtc_health_codes WHERE id = 1",
        "INSERT OR REPLACE INTO database_metadata VALUES "
        "(1, X'0000000000000000', 2, zeroblob(32))",
        "UPDATE database_metadata SET database_schema_version = 2",
        "DELETE FROM database_metadata",
    )
    for statement in statements:
        with pytest.raises(sqlite3.IntegrityError):
            connection.execute(statement)

    assert connection.execute(
        "SELECT group_id, database_schema_version, database_schema_fingerprint "
        "FROM database_metadata WHERE singleton_id = 1"
    ).fetchone() == (
        bytes.fromhex("0102030405060708"),
        generated.DATABASE_SCHEMA_VERSION,
        generated.DATABASE_SCHEMA_FINGERPRINT,
    )

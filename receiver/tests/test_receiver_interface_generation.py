from __future__ import annotations

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


REPO_ROOT = Path(__file__).resolve().parents[2]
RECEIVER_ROOT = REPO_ROOT / "receiver"
GENERATOR = RECEIVER_ROOT / "tools" / "generate.py"
MANIFEST = RECEIVER_ROOT / "schemas" / "receiver_enums.json"
ENTITY_MANIFEST = RECEIVER_ROOT / "schemas" / "receiver_entities.json"
SCHEMA = RECEIVER_ROOT / "db" / "schema.sql"

sys.path.insert(0, str(REPO_ROOT))
from receiver.cura_receiver.generated import (  # noqa: E402
    receiver_entities_generated as generated_entities,
)
from receiver.cura_receiver.generated import (  # noqa: E402
    receiver_interface_generated as generated,
)


def pascal_case(name: str) -> str:
    return "".join(part.lower().capitalize() for part in name.split("_"))


def load_manifest() -> dict[str, object]:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


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


def test_generated_outputs_are_current() -> None:
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--check"],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr


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


def test_schema_fingerprint_is_exact_schema_sql_sha256() -> None:
    schema_bytes = SCHEMA.read_bytes()
    assert hashlib.sha256(schema_bytes).hexdigest() == generated.DATABASE_SCHEMA_SHA256
    assert hashlib.sha256(schema_bytes).digest() == generated.DATABASE_SCHEMA_FINGERPRINT
    assert generated.SQLITE_APPLICATION_ID == 0x43555252


def test_schema_contains_declared_catalogues_and_entity_tables() -> None:
    manifest = load_manifest()
    entity_manifest = json.loads(ENTITY_MANIFEST.read_text(encoding="utf-8"))
    expected_catalogues = {
        enum_spec["persistence"]["table"]
        for enum_spec in manifest["enums"]  # type: ignore[index]
        if enum_spec["persistence"]["mode"] != "encoded_only"
    }
    expected_entity_tables: set[str] = set()
    for entity in entity_manifest["entities"]:
        persistence = entity["persistence"]
        if persistence["mode"] == "multi_table_transaction":
            expected_entity_tables.update(
                target["table"] for target in persistence["targets"]
            )
        else:
            expected_entity_tables.add(persistence["table"])
    connection = open_schema()
    actual_tables = {
        row[0]
        for row in connection.execute(
            "SELECT name FROM sqlite_schema "
            "WHERE type = 'table' AND name NOT LIKE 'sqlite_%'"
        )
    }
    assert actual_tables == (
        expected_catalogues | expected_entity_tables | {"database_metadata"}
    )

    strict_by_table = {
        row[1]: row[5]
        for row in connection.execute("PRAGMA table_list")
        if row[1] in actual_tables
    }
    assert strict_by_table == {table: 1 for table in actual_tables}


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


def test_relational_tables_do_not_persist_validity_masks() -> None:
    connection = open_schema()
    for table in ("clock_observations", "receiver_health", "message_profiles"):
        columns = {row[1] for row in connection.execute(f"PRAGMA table_info({table})")}
        assert "validity_mask" not in columns


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
            insert,
            generated_entities.reading_message_row_v1_parameters(second_canonical),
        )

    assert connection.execute(
        "SELECT message_id, is_canonical_for_sample FROM reading_messages "
        "ORDER BY message_id"
    ).fetchall() == [(10, 1), (11, 0)]


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

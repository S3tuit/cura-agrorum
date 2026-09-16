import hashlib
import struct
from dataclasses import FrozenInstanceError, replace

import pytest

from cura_receiver.communicator_state_persistence import (
    CommunicatorStatePolicy,
    classify_communicator_state_rows,
    validate_communicator_state,
)
from cura_receiver.generated.receiver_entities_generated import (
    RtcProvenanceV1,
    TxAirtimeBucketV1,
    communicator_state_v1_parameters,
    encode_communicator_state_v1,
)
from cura_receiver.generated.receiver_enums_generated import RtcHealth
from cura_receiver.persistence_control_values import (
    CommunicatorStateCondition as Condition,
)
from cura_receiver.persistence_control_values import (
    CommunicatorStateLoadStatus as Status,
)
from cura_receiver.sqlite_repository import SqliteRepository
from tests.support.builders.persistence import INSTANCE


from tests.support.builders.persistence_control import state


def classify(connection, raw_rows):
    connection.execute("DELETE FROM communicator_state")
    connection.executemany(
        "INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)", raw_rows
    )
    repository = SqliteRepository(connection)
    return classify_communicator_state_rows(
        repository.read_communicator_state_rows(), repository, CommunicatorStatePolicy()
    )


# Normal state survives the generated encoding while classification returns the exact immutable value.
def test_valid_state_and_missing_state(setup):
    _, connection, *_ = setup
    value = state()
    result = classify(connection, (communicator_state_v1_parameters(value),))
    assert result.status is Status.LOADED and result.state == value
    with pytest.raises(FrozenInstanceError):
        result.state.generation = 2
    assert classify(connection, ()).state_condition is Condition.MISSING


# Wrong raw SQL classes, lengths, identities and ranges are application corruption.
@pytest.mark.parametrize(
    "column,value",
    [
        (0, None),
        (0, "1"),
        (0, 1.0),
        (0, 2),
        (1, None),
        (1, "1"),
        (1, -1),
        (1, 65536),
        (2, 0),
        (2, -1),
        (2, 1.0),
        (2, "1"),
        (3, None),
        (3, "blob"),
        (3, 42),
        (4, None),
        (4, "digest"),
        (4, b"x"),
        (4, bytes(32)),
    ],
)
def test_raw_envelope_corruption(setup, column, value):
    _, connection, *_ = setup
    row = list(communicator_state_v1_parameters(state()))
    row[column] = value
    assert classify(connection, (tuple(row),)).state_condition is Condition.CORRUPT
    assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]


# The classifier inspects the entire raw relation, including duplicate singleton identities.
def test_multiple_rows_are_corrupt(setup):
    _, connection, *_ = setup
    row = communicator_state_v1_parameters(state())
    assert classify(connection, (row, row)).state_condition is Condition.CORRUPT


# Digest corruption wins over unknown version; valid unknown versions are never decoded as V1.
@pytest.mark.parametrize(
    "version,blob,digest,expected",
    [
        (
            2,
            b"\x02\x00",
            hashlib.sha256(b"\x02\x00").digest(),
            Condition.UNSUPPORTED_VERSION,
        ),
        (2, b"\x02\x00", bytes(32), Condition.CORRUPT),
        (2, b"\x01\x00", hashlib.sha256(b"\x01\x00").digest(), Condition.CORRUPT),
        (2, b"\x02", hashlib.sha256(b"\x02").digest(), Condition.CORRUPT),
    ],
)
def test_version_precedence(setup, version, blob, digest, expected):
    _, connection, *_ = setup
    assert (
        classify(connection, ((1, version, 1, blob, digest),)).state_condition
        is expected
    )


# Supported malformed binary structure precedes a simultaneous policy mismatch.
@pytest.mark.parametrize(
    "offset,fmt,value",
    [
        (2, "<I", 100),
        (6, "<Q", 2),
        (14, "<H", 2),
        (16, "<B", 255),
        (18, "<H", 1),
        (116, "<H", 61),
        (118, "<H", 1),
        (120, "<Q", 1),
    ],
)
def test_supported_structure_precedes_policy(setup, offset, fmt, value):
    _, connection, *_ = setup
    blob = bytearray(
        encode_communicator_state_v1(state(tx_airtime_budget_us=35_000_000))
    )
    struct.pack_into(fmt, blob, offset, value)
    blob = bytes(blob)
    assert (
        classify(
            connection, ((1, 1, 1, blob, hashlib.sha256(blob).digest()),)
        ).state_condition
        is Condition.CORRUPT
    )


# A structurally valid alternate deployment policy has only POLICY_MISMATCH as its condition.
def test_pure_policy_mismatch(setup):
    _, connection, *_ = setup
    result = classify(
        connection,
        (communicator_state_v1_parameters(state(tx_airtime_budget_us=35_000_000)),),
    )
    assert result.state_condition is Condition.POLICY_MISMATCH and result.state is None


# Ledger ordering, canonical trailing empties, expiration, grid and global charge are independent checks.
@pytest.mark.parametrize(
    "buckets",
    [
        (TxAirtimeBucketV1(0, 1),),
        (TxAirtimeBucketV1(1, 0),),
        (TxAirtimeBucketV1(8_000_001, 3_661_000_000),),
        (TxAirtimeBucketV1(0, 0), TxAirtimeBucketV1(1, 3_661_000_000)),
        (TxAirtimeBucketV1(1, 3_661_000_000), TxAirtimeBucketV1(1, 3_661_000_000)),
        (TxAirtimeBucketV1(1, 3_661_000_000), TxAirtimeBucketV1(1, 3_661_000_001)),
        tuple(
            TxAirtimeBucketV1(8_000_000, 3_421_000_000 + n * 60_000_000)
            for n in range(5)
        ),
    ],
)
def test_ledger_semantics(setup, buckets):
    _, connection, *_ = setup
    value = state(buckets=buckets + (TxAirtimeBucketV1(0, 0),) * (64 - len(buckets)))
    assert (
        classify(connection, (communicator_state_v1_parameters(value),)).state_condition
        is Condition.CORRUPT
    )


# Policy sizing must accommodate both the unexpired span and synthetic worst-case ledger.
@pytest.mark.parametrize(
    "changes",
    [
        {"bucket_width_us": 0},
        {"bucket_charge_limit_us": 0},
        {"tx_airtime_budget_us": 1},
        {"rolling_window_us": 3_660_000_001},
        {"bucket_charge_limit_us": 1},
        {"rolling_window_us": 1 << 64},
        {"bucket_expiration_guard_us": True},
        {"bucket_expiration_guard_us": 79_999_999},
        {"receiver_utc_error_budget_us": 40_000_001},
        {"rolling_window_us": 1, "bucket_charge_limit_us": 1_000_000},
    ],
)
def test_invalid_deployment_policy(changes):
    with pytest.raises((TypeError, ValueError)):
        CommunicatorStatePolicy(**changes)


# RTC provenance retains persisted bounds and must resolve a real durable lifecycle row.
@pytest.mark.parametrize(
    "changes,expected",
    [
        ({}, Condition.NONE),
        ({"verified_by_receiver_instance_id": bytes(16)}, Condition.CORRUPT),
        ({"drift_bound_ppm": 0}, Condition.CORRUPT),
        ({"drift_bound_ppm": 1_000_000}, Condition.CORRUPT),
        ({"verification_uncertainty_us": 40_000_000}, Condition.CORRUPT),
    ],
)
def test_provenance_semantics(setup, changes, expected):
    _, connection, *_ = setup
    provenance = replace(
        RtcProvenanceV1(INSTANCE, 500_000, 500_000, 100, 20), **changes
    )
    result = classify(
        connection,
        (communicator_state_v1_parameters(state(rtc_provenance=provenance)),),
    )
    assert result.state_condition is expected


# Canonical projection never erases wrong enum classes or mutable request content before validation.
@pytest.mark.parametrize(
    "changes",
    [
        {"generation": True},
        {"generation": 1 << 63},
        {"last_observed_system_time_quality": RtcHealth.PRESENT},
        {"last_observed_system_time_quality": 1},
        {"buckets": []},
        {"buckets": (TxAirtimeBucketV1(True, 0),) * 64},
    ],
)
def test_request_types_rejected_before_encoding(setup, changes):
    _, connection, *_ = setup
    with pytest.raises((TypeError, ValueError)):
        validate_communicator_state(
            state(**changes), SqliteRepository(connection), CommunicatorStatePolicy()
        )


# The approved 64-slot layout fits this exact span; one more microsecond needs slot 65.
def test_exact_policy_capacity_and_fixed_historical_error_bound():
    assert CommunicatorStatePolicy(rolling_window_us=3_660_000_000)
    assert CommunicatorStatePolicy(receiver_utc_error_budget_us=1)
    assert CommunicatorStatePolicy(bucket_expiration_guard_us=80_000_000)


# Sparse states must fit the same physical ring as dense ones, even before policy comparison.
@pytest.mark.parametrize(
    "span,expected", [(63, Condition.NONE), (64, Condition.CORRUPT)]
)
@pytest.mark.parametrize("mismatch", [False, True])
def test_sparse_ledger_span(setup, span, expected, mismatch):
    _, connection, *_ = setup
    value = state(
        tx_airtime_budget_us=35_000_000 if mismatch else 36_000_000,
        buckets=(
            TxAirtimeBucketV1(1, 120_000_000),
            TxAirtimeBucketV1(1, 120_000_000 + span * 60_000_000),
        )
        + (TxAirtimeBucketV1(0, 0),) * 62,
    )
    if expected is Condition.NONE and mismatch:
        expected = Condition.POLICY_MISMATCH
    assert (
        classify(connection, (communicator_state_v1_parameters(value),)).state_condition
        is expected
    )


# A digest-valid supported state cannot hide arithmetic overflow behind differing policy values.
@pytest.mark.parametrize(
    "changes",
    [
        {
            "airtime_snapshot_utc_us": -(1 << 63),
            "buckets": (TxAirtimeBucketV1(1, -(1 << 63) + 1),)
            + (TxAirtimeBucketV1(0, 0),) * 63,
        },
        {
            "tx_airtime_budget_us": (1 << 64) - 1,
            "bucket_charge_limit_us": 1 << 63,
            "buckets": (
                TxAirtimeBucketV1(1 << 63, 3_780_000_000),
                TxAirtimeBucketV1(1 << 63, 3_840_000_000),
            )
            + (TxAirtimeBucketV1(0, 0),) * 62,
        },
        {"rolling_window_us": (1 << 64) - 1},
        {"bucket_width_us": (1 << 64) - 1},
        {"rolling_window_us": 1, "bucket_charge_limit_us": 1_000_000},
    ],
)
def test_supported_arithmetic_precedes_policy(setup, changes):
    _, connection, *_ = setup
    value = state(**changes)
    assert (
        classify(connection, (communicator_state_v1_parameters(value),)).state_condition
        is Condition.CORRUPT
    )


# The old 1120-byte pilot layout is not silently decoded using the revised V1 shape.
def test_old_pilot_layout_is_rejected_in_new_schema_epoch(setup):
    _, connection, *_ = setup
    blob = bytearray(encode_communicator_state_v1(state())[:-32])
    struct.pack_into("<I", blob, 2, 1120)
    struct.pack_into("<H", blob, 116, 62)
    blob = bytes(blob)
    assert (
        classify(
            connection, ((1, 1, 1, blob, hashlib.sha256(blob).digest()),)
        ).state_condition
        is Condition.CORRUPT
    )

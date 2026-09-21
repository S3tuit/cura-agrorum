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
    AirtimeSnapshotV1,
    TxAirtimeBucketV1,
    communicator_state_v1_parameters,
    encode_communicator_state_v1,
)
from cura_receiver.generated.receiver_enums_generated import RtcHealth, SystemTimeQuality as Q
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
        (14, "<H", 4),
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


# Positional slots preserve zeros and enforce per-bucket and total charge limits.
@pytest.mark.parametrize("charges, expected", [
    ((0, 1, 0, 2, 0), Condition.NONE),
    ((8_000_001,), Condition.CORRUPT),
    ((8_000_000,) * 5, Condition.CORRUPT),
])
def test_ledger_semantics(setup, charges, expected):
    _, connection, *_ = setup
    value = state(buckets=(TxAirtimeBucketV1(0),) * (62 - len(charges)) +
                  tuple(TxAirtimeBucketV1(n) for n in charges))
    assert classify(connection, (communicator_state_v1_parameters(value),)).state_condition is expected


@pytest.mark.parametrize("quality, snapshot, expected", [
    (Q.UNTRUSTED, None, Condition.NONE),
    (Q.NETWORK_SYNCED, AirtimeSnapshotV1(0, 0), Condition.NONE),
    (Q.RTC_HOLDOVER, AirtimeSnapshotV1(-1, 39_999_999), Condition.NONE),
    (Q.NETWORK_SYNCED, None, Condition.CORRUPT),
    (Q.UNTRUSTED, AirtimeSnapshotV1(0, 0), Condition.CORRUPT),
    (Q.RTC_HOLDOVER, AirtimeSnapshotV1(0, 40_000_000), Condition.CORRUPT),
])
def test_snapshot_time_semantics(setup, quality, snapshot, expected):
    _, connection, *_ = setup
    value = state(last_observed_system_time_quality=quality, airtime_snapshot=snapshot)
    assert classify(connection, (communicator_state_v1_parameters(value),)).state_condition is expected


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
        {"buckets": (TxAirtimeBucketV1(True),) * 62},
    ],
)
def test_request_types_rejected_before_encoding(setup, changes):
    _, connection, *_ = setup
    with pytest.raises((TypeError, ValueError)):
        validate_communicator_state(
            state(**changes), SqliteRepository(connection), CommunicatorStatePolicy()
        )


# The normal TX tail makes 62 slots necessary; one extra microsecond exceeds capacity.
def test_exact_policy_capacity_and_fixed_historical_error_bound():
    assert CommunicatorStatePolicy(rolling_window_us=3_659_750_000)
    with pytest.raises(ValueError):
        CommunicatorStatePolicy(rolling_window_us=3_659_750_001)
    assert CommunicatorStatePolicy(receiver_utc_error_budget_us=1)


# A digest-valid supported state cannot hide arithmetic overflow behind differing policy values.
@pytest.mark.parametrize(
    "changes",
    [
        {
            "tx_airtime_budget_us": (1 << 64) - 1,
            "bucket_charge_limit_us": 1 << 63,
            "buckets": (
                TxAirtimeBucketV1(1 << 63),
                TxAirtimeBucketV1(1 << 63),
            )
            + (TxAirtimeBucketV1(0),) * 60,
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


# Truncated current-format content remains corruption even with an updated digest.
def test_truncated_current_layout_is_corrupt(setup):
    _, connection, *_ = setup
    blob = encode_communicator_state_v1(state())[:-8]
    assert classify(connection, ((1, 1, 1, blob, hashlib.sha256(blob).digest()),)).state_condition is Condition.CORRUPT

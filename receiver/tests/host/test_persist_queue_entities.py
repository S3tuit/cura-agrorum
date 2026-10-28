from __future__ import annotations
from tests.support.builders.persistence import REVIEWED_RECEPTION_FRAME

from dataclasses import FrozenInstanceError, fields

import pytest

from cura_receiver.generated.receiver_entities_generated import PacketReceptionV1
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    PersistQueueEntityKind,
    ProcessingResult,
    RadioState,
    RtcHealth,
    SystemTimeQuality,
)
from cura_receiver.persist_queue_entities import (
    CLOCK_OBSERVATION_V1_SPEC,
    RADIO_OBSERVATION_V1_SPEC,
    DIAGNOSTIC_V1_SPEC,
    READING_RECEPTION_V2_SPEC,
    PERSIST_QUEUE_ENTITY_SCHEMA_VERSION,
    RECEPTION_ONLY_V2_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    SUPPORTED_PERSIST_QUEUE_ENTITY_SPECS,
    AuthenticatedReadingCandidateV1,
    ReadingReceptionUnitV2,
    PersistQueueEntitySpec,
    ReceptionOnlyUnitV2,
    ReceiverHealthRequestV1,
)


def _profile() -> PacketReceptionV1:
    return PacketReceptionV1(
        receiver_instance_id=b"r" * 16,
        occurrence_sequence=1,
        received_at_monotonic_us=2,
        domain=1,
        node_id=b"n" * 8,
        message_id=5,
        decoded_sample_id=6,
        rssi_dbm_x2=-140,
        snr_db_x4=12,
        irq_status=7,
        device_errors=0,
        processing_result=ProcessingResult.ACCEPTED,
        ack_tx_result=AckTxResult.TX_DONE,
        busy_wait_total_us=8,
        busy_wait_max_us=9,
        busy_wait_count=10,
        busy_timeout_count=0,
        last_busy_timeout_opcode=None,
        t1_handler_started_monotonic_us=11,
        t2_packet_copied_monotonic_us=12,
        t3_authentication_completed_monotonic_us=13,
        t4_set_tx_attempted_monotonic_us=14,
        t5_tx_done_monotonic_us=15,
        t6_set_rx_issued_monotonic_us=16,
    )


def _health_request() -> ReceiverHealthRequestV1:
    from tests.support.builders.persistence import _health_request as build
    return build()



# The closed V1 descriptor set carries identity metadata but no size policy.
def test_supported_specs_are_closed_v1_descriptors_without_sizes() -> None:
    expected = {
        READING_RECEPTION_V2_SPEC,
        RECEPTION_ONLY_V2_SPEC,
        RECEIVER_HEALTH_REQUEST_V1_SPEC,
        DIAGNOSTIC_V1_SPEC,
        CLOCK_OBSERVATION_V1_SPEC,
    RADIO_OBSERVATION_V1_SPEC,
    }

    assert SUPPORTED_PERSIST_QUEUE_ENTITY_SPECS == expected
    assert {spec.kind for spec in expected} == set(PersistQueueEntityKind)
    assert {
        spec.schema_version for spec in expected
    } == {PERSIST_QUEUE_ENTITY_SCHEMA_VERSION}
    assert [field.name for field in fields(PersistQueueEntitySpec)] == [
        "kind",
        "schema_version",
    ]
    assert not {
        "encoded_size",
        "size_bytes",
        "charge_bytes",
    }.intersection(dir(PersistQueueEntitySpec))
    assert not hasattr(RECEPTION_ONLY_V2_SPEC, "__dict__")
    with pytest.raises(FrozenInstanceError):
        RECEPTION_ONLY_V2_SPEC.schema_version = 2  # type: ignore[misc]


# Composed queue units retain exact leaf references and expose no mutable instance state.
def test_measurement_and_profile_only_units_preserve_leaf_identity() -> None:
    profile = _profile()
    candidate = AuthenticatedReadingCandidateV1(
        node_id=b"n" * 8,
        message_id=1,
        domain=1,
        sample_id=2,
        reading_body=b"b" * 32,
    )

    measurement = ReadingReceptionUnitV2(candidate=candidate, profile=profile, received_frame=REVIEWED_RECEPTION_FRAME)
    profile_only = ReceptionOnlyUnitV2(profile=profile, received_frame=REVIEWED_RECEPTION_FRAME)

    assert measurement.candidate is candidate
    assert measurement.profile is profile
    assert profile_only.profile is profile
    assert not hasattr(measurement, "__dict__")
    assert not hasattr(profile_only, "__dict__")
    with pytest.raises(FrozenInstanceError):
        measurement.profile = _profile()  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        candidate.message_id = 9  # type: ignore[misc]


# Health-request collections use immutable tuples throughout the queue contract.
def test_health_request_uses_only_immutable_container_shapes() -> None:
    request = _health_request()

    assert isinstance(request.radio_recovery_attempts_by_reason, tuple)
    assert all(isinstance(row, tuple) for row in request.persist_queue_admission_counts)
    assert not hasattr(request, "__dict__")
    with pytest.raises(FrozenInstanceError):
        request.health_sequence = 2  # type: ignore[misc]


# Structural construction preserves poison evidence instead of coercing malformed fields.
def test_structural_types_preserve_a_malformed_runtime_value() -> None:
    candidate = AuthenticatedReadingCandidateV1(
        node_id=b"short",
        message_id=1 << 200,
        domain="not-an-integer",  # type: ignore[arg-type]
        sample_id=-1,
        reading_body=b"wrong-length",
    )

    assert candidate.node_id == b"short"
    assert candidate.message_id == 1 << 200
    assert candidate.domain == "not-an-integer"
    assert candidate.sample_id == -1
    assert candidate.reading_body == b"wrong-length"

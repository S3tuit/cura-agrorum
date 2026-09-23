from __future__ import annotations

from dataclasses import FrozenInstanceError, fields

import pytest

from cura_receiver.generated.receiver_entities_generated import MessageProfilingV1
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
    DIAGNOSTIC_V1_SPEC,
    MEASUREMENT_PROFILE_V1_SPEC,
    PERSIST_QUEUE_ENTITY_SCHEMA_VERSION,
    PROFILE_ONLY_V1_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    SUPPORTED_PERSIST_QUEUE_ENTITY_SPECS,
    AuthenticatedReadingCandidateV1,
    MeasurementProfileUnitV1,
    PersistQueueEntitySpec,
    ProfileOnlyUnitV1,
    ReceiverHealthRequestV1,
)


def _profile() -> MessageProfilingV1:
    return MessageProfilingV1(
        receiver_instance_id=b"r" * 16,
        occurrence_sequence=1,
        received_at_monotonic_us=2,
        received_frame_length=3,
        received_frame=b"abc" + bytes(252),
        claimed_control=32,
        claimed_domain=1,
        claimed_node_id=b"n" * 8,
        claimed_message_id=5,
        header_authenticated=True,
        decoded_sample_id=6,
        rssi_dbm_x2=-140,
        snr_db_x4=12,
        irq_status=7,
        device_errors=0,
        processing_result=ProcessingResult.ACCEPTED,
        ack_selected=AckSelection.ACCEPTED,
        ack_tx_result=AckTxResult.TX_DONE,
        ack_frame=b"a" * 23,
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
    return ReceiverHealthRequestV1(
        receiver_instance_id=b"r" * 16,
        health_sequence=1,
        communicator_sampled_at_monotonic_us=2,
        radio_state=RadioState.RX_SINGLE,
        radio_recovery_attempts=3,
        radio_recovery_successes=2,
        radio_recovery_failures=1,
        radio_recovery_attempts_by_reason=(0,) * 8,
        system_time_quality=SystemTimeQuality.NETWORK_SYNCED,
        rtc_health=RtcHealth.PRESENT,
        time_quality_transition_count=4,
        rtc_health_transition_count=5,
        last_time_quality_transition_monotonic_us=6,
        last_rtc_health_transition_monotonic_us=None,
        chrony_step_command_results=(1, 2, 3),
        rtc_write_results=(4, 5, 6),
        rtc_write_readback_verified_count=7,
        rtc_write_trust_invalidated_count=8,
        persist_queue_admission_counts=((0, 0, 0),) * 5,
    )


# The closed V1 descriptor set carries identity metadata but no size policy.
def test_supported_specs_are_closed_v1_descriptors_without_sizes() -> None:
    expected = {
        MEASUREMENT_PROFILE_V1_SPEC,
        PROFILE_ONLY_V1_SPEC,
        RECEIVER_HEALTH_REQUEST_V1_SPEC,
        DIAGNOSTIC_V1_SPEC,
        CLOCK_OBSERVATION_V1_SPEC,
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
    assert not hasattr(PROFILE_ONLY_V1_SPEC, "__dict__")
    with pytest.raises(FrozenInstanceError):
        PROFILE_ONLY_V1_SPEC.schema_version = 2  # type: ignore[misc]


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

    measurement = MeasurementProfileUnitV1(candidate=candidate, profile=profile)
    profile_only = ProfileOnlyUnitV1(profile=profile)

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

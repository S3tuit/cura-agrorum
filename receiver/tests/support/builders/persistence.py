"""Reviewed literal ordinary-persistence inputs; builders calculate no expectations."""

from dataclasses import replace

from cura_receiver.generated import receiver_entities_generated as row
from cura_receiver.generated import receiver_enums_generated as enum

INSTANCE = bytes.fromhex("00112233445546778899aabbccddeeff")
GROUP = b"g" * 8


def _observation(*, sequence=1):
    return row.ClockObservationV1(
        INSTANCE,
        sequence,
        0,
        10,
        None,
        False,
        enum.SystemTimeQuality.UNTRUSTED,
        enum.RtcHealth.PRESENT,
        None, None, None, None, None, None,
    )


def _network_observation(*, sequence=1):
    return row.ClockObservationV1(
        INSTANCE,
        sequence,
        3,
        1_000_000,
        1_800_000_000_000_000,
        False,
        enum.SystemTimeQuality.NETWORK_SYNCED,
        enum.RtcHealth.PRESENT,
        1_025_000,
        999_000,
        1_001_000,
        11_000_000_000,
        row.NetworkClockEvidenceV1(
            900_000, 950_000, -355, 24_404, 9_398, 21_599, 5_867, 233,
            0xB99DE5FE, 1_799_999_990_000_000, 3,
        ),
        None,
    )


def _rtc_observation(*, sequence=1):
    return row.ClockObservationV1(
        INSTANCE,
        sequence,
        4,
        2_000_000,
        1_800_000_000_500_000,
        False,
        enum.SystemTimeQuality.RTC_HOLDOVER,
        enum.RtcHealth.PRESENT,
        5_600_000,
        1_990_000,
        2_010_000,
        9_000_000_000,
        None,
        row.RtcClockEvidenceV1(
            b"v" * 16, 1_799_990_000_000_000, 1_799_990_000_500_000,
            4_000_000, 10, 100_000,
        ),
    )


def _profile(*, sequence=1):
    return row.MessageProfilingV1(
        receiver_instance_id=INSTANCE,
        occurrence_sequence=sequence,
        received_at_monotonic_us=10,
        received_frame_length=0,
        received_frame=bytes(255),
        claimed_control=None,
        claimed_domain=None,
        claimed_node_id=None,
        claimed_message_id=None,
        header_authenticated=False,
        decoded_sample_id=None,
        rssi_dbm_x2=None,
        snr_db_x4=None,
        irq_status=2,
        device_errors=0,
        processing_result=enum.ProcessingResult.REJECTED_MALFORMED_LENGTH,
        ack_selected=enum.AckSelection.NONE,
        ack_tx_result=enum.AckTxResult.NOT_APPLICABLE,
        ack_frame=None,
        busy_wait_total_us=0,
        busy_wait_max_us=0,
        busy_wait_count=0,
        busy_timeout_count=0,
        last_busy_timeout_opcode=None,
        t1_handler_started_monotonic_us=11,
        t2_packet_copied_monotonic_us=12,
        t3_authentication_completed_monotonic_us=None,
        t4_set_tx_attempted_monotonic_us=None,
        t5_tx_done_monotonic_us=None,
        t6_set_rx_issued_monotonic_us=14,
    )


def _health_request(*, sequence=1):
    from cura_receiver.persist_queue_entities import ReceiverHealthRequestV1

    return ReceiverHealthRequestV1(
        receiver_instance_id=INSTANCE,
        health_sequence=sequence,
        communicator_sampled_at_monotonic_us=15,
        radio_state=enum.RadioState.RX_SINGLE,
        radio_recovery_attempts=0,
        radio_recovery_successes=0,
        radio_recovery_failures=0,
        radio_recovery_attempts_by_reason=(0,) * 8,
        system_time_quality=enum.SystemTimeQuality.UNTRUSTED,
        rtc_health=enum.RtcHealth.PRESENT,
        time_quality_transition_count=0,
        rtc_health_transition_count=0,
        last_time_quality_transition_monotonic_us=None,
        last_rtc_health_transition_monotonic_us=None,
        chrony_step_command_results=(0, 0, 0),
        rtc_write_results=(0, 0, 0),
        rtc_write_readback_verified_count=0,
        rtc_write_trust_invalidated_count=0,
        persist_queue_admission_counts=((0, 0, 0),) * 5,
    )


def _diagnostic(*, sequence=1):
    return row.DiagnosticV1(
        INSTANCE,
        sequence,
        10,
        enum.DiagnosticSeverity.FATAL,
        enum.DiagnosticErrorDomain.CORE,
        enum.DiagnosticOperation.VALIDATE,
        1,
        1,
        64,
        bytes.fromhex("00000101") + bytes(124),
    )


def _measurement(
    *,
    sequence=1,
    message=100,
    sample=200,
    domain=1,
    node=b"n" * 8,
    soil=1000,
    instance=INSTANCE
):
    from cura_receiver.persist_queue_entities import (
        AuthenticatedReadingCandidateV1,
        MeasurementProfileUnitV1,
    )

    from tests.support.builders.protocol_ingress import (
        REVIEWED_READING_BODY,
        authenticated_frame,
    )

    body = (
        sample.to_bytes(4, "little")
        + REVIEWED_READING_BODY[4:6]
        + soil.to_bytes(2, "little")
        + REVIEWED_READING_BODY[8:]
    )
    frame = authenticated_frame(
        node_id=node, message_id=message, domain=domain, body=body
    )
    profile = replace(
        _profile(sequence=sequence),
        receiver_instance_id=instance,
        received_frame_length=54,
        received_frame=frame + bytes(201),
        claimed_control=0x20,
        claimed_domain=domain,
        claimed_node_id=node,
        claimed_message_id=message,
        header_authenticated=True,
        decoded_sample_id=sample,
        processing_result=enum.ProcessingResult.ACCEPTED,
        ack_selected=enum.AckSelection.ACCEPTED,
        ack_tx_result=enum.AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
        ack_frame=authenticated_frame(
            node_id=node, message_id=message, domain=3, body=b"\x00"
        ),
        t3_authentication_completed_monotonic_us=13,
    )
    return MeasurementProfileUnitV1(
        AuthenticatedReadingCandidateV1(node, message, domain, sample, body), profile
    )

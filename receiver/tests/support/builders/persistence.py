"""Reviewed literal ordinary-persistence inputs; builders calculate no expectations."""

from dataclasses import replace

from cura_receiver.generated import receiver_entities_generated as row
from cura_receiver.generated import receiver_enums_generated as enum
from tests.support.builders.protocol_ingress import authenticated_frame

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
        enum.SystemTimeQuality.CHRONY_SYNCED,
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
    return row.PacketReceptionV1(
        receiver_instance_id=INSTANCE,
        occurrence_sequence=sequence,
        received_at_monotonic_us=10,
        domain=3,
        node_id=b"n" * 8,
        message_id=100,
        decoded_sample_id=None,
        rssi_dbm_x2=None,
        snr_db_x4=None,
        irq_status=2,
        device_errors=0,
        processing_result=enum.ProcessingResult.WRONG_DIRECTION,
        ack_tx_result=enum.AckTxResult.NOT_APPLICABLE,
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
    )


def _radio_observation(*, sequence=1):
    return row.RadioObservationV1(INSTANCE, sequence, row.RadioSnapshotV1(
        trigger=enum.RadioObservationTrigger.HEALTH,
        software_state=enum.RadioState.RX_SINGLE,
        capture_started_at_monotonic_us=13, capture_finished_at_monotonic_us=14,
        capture_status=enum.RadioCaptureStatus.COMPLETE,
        chip_status=0x52, chip_mode=enum.RadioChipMode.RX,
        irq_status=0, device_errors=0, dio1_before=False, dio1_after=False, busy=False,
        gpio_event_pending=False, event_timestamp_ns=None, event_sequence=None, last_set_rx_issued_monotonic_us=1,
        failure_code=None, failure_stage=None, failure_opcode=None, os_errno=None,
        history=b'[]', history_dropped=0,
    ))


def _health_request(*, sequence=1):
    from cura_receiver.persist_queue_entities import ReceiverHealthRequestV1

    return ReceiverHealthRequestV1(
        receiver_instance_id=INSTANCE,
        health_sequence=sequence,
        communicator_sampled_at_monotonic_us=15,
        radio_observation=_radio_observation(sequence=sequence),
        consistency=enum.HealthConsistency.CONSISTENT,
        radio_header_errors_observed=0, radio_header_errors_handled=0,
        radio_crc_errors_observed=0, radio_crc_errors_handled=0,
        radio_observations_dropped=0,
        radio_recovery_attempts=0,
        radio_recovery_successes=0,
        radio_recovery_failures=0,
        radio_recovery_attempts_by_reason=(0,) * 8,
        system_time_quality=enum.SystemTimeQuality.UNTRUSTED,
        rtc_health=enum.RtcHealth.PRESENT,
        rtc_read_status=enum.RtcReadStatus.OK, rtc_read_errno=None,
        rtc_read_started_at_monotonic_us=14, rtc_read_finished_at_monotonic_us=15,
        rtc_read_utc_s=1_800_000_000,
        time_quality_transition_count=0,
        rtc_health_transition_count=0,
        last_time_quality_transition_monotonic_us=None,
        last_rtc_health_transition_monotonic_us=None,
        chrony_step_command_results=(0, 0, 0),
        rtc_write_results=(0, 0, 0),
        rtc_write_readback_verified_count=0,
        rtc_write_trust_invalidated_count=0,
        persist_queue_admission_counts=((0, 0, 0),) * 6,
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
        ReadingReceptionUnitV2,
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
        domain=domain, node_id=node, message_id=message,
        decoded_sample_id=sample,
        processing_result=enum.ProcessingResult.ACCEPTED,
        ack_tx_result=enum.AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
        t3_authentication_completed_monotonic_us=13,
    )
    return ReadingReceptionUnitV2(
        AuthenticatedReadingCandidateV1(node, message, domain, sample, body), profile, frame
    )


REVIEWED_RECEPTION_FRAME = authenticated_frame(node_id=b"n"*8, message_id=100, domain=3, body=b"\x00")

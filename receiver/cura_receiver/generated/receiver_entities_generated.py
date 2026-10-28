# Generated from receiver/schemas/receiver_entities.json by receiver/tools/generate.py.
# Do not edit by hand.
from __future__ import annotations

import hashlib
import struct

from dataclasses import dataclass

from .receiver_enums_generated import (
    AckTxResult,
    DiagnosticErrorDomain,
    DiagnosticOperation,
    DiagnosticSeverity,
    HealthConsistency,
    PacketExceptionReason,
    PersistenceAdmissionState,
    PersistenceClassification,
    PersistQueueEntityKind,
    ProcessingResult,
    QuarantineFailureReason,
    RadioCaptureStatus,
    RadioChipMode,
    RadioObservationTrigger,
    RadioState,
    RtcHealth,
    RtcReadStatus,
    SystemTimeQuality,
)

RECEIVER_ENTITY_MANIFEST_SHA256 = '68dbca63f31af3f30b38ce74f58b8e998bfb404f7d3fc339f6dbc69c75f738a4'

__all__ = [
    "RECEIVER_ENTITY_MANIFEST_SHA256",
    "REQUIRED_FOREIGN_KEYS",
    "CLOCK_OBSERVATION_V1_TABLE",
    "CLOCK_OBSERVATION_V1_COLUMNS",
    "DIAGNOSTIC_V1_TABLE",
    "DIAGNOSTIC_V1_COLUMNS",
    "QUARANTINED_ENTITY_ROW_V1_TABLE",
    "QUARANTINED_ENTITY_ROW_V1_COLUMNS",
    "RADIO_OBSERVATION_V1_TABLE",
    "RADIO_OBSERVATION_V1_COLUMNS",
    "RECEIVER_HEALTH_V1_TABLE",
    "RECEIVER_HEALTH_V1_COLUMNS",
    "RECEIVED_MESSAGE_ROW_V1_TABLE",
    "RECEIVED_MESSAGE_ROW_V1_COLUMNS",
    "READING_MESSAGE_ROW_V2_TABLE",
    "READING_MESSAGE_ROW_V2_COLUMNS",
    "PACKET_RECEPTION_ROW_V1_TABLE",
    "PACKET_RECEPTION_ROW_V1_COLUMNS",
    "PACKET_RECEPTION_EXCEPTION_ROW_V1_TABLE",
    "PACKET_RECEPTION_EXCEPTION_ROW_V1_COLUMNS",
    "COMMUNICATOR_STATE_V2_TABLE",
    "COMMUNICATOR_STATE_V2_COLUMNS",
    "NetworkClockEvidenceV1",
    "RtcClockEvidenceV2",
    "PacketReceptionV1",
    "RadioSnapshotV1",
    "ClockObservationV1",
    "DiagnosticV1",
    "QuarantinedEntityRowV1",
    "RadioObservationV1",
    "ReceiverHealthV1",
    "ReceivedMessageRowV1",
    "ReadingMessageRowV2",
    "PacketReceptionRowV1",
    "PacketReceptionExceptionRowV1",
    "RtcProvenanceV1",
    "AirtimeEntryV2",
    "AirtimeSnapshotV1",
    "CommunicatorStateV2",
    "encode_communicator_state_v2",
    "decode_communicator_state_v2",
    "clock_observation_v1_parameters",
    "diagnostic_v1_parameters",
    "quarantined_entity_row_v1_parameters",
    "radio_observation_v1_parameters",
    "receiver_health_v1_parameters",
    "received_message_row_v1_parameters",
    "reading_message_row_v2_parameters",
    "packet_reception_row_v1_parameters",
    "packet_reception_exception_row_v1_parameters",
    "communicator_state_v2_parameters",
]

REQUIRED_FOREIGN_KEYS = (
    ('clock_observations', ('receiver_instance_id',), 'receiver_instances', ('receiver_instance_id',)),
    ('clock_observations', ('system_time_quality_id',), 'system_time_quality_codes', ('id',)),
    ('clock_observations', ('rtc_health_id',), 'rtc_health_codes', ('id',)),
    ('diagnostics', ('receiver_instance_id',), 'receiver_instances', ('receiver_instance_id',)),
    ('diagnostics', ('severity_id',), 'diagnostic_severity_codes', ('id',)),
    ('diagnostics', ('error_domain_id',), 'diagnostic_error_domain_codes', ('id',)),
    ('diagnostics', ('operation_id',), 'diagnostic_operation_codes', ('id',)),
    ('diagnostics', ('error_domain_id', 'error_code_id'), 'diagnostic_error_codes', ('error_domain_id', 'id')),
    ('diagnostics', ('error_domain_id', 'context_schema_id'), 'diagnostic_context_schemas', ('error_domain_id', 'id')),
    ('quarantined_entities', ('entity_kind_id',), 'persist_queue_entity_kind_codes', ('id',)),
    ('quarantined_entities', ('receiver_instance_id',), 'receiver_instances', ('receiver_instance_id',)),
    ('quarantined_entities', ('failure_reason_id',), 'quarantine_failure_reason_codes', ('id',)),
    ('quarantined_entities', ('failure_operation_id',), 'diagnostic_operation_codes', ('id',)),
    ('radio_observations', ('receiver_instance_id',), 'receiver_instances', ('receiver_instance_id',)),
    ('radio_observations', ('trigger_id',), 'radio_observation_trigger_codes', ('id',)),
    ('radio_observations', ('software_state_id',), 'radio_state_codes', ('id',)),
    ('radio_observations', ('capture_status_id',), 'radio_capture_status_codes', ('id',)),
    ('radio_observations', ('chip_mode_id',), 'radio_chip_mode_codes', ('id',)),
    ('receiver_health', ('receiver_instance_id',), 'receiver_instances', ('receiver_instance_id',)),
    ('receiver_health', ('system_time_quality_id',), 'system_time_quality_codes', ('id',)),
    ('receiver_health', ('rtc_health_id',), 'rtc_health_codes', ('id',)),
    ('receiver_health', ('persistence_admission_state_id',), 'persistence_admission_state_codes', ('id',)),
    ('receiver_health', ('receiver_instance_id', 'radio_observation_sequence'), 'radio_observations', ('receiver_instance_id', 'observation_sequence')),
    ('receiver_health', ('consistency_id',), 'health_consistency_codes', ('id',)),
    ('receiver_health', ('rtc_read_status_id',), 'rtc_read_status_codes', ('id',)),
    ('reading_messages', ('node_id', 'message_id'), 'received_messages', ('node_id', 'message_id')),
    ('packet_receptions', ('receiver_instance_id',), 'receiver_instances', ('receiver_instance_id',)),
    ('packet_receptions', ('processing_result_id',), 'processing_result_codes', ('id',)),
    ('packet_receptions', ('ack_tx_result_id',), 'ack_tx_result_codes', ('id',)),
    ('packet_receptions', ('persistence_classification_id',), 'persistence_classification_codes', ('id',)),
    ('packet_reception_exceptions', ('receiver_instance_id', 'occurrence_sequence'), 'packet_receptions', ('receiver_instance_id', 'occurrence_sequence')),
    ('packet_reception_exceptions', ('reason_id',), 'packet_exception_reason_codes', ('id',)),
)

CLOCK_OBSERVATION_V1_TABLE = 'clock_observations'

CLOCK_OBSERVATION_V1_COLUMNS = (
    'receiver_instance_id',
    'observation_sequence',
    'clock_state_generation',
    'sampled_at_monotonic_us',
    'sampled_at_utc_us',
    'step_discontinuity_boundary',
    'system_time_quality_id',
    'rtc_health_id',
    'error_bound_us',
    'sample_started_at_monotonic_us',
    'sample_finished_at_monotonic_us',
    'error_budget_expires_at_monotonic_us',
    'network_tracking_started_at_monotonic_us',
    'network_tracking_finished_at_monotonic_us',
    'network_remaining_correction_us',
    'network_root_delay_us',
    'network_root_dispersion_us',
    'network_root_distance_us',
    'network_estimated_frequency_ppb',
    'network_estimated_skew_ppb',
    'network_reference_id',
    'network_reference_time_utc_us',
    'network_stratum',
    'rtc_reference_receiver_instance_id',
    'rtc_reference_verified_at_utc_us',
    'rtc_reference_readback_utc_us',
    'rtc_reference_uncertainty_us',
    'rtc_reference_drift_bound_ppm',
)

DIAGNOSTIC_V1_TABLE = 'diagnostics'

DIAGNOSTIC_V1_COLUMNS = (
    'receiver_instance_id',
    'diagnostic_sequence',
    'sampled_at_monotonic_us',
    'severity_id',
    'error_domain_id',
    'operation_id',
    'error_code_id',
    'context_schema_id',
    'context_length',
    'context',
)

QUARANTINED_ENTITY_ROW_V1_TABLE = 'quarantined_entities'

QUARANTINED_ENTITY_ROW_V1_COLUMNS = (
    'quarantine_id',
    'entity_kind_id',
    'entity_schema_version',
    'entity_length',
    'entity_bytes',
    'receiver_instance_id',
    'quarantined_at_monotonic_us',
    'database_schema_version',
    'failure_reason_id',
    'failure_operation_id',
    'sqlite_primary_code',
    'sqlite_extended_code',
    'os_errno',
    'isolation_attempt_count',
)

RADIO_OBSERVATION_V1_TABLE = 'radio_observations'

RADIO_OBSERVATION_V1_COLUMNS = (
    'receiver_instance_id',
    'observation_sequence',
    'trigger_id',
    'software_state_id',
    'capture_started_at_monotonic_us',
    'capture_finished_at_monotonic_us',
    'capture_status_id',
    'chip_status',
    'chip_mode_id',
    'irq_status',
    'device_errors',
    'dio1_before',
    'dio1_after',
    'busy',
    'gpio_event_pending',
    'event_timestamp_ns',
    'event_sequence',
    'last_set_rx_issued_monotonic_us',
    'failure_code',
    'failure_stage',
    'failure_opcode',
    'os_errno',
    'history',
    'history_dropped',
)

RECEIVER_HEALTH_V1_TABLE = 'receiver_health'

RECEIVER_HEALTH_V1_COLUMNS = (
    'receiver_instance_id',
    'health_sequence',
    'communicator_sampled_at_monotonic_us',
    'radio_observation_sequence',
    'consistency_id',
    'radio_header_errors_observed',
    'radio_header_errors_handled',
    'radio_crc_errors_observed',
    'radio_crc_errors_handled',
    'radio_observations_dropped',
    'radio_recovery_attempts',
    'radio_recovery_successes',
    'radio_recovery_failures',
    'radio_recovery_attempts_by_reason_busy_timeout',
    'radio_recovery_attempts_by_reason_spi_failure',
    'radio_recovery_attempts_by_reason_unexpected_irq',
    'radio_recovery_attempts_by_reason_tx_outcome_uncertain',
    'radio_recovery_attempts_by_reason_rx_profile_restore_failed',
    'radio_recovery_attempts_by_reason_set_rx_failed',
    'radio_recovery_attempts_by_reason_status_unconfirmed',
    'radio_recovery_attempts_by_reason_hardware_unreachable',
    'system_time_quality_id',
    'rtc_health_id',
    'rtc_read_status_id',
    'rtc_read_errno',
    'rtc_read_started_at_monotonic_us',
    'rtc_read_finished_at_monotonic_us',
    'rtc_read_utc_s',
    'time_quality_transition_count',
    'rtc_health_transition_count',
    'last_time_quality_transition_monotonic_us',
    'last_rtc_health_transition_monotonic_us',
    'chrony_step_command_results_submitted',
    'chrony_step_command_results_not_submitted',
    'chrony_step_command_results_outcome_unknown',
    'rtc_write_results_completed',
    'rtc_write_results_not_applied',
    'rtc_write_results_outcome_unknown',
    'rtc_write_readback_verified_count',
    'rtc_write_trust_invalidated_count',
    'persist_queue_admission_counts_measurement_profile_reserved',
    'persist_queue_admission_counts_measurement_profile_persistence_unavailable',
    'persist_queue_admission_counts_measurement_profile_queue_full',
    'persist_queue_admission_counts_profile_only_reserved',
    'persist_queue_admission_counts_profile_only_persistence_unavailable',
    'persist_queue_admission_counts_profile_only_queue_full',
    'persist_queue_admission_counts_receiver_health_request_reserved',
    'persist_queue_admission_counts_receiver_health_request_persistence_unavailable',
    'persist_queue_admission_counts_receiver_health_request_queue_full',
    'persist_queue_admission_counts_diagnostic_reserved',
    'persist_queue_admission_counts_diagnostic_persistence_unavailable',
    'persist_queue_admission_counts_diagnostic_queue_full',
    'persist_queue_admission_counts_clock_observation_reserved',
    'persist_queue_admission_counts_clock_observation_persistence_unavailable',
    'persist_queue_admission_counts_clock_observation_queue_full',
    'persist_queue_admission_counts_radio_observation_reserved',
    'persist_queue_admission_counts_radio_observation_persistence_unavailable',
    'persist_queue_admission_counts_radio_observation_queue_full',
    'persistence_sampled_at_monotonic_us',
    'persistence_admission_generation',
    'persistence_admission_state_id',
    'persistence_admission_changed_at_monotonic_us',
    'persistence_admission_transition_counts_unavailable_starting',
    'persistence_admission_transition_counts_available',
    'persistence_admission_transition_counts_unavailable_low_space',
    'persistence_admission_transition_counts_unavailable_disk_full',
    'persistence_admission_transition_counts_unavailable_corrupt',
    'persistence_admission_transition_counts_unavailable_io',
    'persistence_admission_transition_counts_unavailable_incompatible_schema',
    'durable_quarantine_successes',
    'durable_quarantine_failures',
    'batch_transaction_attempts',
    'batch_transaction_commits',
    'batch_transaction_failures',
    'batch_entities_committed',
    'batch_commit_duration_total_us',
    'batch_commit_duration_max_us',
    'wal_checkpoint_attempts',
    'wal_checkpoint_successes',
    'wal_checkpoint_failures',
    'linux_load_1m_milli',
    'cpu_temperature_milli_c',
    'memory_available_bytes',
    'sqlite_filesystem_available_bytes',
    'sqlite_database_size_bytes',
    'sqlite_wal_size_bytes',
    'ntp_offset_us',
    'storage_device_write_bytes',
    'receiver_process_write_bytes',
)

RECEIVED_MESSAGE_ROW_V1_TABLE = 'received_messages'

RECEIVED_MESSAGE_ROW_V1_COLUMNS = (
    'node_id',
    'message_id',
    'domain',
)

READING_MESSAGE_ROW_V2_TABLE = 'reading_messages'

READING_MESSAGE_ROW_V2_COLUMNS = (
    'node_id',
    'message_id',
    'sample_id',
    'is_canonical',
    'run_ms',
    'soil_0_mv',
    'soil_1_mv',
    'soil_temp_0_centi_c',
    'soil_temp_1_centi_c',
    'enclosure_centi_c',
    'enclosure_pressure_pa',
    'enclosure_humidity_centi_pct',
    'reset_reason',
    'previous_current_tx_attempts',
    'previous_awake_ms',
    'previous_current_delivery_ms',
    'previous_cycle_tx_attempts',
    'previous_cycle_accepted_readings',
    'flags',
)

PACKET_RECEPTION_ROW_V1_TABLE = 'packet_receptions'

PACKET_RECEPTION_ROW_V1_COLUMNS = (
    'receiver_instance_id',
    'occurrence_sequence',
    'received_at_monotonic_us',
    'domain',
    'node_id',
    'message_id',
    'decoded_sample_id',
    'rssi_dbm_x2',
    'snr_db_x4',
    'irq_status',
    'device_errors',
    'processing_result_id',
    'ack_tx_result_id',
    'busy_wait_total_us',
    'busy_wait_max_us',
    'busy_wait_count',
    'busy_timeout_count',
    'last_busy_timeout_opcode',
    't1_handler_started_monotonic_us',
    't2_packet_copied_monotonic_us',
    't3_authentication_completed_monotonic_us',
    't4_set_tx_attempted_monotonic_us',
    't5_tx_done_monotonic_us',
    't6_set_rx_issued_monotonic_us',
    'persistence_classification_id',
)

PACKET_RECEPTION_EXCEPTION_ROW_V1_TABLE = 'packet_reception_exceptions'

PACKET_RECEPTION_EXCEPTION_ROW_V1_COLUMNS = (
    'receiver_instance_id',
    'occurrence_sequence',
    'received_frame_length',
    'received_frame',
    'reason_id',
)

COMMUNICATOR_STATE_V2_TABLE = 'communicator_state'

COMMUNICATOR_STATE_V2_COLUMNS = (
    'singleton_id',
    'state_format_version',
    'generation',
    'state_blob',
    'state_sha256',
)

def _pack(format_code: str, value: object, field: str) -> bytes:
    try:
        return struct.pack(format_code, value)
    except struct.error as exc:
        raise ValueError(f'{field} does not fit its encoded type') from exc

def _unpack(
    blob: memoryview, offset: int, format_code: str, field: str
) -> tuple[object, int]:
    size = struct.calcsize(format_code)
    if offset + size > len(blob):
        raise ValueError(f'{field} is truncated')
    return struct.unpack_from(format_code, blob, offset)[0], offset + size

def _read_exact(
    blob: memoryview, offset: int, length: int, field: str
) -> tuple[bytes, int]:
    end = offset + length
    if end > len(blob):
        raise ValueError(f'{field} is truncated')
    return bytes(blob[offset:end]), end

def _require_bytes_length(value: bytes, length: int, field: str) -> bytes:
    if len(value) != length:
        raise ValueError(f'{field} must contain exactly {length} bytes')
    return value

@dataclass(frozen=True, slots=True)
class NetworkClockEvidenceV1:
    tracking_started_at_monotonic_us: int
    tracking_finished_at_monotonic_us: int
    remaining_correction_us: int
    root_delay_us: int
    root_dispersion_us: int
    root_distance_us: int
    estimated_frequency_ppb: int
    estimated_skew_ppb: int
    reference_id: int
    reference_time_utc_us: int
    stratum: int

@dataclass(frozen=True, slots=True)
class RtcClockEvidenceV2:
    reference_receiver_instance_id: bytes
    reference_verified_at_utc_us: int
    reference_readback_utc_us: int
    reference_uncertainty_us: int
    reference_drift_bound_ppm: int

@dataclass(frozen=True, slots=True)
class PacketReceptionV1:
    receiver_instance_id: bytes
    occurrence_sequence: int
    received_at_monotonic_us: int
    domain: int
    node_id: bytes
    message_id: int
    decoded_sample_id: int | None
    rssi_dbm_x2: int | None
    snr_db_x4: int | None
    irq_status: int | None
    device_errors: int | None
    processing_result: ProcessingResult
    ack_tx_result: AckTxResult
    busy_wait_total_us: int
    busy_wait_max_us: int
    busy_wait_count: int
    busy_timeout_count: int
    last_busy_timeout_opcode: int | None
    t1_handler_started_monotonic_us: int
    t2_packet_copied_monotonic_us: int | None
    t3_authentication_completed_monotonic_us: int | None
    t4_set_tx_attempted_monotonic_us: int | None
    t5_tx_done_monotonic_us: int | None
    t6_set_rx_issued_monotonic_us: int | None

@dataclass(frozen=True, slots=True)
class RadioSnapshotV1:
    trigger: RadioObservationTrigger
    software_state: RadioState
    capture_started_at_monotonic_us: int
    capture_finished_at_monotonic_us: int
    capture_status: RadioCaptureStatus
    chip_status: int | None
    chip_mode: RadioChipMode | None
    irq_status: int | None
    device_errors: int | None
    dio1_before: bool | None
    dio1_after: bool | None
    busy: bool | None
    gpio_event_pending: bool | None
    event_timestamp_ns: int | None
    event_sequence: int | None
    last_set_rx_issued_monotonic_us: int | None
    failure_code: int | None
    failure_stage: int | None
    failure_opcode: int | None
    os_errno: int | None
    history: bytes
    history_dropped: int

@dataclass(frozen=True, slots=True)
class ClockObservationV1:
    receiver_instance_id: bytes
    observation_sequence: int
    clock_state_generation: int
    sampled_at_monotonic_us: int
    sampled_at_utc_us: int | None
    step_discontinuity_boundary: bool
    system_time_quality: SystemTimeQuality
    rtc_health: RtcHealth
    error_bound_us: int | None
    sample_started_at_monotonic_us: int | None
    sample_finished_at_monotonic_us: int | None
    error_budget_expires_at_monotonic_us: int | None
    network_evidence: NetworkClockEvidenceV1 | None
    rtc_evidence: RtcClockEvidenceV2 | None

def clock_observation_v1_parameters(entity: ClockObservationV1) -> tuple[object, ...]:
    return (
        entity.receiver_instance_id,
        entity.observation_sequence,
        entity.clock_state_generation,
        entity.sampled_at_monotonic_us,
        entity.sampled_at_utc_us,
        entity.step_discontinuity_boundary,
        entity.system_time_quality.value,
        entity.rtc_health.value,
        entity.error_bound_us,
        entity.sample_started_at_monotonic_us,
        entity.sample_finished_at_monotonic_us,
        entity.error_budget_expires_at_monotonic_us,
        None if entity.network_evidence is None else entity.network_evidence.tracking_started_at_monotonic_us,
        None if entity.network_evidence is None else entity.network_evidence.tracking_finished_at_monotonic_us,
        None if entity.network_evidence is None else entity.network_evidence.remaining_correction_us,
        None if entity.network_evidence is None else entity.network_evidence.root_delay_us,
        None if entity.network_evidence is None else entity.network_evidence.root_dispersion_us,
        None if entity.network_evidence is None else entity.network_evidence.root_distance_us,
        None if entity.network_evidence is None else entity.network_evidence.estimated_frequency_ppb,
        None if entity.network_evidence is None else entity.network_evidence.estimated_skew_ppb,
        None if entity.network_evidence is None else entity.network_evidence.reference_id,
        None if entity.network_evidence is None else entity.network_evidence.reference_time_utc_us,
        None if entity.network_evidence is None else entity.network_evidence.stratum,
        None if entity.rtc_evidence is None else entity.rtc_evidence.reference_receiver_instance_id,
        None if entity.rtc_evidence is None else entity.rtc_evidence.reference_verified_at_utc_us,
        None if entity.rtc_evidence is None else entity.rtc_evidence.reference_readback_utc_us,
        None if entity.rtc_evidence is None else entity.rtc_evidence.reference_uncertainty_us,
        None if entity.rtc_evidence is None else entity.rtc_evidence.reference_drift_bound_ppm,
    )

@dataclass(frozen=True, slots=True)
class DiagnosticV1:
    receiver_instance_id: bytes
    diagnostic_sequence: int
    sampled_at_monotonic_us: int
    severity: DiagnosticSeverity
    error_domain: DiagnosticErrorDomain
    operation: DiagnosticOperation
    error_code: int
    context_schema: int
    context_length: int
    context: bytes

def diagnostic_v1_parameters(entity: DiagnosticV1) -> tuple[object, ...]:
    return (
        entity.receiver_instance_id,
        entity.diagnostic_sequence,
        entity.sampled_at_monotonic_us,
        entity.severity.value,
        entity.error_domain.value,
        entity.operation.value,
        entity.error_code,
        entity.context_schema,
        entity.context_length,
        entity.context,
    )

@dataclass(frozen=True, slots=True)
class QuarantinedEntityRowV1:
    quarantine_id: bytes
    entity_kind: PersistQueueEntityKind
    entity_schema_version: int
    entity_length: int
    entity_bytes: bytes
    receiver_instance_id: bytes
    quarantined_at_monotonic_us: int
    database_schema_version: int
    failure_reason: QuarantineFailureReason
    failure_operation: DiagnosticOperation
    sqlite_primary_code: int | None
    sqlite_extended_code: int | None
    os_errno: int | None
    isolation_attempt_count: int

def quarantined_entity_row_v1_parameters(entity: QuarantinedEntityRowV1) -> tuple[object, ...]:
    return (
        entity.quarantine_id,
        entity.entity_kind.value,
        entity.entity_schema_version,
        entity.entity_length,
        entity.entity_bytes,
        entity.receiver_instance_id,
        entity.quarantined_at_monotonic_us,
        entity.database_schema_version,
        entity.failure_reason.value,
        entity.failure_operation.value,
        entity.sqlite_primary_code,
        entity.sqlite_extended_code,
        entity.os_errno,
        entity.isolation_attempt_count,
    )

@dataclass(frozen=True, slots=True)
class RadioObservationV1:
    receiver_instance_id: bytes
    observation_sequence: int
    snapshot: RadioSnapshotV1

def radio_observation_v1_parameters(entity: RadioObservationV1) -> tuple[object, ...]:
    return (
        entity.receiver_instance_id,
        entity.observation_sequence,
        entity.snapshot.trigger.value,
        entity.snapshot.software_state.value,
        entity.snapshot.capture_started_at_monotonic_us,
        entity.snapshot.capture_finished_at_monotonic_us,
        entity.snapshot.capture_status.value,
        entity.snapshot.chip_status,
        None if entity.snapshot.chip_mode is None else entity.snapshot.chip_mode.value,
        entity.snapshot.irq_status,
        entity.snapshot.device_errors,
        entity.snapshot.dio1_before,
        entity.snapshot.dio1_after,
        entity.snapshot.busy,
        entity.snapshot.gpio_event_pending,
        entity.snapshot.event_timestamp_ns,
        entity.snapshot.event_sequence,
        entity.snapshot.last_set_rx_issued_monotonic_us,
        entity.snapshot.failure_code,
        entity.snapshot.failure_stage,
        entity.snapshot.failure_opcode,
        entity.snapshot.os_errno,
        entity.snapshot.history,
        entity.snapshot.history_dropped,
    )

@dataclass(frozen=True, slots=True)
class ReceiverHealthV1:
    receiver_instance_id: bytes
    health_sequence: int
    communicator_sampled_at_monotonic_us: int
    radio_observation_sequence: int
    consistency: HealthConsistency
    radio_header_errors_observed: int
    radio_header_errors_handled: int
    radio_crc_errors_observed: int
    radio_crc_errors_handled: int
    radio_observations_dropped: int
    radio_recovery_attempts: int
    radio_recovery_successes: int
    radio_recovery_failures: int
    radio_recovery_attempts_by_reason: tuple[int, ...]
    system_time_quality: SystemTimeQuality
    rtc_health: RtcHealth
    rtc_read_status: RtcReadStatus
    rtc_read_errno: int | None
    rtc_read_started_at_monotonic_us: int
    rtc_read_finished_at_monotonic_us: int
    rtc_read_utc_s: int | None
    time_quality_transition_count: int
    rtc_health_transition_count: int
    last_time_quality_transition_monotonic_us: int | None
    last_rtc_health_transition_monotonic_us: int | None
    chrony_step_command_results: tuple[int, ...]
    rtc_write_results: tuple[int, ...]
    rtc_write_readback_verified_count: int
    rtc_write_trust_invalidated_count: int
    persist_queue_admission_counts: tuple[tuple[int, ...], ...]
    persistence_sampled_at_monotonic_us: int
    persistence_admission_generation: int
    persistence_admission_state: PersistenceAdmissionState
    persistence_admission_changed_at_monotonic_us: int
    persistence_admission_transition_counts: tuple[int, ...]
    durable_quarantine_successes: int
    durable_quarantine_failures: int
    batch_transaction_attempts: int
    batch_transaction_commits: int
    batch_transaction_failures: int
    batch_entities_committed: int
    batch_commit_duration_total_us: int
    batch_commit_duration_max_us: int
    wal_checkpoint_attempts: int
    wal_checkpoint_successes: int
    wal_checkpoint_failures: int
    linux_load_1m_milli: int | None
    cpu_temperature_milli_c: int | None
    memory_available_bytes: int | None
    sqlite_filesystem_available_bytes: int | None
    sqlite_database_size_bytes: int | None
    sqlite_wal_size_bytes: int | None
    ntp_offset_us: int | None
    storage_device_write_bytes: int | None
    receiver_process_write_bytes: int | None

def receiver_health_v1_parameters(entity: ReceiverHealthV1) -> tuple[object, ...]:
    if len(entity.radio_recovery_attempts_by_reason) != 8:
        raise ValueError(
            'radio_recovery_attempts_by_reason' + ' must have length 8'
        )
    if len(entity.chrony_step_command_results) != 3:
        raise ValueError(
            'chrony_step_command_results' + ' must have length 3'
        )
    if len(entity.rtc_write_results) != 3:
        raise ValueError(
            'rtc_write_results' + ' must have length 3'
        )
    if len(entity.persist_queue_admission_counts) != 6:
        raise ValueError(
            'persist_queue_admission_counts' + ' must have length 6'
        )
    if any(len(item) != 3 for item in entity.persist_queue_admission_counts):
        raise ValueError(
            'persist_queue_admission_counts' + ' inner arrays must have length 3'
        )
    if len(entity.persistence_admission_transition_counts) != 7:
        raise ValueError(
            'persistence_admission_transition_counts' + ' must have length 7'
        )
    return (
        entity.receiver_instance_id,
        entity.health_sequence,
        entity.communicator_sampled_at_monotonic_us,
        entity.radio_observation_sequence,
        entity.consistency.value,
        entity.radio_header_errors_observed,
        entity.radio_header_errors_handled,
        entity.radio_crc_errors_observed,
        entity.radio_crc_errors_handled,
        entity.radio_observations_dropped,
        entity.radio_recovery_attempts,
        entity.radio_recovery_successes,
        entity.radio_recovery_failures,
        entity.radio_recovery_attempts_by_reason[0],
        entity.radio_recovery_attempts_by_reason[1],
        entity.radio_recovery_attempts_by_reason[2],
        entity.radio_recovery_attempts_by_reason[3],
        entity.radio_recovery_attempts_by_reason[4],
        entity.radio_recovery_attempts_by_reason[5],
        entity.radio_recovery_attempts_by_reason[6],
        entity.radio_recovery_attempts_by_reason[7],
        entity.system_time_quality.value,
        entity.rtc_health.value,
        entity.rtc_read_status.value,
        entity.rtc_read_errno,
        entity.rtc_read_started_at_monotonic_us,
        entity.rtc_read_finished_at_monotonic_us,
        entity.rtc_read_utc_s,
        entity.time_quality_transition_count,
        entity.rtc_health_transition_count,
        entity.last_time_quality_transition_monotonic_us,
        entity.last_rtc_health_transition_monotonic_us,
        entity.chrony_step_command_results[0],
        entity.chrony_step_command_results[1],
        entity.chrony_step_command_results[2],
        entity.rtc_write_results[0],
        entity.rtc_write_results[1],
        entity.rtc_write_results[2],
        entity.rtc_write_readback_verified_count,
        entity.rtc_write_trust_invalidated_count,
        entity.persist_queue_admission_counts[0][0],
        entity.persist_queue_admission_counts[0][1],
        entity.persist_queue_admission_counts[0][2],
        entity.persist_queue_admission_counts[1][0],
        entity.persist_queue_admission_counts[1][1],
        entity.persist_queue_admission_counts[1][2],
        entity.persist_queue_admission_counts[2][0],
        entity.persist_queue_admission_counts[2][1],
        entity.persist_queue_admission_counts[2][2],
        entity.persist_queue_admission_counts[3][0],
        entity.persist_queue_admission_counts[3][1],
        entity.persist_queue_admission_counts[3][2],
        entity.persist_queue_admission_counts[4][0],
        entity.persist_queue_admission_counts[4][1],
        entity.persist_queue_admission_counts[4][2],
        entity.persist_queue_admission_counts[5][0],
        entity.persist_queue_admission_counts[5][1],
        entity.persist_queue_admission_counts[5][2],
        entity.persistence_sampled_at_monotonic_us,
        entity.persistence_admission_generation,
        entity.persistence_admission_state.value,
        entity.persistence_admission_changed_at_monotonic_us,
        entity.persistence_admission_transition_counts[0],
        entity.persistence_admission_transition_counts[1],
        entity.persistence_admission_transition_counts[2],
        entity.persistence_admission_transition_counts[3],
        entity.persistence_admission_transition_counts[4],
        entity.persistence_admission_transition_counts[5],
        entity.persistence_admission_transition_counts[6],
        entity.durable_quarantine_successes,
        entity.durable_quarantine_failures,
        entity.batch_transaction_attempts,
        entity.batch_transaction_commits,
        entity.batch_transaction_failures,
        entity.batch_entities_committed,
        entity.batch_commit_duration_total_us,
        entity.batch_commit_duration_max_us,
        entity.wal_checkpoint_attempts,
        entity.wal_checkpoint_successes,
        entity.wal_checkpoint_failures,
        entity.linux_load_1m_milli,
        entity.cpu_temperature_milli_c,
        entity.memory_available_bytes,
        entity.sqlite_filesystem_available_bytes,
        entity.sqlite_database_size_bytes,
        entity.sqlite_wal_size_bytes,
        entity.ntp_offset_us,
        entity.storage_device_write_bytes,
        entity.receiver_process_write_bytes,
    )

@dataclass(frozen=True, slots=True)
class ReceivedMessageRowV1:
    node_id: bytes
    message_id: int
    domain: int

def received_message_row_v1_parameters(entity: ReceivedMessageRowV1) -> tuple[object, ...]:
    return (
        entity.node_id,
        entity.message_id,
        entity.domain,
    )

@dataclass(frozen=True, slots=True)
class ReadingMessageRowV2:
    node_id: bytes
    message_id: int
    sample_id: int
    is_canonical: bool
    run_ms: int
    soil_0_mv: int
    soil_1_mv: int
    soil_temp_0_centi_c: int
    soil_temp_1_centi_c: int
    enclosure_centi_c: int
    enclosure_pressure_pa: int
    enclosure_humidity_centi_pct: int
    reset_reason: int
    previous_current_tx_attempts: int
    previous_awake_ms: int
    previous_current_delivery_ms: int
    previous_cycle_tx_attempts: int
    previous_cycle_accepted_readings: int
    flags: int

def reading_message_row_v2_parameters(entity: ReadingMessageRowV2) -> tuple[object, ...]:
    return (
        entity.node_id,
        entity.message_id,
        entity.sample_id,
        entity.is_canonical,
        entity.run_ms,
        entity.soil_0_mv,
        entity.soil_1_mv,
        entity.soil_temp_0_centi_c,
        entity.soil_temp_1_centi_c,
        entity.enclosure_centi_c,
        entity.enclosure_pressure_pa,
        entity.enclosure_humidity_centi_pct,
        entity.reset_reason,
        entity.previous_current_tx_attempts,
        entity.previous_awake_ms,
        entity.previous_current_delivery_ms,
        entity.previous_cycle_tx_attempts,
        entity.previous_cycle_accepted_readings,
        entity.flags,
    )

@dataclass(frozen=True, slots=True)
class PacketReceptionRowV1:
    profile: PacketReceptionV1
    persistence_classification: PersistenceClassification

def packet_reception_row_v1_parameters(entity: PacketReceptionRowV1) -> tuple[object, ...]:
    return (
        entity.profile.receiver_instance_id,
        entity.profile.occurrence_sequence,
        entity.profile.received_at_monotonic_us,
        entity.profile.domain,
        entity.profile.node_id,
        entity.profile.message_id,
        entity.profile.decoded_sample_id,
        entity.profile.rssi_dbm_x2,
        entity.profile.snr_db_x4,
        entity.profile.irq_status,
        entity.profile.device_errors,
        entity.profile.processing_result.value,
        entity.profile.ack_tx_result.value,
        entity.profile.busy_wait_total_us,
        entity.profile.busy_wait_max_us,
        entity.profile.busy_wait_count,
        entity.profile.busy_timeout_count,
        entity.profile.last_busy_timeout_opcode,
        entity.profile.t1_handler_started_monotonic_us,
        entity.profile.t2_packet_copied_monotonic_us,
        entity.profile.t3_authentication_completed_monotonic_us,
        entity.profile.t4_set_tx_attempted_monotonic_us,
        entity.profile.t5_tx_done_monotonic_us,
        entity.profile.t6_set_rx_issued_monotonic_us,
        entity.persistence_classification.value,
    )

@dataclass(frozen=True, slots=True)
class PacketReceptionExceptionRowV1:
    receiver_instance_id: bytes
    occurrence_sequence: int
    received_frame_length: int
    received_frame: bytes
    reason: PacketExceptionReason

def packet_reception_exception_row_v1_parameters(entity: PacketReceptionExceptionRowV1) -> tuple[object, ...]:
    return (
        entity.receiver_instance_id,
        entity.occurrence_sequence,
        entity.received_frame_length,
        entity.received_frame,
        entity.reason.value,
    )

@dataclass(frozen=True, slots=True)
class RtcProvenanceV1:
    verified_by_receiver_instance_id: bytes
    network_utc_at_verification_us: int
    rtc_readback_utc_us: int
    verification_uncertainty_us: int
    drift_bound_ppm: int

@dataclass(frozen=True, slots=True)
class AirtimeEntryV2:
    remaining_us: int

@dataclass(frozen=True, slots=True)
class AirtimeSnapshotV1:
    utc_us: int
    error_bound_us: int

@dataclass(frozen=True, slots=True)
class CommunicatorStateV2:
    generation: int
    last_observed_system_time_quality: SystemTimeQuality
    last_observed_rtc_health: RtcHealth
    rtc_provenance: RtcProvenanceV1 | None
    rolling_window_us: int
    tx_airtime_budget_us: int
    entry_charge_us: int
    airtime_snapshot: AirtimeSnapshotV1 | None
    entries: tuple[AirtimeEntryV2, ...]

def _encode_rtc_provenance_v1(value: RtcProvenanceV1) -> bytes:
    chunks: list[bytes] = []
    chunks.append(_require_bytes_length(
        value.verified_by_receiver_instance_id, 16, 'RtcProvenanceV1.verified_by_receiver_instance_id'
    ))
    chunks.append(_pack('<q', value.network_utc_at_verification_us, 'RtcProvenanceV1.network_utc_at_verification_us'))
    chunks.append(_pack('<q', value.rtc_readback_utc_us, 'RtcProvenanceV1.rtc_readback_utc_us'))
    chunks.append(_pack('<Q', value.verification_uncertainty_us, 'RtcProvenanceV1.verification_uncertainty_us'))
    chunks.append(_pack('<I', value.drift_bound_ppm, 'RtcProvenanceV1.drift_bound_ppm'))
    chunks.append(_pack('<I', 0, 'RtcProvenanceV1.reserved_1'))
    encoded = b''.join(chunks)
    if len(encoded) != 48:
        raise RuntimeError('RTC_PROVENANCE_V1 generated an invalid size')
    return encoded

def _decode_rtc_provenance_v1(
    blob: memoryview, offset: int
) -> tuple[RtcProvenanceV1, int]:
    verified_by_receiver_instance_id, offset = _read_exact(
        blob, offset, 16, 'RtcProvenanceV1.verified_by_receiver_instance_id'
    )
    network_utc_at_verification_us, offset = _unpack(
        blob, offset, '<q', 'RtcProvenanceV1.network_utc_at_verification_us'
    )
    rtc_readback_utc_us, offset = _unpack(
        blob, offset, '<q', 'RtcProvenanceV1.rtc_readback_utc_us'
    )
    verification_uncertainty_us, offset = _unpack(
        blob, offset, '<Q', 'RtcProvenanceV1.verification_uncertainty_us'
    )
    drift_bound_ppm, offset = _unpack(
        blob, offset, '<I', 'RtcProvenanceV1.drift_bound_ppm'
    )
    reserved_1, offset = _unpack(
        blob, offset, '<I', 'RtcProvenanceV1.reserved_1'
    )
    if reserved_1 != 0:
        raise ValueError('RtcProvenanceV1.reserved_1' + ' has an invalid constant value')
    return RtcProvenanceV1(
        verified_by_receiver_instance_id=verified_by_receiver_instance_id,
        network_utc_at_verification_us=network_utc_at_verification_us,
        rtc_readback_utc_us=rtc_readback_utc_us,
        verification_uncertainty_us=verification_uncertainty_us,
        drift_bound_ppm=drift_bound_ppm,
    ), offset

def _encode_airtime_entry_v2(value: AirtimeEntryV2) -> bytes:
    chunks: list[bytes] = []
    chunks.append(_pack('<Q', value.remaining_us, 'AirtimeEntryV2.remaining_us'))
    encoded = b''.join(chunks)
    if len(encoded) != 8:
        raise RuntimeError('AIRTIME_ENTRY_V2 generated an invalid size')
    return encoded

def _decode_airtime_entry_v2(
    blob: memoryview, offset: int
) -> tuple[AirtimeEntryV2, int]:
    remaining_us, offset = _unpack(
        blob, offset, '<Q', 'AirtimeEntryV2.remaining_us'
    )
    return AirtimeEntryV2(
        remaining_us=remaining_us,
    ), offset

def _encode_airtime_snapshot_v1(value: AirtimeSnapshotV1) -> bytes:
    chunks: list[bytes] = []
    chunks.append(_pack('<q', value.utc_us, 'AirtimeSnapshotV1.utc_us'))
    chunks.append(_pack('<Q', value.error_bound_us, 'AirtimeSnapshotV1.error_bound_us'))
    encoded = b''.join(chunks)
    if len(encoded) != 16:
        raise RuntimeError('AIRTIME_SNAPSHOT_V1 generated an invalid size')
    return encoded

def _decode_airtime_snapshot_v1(
    blob: memoryview, offset: int
) -> tuple[AirtimeSnapshotV1, int]:
    utc_us, offset = _unpack(
        blob, offset, '<q', 'AirtimeSnapshotV1.utc_us'
    )
    error_bound_us, offset = _unpack(
        blob, offset, '<Q', 'AirtimeSnapshotV1.error_bound_us'
    )
    return AirtimeSnapshotV1(
        utc_us=utc_us,
        error_bound_us=error_bound_us,
    ), offset

def encode_communicator_state_v2(entity: CommunicatorStateV2) -> bytes:
    if len(entity.entries) != 18:
        raise ValueError(
            'entries' + ' must have length 18'
        )
    encoded_length = 264
    validity_mask = ((1 << 0) if entity.rtc_provenance is not None else 0) | ((1 << 1) if entity.airtime_snapshot is not None else 0)
    entry_count = len(entity.entries)
    chunks: list[bytes] = []
    chunks.append(_pack('<H', 2, 'CommunicatorStateV2.state_format_version'))
    chunks.append(_pack('<I', encoded_length, 'CommunicatorStateV2.encoded_length'))
    chunks.append(_pack('<Q', entity.generation, 'CommunicatorStateV2.generation'))
    chunks.append(_pack('<H', validity_mask, 'CommunicatorStateV2.validity_mask'))
    chunks.append(_pack('<B', entity.last_observed_system_time_quality.value, 'CommunicatorStateV2.last_observed_system_time_quality'))
    chunks.append(_pack('<B', entity.last_observed_rtc_health.value, 'CommunicatorStateV2.last_observed_rtc_health'))
    chunks.append(_pack('<H', 0, 'CommunicatorStateV2.reserved_0'))
    if entity.rtc_provenance is None:
        chunks.append(bytes(48))
    else:
        chunks.append(_encode_rtc_provenance_v1(entity.rtc_provenance))
    chunks.append(_pack('<Q', entity.rolling_window_us, 'CommunicatorStateV2.rolling_window_us'))
    chunks.append(_pack('<Q', entity.tx_airtime_budget_us, 'CommunicatorStateV2.tx_airtime_budget_us'))
    chunks.append(_pack('<Q', entity.entry_charge_us, 'CommunicatorStateV2.entry_charge_us'))
    if entity.airtime_snapshot is None:
        chunks.append(bytes(16))
    else:
        chunks.append(_encode_airtime_snapshot_v1(entity.airtime_snapshot))
    chunks.append(_pack('<H', entry_count, 'CommunicatorStateV2.entry_count'))
    chunks.append(_pack('<H', 0, 'CommunicatorStateV2.reserved_2'))
    chunks.append(_pack('<Q', 0, 'CommunicatorStateV2.reserved_3'))
    for item in entity.entries:
        chunks.append(_encode_airtime_entry_v2(item))
    encoded = b''.join(chunks)
    if len(encoded) != encoded_length:
        raise RuntimeError('COMMUNICATOR_STATE_ENCODING_V2 generated an invalid length')
    return encoded

def decode_communicator_state_v2(blob: bytes) -> CommunicatorStateV2:
    try:
        view = memoryview(blob)
    except TypeError as exc:
        raise ValueError('canonical blob must be bytes-like') from exc
    blob = view
    offset = 0
    state_format_version, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV2.state_format_version'
    )
    if state_format_version != 2:
        raise ValueError('CommunicatorStateV2.state_format_version' + ' has an invalid constant value')
    encoded_length, offset = _unpack(
        blob, offset, '<I', 'CommunicatorStateV2.encoded_length'
    )
    if encoded_length != len(view):
        raise ValueError('CommunicatorStateV2.encoded_length' + ' does not match the blob length')
    generation, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV2.generation'
    )
    validity_mask, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV2.validity_mask'
    )
    if validity_mask & ~3:
        raise ValueError('CommunicatorStateV2.validity_mask' + ' has reserved bits set')
    last_observed_system_time_quality_value, offset = _unpack(
        blob, offset, '<B', 'CommunicatorStateV2.last_observed_system_time_quality'
    )
    try:
        last_observed_system_time_quality = SystemTimeQuality(last_observed_system_time_quality_value)
    except ValueError as exc:
        raise ValueError('CommunicatorStateV2.last_observed_system_time_quality' + ' has an unknown enum value') from exc
    last_observed_rtc_health_value, offset = _unpack(
        blob, offset, '<B', 'CommunicatorStateV2.last_observed_rtc_health'
    )
    try:
        last_observed_rtc_health = RtcHealth(last_observed_rtc_health_value)
    except ValueError as exc:
        raise ValueError('CommunicatorStateV2.last_observed_rtc_health' + ' has an unknown enum value') from exc
    reserved_0, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV2.reserved_0'
    )
    if reserved_0 != 0:
        raise ValueError('CommunicatorStateV2.reserved_0' + ' has an invalid constant value')
    if validity_mask & (1 << 0):
        rtc_provenance, offset = _decode_rtc_provenance_v1(view, offset)
    else:
        rtc_provenance_absent, offset = _read_exact(
            view, offset, 48, 'CommunicatorStateV2.rtc_provenance'
        )
        if any(rtc_provenance_absent):
            raise ValueError('CommunicatorStateV2.rtc_provenance' + ' absent representation is not zero')
        rtc_provenance = None
    rolling_window_us, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV2.rolling_window_us'
    )
    tx_airtime_budget_us, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV2.tx_airtime_budget_us'
    )
    entry_charge_us, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV2.entry_charge_us'
    )
    if validity_mask & (1 << 1):
        airtime_snapshot, offset = _decode_airtime_snapshot_v1(view, offset)
    else:
        airtime_snapshot_absent, offset = _read_exact(
            view, offset, 16, 'CommunicatorStateV2.airtime_snapshot'
        )
        if any(airtime_snapshot_absent):
            raise ValueError('CommunicatorStateV2.airtime_snapshot' + ' absent representation is not zero')
        airtime_snapshot = None
    entry_count, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV2.entry_count'
    )
    reserved_2, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV2.reserved_2'
    )
    if reserved_2 != 0:
        raise ValueError('CommunicatorStateV2.reserved_2' + ' has an invalid constant value')
    reserved_3, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV2.reserved_3'
    )
    if reserved_3 != 0:
        raise ValueError('CommunicatorStateV2.reserved_3' + ' has an invalid constant value')
    if entry_count != 18:
        raise ValueError('CommunicatorStateV2.entries' + ' has an invalid fixed length')
    entries_items: list[AirtimeEntryV2] = []
    for _ in range(18):
        item, offset = _decode_airtime_entry_v2(view, offset)
        entries_items.append(item)
    entries = tuple(entries_items)
    if offset != len(view):
        raise ValueError('canonical blob has trailing bytes')
    return CommunicatorStateV2(
        generation=generation,
        last_observed_system_time_quality=last_observed_system_time_quality,
        last_observed_rtc_health=last_observed_rtc_health,
        rtc_provenance=rtc_provenance,
        rolling_window_us=rolling_window_us,
        tx_airtime_budget_us=tx_airtime_budget_us,
        entry_charge_us=entry_charge_us,
        airtime_snapshot=airtime_snapshot,
        entries=entries,
    )

def communicator_state_v2_parameters(entity: CommunicatorStateV2) -> tuple[object, ...]:
    state_blob = encode_communicator_state_v2(entity)
    return (
        1,
        2,
        entity.generation,
        state_blob,
        hashlib.sha256(state_blob).digest(),
    )

# Generated from receiver/schemas/receiver_entities.json by receiver/tools/generate.py.
# Do not edit by hand.
from __future__ import annotations

import hashlib
import struct

from dataclasses import dataclass

from .receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    DiagnosticErrorDomain,
    DiagnosticOperation,
    DiagnosticSeverity,
    PersistenceAdmissionState,
    PersistenceClassification,
    PersistQueueEntityKind,
    ProcessingResult,
    QuarantineFailureReason,
    RadioState,
    RtcHealth,
    SystemTimeQuality,
)

RECEIVER_ENTITY_MANIFEST_SHA256 = '66a0507e495fab9ef737fadffa39e18ae059a07492d5a3dde9809423fdb64189'

__all__ = [
    "RECEIVER_ENTITY_MANIFEST_SHA256",
    "CLOCK_OBSERVATION_V1_TABLE",
    "CLOCK_OBSERVATION_V1_COLUMNS",
    "DIAGNOSTIC_V1_TABLE",
    "DIAGNOSTIC_V1_COLUMNS",
    "QUARANTINED_ENTITY_ROW_V1_TABLE",
    "QUARANTINED_ENTITY_ROW_V1_COLUMNS",
    "RECEIVER_HEALTH_V1_TABLE",
    "RECEIVER_HEALTH_V1_COLUMNS",
    "MESSAGE_PROFILE_ROW_V1_TABLE",
    "MESSAGE_PROFILE_ROW_V1_COLUMNS",
    "READING_MESSAGE_ROW_V1_TABLE",
    "READING_MESSAGE_ROW_V1_COLUMNS",
    "COMMUNICATOR_STATE_V1_TABLE",
    "COMMUNICATOR_STATE_V1_COLUMNS",
    "ClockObservationV1",
    "DiagnosticV1",
    "QuarantinedEntityRowV1",
    "ReceiverHealthV1",
    "MessageProfileRowV1",
    "ReadingMessageRowV1",
    "RtcProvenanceV1",
    "TxAirtimeBucketV1",
    "CommunicatorStateV1",
    "encode_communicator_state_v1",
    "decode_communicator_state_v1",
    "clock_observation_v1_parameters",
    "diagnostic_v1_parameters",
    "quarantined_entity_row_v1_parameters",
    "receiver_health_v1_parameters",
    "message_profile_row_v1_parameters",
    "reading_message_row_v1_parameters",
    "communicator_state_v1_parameters",
]

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

RECEIVER_HEALTH_V1_TABLE = 'receiver_health'

RECEIVER_HEALTH_V1_COLUMNS = (
    'receiver_instance_id',
    'health_sequence',
    'communicator_sampled_at_monotonic_us',
    'radio_state_id',
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
    'batch_encoded_bytes_committed',
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
)

MESSAGE_PROFILE_ROW_V1_TABLE = 'message_profiles'

MESSAGE_PROFILE_ROW_V1_COLUMNS = (
    'receiver_instance_id',
    'occurrence_sequence',
    'received_at_monotonic_us',
    'persist_queue_used_bytes_before_admission',
    'persist_queue_capacity_bytes',
    'received_frame_length',
    'received_frame',
    'claimed_control',
    'claimed_domain',
    'claimed_node_id',
    'claimed_message_id',
    'header_authenticated',
    'decoded_sample_id',
    'rssi_dbm_x2',
    'snr_db_x4',
    'irq_status',
    'device_errors',
    'processing_result_id',
    'ack_selected_id',
    'ack_tx_result_id',
    'ack_frame',
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

READING_MESSAGE_ROW_V1_TABLE = 'reading_messages'

READING_MESSAGE_ROW_V1_COLUMNS = (
    'node_id',
    'message_id',
    'sample_id',
    'reading_body',
    'is_canonical_for_sample',
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
    'first_receiver_instance_id',
    'first_occurrence_sequence',
)

COMMUNICATOR_STATE_V1_TABLE = 'communicator_state'

COMMUNICATOR_STATE_V1_COLUMNS = (
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
class ClockObservationV1:
    receiver_instance_id: bytes
    observation_sequence: int
    clock_state_generation: int
    sampled_at_monotonic_us: int
    sampled_at_utc_us: int | None
    step_discontinuity_boundary: bool
    system_time_quality: SystemTimeQuality
    rtc_health: RtcHealth

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
class ReceiverHealthV1:
    receiver_instance_id: bytes
    health_sequence: int
    communicator_sampled_at_monotonic_us: int
    radio_state: RadioState
    radio_recovery_attempts: int
    radio_recovery_successes: int
    radio_recovery_failures: int
    radio_recovery_attempts_by_reason: tuple[int, ...]
    system_time_quality: SystemTimeQuality
    rtc_health: RtcHealth
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
    batch_encoded_bytes_committed: int
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
    if len(entity.persist_queue_admission_counts) != 5:
        raise ValueError(
            'persist_queue_admission_counts' + ' must have length 5'
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
        entity.radio_state.value,
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
        entity.batch_encoded_bytes_committed,
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
    )

@dataclass(frozen=True, slots=True)
class MessageProfileRowV1:
    receiver_instance_id: bytes
    occurrence_sequence: int
    received_at_monotonic_us: int
    persist_queue_used_bytes_before_admission: int
    persist_queue_capacity_bytes: int
    received_frame_length: int | None
    received_frame: bytes | None
    claimed_control: int | None
    claimed_domain: int | None
    claimed_node_id: bytes | None
    claimed_message_id: int | None
    header_authenticated: bool
    decoded_sample_id: int | None
    rssi_dbm_x2: int | None
    snr_db_x4: int | None
    irq_status: int | None
    device_errors: int | None
    processing_result: ProcessingResult
    ack_selected: AckSelection
    ack_tx_result: AckTxResult
    ack_frame: bytes | None
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
    persistence_classification: PersistenceClassification

def message_profile_row_v1_parameters(entity: MessageProfileRowV1) -> tuple[object, ...]:
    return (
        entity.receiver_instance_id,
        entity.occurrence_sequence,
        entity.received_at_monotonic_us,
        entity.persist_queue_used_bytes_before_admission,
        entity.persist_queue_capacity_bytes,
        entity.received_frame_length,
        entity.received_frame,
        entity.claimed_control,
        entity.claimed_domain,
        entity.claimed_node_id,
        entity.claimed_message_id,
        entity.header_authenticated,
        entity.decoded_sample_id,
        entity.rssi_dbm_x2,
        entity.snr_db_x4,
        entity.irq_status,
        entity.device_errors,
        entity.processing_result.value,
        entity.ack_selected.value,
        entity.ack_tx_result.value,
        entity.ack_frame,
        entity.busy_wait_total_us,
        entity.busy_wait_max_us,
        entity.busy_wait_count,
        entity.busy_timeout_count,
        entity.last_busy_timeout_opcode,
        entity.t1_handler_started_monotonic_us,
        entity.t2_packet_copied_monotonic_us,
        entity.t3_authentication_completed_monotonic_us,
        entity.t4_set_tx_attempted_monotonic_us,
        entity.t5_tx_done_monotonic_us,
        entity.t6_set_rx_issued_monotonic_us,
        entity.persistence_classification.value,
    )

@dataclass(frozen=True, slots=True)
class ReadingMessageRowV1:
    node_id: bytes
    message_id: int
    sample_id: int
    reading_body: bytes
    is_canonical_for_sample: bool
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
    first_receiver_instance_id: bytes
    first_occurrence_sequence: int

def reading_message_row_v1_parameters(entity: ReadingMessageRowV1) -> tuple[object, ...]:
    return (
        entity.node_id,
        entity.message_id,
        entity.sample_id,
        entity.reading_body,
        entity.is_canonical_for_sample,
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
        entity.first_receiver_instance_id,
        entity.first_occurrence_sequence,
    )

@dataclass(frozen=True, slots=True)
class RtcProvenanceV1:
    verified_by_receiver_instance_id: bytes
    network_utc_at_verification_us: int
    rtc_readback_utc_us: int
    verification_uncertainty_us: int
    drift_bound_ppm: int

@dataclass(frozen=True, slots=True)
class TxAirtimeBucketV1:
    charged_airtime_us: int
    expires_at_utc_us: int

@dataclass(frozen=True, slots=True)
class CommunicatorStateV1:
    generation: int
    last_observed_system_time_quality: SystemTimeQuality
    last_observed_rtc_health: RtcHealth
    rtc_provenance: RtcProvenanceV1 | None
    rolling_window_us: int
    tx_airtime_budget_us: int
    bucket_width_us: int
    bucket_charge_limit_us: int
    bucket_expiration_guard_us: int
    airtime_snapshot_utc_us: int
    buckets: tuple[TxAirtimeBucketV1, ...]

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

def _encode_tx_airtime_bucket_v1(value: TxAirtimeBucketV1) -> bytes:
    chunks: list[bytes] = []
    chunks.append(_pack('<Q', value.charged_airtime_us, 'TxAirtimeBucketV1.charged_airtime_us'))
    chunks.append(_pack('<q', value.expires_at_utc_us, 'TxAirtimeBucketV1.expires_at_utc_us'))
    encoded = b''.join(chunks)
    if len(encoded) != 16:
        raise RuntimeError('TX_AIRTIME_BUCKET_V1 generated an invalid size')
    return encoded

def _decode_tx_airtime_bucket_v1(
    blob: memoryview, offset: int
) -> tuple[TxAirtimeBucketV1, int]:
    charged_airtime_us, offset = _unpack(
        blob, offset, '<Q', 'TxAirtimeBucketV1.charged_airtime_us'
    )
    expires_at_utc_us, offset = _unpack(
        blob, offset, '<q', 'TxAirtimeBucketV1.expires_at_utc_us'
    )
    return TxAirtimeBucketV1(
        charged_airtime_us=charged_airtime_us,
        expires_at_utc_us=expires_at_utc_us,
    ), offset

def encode_communicator_state_v1(entity: CommunicatorStateV1) -> bytes:
    if len(entity.buckets) != 62:
        raise ValueError(
            'buckets' + ' must have length 62'
        )
    encoded_length = 1120
    validity_mask = ((1 << 0) if entity.rtc_provenance is not None else 0)
    bucket_count = len(entity.buckets)
    chunks: list[bytes] = []
    chunks.append(_pack('<H', 1, 'CommunicatorStateV1.state_format_version'))
    chunks.append(_pack('<I', encoded_length, 'CommunicatorStateV1.encoded_length'))
    chunks.append(_pack('<Q', entity.generation, 'CommunicatorStateV1.generation'))
    chunks.append(_pack('<H', validity_mask, 'CommunicatorStateV1.validity_mask'))
    chunks.append(_pack('<B', entity.last_observed_system_time_quality.value, 'CommunicatorStateV1.last_observed_system_time_quality'))
    chunks.append(_pack('<B', entity.last_observed_rtc_health.value, 'CommunicatorStateV1.last_observed_rtc_health'))
    chunks.append(_pack('<H', 0, 'CommunicatorStateV1.reserved_0'))
    if entity.rtc_provenance is None:
        chunks.append(bytes(48))
    else:
        chunks.append(_encode_rtc_provenance_v1(entity.rtc_provenance))
    chunks.append(_pack('<Q', entity.rolling_window_us, 'CommunicatorStateV1.rolling_window_us'))
    chunks.append(_pack('<Q', entity.tx_airtime_budget_us, 'CommunicatorStateV1.tx_airtime_budget_us'))
    chunks.append(_pack('<Q', entity.bucket_width_us, 'CommunicatorStateV1.bucket_width_us'))
    chunks.append(_pack('<Q', entity.bucket_charge_limit_us, 'CommunicatorStateV1.bucket_charge_limit_us'))
    chunks.append(_pack('<Q', entity.bucket_expiration_guard_us, 'CommunicatorStateV1.bucket_expiration_guard_us'))
    chunks.append(_pack('<q', entity.airtime_snapshot_utc_us, 'CommunicatorStateV1.airtime_snapshot_utc_us'))
    chunks.append(_pack('<H', bucket_count, 'CommunicatorStateV1.bucket_count'))
    chunks.append(_pack('<H', 0, 'CommunicatorStateV1.reserved_2'))
    chunks.append(_pack('<Q', 0, 'CommunicatorStateV1.reserved_3'))
    for item in entity.buckets:
        chunks.append(_encode_tx_airtime_bucket_v1(item))
    encoded = b''.join(chunks)
    if len(encoded) != encoded_length:
        raise RuntimeError('COMMUNICATOR_STATE_ENCODING_V1 generated an invalid length')
    return encoded

def decode_communicator_state_v1(blob: bytes) -> CommunicatorStateV1:
    try:
        view = memoryview(blob)
    except TypeError as exc:
        raise ValueError('canonical blob must be bytes-like') from exc
    blob = view
    offset = 0
    state_format_version, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV1.state_format_version'
    )
    if state_format_version != 1:
        raise ValueError('CommunicatorStateV1.state_format_version' + ' has an invalid constant value')
    encoded_length, offset = _unpack(
        blob, offset, '<I', 'CommunicatorStateV1.encoded_length'
    )
    if encoded_length != len(view):
        raise ValueError('CommunicatorStateV1.encoded_length' + ' does not match the blob length')
    generation, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV1.generation'
    )
    validity_mask, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV1.validity_mask'
    )
    if validity_mask & ~1:
        raise ValueError('CommunicatorStateV1.validity_mask' + ' has reserved bits set')
    last_observed_system_time_quality_value, offset = _unpack(
        blob, offset, '<B', 'CommunicatorStateV1.last_observed_system_time_quality'
    )
    try:
        last_observed_system_time_quality = SystemTimeQuality(last_observed_system_time_quality_value)
    except ValueError as exc:
        raise ValueError('CommunicatorStateV1.last_observed_system_time_quality' + ' has an unknown enum value') from exc
    last_observed_rtc_health_value, offset = _unpack(
        blob, offset, '<B', 'CommunicatorStateV1.last_observed_rtc_health'
    )
    try:
        last_observed_rtc_health = RtcHealth(last_observed_rtc_health_value)
    except ValueError as exc:
        raise ValueError('CommunicatorStateV1.last_observed_rtc_health' + ' has an unknown enum value') from exc
    reserved_0, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV1.reserved_0'
    )
    if reserved_0 != 0:
        raise ValueError('CommunicatorStateV1.reserved_0' + ' has an invalid constant value')
    if validity_mask & (1 << 0):
        rtc_provenance, offset = _decode_rtc_provenance_v1(view, offset)
    else:
        rtc_provenance_absent, offset = _read_exact(
            view, offset, 48, 'CommunicatorStateV1.rtc_provenance'
        )
        if any(rtc_provenance_absent):
            raise ValueError('CommunicatorStateV1.rtc_provenance' + ' absent representation is not zero')
        rtc_provenance = None
    rolling_window_us, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV1.rolling_window_us'
    )
    tx_airtime_budget_us, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV1.tx_airtime_budget_us'
    )
    bucket_width_us, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV1.bucket_width_us'
    )
    bucket_charge_limit_us, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV1.bucket_charge_limit_us'
    )
    bucket_expiration_guard_us, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV1.bucket_expiration_guard_us'
    )
    airtime_snapshot_utc_us, offset = _unpack(
        blob, offset, '<q', 'CommunicatorStateV1.airtime_snapshot_utc_us'
    )
    bucket_count, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV1.bucket_count'
    )
    reserved_2, offset = _unpack(
        blob, offset, '<H', 'CommunicatorStateV1.reserved_2'
    )
    if reserved_2 != 0:
        raise ValueError('CommunicatorStateV1.reserved_2' + ' has an invalid constant value')
    reserved_3, offset = _unpack(
        blob, offset, '<Q', 'CommunicatorStateV1.reserved_3'
    )
    if reserved_3 != 0:
        raise ValueError('CommunicatorStateV1.reserved_3' + ' has an invalid constant value')
    if bucket_count != 62:
        raise ValueError('CommunicatorStateV1.buckets' + ' has an invalid fixed length')
    buckets_items: list[TxAirtimeBucketV1] = []
    for _ in range(62):
        item, offset = _decode_tx_airtime_bucket_v1(view, offset)
        buckets_items.append(item)
    buckets = tuple(buckets_items)
    if offset != len(view):
        raise ValueError('canonical blob has trailing bytes')
    return CommunicatorStateV1(
        generation=generation,
        last_observed_system_time_quality=last_observed_system_time_quality,
        last_observed_rtc_health=last_observed_rtc_health,
        rtc_provenance=rtc_provenance,
        rolling_window_us=rolling_window_us,
        tx_airtime_budget_us=tx_airtime_budget_us,
        bucket_width_us=bucket_width_us,
        bucket_charge_limit_us=bucket_charge_limit_us,
        bucket_expiration_guard_us=bucket_expiration_guard_us,
        airtime_snapshot_utc_us=airtime_snapshot_utc_us,
        buckets=buckets,
    )

def communicator_state_v1_parameters(entity: CommunicatorStateV1) -> tuple[object, ...]:
    state_blob = encode_communicator_state_v1(entity)
    return (
        1,
        1,
        entity.generation,
        state_blob,
        hashlib.sha256(state_blob).digest(),
    )

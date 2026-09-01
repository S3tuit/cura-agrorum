# Generated from receiver/schemas/receiver_enums.json by receiver/tools/generate.py.
# Entity layout: receiver/schemas/receiver_entities.json.
# Database schema: receiver/db/schema.sql. Do not edit by hand.
from __future__ import annotations

from enum import Enum, unique

RECEIVER_ENUM_MANIFEST_SHA256 = 'fc2067c94c564db57279ee35e78620baa8486471df1e32aae6006789c4c8142a'
RECEIVER_ENTITY_MANIFEST_SHA256 = '66a0507e495fab9ef737fadffa39e18ae059a07492d5a3dde9809423fdb64189'
SQLITE_APPLICATION_ID = 0x43555252
DATABASE_SCHEMA_VERSION = 6
DATABASE_SCHEMA_SHA256 = '72f56f7717d21b1dd99e8ccc89ef2a0961657542de6d70ff311da5f8892bc320'
DATABASE_SCHEMA_FINGERPRINT = bytes.fromhex(DATABASE_SCHEMA_SHA256)

__all__ = [
    "RECEIVER_ENUM_MANIFEST_SHA256",
    "RECEIVER_ENTITY_MANIFEST_SHA256",
    "SQLITE_APPLICATION_ID",
    "DATABASE_SCHEMA_VERSION",
    "DATABASE_SCHEMA_SHA256",
    "DATABASE_SCHEMA_FINGERPRINT",
    "SystemTimeQuality",
    "RtcHealth",
    "RadioState",
    "RadioRecoveryReason",
    "PersistenceAdmissionState",
    "AdmissionResult",
    "PersistQueueEntityKind",
    "ProcessingResult",
    "AckSelection",
    "AckTxResult",
    "PersistenceClassification",
    "QuarantineFailureReason",
    "DiagnosticSeverity",
    "DiagnosticErrorDomain",
    "DiagnosticOperation",
    "RadioDiagnosticErrorCode",
    "RadioDiagnosticContextSchema",
    "RadioBackendStatusKind",
    "RadioFailureStage",
    "RadioCommandOutcome",
    "RadioRecoveryLevelResult",
    "TimeDiagnosticErrorCode",
    "TimeDiagnosticContextSchema",
    "TimeComponent",
    "TimeFailureStage",
    "TimeBackendStatusKind",
    "PersistenceControlDiagnosticErrorCode",
    "PersistenceControlDiagnosticContextSchema",
    "PersistenceControlCommand",
    "PersistenceControlPurpose",
    "PersistenceControlDispositionKind",
    "PersistenceControlDisposition",
    "PersistenceControlFailureKind",
    "PersistenceControlStateCondition",
    "CoreDiagnosticErrorCode",
    "CoreDiagnosticContextSchema",
    "CorePhase",
    "CoreFailureStage",
    "CoreDetailKind",
    "PersistQueueViolationDetailCode",
    "PersistenceControlViolationDetailCode",
]

@unique
class SystemTimeQuality(Enum):
    UNTRUSTED = 0
    RTC_HOLDOVER = 1
    NETWORK_SYNCED = 2

@unique
class RtcHealth(Enum):
    PRESENT = 1
    MISSING = 2
    INVALID = 3

@unique
class RadioState(Enum):
    INITIALIZING = 1
    RX_SINGLE = 2
    RX_EVENT_PENDING = 3
    TX_ACTIVE = 4
    RECOVERING = 5
    SHUTDOWN = 6
    INITIALIZATION_FAILED = 7
    RECOVERY_EXHAUSTED = 8
    HARDWARE_MISSING = 9

@unique
class RadioRecoveryReason(Enum):
    NONE = 0
    BUSY_TIMEOUT = 1
    SPI_FAILURE = 2
    UNEXPECTED_IRQ = 3
    TX_OUTCOME_UNCERTAIN = 4
    RX_PROFILE_RESTORE_FAILED = 5
    SET_RX_FAILED = 6
    STATUS_UNCONFIRMED = 7
    HARDWARE_UNREACHABLE = 8

@unique
class PersistenceAdmissionState(Enum):
    UNAVAILABLE_STARTING = 0
    AVAILABLE = 1
    UNAVAILABLE_LOW_SPACE = 2
    UNAVAILABLE_DISK_FULL = 3
    UNAVAILABLE_CORRUPT = 4
    UNAVAILABLE_IO = 5
    UNAVAILABLE_INCOMPATIBLE_SCHEMA = 6

@unique
class AdmissionResult(Enum):
    RESERVED = 0
    PERSISTENCE_UNAVAILABLE = 1
    QUEUE_FULL = 2

@unique
class PersistQueueEntityKind(Enum):
    MEASUREMENT_PROFILE = 1
    PROFILE_ONLY = 2
    RECEIVER_HEALTH_REQUEST = 3
    DIAGNOSTIC = 4
    CLOCK_OBSERVATION = 5

@unique
class ProcessingResult(Enum):
    RADIO_ERROR = 1
    UNKNOWN_NODE = 2
    AUTHENTICATION_FAILED = 3
    WRONG_DIRECTION = 4
    REJECTED_UNSUPPORTED_CONTROL = 5
    REJECTED_UNSUPPORTED_DOMAIN = 6
    REJECTED_MALFORMED_LENGTH = 7
    REJECTED_MALFORMED_BODY = 8
    RETRY_LATER_QUEUE_FULL = 9
    RETRY_LATER_PERSISTENCE_UNAVAILABLE = 10
    ACCEPTED = 11

@unique
class AckSelection(Enum):
    NONE = 0
    ACCEPTED = 3
    RETRY_LATER = 4
    REJECTED_UNSUPPORTED = 5
    REJECTED_MALFORMED = 6

@unique
class AckTxResult(Enum):
    NOT_APPLICABLE = 1
    SUPPRESSED_AIRTIME_BUDGET = 2
    SET_TX_FAILED = 3
    TX_TIMEOUT = 4
    TX_DONE = 5
    UNKNOWN_INTERRUPTED = 6

@unique
class PersistenceClassification(Enum):
    NOT_APPLICABLE = 0
    FIRST_SEEN = 1
    RETRANSMISSION = 2
    DUPLICATE_SAME_CONTENT = 3
    DUPLICATE_CONFLICT = 4
    MESSAGE_ID_CONFLICT = 5

@unique
class QuarantineFailureReason(Enum):
    UNSUPPORTED_ENTITY_SCHEMA = 1
    ENTITY_DECODING_INVARIANT = 2
    SQL_BINDING_INVARIANT = 3
    SQL_RANGE_VIOLATION = 4
    UNEXPECTED_SQL_CONSTRAINT = 5
    PERSISTENCE_DERIVATION_INVARIANT = 6

@unique
class DiagnosticSeverity(Enum):
    WARN = 1
    ERROR = 2
    FATAL = 3

@unique
class DiagnosticErrorDomain(Enum):
    NONE = 0
    RADIO = 1
    TIME = 2
    PERSISTENCE_CONTROL = 3
    CORE = 4

@unique
class DiagnosticOperation(Enum):
    NONE = 0
    INITIALIZE = 1
    VALIDATE = 2
    READ = 3
    WRITE = 4
    APPEND = 5
    SYNC = 6
    RECOVER = 7
    ENCODE = 8
    DECODE = 9
    TRANSMIT = 10
    RECEIVE = 11
    CLEANUP = 12

@unique
class RadioDiagnosticErrorCode(Enum):
    NONE = 0
    INVALID_ARGUMENT = 1
    INVALID_STATE = 2
    IO = 3
    BUSY_TIMEOUT = 4
    COMMAND_STATUS = 5
    DEADLINE = 6
    UNEXPECTED_IRQ = 7
    DEVICE_ERROR = 8
    MALFORMED_RESPONSE = 9

@unique
class RadioDiagnosticContextSchema(Enum):
    NONE = 0
    RECEIVER_RADIO_EPISODE_CONTEXT_V1 = 1

@unique
class RadioBackendStatusKind(Enum):
    NONE = 0
    ERRNO = 1
    SX1262_DRIVER_STATUS = 2

@unique
class RadioFailureStage(Enum):
    NONE = 0
    STATE_CHECK = 1
    VALIDATE_INPUT = 2
    CONFIGURE_GPIO = 3
    CONFIGURE_SPI = 4
    RESET = 5
    WAKE = 6
    WAIT_BUSY = 7
    WRITE_COMMAND = 8
    READ_COMMAND = 9
    CONFIGURE_IRQ = 10
    WAIT_IRQ = 11
    READ_IRQ = 12
    CLEAR_IRQ = 13
    WRITE_BUFFER = 14
    READ_BUFFER = 15
    READ_PACKET_STATUS = 16
    DETACH_IRQ = 17
    CAPTURE_TIME = 18

@unique
class RadioCommandOutcome(Enum):
    NOT_APPLICABLE = 0
    DEFINITELY_NOT_APPLIED = 1
    CONFIRMED_APPLIED = 2
    UNCERTAIN = 3

@unique
class RadioRecoveryLevelResult(Enum):
    NOT_APPLICABLE = 0
    NOT_ATTEMPTED = 1
    SUCCEEDED = 2
    FAILED = 3

@unique
class TimeDiagnosticErrorCode(Enum):
    NONE = 0
    IO = 1
    DEADLINE = 2
    INVALID_RESPONSE = 3
    COMMAND_REJECTED = 4
    OUTCOME_UNKNOWN = 5
    CLOCK_INTERFERENCE = 6
    RTC_READBACK_MISMATCH = 7
    CALCULATION_RANGE = 8

@unique
class TimeDiagnosticContextSchema(Enum):
    NONE = 0
    RECEIVER_TIME_EPISODE_CONTEXT_V1 = 1

@unique
class TimeComponent(Enum):
    NONE = 0
    CHRONY = 1
    KERNEL_CLOCK = 2
    DS3231 = 3
    RTC_WRITE_HELPER = 4
    TIME_POLICY = 5

@unique
class TimeFailureStage(Enum):
    NONE = 0
    INITIALIZE_ADAPTER = 1
    QUERY_TRACKING = 2
    VALIDATE_TRACKING = 3
    SAMPLE_SYSTEM_CLOCK = 4
    CALCULATE_ERROR_BOUND = 5
    PREPARE_STEP = 6
    SUBMIT_STEP = 7
    WAIT_STABLE_TIME = 8
    READ_RTC = 9
    DERIVE_RTC_WRITE = 10
    EXECUTE_RTC_HELPER = 11
    READ_BACK_RTC = 12
    VERIFY_RTC_READBACK = 13
    VALIDATE_RTC_PROVENANCE = 14
    UPDATE_TIME_STATE = 15
    SCHEDULE_OBSERVATION = 16

@unique
class TimeBackendStatusKind(Enum):
    NONE = 0
    CHRONY_QUERY_STATUS = 1
    CHRONY_STEP_DISPOSITION = 2
    DS3231_READ_STATUS = 3
    DS3231_WRITE_DISPOSITION = 4
    DS3231_FAILURE = 5
    ADJTIMEX_RETURN = 6

@unique
class PersistenceControlDiagnosticErrorCode(Enum):
    NONE = 0
    CONFIGURATION_REJECTED = 1
    HOST_IDENTITY_REJECTED = 2
    STATE_MISSING = 3
    STATE_CORRUPT = 4
    UNSUPPORTED_STATE_VERSION = 5
    STATE_POLICY_MISMATCH = 6
    IO = 7
    DATABASE = 8
    DEADLINE = 9
    CHANNEL_CLOSED = 10

@unique
class PersistenceControlDiagnosticContextSchema(Enum):
    NONE = 0
    RECEIVER_PERSISTENCE_CONTROL_CONTEXT_V1 = 1

@unique
class PersistenceControlCommand(Enum):
    NONE = 0
    LOAD_RECEIVER_CONFIGURATION = 1
    LOAD_COMMUNICATOR_STATE = 2
    COMMIT_COMMUNICATOR_STATE = 3
    COMMIT_RECEIVER_CLEAN_STOP = 4

@unique
class PersistenceControlPurpose(Enum):
    NONE = 0
    STARTUP_CONFIGURATION = 1
    STARTUP_STATE = 2
    AIRTIME_BUCKET_GRANT = 3
    AIRTIME_BUCKET_SETTLEMENT = 4
    RTC_PROVENANCE = 5
    AIRTIME_HISTORY_RECOVERY = 6
    CLEAN_STOP = 7
    RECONCILIATION = 8

@unique
class PersistenceControlDispositionKind(Enum):
    NONE = 0
    COMMUNICATOR_STATE_COMMIT = 1
    RECEIVER_CLEAN_STOP_COMMIT = 2

@unique
class PersistenceControlDisposition(Enum):
    NOT_APPLICABLE = 0
    COMMITTED = 1
    ALREADY_COMMITTED = 2
    DEFINITELY_NOT_COMMITTED = 3
    OUTCOME_UNKNOWN = 4

@unique
class PersistenceControlFailureKind(Enum):
    NONE = 0
    CONFIGURATION_REJECTED = 1
    HOST_IDENTITY_REJECTED = 2
    STATE_UNAVAILABLE = 3
    IO_ERROR = 4
    DATABASE_ERROR = 5
    DEADLINE_EXCEEDED = 6
    CHANNEL_CLOSED = 7

@unique
class PersistenceControlStateCondition(Enum):
    NONE = 0
    MISSING = 1
    CORRUPT = 2
    UNSUPPORTED_VERSION = 3
    POLICY_MISMATCH = 4

@unique
class CoreDiagnosticErrorCode(Enum):
    NONE = 0
    INVALID_ARGUMENT = 1
    INVALID_STATE = 2
    REPRESENTATION_INVARIANT = 3
    PERSIST_QUEUE_CONTRACT = 4
    PERSISTENCE_CONTROL_CONTRACT = 5
    MEMORY_EXHAUSTED = 6
    CODEC_BACKEND = 7
    CRYPTO_BACKEND = 8
    ARITHMETIC_RANGE = 9
    UNEXPECTED_EXCEPTION = 10

@unique
class CoreDiagnosticContextSchema(Enum):
    NONE = 0
    RECEIVER_CORE_FAILURE_CONTEXT_V1 = 1

@unique
class CorePhase(Enum):
    NONE = 0
    STARTUP = 1
    IDLE = 2
    PACKET_PROCESSING = 3
    ACK_PREPARATION = 4
    POST_RESPONSE_FINALIZATION = 5
    PERIODIC_HEALTH = 6
    PERIODIC_TIME = 7
    AIRTIME_STATE = 8
    CONTROL_OPERATION = 9
    SHUTDOWN = 10

@unique
class CoreFailureStage(Enum):
    NONE = 0
    VALIDATE_ARGUMENT = 1
    TRANSITION_STATE = 2
    DERIVE_KEY = 3
    AUTHENTICATE_FRAME = 4
    DECODE_FRAME = 5
    VALIDATE_MESSAGE = 6
    CONSTRUCT_ACK = 7
    ENCRYPT_ACK = 8
    CONSTRUCT_ENTITY = 9
    VALIDATE_ENTITY = 10
    RESERVE_QUEUE = 11
    PUBLISH_QUEUE = 12
    CANCEL_QUEUE = 13
    LOAD_CONFIGURATION = 14
    LOAD_STATE = 15
    COMMIT_STATE = 16
    COMMIT_CLEAN_STOP = 17
    FINALIZE_EXCEPTION = 18
    ALLOCATE_MEMORY = 19
    INVOKE_ADAPTER = 20

@unique
class CoreDetailKind(Enum):
    NONE = 0
    PERSIST_QUEUE_VIOLATION = 1
    PERSISTENCE_CONTROL_VIOLATION = 2

@unique
class PersistQueueViolationDetailCode(Enum):
    INVALID_SPEC = 1
    PRODUCER_CLOSED = 2
    INVALID_TOKEN = 3
    BUFFER_SPEC_MISMATCH = 4
    USE_AFTER_TRANSFER = 5
    DOUBLE_TRANSITION = 6
    INVALID_PUBLICATION_CONTENT = 7

@unique
class PersistenceControlViolationDetailCode(Enum):
    INVALID_ARGUMENT = 1
    INVALID_DEADLINE = 2
    WRONG_CALLER = 3
    INVALID_STATE = 4
    GENERATION_CONTENT_CONFLICT = 5
    STALE_GENERATION = 6
    GENERATION_GAP = 7
    CLEAN_STOP_PRECONDITION = 8
    CLEAN_STOP_CONFLICT = 9

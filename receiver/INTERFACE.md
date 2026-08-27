# Receiver interfaces

Status: provisional pilot contract. This document defines the shared values and
fixed-size `PersistQueue` entities that communicator and persistence
implementations must interpret identically. Names, numeric values and layouts
may be revised during initial implementation. Once deployed, persisted enum
assignments and schema versions are append-only.

The protocol schema and
[`protocol-v2-lora/README.md`](../protocol/protocol-v2-lora/README.md) remain
authoritative for LoRa frame contents. This document defines receiver-local
representations and does not redefine the wire protocol.

## Encoding conventions

Fixed-size queue entities use the following canonical encoding:

- integers are little-endian;
- signed integers use two's-complement representation;
- byte identifiers are opaque byte sequences and are not endian-converted;
- fields appear in the order shown in this document;
- no alignment padding is inserted;
- `bool8` is a `u8` whose only valid values are `0` and `1`; and
- every reserved bit and byte is zero.

Every entity begins with this two-byte envelope:

```text
entity_kind: u8
entity_schema_version: u8
```

The pilot uses `entity_schema_version = 1`. The `(entity_kind,
entity_schema_version)` pair determines the complete encoded size; no
entity-length field is needed.

The implementation may expose frozen typed objects, canonical byte buffers or
both. Queue accounting always uses the exact encoded sizes in this document,
never Python object size or an estimate. A quarantined entity preserves this
exact canonical representation.

## Common scalar values

| Type | Encoding | Meaning |
|---|---:|---|
| `bool8` | `u8` | Boolean; only `0` and `1` are valid |
| `MonotonicUs` | `u64` | Linux monotonic microseconds |
| `UtcUs` | `i64` | POSIX UTC microseconds since the Unix epoch |
| `DurationUs` | `u64` | Non-negative elapsed microseconds |
| `Counter64` | `u64` | Cumulative counter that saturates instead of wrapping |
| `FrameBuffer` | `bytes[255]` | Complete SX1262 payload capacity |
| `AckFrame` | `bytes[23]` | Exact protocol ACK frame |
| `ReadingBody` | `bytes[32]` | Canonical plaintext protocol reading body |
| `DiagnosticContext` | `bytes[128]` | Bounded domain-local diagnostic context |

SQLite represents integers as signed 64-bit values. Every `u64` that is bound
to SQLite must therefore be in `0..INT64_MAX`; crossing that bound is an
interface invariant failure. The receiver terminates or saturates the
applicable counter before it can cross the bound. The queue and canonical
binary encodings remain unsigned and retain their `u64` representation.

## Identities and sequences

| Value | Encoding | Contract |
|---|---:|---|
| `GroupId` | `bytes[8]` | Opaque receiver-group identifier |
| `NodeId` | `bytes[8]` | Opaque provisioned node identifier |
| `MessageId` | `u32` | Transport identity scoped to one node/key lifetime |
| `SampleId` | `u32` | Application-reading identity scoped to one node identity |
| `ReceiverInstanceId` | `bytes[16]` | Random UUIDv4 generated for every receiver-process start |
| `LinuxBootId` | `bytes[16]` | Raw UUID from `/proc/sys/kernel/random/boot_id` |
| `OccurrenceSequence` | `u64` | Per-instance radio-occurrence sequence |
| `HealthSequence` | `u64` | Per-instance health-attempt sequence |
| `DiagnosticSequence` | `u64` | Per-instance communicator-diagnostic sequence |
| `StateGeneration` | `u64` | Durable communicator-state generation |
| `AdmissionGeneration` | `u64` | Persistence-admission publication generation |
| `AirtimeReservationId` | `ReceiverInstanceId + u64` | Owner instance plus per-instance reservation sequence |
| `QueueReservationToken` | opaque process-local `u64` | Unique single-use queue reservation token |

`MessageId` and `SampleId` never wrap or repeat within their protocol-defined
identity lifetimes.

`OccurrenceSequence`, `HealthSequence`, `DiagnosticSequence` and the airtime
reservation sequence start at `0` and advance before the corresponding
attempt. They never wrap. Sequence exhaustion is a terminal failure for the
current receiver instance.

`(receiver_instance_id, occurrence_sequence)` identifies one physical radio
delivery independently of protocol identity. Retransmissions with the same
`NodeId`, `MessageId` and `SampleId` therefore retain distinct profiling rows.
The stable identity also permits idempotent persistence retry and exposes gaps
caused by failed profile admission.

`(receiver_instance_id, diagnostic_sequence)` similarly identifies one
diagnostic before SQLite insertion. Equal domain, operation, code and context
values may describe multiple distinct failures and must not collapse into one
row.

`StateGeneration = 0` means that no authoritative state generation exists.
Explicitly initialized durable state starts at generation `1`.
`AdmissionGeneration` starts at `0` and increments whenever the persistence
thread publishes a different `PersistenceAdmissionState`.

`QueueReservationToken` is never serialized, stored in SQLite or reused after
publication or cancellation.

## Timestamp representations

`MonotonicUs` is meaningful only within one `LinuxBootId`. Monotonic values
from different Linux boots must never be compared or subtracted. Consecutive
reads may be equal but must not move backwards within one Linux boot.

`UtcUs` stores UTC rather than local civil time. A UTC value is canonical only
when its validity bit is set and its accompanying `SystemTimeQuality` is
`RTC_HOLDOVER` or `NETWORK_SYNCED`.

Zero is a valid numeric timestamp and is never an absence sentinel. Optional
timestamps use an explicit validity bit and contain zero when absent.

In `MessageProfilingV1`, `received_at_monotonic_us` is `T0`, the
kernel-recorded DIO1 edge. `T1` through `T6` use the same monotonic clock and
Linux boot. Logical reading timestamps do not cross `PersistQueue`; the
persistence thread derives them from protocol rules and SQLite history.

## Optional-field rules

Every fixed-size entity obeys these rules:

- a validity bitmap is authoritative;
- an absent scalar is zero;
- an absent byte array is entirely zero;
- a bounded byte array has an explicit meaningful length and a zeroed unused
  tail;
- a present zero-valued scalar remains distinguishable from an absent value;
- no string, dictionary, arbitrary exception object, list, float or `None`
  value crosses `PersistQueue`;
- an enum value of zero is invalid unless that enum explicitly assigns it a
  meaning; and
- reserved validity bits remain zero.

Before committing to an ACK outcome, the communicator fills and validates
every pre-TX field in the reserved entity. After that decision, it modifies
only the preallocated terminal-result and timestamp slots. Publication performs
no allocation, variable-size serialization or fallible representation
conversion.

## Time and radio enums

### `SystemTimeQuality`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `UNTRUSTED` | System UTC is not canonical |
| `1` | `RTC_HOLDOVER` | System UTC was bootstrapped from a proven RTC |
| `2` | `NETWORK_SYNCED` | Current time service confirms synchronization |

### `RtcHealth`

| Value | Name | Meaning |
|---:|---|---|
| `1` | `PRESENT` | RTC responds and its time is valid |
| `2` | `MISSING` | RTC is absent or unreachable |
| `3` | `INVALID` | RTC responds but its time is invalid |

`RtcHealth = 0` is invalid.

### `RadioState`

| Value | Name | Meaning |
|---:|---|---|
| `1` | `INITIALIZING` | Bounded initialization is in progress |
| `2` | `RX_SINGLE` | Complete RX profile and `SetRx` are confirmed |
| `3` | `RX_EVENT_PENDING` | A DIO1 event is being handled |
| `4` | `TX_ACTIVE` | `SetTx` is confirmed or its effect is uncertain |
| `5` | `RECOVERING` | Bounded radio recovery is in progress |
| `6` | `SHUTDOWN` | Intentional terminal shutdown |
| `7` | `INITIALIZATION_FAILED` | Bounded initialization failed |
| `8` | `RECOVERY_EXHAUSTED` | Bounded runtime recovery failed |
| `9` | `HARDWARE_MISSING` | Required radio hardware is unavailable |

`RadioState = 0` is invalid.

### `RadioRecoveryReason`

These values index `ReceiverHealthRequestV1` recovery counters.

| Value | Name |
|---:|---|
| `1` | `BUSY_TIMEOUT` |
| `2` | `SPI_FAILURE` |
| `3` | `UNEXPECTED_IRQ` |
| `4` | `TX_OUTCOME_UNCERTAIN` |
| `5` | `RX_PROFILE_RESTORE_FAILED` |
| `6` | `SET_RX_FAILED` |
| `7` | `STATUS_UNCONFIRMED` |
| `8` | `HARDWARE_UNREACHABLE` |

Value `0` means no recovery reason and has no counter slot.

## Persistence and queue enums

### `PersistenceAdmissionState`

This single enum intentionally conflates availability with its unavailable
reason.

| Value | Name | Meaning |
|---:|---|---|
| `0` | `UNAVAILABLE_STARTING` | Startup validation has not completed |
| `1` | `AVAILABLE` | New ordinary queue admission is permitted |
| `2` | `UNAVAILABLE_LOW_SPACE` | Free space is below the configured low-water mark |
| `3` | `UNAVAILABLE_DISK_FULL` | A write failed because storage is full |
| `4` | `UNAVAILABLE_CORRUPT` | SQLite corruption or integrity failure was detected |
| `5` | `UNAVAILABLE_IO` | Another persistence or quarantine I/O failure prevents admission |
| `6` | `UNAVAILABLE_INCOMPATIBLE_SCHEMA` | Schema history, enum catalogue or immutable database metadata is incompatible with this receiver build or configured group |

The initial state is `UNAVAILABLE_STARTING`. A controlled shutdown stops
communicator admission directly and does not require another persistence
state. A failed quarantine write uses `UNAVAILABLE_IO`.

The published snapshot is:

```text
generation: AdmissionGeneration
state: PersistenceAdmissionState
changed_at_monotonic_us: MonotonicUs
```

The persistence thread is the only publisher. A snapshot reports the state
observed at publication; it does not promise that the next SQLite transaction
will succeed.

### `AdmissionResult`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `RESERVED` | Exact queue capacity was reserved while persistence was available |
| `1` | `PERSISTENCE_UNAVAILABLE` | Admission state prevented reservation |
| `2` | `QUEUE_FULL` | Exact capacity was unavailable |

Invalid kinds, sizes, tokens or publication contents are interface violations,
not operational admission results.

### `PersistQueueEntityKind`

| Value | Name | V1 encoded size |
|---:|---|---:|
| `1` | `MEASUREMENT_PROFILE` | 516 bytes |
| `2` | `PROFILE_ONLY` | 467 bytes |
| `3` | `RECEIVER_HEALTH_REQUEST` | 238 bytes |
| `4` | `DIAGNOSTIC` | 182 bytes |

Persistence-batch metrics originate in the persistence thread and do not
cross `PersistQueue`. Receiver lifecycle rows are also written directly by
the persistence thread.

## Packet-processing enums

### `ProcessingResult`

| Value | Name |
|---:|---|
| `1` | `RADIO_ERROR` |
| `2` | `UNKNOWN_NODE` |
| `3` | `AUTHENTICATION_FAILED` |
| `4` | `WRONG_DIRECTION` |
| `5` | `REJECTED_UNSUPPORTED_CONTROL` |
| `6` | `REJECTED_UNSUPPORTED_DOMAIN` |
| `7` | `REJECTED_MALFORMED_LENGTH` |
| `8` | `REJECTED_MALFORMED_BODY` |
| `9` | `RETRY_LATER_QUEUE_FULL` |
| `10` | `RETRY_LATER_PERSISTENCE_UNAVAILABLE` |
| `11` | `ACCEPTED` |

Value `0` is an incomplete builder value and is forbidden at publication.
The two retry-later values normally create a profiling gap because the profile
itself could not be admitted. They remain valid communicator decision and
counter values.

### `AckSelection`

The nonzero values equal the protocol ACK-domain bytes.

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `3` | `ACCEPTED` |
| `4` | `RETRY_LATER` |
| `5` | `REJECTED_UNSUPPORTED` |
| `6` | `REJECTED_MALFORMED` |

### `AckTxResult`

| Value | Name |
|---:|---|
| `1` | `NOT_APPLICABLE` |
| `2` | `SUPPRESSED_AIRTIME_BUDGET` |
| `3` | `SET_TX_FAILED` |
| `4` | `TX_TIMEOUT` |
| `5` | `TX_DONE` |
| `6` | `UNKNOWN_INTERRUPTED` |

Value `0` is permitted only in a private pre-TX builder. Every published
profile contains a terminal value.

### `PersistenceClassification`

The persistence thread derives this value while applying canonical transport
and reading identities. It is not part of a queued `MessageProfilingV1`.

| Value | Name |
|---:|---|
| `0` | `NOT_APPLICABLE` |
| `1` | `FIRST_SEEN` |
| `2` | `RETRANSMISSION` |
| `3` | `DUPLICATE_SAME_CONTENT` |
| `4` | `DUPLICATE_CONFLICT` |
| `5` | `MESSAGE_ID_CONFLICT` |

### `QuarantineFailureReason`

These values describe why an otherwise admitted immutable queue unit was
classified as an item-specific poison. They are persistence provenance, not a
second diagnostic record.

| Value | Name |
|---:|---|
| `1` | `UNSUPPORTED_ENTITY_SCHEMA` |
| `2` | `ENTITY_DECODING_INVARIANT` |
| `3` | `SQL_BINDING_INVARIANT` |
| `4` | `SQL_RANGE_VIOLATION` |
| `5` | `UNEXPECTED_SQL_CONSTRAINT` |
| `6` | `PERSISTENCE_DERIVATION_INVARIANT` |

Disk full, low space, SQLite corruption, general I/O failure, locking,
expected uniqueness conflicts and normal persistence classifications are not
quarantine reasons.

## `MessageProfilingV1`

`MessageProfilingV1` is the 465-byte packet-occurrence payload shared by the
two packet-related queue entities. `received_at_monotonic_us` is `T0`.

Fields in canonical order:

```text
receiver_instance_id: bytes[16]
linux_boot_id: bytes[16]
occurrence_sequence: u64
validity_mask: u64

received_at_monotonic_us: u64
received_at_utc_us: i64
system_time_quality: u8
rtc_health: u8

persist_queue_used_bytes_before_admission: u64
persist_queue_capacity_bytes: u64

received_frame_length: u16
received_frame: bytes[255]

claimed_control: u8
claimed_domain: u8
claimed_node_id: bytes[8]
claimed_message_id: u32
header_authenticated: bool8
decoded_sample_id: u32

rssi_dbm_x2: i16
snr_db_x4: i16
irq_status: u16
device_errors: u16

processing_result: u8
ack_selected: u8
ack_tx_result: u8
ack_frame: bytes[23]

busy_wait_total_us: u64
busy_wait_max_us: u64
busy_wait_count: u32
busy_timeout_count: u32
last_busy_timeout_opcode: u8

t1_handler_started_monotonic_us: u64
t2_packet_copied_monotonic_us: u64
t3_authentication_completed_monotonic_us: u64
t4_set_tx_attempted_monotonic_us: u64
t5_tx_done_monotonic_us: u64
t6_set_rx_issued_monotonic_us: u64
```

The validity-mask assignments are:

| Bit | Field |
|---:|---|
| `0` | `received_at_utc_us` |
| `1` | received frame |
| `2` | claimed control |
| `3` | claimed domain |
| `4` | claimed node ID |
| `5` | claimed message ID |
| `6` | decoded sample ID |
| `7` | RSSI |
| `8` | SNR |
| `9` | IRQ status |
| `10` | device errors |
| `11` | ACK frame |
| `12` | T2 |
| `13` | T3 |
| `14` | T4 |
| `15` | T5 |
| `16` | T6 |
| `17` | last BUSY-timeout opcode |
| `18`–`63` | Reserved; zero |

`T0` and `T1` are mandatory. UTC is valid only for `RTC_HOLDOVER` or
`NETWORK_SYNCED`. RSSI and SNR validity bits must be equal.

When the received-frame bit is clear, `received_frame_length` and every frame
byte are zero. When it is set, the length is from `0` through `255` and every
byte after that length is zero.

Claimed-header validity follows received length:

```text
control valid    only when received_frame_length >= 1
domain valid     only when received_frame_length >= 2
node ID valid    only when received_frame_length >= 10
message ID valid only when received_frame_length >= 14
```

`header_authenticated` may be true only when the complete clear header and CCM
tag were present and authentication succeeded. `decoded_sample_id` may be
valid only after authentication and exact reading-body decoding.

ACK-frame validity is equivalent to `ack_selected != NONE`. When no ACK is
selected, the frame is zero and `ack_tx_result = NOT_APPLICABLE`. When an ACK
is selected, `ack_frame` contains the exact 23 protocol bytes whether TX later
succeeds, fails or is suppressed.

When multiple event timestamps are present, their values must follow the
ordering permitted by the receive-to-ACK flow. Missing error-path timestamps
remain explicitly absent rather than being reconstructed.

`persistence_classification` is deliberately absent. Persistence creates the
stored profiling row by adding its derived classification without mutating the
queued value.

## Fixed-size queue entities

### `MeasurementProfileUnitV1`

Encoded size: 516 bytes.

```text
entity envelope: 2 bytes
measurement candidate: 49 bytes
MessageProfilingV1: 465 bytes
```

The measurement candidate is:

```text
node_id: bytes[8]
message_id: u32
domain: u8
sample_id: u32
reading_body: bytes[32]
```

It represents one authenticated, structurally valid reading and its complete
packet-occurrence profile. Required invariants are:

- `domain` is `CURRENT_READING_UPLINK` or `BACKLOG_READING_UPLINK`;
- `sample_id` equals the sample ID encoded in `reading_body`;
- candidate identity equals the authenticated profiling identity;
- `processing_result = ACCEPTED`;
- `header_authenticated = true`; and
- the received frame is exactly 54 bytes.

The measurement candidate and profile form one queue and SQLite transaction
unit and must never be split.

### `ProfileOnlyUnitV1`

Encoded size: 467 bytes.

```text
entity envelope: 2 bytes
MessageProfilingV1: 465 bytes
```

It represents one complete packet-occurrence profile without an application
candidate. It covers unknown nodes, authentication failures, malformed or
unsupported packets, wrong-direction packets and radio events without a usable
application frame.

The common fixed representation permits frame and identity fields to be absent;
a separate variable-sized radio-event entity is unnecessary.

### `ReceiverHealthRequestV1`

Encoded size: 238 bytes.

This is the immutable communicator-owned portion of a receiver-health row.

```text
receiver_instance_id: bytes[16]
health_sequence: u64
validity_mask: u8
communicator_sampled_at_monotonic_us: u64
radio_state: u8

radio_recovery_attempts: u64
radio_recovery_successes: u64
radio_recovery_failures: u64
radio_recovery_attempts_by_reason: u64[8]

system_time_quality: u8
rtc_health: u8
time_quality_transition_count: u64
rtc_health_transition_count: u64
last_time_quality_transition_monotonic_us: u64
last_rtc_health_transition_monotonic_us: u64

queue_full_admission_rejections: u64
persistence_unavailable_rejections_by_state: u64[7]
message_profiling_admission_failures: u64
failed_receiver_health_enqueue_attempts: u64
```

The recovery array is ordered by `RadioRecoveryReason` values `1` through `8`.
The persistence-state array is indexed by `PersistenceAdmissionState` value;
its `AVAILABLE` element remains zero.

The validity-mask assignments are:

| Bit | Field |
|---:|---|
| `0` | last time-quality transition |
| `1` | last RTC-health transition |
| `2`–`7` | Reserved; zero |

The persistence thread does not mutate this request. It samples its own and
host observations once, creates a separate complete `ReceiverHealth` value and
keeps that enrichment stable across SQLite retry. It adds `linux_boot_id` when
constructing the stored row.

### `ReceiverHealthV1`

`ReceiverHealthV1` is the persistence-created, SQLite-bound value formed from
one immutable `ReceiverHealthRequestV1`. It is not a queue entity. It retains
every communicator-owned request field and adds the following mandatory
fields:

```text
linux_boot_id: bytes[16]
persistence_sampled_at_monotonic_us: u64

persistence_admission_generation: u64
persistence_admission_state: u8
persistence_admission_changed_at_monotonic_us: u64
persistence_admission_transition_counts: u64[7]

durable_quarantine_successes: u64
durable_quarantine_failures: u64

batch_transaction_attempts: u64
batch_transaction_commits: u64
batch_transaction_failures: u64
batch_entities_committed: u64
batch_encoded_bytes_committed: u64
batch_commit_duration_total_us: u64
batch_commit_duration_max_us: u64

wal_checkpoint_attempts: u64
wal_checkpoint_successes: u64
wal_checkpoint_failures: u64
```

The transition-count array is indexed by `PersistenceAdmissionState`,
including `UNAVAILABLE_INCOMPATIBLE_SCHEMA`. All counters are cumulative within
one `receiver_instance_id` and obey the SQLite `INT64_MAX` binding limit.
Batch counters cover ordinary queue-persistence transaction attempts,
including retries and isolation attempts. `batch_entities_committed` and
`batch_encoded_bytes_committed` advance only for queue units whose ordinary
SQLite effects committed; quarantined units advance the quarantine counters
instead. Checkpoint counters cover explicit receiver-initiated checkpoints,
not SQLite's internal page writes.

These host observations are optional and become SQL `NULL` when unavailable:

```text
linux_load_1m_milli: u32 or absent
cpu_temperature_milli_c: i32 or absent
memory_available_bytes: u64 or absent
sqlite_filesystem_available_bytes: u64 or absent
sqlite_database_size_bytes: u64 or absent
sqlite_wal_size_bytes: u64 or absent
ntp_offset_us: i64 or absent
```

`linux_load_1m_milli` is the first Linux load-average value multiplied by
1,000; it is not a percentage. Memory availability is Linux `MemAvailable`.
Filesystem availability is the space available to the receiver service
account. A successfully observed absent WAL file has the valid size zero.
Positive `ntp_offset_us` means Linux system UTC is ahead of the network
reference; time-service adapters normalize their native sign convention.

The persistence thread samples every added field once, freezes the complete
value before attempting its transaction and reuses that exact value across
retry. SQLite uses `NULL`, not numeric zero, for absent stored observations.

### `DiagnosticV1`

Encoded size: 182 bytes.

```text
receiver_instance_id: bytes[16]
diagnostic_sequence: u64

flags: u8
sampled_at_monotonic_us: u64
sampled_at_utc_us: i64
system_time_quality: u8
rtc_health: u8

severity: u8
error_domain: u16
operation: u16
error_code: u16
context_schema: u8
context_length: u8
context: bytes[128]
```

Initial flags:

| Bit | Name |
|---:|---|
| `0` | `SAMPLED_AT_UTC_VALID` |
| `1`–`7` | Reserved; zero |

When UTC is absent, `sampled_at_utc_us` is zero. It may be present only for
`RTC_HOLDOVER` or `NETWORK_SYNCED`.

Diagnostics use the same stable decomposition as firmware:

- `error_domain` identifies the subsystem responsible for the failure;
- `operation` identifies the stable action being attempted, not a function or
  private-helper name;
- `error_code` identifies the domain-specific reason;
- `context_schema` selects an encoding scoped to `error_domain`; and
- `context` identifies affected resources, backends and internal stages under
  that schema.

The value zero is reserved for success in both `error_domain` and `error_code`;
both fields must be nonzero in a diagnostic. Domain and error-code assignments
are append-only after deployment. Their concrete registries are defined
together with the receiver diagnostic-context schemas rather than guessed
before those schemas exist.

`context_schema = 0` if and only if `context_length = 0`.
`context_length` is at most 128 and every unused context byte is zero. Context
must not contain keys, arbitrary object dumps or unrestricted plaintext.
Any diagnostic with nonempty context must use an operation other than `NONE`.

The persistence thread adds `linux_boot_id` when constructing the stored row,
which scopes the monotonic timestamp. Normal protocol outcomes already captured
by `MessageProfilingV1` do not create redundant diagnostics merely to restate
that outcome. Only the communicator creates `DiagnosticV1` values. The
persistence thread may return synchronous control errors for the communicator
to convert into diagnostics, but it never allocates a diagnostic identity or
inserts a persistence-created diagnostic row.

#### `DiagnosticSeverity`

| Value | Name | Meaning |
|---:|---|---|
| `1` | `WARN` | Useful recovered anomaly or degraded condition |
| `2` | `ERROR` | An operation failed or affected data processing |
| `3` | `FATAL` | The receiver instance is entering a terminal state |

Value `0` is invalid. The pilot records no `INFO` diagnostics.

#### Stable diagnostic operations

The receiver reuses the firmware operation assignments. These actions remain
coarse and stable; new values are added only for genuinely new actions.

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `INITIALIZE` |
| `2` | `VALIDATE` |
| `3` | `READ` |
| `4` | `WRITE` |
| `5` | `APPEND` |
| `6` | `REMOVE` |
| `7` | `SYNC` |
| `8` | `RECOVER` |
| `9` | `COMPACT` |
| `10` | `POWER_ON` |
| `11` | `POWER_OFF` |
| `12` | `ENCODE` |
| `13` | `DECODE` |
| `14` | `ENCRYPT` |
| `15` | `DECRYPT` |
| `16` | `TRANSMIT` |
| `17` | `RECEIVE` |
| `18` | `SLEEP` |
| `19` | `CLEANUP` |

`NONE` is valid only when no operation is meaningful. Otherwise a diagnostic
uses the most precise stable action from this table. Backend resource and
private-stage detail belongs in the domain-local context.

## `PersistQueue` contract

### Role and thread ownership

`PersistQueue` is a volatile, bounded, single-producer/single-consumer handoff.
The communicator is its only producer: it reserves, fills, publishes or
cancels entities and closes the producer side during controlled shutdown. The
persistence thread is its only consumer: it publishes persistence-admission
state, claims FIFO batches and acknowledges entities only after durable
handling. Either thread may take an immutable queue snapshot.

`PersistQueue` performs no SQLite, quarantine-table or communicator-state I/O. It
protects capacity, visibility, ordering and ownership only. All compound state
changes use one queue lock; correctness must not depend on the CPython GIL or
on the atomicity of an individual container operation.

### Entity specifications and storage

The Python interface uses an immutable entity specification:

```python
@dataclass(frozen=True, slots=True)
class PersistQueueEntitySpec:
    kind: PersistQueueEntityKind
    schema_version: int
```

For the pilot, every accepted specification has `schema_version = 1` and the
size shown in the `PersistQueueEntityKind` table. The queue derives the exact
encoded size from `(kind, schema_version)`; callers never provide or estimate a
size. An unknown kind, version or pair is an interface error.

A successful reservation preallocates:

- the complete canonical entity buffer;
- a mutable kind-specific entity builder over that buffer; and
- every internal entry and bookkeeping object needed for later publication.

The queue initializes the two-byte entity envelope. The builder may modify the
entity fields but cannot replace its buffer, kind, schema version or encoded
size. It owns no variable-sized data outside the canonical buffer.

The initial implementation may use `collections.deque` for FIFO storage. To
keep publication allocation-free, reservation appends the already allocated
internal entry at the unpublished tail, or provides an equivalent preallocated
slot. An unpublished entry is invisible to batch claims. Because the pilot has
one producer and at most one outstanding reservation, no published entry can
appear behind that unpublished tail entry. Publication changes its visibility;
it does not append a newly allocated queue node.

The configured bounds are:

```text
capacity_bytes
capacity_entities
```

`capacity_bytes` charges the complete canonical encoding, including the entity
envelope. `capacity_entities` separately bounds Python builder, entry and queue
metadata overhead. A reservation must fit both limits. Internal Python object
sizes are not added to byte accounting.

### Persistence-admission publication

The persistence thread updates the queue's admission snapshot through:

```python
publish_admission_state(
    snapshot: PersistenceAdmissionSnapshot,
) -> None
```

The queue validates the snapshot generation and stores the complete immutable
snapshot while holding the same lock used for capacity reservation. The
persistence thread remains the only publisher. Invalid generation ordering is
an interface error and leaves the preceding snapshot unchanged.

The initial snapshot has generation `0` and state `UNAVAILABLE_STARTING`.
Queue closure is separate from persistence admission and does not create
another `PersistenceAdmissionState` value.

### Reservation result

The nonblocking producer operation is:

```python
try_reserve_one(
    spec: PersistQueueEntitySpec,
) -> PersistQueueReserveResult
```

The immutable result contains:

```text
status: AdmissionResult
reservation: PersistQueueReservation or absent
admission_snapshot: PersistenceAdmissionSnapshot
used_bytes_before: u64
used_entities_before: u64
capacity_bytes: u64
capacity_entities: u64
```

`used_bytes_before` and `used_entities_before` are sampled before charging the
requested entity. Together with `admission_snapshot`, they come from the same
critical section as the admission decision. `MessageProfilingV1` uses this
`used_bytes_before` value rather than taking a separate, racy snapshot.

The operation applies checks in this order while holding the queue lock:

1. reject use after producer closure as an interface error;
2. if admission is not `AVAILABLE`, return `PERSISTENCE_UNAVAILABLE` without
   allocating or charging capacity;
3. if either configured capacity would be exceeded, return `QUEUE_FULL`
   without allocating or charging capacity; and
4. allocate and register the complete unpublished entry, charge both limits
   and return `RESERVED` with its reservation handle.

Failure to allocate Python memory does not return `RESERVED` and leaves all
queue counters and contents unchanged. `MemoryError` is a process-level failure,
not an operational `QUEUE_FULL` result.

### Reservation ownership

`PersistQueueReservation` is an opaque, unique and non-copyable handle exposing:

```python
reservation.token
reservation.spec
reservation.entity
reservation.publish() -> None
reservation.cancel() -> None
```

There is deliberately no separate `seal()` operation. While the reservation is
outstanding, its entity is communicator-owned and mutable. The queue does not
know which fields are pre-TX fields, which are terminal fields, or when the
communicator commits to a protocol outcome.

The only queue-level transitions are:

```text
RESERVED -> PUBLISHED
RESERVED -> CANCELLED
```

`publish()` atomically:

- verifies the live token, entity specification, envelope and exact size;
- makes the existing canonical buffer immutable through ownership transfer;
- invalidates all communicator access through the reservation and builder;
- moves the entry from reserved to published accounting without changing total
  used capacity;
- makes the entry visible at the FIFO tail; and
- wakes the persistence thread.

Publication performs no admission-state or capacity check and remains valid if
persistence became unavailable or the producer side was closed after the
reservation succeeded. It performs no allocation, copying or entity
serialization. A correctly used reservation therefore cannot fail due to
queue pressure.

`cancel()` atomically removes the unpublished entry, releases its byte and
entity capacity, invalidates the handle and builder, and wakes the persistence
thread if this completes a closed queue's drain condition. It is forbidden
after publication.

The queue permits either transition and does not understand radio acceptance.
The communicator decides which transition is legal:

- before committing to a response or accepted outcome, it may cancel when the
  enclosing operation is abandoned;
- after an occurrence is accepted or any response is attempted, it must fill
  the best terminal outcome available and publish; and
- a hard process loss may destroy an outstanding volatile reservation under
  the pilot's explicitly non-durable ACK semantics.

A reservation is not an auto-cancelling context manager because automatic
cancellation during exception unwinding could discard an occurrence after an
ACK attempt. The communicator's top-level event finalizer explicitly chooses
publication or cancellation from its own processing state.

Any retained mutable reference or attempted builder use after publication or
cancellation is an interface violation. The persistence thread receives only a
read-only view of the canonical bytes.

### Queue snapshot

```python
snapshot() -> PersistQueueSnapshot
```

The returned immutable snapshot contains:

```text
admission_snapshot
capacity_bytes
capacity_entities
reserved_bytes
reserved_entities
published_bytes
published_entities
claimed_bytes
claimed_entities
closed
closed_and_drained
```

The accounting invariants are:

```text
used_bytes    = reserved_bytes + published_bytes
used_entities = reserved_entities + published_entities

claimed_bytes    <= published_bytes
claimed_entities <= published_entities
```

Claimed entries remain published and are not counted twice. A snapshot is an
observation, not a capacity reservation; callers must never use it for a later
check-then-reserve decision.

### Batch claims

The nonblocking consumer operation is:

```python
claim_batch(
    *,
    max_bytes: int,
    max_entities: int,
) -> PersistQueueBatchLease | None
```

Both limits must be positive, and `max_bytes` must be at least the largest
registered entity size. Invalid limits are configuration errors. If no
published entry is available, the operation returns `None`.

Otherwise it claims the longest complete FIFO prefix that fits both limits.
It never splits an entity or a `MeasurementProfileUnitV1`. A nonempty queue
therefore produces at least one entry. Only one batch lease may exist; an
overlapping claim is an interface error. Reservations and publication may
continue at the tail while a batch is claimed.

The immutable lease contains:

```text
entries: tuple[PersistQueueEntryView, ...]
total_bytes: u64
```

Each entry view exposes its reservation token, specification, encoded size and
a read-only view of its exact canonical buffer. Claiming allocates only
persistence-thread view and tuple metadata; it does not concatenate or copy
the canonical entity bytes. Persistence-derived values such as duplicate
classification and enriched receiver-health fields remain separate work data
and never mutate a claimed entity.

### Durable batch disposition

`PersistQueueBatchDisposition` is process-local and is not encoded in queue
entities:

| Value | Name | Meaning |
|---:|---|---|
| `1` | `SQLITE_COMMITTED` | The entity's required SQLite effects committed successfully |
| `2` | `QUARANTINED` | The exact entity and required failure metadata were durably quarantined |

The batch lease exposes:

```python
batch.acknowledge_durable(
    dispositions: tuple[PersistQueueBatchDisposition, ...],
) -> None

batch.release_for_retry() -> None
```

`acknowledge_durable()` requires exactly one disposition in entry order. The
persistence thread may acknowledge only after every entry has reached the
reported durable outcome. For example, a batch containing valid `X`, poisoned
`Y` and valid `Z` may be removed only after `X` and `Z` commit to SQLite and
`Y` is durably quarantined. Failure of either durable operation retains the
complete batch. Persistence must retain completed per-entry work or make it
idempotent so retry does not invalidate an already durable disposition.

Acknowledgement atomically validates that the lease still names the claimed
queue-head prefix, removes the entire prefix, releases its byte and entity
capacity, invalidates the lease and wakes any shutdown waiter. The queue trusts
the persistence thread's durability assertion; it does not perform I/O to
verify it.

`release_for_retry()` removes nothing. It clears the active claim, invalidates
the lease and leaves every entry in its original FIFO position so persistence
may retry or claim a narrower batch. Exiting a batch-lease context manager
without successful acknowledgement performs `release_for_retry()`.

### Wakeup and controlled closure

The queue is constructed with a shared `threading.Event` used to wake the
persistence thread. Successful publication and closure set this event. The
separate persistence control channel uses the same event, allowing the
persistence loop to wait on queue work, synchronous control operations and
shutdown without polling independent condition variables.

The event is only a wakeup hint. After waking, the persistence thread checks
all work sources and takes a queue snapshot under their respective contracts.
It retains the configured batch threshold and flush deadline; a wakeup does not
require an immediate undersized SQLite transaction. Clearing and rechecking
the shared event must be ordered so that concurrent publication cannot create
a lost wakeup.

```python
close() -> None
```

`close()` is idempotent, prevents new reservations and wakes the persistence
thread. An existing reservation may still publish or cancel. The queue becomes
`closed_and_drained` only when it is closed and has no reserved, published or
claimed entities. Controlled receiver shutdown waits for that condition only
within its configured deadline.

There is no public pop, arbitrary removal, requeue, eviction or priority API.
Every published entity leaves the queue only through durable batch
acknowledgement.

### Interface errors

Invalid entity specifications, stale or foreign tokens, buffer/specification
mismatch, use after publication or cancellation, double publication or
cancellation, overlapping batch claims, disposition-count mismatch and
acknowledgement of a stale queue prefix are implementation invariant failures.
They do not become `AdmissionResult` values and must not trigger another
capacity attempt after a response may have been sent.

## Persistence control channel

### Role and execution model

The persistence control channel is the synchronous interface through which the
communicator asks the persistence thread to load operator configuration, read
or commit communicator state, and write the clean-stop lifecycle marker. It is
independent from the FIFO `PersistQueue`; successful submission says nothing
about durable completion.

The public channel exposes:

```python
load_receiver_configuration(
    *, deadline_monotonic_us: MonotonicUs,
) -> ReceiverConfigurationLoadResult

load_communicator_state(
    *, deadline_monotonic_us: MonotonicUs,
) -> CommunicatorStateLoadResult

commit_communicator_state(
    state: CommunicatorStateV1,
    *, deadline_monotonic_us: MonotonicUs,
) -> CommunicatorStateCommitResult

commit_receiver_clean_stop(
    marker: ReceiverCleanStopV1,
    *, deadline_monotonic_us: MonotonicUs,
) -> ReceiverCleanStopCommitResult
```

Every operation is explicit, blocking and deadline-bounded. The deadline is an
absolute Linux-monotonic timestamp from the current boot. A public method
creates a private immutable command, submits it to the control mailbox, sets
the shared persistence-thread wakeup event and blocks on a private completion
event. No public control command crosses `PersistQueue`.

The persistence thread completes a command only by installing its immutable
result and setting the completion event. It never invokes communicator logic
and exposes no callback that could execute on the persistence thread.

Control commands are serialized in submission order. Persistence services them
before beginning another ordinary SQLite batch once any transaction already in
progress reaches a safe boundary. It never interrupts an open transaction and
must not starve either control work or ordinary FIFO persistence indefinitely.
A command is never sampled, merged or reported complete merely because it
entered the mailbox. The clean-stop command remains available after
`PersistQueue.close()` and closes only with the control channel itself.

Process-local control enums are ordinary Python `Enum` values. Their names and
semantics are part of this interface but have no persisted numeric encoding.
The `operation` field in failure results reuses `DiagnosticOperation` and is
`NONE` on success.

### Common errors

`PersistenceControlInterfaceViolation` has these members:

| Name | Meaning |
|---|---|
| `NONE` | No interface violation |
| `INVALID_ARGUMENT` | An argument has the wrong type, range or internal relationship |
| `INVALID_DEADLINE` | The deadline is not a valid current-boot monotonic value |
| `WRONG_CALLER` | A method was invoked by a thread that does not own that side of the channel |
| `INVALID_STATE` | A submitted communicator state violates its canonical schema or policy invariants |
| `GENERATION_CONTENT_CONFLICT` | The requested generation is installed with different canonical contents |
| `STALE_GENERATION` | The requested generation is older than the installed generation |
| `GENERATION_GAP` | The requested generation is more than one beyond the installed generation |
| `CLEAN_STOP_PRECONDITION` | Queue, radio or communicator-state prerequisites for a clean marker are not satisfied |
| `CLEAN_STOP_CONFLICT` | A different clean-stop marker is already installed for the same receiver instance |

When applicable, failed control results carry:

```text
operation: DiagnosticOperation
interface_violation: PersistenceControlInterfaceViolation
sqlite_primary_code: i32 or absent
sqlite_extended_code: i32 or absent
os_errno: int or absent
```

SQLite result codes use the values reported by the linked SQLite library.
`os_errno` contains the original `OSError.errno` when available. Results never
contain exception objects, tracebacks, unrestricted strings, file contents or
secret configuration. The communicator may convert a failed result into one
bounded `DiagnosticV1`; diagnostic admission remains best effort and never
controls recovery.

### Receiver configuration loading

`receiver-group.json` remains operator-controlled configuration, not SQLite
state. `load_receiver_configuration()` is startup-only and returns the
protocol-owned immutable receiver-group value after the protocol loader
enforces its complete schema and filesystem-safety contract.

`ReceiverConfigurationLoadStatus` has these members:

| Name | Meaning |
|---|---|
| `LOADED` | The immutable protocol configuration is present |
| `CONFIGURATION_REJECTED` | The protocol loader rejected the file or its security properties |
| `HOST_IDENTITY_REJECTED` | Linux boot ID was readable but not one canonical UUID |
| `INTERFACE_VIOLATION` | The caller violated the control interface |
| `OS_ERROR` | A filesystem operation failed |
| `DEADLINE_EXCEEDED` | Loading did not complete by the deadline |
| `CHANNEL_CLOSED` | Controlled shutdown closed the channel before execution |

The result contains:

```text
status: ReceiverConfigurationLoadStatus
operation: DiagnosticOperation
interface_violation: PersistenceControlInterfaceViolation
protocol_rejection: protocol-defined configuration error or absent
os_errno: int or absent
linux_boot_id: bytes[16] or absent
configuration: immutable protocol ReceiverGroupState or absent
```

`configuration` and `linux_boot_id` are present only for `LOADED`. The
persistence thread reads and validates `/proc/sys/kernel/random/boot_id` and
returns that immutable identity so the communicator can place it in fixed-size
profiling entities without filesystem access. Failure to obtain the exact
16-byte UUID is `OS_ERROR` or `HOST_IDENTITY_REJECTED`, as applicable, and
prevents normal radio startup. Configuration load is read-only
and may be repeated without a receiver-side filesystem effect, but the pilot
calls it once before radio operation. There is no runtime configuration write
or hot reload. Rejected configuration always requires operator action and a
receiver restart.

## `CommunicatorStateV1`

### Logical role and SQLite envelope

`CommunicatorStateV1` is the immutable canonical value returned by a successful
state load and accepted by `commit_communicator_state()`. The communicator owns
clock and airtime policy; persistence validates and durably stores the exact
value without editing it.

SQLite holds at most one active state row:

```sql
CREATE TABLE communicator_state (
    singleton_id INTEGER PRIMARY KEY CHECK (singleton_id = 1),
    state_format_version INTEGER NOT NULL
        CHECK (state_format_version BETWEEN 1 AND 65535),
    generation INTEGER NOT NULL
        CHECK (generation BETWEEN 1 AND 9223372036854775807),
    state_blob BLOB NOT NULL,
    state_sha256 BLOB NOT NULL CHECK (length(state_sha256) = 32)
) STRICT;
```

No row represents generation zero. Generation zero is conservative runtime
state and is never inserted. `state_sha256` covers the complete `state_blob`.
The version and generation encoded inside the blob must equal the SQL columns.

The communicator never mutates a loaded value. It creates a complete new
snapshot, normally with `dataclasses.replace`, and advances generation exactly
once.

### Canonical V1 encoding

All values use the common canonical little-endian rules. The fixed header is
128 bytes:

```text
state_format_version: u16
encoded_length: u32
generation: u64

validity_mask: u16
last_observed_system_time_quality: u8
last_observed_rtc_health: u8
airtime_history_state: u8
reserved_0: u8

rtc_verified_by_receiver_instance_id: bytes[16]
rtc_verified_on_linux_boot_id: bytes[16]
network_utc_at_rtc_verification_us: i64
rtc_readback_utc_us: i64

rolling_window_us: u64
tx_airtime_budget_us: u64
bucket_width_us: u64
reservation_max_charge_us: u64
reservation_spend_lifetime_us: u64
reservation_expiration_guard_us: u64

airtime_snapshot_utc_us: i64
bucket_count: u16
reservation_count: u16
```

Validity bit 0 selects the RTC-provenance block; bits 1 through 15 are reserved
and zero. When provenance is absent, its identifiers and timestamps are zero.
The last-observed quality and health are diagnostic snapshots and are never
restored as current-instance observations.

`AirtimeHistoryState` has these persisted values:

| Value | Name | Meaning |
|---:|---|---|
| `0` | `UNKNOWN_EXHAUSTED` | The complete configured TX budget is treated as consumed |
| `1` | `KNOWN` | The following UTC-expiration entries are authoritative |

For `UNKNOWN_EXHAUSTED`, `airtime_snapshot_utc_us`, `bucket_count` and
`reservation_count` are zero and no TX reservation may be opened. This permits
new RTC provenance to become durable while airtime history remains unknown.
It may transition to `KNOWN` with an empty ledger only after the current
receiver instance has suppressed all TX for at least
`rolling_window_us + bucket_width_us + reservation_expiration_guard_us` on
Linux monotonic time and can capture trusted canonical UTC for the new
snapshot. A process restart restarts that waiting interval. No operator claim
that an installation is new bypasses this conservative aging rule.

For `KNOWN`, `airtime_snapshot_utc_us` is canonical UTC captured under
`NETWORK_SYNCED` or valid `RTC_HOLDOVER`. Every stored expiration is later than
that snapshot.

Each nonempty runtime bucket becomes this 16-byte durable entry:

```text
charged_airtime_us: u64
expires_at_utc_us: i64
```

Entries are sorted by strictly increasing expiration. Charges with the same
expiration are combined. The expiration is no earlier than the conservatively
converted bucket end plus the rolling window and configured clock guard. The
runtime ring's monotonic origin, physical start index, zero buckets and cached
`total_used` are not persisted.

Each reservation is 56 bytes:

```text
owner_receiver_instance_id: bytes[16]
reservation_sequence: u64
reserved_airtime_us: u64
reserved_at_utc_us: i64
spend_deadline_utc_us: i64
expires_at_utc_us: i64
```

Reservations are sorted lexicographically by
`(owner_receiver_instance_id, reservation_sequence)`. Identity is unique and
at most one reservation may exist for one owner instance. The reserved charge
is positive and no greater than `reservation_max_charge_us`. Times satisfy:

```text
reserved_at_utc_us <= spend_deadline_utc_us
expires_at_utc_us >=
    spend_deadline_utc_us
    + rolling_window_us
    + bucket_width_us
    + reservation_expiration_guard_us
```

Only a reservation owned by the current `receiver_instance_id` can be
spendable. Earlier-instance reservations are fully charged and unspendable.
The current instance's live monotonic spend deadline is separate runtime state
and is never serialized. After an unknown commit outcome, it may be reused only
when the loaded value exactly matches the originally requested state.

The complete length is:

```text
encoded_length = 128 + bucket_count * 16 + reservation_count * 56
```

The V1 count fields permit the implementation to choose 60, 61, 62 or another
bounded bucket capacity. Implementation limits may be stricter than `u16` and
are validated before encoding and after decoding.

For `KNOWN` state:

```text
sum(bucket charges) + sum(full reservation charges)
    <= tx_airtime_budget_us
```

Every bucket and reservation charge is positive, expired entries are absent,
arithmetic is checked for overflow and all reserved bytes are zero.
`total_used` is recomputed on load. The six stored airtime-policy parameters
must exactly match active deployment policy; mismatch never silently
reinterprets state.

### State-row conditions and loading

`CommunicatorStateCondition` has these members:

| Name | Meaning |
|---|---|
| `NONE` | No state condition |
| `MISSING` | The singleton row is absent; use conservative generation zero |
| `CORRUPT` | SQL envelope, digest, canonical encoding or structural validation failed |
| `UNSUPPORTED_VERSION` | The state value belongs to an unsupported format and is never overwritten automatically |
| `POLICY_MISMATCH` | The stored airtime policy differs from active deployment policy |

`CommunicatorStateLoadStatus` has these members:

| Name | Meaning |
|---|---|
| `LOADED` | A valid immutable state is present |
| `STATE_UNAVAILABLE` | A state-row condition prevented loading |
| `INTERFACE_VIOLATION` | The caller violated the control interface |
| `DATABASE_ERROR` | SQLite or its underlying storage failed |
| `DEADLINE_EXCEEDED` | Loading did not complete by the deadline |
| `CHANNEL_CLOSED` | The channel closed before execution |

`CommunicatorStateLoadResult` contains:

```text
status: CommunicatorStateLoadStatus
operation: DiagnosticOperation
interface_violation: PersistenceControlInterfaceViolation
state_condition: CommunicatorStateCondition
sqlite_primary_code: i32 or absent
sqlite_extended_code: i32 or absent
os_errno: int or absent
state: CommunicatorStateV1 or absent
```

`state` is present if and only if status is `LOADED`. Loading is read-only and
serialized after earlier control commands. A reconciliation load submitted
after an unknown commit observes that commit's terminal database state or
reaches its own deadline.

`MISSING` and `CORRUPT` produce conservative generation-zero runtime state.
A later valid generation-one commit may create a missing row. For a corrupt
row, generation one is permitted only through the atomic preservation and
replacement transaction defined below. Unsupported-version and policy-mismatch
rows suppress TX and are never overwritten automatically. A database error
uses the common SQLite corruption/I/O policy rather than state-row recovery.

### Communicator-state commit

`CommunicatorStateCommitDisposition` has these members:

| Name | Meaning |
|---|---|
| `COMMITTED` | The requested generation and exact canonical contents are durably installed |
| `ALREADY_COMMITTED` | The same generation and exact contents were already installed |
| `NOT_INSTALLED` | The preceding authoritative database state definitely remains in place |
| `OUTCOME_UNKNOWN` | Installation or required durability may have occurred and must be reconciled |

`CommunicatorStateCommitFailureKind` has these members:

| Name | Meaning |
|---|---|
| `NONE` | No failure |
| `INTERFACE_VIOLATION` | The caller or submitted value violated the interface |
| `STATE_UNAVAILABLE` | The installed state condition forbids replacement |
| `DATABASE_ERROR` | SQLite or its underlying storage failed |
| `DEADLINE_EXCEEDED` | The caller deadline expired |
| `CHANNEL_CLOSED` | Controlled shutdown prevented execution |

The result contains:

```text
disposition: CommunicatorStateCommitDisposition
failure_kind: CommunicatorStateCommitFailureKind
operation: DiagnosticOperation
interface_violation: PersistenceControlInterfaceViolation
state_condition: CommunicatorStateCondition
sqlite_primary_code: i32 or absent
sqlite_extended_code: i32 or absent
os_errno: int or absent
```

Generation handling is deterministic:

- installed generation and exact blob equal the request:
  `ALREADY_COMMITTED`;
- equal generation but different contents:
  `NOT_INSTALLED + GENERATION_CONTENT_CONFLICT`;
- requested generation is installed generation plus one: attempt commit;
- requested generation is older: `NOT_INSTALLED + STALE_GENERATION`; and
- requested generation skips ahead: `NOT_INSTALLED + GENERATION_GAP`.

A missing baseline accepts only a valid conservative generation-one value. A
corrupt baseline accepts generation one only in one SQLite transaction that:

1. inserts one append-only `quarantined_communicator_states` row for every
   observed row in the invalid `communicator_state` relation, preserving every
   exact rejected SQL value;
2. removes the invalid relation contents and inserts the requested valid
   generation-one singleton; and
3. commits both effects atomically.

Unsupported-version and policy-mismatch baselines cannot be replaced
automatically.

Ordinary next-generation commit uses one `BEGIN IMMEDIATE` transaction,
validates the installed row and requested canonical value, updates the
singleton and commits under WAL with `synchronous=FULL`. An error or deadline
known to precede `COMMIT` returns `NOT_INSTALLED`. Once `COMMIT` may have been
issued, lack of confirmation returns `OUTCOME_UNKNOWN`. Caller timeout never
cancels possibly committed work. A later serialized load reconciles the exact
installed generation and bytes.

## Receiver clean-stop control

Process bootstrap creates one immutable `ReceiverInstanceStartV1` before either
worker begins normal operation and supplies it to the persistence-thread
constructor:

```text
receiver_instance_id: bytes[16]
started_at_monotonic_us: u64
started_at_utc_us: i64 or absent
started_system_time_quality: SystemTimeQuality
started_rtc_health: RtcHealth or absent
```

The start value is not a `PersistQueue` entity or a public control-channel
request. The persistence thread combines it with the `linux_boot_id` it reads,
inserts the `receiver_instances` row after database validation and before
publishing ordinary admission as available, and refuses to operate if that
identity already exists. Start UTC follows the normal time-quality rule. RTC
health may be absent because process identity is created before the receiver's
current-instance RTC probe completes.

`ReceiverCleanStopV1` is immutable control data:

```text
receiver_instance_id: bytes[16]
stopped_at_monotonic_us: u64
stopped_at_utc_us: i64 or absent
system_time_quality: SystemTimeQuality
rtc_health: RtcHealth
communicator_state_generation: u64
```

UTC is absent for `UNTRUSTED` and present for `RTC_HOLDOVER` or
`NETWORK_SYNCED`. Generation zero is permitted when conservative generation
zero is the known authoritative state.

The communicator calls `commit_receiver_clean_stop()` only after:

- new radio admission and TX have stopped;
- the radio reached its safe shutdown state;
- `PersistQueue` is closed and drained;
- airtime settlement has a known outcome; and
- the supplied communicator-state generation is confirmed authoritative.

Persistence verifies the current receiver-instance row, the closed-and-drained
queue and either the installed state generation or the still-current
generation-zero state condition. It can validate queue and database conditions
directly; radio-state and communicator-policy preconditions remain caller
invariants.

`ReceiverCleanStopCommitDisposition` has these members:

| Name | Meaning |
|---|---|
| `COMMITTED` | The complete marker is durably installed |
| `ALREADY_COMMITTED` | The identical marker was already installed |
| `NOT_COMMITTED` | The marker definitely was not installed |
| `OUTCOME_UNKNOWN` | `COMMIT` may have installed it; repeat the exact request to reconcile |

`ReceiverCleanStopCommitFailureKind` has these members:

| Name | Meaning |
|---|---|
| `NONE` | No failure |
| `INTERFACE_VIOLATION` | The caller, marker or shutdown precondition violated the interface |
| `DATABASE_ERROR` | SQLite or its underlying storage failed |
| `DEADLINE_EXCEEDED` | The caller deadline expired |
| `CHANNEL_CLOSED` | The control channel closed before execution |

The result contains:

```text
disposition: ReceiverCleanStopCommitDisposition
failure_kind: ReceiverCleanStopCommitFailureKind
operation: DiagnosticOperation
interface_violation: PersistenceControlInterfaceViolation
sqlite_primary_code: i32 or absent
sqlite_extended_code: i32 or absent
os_errno: int or absent
```

A different installed marker produces
`NOT_COMMITTED + INTERFACE_VIOLATION + CLEAN_STOP_CONFLICT`. An unsatisfied
prerequisite produces
`NOT_COMMITTED + INTERFACE_VIOLATION + CLEAN_STOP_PRECONDITION`. A known
pre-`COMMIT` deadline returns `NOT_COMMITTED + DEADLINE_EXCEEDED`; after
`COMMIT` may have run it returns `OUTCOME_UNKNOWN + DEADLINE_EXCEEDED`.
Repeating the exact request is idempotent.

## SQLite schema contract

### Migration layout and execution

Database migrations live at:

```text
receiver/db/
├── README.md
└── migrations/
    ├── 0001_initial.sql
    ├── 0002_<description>.sql
    └── ...
```

There is one immutable forward-only SQL migration for each database schema
change, not for every software release. A new database applies every migration
in filename order. Applied files are never edited, reordered, deleted or
reused, and the pilot has no downgrade migrations or automatic retention
deletion.

`schema_migrations` is authoritative:

```text
version: INTEGER PRIMARY KEY
filename: TEXT UNIQUE
sha256: BLOB[32]
```

SQLite `application_id` is `0x43555252` (`CURR`, decimal `1129665106`) for this
receiver database and `user_version` mirrors the latest applied version. Each
migration and its ledger insert commit atomically in a transaction owned by the
migration runner. Migration files do not contain transaction boundaries,
`journal_mode` or `synchronous` settings.

A missing version, changed migration digest, modified enum catalogue, database
newer than the binary, unknown schema gap or mismatched immutable
`database_metadata.group_id` publishes
`UNAVAILABLE_INCOMPATIBLE_SCHEMA` and is not modified automatically. Structural
SQLite corruption instead publishes `UNAVAILABLE_CORRUPT` and follows the
whole-database preservation policy. Migration completes before ordinary
persistence admission becomes `AVAILABLE`. Large future table rebuilds require
explicit maintenance rather than an unexpected receiver-startup migration.

Migration resources are resolved relative to the installed receiver package,
never the process working directory.

### Generated enum catalogues

Persisted enums use one strict SQLite reference table per enum:

```text
id: INTEGER PRIMARY KEY
code: TEXT UNIQUE
```

The uppercase `code` exactly equals the Python enum member name. Event tables
store numeric IDs with foreign keys; analysis views may expose joined code
columns. Reference rows are append-only, with update and delete forbidden.
`PRAGMA foreign_keys=ON` is set and verified on every connection.

Before implementation generates persisted enum code or the initial migration,
it must add `receiver/schemas/receiver_interface.json` as the receiver-local
enum source of truth. Each value records its name, numeric assignment, whether
it is persisted and the database version that introduced it. Generation
produces:

- static Python `Enum` classes;
- enum insert blocks embedded in the applicable checked-in migration;
- the expected catalogue used by startup validation; and
- freshness-test artifacts.

Already introduced assignments cannot change or disappear. Python never loads
its enum definitions dynamically from SQLite because queue decoding, radio
handling and persistence-unavailable behavior must work without a usable
database. Protocol-owned enum assignments continue to come from the protocol
schema and are consumed rather than redeclared.

### Core tables and identities

The initial database contains:

| Table | Canonical identity and role |
|---|---|
| `database_metadata` | Singleton binding to non-secret `group_id`; no master key |
| `schema_migrations` | Applied migration versions and digests |
| `receiver_instances` | Database-local instance order and process lifecycle |
| `communicator_state` | Singleton authoritative clock/airtime state |
| `quarantined_communicator_states` | Exact invalid state rows preserved before conservative replacement |
| `message_profiles` | `(receiver_instance_id, occurrence_sequence)`; every admitted packet occurrence |
| `transport_messages` | `(node_id, message_id)`; first canonical authenticated frame/domain/sample |
| `readings` | `(node_id, sample_id)`; first canonical reading body and decoded measurement columns |
| `reading_timestamps` | `(node_id, sample_id)`; immutable logical timestamp assignment |
| `diagnostics` | `(receiver_instance_id, diagnostic_sequence)`; communicator-created diagnostics only |
| `receiver_health` | `(receiver_instance_id, health_sequence)`; complete enriched `ReceiverHealthV1` |
| `quarantined_entities` | Exact canonical bytes and minimal failure provenance for poisoned queue units |

`database_metadata` permanently binds one database to one eight-byte
`group_id`. It never contains `group_master_key`, derived node keys or other
secret provisioning material.

Tables use `STRICT` mode, fixed-length BLOB checks, foreign keys and explicit
range constraints. Composite canonical tables should use `WITHOUT ROWID`.
Optional stored values are SQL `NULL`, never numeric sentinels. No canonical
insert uses `INSERT OR REPLACE`.

`message_profiles` stores all `MessageProfilingV1` fields with absent fields
mapped to `NULL`, plus the derived `persistence_classification`.
`transport_messages` stores the first exact authenticated frame, domain,
decoded sample ID and first occurrence identity. `readings` retains the exact
32-byte canonical reading body for equality and also stores its decoded
protocol fields for analysis.

`reading_timestamps` contains:

```text
node_id: bytes[8]
sample_id: u32
logical_utc_us: i64
timestamp_source: DIRECT or EXTRAPOLATED
anchor_sample_id: u32
```

Its primary key makes timestamp assignment append-only. A conflicting second
assignment is an invariant failure rather than an update.

Duplicate and identity conflicts create no `DiagnosticV1`. Their evidence is
the occurrence profile's classification together with immutable canonical
transport and reading rows. An analysis view may join those tables to expose
conflicts directly.

### Receiver instances

`receiver_instances` uses this logical SQL contract:

```sql
CREATE TABLE receiver_instances (
    instance_ordinal INTEGER PRIMARY KEY,

    receiver_instance_id BLOB NOT NULL UNIQUE
        CHECK (length(receiver_instance_id) = 16),
    linux_boot_id BLOB NOT NULL
        CHECK (length(linux_boot_id) = 16),

    started_at_monotonic_us INTEGER NOT NULL
        CHECK (started_at_monotonic_us >= 0),
    started_at_utc_us INTEGER,
    started_system_time_quality_id INTEGER NOT NULL
        REFERENCES system_time_quality_codes(id),
    started_rtc_health_id INTEGER
        REFERENCES rtc_health_codes(id),

    clean_stopped_at_monotonic_us INTEGER,
    clean_stopped_at_utc_us INTEGER,
    clean_stop_system_time_quality_id INTEGER
        REFERENCES system_time_quality_codes(id),
    clean_stop_rtc_health_id INTEGER
        REFERENCES rtc_health_codes(id),
    clean_stop_state_generation INTEGER
        CHECK (
            clean_stop_state_generation IS NULL
            OR clean_stop_state_generation >= 0
        )
) STRICT;
```

`instance_ordinal` is database-local analysis order and is never reused because
lifecycle rows are never deleted. `receiver_instance_id` remains the public
identity referenced by other tables.

The row is inserted as soon as SQLite is usable and before ordinary persistence
admission becomes available. `started_at_monotonic_us` is captured when the
receiver instance ID is created. Start UTC is present only if time was trusted
then; start RTC health may be absent if the RTC had not been probed.

There is no separate clean-stop Boolean. Presence of
`clean_stopped_at_monotonic_us` is the marker. When absent, every other
clean-stop field is absent. When present, stop quality, health and state
generation are present; stop UTC follows the normal time-quality rule and stop
monotonic time is not earlier than start. Start fields are immutable. The only
permitted update is one transition from no marker to one complete marker;
triggers reject later mutation or removal.

The preceding durable instance is the row with the greatest lower
`instance_ordinal`. Absence of its marker establishes only that controlled
shutdown was not durably confirmed.

### Poisoned queue units

A poisoned unit is an admitted, representation-valid immutable queue unit that
reproducibly fails persistence when isolated because of an entity-specific
decoder, binding, derivation, range or unexpected schema-constraint defect. It
is not malformed radio input, a duplicate classification, disk full, database
corruption, locking or a transient/global I/O failure.

Persistence rolls back the failed batch, excludes global and transient causes,
narrows the batch and retries the suspected unit alone. Only reproduction of
the item-specific failure permits quarantine. A
`MeasurementProfileUnitV1` is always quarantined as one complete 516-byte unit.

SQLite contains one generic append-only table:

```text
quarantine_id: bytes[32]
entity_kind: u8
entity_schema_version: u8
entity_length: u32
entity_bytes: bytes[entity_length]

receiver_instance_id: bytes[16]
linux_boot_id: bytes[16]
quarantined_at_monotonic_us: u64

database_schema_version: u32
failure_reason: QuarantineFailureReason
failure_operation: DiagnosticOperation
sqlite_primary_code: i32 or absent
sqlite_extended_code: i32 or absent
os_errno: i32 or absent
isolation_attempt_count: u32
```

`quarantine_id` is SHA-256 of the exact canonical entity bytes and is the
primary key. The explicit kind, version and length duplicate the entity
envelope so quarantine can be indexed and validated without decoding the
problematic payload. `length(entity_bytes)` must equal `entity_length`.
`receiver_instance_id` and `linux_boot_id` come from the immutable process
startup context, not by decoding the suspected entity.

After the normal transaction rolls back, persistence inserts the quarantine
row in a separate WAL/`FULL` transaction. A primary-key conflict is successful
idempotent recovery only when every existing column exactly matches the frozen
quarantine row from the ambiguous attempt. Otherwise it is a
database/invariant failure. Rows are never updated or deleted; triggers protect
the append-only contract.

Only after the quarantine transaction commits may the batch disposition be
`QUARANTINED`. An ambiguous commit is reconciled by selecting and comparing the
row. Failure to make quarantine durable retains the complete queue batch,
publishes `UNAVAILABLE_IO` and closes new ordinary admission.

Quarantine carries minimal failure provenance because asynchronous persistence
cannot return an entity-specific error to the communicator and therefore no
corresponding `DiagnosticV1` is guaranteed. It contains no arbitrary exception
text, severity, diagnostic sequence or unrestricted context.

### Preserved corrupt communicator state

`quarantined_communicator_states` uses this logical SQL contract:

```sql
CREATE TABLE quarantined_communicator_states (
    quarantined_state_id INTEGER PRIMARY KEY,

    observed_singleton_id ANY,
    observed_state_format_version ANY,
    observed_generation ANY,
    observed_state_blob ANY,
    observed_state_sha256 ANY,
    calculated_blob_sha256 BLOB
        CHECK (
            calculated_blob_sha256 IS NULL
            OR length(calculated_blob_sha256) = 32
        ),

    preserved_by_receiver_instance_id BLOB NOT NULL
        REFERENCES receiver_instances(receiver_instance_id)
        CHECK (length(preserved_by_receiver_instance_id) = 16),
    linux_boot_id BLOB NOT NULL
        CHECK (length(linux_boot_id) = 16),
    preserved_at_monotonic_us INTEGER NOT NULL
        CHECK (preserved_at_monotonic_us >= 0),
    database_schema_version INTEGER NOT NULL
        CHECK (database_schema_version > 0)
) STRICT;
```

The five `ANY` columns preserve the exact SQLite storage class and value seen
in each rejected row, including an invalid type or `NULL`; STRICT-table `ANY`
performs no numeric coercion. `calculated_blob_sha256` is present only when the
observed state blob can be hashed as a BLOB. Each archive row is
append-only and is inserted in the same transaction that replaces the corrupt
singleton, so a separate archive idempotency token is unnecessary: after an
ambiguous successful commit, reconciliation sees the valid replacement rather
than the corrupt baseline.

This table is used only when SQLite itself is structurally healthy but
`CommunicatorStateV1` validation fails. SQLite corruption preserves the entire
database, WAL and shared-memory set under the database-corruption policy
instead. Unsupported-version and policy-mismatch rows are not automatically
moved.

### Receiver lifecycle and record ownership

`diagnostics` accepts only communicator-created `DiagnosticV1` rows and uses
`(receiver_instance_id, diagnostic_sequence)` as its idempotent key.
Persistence-control failures may be returned to the communicator and converted
there. Asynchronous SQLite failures are represented by admission state,
`ReceiverHealthV1` counters, quarantine provenance when applicable and bounded
service logging.

`receiver_health` expands enum-indexed arrays into named numeric columns for
analysis and stores optional host observations as `NULL`. Persistence creates
the complete immutable row once and never resamples it across transaction
retry.

No `persistence_batches` or `receiver_state_operations` tables exist in the
pilot. Batch throughput, failure, byte and checkpoint aggregates are carried by
`ReceiverHealthV1`. Communicator-state control failures use `DiagnosticV1` when
the communicator can admit one; authoritative state remains only in
`communicator_state`.

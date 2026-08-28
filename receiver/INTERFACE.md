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

The complete diagnostic entity, common diagnostic enums and domain-local
catalogues are defined separately in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md). Numeric assignments in
that document are part of this interface.

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
| `ClockObservationSequence` | `u64` | Per-instance clock-observation sequence |
| `ClockStateGeneration` | `u64` | Per-instance live time/RTC-state generation |
| `HealthSequence` | `u64` | Per-instance health-attempt sequence |
| `DiagnosticSequence` | `u64` | Per-instance communicator-diagnostic sequence |
| `StateGeneration` | `u64` | Durable communicator-state generation |
| `AdmissionGeneration` | `u64` | Persistence-admission publication generation |
| `AirtimeReservationId` | `ReceiverInstanceId + u64` | Owner instance plus per-instance reservation sequence |
| `QueueReservationToken` | opaque process-local `u64` | Unique single-use queue reservation token |

`MessageId` and `SampleId` never wrap or repeat within their protocol-defined
identity lifetimes.

`OccurrenceSequence`, `ClockObservationSequence`, `HealthSequence`,
`DiagnosticSequence` and the airtime reservation sequence start at `0` and
advance before the corresponding attempt. They never wrap. Sequence exhaustion
is a terminal failure for the current receiver instance.

`ClockStateGeneration` starts at `0` for each receiver instance and advances
whenever the communicator changes `SystemTimeQuality` or `RtcHealth`, and
before every intentional clock-step boundary even when quality is already
`UNTRUSTED`. Periodic observations do not advance it. It is runtime observation
provenance, not the durable `StateGeneration` used by
`CommunicatorStateV1`.
One atomic transition that changes both quality and health advances the
generation once.

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
reads may be equal but must not move backwards within one Linux boot. Linux
applies chrony's incremental frequency corrections to `CLOCK_MONOTONIC`; it
therefore remains monotonic but its rate is not assumed to equal physical
elapsed time exactly. The pilot does not use `CLOCK_MONOTONIC_RAW`.

`UtcUs` stores UTC rather than local civil time. A clock-observation UTC is
canonical only when its validity bit is set and that observation's
`SystemTimeQuality` is `RTC_HOLDOVER` or `NETWORK_SYNCED`. An event UTC is
canonical only when analysis has derived it from such an observation in the
same `LinuxBootId` and returns the value together with the source-observation
identity. Receiver event and lifecycle rows are not updated with that result.

Zero is a valid numeric timestamp and is never an absence sentinel. Optional
timestamps use an explicit validity bit and contain zero when absent.

In `MessageProfilingV1`, `received_at_monotonic_us` is `T0`, the
kernel-recorded DIO1 edge. `T1` through `T6` use the same monotonic clock and
Linux boot. Profiles, diagnostics and receiver lifecycle controls carry no UTC
sample or per-event clock quality. Analysis derives their optional UTC from
`ClockObservationV1` without updating their stored rows. Logical reading
timestamps do not cross `PersistQueue`; analysis derives them from protocol
rules and immutable SQLite history.

## Elapsed-duration policy

Policy durations such as one hour, 30 seconds or one minute denote physical
elapsed time even when their encoded type is `DurationUs`. Safety-sensitive
uses convert them to monotonic duration with one centralized checked-integer
implementation. Let:

```text
P = 1_000_000
R = monotonic_elapsed_rate_bound_ppm

minimum_wait_monotonic_us(D) = ceil(D * (P + R) / P)
maximum_lifetime_monotonic_us(D) = floor(D * (P - R) / P)
```

`0 <= R < P`; multiplication and addition are checked before evaluation.
`minimum_wait_monotonic_us()` is used when acting too early is unsafe,
including rolling-airtime retention, unknown-history aging and minimum retry
backoff. `maximum_lifetime_monotonic_us()` is used when acting too late is
unsafe, including radio/time-service operation deadlines and the spendable
life of an airtime reservation. The caller constructs an absolute monotonic
deadline by checked addition of the applicable converted duration to a current
boot monotonic sample. Profiling, health and periodic clock-observation
intervals are observational and use their nominal monotonic duration unless
their contract explicitly requires a safety conversion.

The pilot deployment defaults are:

```text
chrony_max_slew_rate_ppm = 3_500
monotonic_elapsed_rate_bound_ppm = 3_700
network_trust_error_threshold_us = 35_000_000
network_step_error_threshold_us = 40_000_000
clock_observation_period_us = 10_800_000_000
```

`chrony_max_slew_rate_ppm` is the receiver's declared expectation; the tracking
adapter cannot discover the installed daemon value. Deployment validation must
separately verify that chronyd uses that exact ceiling. Receiver configuration
is valid only when
`chrony_max_slew_rate_ppm <= monotonic_elapsed_rate_bound_ppm < P` and
`network_trust_error_threshold_us < network_step_error_threshold_us`. The
configured RTC-holdover age/drift/bootstrap uncertainty budget must not exceed
`network_step_error_threshold_us`, preserving the same 40-second maximum
trusted receiver-UTC error for direct protocol anchors. The
3,700 ppm receiver bound intentionally includes margin above chrony's 3,500
ppm phase-correction ceiling. It yields these normative examples:

```text
minimum_wait_monotonic_us(3_600_000_000) = 3_613_320_000
maximum_lifetime_monotonic_us(30_000_000) = 29_889_000
minimum_wait_monotonic_us(10_800_000_000) = 10_839_960_000
```

The last result shows the maximum 39.96-second monotonic-rate allowance over
three physical hours. The three-hour `ClockObservationV1` period is a
provenance and recovery interval, not an accuracy proof. These values are
configurable pilot defaults and must be revised together with deployment
chrony configuration and the receiver's startup validation.

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

## Chrony control interface

The pilot isolates all chronyc-specific behavior behind one runtime adapter.
It is not a persistence-control interface and none of its values cross
`PersistQueue` or enter SQLite directly:

```python
class ChronyControl(Protocol):
    def read_tracking(
        self,
        *,
        deadline_monotonic_us: MonotonicUs,
    ) -> ChronyTrackingResult: ...

    def apply_pending_correction_by_step(
        self,
        *,
        deadline_monotonic_us: MonotonicUs,
    ) -> ChronyStepResult: ...
```

Both methods are synchronous but have a short absolute monotonic deadline and
are called only outside the RX-to-ACK critical path. The implementation invokes
`chronyc` directly without a shell, with a fixed argument vector and the local
Unix command socket explicitly supplied to chronyc; it permits no localhost
UDP fallback. It never accepts a caller-supplied command, host, source, UTC
value, offset or chronyc option. A future local coordinator can implement the
same Python protocol. The adapter does not expose or invoke `chronyc
waitsync`; the communicator schedules bounded `read_tracking()` polls between
radio deadlines.

The proposed pilot subprocess backend uses a validated deployment socket path,
`LC_ALL=C` and these exact argument shapes:

```text
chronyc -n -c -h <socket-path> tracking
chronyc -n -c -h <socket-path> makestep
```

`<socket-path>` is immutable adapter construction data from deployment
configuration, not a per-call argument. The backend pins and startup-checks a
supported chronyc output version before time quality can become
`NETWORK_SYNCED`.

`read_tracking()` executes the fixed read-only equivalent of `chronyc
tracking`, validates the complete response from a deployment-supported chrony
version and normalizes it into integer fields:

```text
status: ChronyQueryStatus
sample_started_at_monotonic_us: u64
sample_finished_at_monotonic_us: u64

source_selected: bool
synchronized: bool
remaining_correction_us: i64
root_distance_us: u64
estimated_skew_ppb: u64
```

The normalized fields are valid only when `status = OK`. `synchronized`
requires chrony's tracking result to report a selected usable source and a
normal synchronized leap state; the communicator additionally applies its
configured total-error, skew and freshness bounds. This result is
policy input, not by itself permission to label a clock observation
`NETWORK_SYNCED`. A trusted observation must also pass the read-only
`adjtimex()` sampling contract.

`remaining_correction_us` is the signed correction still to be applied to Linux
system UTC: positive means the clock needs to advance and negative means its
progress needs to be retarded by slew. The backend derives
`root_distance_us` from the tracking response as:

```text
root_distance_us = ceil(root_delay_us / 2 + root_dispersion_us)
```

The operands and result must be finite and non-negative, and the final value is
rounded upward to an integer microsecond. `estimated_skew_ppb` is also a
non-negative conservative integer conversion; chrony's skew in ppm is
multiplied by 1,000 and rounded upward. Conversion overflow or a negative value
for an unsigned input makes the response invalid.

For a fresh `OK` response from a selected, synchronized, reliable source, the
communicator computes:

```text
network_error_bound_us =
    abs(remaining_correction_us)
    + root_distance_us
    + observation_sampling_margin_us
```

`observation_sampling_margin_us` is a configured non-negative conservative
allowance for tracking-response age, subprocess and observation-bracket
latency. Absolute-value overflow and every addition are checked; failure
invalidates the result. At or below the pilot 35,000,000 us trust threshold a
qualifying sample may establish `NETWORK_SYNCED`. Above the 40,000,000 us step
threshold it requires an ordered `UNTRUSTED` transition before a step can be
submitted. Between the thresholds, including exactly 40,000,000 us, the
communicator retains current quality: it does not promote `UNTRUSTED` or
`RTC_HOLDOVER`, and it does not demote an existing `NETWORK_SYNCED` solely for
that value. A stale, unselected, unsynchronized or unreliable result cannot
establish network trust or authorize a step.

`ChronyQueryStatus` has these runtime-only members:

| Name | Meaning |
|---|---|
| `OK` | A complete validated tracking result is present |
| `UNAVAILABLE` | chronyd or its command socket was unavailable |
| `DEADLINE_EXCEEDED` | The operation did not complete by its deadline |
| `INVALID_RESPONSE` | Output or values did not match the supported contract |

`apply_pending_correction_by_step()` executes only the fixed privileged
equivalent of `chronyc makestep`, which asks chronyd to apply its current
pending correction. It does not calculate or supply a correction. Its result
contains:

```text
disposition: ChronyStepDisposition
operation_started_at_monotonic_us: u64
operation_finished_at_monotonic_us: u64
```

`ChronyStepDisposition` has these runtime-only members:

| Name | Meaning |
|---|---|
| `SUBMITTED` | chronyd confirmed the command |
| `NOT_SUBMITTED` | failure is known to precede command acceptance |
| `OUTCOME_UNKNOWN` | the deadline or connection failed after acceptance became possible |

The communicator calls the step method at most once for one local step
operation generation. `SUBMITTED` and `OUTCOME_UNKNOWN` both enter
`WAITING_FOR_STABLE_TIME`; neither causes an immediate second step. Detailed
adapter failures may be converted into bounded communicator diagnostics and do
not expand this interface with arbitrary process output. Such conversion is
permitted only after the applicable time/core diagnostic catalogue is defined
in [`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md); until then, an adapter
failure uses bounded service logging and the fail-closed time state machine.

The communicator owns this runtime-only state:

```text
chrony_step_state:
    IDLE
    STEP_COMMAND_PENDING
    WAITING_FOR_STABLE_TIME
    RETRY_BACKOFF

operation_generation: u64
step_started_at_monotonic_us: u64
step_deadline_monotonic_us: u64
next_status_poll_monotonic_us: u64
retry_not_before_monotonic_us: u64
```

The timestamp fields are present only in the states that use them. This state
is rebuilt after process restart and is never added to
`CommunicatorStateV1`.

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
| `6` | `UNAVAILABLE_INCOMPATIBLE_SCHEMA` | Schema history, enum catalogue, immutable database metadata or a correctness-critical fixed entity is incompatible with this receiver build or configured group |

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
| `1` | `MEASUREMENT_PROFILE` | 506 bytes |
| `2` | `PROFILE_ONLY` | 457 bytes |
| `3` | `RECEIVER_HEALTH_REQUEST` | 278 bytes |
| `4` | `DIAGNOSTIC` | 171 bytes |
| `5` | `CLOCK_OBSERVATION` | 69 bytes |

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

`RETRANSMISSION` means that a later occurrence has the same `node_id`,
`message_id`, decoded `sample_id` and exact authenticated frame as the canonical
transport row. It does not claim why the logical transport message repeated.
For a new message ID, an existing sample with a byte-identical reading body is
`DUPLICATE_SAME_CONTENT`; an existing sample with different contents is
`DUPLICATE_CONFLICT`. Rewrapping an unaccepted current reading as backlog is
therefore `DUPLICATE_SAME_CONTENT`, not a conflict.

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

## `ClockObservationV1`

Encoded size: 69 bytes, including the two-byte entity envelope.

```text
receiver_instance_id: bytes[16]
linux_boot_id: bytes[16]
observation_sequence: u64
clock_state_generation: u64
validity_mask: u8
sampled_at_monotonic_us: u64
sampled_at_utc_us: i64
system_time_quality: u8
rtc_health: u8
```

Validity bit 0 is `SAMPLED_AT_UTC_VALID`; bit 1 is
`STEP_DISCONTINUITY_BOUNDARY`; bits 2 through 7 are reserved and zero. UTC is
absent exactly when `system_time_quality = UNTRUSTED` and present exactly when
quality is `RTC_HOLDOVER` or `NETWORK_SYNCED`.

`STEP_DISCONTINUITY_BOUNDARY` may be set only on the `UNTRUSTED` observation
created for one pending explicit chrony step. Ordinary quality-loss
observations clear it. The step cannot be submitted until that observation is
published and persistence has confirmed the exact boundary durable under the
commit-snapshot contract below. Once stored, the boundary terminates the
preceding correlation segment. Analysis assigns no UTC to an event in the
half-open monotonic interval from the boundary through, but excluding, the
first later trusted observation. That later observation begins a new segment
and is never used to extrapolate any event backward across the step boundary,
including an event before the boundary that lacked an earlier trusted
observation. This remains the conservative rule when step submission fails,
its outcome is unknown or the receiver process restarts.

The communicator assigns `observation_sequence` before each enqueue attempt.
A failed attempt therefore creates a visible sequence gap after a later
observation succeeds. One observation is emitted after initial time and RTC
state establishment, on every quality or RTC-health transition, immediately
before an intentional clock step and at the configured periodic interval. A
periodic observation retains the current `clock_state_generation`; a
transition or step-boundary observation carries the newly advanced generation.

For a trusted observation, `sampled_at_monotonic_us` is the midpoint of the
bounded monotonic bracket around one read-only `adjtimex(modes = 0)` call, and
`sampled_at_utc_us` is the system time returned by that call. The generation
must be equal before and after the bracket. `NETWORK_SYNCED` also requires a
fresh acceptable `ChronyTrackingResult` under the total-error entry/retention
policy in the Chrony control interface. Because deployment disables chrony's
`rtcsync`, Linux may return `TIME_ERROR` with `STA_UNSYNC` even when chrony is
synchronized; that expected pair is not a rejection. Other kernel metadata
that indicates clock interference or an invalid sample remains a rejection.
`UNTRUSTED` transition observations carry the transition boundary monotonic
time and zero UTC; they need no `adjtimex()` sample.

Chrony slew adjusts `CLOCK_REALTIME` and `CLOCK_MONOTONIC` together, so it does
not invalidate this pair or a same-boot UTC derivation. An explicit step does
change their offset and is separated by the required durably stored `UNTRUSTED`
boundary. Observation spacing is not a substitute for the applicable network
or RTC absolute-error acceptance policy.

The observation's identity is
`(receiver_instance_id, observation_sequence)`. Its `receiver_instance_id`
identifies the process that observed the correlation, while `linux_boot_id`
defines the clock domain. Analysis may use it for an event created by a
different receiver instance only when both have the same `linux_boot_id`.

Clock observations have ordinary FIFO importance. Every reservation attempt,
including a failed observation attempt, increments exactly one
`persist_queue_admission_counts[CLOCK_OBSERVATION][result]` cell. A pending
transition boundary is offered before subsequent ordinary queue admissions; an
explicit chrony step additionally requires its exact `UNTRUSTED` boundary
observation to have been durably committed and acknowledged by persistence.

### Clock-observation durability publication

The persistence thread exposes this immutable process-local snapshot through a
nonblocking read:

```python
get_clock_observation_durability_snapshot() -> ClockObservationDurabilitySnapshot
```

The returned value contains:

```text
last_durable_identity:
    (receiver_instance_id: bytes[16], observation_sequence: u64)
    or absent
```

The initial value is absent. Persistence replaces it only after a
`ClockObservationV1` has reached `SQLITE_COMMITTED` through a confirmed commit
or exact reconciliation and its queue entry has been acknowledged. The
snapshot may advance for ordinary observations as well as step boundaries; it
does not change queue priority or SQLite state. An immutable Python object
reference or an equivalently synchronized implementation makes each read
atomic. A stale read can only delay a step because observation identities never
repeat.

After publishing a step-boundary observation, the communicator keeps later
ordinary queue admission blocked and polls this snapshot outside the
RX-to-ACK critical path. It may submit the chrony step only when the snapshot
equals the exact pending boundary identity. An absent, older, different or
ambiguous result keeps quality `UNTRUSTED` and prohibits the step. A process
restart loses the snapshot but not a committed boundary row; no step operation
survives in communicator RAM, so the new instance never infers permission from
the old snapshot.

## `MessageProfilingV1`

`MessageProfilingV1` is the 455-byte packet-occurrence payload shared by the
two packet-related queue entities. `received_at_monotonic_us` is `T0`.

Fields in canonical order:

```text
receiver_instance_id: bytes[16]
linux_boot_id: bytes[16]
occurrence_sequence: u64
validity_mask: u64

received_at_monotonic_us: u64

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
| `0` | received frame |
| `1` | claimed control |
| `2` | claimed domain |
| `3` | claimed node ID |
| `4` | claimed message ID |
| `5` | decoded sample ID |
| `6` | RSSI |
| `7` | SNR |
| `8` | IRQ status |
| `9` | device errors |
| `10` | ACK frame |
| `11` | T2 |
| `12` | T3 |
| `13` | T4 |
| `14` | T5 |
| `15` | T6 |
| `16` | last BUSY-timeout opcode |
| `17`–`63` | Reserved; zero |

`T0` and `T1` are mandatory. RSSI and SNR validity bits must be equal. The
queued profile and stored profiling row contain no UTC, `SystemTimeQuality` or
`RtcHealth`. Analysis may correlate their monotonic fields with
`ClockObservationV1` without updating the row.

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

Encoded size: 506 bytes.

```text
entity envelope: 2 bytes
measurement candidate: 49 bytes
MessageProfilingV1: 455 bytes
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

Encoded size: 457 bytes.

```text
entity envelope: 2 bytes
MessageProfilingV1: 455 bytes
```

It represents one complete packet-occurrence profile without an application
candidate. It covers unknown nodes, authentication failures, malformed or
unsupported packets, wrong-direction packets and radio events without a usable
application frame.

The common fixed representation permits frame and identity fields to be absent;
a separate variable-sized radio-event entity is unnecessary.

### `ReceiverHealthRequestV1`

Encoded size: 278 bytes.

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

persist_queue_admission_counts: u64[5][3]
```

The recovery array is ordered by `RadioRecoveryReason` values `1` through `8`.

`persist_queue_admission_counts` is a cumulative reservation-attempt matrix.
Its rows are ordered by `PersistQueueEntityKind` values `1` through `5`, and
its columns are ordered by `AdmissionResult` values `0` through `2`. One call
to `try_reserve_one()` increments exactly one cell after the result is known.
The counting unit is one logical queue-unit reservation attempt, not one radio
packet, SQLite row, encoded byte or eventual publication. In particular,
`MeasurementProfileUnitV1` is one attempt and not separate measurement and
profile attempts.

`RESERVED` records successful capacity reservation; it does not assert that
the reservation was later published or durably committed. Interface
violations and process-level `MemoryError` do not produce an
`AdmissionResult` and do not increment the matrix. Aggregate counts such as
all queue-full rejections or all message-profiling admission failures are
derived by summing the applicable cells rather than stored as overlapping
counters.

The matrix intentionally does not subdivide `PERSISTENCE_UNAVAILABLE` by its
observed `PersistenceAdmissionState`. `ReceiverHealthV1` separately carries
the persistence-owned current state and cumulative transition counts by
state. The pilot accepts that these fields cannot reconstruct the precise
unavailable reason for every rejected attempt.

The validity-mask assignments are:

| Bit | Field |
|---:|---|
| `0` | last time-quality transition |
| `1` | last RTC-health transition |
| `2`–`7` | Reserved; zero |

For a periodic health attempt, the communicator advances `health_sequence`,
calls `try_reserve_one(RECEIVER_HEALTH_REQUEST)`, and increments the returned
matrix cell. On `RESERVED`, it then copies the updated matrix and the other
communicator-owned observations into the reserved builder and publishes it.
Consequently, a successfully admitted health request includes its own
`RESERVED` attempt. A failed health attempt appears only in a later
successfully persisted health request. Publication has no operational failure
result and therefore no matrix column.

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

Encoded size: 171 bytes. Its canonical field layout, ownership, replay rules,
severity and operation assignments, domain/error catalogues and fixed context
schemas are normative in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md). `PersistQueue` relies only
on the kind, schema version and exact encoded size defined by the two documents;
it never interprets domain-local context.

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

SQLite execution first produces an internal `OrdinaryBatchCommitOutcome`:

| Name | Meaning |
|---|---|
| `COMMITTED` | `COMMIT` was confirmed |
| `NOT_COMMITTED` | Failure is known to precede durable commit |
| `OUTCOME_UNKNOWN` | `COMMIT` may have executed but was not confirmed |

`NOT_COMMITTED` leaves every entry pending for retry. `OUTCOME_UNKNOWN` retains
the active lease and every frozen persistence-derived value while persistence
reconciles the exact rows; it neither acknowledges nor blindly reconstructs
the batch. Once an entity's `COMMIT` may have succeeded, a later existing
identity is handled by the idempotent replay contract below rather than as an
unexpected uniqueness failure.

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

### Ordinary SQLite idempotent replay

The primary durable identity for each queue entity is:

| Entity | Identity |
|---|---|
| `ClockObservationV1` | `(receiver_instance_id, observation_sequence)` |
| `ProfileOnlyUnitV1` | `(receiver_instance_id, occurrence_sequence)` |
| `MeasurementProfileUnitV1` | `(receiver_instance_id, occurrence_sequence)` |
| `ReceiverHealthRequestV1` | `(receiver_instance_id, health_sequence)` |
| `DiagnosticV1` | `(receiver_instance_id, diagnostic_sequence)` |

For an absent identity, persistence executes normal insertion. For an existing
identity, every stored column must equal the frozen intended value, using exact
byte equality for BLOBs and null-safe equality for optional values. Exact
equality is successful `SQLITE_COMMITTED` reconciliation and leaves the row
unchanged. Any difference is a correctness-critical identity collision:
persistence retains the lease, performs no update or quarantine, publishes
`UNAVAILABLE_INCOMPATIBLE_SCHEMA` and requires operator or implementation
recovery.

All queue-backed SQLite rows are immutable. They store monotonic event time
only; UTC correlation belongs to analysis and creates no replay exception.
`ClockObservationV1` is the sole queue entity that contains a sampled UTC when
trusted, and its complete row must also match exactly.

For `ProfileOnlyUnitV1`, comparison includes every profiling column and
`persistence_classification = NOT_APPLICABLE`. For
`ReceiverHealthRequestV1`, persistence freezes the complete enriched
`ReceiverHealthV1` before its first commit attempt and compares every request,
persistence and optional host-observation column without resampling. For
`DiagnosticV1`, comparison includes the persistence-added `linux_boot_id`.

For `MeasurementProfileUnitV1`, the stored profile classification is immutable
and authoritative during replay. Persistence validates it against the
canonical transport and reading rows as specified by the SQLite schema
contract; it never recomputes a committed occurrence's classification from the
now populated tables. Matching already committed entities may be no-ops in a
retry transaction while absent entities are inserted. Queue acknowledgement is
legal only after confirmed commit or reconciliation proves every entity's
complete durable effect.

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
bounded `DiagnosticV1` only when its error domain and context schema are
defined in [`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md); diagnostic
admission remains best effort and never controls recovery.

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
`network_utc_at_rtc_verification_us` is obtained from the fresh trusted clock
observation used for the RTC write/read-back episode, advanced only by measured
monotonic elapsed time.

`rolling_window_us`, `bucket_width_us`, `reservation_spend_lifetime_us` and
`reservation_expiration_guard_us` are physical policy durations. Their durable
UTC calculations use the stored values directly. Runtime monotonic decisions
apply the elapsed-duration policy: minimum rolling/unknown-history waits are
lengthened with `minimum_wait_monotonic_us()`, and the reservation spend
lifetime is shortened with `maximum_lifetime_monotonic_us()`.

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
`minimum_wait_monotonic_us(rolling_window_us + bucket_width_us +
reservation_expiration_guard_us)` on Linux monotonic time and can capture
trusted canonical UTC for the new snapshot. The sum and conversion use checked
integer arithmetic. A process restart restarts that waiting interval. No
operator claim that an installation is new bypasses this conservative aging
rule.

For `KNOWN`, `airtime_snapshot_utc_us` is canonical UTC derived for the
snapshot's monotonic construction time from the communicator's latest live
trusted `ClockObservationV1` correlation under `NETWORK_SYNCED` or valid
`RTC_HOLDOVER`. It is not a direct per-state `CLOCK_REALTIME` read. Every stored
expiration is later than that snapshot.

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
The current instance's live monotonic spend deadline is constructed from
`maximum_lifetime_monotonic_us(reservation_spend_lifetime_us)`, is separate
runtime state and is never serialized. After an unknown commit outcome, it may
be reused only when the loaded value exactly matches the originally requested
state.

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
```

The start value is not a `PersistQueue` entity or a public control-channel
request. The persistence thread combines it with the `linux_boot_id` it reads,
inserts the `receiver_instances` row after database validation and before
publishing ordinary admission as available, and refuses to operate if that
identity already exists. A start UTC and source clock-observation identity may
be derived during analysis from the same-boot observation timeline; neither
result nor time quality and RTC health is duplicated into the lifecycle row or
request.

`ReceiverCleanStopV1` is immutable control data:

```text
receiver_instance_id: bytes[16]
stopped_at_monotonic_us: u64
communicator_state_generation: u64
```

Analysis may derive stop UTC and its source observation later through the same
correlation contract without updating the lifecycle row. Generation zero is
permitted when conservative generation zero is the known authoritative state.

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
Repeating the exact request is idempotent. Equality compares only the three
caller-supplied `ReceiverCleanStopV1` fields.

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
| `clock_observations` | `(receiver_instance_id, observation_sequence)`; monotonic/UTC correlations and time-state boundaries |
| `message_profiles` | `(receiver_instance_id, occurrence_sequence)`; every admitted packet occurrence |
| `transport_messages` | `(node_id, message_id)`; first canonical authenticated frame/domain/sample |
| `readings` | `(node_id, sample_id)`; first canonical reading body and decoded measurement columns |
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
mapped to `NULL`, plus the derived `persistence_classification`. It stores no
UTC or source-observation columns and is immutable after insertion.
`transport_messages` stores the first exact authenticated frame, domain,
decoded sample ID and first occurrence identity. `readings` retains the exact
32-byte canonical reading body for equality, its decoded protocol fields for
analysis and the first occurrence identity. Transport and reading canonical
rows are immutable.

Ordinary replay compares the complete stored representation, not a subset of
business fields. For profiles this includes every `MessageProfilingV1` column
and the frozen persistence classification. For transport rows it includes
`node_id`, `message_id`, domain, decoded sample ID, the exact authenticated
frame and the first `(receiver_instance_id, occurrence_sequence)`. For reading
rows it includes `node_id`, `sample_id`, the exact reading body, every decoded
column and the first occurrence identity. No field in these rows is eligible
for a later update.

For a replayed `MeasurementProfileUnitV1`, persistence also verifies that its
frozen classification agrees with the immutable canonical effects:

- `FIRST_SEEN`: the current occurrence is the first owner of matching
  transport and reading rows;
- `RETRANSMISSION`: an earlier occurrence owns the exact matching transport
  row;
- `DUPLICATE_SAME_CONTENT`: the current occurrence owns its new transport row
  and an earlier occurrence owns a byte-identical reading row;
- `DUPLICATE_CONFLICT`: either an earlier transport row has the same sample but
  a different exact frame, or the current occurrence owns its transport row
  and an earlier occurrence owns a different reading body; and
- `MESSAGE_ID_CONFLICT`: an earlier occurrence owns the transport key with a
  different sample ID.

The profile and all canonical effects created for one occurrence commit in one
transaction. A canonical row that names the current occurrence as first owner
while its profile row is absent is therefore an invariant failure, not a
partially successful replay. Matching existing effects are no-op success;
absent required effects are inserted; any different row under the same durable
identity closes admission as `UNAVAILABLE_INCOMPATIBLE_SCHEMA` and is never
updated or quarantined.

Analysis may materialize this separate output outside the live receiver's
ordinary persistence path:

```text
node_id: bytes[8]
sample_id: u32
logical_utc_us: i64
timestamp_source: DIRECT or EXTRAPOLATED
anchor_sample_id: u32
clock_observation_receiver_instance_id: bytes[16]
clock_observation_sequence: u64
```

`(node_id, sample_id)` is the output identity. An existing exact value is
idempotent success; a conflicting value is an analysis invariant failure, not
an update. A direct timestamp references the observation used for the anchor
occurrence. An extrapolated timestamp references the same observation behind
its direct anchor. This output is not an initial receiver database table and
is never written by ordinary `PersistQueue` processing.

Duplicate and identity conflicts create no `DiagnosticV1`. Their evidence is
the occurrence profile's classification together with immutable canonical
transport and reading rows. An analysis view may join those tables to expose
conflicts directly.

### Clock observations and UTC assignment

`clock_observations` uses this logical SQL contract:

```sql
CREATE TABLE clock_observations (
    receiver_instance_id BLOB NOT NULL
        REFERENCES receiver_instances(receiver_instance_id)
        CHECK (length(receiver_instance_id) = 16),
    observation_sequence INTEGER NOT NULL
        CHECK (observation_sequence >= 0),

    linux_boot_id BLOB NOT NULL
        CHECK (length(linux_boot_id) = 16),
    clock_state_generation INTEGER NOT NULL
        CHECK (clock_state_generation >= 0),
    sampled_at_monotonic_us INTEGER NOT NULL
        CHECK (sampled_at_monotonic_us >= 0),
    sampled_at_utc_us INTEGER,
    step_discontinuity_boundary INTEGER NOT NULL
        CHECK (step_discontinuity_boundary IN (0, 1)),
    system_time_quality_id INTEGER NOT NULL
        REFERENCES system_time_quality_codes(id),
    rtc_health_id INTEGER NOT NULL
        REFERENCES rtc_health_codes(id),

    PRIMARY KEY (receiver_instance_id, observation_sequence),
    CHECK (
        (system_time_quality_id = 0 AND sampled_at_utc_us IS NULL)
        OR
        (system_time_quality_id IN (1, 2) AND sampled_at_utc_us IS NOT NULL)
    ),
    CHECK (
        step_discontinuity_boundary = 0
        OR system_time_quality_id = 0
    )
) STRICT, WITHOUT ROWID;
```

Persistence verifies that `linux_boot_id` equals the source receiver
instance's boot ID and that sequence, generation and monotonic ordering obey
the `ClockObservationV1` contract. Rows are append-only. Multiple periodic
observations may carry the same generation.

For a stored event with `(linux_boot_id, event_monotonic_us)`, analysis
orders observations by sampled monotonic value, source `instance_ordinal` and
observation sequence. It applies this deterministic rule:

1. Any `UNTRUSTED` observation closes the preceding trusted segment at its
   monotonic boundary.
2. If the event lies in an open trusted segment, select the latest trusted
   observation at or before the event.
3. If the event precedes the first trusted observation or lies after an
   ordinary untrusted boundary, select the first later trusted observation in
   the same boot only when no step-discontinuity boundary lies between the
   event and that observation.
4. A step-discontinuity boundary terminates the preceding segment. Assign no
   UTC when the event is at or after that boundary and before the first later
   trusted observation. That later observation starts the new segment and is
   never extrapolated backward across the boundary, including to an event
   before the boundary that has no eligible preceding trusted observation.
5. Treat an observation at the event's same monotonic microsecond as preceding
   the event.
6. Derive UTC with
   `observation_utc + event_monotonic - observation_monotonic`.

Analysis returns the derived UTC and source-observation identity together; it
does not update the event row. An observation from a different receiver process
is valid only within the same `linux_boot_id`; no cross-boot assignment is
permitted.

Before producing a `DIRECT` reading timestamp, analysis evaluates every stored
accepted `CURRENT_READING_UPLINK` occurrence for that sample. It selects the
earliest occurrence that has derivable UTC, satisfies
`run_ms + Tair <= 30,000 ms`, and is classified `FIRST_SEEN`,
`RETRANSMISSION` or `DUPLICATE_SAME_CONTENT`. Authentication without successful
queue admission, either conflict classification, and a step-gap occurrence are
not anchor-eligible.

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

    clean_stopped_at_monotonic_us INTEGER,
    clean_stop_state_generation INTEGER
        CHECK (
            clean_stop_state_generation IS NULL
            OR clean_stop_state_generation >= 0
        ),
    CHECK (
        (clean_stopped_at_monotonic_us IS NULL
         AND clean_stop_state_generation IS NULL)
        OR
        (clean_stopped_at_monotonic_us IS NOT NULL
         AND clean_stop_state_generation IS NOT NULL
         AND clean_stopped_at_monotonic_us >= started_at_monotonic_us)
    )
) STRICT;
```

`instance_ordinal` is database-local analysis order and is never reused because
lifecycle rows are never deleted. `receiver_instance_id` remains the public
identity referenced by other tables.

The row is inserted as soon as SQLite is usable and before ordinary persistence
admission becomes available. `started_at_monotonic_us` is captured when the
receiver instance ID is created. Analysis may derive start UTC from a
qualifying same-boot observation, but the lifecycle row remains unchanged.

There is no separate clean-stop Boolean. Presence of
`clean_stopped_at_monotonic_us` is the marker. When absent, every other
clean-stop field is absent. When present, state generation is present and stop
monotonic time is not earlier than start. Lifecycle control may make the one
transition from no marker to a complete monotonic marker. Triggers reject every
other mutation or removal. Analysis derives any start or stop UTC and returns
its source-observation identity without storing either value in this table.

The preceding durable instance is the row with the greatest lower
`instance_ordinal`. Absence of its marker establishes only that controlled
shutdown was not durably confirmed.

### Poisoned queue units

A poisoned unit is an admitted, representation-valid immutable queue unit that
reproducibly fails persistence when isolated because of an entity-specific
decoder, binding, derivation, range or unexpected schema-constraint defect. It
is not malformed radio input, a duplicate classification, disk full, database
corruption, locking or a transient/global I/O failure.

`ClockObservationV1` is not eligible for item quarantine. Its isolated failure
is treated as a receiver-interface/schema incompatibility: persistence retains
the clock observation and claimed batch, publishes
`UNAVAILABLE_INCOMPATIBLE_SCHEMA` and does not process later FIFO entities. A
missing time-state boundary must never be converted into a quarantined gap
while analysis continues correlating later events with UTC.

Persistence rolls back the failed batch, excludes global and transient causes,
narrows the batch and retries the suspected unit alone. Except for the clock
observation rule above, only reproduction of the item-specific failure permits quarantine. A
`MeasurementProfileUnitV1` is always quarantined as one complete 506-byte unit.

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
It stores the queued monotonic fields plus `linux_boot_id`; every column is
immutable after insertion. Analysis may derive a UTC value and
source-observation identity without updating the diagnostic row.
Persistence-control failures may be returned to the communicator and converted
there according to the catalogues in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md). Asynchronous SQLite
failures may be exposed by admission state, a later `ReceiverHealthV1`,
quarantine provenance when applicable and bounded service logging. None is a
guaranteed durable incident record while SQLite itself is unavailable; this is
the explicit pilot limitation defined in
[`ARCHITECTURE.md`](ARCHITECTURE.md#persistence-unavailable-observability-limitation).

`receiver_health` expands enum-indexed arrays into named numeric columns for
analysis and stores optional host observations as `NULL`. Persistence creates
the complete immutable row once and never resamples it across transaction
retry.

No `persistence_batches` or `receiver_state_operations` tables exist in the
pilot. Batch throughput, failure, byte and checkpoint aggregates are carried by
`ReceiverHealthV1`. Communicator-state control failures may use `DiagnosticV1`
only after the applicable catalogue is defined and when the communicator can
admit one; authoritative state remains only in `communicator_state`.

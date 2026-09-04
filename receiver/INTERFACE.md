# Receiver interfaces

Status: provisional pilot contract. This document defines the shared values and
typed `PersistQueue` entities that communicator and persistence implementations
must interpret identically. Names, numeric values and layouts
may be revised during initial implementation. During the pilot, an incompatible
persisted change starts a new explicitly archived-and-recreated database schema
epoch; an existing database is never reinterpreted or upgraded in place.

The protocol schema and
[`protocol-v2-lora/README.md`](../protocol/protocol-v2-lora/README.md) remain
authoritative for LoRa frame contents. This document defines receiver-local
representations and does not redefine the wire protocol.

The complete diagnostic entity, common diagnostic enums and domain-local
catalogues are defined separately in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md). Numeric assignments in
that document are part of this interface.

## Value and encoding conventions

The scalar notation used throughout this document has the following meaning
when a value is projected into SQLite or a section explicitly defines a
canonical binary encoding:

- integers are little-endian;
- signed integers use two's-complement representation;
- byte identifiers are opaque byte sequences and are not endian-converted;
- fields appear in the order shown in this document;
- no alignment padding is inserted;
- `bool8` is a `u8` whose only valid values are `0` and `1`; and
- every reserved bit and byte is zero.

`PersistQueue` does not encode these values. It transfers strong references to
complete immutable Python objects together with a separate entity kind and
schema version. Its capacity is counted only in occupied entity slots; neither
serialized length nor Python heap size participates in admission. The normal
queue path therefore has no entity encoder or decoder. When a poisoned typed
entity must be preserved, the separate bounded `QuarantineEvidenceV1` tagged-
JSON codec captures its logical value.

## Common scalar values

| Type | Encoding | Meaning |
|---|---:|---|
| `bool8` | `u8` | Boolean; only `0` and `1` are valid |
| `MonotonicUs` | `u64` | Linux monotonic microseconds |
| `UtcUs` | `i64` | POSIX UTC microseconds since the Unix epoch |
| `UtcSeconds` | `i64` | Whole POSIX UTC seconds since the Unix epoch |
| `DurationUs` | `u64` | Non-negative elapsed microseconds |
| `Counter64` | `u64` | Cumulative counter that saturates instead of wrapping |
| `FrameBuffer` | `bytes[255]` | Complete SX1262 payload capacity |
| `AckFrame` | `bytes[23]` | Exact protocol ACK frame |
| `ReadingBody` | `bytes[32]` | Canonical plaintext protocol reading body |
| `DiagnosticContext` | `bytes[128]` | Bounded domain-local diagnostic context |

SQLite represents integers as signed 64-bit values. Every `u64` that is bound
to SQLite must therefore be in `0..INT64_MAX`; crossing that bound is an
interface invariant failure. The receiver terminates or saturates the
applicable counter before it can cross the bound. Canonical binary encodings
remain unsigned and retain their `u64` representation.

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
| `QueueReservationToken` | opaque process-local object identity | Unique single-use queue reservation token |

`MessageId` and `SampleId` never wrap or repeat within their protocol-defined
identity lifetimes.

`OccurrenceSequence`, `ClockObservationSequence`, `HealthSequence` and
`DiagnosticSequence` start at `0` and advance before the corresponding attempt.
They never wrap. Sequence exhaustion is a terminal failure for the current
receiver instance.

`ClockStateGeneration` starts at `0` for each receiver instance and is the one
runtime revision for every decision that depends on live clock trust. The
communicator advances it when it processes a completed chrony tracking result,
whenever it changes `SystemTimeQuality` or `RtcHealth` outside that atomic poll
update, and before every intentional clock-step boundary even when quality is
already `UNTRUSTED`. One atomic update that changes multiple inputs advances
the generation once. A purely periodic observation with no new time-policy
input does not advance it.

A time-derived operation captures this generation before deriving UTC and
must compare it after every call that can yield or block and immediately before
publishing or committing its trusted result. A mismatch invalidates that
result. After a successful comparison, the same operation may atomically apply
its own returned RTC-health/quality transition, advance the generation once and
continue using that new value as its captured generation. It may not adopt a
generation advanced by another scheduled task. This single generation
therefore protects both observation sampling and the longer RTC
write/read-back/provenance episode without another runtime trust-snapshot
identity. It is runtime observation provenance, not the durable
`StateGeneration` used by `CommunicatorStateV1`.

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
`AdmissionGeneration` starts at `0` and increments for every accepted
persistence-admission snapshot publication. The persistence owner normally
publishes only a state transition; the queue enforces publication order but
does not decide whether repeating a state was useful.

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
canonical only when `sampled_at_utc_us` is present and that observation's
`SystemTimeQuality` is `RTC_HOLDOVER` or `NETWORK_SYNCED`. An event UTC is
canonical only when analysis has derived it from such an observation in the
same `LinuxBootId` and returns the value together with the source-observation
identity. Receiver event and lifecycle rows are not updated with that result.

Zero is a valid numeric timestamp and is never an absence sentinel. Optional
timestamps use `None` in logical Python values and `NULL` in relational rows.
An explicit canonical binary encoding may instead define its own validity bit
and zero representation.

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
including rolling-airtime retention, incompatible-history aging and minimum retry
backoff. `maximum_lifetime_monotonic_us()` is used when acting too late is
unsafe, including radio/time-service operation deadlines and the remaining
physical lifetime of an active airtime bucket grant. The caller constructs an absolute monotonic
deadline by checked addition of the applicable converted duration to a current
boot monotonic sample. Profiling, health and periodic clock-observation
intervals are observational and use their nominal monotonic duration unless
their contract explicitly requires a safety conversion.

The pilot deployment defaults are:

```text
chrony_max_slew_rate_ppm = 3_500
monotonic_elapsed_rate_bound_ppm = 3_700
rtc_drift_bound_ppm = 10
time_sampling_margin_us = 1_000_000
network_trust_error_threshold_us = 35_000_000
network_step_error_threshold_us = 40_000_000
receiver_utc_error_budget_us = 40_000_000
network_rtc_write_error_threshold_us = 5_000_000
chrony_tracking_poll_period_cap_us = 60_000_000
clock_observation_period_cap_us = 10_800_000_000
network_rtc_refresh_period_us = 10_800_000_000
rtc_holdover_observation_period_cap_us = 3_600_000_000
rolling_window_us = 3_600_000_000
tx_airtime_budget_us = 36_000_000
bucket_width_us = 60_000_000
bucket_charge_limit_us = 8_000_000
bucket_expiration_guard_us = 1_000_000
```

`time_sampling_margin_us` is one fixed pilot constant shared by network-clock
sampling, direct RTC reads and verified RTC read-back comparison. It is not an
independently tunable margin per backend. The pilot value reserves one second
for the bounded tracking-response age and local sampling effects described
below; measured operation brackets and the actual accepted RTC read-back
difference remain separate charged terms. An operation that violates its
bounded bracket or freshness contract is rejected rather than assigned a
larger ad hoc margin.

The same constant supplies the fixed acceptance bounds. For one network
observation, checked monotonic ordering must hold from
`ChronyTrackingResult.sample_started_at_monotonic_us` through the end of the
`adjtimex()` bracket, and that complete span must not exceed
`time_sampling_margin_us`. For one direct or read-back RTC read, the checked
operation bracket must not exceed `time_sampling_margin_us`. Exceeding either
bound makes that sample unusable; the code never stretches the margin at
runtime.

`chrony_max_slew_rate_ppm` is the receiver's declared expectation; the tracking
adapter cannot discover the installed daemon value. Deployment validation must
separately verify that chronyd uses that exact ceiling. Receiver configuration
is valid only when
`0 < rtc_drift_bound_ppm < P`,
`chrony_max_slew_rate_ppm <= monotonic_elapsed_rate_bound_ppm < P`,
`0 < time_sampling_margin_us < network_rtc_write_error_threshold_us`,
`network_rtc_write_error_threshold_us <= network_trust_error_threshold_us <
network_step_error_threshold_us`, and
`receiver_utc_error_budget_us <= network_step_error_threshold_us`. Every period
is positive. Every constructed RTC-holdover verification, age/drift and
direct-read uncertainty sum must be strictly less than
`receiver_utc_error_budget_us`, preserving the 40-second maximum trusted
receiver-UTC error for direct protocol anchors. The
3,700 ppm receiver bound intentionally includes margin above chrony's 3,500
ppm phase-correction ceiling. It yields these normative examples:

```text
minimum_wait_monotonic_us(3_600_000_000) = 3_613_320_000
maximum_lifetime_monotonic_us(30_000_000) = 29_889_000
minimum_wait_monotonic_us(10_800_000_000) = 10_839_960_000
```

The last result shows the maximum 39.96-second monotonic-rate allowance over
three physical hours. The configured observation periods are upper scheduling
caps, not accuracy proofs. The communicator schedules an earlier observation
or ends the trusted segment when the absolute-error calculation below produces
an earlier deadline. These values are configurable pilot defaults and must be
revised together with deployment chrony configuration and the receiver's
startup validation.

The initial 10 ppm RTC bound is for a deployment-validated genuine DS3231-class
part and conservatively covers the specified full-temperature stability,
long-term aging and additional pilot margin. A different RTC variant, unknown
module provenance, environment outside the validated range or pilot evidence
outside this bound must use a larger validated value or disable
`RTC_HOLDOVER`. Each successful RTC refresh stores the bound it used, so a
later software/configuration change does not silently reinterpret older
provenance.

### UTC-error growth and observation deadlines

Rate bounds are non-negative fractional-rate limits expressed in ppm. The
monotonic bound limits the difference between Linux `CLOCK_MONOTONIC` and
physical elapsed time. The RTC drift bound limits the difference between the
DS3231 counting rate and UTC under the validated deployment temperature,
aging, supply and backup-power conditions. One ppm accumulates at most one
microsecond of error per physical second. All conversions use checked integer
arithmetic and round error upward.

Both elapsed values below are measured by the clock whose rate is bounded, not
by an independent perfect clock. With `P = 1_000_000`, their conservative
growth function is therefore:

```text
rate_growth_us(rate_bound_ppm, observed_elapsed_us) = ceil(
    observed_elapsed_us * rate_bound_ppm
    / (P - rate_bound_ppm)
)
```

The denominator accounts for the worst case in which the observed clock runs
slow. Using `observed_elapsed_us * rate_bound_ppm / P` would be a first-order
approximation and is not the normative checked calculation.
`rtc_drift_growth_us()` and `monotonic_rate_growth_us()` below are semantic
names for this same centralized function with the corresponding bound.

For one operation bracket measured by `CLOCK_MONOTONIC`, the conservative
physical half-bracket is:

```text
maximum_physical_half_bracket_us(observed_bracket_us) = ceil(
    observed_bracket_us * P
    / (2 * (P - monotonic_elapsed_rate_bound_ppm))
)
```

The caller first checks that the finish is not earlier than the start and that
the subtraction, multiplication and denominator are valid. A direct successful
RTC read then has exactly this uncertainty at its bracket midpoint:

```text
rtc_read_whole_second_uncertainty_us =
    500_000
    + maximum_physical_half_bracket_us(
          operation_finished_at_monotonic_us
          - operation_started_at_monotonic_us)
    + time_sampling_margin_us
```

The 500,000 us term is the DS3231 whole-second representation. The shared
margin covers the fixed pilot device and local sampling allowance; the measured
bracket is not hidden inside that margin.

For a completed or uncertain RTC write followed by a successful read-back, the
communicator advances the supporting trusted network observation to the
read-back bracket midpoint and computes:

```text
network_error_at_rtc_verification_us =
    network_observation_error_bound_us
    + monotonic_rate_growth_us(
          monotonic_elapsed_rate_bound_ppm,
          abs(readback_midpoint_monotonic_us
              - network_observation_monotonic_us))

rtc_readback_difference_us = abs(
    network_utc_at_rtc_verification_us - rtc_readback_utc_us)
```

The read-back matches only when
`rtc_readback_difference_us <= time_sampling_margin_us`. The actual difference,
not the full permitted margin, is charged. The complete durable bound is:

```text
rtc_verification_uncertainty_us =
    network_error_at_rtc_verification_us
    + rtc_readback_difference_us
    + rtc_read_whole_second_uncertainty_us
```

Immediately before state-commit submission, the advanced network error must
still be at or below `network_rtc_write_error_threshold_us`, and the complete
verification uncertainty must be strictly less than
`receiver_utc_error_budget_us`. Failure of either condition establishes no new
provenance. This read-back-based equation needs no separate write-bracket,
device-margin or configurable read-back-tolerance term: a possibly applied
write becomes trustworthy only through the directly observed final RTC value.

For a direct RTC observation, the RTC's error at the observation is:

```text
rtc_observation_error_bound_us =
    rtc_verification_uncertainty_us
    + rtc_drift_growth_us(
          persisted_rtc_drift_bound_ppm,
          rtc_age_since_verification)
    + rtc_read_whole_second_uncertainty_us
```

`rtc_verification_uncertainty_us` is the durable bound established by the last
network-to-RTC write/read-back episode. Drift before the new RTC read belongs
in the second term. The read term covers the bounded device-operation bracket
and the DS3231's whole-second representation.

After the RTC has been read at `observation_monotonic_us`, event UTC is derived
only from that immutable UTC/monotonic pair. The RTC is not consulted again for
that extrapolation, so RTC drift does not continue accumulating over the
post-observation interval:

```text
event_utc_error_bound_us =
    observation_utc_error_bound_us
    + monotonic_rate_growth_us(
          monotonic_elapsed_rate_bound_ppm,
          abs(event_monotonic_us - observation_monotonic_us))
```

There is therefore no general `combined_rate_bound` for this model. A sum of
the monotonic and RTC rate bounds is permitted only as an explicitly
conservative scheduling shortcut when one interval is deliberately used as an
upper bound for both distinct ages; it is not the normative error equation.
For one rate bound expressed in ppm, the maximum observation-to-event distance
is:

```text
remaining_budget_us =
    receiver_utc_error_budget_us - observation_utc_error_bound_us

maximum_interval_us = floor(
    remaining_budget_us * (1_000_000 - rate_bound_ppm)
    / rate_bound_ppm
)
```

The interval ends at an exclusive trust boundary. If
`observation_utc_error_bound_us >= receiver_utc_error_budget_us`, if the
applicable rate bound is invalid, or if checked calculation fails, no positive
trusted interval exists. The communicator transitions to `UNTRUSTED` and
applies the pending-boundary FIFO rule. Budget expiry alone never authorizes a
clock step; only the separate fresh chrony step rule can do that.
A zero rate bound yields no rate-derived deadline but does not remove the
configured observation-period cap. Runtime uses the monotonic bound for the
distance from an observation to an event and the persisted RTC drift bound for
RTC age since network verification; it never charges both against the same
post-read interval.

With the pilot defaults, stored RTC verification uncertainty of 4,000,000 us
and a zero-width illustrative read bracket give initial direct-read uncertainty
of 1,500,000 us. The strict maximum RTC age before the direct observation itself
reaches the 40-second budget is:

```text
floor(
    (40_000_000 - 4_000_000 - 1_500_000 - 1)
    * (1_000_000 - 10) / 10
) = 3_449_965_400_001 us
```

This is about 39.93 days; any nonzero read bracket shortens it. Re-reading the
RTC does not reset this age because only a verified network-to-RTC refresh
replaces durable provenance. One hour of post-observation monotonic extrapolation
adds 13,369,468 us with the 3,700 ppm bound, so the ordinary one-hour RTC-read
cadence remains sufficient only until about 24.46 days in this example. After
that point the calculated horizon shortens the cadence progressively until no
positive interval remains.

## Logical optional-field rules

For the typed Python values in this interface:

- an absent optional field is `None`, while a present zero remains distinct;
- a `bytes[N]` field has exactly `N` bytes, while an explicitly variable
  `bytes` field contains only its meaningful bytes;
- a sibling meaningful-length field, zeroed fixed-capacity tail, fixed tuple
  shape or bounded byte length remains an entity contract that the producing
  component validates rather than queue capacity metadata;
- enum fields contain members of their declared enum and a numeric zero is
  valid only when that enum assigns it a meaning; and
- lists, dictionaries, mutable buffers and arbitrary exception objects are not
  valid members of a production queue entity.

`PersistQueue` treats the object as opaque and neither enforces nor normalizes
these rules. This is intentional: a defective typed entity can still reach the
persistence boundary and the quarantine-evidence encoder can preserve any
supported malformed scalar, length or tuple value exactly. Unsupported mutable
or unbounded values make evidence encoding fail closed and leave the queue head
owned.

Where a section explicitly defines a canonical binary encoding, its validity
bitmap, zero representation for absent fields, padded arrays and reserved bits
remain authoritative for those bytes only. They are not the in-memory queue
representation.

Before committing to an ACK outcome, the communicator validates every stable
pre-TX input. After the terminal outcome it constructs one complete frozen
typed object and publishes that existing reference against the reservation.
Publication itself performs no payload allocation, conversion or
serialization; construction remains ordinary Python process work.

## Time and radio enums

### `SystemTimeQuality`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `UNTRUSTED` | No receiver UTC source currently satisfies the trust contract |
| `1` | `RTC_HOLDOVER` | A direct DS3231 observation with durable provenance satisfies the holdover error budget |
| `2` | `NETWORK_SYNCED` | Current time service and system UTC satisfy the network error contract |

### `RtcHealth`

| Value | Name | Meaning |
|---:|---|---|
| `1` | `PRESENT` | RTC responds and its time is valid |
| `2` | `MISSING` | RTC is absent or unreachable |
| `3` | `INVALID` | RTC responds but its time is invalid |

`RtcHealth = 0` is invalid.

## DS3231 control interface

The communicator owns runtime DS3231 policy and accesses the device only
through one fakeable adapter. The boot helper remains a separate, ordered
once-per-Linux-boot RTC-to-system-clock bootstrap component; it does not own
runtime receiver policy. The persistence thread never accesses RTC hardware.

```python
class Ds3231Control(Protocol):
    def read_time(
        self,
        *,
        deadline_monotonic_us: MonotonicUs,
    ) -> Ds3231ReadResult: ...

    def write_time(
        self,
        *,
        rtc_utc_s: UtcSeconds,
        deadline_monotonic_us: MonotonicUs,
    ) -> Ds3231WriteResult: ...
```

Both methods are synchronous, have a short absolute monotonic deadline and are
called only outside the RX-to-ACK critical path. The backend must have a
deployment-validated finite kernel/device-operation bound consistent with that
deadline. It records the actual return time even when an underlying operation
finishes after its deadline. Neither method reads `CLOCK_REALTIME`, changes
Linux system time, performs persistence I/O or retries without returning to
communicator policy.

`UtcSeconds` reflects the DS3231's whole-second resolution. `write_time()`
accepts only a canonical value in the deployment-validated RTC/driver range.
The communicator derives that value from a fresh trusted clock observation and
measured monotonic elapsed time. An out-of-range or otherwise noncanonical time
value is an interface violation before physical I/O; it is not an operational
device result. A deadline already expired at entry returns
`DEADLINE_EXCEEDED` for a read and
`NOT_APPLIED + DEADLINE_EXCEEDED` for a write. The adapter accepts no
caller-supplied device path, helper path, time source, local-time value or
option.

### Read result

`Ds3231ReadResult` contains:

```text
status: Ds3231ReadStatus
operation_started_at_monotonic_us: u64
operation_finished_at_monotonic_us: u64
rtc_utc_s: i64 or absent
os_errno: i32 or absent
```

`Ds3231ReadStatus` has these runtime-only members:

| Name | Meaning |
|---|---|
| `OK` | One valid whole-second UTC value was read |
| `MISSING` | The configured RTC device was absent or did not respond |
| `INVALID` | The device responded but its oscillator-stop/validity state, calendar fields or represented time were invalid |
| `IO_ERROR` | Another device, driver or local-helper I/O failure prevented a valid read |
| `DEADLINE_EXCEEDED` | No valid read completed within the absolute deadline |

`rtc_utc_s` is present if and only if `status = OK`. `os_errno` is absent for
`OK` and otherwise is present only when the backend received a meaningful OS
error number. The two monotonic samples are always present, use the current
Linux boot and bracket the attempted read. Read failure has no physical-effect
ambiguity.

The communicator maps `OK` to `RtcHealth.PRESENT` and `INVALID` to
`RtcHealth.INVALID`. `MISSING`, `IO_ERROR` and `DEADLINE_EXCEEDED` mean the RTC
is currently unreachable and map to `RtcHealth.MISSING`; the more precise
operation result remains available to the running process. Exceptional I/O and
deadline results use the `TIME` catalogue in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md#time-diagnostic-catalogue),
while ordinary `MISSING` and `INVALID` states do not create diagnostics.
Time-quality policy remains independent, so a missing RTC does not remove
otherwise valid `NETWORK_SYNCED` time.

### Write result

Write effect and failure reason are separate because a failed or timed-out
operation may already have changed the device:

```text
disposition: Ds3231WriteDisposition
failure: Ds3231Failure
operation_started_at_monotonic_us: u64
operation_finished_at_monotonic_us: u64
os_errno: i32 or absent
```

`Ds3231WriteDisposition` has these runtime-only members:

| Name | Meaning |
|---|---|
| `COMPLETED` | The RTC driver reported successful completion of the write |
| `NOT_APPLIED` | Failure is known to precede any possible `RTC_SET_TIME` submission |
| `OUTCOME_UNKNOWN` | `RTC_SET_TIME` may have been submitted, but completion was not established |

`Ds3231Failure` has these runtime-only members:

| Name | Meaning |
|---|---|
| `NONE` | No operation failure |
| `MISSING` | The configured device was absent or became unreachable |
| `IO_ERROR` | Another device, driver, helper-execution or helper-protocol failure occurred |
| `DEADLINE_EXCEEDED` | The absolute deadline expired |

`COMPLETED` requires `failure = NONE`. `NOT_APPLIED` and `OUTCOME_UNKNOWN`
require a non-`NONE` failure. `os_errno` follows the same presence rule as the
read result. Failure before helper execution or before its ioctl-submission
point is `NOT_APPLIED`; any ioctl error, helper timeout, signal termination,
crash, malformed helper result or loss of completion after submission became
possible is conservatively `OUTCOME_UNKNOWN`. The adapter reports `COMPLETED`
only from the helper protocol's recognized success result; elapsed time, an
unrecognized normal process exit or an inferred device state is insufficient.

Neither `COMPLETED` nor `OUTCOME_UNKNOWN` establishes RTC provenance. Before
deriving the write value, the communicator captures the current
`clock_state_generation`; the generation covers the complete write/read-back
episode. The communicator follows either result with `read_time()` before
another write. It compares a successful read with the same trusted clock
observation advanced to the read bracket midpoint using monotonic elapsed time.
The checked `rtc_readback_difference_us` must be no greater than the fixed
`time_sampling_margin_us`, and the actual difference is included in the exact
`rtc_verification_uncertainty_us` equation above. Only that matching read-back,
the episode's still-current `clock_state_generation`, current
`NETWORK_SYNCED` quality, a still-qualifying advanced network error and a
verification uncertainty strictly below the receiver UTC budget permit the
next-generation communicator-state commit to record verified RTC provenance.
Those conditions are checked again immediately before submitting the state
commit. A mismatch, invalid read, failed read-back or invalidated generation
creates no provenance, and an unknown write is never retried blindly. A
physical write may therefore succeed without becoming trusted; a later stable
eligible episode overwrites and verifies it.

### Pilot Linux backend and privilege boundary

The pilot backend uses a stable deployment-created device path such as
`/dev/rtc-ds3231`, backed by Linux's RTC-class driver for the I2C-connected
DS3231. It uses `RTC_RD_TIME` directly for reads. It does not use GPIO, SPI,
chrony's command socket, `hwclock` or raw `/dev/i2c-*` access while the kernel
RTC driver is bound to the device. Deployment validates that the stable path
names the expected driver/device and that the kernel driver reports the
DS3231 oscillator-stop condition as an invalid read.

Linux requires `CAP_SYS_TIME` for `RTC_SET_TIME`, but the receiver process must
not receive that capability: possession would let it bypass the ordered
chrony-step policy. Instead, `write_time()` executes one small native helper,
for example `/usr/libexec/cura-agrorum/ds3231-set`, using a fixed absolute path,
one canonical decimal `rtc_utc_s` argument, a fixed environment and no shell.
The helper:

- is owned by root, is not writable by the receiver identity and is executable
  only by the configured receiver group;
- has the file capability `cap_sys_time=ep` and no set-user-ID bit;
- uses one compiled-in `/dev/rtc-ds3231` path;
- independently validates exactly one UTC-second argument;
- performs no operation other than opening that device and attempting exactly
  one `RTC_SET_TIME` ioctl;
- never changes Linux system time, reads configuration, invokes another
  executable or performs read-back; and
- returns only its versioned fixed exit-status protocol.

The helper child receives `CAP_SYS_TIME` during `execve()`; the receiver parent
retains no permitted, effective or ambient capability. Device-node permissions
must still allow the receiver identity to open the RTC because `CAP_SYS_TIME`
does not bypass discretionary file permissions. Deployment verifies the helper
owner, mode, file capability, immutable path and supported result protocol
before runtime time quality or RTC provenance may become trusted.

Direct helper execution requires the receiver service's capability bounding
set to retain `CAP_SYS_TIME`, while granting it neither effectively nor
ambiently, and requires `NoNewPrivileges=no`; `NoNewPrivileges=yes`, a bounding
set without `CAP_SYS_TIME`, a `nosuid` helper filesystem or an incompatible
user namespace would prevent the file capability from taking effect. If a
future deployment requires `NoNewPrivileges=yes` for the receiver, a separate
socket-activated privileged helper may implement the same `Ds3231Control`
protocol without changing communicator policy.

### Recovery and scheduling

`Ds3231Control` deliberately exposes no reset operation. The DS3231 `RST` pin
is a processor-reset/power-fail signal and does not reset the device's RTC,
I2C interface or oscillator. Removing both main and backup power would destroy
the time state whose provenance the receiver protects. I2C-controller recovery
is a kernel/board transport concern and may affect other devices; it is not
communicator RTC policy.

The communicator performs a bounded read after receiver startup, after every
completed or uncertain write, and whenever a direct `RTC_HOLDOVER`
`ClockObservationV1` is due. The holdover observation period is capped by
`rtc_holdover_observation_period_cap_us` and shortened by the current UTC-error
budget. Failures update live health and are retried later through normal
scheduling; they never cause an unbounded local reset or retry loop. Offline
operation never writes the DS3231 and never copies it repeatedly into Linux
system UTC. While online, the communicator attempts a refresh on the first
stable RTC-write-eligible observation after startup or a step episode and then
at `network_rtc_refresh_period_us`; an earlier clock observation alone does not
force an RTC write. A future board with a dedicated, validated DS3231
power-control circuit would require a new board-specific recovery interface and
documentation revision, not reinterpretation of `write_time()`.

Unit tests inject a fake `Ds3231Control` and must cover every read status, every
valid write disposition/failure combination, deadline expiry before submission,
unknown outcome after submission becomes possible, matching and mismatching
read-back, and the rule that no failed or unverified episode commits RTC
provenance. They also cover a generation change during write, during read-back
and before state-commit submission; direct holdover observation midpoint, exact
half-bracket conversion and whole-second uncertainty; equality and one-unit
boundaries around the fixed read-back tolerance, advanced source-error
threshold and receiver UTC budget; progressively shortened offline read cadence;
and the rule that offline scheduling never invokes `write_time()`. Target-Pi
backend tests must additionally verify stable-device
selection, oscillator-stop handling, fixed helper invocation, file-capability
deployment, successful write/read-back and termination at the configured
deadline. Tests never require a fake implementation to open a device, execute
the helper or use real time.

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
    + time_sampling_margin_us
```

`remaining_correction_us` is the current displacement between chrony's
software NTP clock and Linux system UTC that still has to be removed by slew;
its sign describes direction, not confidence, so the error calculation uses
its absolute value. `root_distance_us` bounds the selected NTP clock's error
relative to its primary reference under the NTP delay and dispersion model.
The sampling margin covers uncertainty introduced after chrony formed the
tracking response. A small offset alone is insufficient: it says nothing about
an uncertain, delayed or stale reference. Every poll recomputes the complete
sum because the remaining slew, selected-source distance and response age may
all change.

`time_sampling_margin_us` is the fixed shared pilot allowance defined with the
deployment defaults. On this path the complete checked span from tracking-query
start through the end of the `adjtimex()` bracket must be no greater than that
margin; the charged margin covers that bounded response age, subprocess and
observation-bracket latency. Absolute-value overflow and every addition are
checked; failure invalidates the result. At or below the pilot 35,000,000 us
trust threshold a
qualifying sample may establish `NETWORK_SYNCED`. Above the 40,000,000 us step
threshold it requires an ordered `UNTRUSTED` transition before a step can be
submitted. Between the thresholds, including exactly 40,000,000 us, the
communicator retains current quality: it does not promote `UNTRUSTED` or
`RTC_HOLDOVER`, and it does not demote an existing `NETWORK_SYNCED` solely for
that value. A stale, unselected, unsynchronized or unreliable result cannot
establish network trust or authorize a step.

The communicator schedules a tracking poll no later than
`chrony_tracking_poll_period_cap_us`, with a short deadline and outside the
RX-to-ACK critical path. It schedules an earlier poll when the current trusted
observation's error horizon would otherwise expire first. Processing any
completed result, including an unavailable, expired or invalid result,
atomically updates live time policy and advances `clock_state_generation`
once. A qualifying result remains usable
only until the next required poll deadline; missing that deadline removes
`NETWORK_SYNCED` through the ordinary generation-changing transition.

The 35-second threshold governs `NETWORK_SYNCED`; it is intentionally too
loose for overwriting durable RTC provenance. An RTC refresh may start only
from a fresh trusted observation whose complete network error is at or below
`network_rtc_write_error_threshold_us` (five seconds initially). The stricter
threshold is also rechecked, together with the captured generation, before
provenance commit.

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
not expand this interface with arbitrary process output. Conversion follows
the `TIME` catalogue, or `CORE` for an implementation escape, in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md); the fail-closed time state
machine and in-memory health counters remain authoritative even when the
best-effort diagnostic cannot be admitted.

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
| `0` | `NONE` |
| `1` | `BUSY_TIMEOUT` |
| `2` | `SPI_FAILURE` |
| `3` | `UNEXPECTED_IRQ` |
| `4` | `TX_OUTCOME_UNCERTAIN` |
| `5` | `RX_PROFILE_RESTORE_FAILED` |
| `6` | `SET_RX_FAILED` |
| `7` | `STATUS_UNCONFIRMED` |
| `8` | `HARDWARE_UNREACHABLE` |

`NONE` means no recovery reason and has no counter slot.

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
| `5` | `UNAVAILABLE_IO` | Another global database, storage, access, locking or quarantine failure prevents admission |
| `6` | `UNAVAILABLE_INCOMPATIBLE_SCHEMA` | Schema version/fingerprint, immutable database metadata or a correctness-critical fixed entity is incompatible with this receiver build or configured group |

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

`UNAVAILABLE_IO` covers a lock/contention result, a temporary access failure
and another non-capacity, non-corruption global SQLite or storage failure. It
does not reinterpret low space, disk full, corruption, schema incompatibility
or an item-specific poison. An ordinary transaction publishes this state on
the first such classified global failure. Exact commit-outcome certainty is
decided before the unavailable reason: once `COMMIT` may have run, the pending
batch follows reconciliation rather than blind retry.

The failure classifier is closed and applies primary SQLite results, extended
results and available host `errno` together:

| Observed condition | Required state or path |
|---|---|
| Free space crossed the configured preventive threshold | `UNAVAILABLE_LOW_SPACE` |
| `SQLITE_FULL`, host `ENOSPC` or an equivalent confirmed capacity failure | `UNAVAILABLE_DISK_FULL` |
| `SQLITE_CORRUPT`, `SQLITE_NOTADB`, a corruption-specific extended result or failed configured integrity check | `UNAVAILABLE_CORRUPT`; preserve artifacts and require operator recovery |
| Schema version/fingerprint, immutable metadata, global identity or entity/schema compatibility failure | `UNAVAILABLE_INCOMPATIBLE_SCHEMA`; require compatible software or operator recovery |
| `SQLITE_BUSY`, `SQLITE_LOCKED`, temporary access failure, non-capacity `SQLITE_IOERR`, or another global database/storage result not classified above | `UNAVAILABLE_IO`; automatic paced recovery |
| Reproducible failure of one complete typed entity after every global/transient cause is excluded | Item-specific poison-isolation path; no global unavailable transition unless quarantine itself fails |
| `COMMIT` may have run | Preserve the applicable lease and frozen work; reconcile before retry regardless of the unavailable reason |

An unrecognized SQLite or host-storage result fails closed as
`UNAVAILABLE_IO`; it is never guessed to be item-specific poison. The
implementation's result-code classifier and tests must enumerate every result
exposed by the selected SQLite binding without changing these semantic rows.

### `AdmissionResult`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `RESERVED` | One queue slot was reserved while persistence was available |
| `1` | `PERSISTENCE_UNAVAILABLE` | Admission state prevented reservation |
| `2` | `QUEUE_FULL` | No entity slot was available |

`PERSISTENCE_UNAVAILABLE` and `QUEUE_FULL` complete the original admission
attempt. For either result, the communicator increments only the original
entity-kind/admission-result matrix cell. It does not allocate an additional
diagnostic sequence, construct a `DiagnosticV1` about that result or attempt
another `PersistQueue` reservation because of it. This rule applies to every
entity kind. If the original entity was itself a diagnostic, its already
allocated identity belongs only to that failed original attempt.

Invalid kinds, capacities, tokens or publication contents are interface violations,
not operational admission results.

### `PersistQueueEntityKind`

| Value | Name |
|---:|---|
| `1` | `MEASUREMENT_PROFILE` |
| `2` | `PROFILE_ONLY` |
| `3` | `RECEIVER_HEALTH_REQUEST` |
| `4` | `DIAGNOSTIC` |
| `5` | `CLOCK_OBSERVATION` |

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

Value `0` is reserved for private in-progress communicator state and is
forbidden in a published entity.
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

Value `0` is permitted only in private pre-TX communicator state. Every
published profile contains a terminal value.

### `PersistenceClassification`

The persistence thread derives this value while applying the reading-message
key and canonical-sample identity. It is not part of a queued
`MessageProfilingV1`.

| Value | Name |
|---:|---|
| `0` | `NOT_APPLICABLE` |
| `1` | `FIRST_SEEN` |
| `2` | `RETRANSMISSION` |
| `3` | `DUPLICATE_SAME_CONTENT` |
| `4` | `DUPLICATE_CONFLICT` |
| `5` | `MESSAGE_ID_CONFLICT` |

`RETRANSMISSION` means that a later occurrence has the same `node_id`,
`message_id`, decoded `sample_id` and exact authenticated frame as the first
profile owning the existing reading-message row. It does not claim why the
logical transport message repeated. For a new message ID, an existing canonical
sample with a byte-identical reading body is `DUPLICATE_SAME_CONTENT`; an
existing canonical sample with different contents is `DUPLICATE_CONFLICT`.
Rewrapping an unaccepted current reading as backlog is therefore
`DUPLICATE_SAME_CONTENT`, not a conflict.

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

```text
receiver_instance_id: bytes[16]
observation_sequence: u64
clock_state_generation: u64
sampled_at_monotonic_us: u64
sampled_at_utc_us: i64 or absent
step_discontinuity_boundary: bool
system_time_quality: u8
rtc_health: u8
```

The generated immutable logical entity crosses the queue directly. Its
relational binder stores absent UTC as SQL `NULL` and the boundary as a Boolean
column. UTC is absent exactly when `system_time_quality = UNTRUSTED` and
present exactly when quality is `RTC_HOLDOVER` or `NETWORK_SYNCED`.

`STEP_DISCONTINUITY_BOUNDARY` may be set only on the `UNTRUSTED` observation
created for one pending explicit chrony step. Ordinary quality-loss
observations clear it. The step cannot be submitted until the complete boundary
observation has been successfully published to the global FIFO. Once stored,
the boundary terminates the preceding correlation segment. Analysis assigns no
UTC to an event in the half-open monotonic interval from the boundary through,
but excluding, the first later trusted observation. That later observation
begins a new segment and is never used to extrapolate any event backward across
the step boundary, including an event before the boundary that lacked an
earlier trusted observation. This remains the conservative rule when step
submission fails or its outcome is unknown. If the process restarts before the
boundary becomes durable, its later volatile FIFO entries are lost and the new
durable receiver-instance start supplies the correlation boundary instead.

The communicator assigns `observation_sequence` before each enqueue attempt.
A failed attempt therefore creates a visible sequence gap after a later
observation succeeds. One observation is emitted after initial time and RTC
state establishment, on every quality or RTC-health transition, immediately
before an intentional clock step and at the configured periodic interval. A
periodic observation retains the current `clock_state_generation`; a
transition or step-boundary observation carries the newly advanced generation.

For a `NETWORK_SYNCED` observation, `sampled_at_monotonic_us` is the midpoint
of the bounded monotonic bracket around one read-only `adjtimex(modes = 0)`
call, and `sampled_at_utc_us` is the system time returned by that call. The
generation must be equal before and after the bracket, and a fresh acceptable
`ChronyTrackingResult` must satisfy the total-error entry/retention policy in
the Chrony control interface. Because deployment disables chrony's `rtcsync`,
Linux may return `TIME_ERROR` with `STA_UNSYNC` even when chrony is
synchronized; that expected pair is not a rejection. Other kernel metadata
that indicates clock interference or an invalid sample remains a rejection.

For an `RTC_HOLDOVER` observation, the communicator instead calls
`Ds3231Control.read_time()` and uses its bounded operation-start/finish
monotonic bracket. `sampled_at_monotonic_us` is the bracket midpoint and
`sampled_at_utc_us` is the midpoint of the returned whole UTC second:

```text
sampled_at_utc_us = rtc_utc_s * 1_000_000 + 500_000
```

The associated `rtc_read_whole_second_uncertainty_us` is exactly 500,000 us plus
the checked conservative physical half-bracket and the fixed
`time_sampling_margin_us`, as defined in the UTC-error-growth section. The read
is usable only when the generation is equal before and after the call. The
communicator then atomically applies the read's `RtcHealth` and any resulting
quality transition, advances the generation once when required, and puts that
resulting generation in the observation. RTC provenance must be valid,
resulting `RtcHealth` must be `PRESENT`, RTC age must be non-negative and the
complete `rtc_observation_error_bound_us` must be strictly less than
`receiver_utc_error_budget_us`. Otherwise quality becomes `UNTRUSTED`; an
expired RTC bound neither refreshes provenance nor authorizes a clock step. The
receiver never uses Linux `CLOCK_REALTIME` as the UTC source for an
`RTC_HOLDOVER` observation.

`UNTRUSTED` transition observations carry the transition boundary monotonic
time and zero UTC; they need neither an `adjtimex()` sample nor an RTC read.

Chrony slew adjusts `CLOCK_REALTIME` and `CLOCK_MONOTONIC` together, so it does
not invalidate this pair or a same-instance UTC derivation. An explicit step
does change their offset and is separated by the required published
`UNTRUSTED` boundary, global FIFO ordering and process-start correlation fence.
Observation spacing is not a substitute for the applicable network or RTC
absolute-error acceptance policy. For either trusted source, the communicator
computes the maximum safe observation-to-event interval from the observation's
error bound and `monotonic_elapsed_rate_bound_ppm`. It schedules
the next observation no later than the smaller of that interval and the
configured source-specific period cap. It starts the required poll/read,
sampling and queue-admission work early enough for their configured bounded
worst-case durations. If the remaining interval cannot contain that lead time,
or if it cannot obtain and publish the replacement before the safe interval
ends, it changes quality to `UNTRUSTED` at that calculated boundary and applies
the pending-boundary FIFO rule. This independent UTC-budget expiry may remove
`NETWORK_SYNCED` in chrony's 35-to-40-second hysteresis band even though the
tracking value alone would retain the current quality; it does not authorize a
step unless the separate step rule does.

`ClockObservationV1` does not store the calculated uncertainty. Publication as
a trusted quality asserts that the complete bound and its scheduled segment
horizon satisfy `receiver_utc_error_budget_us`; analysis may conservatively use
that configured ceiling. The pilot deliberately gives up the exact per-
observation uncertainty to keep the fixed entity and analysis contract simple.

The observation's identity is
`(receiver_instance_id, observation_sequence)`. Its `receiver_instance_id`
identifies the process that observed the correlation. Its Linux clock domain
is obtained from that instance's immutable `receiver_instances.linux_boot_id`
mapping. Analysis may use it only for an event carrying that same
`receiver_instance_id`; neither a shared Linux boot nor comparable monotonic
values permit cross-instance assignment.

Clock observations have ordinary FIFO importance. After every reservation
attempt returns, including a failed observation attempt, the communicator
increments exactly one
`persist_queue_admission_counts[CLOCK_OBSERVATION][result]` cell. A pending
transition boundary is offered before subsequent ordinary queue admissions; an
explicit chrony step additionally requires successful complete publication of
its exact `UNTRUSTED` boundary. Later ordinary publication may then continue
behind it. Global FIFO order prevents any later ordinary FIFO entity from
becoming durable first. If persistence cannot store the boundary, it retains
the non-quarantinable entry and all following entries, makes admission
unavailable and processes no later ordinary FIFO entity.

## `MessageProfilingV1`

`MessageProfilingV1` is the immutable packet-occurrence value shared by the two
packet-related queue entities. `received_at_monotonic_us` is `T0`.

Logical fields:

```text
receiver_instance_id: bytes[16]
occurrence_sequence: u64

received_at_monotonic_us: u64

received_frame_length: u16 or absent
received_frame: bytes[255] or absent

claimed_control: u8 or absent
claimed_domain: u8 or absent
claimed_node_id: bytes[8] or absent
claimed_message_id: u32 or absent
header_authenticated: bool8
decoded_sample_id: u32 or absent

rssi_dbm_x2: i16 or absent
snr_db_x4: i16 or absent
irq_status: u16 or absent
device_errors: u16 or absent

processing_result: u8
ack_selected: u8
ack_tx_result: u8
ack_frame: bytes[23] or absent

busy_wait_total_us: u64
busy_wait_max_us: u64
busy_wait_count: u32
busy_timeout_count: u32
last_busy_timeout_opcode: u8 or absent

t1_handler_started_monotonic_us: u64
t2_packet_copied_monotonic_us: u64 or absent
t3_authentication_completed_monotonic_us: u64 or absent
t4_set_tx_attempted_monotonic_us: u64 or absent
t5_tx_done_monotonic_us: u64 or absent
t6_set_rx_issued_monotonic_us: u64 or absent
```

Optional Python fields use `None`; `message_profiles` projects them to nullable
columns. The exact `received_frame` bytes remain separate evidence of the
incoming protocol packet.

`T0` and `T1` are mandatory. `rssi_dbm_x2` and `snr_db_x4` must either both be
present or both be absent. The queued profile and stored profiling row contain
no UTC, `SystemTimeQuality` or `RtcHealth`. Analysis may correlate their
monotonic fields with `ClockObservationV1` without updating the row.

`received_frame_length` and `received_frame` are either both absent or both
present. When present, the length is from `0` through `255`, the byte string has
exactly 255 bytes, its prefix through that length is the received frame and its
remaining tail is zero. This fixed-capacity entity field is independent of
queue accounting: the queue still stores only the existing object reference
and assigns the enclosing entity one slot.

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

ACK-frame presence is equivalent to `ack_selected != NONE`. When no ACK is
selected, the frame is absent and `ack_tx_result = NOT_APPLICABLE`. When an ACK
is selected, `ack_frame` contains the exact 23 protocol bytes whether TX later
succeeds, fails or is suppressed.

When multiple event timestamps are present, their values must follow the
ordering permitted by the receive-to-ACK flow. Missing error-path timestamps
remain explicitly absent rather than being reconstructed.

`persistence_classification` is deliberately absent. Persistence creates the
stored profiling row by adding its derived classification without mutating the
queued value.

## Queue entity values

### `MeasurementProfileUnitV1`

```text
candidate: AuthenticatedReadingCandidateV1
profile: MessageProfilingV1
```

`AuthenticatedReadingCandidateV1` is:

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

The authenticated reading candidate and profile form one queue and SQLite
transaction unit and must never be split. The candidate is a pre-SQL value,
not a prospective `ReadingMessageRowV1`: canonicality and first-occurrence
provenance do not exist until persistence consults the database.

### `ProfileOnlyUnitV1`

```text
profile: MessageProfilingV1
```

It represents one complete packet-occurrence profile without an application
candidate. It covers unknown nodes, authentication failures, malformed or
unsupported packets, wrong-direction packets and radio events without a usable
application frame.

The common typed profile permits frame and identity fields to be absent; a
separate radio-event entity is unnecessary.

### `ReceiverHealthRequestV1`

This is the immutable communicator-owned portion of a receiver-health row.

```text
receiver_instance_id: bytes[16]
health_sequence: u64
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
last_time_quality_transition_monotonic_us: u64 or absent
last_rtc_health_transition_monotonic_us: u64 or absent

chrony_step_command_results: u64[3]
rtc_write_results: u64[3]
rtc_write_readback_verified_count: u64
rtc_write_trust_invalidated_count: u64

persist_queue_admission_counts: u64[5][3]
```

The recovery array is ordered by `RadioRecoveryReason` values `1` through `8`.

`chrony_step_command_results` is ordered as `SUBMITTED`, `NOT_SUBMITTED`,
`OUTCOME_UNKNOWN`. Exactly one cell advances after every returned
`apply_pending_correction_by_step()` result; their sum is the number of chrony
step-command attempts, so no overlapping `chrony_step_required_count` is
stored. A policy decision that never reaches the adapter because its durable
step boundary cannot be established is not a command attempt.

`rtc_write_results` is ordered as `COMPLETED`, `NOT_APPLIED`,
`OUTCOME_UNKNOWN`. Exactly one cell advances after every returned
`write_time()` result, and their sum is the number of RTC write attempts. Every
such attempt must have started under `NETWORK_SYNCED` and the stricter RTC-write
source-error threshold; violating that precondition is an interface failure,
not another counter category. `rtc_write_readback_verified_count` advances
when a completed or uncertain write is read back successfully and matches the
intended UTC while the captured generation remains valid.
`rtc_write_trust_invalidated_count` advances when a possibly applied write
cannot establish provenance because `clock_state_generation`, time quality or
the stricter network-error condition changed before the provenance commit. It
advances at most once for one write episode even if multiple postchecks fail.
Neither counter asserts that the subsequent communicator-state commit became
durable.

`persist_queue_admission_counts` is a cumulative reservation-attempt matrix.
Its rows are ordered by `PersistQueueEntityKind` values `1` through `5`, and
its columns are ordered by `AdmissionResult` values `0` through `2`. After one
`try_reserve_one()` call returns an operational result, the communicator
increments exactly one corresponding cell; the queue does not own or mutate
the matrix.
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

For a periodic health attempt, the communicator advances `health_sequence`,
calls `try_reserve_one(RECEIVER_HEALTH_REQUEST)`, and increments the returned
matrix cell. On `RESERVED`, it then constructs one complete immutable request
from the updated matrix and the other communicator-owned observations and
publishes that object against the reservation.
Consequently, a successfully admitted health request includes its own
`RESERVED` attempt. A failed health attempt appears only in a later
successfully persisted health request. Publication has no operational failure
result and therefore no matrix column.

The persistence thread does not mutate this request. It samples its own and
host observations once, creates a separate complete `ReceiverHealth` value and
keeps that enrichment stable across SQLite retry. The row's Linux boot ID is
obtained when needed by joining its `receiver_instance_id` to
`receiver_instances`; it is not copied into `ReceiverHealthV1`.

### `ReceiverHealthV1`

`ReceiverHealthV1` is the persistence-created, SQLite-bound value formed from
one immutable `ReceiverHealthRequestV1`. It is not a queue entity. It retains
every communicator-owned logical request field and adds the following
mandatory fields:

```text
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
including retries and isolation attempts. `batch_entities_committed` advances
only for queue units whose ordinary SQLite effects committed; quarantined units
advance the quarantine counters instead. Checkpoint counters cover explicit
receiver-initiated checkpoints, not SQLite's internal page writes.

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

Its logical fields, ownership, replay rules, severity and operation
assignments, domain/error catalogues and fixed context schemas are normative in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md). `PersistQueue` carries the
immutable object with its kind and schema version and never interprets the
domain-local context.

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
on the atomicity of an individual container operation. The two owners run in
ordinary `threading.Thread` OS threads under the rationale and limitations in
[`ARCHITECTURE.md`](ARCHITECTURE.md#why-the-pilot-uses-threadingthread).

### Entity specifications and storage

The Python interface uses an immutable entity specification:

```python
@dataclass(frozen=True, slots=True)
class PersistQueueEntitySpec:
    kind: PersistQueueEntityKind
    schema_version: int
```

For the pilot, every accepted specification has `schema_version = 1`. An
unknown kind, version or pair is an interface error. A specification identifies
the logical contract for persistence and quarantine; it does not supply a size
and the queue does not dispatch on it.

The queue constructor accepts `capacity_entities` in `1..500`; production uses
500. It allocates fixed-length parallel storage for payload references,
specifications and reservation-token references during construction and never grows or
replaces those backing arrays. Smaller capacities exist for focused tests, not
as a second production sizing policy.

A successful reservation exclusively claims the currently empty tail slot but
does not construct an entity. The communicator constructs the complete
immutable typed object in its own local state and publication assigns that
existing object reference into the reserved slot. The queue performs no
serialization, copying, recursive size inspection or kind-specific building.
An unpublished reserved tail is invisible to batch claims. Because there is
only one producer and at most one outstanding reservation, no published entry
can appear behind it.

Capacity is only the number of occupied slots. One reserved or published entry
occupies one slot regardless of its entity kind or Python representation.
There is no byte-capacity field, per-kind charge or claim-batch byte limit. The
500-slot bound is a deliberately conservative pilot limit; measuring recursive
object memory or calibrating it against RSS is deferred.

### Persistence-admission publication

The persistence thread updates the queue's admission snapshot through:

```python
publish_admission_state(
    snapshot: PersistenceAdmissionSnapshot,
) -> None
```

The queue validates the snapshot generation and stores the complete immutable
snapshot while holding the same lock used for capacity reservation. The
persistence thread remains the only publisher. Each publication uses exactly
the preceding generation plus one and a nondecreasing
`changed_at_monotonic_us`; the state must be a `PersistenceAdmissionState`.
A replay, gap, time regression or invalid state is an interface error and
leaves the preceding snapshot unchanged.

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
used_entities_before: u64
capacity_entities: u64
```

`used_entities_before` and `admission_snapshot` are sampled in the same critical
section as the admission decision. The count is the number of occupied slots
before this call's successful reservation; because the SPSC queue permits only
one outstanding reservation, another call made while that reservation exists
is an interface violation rather than a second admission result.

The operation applies checks in this order while holding the queue lock:

1. reject use after producer closure as an interface error;
2. require a supported exact kind/version specification and no outstanding
   producer reservation;
3. if admission is not `AVAILABLE`, return `PERSISTENCE_UNAVAILABLE` without
   occupying a slot;
4. if `capacity_entities` is already occupied, return `QUEUE_FULL` without
   changing the ring; and
5. register the preallocated tail slot as unpublished and return `RESERVED`
   with its reservation handle.

Failure to allocate Python memory does not return `RESERVED` and leaves all
queue counters and contents unchanged. `MemoryError` is a process-level failure,
not an operational `QUEUE_FULL` result.

### Reservation ownership

`PersistQueueReservation` is an opaque, unique and non-copyable handle exposing:

```python
reservation.token
reservation.spec
reservation.publish(entity: object) -> None
reservation.cancel() -> None
```

There is deliberately no queue-owned builder or separate `seal()` operation.
While the reservation is outstanding, all entity construction state remains
communicator-owned. The queue does not know which fields are pre-TX fields,
which are terminal fields, or when the communicator commits to a protocol
outcome.

The only queue-level transitions are:

```text
RESERVED -> PUBLISHED
RESERVED -> CANCELLED
```

`publish(entity)` atomically:

- verifies the live token and reservation specification;
- stores one strong reference to the already complete entity in the reserved
  slot without interpreting its fields;
- invalidates the reservation handle;
- changes the slot from reserved to published without changing occupied count;
- makes the entry visible at the FIFO tail; and
- wakes the persistence thread.

Publication performs no admission-state or capacity check and remains valid if
persistence became unavailable or the producer side was closed after the
reservation succeeded. It performs no backing-store growth, payload copying or
entity serialization. A correctly used reservation therefore cannot fail due
to queue pressure.

`cancel()` atomically releases the unpublished slot, invalidates the handle and
wakes the persistence thread if this completes a closed queue's drain
condition. It is forbidden after publication.

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

Publication transfers logical ownership, not Python alias reachability. The
queue holds the strong reference until durable acknowledgement. Queue entities
must therefore be deeply immutable: frozen slotted dataclasses composed only
of other immutable admitted dataclasses, tuples, `bytes`, enums, integers,
Booleans and declared `None` values. Producer mutation or semantic reuse after
publication is an interface violation even though Python cannot revoke an
alias. The persistence thread receives the exact same object (`is` identity),
not a copy.

### Queue snapshot

```python
snapshot() -> PersistQueueSnapshot
```

The returned immutable snapshot contains:

```text
admission_snapshot
capacity_entities
reserved_entities
published_entities
claimed_entities
closed
closed_and_drained
```

The accounting invariants are:

```text
used_entities = reserved_entities + published_entities

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
    max_entities: int,
) -> PersistQueueBatchLease | None
```

`max_entities` must be positive. An invalid limit is a configuration error. If
no published entry is available, the operation returns `None`.

Otherwise it claims the longest complete FIFO prefix up to that limit.
It never splits an entity or a `MeasurementProfileUnitV1`. A nonempty queue
therefore produces at least one entry. Only one batch lease may exist; an
overlapping claim is an interface error. Reservations and publication may
continue at the tail while a batch is claimed.

The lease exposes an immutable borrowed entry tuple:

```text
entries: tuple[PersistQueueEntryView, ...]
```

Each entry view exposes its reservation token, specification and immutable
payload reference. Claiming may allocate persistence-thread view and tuple
metadata but does not copy, serialize or inspect the payload. Persistence-
derived values such as duplicate classification and enriched receiver-health
fields remain separate work data and never mutate a claimed entity.
Python cannot revoke a retained entry-view or payload alias after the lease
ends; using either beyond the active lease is an interface violation.

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

`acknowledge_durable()` requires exactly one disposition per entry in order. The
persistence thread may acknowledge only after every entry has reached the
reported durable outcome. For example, a batch containing valid `X`, poisoned
`Y` and valid `Z` may be removed only after `X` and `Z` commit to SQLite and
`Y` is durably quarantined. Failure of either durable operation retains the
complete batch. Persistence must retain completed per-entry work or make it
idempotent so retry does not invalidate an already durable disposition.

Acknowledgement atomically validates that the lease still names the claimed
queue-head prefix, removes the entire prefix, releases its entity slots,
invalidates the lease and wakes any shutdown waiter. The queue trusts
the persistence thread's durability assertion; it does not perform I/O to
verify it.

`release_for_retry()` removes nothing. It clears the active claim, invalidates
the lease and leaves every entry in its original FIFO position so persistence
may retry or claim a narrower batch. It also sets the shared wakeup hint so the
still-published head remains visible to the surrounding scheduler. Exiting a batch-lease context manager
without successful acknowledgement performs `release_for_retry()`.

### Transient and global failure recovery

A definite pre-commit lock/contention, temporary access or other
non-capacity, non-corruption global SQLite or storage failure produces
`NOT_COMMITTED`. Persistence rolls back when necessary, retains the complete
original queue units and every frozen persistence-derived value, calls
`release_for_retry()` at a safe boundary and publishes `UNAVAILABLE_IO`. It
does not isolate, quarantine or reconstruct an entity.

Recovery uses an interruptible monotonic backoff. The pilot defaults are an
initial physical delay of 250 milliseconds, doubled after each unsuccessful
recovery attempt to a five-second cap using
`minimum_wait_monotonic_us()`. There is no maximum attempt count. Once capped,
the persistence thread continues low-frequency recovery attempts so an
operator-corrected mount, access or contention condition can recover without a
process restart. The backoff resets only after the pending batch transaction
commits or exact reconciliation proves its complete durable effect.

Every SQLite connection uses a finite 250-millisecond pilot busy timeout for
ordinary, reconciliation and quarantine lock acquisition. A control operation
shortens that bound when its remaining absolute deadline is earlier. SQLite's
busy timeout bounds one attempt; it does not replace the persistence-level
backoff between attempts.

At each retry deadline persistence reopens the database when required,
re-establishes and verifies the applicable connection, durability, schema and
metadata invariants, and retries the original FIFO head with its frozen work.
It returns to `AVAILABLE` only after those checks and the pending transaction
succeeds. `OUTCOME_UNKNOWN` retains its active lease and frozen values and must
complete exact reconciliation before this ordinary retry path may run. If a
reconciliation attempt encounters another classified global transient failure,
persistence remains `UNAVAILABLE_IO`, retains that active lease and uses the
same capped backoff for the next reconciliation attempt; it does not release or
blindly retry the batch.

A definite transient/global failure while committing a frozen quarantine row
uses the same unavailable state and backoff but retains the active batch lease,
the isolation result and the exact intended quarantine row. An ambiguous
quarantine commit retains those values and reconciles the exact row under the
same paced scheduler. `release_for_retry()` above applies to a definitely
uncommitted ordinary batch only, not to these active reconciliation or
quarantine dispositions.

The shared wakeup event interrupts this backoff for a control command or
shutdown. After any such wake, persistence checks all work predicates but does
not run the ordinary retry before its stored retry deadline merely because the
event was set. A control command is serviced at the next safe boundary before
that retry. Corruption and incompatible schema never enter this automatic
recovery path.

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
`DiagnosticV1`, comparison includes every queued field; persistence adds no
boot-identity column.

For `MeasurementProfileUnitV1`, the stored profile classification is immutable
and authoritative during replay. Persistence validates it against the
reading-message key and canonical-sample relation specified by the SQLite
schema contract; it never recomputes a committed occurrence's classification
from the now populated table. A matching already committed row may be a no-op
in a retry transaction while an absent required row is inserted. Queue
acknowledgement is legal only after confirmed commit or reconciliation proves
every entity's complete durable effect.

### Wakeup and controlled closure

The queue is constructed with a shared `threading.Event` used to wake the
persistence thread. Successful publication, closure, retry release and
completion of a closed drain set this event; the queue never clears it. The
separate persistence control channel uses the same event, allowing the
persistence loop to wait on queue work, synchronous control operations and
shutdown without polling independent condition variables.

The event is only a wakeup hint. After waking, the persistence thread checks
all work sources and takes a queue snapshot under their respective contracts.
It retains the configured batch threshold, flush deadline and ordinary-retry
deadline; a wakeup does not require an immediate undersized transaction or an
early retry. Idle and retry-backoff waits use `Event.wait()` with the duration
to the nearest applicable deadline, never an uninterruptible sleep. Clearing
the event followed by rechecking the control-mailbox, queue, shutdown and timer
predicates must be ordered so concurrent publication or command submission
cannot create a lost wakeup.

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

Invalid entity specifications or capacities, stale or foreign tokens, use
after publication or cancellation, double publication or cancellation,
overlapping batch claims, disposition-count mismatch and
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
creates a private immutable command, submits it to the in-memory control
mailbox, sets the shared persistence-thread wakeup event and waits without
polling on the command's private completion event for the remaining deadline.
The single communicator thread cannot initiate another synchronous call while
its current public call is waiting. After a caller timeout, however, a later
reconciliation command may be queued behind the unresolved or cancelled
private command; serialization prevents it from overtaking that command. No
public control command crosses `PersistQueue`.

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

Control priority is cooperative. The persistence scheduler checks the mailbox
after an interruptible wait, before dispatching an ordinary batch, and after
each commit, rollback or reconciliation attempt. It never calls a control
operation reentrantly from an open ordinary transaction. Mailbox insertion and
the decision to dispatch one ordinary attempt use the same short scheduler
lock as their ordering point. If submission wins while the control bypass is
available, the control command runs first. If ordinary dispatch wins, that one
bounded attempt reaches its safe boundary and the control command then runs
before any retry or later batch. The scheduler lock is not held during SQLite
or filesystem I/O.

At most one control command may bypass an already-due ordinary attempt at a
scheduler boundary. After that command completes, one due ordinary attempt is
given a dispatch turn before a later control command; when no ordinary attempt
is due, control work may continue. This bounded alternation preserves control
priority without allowing sequential submissions from the communicator to
starve FIFO persistence, and it prevents continuous ordinary work from starving
a pending control command.

### Control-command deadlines and cancellation

Each private command has a lock-protected process-local execution state:

```text
QUEUED
RUNNING_PRECOMMIT
COMMIT_MAY_HAVE_RUN
DONE
CANCELLED
```

The persistence thread checks the absolute deadline before starting a command,
at bounded operation boundaries and immediately before `COMMIT`. Applicable
SQLite lock waits are configured not to exceed the command's remaining time.
Immediately before invoking `COMMIT`, persistence atomically checks that no
pre-commit cancellation was requested and advances the command from
`RUNNING_PRECOMMIT` to `COMMIT_MAY_HAVE_RUN`.

If the private completion wait reaches its deadline, the communicator resolves
the command state under the same command lock:

- `QUEUED` becomes `CANCELLED`; persistence skips it and no effect occurred;
- `RUNNING_PRECOMMIT` receives a cancellation request that forbids crossing
  `COMMIT`; the worker completes any required rollback and no durable effect
  occurred;
- `COMMIT_MAY_HAVE_RUN` cannot be cancelled and a mutating operation reports
  its command-specific unknown-outcome disposition; and
- `DONE` returns the already installed immutable result.

A read-only configuration or communicator-state load has no unknown durable
outcome and returns its command-specific `DEADLINE_EXCEEDED` result. A mutating
command cancelled before `COMMIT` returns its definite not-installed or
not-committed disposition with `DEADLINE_EXCEEDED`. A timed-out command remains
in serialization order until the persistence thread has skipped it, rolled it
back or reached its terminal database outcome; a later reconciliation load
cannot overtake it. The caller-side deadline is the final bound when an SQLite
call or kernel I/O cannot itself be preempted.

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
returns that immutable identity so the communicator can reason about the
current Linux clock domain without filesystem access. Queue and persisted
entities identify their source only by `receiver_instance_id`; the canonical
boot mapping is the current `receiver_instances` row. Failure to obtain the exact
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
    state_format_version INTEGER NOT NULL CHECK (state_format_version = 1),
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
reserved_0: u16

rtc_verified_by_receiver_instance_id: bytes[16]
network_utc_at_rtc_verification_us: i64
rtc_readback_utc_us: i64
rtc_verification_uncertainty_us: u64
rtc_drift_bound_ppm: u32
reserved_1: u32

rolling_window_us: u64
tx_airtime_budget_us: u64
bucket_width_us: u64
bucket_charge_limit_us: u64
bucket_expiration_guard_us: u64

airtime_snapshot_utc_us: i64
bucket_count: u16
reserved_2: u16
reserved_3: u64
```

The generated logical Python `CommunicatorStateV1` exposes
`rtc_provenance: RtcProvenanceV1 | None`, not `validity_mask` or the
representation-only version, length and count fields. `RtcProvenanceV1` groups
the five non-reserved RTC-provenance values shown in the header.
`CommunicatorStateV1.buckets` is a tuple of exactly 62
`TxAirtimeBucketV1` values. Durable airtime allowance exists only as bucket
charge; there is no second collection. The canonical-BLOB codec consumes and derives the version, encoded
length, mask, fixed count and reserved zeros so the binary
representation remains deterministic; the mask is not a separate relational
column.

Validity bit 0 selects the RTC-provenance block; bits 1 through 15 are reserved
and zero. When provenance is absent, its identifier and timestamps are zero.
Its uncertainty and drift bound are also zero. `reserved_1` is always zero.
When provenance is present, `rtc_verification_uncertainty_us` is the complete
conservative error bound established by the verified write/read-back episode
and `0 < rtc_drift_bound_ppm < 1_000_000`. The stored drift bound, rather than
a possibly changed runtime default, governs that provenance for its complete
lifetime. `rtc_verified_by_receiver_instance_id` must resolve to exactly one
immutable `receiver_instances` row; that row supplies the verification Linux
boot ID for audit and analysis. A missing lifecycle row makes the state invalid
rather than permitting a second stored boot identity.
The last-observed quality and health are diagnostic snapshots and are never
restored as current-instance observations.
`network_utc_at_rtc_verification_us` is obtained from the fresh trusted clock
observation used for the RTC write/read-back episode, advanced only by measured
monotonic elapsed time to the successful read-back bracket midpoint.
`rtc_readback_utc_us` is the midpoint of the whole UTC second returned by that
read-back. A later direct RTC read computes non-negative
`rtc_age_since_verification` from its whole-second midpoint minus
`rtc_readback_utc_us`; a negative or overflowing difference invalidates
provenance. `rtc_verification_uncertainty_us` is exactly the sum of the advanced
network error, actual accepted read-back difference and direct read uncertainty
defined in the UTC-error-growth section. It is strictly less than the configured
receiver UTC budget. No load reconstructs that value from changed runtime
defaults; the persisted uncertainty and drift bound govern the provenance for
its complete lifetime.

`rolling_window_us`, `bucket_width_us` and `bucket_expiration_guard_us` are
physical policy durations. Their durable UTC calculations use the stored values
directly. Runtime minimum no-TX waits use `minimum_wait_monotonic_us()`. An
active bucket grant ends at its durable bucket boundary mapped into the current
Linux boot and shortened with `maximum_lifetime_monotonic_us()`; it does not
remain spendable merely because UTC later steps backward.

Every valid V1 value has an authoritative bucket ledger.
`airtime_snapshot_utc_us` is canonical UTC derived for the
snapshot's monotonic construction time from the communicator's latest live
trusted `ClockObservationV1` correlation under `NETWORK_SYNCED` or valid
`RTC_HOLDOVER`. It is not a direct per-state `CLOCK_REALTIME` read. Every
nonempty bucket expiration is later than that snapshot.

The V1 blob contains exactly 62 bucket slots. Each slot is this 16-byte entry:

```text
charged_airtime_us: u64
expires_at_utc_us: i64
```

Nonempty entries precede empty entries and are sorted by strictly increasing
expiration. Charges with the same expiration are combined. The expiration is
exactly the checked sum of the logical bucket end, `rolling_window_us` and
`bucket_expiration_guard_us`. Consequently, the bucket end is recovered by
checked subtraction. Nonempty bucket expirations belong to one grid: the
difference between any two is a positive integer multiple of
`bucket_width_us`. Every unused trailing slot is encoded
canonically as `charged_airtime_us = 0` and `expires_at_utc_us = 0`; no nonempty
entry may follow one. The runtime ring's monotonic origin, physical start index
and cached `total_used` are not persisted.

Each nonempty charge is positive and no greater than
`bucket_charge_limit_us`. A loaded charge is never spendable by the loading
process: it is a conservative baseline that may represent historical TX, an
uncertain TX or unused write-ahead allowance from an earlier process. It stays
in its original logical bucket until expiration.

To obtain a grant, the communicator selects the grid bucket whose end is the
first bucket boundary strictly later than the trusted snapshot. If a retained
grid exists, empty elapsed intervals are skipped without relocating an older
charge. If every prior entry expired or the valid ledger is empty, the
communicator may start a new grid with a bucket end exactly one
`bucket_width_us` after the snapshot. It computes with checked arithmetic:

```text
bucket_headroom = bucket_charge_limit_us - current_bucket_charge
global_headroom = tx_airtime_budget_us - sum(all unexpired bucket charges)
grant = min(bucket_headroom, global_headroom)
```

The communicator submits a complete next-generation value that adds `grant` to
the selected bucket. Only that durably acknowledged increment is spendable by
the current process, and only until the mapped monotonic bucket deadline. Its
runtime record contains the bucket expiration, loaded baseline, acknowledged
increment, unspent increment, state generation and monotonic deadline; none of
that process-local ownership metadata is serialized.

At the bucket boundary, or earlier when a new grant is needed, settlement keeps
the loaded baseline unchanged and replaces only the current process's
provisional increment with airtime actually or possibly transmitted under that
increment. A definite failure before `SetTx` can take effect permits reclaiming
the corresponding unused part. The same atomic next-generation commit may
precharge the next grid bucket. No later TX uses the next increment before that
commit is acknowledged. An unknown commit outcome suppresses TX until an exact
load reconciles the installed generation and bytes.

On restart, a previous process's entire increment is just part of the loaded
unspendable charge. The new process may top up the same bucket only when it is
still the selected current grid bucket and both headrooms permit the addition;
otherwise it adds a charge to the newly selected bucket. It never moves the old
charge to the current bucket. The previous process's write-ahead increment is
therefore attributed to the bucket that process selected and persisted, not to
the new process merely because it loaded the file.

Missing or structurally corrupt state does not wait for unknown history to age
and does not start empty. Once trusted canonical UTC is available, the receiver
constructs and durably installs a generation-one synthetic worst-case ledger
before any TX. Let:

```text
q = tx_airtime_budget_us // bucket_charge_limit_us
r = tx_airtime_budget_us % bucket_charge_limit_us
```

The chronological charges are one oldest remainder `r` when `r != 0`, followed
by `q` full `bucket_charge_limit_us` charges. If `r == 0`, they are just `q`
full charges. Their bucket ends are spaced by `bucket_width_us`; the newest end
is one full bucket width after the recovery snapshot. Thus the pilot values
`B = 36 s`, `Y = 8 s`, `X = 60 s` produce `[4, 8, 8, 8, 8]` seconds in the five
most recent possible buckets. The complete budget is initially unavailable,
then returns only as those conservative charges expire. The synthetic entry
count and all time arithmetic must fit the V1 capacity and integer ranges.

The complete length is:

```text
encoded_length = 128 + bucket_count * 16
               = 1120
```

`bucket_count` is exactly 62 in V1. Changing it requires a new state encoding
version once a deployed database must remain readable.

For every valid state:

```text
sum(bucket charges) <= tx_airtime_budget_us
```

Expired entries are absent, arithmetic is checked for overflow and all reserved
bytes are zero. `total_used` is recomputed on load. The five stored
airtime-policy parameters must exactly match active deployment policy; mismatch
never silently reinterprets state. Policy validation also requires
`0 < bucket_width_us`, `0 < bucket_charge_limit_us <= tx_airtime_budget_us`, and
enough fixed slots both for the maximum unexpired grid span and for the
synthetic worst-case ledger.

### State-row conditions and loading

`CommunicatorStateCondition` has these members:

| Name | Meaning |
|---|---|
| `NONE` | No state condition |
| `MISSING` | The singleton row is absent; use conservative generation zero |
| `CORRUPT` | SQL envelope, digest, canonical encoding or structural validation failed |
| `UNSUPPORTED_VERSION` | The digest-protected state value belongs to an unsupported format and requires guarded no-TX recovery |
| `POLICY_MISMATCH` | The stored airtime policy differs from active deployment policy |

Classification follows this exclusive decision tree after database identity,
schema metadata and whole-database integrity validation have succeeded. The first
applicable outcome is final:

1. If the singleton is absent, return `MISSING`.
2. Validate the SQL row envelope and stored digest before interpreting a format.
   The relation must contain exactly one `singleton_id = 1` row; every SQL value
   must have its required storage class, presence, range and fixed length; and
   `state_sha256` must equal SHA-256 of the exact `state_blob`. Any failure is
   `CORRUPT`.
3. Read the digest-protected common `state_format_version` prefix from the blob
   and require it to equal the SQL `state_format_version`. A missing prefix or
   mismatch is `CORRUPT`. If that equal version is not supported by this build,
   return `UNSUPPORTED_VERSION` without attempting version-specific structural
   or policy validation.
4. For a supported version, decode and validate the complete canonical value,
   including encoded length, SQL/blob generation equality, reserved fields,
   ranges, ordering, lifecycle references, checked arithmetic and all other
   structural invariants. Any failure is `CORRUPT`.
5. Only a structurally valid supported value is compared with the active five
   airtime-policy parameters. Any difference is `POLICY_MISMATCH`; otherwise the
   condition is `NONE` and the state is `LOADED`.

Only that primary condition is returned and used for recovery. In particular, a
bad digest remains `CORRUPT` even when untrusted bytes resemble an unsupported
version, and a structural failure remains `CORRUPT` even when decodable policy
fields also differ. Unsupported formats are not decoded far enough to report a
secondary structural defect or policy mismatch.

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

Every non-`NONE` condition produces conservative generation-zero runtime state
and suppresses TX. `MISSING` and `CORRUPT` become usable only after trusted UTC
is available and the exact synthetic worst-case generation-one ledger defined
above is durably installed. A corrupt relation is preserved in the same atomic
replacement transaction.

`UNSUPPORTED_VERSION` and `POLICY_MISMATCH` are not decoded or reinterpreted as
current-policy history. Starting only after the communicator has made the
transmitter known unable, the current process suppresses TX continuously for
`minimum_wait_monotonic_us(active rolling_window_us)`. A process restart loses
that proof and restarts the complete wait. After it completes and trusted UTC
is available, persistence atomically archives the exact rejected singleton and
installs a valid current-policy generation-one empty ledger. Calling that
recovery commit before the no-TX wait completes is a caller invariant failure;
persistence cannot independently observe radio silence. Whole-database
corruption remains governed by the common SQLite corruption/I/O policy and is
not repaired by state-row recovery.

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

A missing baseline accepts only the exact valid synthetic generation-one value
derived from the request's trusted snapshot and the active policy. A corrupt
baseline accepts that same form of generation one only in one SQLite
transaction that:

1. inserts one append-only `quarantined_communicator_states` row for every
   observed row in the invalid `communicator_state` relation, preserving every
   exact rejected SQL value;
2. removes the invalid relation contents and inserts the requested valid
   generation-one singleton; and
3. commits both effects atomically.

An unsupported-version or policy-mismatch baseline accepts only a valid
current-policy generation-one empty ledger, and only after the communicator's
complete no-TX recovery precondition above. Persistence archives the exact old
singleton and installs the replacement in the same transaction used for corrupt
state. This recovery is an explicit caller-requested state transition, not an
attempt to upgrade or reinterpret the old blob. In every recovery case,
`OUTCOME_UNKNOWN` is reconciled by loading and comparing the exact requested
generation-one bytes before TX can resume.

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
identity already exists. That durable row is also an implicit ordinary
correlation boundary at `started_at_monotonic_us`: it closes the preceding
instance's segment before this instance can publish ordinary work. A start UTC
and source clock-observation identity may be derived from the first qualifying
later observation from this same instance when no explicit step boundary
intervenes; neither result nor time quality and RTC health is duplicated into
the lifecycle row or request.

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

### Host generation and pilot schema epochs

The checked-in schema pipeline is:

```text
receiver/schemas/receiver_enums.json
    -> generated enum SQL catalogues
receiver/schemas/receiver_entities.json
    -> reusable logical records, persisted entities, binders, and table DDL
receiver/db/schema_source.sql + generated catalogues
    -> receiver/db/schema.sql
exact schema.sql bytes
    -> generated Python schema constants
```

`receiver/tools/generate.py` performs deterministic enum, catalogue, entity,
binding, schema-assembly and fingerprint generation and supports `--check` for
freshness validation. `schema_source.sql` and both JSON manifests are
handwritten; `schema.sql`,
`cura_receiver/generated/receiver_enums_generated.py`, and
`cura_receiver/generated/receiver_entities_generated.py` are generated and
must not be edited. Resources are resolved relative to the installed receiver
package, never the process working directory.

`schema_source.sql` owns the exceptional `database_metadata`,
`receiver_instances` and `quarantined_communicator_states` tables. Generation
executes the fully assembled schema in memory and verifies that every foreign
key names an existing parent column set backed by a complete primary or unique
key; `--validate-only` performs the same closure check without writing outputs.

The pilot deliberately has no migration runner, `schema_migrations` table,
downgrade path or `PRAGMA user_version` contract. A schema change increments
the manifest's `database_schema_version`, regenerates the checked-in artifacts
and starts a new database schema epoch. Deployment stops the receiver,
preserves the preceding database and its WAL/shared-memory state through a
validated archive or backup procedure, creates a fresh database, and installs
it only after validation. Startup never drops, upgrades or rewrites an
incompatible database. Complete automatic retention applies inside one active
epoch; earlier epochs remain read-only archives.

SQLite `application_id = 0x43555252` (`CURR`, decimal `1129665106`) remains the
separate SQLite-header file-type marker. `database_metadata` is the single
schema compatibility authority and contains exactly one immutable installed
row:

```text
singleton_id: 1
group_id: bytes[8]
database_schema_version: u32
database_schema_fingerprint: bytes[32]
```

`database_schema_fingerprint` is SHA-256 of the exact packaged
`receiver/db/schema.sql` bytes. The generated SQL creates the metadata table
but does not insert its row: `group_id` is deployment-specific and embedding
the fingerprint in its own input would be self-referential.

`cura_receiver.database_initializer.initialize_database()` reads and hashes
the exact packaged `schema.sql` before executing it, compares that hash with
the generated Python constant, creates a temporary database, sets
`application_id`, executes the already verified SQL and inserts the metadata
singleton in the same creation transaction. Only after integrity and foreign-
key checks succeed does it atomically install the new database at an absent
destination. It never overwrites an existing database, discovers or sorts SQL
fragments on the Pi, or computes a new local expected value with which to bless
arbitrary SQL.

The initializer synchronizes the containing directory immediately after
creating the absent destination hard link and before removing the validated
temporary name. Failure before that synchronization completes is an uncertain
installation: it preserves both names and returns
`DatabaseInstallationUncertainError`. Deployment must pass that exact error and
the requested group identity to `reconcile_database_installation()`, which
verifies both names still identify the original file, revalidates the database,
and retries the directory synchronization. After the destination is durably
installed, failure to remove or durably remove the temporary name is reported
only as `DatabaseInitializationResult.cleanup_pending_path`; it does not turn a
successful database installation into failure.

Existing-database startup checks `application_id`, the one metadata row, exact
configured `group_id`, exact generated schema version and exact generated
fingerprint. It does not issue one validation query per enum table. A mismatch
publishes `UNAVAILABLE_INCOMPATIBLE_SCHEMA` without modifying the database;
structural SQLite corruption publishes `UNAVAILABLE_CORRUPT` and follows the
whole-database preservation policy.

### Generated enum catalogues

[`schemas/receiver_enums.json`](schemas/receiver_enums.json) is the
source of truth for stable receiver-local enum assignments that cross a
persistence boundary. Generation emits ordinary static Python `Enum` classes,
the relational SQL catalogues and the schema identity constants. Python does
not load enum definitions dynamically from SQLite, so queue specification
handling, radio behavior and persistence-unavailable behavior work without a
usable database.

The manifest has exactly three persistence modes:

- `scalar_foreign_key` generates one strict SQLite reference table with
  `id INTEGER PRIMARY KEY` and `code TEXT UNIQUE`;
- `scoped_foreign_key` generates a composite catalogue such as
  `(error_domain_id, id, code)` so reused domain-local IDs cannot be referenced
  without their scope; and
- `encoded_only` generates Python assignments but no SQLite catalogue because
  the value occurs only inside a canonical binary encoding or selects an array
  slot.

The six time-backend status-byte families selected by
`TimeBackendStatusKind` are separate `encoded_only` enums. Runtime adapter
enums remain independent and convert through explicit mappings rather than
assuming library numeric values match the persisted diagnostic encoding.

Python class names are `PascalCase`; SQL identifiers are `snake_case`; Python
members and the exact SQL `code` values are `UPPER_SNAKE_CASE`. Generated
classes use `Enum`, not `IntEnum`; generated field-specific binders extract
`.value` only for fields declared as enums. Protocol-owned enum assignments
continue to come from the protocol schema and are consumed rather than
redeclared.

Generated catalogue seed rows are followed by triggers that reject every
insert, update and delete. Event tables store numeric IDs with foreign keys,
and `PRAGMA foreign_keys=ON` is set and verified on every connection. SQLite
enforces storage types, relational enum membership and scope identity. Code,
not SQLite, enforces flag masks and the allowed diagnostic
error-code/operation/severity/context combinations.

Runtime-only result enums and flags are outside the enum generator. When a
runtime result selects an expanded SQLite column, the entity manifest records
only its ordered layout labels; it does not assign a persisted enum ID. Array
axes backed by receiver enums name those central members and record only the
source-array index.

### Generated entities and relational bindings

[`schemas/receiver_entities.json`](schemas/receiver_entities.json) is the
source for generated logical Python entities, explicit persistence-only rows,
table/column layouts, deterministic array projection and multi-table
transaction target layouts. Its top-level `logical_records` are immutable
Python value types with no standalone table or binder. A relational field of
type `logical_record:<NAME>` with `sql.flatten = true` composes one such value
into a generated row while retaining the logical record's flat column layout.
Logical records cannot nest.

Persisted entities have four current modes:

- `direct` maps each logical Python field to exactly one SQLite column;
- `array_expansion` retains a one- or two-dimensional counter array in the
  logical entity and binds one named numeric column per axis combination; and
- `multi_table_transaction` defines persistence-row targets that handwritten
  code must handle atomically; and
- `canonical_blob` maps one generated logical entity through a named canonical
  encoding into a controlled SQLite row containing the exact blob and its
  SHA-256 digest.

For every generated entity or persistence row, the Python module exposes an
`<ENTITY_NAME>_TABLE` string and an `<ENTITY_NAME>_COLUMNS` tuple in the exact
order consumed by its generated `*_parameters()` binder. `ClockObservationV1`,
`DiagnosticV1` and
`ReceiverHealthV1` are the logical objects consumed directly by their binders;
there is no intermediate SQLite `Row` object. `ReceiverHealthV1` keeps its
arrays in Python, while binding consumes their exact declared shapes and emits
columns in deterministic axis order. `QuarantinedEntityRowV1` remains a
persistence-only row because persistence constructs it from an isolated queue
entity and frozen failure provenance.

The packet transaction uses `MessageProfileRowV1` and `ReadingMessageRowV1`
targets because their values come from multiple inputs or handwritten derived
decisions. Generated `MessageProfilingV1` contains the communicator-owned
physical-occurrence facts. `MessageProfileRowV1` composes that logical record
with persistence's per-occurrence `PersistenceClassification` and flattens the
profile into `message_profiles`. `ReadingMessageRowV1` remains a separate,
conditional effect. The generator does not declare variants, decide which
target applies, or execute a transaction.

`AuthenticatedReadingCandidateV1` and the composed queue units are volatile
typed handoff contracts rather than SQL mappings. Their immutable dataclasses,
the quarantine-evidence codec and the diagnostic-context semantic validators
remain handwritten. Normal queue handoff has no entity codec or builder. These
types may reuse generated logical values and enums, but they do not extend this
relational generator into a general policy or queue-layout engine.

`CommunicatorStateV1`, `RtcProvenanceV1` and `TxAirtimeBucketV1` are generated
from the named communicator-state
encoding. The generated encoder derives the format version, encoded length,
validity mask, fixed bucket count and reserved zeros. Its decoder performs only
the mechanical checks needed to recover one canonical value: exact constants,
lengths and fixed array shapes, known enum values, reserved bits and bytes,
presence-controlled zero representations, and absence of trailing bytes. The
generated SQLite binder encodes once, hashes those exact bytes and returns the
singleton envelope parameters.

Generated binders perform projection only: `None` remains SQL `NULL`, declared
enum fields use `.value`, other fields pass through unchanged, and arrays are
expanded in manifest order. They perform no numeric, length, validity-mask,
cross-field or policy validation and no SQLite I/O. Exact array-shape rejection
prevents silent loss during projection; it is not domain validation. SQLite
enforces its generated type, range, length, foreign-key and uniqueness
constraints. A non-null variable-byte field may declare `length_field` naming
a non-null integer sibling; generation then emits their exact SQLite length
equality. Handwritten code owns semantic validation, classification,
conditional effects, replay, transaction boundaries and reconciliation.
Canonical-BLOB codecs additionally enforce their binary grammar because decoding
cannot safely be delegated to SQLite. They do not enforce airtime policy,
ordering, time relationships, lifecycle references or state
transitions; handwritten communicator and persistence code retain those
responsibilities.

Validity masks belong to binary encodings, not logical Python entities or
relational tables. A protocol or canonical-BLOB codec translates
between mask bits and Python `None`/Boolean values. The relational binder then
stores optional values as SQL `NULL`; it never stores a second mask column.
Exact raw bytes retain their encoded mask only where preserving those bytes is
an explicit evidence or canonical-state requirement.

### Core tables and identities

The assembled pilot schema contains the following core tables in addition to
the generated enum catalogues. Completing these Stage 1 tables does not make
the receiver operational: queue behavior and validators, runtime persistence,
classification, replay and recovery remain deferred to their owning stages.

| Table | Canonical identity and role |
|---|---|
| `database_metadata` | Immutable singleton binding `group_id` to schema version and exact schema fingerprint; no master key |
| `receiver_instances` | Database-local instance order and process lifecycle |
| `communicator_state` | Singleton authoritative clock/airtime state |
| `quarantined_communicator_states` | Exact invalid state rows preserved before conservative replacement |
| `clock_observations` | `(receiver_instance_id, observation_sequence)`; monotonic/UTC correlations and time-state boundaries |
| `message_profiles` | `(receiver_instance_id, occurrence_sequence)`; every admitted packet occurrence |
| `reading_messages` | `(node_id, message_id)`; every new logical reading message, with one canonical row per `(node_id, sample_id)` |
| `diagnostics` | `(receiver_instance_id, diagnostic_sequence)`; communicator-created diagnostics only |
| `receiver_health` | `(receiver_instance_id, health_sequence)`; complete enriched `ReceiverHealthV1` |
| `quarantined_entities` | Exact canonical tagged-JSON evidence and minimal failure provenance for poisoned queue units |

`database_metadata` permanently binds one database to one eight-byte
`group_id`, one schema version and one exact schema fingerprint. It never
contains `application_id`, `group_master_key`, derived node keys or other
secret provisioning material.

Tables use `STRICT` mode, fixed-length BLOB checks, foreign keys and explicit
range constraints. Composite canonical tables should use `WITHOUT ROWID`.
Optional stored values are SQL `NULL`, never numeric sentinels. No canonical
insert uses `INSERT OR REPLACE`. Every append-only table rejects a conflicting
primary or declared unique identity in a `BEFORE INSERT` trigger, before a
statement-level conflict algorithm could delete the existing row. This remains
effective with SQLite's default `recursive_triggers=OFF`; ordinary idempotent
replay selects and compares the complete existing row instead of suppressing or
replacing the conflict.

`message_profiles` stores all logical `MessageProfilingV1` fields with absent
fields mapped to `NULL` and adds the derived `persistence_classification`. It
does not store a separate validity mask, UTC or source-observation columns and
is immutable after insertion.
`reading_messages` stores one row for each new `(node_id, message_id)`, including
new logical messages that persistence classifies as duplicate or conflicting.
It retains `sample_id`, the exact clear 32-byte reading body, decoded protocol
fields, `is_canonical_for_sample`, and the first occurrence identity. That
identity references the `message_profiles` row containing the exact
authenticated frame and domain. A partial unique index permits at most one row
with `is_canonical_for_sample = 1` for each `(node_id, sample_id)`.

Every reading-message row represents an authenticated, structurally valid
candidate. `is_canonical_for_sample = 0` means only that the sample already had
a canonical row or conflicted with its body. Decoded sensor columns retain the
canonical numeric values, including protocol-defined zero for an invalid
flagged field; the stored `flags` column remains the sensor-validity authority
rather than projecting those values to SQL `NULL`. Reading-message rows are
immutable.

Ordinary replay compares the complete stored representation, not a subset of
business fields. For profiles this includes every `MessageProfilingV1` column
and the frozen persistence classification. For reading-message rows it includes
`node_id`, `message_id`, `sample_id`, the exact reading body, the canonical
Boolean, every decoded column and the first occurrence identity. Exact
transport comparison additionally uses the immutable frame in that first
profile. No field in these rows is eligible for a later update.

For a replayed `MeasurementProfileUnitV1`, persistence also verifies that its
frozen classification agrees with the immutable canonical effects:

- `FIRST_SEEN`: the current occurrence owns a matching reading-message row with
  `is_canonical_for_sample = 1`;
- `RETRANSMISSION`: an earlier occurrence owns the reading-message key and its
  first profile has the same exact authenticated frame;
- `DUPLICATE_SAME_CONTENT`: the current occurrence owns a new noncanonical row
  whose body matches the earlier canonical sample row;
- `DUPLICATE_CONFLICT`: either an earlier occurrence owns the same message key
  and sample with a different exact frame, or the current occurrence owns a new
  noncanonical row whose body differs from the earlier canonical sample row;
  and
- `MESSAGE_ID_CONFLICT`: an earlier reading-message row owns the same message
  key with a different sample ID.

The profile and reading-message effect created for one occurrence commit in one
transaction. A reading-message row that names the current occurrence as first
owner while its profile row is absent is therefore an invariant failure, not a
partially successful replay. A matching existing effect is no-op success; an
absent required effect is inserted; any different row under the same durable
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
the occurrence profile's classification together with the immutable
reading-message row and its first profile. An analysis view may join those rows
to expose conflicts directly.

### Clock observations and UTC assignment

`clock_observations` uses this logical SQL contract:

```sql
CREATE TABLE clock_observations (
    receiver_instance_id BLOB NOT NULL
        REFERENCES receiver_instances(receiver_instance_id)
        CHECK (length(receiver_instance_id) = 16),
    observation_sequence INTEGER NOT NULL
        CHECK (observation_sequence >= 0),

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

    PRIMARY KEY (receiver_instance_id, observation_sequence)
) STRICT, WITHOUT ROWID;
```

Persistence verifies that the source receiver instance exists and that
sequence, generation, monotonic ordering, UTC presence/quality relationship and
step-boundary relationship obey the `ClockObservationV1` contract before
insertion. The latter relationships remain code responsibilities rather than
cross-column SQLite checks. Rows are append-only. Multiple periodic
observations may carry the same generation.

For a stored event with `(receiver_instance_id, event_monotonic_us)`, analysis
first resolves its `receiver_instances` row and considers only observations
carrying that exact receiver instance ID. It orders them by sampled monotonic
value and observation sequence and applies this deterministic rule:

1. The instance start at `started_at_monotonic_us` is an implicit ordinary
   boundary. It closes the preceding receiver instance's segment, and no
   observation is assigned across an instance boundary.
2. Any `UNTRUSTED` observation closes the current instance's preceding trusted
   segment at its monotonic boundary.
3. If the event lies in an open trusted segment, select the latest trusted
   observation at or before the event.
4. If the event lies from the instance start or an ordinary untrusted boundary
   through the first later trusted observation, select that later same-instance
   observation only when no step-discontinuity boundary lies between the event
   and that observation.
5. A step-discontinuity boundary terminates the preceding segment. Assign no
   UTC when the event is at or after that boundary and before the first later
   trusted observation. That later observation starts the new segment and is
   never extrapolated backward across the boundary, including to an event
   before the boundary that has no eligible preceding trusted observation.
6. Treat an observation at the event's same monotonic microsecond as preceding
   the event.
7. Derive UTC with
   `observation_utc + event_monotonic - observation_monotonic`.

Analysis returns the derived UTC and source-observation identity together; it
does not update the event row. A shared `linux_boot_id` does not permit
cross-instance assignment, and no cross-boot assignment is permitted.

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

Persistence omits `instance_ordinal` on insertion so SQLite assigns the next
row identity. A trigger additionally requires every inserted ordinal to equal
the resulting one-based row count; explicit gaps, backfilling and reordered
inserts therefore fail even if a caller supplies the column. The ordinal is
database-local analysis order and is never reused because lifecycle rows are
never deleted. `receiver_instance_id` remains the public identity referenced by
other tables.

`receiver_instances` is the sole persisted source of `linux_boot_id`. Queue
entities, `CommunicatorStateV1` and every other database row store only the
applicable `receiver_instance_id`; foreign keys or explicit state validation
must ensure that identifier resolves to this table. Analysis views expose a
boot ID only by joining through this immutable mapping. `linux_boot_id` is not
unique because multiple receiver processes may run during one Linux boot.

The row is inserted as soon as SQLite is usable and before ordinary persistence
admission becomes available. `started_at_monotonic_us` is captured when the
receiver instance ID is created. Its durability before ordinary admission makes
it an implicit ordinary correlation boundary. It closes the preceding
receiver-instance segment even within one Linux boot and ensures that a step
boundary lost with the preceding process's volatile FIFO cannot be crossed by
new durable data. Analysis may derive start UTC from a qualifying later
same-instance observation when no explicit step boundary intervenes, but the
lifecycle row remains unchanged.

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

A poisoned unit is a complete immutable typed queue unit that reproducibly
fails persistence when isolated because of an entity-specific binding,
derivation, range, schema-version or unexpected schema-constraint defect. It is
not malformed radio input, a duplicate classification, disk full, database
corruption, locking or a transient/global I/O failure.

`ClockObservationV1` is not eligible for item quarantine. Its isolated failure
is treated as a receiver-interface/schema incompatibility: persistence retains
the clock observation and claimed batch, publishes
`UNAVAILABLE_INCOMPATIBLE_SCHEMA` and does not process later FIFO entities. A
missing time-state boundary must never be converted into a quarantined gap
while analysis continues correlating later events with UTC.

Persistence rolls back the failed batch, excludes global and transient causes,
narrows the batch and retries the suspected unit alone. Except for the clock
observation rule above, only reproduction of the item-specific failure permits
quarantine. A `MeasurementProfileUnitV1` is always quarantined as one complete
logical unit.

#### `QuarantineEvidenceV1`

The normal queue path has no encoder. When persistence has isolated a poison-
eligible typed unit, it invokes the handwritten `QuarantineEvidenceV1` codec to
snapshot the exact admitted logical value through:

```python
encode_quarantine_evidence_v1(
    entity: object,
    *,
    spec: PersistQueueEntitySpec,
) -> bytes

decode_quarantine_evidence_v1(encoded: bytes) -> QuarantineEvidenceV1

quarantine_evidence_sha256(encoded: bytes) -> bytes
```

The hash helper is applied to the exact successfully encoded bytes; it does not
replace decoder validation. The canonical evidence bytes are
ASCII JSON with no whitespace or trailing newline, lexicographically sorted
object keys and this exact top-level shape:

```json
{"entity_kind":1,"entity_schema_version":1,"format":"cura-agrorum-quarantine-evidence","format_version":1,"value":{"tag":"none"}}
```

`entity_kind` and `entity_schema_version` duplicate the queue specification and
must be integer values in `0..255`; Boolean values are not integers here. The
format name and version are fixed as shown. Every logical value is one tagged
object with an exact key set:

| Tag | Additional keys | Logical value |
|---|---|---|
| `none` | none | `None` |
| `bool` | `value` as JSON Boolean | Python `bool` |
| `int` | `value` as minimal decimal string | Arbitrary signed Python integer within the evidence bound |
| `str` | `value` as JSON string | Exact Python string/code points |
| `bytes` | `encoding = "base64"`, `value` | Exact bytes using padded standard base64 |
| `enum` | `class`, `member`, decimal-string `value` | Allowlisted enum class, member name and integer value |
| `tuple` | `items` | Ordered immutable sequence of tagged nodes |
| `record` | `class`, `fields` | Allowlisted dataclass; `fields` is an ordered array of exact `{"name", "value"}` pairs |

The record allowlist contains the handwritten queue-unit and candidate classes
plus generated `MessageProfilingV1`, `ClockObservationV1` and `DiagnosticV1`.
The enum allowlist contains only enum classes used by those records. Canonical
class labels are their fully qualified module and class names. Record fields
must occur exactly once in declared order. This schema intentionally has no
generic mapping, list, float, mutable-buffer, pickle, `repr` or arbitrary-
object node. A field containing the wrong supported scalar type, wrong byte
length or out-of-domain integer remains distinguishable and exact; an
unsupported type fails closed instead of being coerced.

The fixed bounds are 262,144 canonical evidence bytes, 32 nested node levels,
4,096 total nodes, 1,024 items in one tuple, 65,536 UTF-8 bytes in one string,
65,536 bytes in one byte value and 1,024 digits in one integer magnitude.
Cycles and every exceeded bound are errors. Decoding additionally rejects
invalid UTF-8/ASCII, duplicate or unknown keys/tags, unknown class labels,
noncanonical integers or base64, noncanonical JSON spelling/key order/spacing,
trailing bytes and any structurally invalid tagged node.

The decoder returns only frozen neutral evidence nodes containing scalar values
and record/enum labels. It never imports a label, invokes application code or
reconstructs a production entity. Later offline analysis owns interpretation.
Before returning that tree, the decoder canonically re-encodes the parsed JSON
and requires byte-for-byte identity with the supplied evidence. If encoding
fails for a poisoned queue head, persistence cannot
manufacture substitute bytes or acknowledge it: the active lease and every
following FIFO entry remain queue-owned and admission closes under the
applicable incompatible state.

SQLite contains one generic append-only table:

```text
quarantine_id: bytes[32]
entity_kind: u8
entity_schema_version: u8
entity_length: u32
entity_bytes: bytes[entity_length]

receiver_instance_id: bytes[16]
quarantined_at_monotonic_us: u64

database_schema_version: u32
failure_reason: QuarantineFailureReason
failure_operation: DiagnosticOperation
sqlite_primary_code: i32 or absent
sqlite_extended_code: i32 or absent
os_errno: i32 or absent
isolation_attempt_count: u32
```

`quarantine_id` is SHA-256 of the exact canonical `QuarantineEvidenceV1` bytes
and is the primary key. The explicit entity kind and schema version duplicate
the tagged-JSON envelope so quarantine can be indexed and screened without
interpreting the problematic payload. `length(entity_bytes)` must equal
`entity_length`.
`receiver_instance_id` comes from the immutable process startup context, not
by decoding the suspected entity. Its durable lifecycle row supplies the Linux
boot ID when quarantine evidence is analyzed.

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
It stores the queued monotonic fields without copying `linux_boot_id`; every
column is immutable after insertion. Analysis resolves the boot through
`receiver_instances` and may derive a UTC value and source-observation identity
without updating the diagnostic row.
Persistence-control failures may be returned to the communicator and converted
there according to the catalogues in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md). Asynchronous SQLite
failures may be exposed by admission state, a later `ReceiverHealthV1`,
and quarantine provenance when applicable. None is a guaranteed durable
incident record while SQLite itself is unavailable; this is the explicit pilot
limitation defined in
[`ARCHITECTURE.md`](ARCHITECTURE.md#persistence-unavailable-observability-limitation).

`receiver_health` expands enum-indexed arrays into named numeric columns for
analysis, including one column for every chrony-step disposition, RTC-write
disposition, persist-queue entity/admission-result pair and persistence-state
transition count. It also stores
`rtc_write_readback_verified_count` and
`rtc_write_trust_invalidated_count` as distinct scalar columns and stores
optional transition timestamps and host observations as `NULL`. No redundant
step-required or total RTC-write-attempt column exists; those totals are derived by summing
their disposition columns. Persistence creates the complete immutable row once
and never resamples it across transaction retry.

No `persistence_batches` or `receiver_state_operations` tables exist in the
pilot. Batch throughput, failure and checkpoint aggregates are carried by
`ReceiverHealthV1`. Communicator-state control failures may use `DiagnosticV1`
according to the `PERSISTENCE_CONTROL` or caller-violation `CORE` catalogue
when the communicator can admit one; authoritative state remains only in
`communicator_state`.

# Receiver diagnostic interface

Status: provisional pilot contract. This document defines the receiver's
diagnostic entity, stable common enums and domain-local diagnostic catalogues.
[`INTERFACE.md`](INTERFACE.md) remains authoritative for common encodings,
identities, `PersistQueue`, SQLite replay and receiver state enums.

The pilot catalogues are `RADIO`, `TIME`, `PERSISTENCE_CONTROL` and `CORE`.
A receiver diagnostic must not use another domain until that domain's error
codes, allowed operations and every referenced context schema have been added
to this document and to the receiver enum source of truth,
[`schemas/receiver_enums.json`](schemas/receiver_enums.json).

This is a receiver-local contract. Firmware diagnostic enums, numeric values
and context schemas are not imported, reserved or consulted by receiver code or
generation tooling. A receiver value may use the same descriptive name when
the concept is genuinely shared, but its identity and evolution remain
independent. A firmware change therefore never requires a receiver diagnostic
change, or the reverse.

## Implementation feedback and revision

This contract is intentionally reviewable during implementation. Both the
human implementer/reviewer and an AI agent working on the implementation may
propose a diagnostic-contract change when implementation or testing reveals:

- a documented state or combination that is unreachable;
- a simpler representation that preserves the required evidence;
- a reachable state, transition or failure that the documentation omitted; or
- an ambiguity that prevents one deterministic encoding or recovery action.

Such a finding must not be handled by silently deviating from this document.
The proposer should give the concrete execution path or test evidence, the
smallest documentation/schema change and its compatibility effect. The agreed
contract, generated enum source and tests are then updated before code relies
on the change. During the pilot, an incompatible persisted assignment starts a
new schema version and explicit archive-and-recreate database epoch; an
existing database and its archived rows are never reinterpreted in place.

## Diagnostic role

Diagnostics record exceptional communicator failures and bounded recovery
episodes. They do not duplicate:

- ordinary protocol outcomes already present in `MessageProfilingV1`;
- duplicate/conflict classifications and immutable reading-message evidence;
- aggregate rates already represented by `ReceiverHealthV1`; or
- exact poisoned-unit bytes and failure provenance in quarantine tables.

Only the communicator allocates diagnostic identity, constructs
`DiagnosticV1` and attempts its nonblocking `PersistQueue` admission. The
persistence thread may return a synchronous control failure for the
communicator to map through the `PERSISTENCE_CONTROL` or caller-violation
`CORE` catalogue, but it never creates a diagnostic identity or inserts a
persistence-created diagnostic.

Diagnostics are immutable once published. Failure to admit a diagnostic never
recursively creates another diagnostic. The communicator increments exactly
the original `DIAGNOSTIC` entity-kind/admission-result matrix cell and takes no
second queue action. More generally, a `PERSISTENCE_UNAVAILABLE` or `QUEUE_FULL`
result for any entity kind is not a diagnostic trigger: the communicator does
not allocate another diagnostic identity, construct a `DiagnosticV1` about the
result or attempt another reservation because of it. A diagnostic whose own
original admission failed keeps its already allocated identity only as part of
that failed attempt.

## `DiagnosticV1`

Encoded size: 171 bytes, including the two-byte entity envelope.

```text
entity envelope: 2 bytes

receiver_instance_id: bytes[16]
diagnostic_sequence: u64
sampled_at_monotonic_us: u64

severity: u8
error_domain: u16
operation: u16
error_code: u16
context_schema: u8
context_length: u8
context: bytes[128]
```

`sampled_at_monotonic_us` is the time at which the exceptional trigger was
observed. For a diagnostic finalized after bounded recovery, the context stores
the complete episode duration. The queued and stored representations remain
monotonic-only and contain only `receiver_instance_id`; analysis obtains the
Linux boot through `receiver_instances` and may derive UTC from
`ClockObservationV1` without updating the row.

`(receiver_instance_id, diagnostic_sequence)` is the durable identity.
Ordinary replay compares every queued field; exact equality is no-op success
and any difference is the global identity-invariant failure defined by
`INTERFACE.md`.

`context_schema = 0` if and only if `context_length = 0`. Every unused context
byte is zero. A nonempty context requires an operation other than `NONE`.
Context never contains keys, unrestricted plaintext, arbitrary exception text,
object dumps or private Python representations.

The generated relational catalogues enforce only scalar enum membership and
the `(error_domain, error_code)` and `(error_domain, context_schema)` scopes.
Receiver code validates the closed error-code/operation/severity/context
combinations, fixed context lengths, flag masks and reserved bits before queue
publication. SQLite does not duplicate those semantic checks.

## Common diagnostic enums

### `DiagnosticSeverity`

| Value | Name | Meaning |
|---:|---|---|
| `1` | `WARN` | A useful anomaly was handled without entering exceptional recovery or losing a packet/selected ACK outcome |
| `2` | `ERROR` | An operation failed, an occurrence outcome was affected, or exceptional recovery was required and succeeded |
| `3` | `FATAL` | The receiver instance is entering a terminal state because the condition could not be recovered safely |

Value `0` is invalid. The pilot records no `INFO` diagnostics.

### `DiagnosticErrorDomain`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `NONE` | Success only; invalid in `DiagnosticV1` |
| `1` | `RADIO` | Receiver SX1262 and radio-backend failures defined below |
| `2` | `TIME` | Runtime chrony, Linux-clock, DS3231 and time-policy failures |
| `3` | `PERSISTENCE_CONTROL` | Failures returned through the synchronous persistence control channel |
| `4` | `CORE` | Communicator orchestration, construction and caller-contract failures |

No values are reserved for possible future domains. A future receiver domain
is added only together with its usable error codes, operation combinations and
complete context schemas.

### `DiagnosticOperation`

An operation describes the stable receiver action being attempted, not a
Python function, private helper or low-level primitive. The table contains only
actions already used by the receiver's radio, persistence-control, encoding,
protocol-processing or shutdown contracts.

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `INITIALIZE` |
| `2` | `VALIDATE` |
| `3` | `READ` |
| `4` | `WRITE` |
| `5` | `APPEND` |
| `6` | `SYNC` |
| `7` | `RECOVER` |
| `8` | `ENCODE` |
| `9` | `DECODE` |
| `10` | `TRANSMIT` |
| `11` | `RECEIVE` |
| `12` | `CLEANUP` |

`NONE` is valid only when no operation is meaningful and is invalid in every
current diagnostic catalogue. The catalogues below reuse these assignments;
none introduces a private-function operation.

## Radio diagnostic catalogue

The receiver uses `DiagnosticErrorDomain.RADIO = 1`. Its codes are assigned by
this receiver contract. Some names also describe failures found in firmware,
but there is no shared enum or cross-component numeric compatibility rule.

### `RadioDiagnosticErrorCode`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `NONE` | Success only; invalid in a diagnostic |
| `1` | `INVALID_ARGUMENT` | Receiver code violated a radio-adapter input invariant |
| `2` | `INVALID_STATE` | A radio operation or event is impossible in the communicator's current state |
| `3` | `IO` | A GPIO, SPI, IRQ or other radio-backend primitive failed |
| `4` | `BUSY_TIMEOUT` | SX1262 BUSY did not deassert within its bounded wait |
| `5` | `COMMAND_STATUS` | Confirmed SX1262 command status reported processing, execution or timeout failure |
| `6` | `DEADLINE` | A bounded radio operation ended without its required terminal result |
| `7` | `UNEXPECTED_IRQ` | IRQ bits cannot represent a valid event for the active operation/state |
| `8` | `DEVICE_ERROR` | SX1262 device-error bits report an oscillator, PLL, calibration or PA fault |
| `9` | `MALFORMED_RESPONSE` | A structurally invalid command response, buffer description or packet-status result was returned |

The error code records the observed trigger, not the eventual recovery result.
For example, a BUSY timeout keeps `BUSY_TIMEOUT` whether soft recovery succeeds
or hard recovery is exhausted. Command-effect certainty and the terminal state
are separate context fields. `HARDWARE_MISSING`, `RECOVERY_EXHAUSTED`,
`INITIALIZATION_FAILED` and `RX_PROFILE_RESTORE_FAILED` are therefore not
duplicate error codes: they are terminal states or recovery reasons already
defined in `INTERFACE.md`. Missing or unreachable hardware uses the precise
`IO` trigger plus backend status when available and the final
`HARDWARE_MISSING` state.

Radio diagnostics use only these operations:

| Operation | Use |
|---|---|
| `INITIALIZE` | Initial GPIO/SPI/SX1262 setup and complete receive-profile installation |
| `VALIDATE` | Receiver/radio state and internal input invariants |
| `TRANSMIT` | ACK profile installation, buffer write, `SetTx` and terminal TX handling |
| `RECEIVE` | Receive-profile installation, `SetRx`, DIO1/IRQ handling and packet copying |
| `RECOVER` | A recovery episode whose initial trigger occurs while no earlier semantic operation applies |
| `CLEANUP` | Establishing the configured safe radio state, including any sleep command, during controlled shutdown |

Low-level `READ`, `WRITE` and private command names are encoded as context
stages/opcodes while the operation retains the enclosing semantic action. When
recovery follows a failed `TRANSMIT` or `RECEIVE`, the completed diagnostic
keeps that original operation instead of changing it to `RECOVER`.

The valid error-code/operation combinations are closed rather than advisory:

| Error code | Allowed operations |
|---|---|
| `INVALID_ARGUMENT` | `VALIDATE` |
| `INVALID_STATE` | `VALIDATE` |
| `IO` | `INITIALIZE`, `TRANSMIT`, `RECEIVE`, `RECOVER`, `CLEANUP` |
| `BUSY_TIMEOUT` | `INITIALIZE`, `TRANSMIT`, `RECEIVE`, `RECOVER`, `CLEANUP` |
| `COMMAND_STATUS` | `INITIALIZE`, `TRANSMIT`, `RECEIVE`, `RECOVER`, `CLEANUP` |
| `DEADLINE` | `INITIALIZE`, `TRANSMIT`, `RECEIVE`, `RECOVER`, `CLEANUP` |
| `UNEXPECTED_IRQ` | `TRANSMIT`, `RECEIVE`, `RECOVER` |
| `DEVICE_ERROR` | `INITIALIZE`, `TRANSMIT`, `RECEIVE`, `RECOVER`, `CLEANUP` |
| `MALFORMED_RESPONSE` | `INITIALIZE`, `TRANSMIT`, `RECEIVE`, `RECOVER`, `CLEANUP` |

`RECEIVE + DEADLINE` is an error only when a required receive-side command,
IRQ-handling step or RX re-arm fails to complete by its bound. Normal absence
of an uplink is not a diagnostic. Initial or runtime device disappearance uses
the operation's `IO` trigger and records terminal `HARDWARE_MISSING`.

Severity is also constrained. `INVALID_ARGUMENT` is always `FATAL`.
`UNEXPECTED_IRQ` may be `WARN` when direct IRQ clearing and RX re-arming
succeed without losing an occurrence or selected ACK outcome; otherwise it is
`ERROR` after recovery succeeds or `FATAL` after terminal failure. Every other
defined code is `ERROR` when its direct handling or recovery returns the radio
to a confirmed safe operational state and `FATAL` when the receiver instance
terminates because of it. Initialization failure, including missing hardware,
is therefore `FATAL`; a definite pre-`SetTx` failure followed by confirmed RX
restoration is `ERROR`.

### Context-schema assignments

Context schemas are scoped by error domain.

| Value | Name | Length | Receiver use |
|---:|---|---:|---|
| `0` | `NONE` | `0` | Invalid for a nonzero radio error |
| `1` | `RECEIVER_RADIO_EPISODE_CONTEXT_V1` | `64` | Every receiver radio diagnostic |

Every receiver radio error uses schema `1`, even when no transition into
`RECOVERING` was required; the recovery fields then carry their explicit
not-applicable values.

### `RadioFailureDetailV1`

The episode context contains two receiver-owned 14-byte failure-detail slots.
They compactly retain the raw radio/backend evidence needed by this design:

| Offset | Field | Encoding | Meaning |
|---:|---|---:|---|
| `0` | `state` | `u8` | Receiver `RadioState` when this failure was observed |
| `1` | `command_opcode` | `u8` | Raw SX1262 opcode, or zero when none applies |
| `2` | `stage` | `u8` | `RadioFailureStage` |
| `3` | `flags` | `u8` | Detail validity and hardware-touch flags |
| `4` | `backend_status_kind` | `u8` | `RadioBackendStatusKind` |
| `5` | `backend_status` | `i32` | Exact normalized backend result, little-endian |
| `9` | `chip_status` | `u8` | Raw SX1262 status byte when valid |
| `10` | `irq_status` | `u16` | Raw SX1262 IRQ bits when valid, little-endian |
| `12` | `device_errors` | `u16` | Raw SX1262 device-error bits when valid, little-endian |

Flag bits are:

| Bit | Name |
|---:|---|
| `0` | `CHIP_STATUS_VALID` |
| `1` | `IRQ_STATUS_VALID` |
| `2` | `DEVICE_ERRORS_VALID` |
| `3` | `HARDWARE_TOUCHED` |
| `4`–`7` | Reserved; zero |

Fields whose validity bit is clear are zero. `HARDWARE_TOUCHED` records that
some radio hardware action occurred; it is not proof that the last command had
an effect.

`RadioBackendStatusKind` values are:

| Value | Name | Constraint |
|---:|---|---|
| `0` | `NONE` | `backend_status = 0` |
| `1` | `ERRNO` | Exact positive host errno normalized to signed `i32` |
| `2` | `SX1262_DRIVER_STATUS` | Exact signed driver status normalized to `i32` |

Unexpected Python exception names or messages are never encoded as backend
status. Production radio adapters must normalize expected OS/driver failures;
an unclassified implementation exception belongs to the `CORE` catalogue and
must not be forced into the radio context.

`RadioFailureStage` uses these receiver-local assignments:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `STATE_CHECK` |
| `2` | `VALIDATE_INPUT` |
| `3` | `CONFIGURE_GPIO` |
| `4` | `CONFIGURE_SPI` |
| `5` | `RESET` |
| `6` | `WAKE` |
| `7` | `WAIT_BUSY` |
| `8` | `WRITE_COMMAND` |
| `9` | `READ_COMMAND` |
| `10` | `CONFIGURE_IRQ` |
| `11` | `WAIT_IRQ` |
| `12` | `READ_IRQ` |
| `13` | `CLEAR_IRQ` |
| `14` | `WRITE_BUFFER` |
| `15` | `READ_BUFFER` |
| `16` | `READ_PACKET_STATUS` |
| `17` | `DETACH_IRQ` |
| `18` | `CAPTURE_TIME` |

Profile installation failures use the exact primitive stage and command
opcode rather than adding a stage for every private profile helper.

### Radio episode sub-enums

`RadioCommandOutcome` separates the command's possible effect from the reason
an operation failed:

| Value | Name |
|---:|---|
| `0` | `NOT_APPLICABLE` |
| `1` | `DEFINITELY_NOT_APPLIED` |
| `2` | `CONFIRMED_APPLIED` |
| `3` | `UNCERTAIN` |

`RadioRecoveryLevelResult` is used independently for soft and hard recovery:

| Value | Name |
|---:|---|
| `0` | `NOT_APPLICABLE` |
| `1` | `NOT_ATTEMPTED` |
| `2` | `SUCCEEDED` |
| `3` | `FAILED` |

For a diagnostic that never enters `RECOVERING`, both level results are
`NOT_APPLICABLE` and `recovery_reason = 0`. In a recovery episode, an unneeded
later level is `NOT_ATTEMPTED`; for example, hard recovery is not attempted
after soft recovery succeeds.

`recovery_reason` describes why the communicator had to leave its normal state,
not the error-code namespace and not every failure encountered afterward. When
more than one fact applies, select the first matching row in this order:

| Condition forcing `RECOVERING` | `RadioRecoveryReason` |
|---|---|
| Required radio hardware is absent or unreachable | `HARDWARE_UNREACHABLE` |
| A TX command/profile may have taken effect but its outcome is uncertain | `TX_OUTCOME_UNCERTAIN` |
| `SetRx` itself did not restore a confirmed receive state | `SET_RX_FAILED` |
| Another complete `UPLINK_RX_PROFILE` restoration step failed | `RX_PROFILE_RESTORE_FAILED` |
| BUSY did not deassert by its bound | `BUSY_TIMEOUT` |
| An SPI primitive failed without a more specific condition above | `SPI_FAILURE` |
| An unexpected IRQ could not be resolved directly | `UNEXPECTED_IRQ` |
| The radio's status or mode remains unconfirmed for another reason | `STATUS_UNCONFIRMED` |

Later soft/hard failures do not change this root recovery reason.

### `ReceiverRadioEpisodeContextV1`

Schema `RECEIVER_RADIO_EPISODE_CONTEXT_V1 = 1` has this fixed 64-byte encoding:

| Offset | Field | Encoding |
|---:|---|---:|
| `0` | `trigger_detail` | `RadioFailureDetailV1` (`14` bytes) |
| `14` | `last_recovery_failure_detail` | `RadioFailureDetailV1` (`14` bytes) |
| `28` | `validity_mask` | `u16` |
| `30` | `recovery_reason` | `u8` (`RadioRecoveryReason`, or zero) |
| `31` | `trigger_command_outcome` | `u8` (`RadioCommandOutcome`) |
| `32` | `soft_recovery_result` | `u8` (`RadioRecoveryLevelResult`) |
| `33` | `hard_recovery_result` | `u8` (`RadioRecoveryLevelResult`) |
| `34` | `terminal_state` | `u8` (`RadioState`) |
| `35` | reserved | `u8`, zero |
| `36` | `last_recovery_error_code` | `u16` (`RadioDiagnosticErrorCode`, or zero) |
| `38` | reserved | `bytes[2]`, zero |
| `40` | `related_occurrence_sequence` | `u64` |
| `48` | `airtime_bucket_expiration_utc_us` | `i64` |
| `56` | `episode_duration_us` | `u64` |

Validity bits are:

| Bit | Field |
|---:|---|
| `0` | `last_recovery_failure_detail` and `last_recovery_error_code` |
| `1` | `related_occurrence_sequence` |
| `2` | `airtime_bucket_expiration_utc_us` |
| `3`–`15` | Reserved; zero |

When bit 0 is clear, the complete second detail and its error code are zero.
When bit 1 is clear, its sequence is zero. When bit 2 is clear, the bucket
expiration is zero. A present bucket expiration identifies the durable logical
bucket whose current-process grant covered the radio episode.
`episode_duration_us` is checked subtraction from the trigger time
to finalization time and is zero only when both monotonic reads were equal.

`terminal_state` is the communicator's best-known `RadioState` at diagnostic
finalization. A directly handled nonfatal anomaly normally records confirmed
`RX_SINGLE`; a recovery episode records `RX_SINGLE`, `RECOVERY_EXHAUSTED` or
`HARDWARE_MISSING`; and another fatal path records the terminal state selected
before process exit. It never claims `RX_SINGLE` unless the complete receive
profile and `SetRx` have been confirmed.

`last_recovery_failure_detail` records the last recovery-stage failure that
caused escalation or the terminal result. If soft recovery fails and hard
recovery succeeds, it describes the soft failure. If hard recovery fails, it
describes the hard failure; the earlier soft failure remains represented by
`soft_recovery_result = FAILED` and aggregate health counters.

## Radio diagnostic emission contract

Entry into `RECOVERING` creates one mutable communicator-owned episode builder
in RAM. It is not a queue entity and does not reserve queue capacity. The
builder freezes the initial `operation`, `error_code`, trigger time, trigger
detail, command outcome, recovery reason and any correlation identifiers.
Recovery-stage failures update only the bounded recovery-result and last-failure
fields; they never emit independent diagnostics.

The same entry increments `radio_recovery_attempts` and exactly one
`radio_recovery_attempts_by_reason` slot. Returning to confirmed `RX_SINGLE`
increments `radio_recovery_successes`; entering `RECOVERY_EXHAUSTED` or
`HARDWARE_MISSING` increments `radio_recovery_failures`. Soft and hard levels
do not independently increment those episode counters.

When recovery reaches `RX_SINGLE`, `RECOVERY_EXHAUSTED` or `HARDWARE_MISSING`,
the communicator finalizes exactly one immutable `DiagnosticV1`. A recovered
episode uses `ERROR`; either terminal failure uses `FATAL`. For a packet-related
episode, the communicator first publishes the already reserved complete
measurement/profile or profile-only unit, then attempts a separate ordinary
diagnostic reservation. Diagnostic failure cannot invalidate or delay the
profile publication.

A radio anomaly handled without entering `RECOVERING` creates one diagnostic
directly with `recovery_reason = 0` and both recovery results
`NOT_APPLICABLE`. The closed severity rules above determine whether it is
`WARN`, `ERROR` or `FATAL`. Initialization failure finalizes one `FATAL`
diagnostic with terminal state `INITIALIZATION_FAILED` or `HARDWARE_MISSING`.
A controlled-shutdown cleanup failure creates one `ERROR` unless the receiver
is terminating because it cannot establish any safe radio state, in which case
it is `FATAL`.

If the process crashes during recovery, the in-RAM builder may be lost. If
diagnostic admission or SQLite is unavailable at finalization, the completed
diagnostic may also be lost. Both are accepted pilot observability limitations;
they do not relax radio recovery, airtime charging or terminal-state policy.

## Radio failure-scenario policy

| Scenario | Operation and error code | Required handling |
|---|---|---|
| Invalid adapter argument | `VALIDATE + INVALID_ARGUMENT` | Treat as a receiver implementation invariant, suppress further TX, emit one best-effort `FATAL`, and terminate the instance after attempting a safe radio state |
| Impossible communicator/radio state or event | `VALIDATE + INVALID_STATE` | If hardware state is uncertain, run the bounded recovery episode; otherwise emit one best-effort `FATAL` and terminate because retrying the same code path is unsafe |
| Required SPI/GPIO device or SX1262 absent during initialization | `INITIALIZE + IO`, with exact backend status when available | Enter `HARDWARE_MISSING`; emit one `FATAL`; supervisor restart policy remains bounded |
| Initialization backend, BUSY, command-status or device failure | `INITIALIZE` plus the precise code | Complete the bounded initialization policy; emit one `FATAL` only after terminal `INITIALIZATION_FAILED` or `HARDWARE_MISSING` is selected |
| BUSY timeout | Current semantic operation plus `BUSY_TIMEOUT`; stage `WAIT_BUSY` | Preserve whether the associated command was definitely not applied or uncertain; enter recovery whenever mode/profile is not confirmed |
| GPIO/SPI/IRQ primitive failure | Current semantic operation plus `IO` | Preserve errno when available; enter recovery if the radio mode/profile or IRQ state cannot be confirmed |
| Confirmed SX1262 command-status failure | Current semantic operation plus `COMMAND_STATUS` | Preserve opcode and chip status; use command-outcome context to decide recovery and conservative airtime treatment |
| Nonzero SX1262 device-error bits | Current semantic operation plus `DEVICE_ERROR` | Preserve raw bits and enter bounded recovery unless the architecture explicitly proves the operation and receive state unaffected |
| DIO1/IRQ combination invalid for the active state | `RECEIVE` or `TRANSMIT` plus `UNEXPECTED_IRQ` | If IRQ clear and RX re-arm succeed directly, emit one `WARN`; otherwise make it the trigger of one recovery diagnostic |
| PHY header error, CRC error or confirmed TX-timeout IRQ followed by successful RX re-arm | No diagnostic | Record the normal radio/profile outcome and counters; do not duplicate it in `DiagnosticV1` |
| Structurally invalid packet-status, buffer-status or command response | Current semantic operation plus `MALFORMED_RESPONSE` | Discard untrusted returned fields, preserve raw status only when its validity is known, and recover if the radio state cannot be confirmed |
| Definite failure before `SetTx` can take effect | `TRANSMIT` plus the precise trigger code | Reclaim tentative airtime, record `SET_TX_FAILED`, restore confirmed RX, and emit one `ERROR`; if restoration becomes uncertain, continue the same diagnostic as a recovery episode |
| `SetTx` or a TX-profile command crossed SPI but its effect is uncertain | `TRANSMIT` plus the precise trigger code and command outcome `UNCERTAIN` | Retain the complete airtime charge, record the terminal profiling outcome, and enter one recovery episode; do not create a separate “uncertain TX” diagnostic code |
| No valid terminal TX IRQ before the bound | `TRANSMIT + DEADLINE` | Retain the charge when TX may have started and enter recovery; a confirmed timeout IRQ handled normally is the no-diagnostic case above |
| RX-profile restoration or `SetRx` fails on the normal path | `RECEIVE` plus the precise trigger code | Select `RX_PROFILE_RESTORE_FAILED` or `SET_RX_FAILED` as recovery reason and enter one recovery episode |
| Soft recovery fails | Keep the original operation/error code | Set soft result to `FAILED`, retain its detail as the last recovery failure, increment health counters, and attempt the one bounded hard recovery without emitting another diagnostic |
| Soft recovery succeeds | Keep the original operation/error code | Set soft result to `SUCCEEDED`, hard result to `NOT_ATTEMPTED`, return to confirmed `RX_SINGLE`, and emit the one completed `ERROR` diagnostic |
| Hard recovery succeeds | Keep the original operation/error code | Set hard result to `SUCCEEDED`, return to confirmed `RX_SINGLE`, and emit the one completed `ERROR` diagnostic |
| Hard recovery fails or hardware becomes unreachable | Keep the original operation/error code | Preserve the final recovery failure detail, enter `RECOVERY_EXHAUSTED` or `HARDWARE_MISSING`, emit the one completed `FATAL` diagnostic, then terminate the receiver instance |
| Safe-state or sleep command fails during controlled shutdown | `CLEANUP` plus the precise trigger code | Emit one best-effort diagnostic and continue bounded shutdown; correctness must not depend on its persistence |

A synchronous communicator-state settlement failure that follows an uncertain
TX is not a radio-recovery stage. It uses the `PERSISTENCE_CONTROL` catalogue
for an operational control failure or `CORE` when the caller violated that
interface, and may carry the same occurrence or airtime-bucket expiration
for analysis. It must not be folded into the radio diagnostic merely because
both were observed while handling the same packet.

## Required radio-diagnostic tests

At minimum, test:

- receiver diagnostic generation and freshness checks have no dependency on
  firmware enum or context definitions;
- only defined receiver domains, operations and context schemas are generated,
  and unused/reserved compatibility placeholders do not exist;
- every allowed operation/error/context combination and rejection of every
  undefined domain, code, schema, flag, enum and reserved bit;
- exact little-endian encoding, optional-field zeroing and the complete
  64-byte context length;
- one direct diagnostic for a handled anomaly that never enters recovery;
- one and only one diagnostic for soft success, soft-fail/hard-success and
  recovery-exhausted episodes;
- preservation of the original error code while recovery state and severity
  change;
- selection of the last recovery failure detail when both levels fail;
- packet occurrence and airtime-bucket correlation present and absent;
- definite pre-`SetTx` failure versus uncertain command effect and their
  different airtime decisions;
- normal header/CRC/TX-timeout outcomes producing no redundant diagnostic;
- profile publication succeeding even when later diagnostic admission fails;
  and
- process crash or persistence unavailability losing the diagnostic without
  changing the required radio/airtime safety outcome.

## Time diagnostic catalogue

The receiver uses `DiagnosticErrorDomain.TIME = 2` for exceptional failures in
the communicator-owned runtime time policy. The catalogue covers the
`ChronyControl` and `Ds3231Control` adapters, read-only Linux-clock sampling and
the checked calculations that join those inputs. It does not turn every loss
of time trust into a diagnostic.

The authoritative time history remains `ClockObservationV1`; current state and
aggregate command results remain in `ReceiverHealthV1`; and durable RTC
provenance remains in `CommunicatorStateV1`. A time diagnostic explains why an
exceptional operation failed. It neither creates another UTC anchor nor
substitutes for any of those records.

### `TimeDiagnosticErrorCode`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `NONE` | Success only; invalid in a diagnostic |
| `1` | `IO` | A runtime clock, RTC device or fixed helper primitive failed |
| `2` | `DEADLINE` | A bounded time operation did not finish by its monotonic deadline |
| `3` | `INVALID_RESPONSE` | A chrony, kernel-clock or helper response violated its supported structural contract |
| `4` | `COMMAND_REJECTED` | A requested chrony step was definitely not submitted |
| `5` | `OUTCOME_UNKNOWN` | A clock or RTC command may have taken effect but completion is unknown |
| `6` | `CLOCK_INTERFERENCE` | Kernel clock metadata indicates an unapproved writer or state inconsistent with the deployment contract |
| `7` | `RTC_READBACK_MISMATCH` | A completed or possibly applied RTC write did not match its required read-back comparison |
| `8` | `CALCULATION_RANGE` | Valid policy inputs could not be combined using the required checked time arithmetic |

`MISSING` and `INVALID` RTC states are deliberately absent from this error-code
enum. They are expected field conditions represented by `RtcHealth` and the
clock-observation timeline. Chrony being offline or temporarily unavailable,
an unselected or unsynchronized source, a large but valid error bound, ordinary
time-quality loss and expiry of a trusted observation horizon are policy
inputs, not exceptional implementation failures.

`CALCULATION_RANGE` applies only after all input values independently satisfy
their interface ranges. A malformed argument, impossible communicator state or
violated representation invariant uses `CORE`, not this code.

Time diagnostics use these operations:

| Operation | Use |
|---|---|
| `INITIALIZE` | Validating and constructing the fixed chrony, kernel-clock, RTC or helper adapter |
| `READ` | One chrony tracking query, `adjtimex()` sample or direct RTC read |
| `VALIDATE` | Applying the documented trust, interference, range and error-bound rules |
| `SYNC` | One explicit chrony-step command or one complete RTC write/read-back refresh episode |

The valid error-code/operation combinations are closed:

| Error code | Allowed operations |
|---|---|
| `IO` | `INITIALIZE`, `READ`, `SYNC` |
| `DEADLINE` | `READ`, `SYNC` |
| `INVALID_RESPONSE` | `INITIALIZE`, `READ`, `SYNC` |
| `COMMAND_REJECTED` | `SYNC` |
| `OUTCOME_UNKNOWN` | `SYNC` |
| `CLOCK_INTERFERENCE` | `READ`, `VALIDATE` |
| `RTC_READBACK_MISMATCH` | `SYNC` |
| `CALCULATION_RANGE` | `VALIDATE`, `SYNC` |

Every `TIME` diagnostic has severity `ERROR`. Its associated policy path must
fail closed to `UNTRUSTED` or leave a separately valid source unchanged, as
specified by `INTERFACE.md`; the receiver may continue RX. If the observed
condition instead proves a communicator invariant broken and requires process
termination, encode one `CORE + FATAL` diagnostic rather than relabeling a
`TIME` diagnostic.

### Time context-schema assignments

| Value | Name | Length | Receiver use |
|---:|---|---:|---|
| `0` | `NONE` | `0` | Invalid for a nonzero time error |
| `1` | `RECEIVER_TIME_EPISODE_CONTEXT_V1` | `80` | Every receiver time diagnostic |

### Time context sub-enums

`TimeComponent` identifies the stable boundary at which the failure was
observed:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `CHRONY` |
| `2` | `KERNEL_CLOCK` |
| `3` | `DS3231` |
| `4` | `RTC_WRITE_HELPER` |
| `5` | `TIME_POLICY` |

`NONE` is invalid in a time diagnostic. `TimeFailureStage` gives the bounded
public episode stage rather than a Python helper name:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `INITIALIZE_ADAPTER` |
| `2` | `QUERY_TRACKING` |
| `3` | `VALIDATE_TRACKING` |
| `4` | `SAMPLE_SYSTEM_CLOCK` |
| `5` | `CALCULATE_ERROR_BOUND` |
| `6` | `PREPARE_STEP` |
| `7` | `SUBMIT_STEP` |
| `8` | `WAIT_STABLE_TIME` |
| `9` | `READ_RTC` |
| `10` | `DERIVE_RTC_WRITE` |
| `11` | `EXECUTE_RTC_HELPER` |
| `12` | `READ_BACK_RTC` |
| `13` | `VERIFY_RTC_READBACK` |
| `14` | `VALIDATE_RTC_PROVENANCE` |
| `15` | `UPDATE_TIME_STATE` |
| `16` | `SCHEDULE_OBSERVATION` |

`NONE` is invalid. Queue reservation/publication is not a time stage because
its outcome is represented by the admission matrix. A communicator-state
provenance commit is a `PERSISTENCE_CONTROL` operation.

`TimeBackendStatusKind` determines how each status byte is interpreted:

| Value | Name | Status-byte assignments |
|---:|---|---|
| `0` | `NONE` | Status must be zero |
| `1` | `CHRONY_QUERY_STATUS` | `1 OK`, `2 UNAVAILABLE`, `3 DEADLINE_EXCEEDED`, `4 INVALID_RESPONSE` |
| `2` | `CHRONY_STEP_DISPOSITION` | `1 SUBMITTED`, `2 NOT_SUBMITTED`, `3 OUTCOME_UNKNOWN` |
| `3` | `DS3231_READ_STATUS` | `1 OK`, `2 MISSING`, `3 INVALID`, `4 IO_ERROR`, `5 DEADLINE_EXCEEDED` |
| `4` | `DS3231_WRITE_DISPOSITION` | `1 COMPLETED`, `2 NOT_APPLIED`, `3 OUTCOME_UNKNOWN` |
| `5` | `DS3231_FAILURE` | `0 NONE`, `1 MISSING`, `2 IO_ERROR`, `3 DEADLINE_EXCEEDED` |
| `6` | `ADJTIMEX_RETURN` | Linux `TIME_OK=0`, `TIME_INS=1`, `TIME_DEL=2`, `TIME_OOP=3`, `TIME_WAIT=4`, `TIME_ERROR=5` |

These are receiver-diagnostic encodings of runtime results. They do not change
the process-local enum rule in `INTERFACE.md` and are independent from any
library's Python numeric values. For `ADJTIMEX_RETURN`, zero is a valid present
status because the nonzero status kind supplies presence.

### `ReceiverTimeEpisodeContextV1`

Schema `RECEIVER_TIME_EPISODE_CONTEXT_V1 = 1` has this fixed 80-byte encoding:

| Offset | Field | Encoding |
|---:|---|---:|
| `0` | `validity_mask` | `u32` |
| `4` | `component` | `u8` (`TimeComponent`) |
| `5` | `stage` | `u8` (`TimeFailureStage`) |
| `6` | `primary_status_kind` | `u8` (`TimeBackendStatusKind`) |
| `7` | `primary_status` | `u8` |
| `8` | `secondary_status_kind` | `u8` (`TimeBackendStatusKind`) |
| `9` | `secondary_status` | `u8` |
| `10` | `quality_before` | `u8` (`SystemTimeQuality`) |
| `11` | `quality_after` | `u8` (`SystemTimeQuality`) |
| `12` | `rtc_health_before` | `u8` (`RtcHealth`) |
| `13` | `rtc_health_after` | `u8` (`RtcHealth`) |
| `14` | `flags` | `u16` |
| `16` | `os_errno` | `i32` |
| `20` | `kernel_status_bits` | `u32` |
| `24` | `clock_state_generation` | `u64` |
| `32` | `operation_generation` | `u64` |
| `40` | `related_clock_observation_sequence` | `u64` |
| `48` | `observed_value_us` | `i64` |
| `56` | `comparison_value_us` | `i64` |
| `64` | `threshold_us` | `u64` |
| `72` | `operation_duration_us` | `u64` |

Validity bits are:

| Bit | Field |
|---:|---|
| `0` | primary status kind and value |
| `1` | secondary status kind and value |
| `2` | quality before and after |
| `3` | RTC health before and after |
| `4` | `os_errno` |
| `5` | `kernel_status_bits` |
| `6` | `clock_state_generation` |
| `7` | `operation_generation` |
| `8` | `related_clock_observation_sequence` |
| `9` | `observed_value_us` |
| `10` | `comparison_value_us` |
| `11` | `threshold_us` |
| `12`-`31` | Reserved; zero |

When a validity bit is clear, every field it controls is zero. A present
status has a nonzero kind and a value permitted by that kind; an absent status
has both bytes zero. `quality_before/after` and `rtc_health_before/after` are
pair-valued so the context never asserts one side of a transition without the
other. `operation_duration_us` is mandatory checked subtraction of the
adapter or episode's finish sample from its start sample.

Flag bits are:

| Bit | Name |
|---:|---|
| `0` | `SOURCE_SELECTED` |
| `1` | `SYNCHRONIZED` |
| `2` | `COMMAND_MAY_HAVE_APPLIED` |
| `3` | `QUALITY_CHANGED` |
| `4` | `RTC_HEALTH_CHANGED` |
| `5` | `GENERATION_RECHECK_MATCHED` |
| `6` | `READBACK_MATCHED` |
| `7` | `TRUST_SUPPRESSED` |
| `8`-`15` | Reserved; zero |

The two signed values are domain-local evidence. Depending on the stage they
may hold a remaining correction, sampled UTC difference or intended/read-back
UTC difference. Their meaning must be selected by the fixed component/stage
pair; they are not an extensible key-value area. `threshold_us` records the
exact configured comparison threshold only when one governed the failed
decision.

## Time diagnostic emission contract

One explicit chrony-step attempt and one RTC write/read-back/provenance attempt
are each one logical diagnostic episode. The communicator freezes the first
exceptional trigger and later fills only the bounded outcome, transition and
duration fields. A failed read-back does not emit a second diagnostic after a
write failure; the complete refresh produces at most one record whose primary
status retains the root trigger and whose secondary status records the final
read-back result when useful.

A periodic chrony-poll or RTC-read loop keeps one in-memory failure latch. Its
signature is `(component, operation, error_code, stage)`. The first failure
emits one diagnostic; an identical consecutive failure does not. A qualifying
successful invocation clears the latch. A different signature replaces it and
may emit one new diagnostic. The latch is observational process state: it is
not persisted, restored or used to establish clock trust.

The communicator allocates the diagnostic only after applying the required
fail-closed time transition. A failed diagnostic reservation does not delay
that transition, a required clock observation or an RTC-provenance decision.

## Time failure-scenario policy

| Scenario | Diagnostic policy |
|---|---|
| Chrony is offline/unavailable, has no selected synchronized source, reports a valid high error bound, or misses the trust threshold | Apply the time state machine and observations; no diagnostic |
| A valid network or RTC observation error reaches or exceeds the receiver UTC budget | Transition to `UNTRUSTED` and apply the pending-boundary FIFO rule; no diagnostic and no step unless a separate fresh chrony result satisfies the step rule |
| A tracking query reaches its adapter deadline or returns malformed/unsupported output | Emit the first `READ + DEADLINE` or `READ + INVALID_RESPONSE` in the consecutive failure episode and fail closed |
| `adjtimex()` returns the deployment-expected `TIME_ERROR` with `STA_UNSYNC` while chrony is otherwise accepted | No diagnostic and no rejection for that pair alone |
| Other kernel metadata violates the exclusive-writer/clock-sampling contract | Emit `READ` or `VALIDATE + CLOCK_INTERFERENCE`, suppress trust and preserve the raw status bits |
| A checked trust, horizon or write-value calculation overflows despite individually valid inputs | Emit `VALIDATE` or `SYNC + CALCULATION_RANGE` and suppress the derived trusted result |
| A required chrony step is definitely not submitted | Emit one `SYNC + COMMAND_REJECTED`; retain the published discontinuity boundary and follow bounded retry policy |
| Chrony step submission may have occurred but confirmation is lost | Emit one `SYNC + OUTCOME_UNKNOWN`, set `COMMAND_MAY_HAVE_APPLIED` and enter `WAITING_FOR_STABLE_TIME` without blind resubmission |
| RTC read returns `MISSING` or `INVALID` | Update `RtcHealth` and clock observations; no diagnostic |
| RTC read returns `IO_ERROR` or `DEADLINE_EXCEEDED` | Emit the first `READ + IO` or `READ + DEADLINE` in the consecutive failure episode and apply the documented health transition |
| RTC write is `NOT_APPLIED` because of I/O/deadline failure | Emit one `SYNC + IO` or `SYNC + DEADLINE`; follow the required read-back policy only when the DS3231 interface requires it |
| RTC write is `OUTCOME_UNKNOWN` | Emit one `SYNC + OUTCOME_UNKNOWN`, set `COMMAND_MAY_HAVE_APPLIED`, perform the mandatory read-back and never retry blindly |
| Completed or uncertain RTC write has a successful but mismatching read-back | Emit one `SYNC + RTC_READBACK_MISMATCH` for the complete refresh and do not establish provenance |
| The captured generation, time quality or source-error condition changes before provenance commit | Increment the existing RTC trust-invalidated counter and reject provenance; no additional diagnostic |
| Clock-observation queue admission fails | Record only the original `AdmissionResult`; no diagnostic |
| Communicator-state provenance commit fails | Use `PERSISTENCE_CONTROL`, or `CORE` for a caller-contract violation; no time diagnostic |
| Once-per-Linux-boot RTC bootstrap helper fails before the receiver process exists | Keep deployment/service evidence; no receiver `DiagnosticV1` can be guaranteed |

## Required time-diagnostic tests

At minimum, test:

- every allowed time operation/error/status/context combination and rejection
  of undefined codes, stages, status values, flags and reserved bits;
- exact 80-byte little-endian encoding and zeroing of every absent field;
- a present zero-valued `ADJTIMEX_RETURN` distinguished from absent status;
- ordinary offline, unsynchronized, high-error, RTC-missing, RTC-invalid and
  quality-transition paths producing no redundant diagnostic;
- equality and one-unit boundaries around the fixed time-sampling/read-back
  margin and receiver UTC budget, including ordinary budget expiry producing no
  diagnostic and no independent step authority;
- the expected `TIME_ERROR + STA_UNSYNC` deployment pair versus actual clock
  interference;
- first-failure emission, identical consecutive-failure suppression, reset on
  success and a changed failure signature starting a new episode;
- definite chrony-step rejection versus unknown submission outcome;
- RTC write failure plus read-back producing at most one episode diagnostic;
- read-back match, mismatch and generation invalidation taking their distinct
  provenance and diagnostic paths;
- observation-admission and provenance-commit failures remaining in their
  owning admission or persistence-control contracts; and
- diagnostic admission failure leaving time quality, step boundaries and RTC
  provenance decisions unchanged.

## Persistence-control diagnostic catalogue

The receiver uses `DiagnosticErrorDomain.PERSISTENCE_CONTROL = 3` only for an
operational failure returned synchronously by the persistence control channel.
It covers configuration loading, communicator-state loading/commit and the
clean-stop commit. It does not cover ordinary `PersistQueue` admission,
asynchronous batch processing, classification, quarantine or admission-state
publication.

The persistence thread still never creates `DiagnosticV1`. It returns one of
the closed results in `INTERFACE.md`; the communicator maps that result into
this catalogue and makes at most one ordinary best-effort diagnostic admission.
A `PersistenceControlInterfaceViolation` means the communicator called the
interface incorrectly and therefore uses `CORE`, not this domain.

### `PersistenceControlDiagnosticErrorCode`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `NONE` | Success only; invalid in a diagnostic |
| `1` | `CONFIGURATION_REJECTED` | The protocol loader rejected receiver configuration content or file-security policy |
| `2` | `HOST_IDENTITY_REJECTED` | The current Linux boot identity was readable but not one canonical UUID |
| `3` | `STATE_MISSING` | The communicator-state singleton is absent and conservative generation zero is in use |
| `4` | `STATE_CORRUPT` | The state SQL envelope, digest, canonical encoding or invariants failed validation |
| `5` | `UNSUPPORTED_STATE_VERSION` | The stored state format is unsupported and requires guarded no-TX recovery before replacement |
| `6` | `STATE_POLICY_MISMATCH` | Stored airtime-policy parameters do not equal the active deployment policy |
| `7` | `IO` | A non-SQLite filesystem or host-identity operation failed |
| `8` | `DATABASE` | SQLite or its underlying database storage returned an operational failure |
| `9` | `DEADLINE` | The synchronous control command reached its absolute monotonic deadline |
| `10` | `CHANNEL_CLOSED` | The control channel closed before the command could complete |

Result mapping is exact:

- `ReceiverConfigurationLoadStatus.CONFIGURATION_REJECTED` maps to
  `CONFIGURATION_REJECTED`;
- `HOST_IDENTITY_REJECTED` maps to the same-named code;
- a `STATE_UNAVAILABLE` result maps its nonzero
  `CommunicatorStateCondition` to one of the four state codes;
- `OS_ERROR`, `DATABASE_ERROR`, `DEADLINE_EXCEEDED` and `CHANNEL_CLOSED` map to
  `IO`, `DATABASE`, `DEADLINE` and `CHANNEL_CLOSED`; and
- any result carrying a non-`NONE` interface violation maps to
  `CORE + PERSISTENCE_CONTROL_CONTRACT` instead.

The diagnostic layer uses the one primary `CommunicatorStateCondition` produced
by the ordered state-row decision tree in `INTERFACE.md`. It never re-examines
the row or substitutes a secondary apparent defect, so classification and
automatic-recovery permission cannot diverge from the persistence result.

The command disposition is not an error code. In particular,
`OUTCOME_UNKNOWN` is encoded in context while `DATABASE` or `DEADLINE` retains
the known failure reason. A result combination that cannot be produced by
`INTERFACE.md` is a `CORE` invariant failure.

Persistence-control diagnostics use these operations:

| Operation | Use |
|---|---|
| `READ` | Loading receiver configuration or communicator state |
| `WRITE` | Committing a complete next-generation communicator state |
| `CLEANUP` | Committing the controlled receiver clean-stop marker |

The valid error-code/operation combinations are closed:

| Error code | Allowed operations |
|---|---|
| `CONFIGURATION_REJECTED` | `READ` |
| `HOST_IDENTITY_REJECTED` | `READ` |
| `STATE_MISSING` | `READ` |
| `STATE_CORRUPT` | `READ` |
| `UNSUPPORTED_STATE_VERSION` | `READ`, `WRITE` |
| `STATE_POLICY_MISMATCH` | `READ`, `WRITE` |
| `IO` | `READ`, `WRITE`, `CLEANUP` |
| `DATABASE` | `READ`, `WRITE`, `CLEANUP` |
| `DEADLINE` | `READ`, `WRITE`, `CLEANUP` |
| `CHANNEL_CLOSED` | `READ`, `WRITE`, `CLEANUP` |

`STATE_MISSING` is `WARN` when startup intentionally adopts conservative
generation zero. The other state-condition codes are `ERROR`: RX may continue
where the architecture permits it, but TX remains suppressed until the
condition is resolved. A failure of startup receiver-configuration or host-ID
loading is `FATAL` because normal radio operation cannot start. Other control
failures are `ERROR` while the communicator can continue safely and `FATAL`
only when the receiver instance terminates because the required startup or
shutdown/control state cannot be established.

### Persistence-control context-schema assignments

| Value | Name | Length | Receiver use |
|---:|---|---:|---|
| `0` | `NONE` | `0` | Invalid for a nonzero persistence-control error |
| `1` | `RECEIVER_PERSISTENCE_CONTROL_CONTEXT_V1` | `64` | Every persistence-control diagnostic |

### Persistence-control context sub-enums

`PersistenceControlCommand` identifies the public call:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `LOAD_RECEIVER_CONFIGURATION` |
| `2` | `LOAD_COMMUNICATOR_STATE` |
| `3` | `COMMIT_COMMUNICATOR_STATE` |
| `4` | `COMMIT_RECEIVER_CLEAN_STOP` |

`NONE` is invalid. `PersistenceControlPurpose` records why the communicator
made a state-changing or reconciliation request:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `STARTUP_CONFIGURATION` |
| `2` | `STARTUP_STATE` |
| `3` | `AIRTIME_BUCKET_GRANT` |
| `4` | `AIRTIME_BUCKET_SETTLEMENT` |
| `5` | `RTC_PROVENANCE` |
| `6` | `AIRTIME_HISTORY_RECOVERY` |
| `7` | `CLEAN_STOP` |
| `8` | `RECONCILIATION` |

`STARTUP_CONFIGURATION` is required for configuration load and
`STARTUP_STATE` for the initial state load. Later state loads use
`RECONCILIATION`. A state commit selects the policy mutation that caused the
new generation; it does not use `NONE`. Clean-stop uses `CLEAN_STOP`.

`PersistenceControlDispositionKind` makes the disposition byte unambiguous:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `COMMUNICATOR_STATE_COMMIT` |
| `2` | `RECEIVER_CLEAN_STOP_COMMIT` |

When present, `PersistenceControlDisposition` has these receiver-diagnostic
assignments:

| Value | Name | Source result |
|---:|---|---|
| `0` | `NOT_APPLICABLE` | Invalid when a disposition kind is present |
| `1` | `COMMITTED` | Source `COMMITTED` |
| `2` | `ALREADY_COMMITTED` | Source `ALREADY_COMMITTED` |
| `3` | `DEFINITELY_NOT_COMMITTED` | Source `NOT_INSTALLED` or `NOT_COMMITTED` |
| `4` | `OUTCOME_UNKNOWN` | Source `OUTCOME_UNKNOWN` |

`PersistenceControlFailureKind` normalizes the command-specific result enums:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `CONFIGURATION_REJECTED` |
| `2` | `HOST_IDENTITY_REJECTED` |
| `3` | `STATE_UNAVAILABLE` |
| `4` | `IO_ERROR` |
| `5` | `DATABASE_ERROR` |
| `6` | `DEADLINE_EXCEEDED` |
| `7` | `CHANNEL_CLOSED` |

`PersistenceControlStateCondition` supplies the context's stable assignments:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `MISSING` |
| `2` | `CORRUPT` |
| `3` | `UNSUPPORTED_VERSION` |
| `4` | `POLICY_MISMATCH` |

They mirror the process-local names but do not rely on their Python numeric
values.

### `ReceiverPersistenceControlContextV1`

Schema `RECEIVER_PERSISTENCE_CONTROL_CONTEXT_V1 = 1` has this fixed 64-byte
encoding:

| Offset | Field | Encoding |
|---:|---|---:|
| `0` | `validity_mask` | `u16` |
| `2` | `control_command` | `u8` (`PersistenceControlCommand`) |
| `3` | `purpose` | `u8` (`PersistenceControlPurpose`) |
| `4` | `disposition_kind` | `u8` (`PersistenceControlDispositionKind`) |
| `5` | `disposition` | `u8` (`PersistenceControlDisposition`) |
| `6` | `failure_kind` | `u8` (`PersistenceControlFailureKind`) |
| `7` | `state_condition` | `u8` (`PersistenceControlStateCondition`) |
| `8` | `protocol_rejection_code` | `u16` |
| `10` | `flags` | `u16` |
| `12` | `sqlite_primary_code` | `i32` |
| `16` | `sqlite_extended_code` | `i32` |
| `20` | `os_errno` | `i32` |
| `24` | `requested_generation` | `u64` |
| `32` | `authoritative_generation_before` | `u64` |
| `40` | `related_occurrence_sequence` | `u64` |
| `48` | `airtime_bucket_expiration_utc_us` | `i64` |
| `56` | `operation_duration_us` | `u64` |

Validity bits are:

| Bit | Field |
|---:|---|
| `0` | disposition kind and disposition |
| `1` | `state_condition` |
| `2` | `protocol_rejection_code` |
| `3` | `sqlite_primary_code` |
| `4` | `sqlite_extended_code` |
| `5` | `os_errno` |
| `6` | `requested_generation` |
| `7` | `authoritative_generation_before` |
| `8` | `related_occurrence_sequence` |
| `9` | `airtime_bucket_expiration_utc_us` |
| `10`-`15` | Reserved; zero |

An absent optional field is zero. SQLite extended code requires the primary
code to be present. `protocol_rejection_code` is the exact stable unsigned
protocol-loader rejection value returned by the protocol interface; it is not
a Python exception class or message. If the protocol result has no such stable
value, this field is absent. `operation_duration_us` is always present and is
checked subtraction from submission to immutable completion result.

Flag bits are:

| Bit | Name |
|---:|---|
| `0` | `MUTATING` |
| `1` | `COMMIT_MAY_HAVE_RUN` |
| `2` | `RECONCILIATION` |
| `3` | `STARTUP` |
| `4` | `SHUTDOWN` |
| `5` | `TX_SUPPRESSED` |
| `6`-`15` | Reserved; zero |

`MUTATING` is set exactly for commit commands. `COMMIT_MAY_HAVE_RUN` requires a
present `OUTCOME_UNKNOWN` disposition. The occurrence sequence is scoped by the
base diagnostic's `receiver_instance_id`. The bucket expiration is present only
when the failed state command granted or settled that durable logical bucket.

`failure_kind` is nonzero and exactly matches the normalized failed source
result for every persistence-control diagnostic. `state_condition` is present
exactly for a state-condition error. Configuration and read-only state-load
diagnostics have no disposition; commit diagnostics always carry their exact
disposition kind and disposition.

## Persistence-control diagnostic emission contract

One public control command and any mandatory exact reconciliation of its
unknown result form one logical failure episode. The communicator retains the
first returned operational failure as the root and may add the final
disposition and authoritative generation before constructing at most one
diagnostic. Repeating an exact command to reconcile `OUTCOME_UNKNOWN` does not
emit an independent diagnostic merely because it observes the same root
failure. A later failure with a different actionable code, or a new control
purpose/generation after the earlier episode ended, is a new episode.

For startup conditions that are sampled once, the communicator emits at most
one diagnostic for the receiver instance. A successful or `ALREADY_COMMITTED`
command emits no diagnostic. Recovery, TX suppression and process termination
follow the control result even when diagnostic admission is unavailable.

## Persistence-control failure-scenario policy

| Scenario | Diagnostic policy |
|---|---|
| Receiver configuration is rejected | Emit one `READ + CONFIGURATION_REJECTED + FATAL`, preserve the stable protocol rejection code when supplied and do not start normal radio operation |
| Linux boot identity is noncanonical | Emit one `READ + HOST_IDENTITY_REJECTED + FATAL` and do not start normal radio operation |
| Communicator-state row is missing | Adopt conservative generation zero and emit one startup `READ + STATE_MISSING + WARN`; after trusted UTC is available, commit the exact synthetic worst-case generation-one ledger before TX |
| Communicator-state row is corrupt | Emit one `READ + STATE_CORRUPT + ERROR`, use conservative runtime state, suppress TX, then atomically preserve the rejected rows and install the exact synthetic worst-case generation-one ledger before TX |
| State version is unsupported or policy mismatches | Emit one corresponding `READ + ERROR`, suppress TX for the complete conservative active-policy rolling-window wait, then archive the exact old singleton and atomically install an empty current-policy generation-one ledger; restart restarts the wait |
| Configuration filesystem or host-ID read fails | Emit `READ + IO`; it is `FATAL` when startup cannot establish receiver identity/configuration |
| State load/commit or clean-stop SQLite operation fails | Emit the applicable `DATABASE`; preserve exact SQLite and errno fields, apply admission/recovery policy and retain any unresolved command serialization |
| A read-only command reaches its deadline | Emit `READ + DEADLINE`; no durable effect ambiguity exists |
| A mutating command fails before `COMMIT` | Emit `WRITE` or `CLEANUP` with its root code and `DEFINITELY_NOT_COMMITTED`; the preceding authoritative state remains in force |
| A mutating command may have crossed `COMMIT` | Emit the root `DATABASE` or `DEADLINE` code with `OUTCOME_UNKNOWN` and `COMMIT_MAY_HAVE_RUN`; freeze TX where required and reconcile exact bytes before proceeding |
| The channel is closed as an expected final shutdown action after all required work completed | No diagnostic |
| The channel closes before a required command completes | Emit the applicable operation plus `CHANNEL_CLOSED`; follow the command's definite/unknown disposition |
| A result carries a non-`NONE` `PersistenceControlInterfaceViolation` or an impossible field combination | Use `CORE + PERSISTENCE_CONTROL_CONTRACT`; do not emit a persistence-control diagnostic |
| Ordinary batch, quarantine, admission-state or asynchronous SQLite failure occurs | No `DiagnosticV1` is created by persistence; use its owning state, health, quarantine and service evidence |

Because this diagnostic uses the same queue and SQLite database as the failed
subsystem, the record may be impossible to retain during a persistence outage.
That is the accepted persistence-unavailable observability limitation, not a
reason to create a side file or let persistence insert its own diagnostic.

## Required persistence-control diagnostic tests

At minimum, test:

- exact mapping of every configuration, state-load, state-commit and clean-stop
  result into a permitted error/operation/context combination;
- rejection of every impossible source-result combination before diagnostic
  construction;
- exact 64-byte little-endian encoding, protocol-code handling and optional
  SQLite/errno/correlation zeroing;
- missing, corrupt, unsupported-version and policy-mismatch state producing
  their distinct severity and recovery decisions;
- exact synthetic worst-case bucket construction for missing/corrupt state,
  including the pilot `[4, 8, 8, 8, 8]` second ledger;
- complete conservative rolling-window suppression for unsupported-version and
  policy-mismatch state, including restart of the wait after process restart;
- combined state defects mapping from the primary ordered condition, including
  bad-digest plus unknown-version as `STATE_CORRUPT` and structural-failure plus
  policy-mismatch as `STATE_CORRUPT`;
- known-not-committed versus unknown commit outcome and exact reconciliation;
- one logical diagnostic across an unknown command and its mandatory
  reconciliation rather than one record per control call;
- success, exact `ALREADY_COMMITTED` and expected final channel closure
  producing no diagnostic;
- every caller interface violation selecting `CORE` with the matching detail
  rather than this domain; and
- persistence unavailability losing the best-effort diagnostic without
  changing queue ownership, TX suppression or control reconciliation.

## Core diagnostic catalogue

The receiver uses `DiagnosticErrorDomain.CORE = 4` for communicator
implementation failures: impossible orchestration state, fixed-representation
failure, failure of a required local codec/crypto primitive, and caller-side
violation of `PersistQueue` or the synchronous persistence-control interface.
It is not a catch-all for ordinary radio, protocol, time or storage outcomes.

All `CORE` diagnostics are `FATAL` in the pilot. The communicator first follows
the bounded exception-finalization and safe-radio policy where control remains
available, then terminates the receiver instance so systemd can apply its
restart policy. Retrying the same violated invariant inside the process is not
recovery.

### `CoreDiagnosticErrorCode`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `NONE` | Success only; invalid in a diagnostic |
| `1` | `INVALID_ARGUMENT` | Communicator code supplied an invalid value to an internal receiver interface not covered by a more specific contract code |
| `2` | `INVALID_STATE` | The communicator reached an impossible orchestration state or transition |
| `3` | `REPRESENTATION_INVARIANT` | A supposedly validated fixed-size entity or immutable receiver value could not be represented canonically |
| `4` | `PERSIST_QUEUE_CONTRACT` | The communicator violated the producer side of the `PersistQueue` contract |
| `5` | `PERSISTENCE_CONTROL_CONTRACT` | A synchronous control result reports a caller-side `PersistenceControlInterfaceViolation` |
| `6` | `MEMORY_EXHAUSTED` | Required bounded runtime allocation failed outside normal queue-capacity admission |
| `7` | `CODEC_BACKEND` | A protocol codec failed outside a documented malformed-input result |
| `8` | `CRYPTO_BACKEND` | A configured cryptographic primitive failed outside ordinary authentication failure |
| `9` | `ARITHMETIC_RANGE` | Checked non-time arithmetic crossed a required receiver representation bound |
| `10` | `UNEXPECTED_EXCEPTION` | An unclassified implementation exception escaped a boundary that admits no operational result |

Malformed or unsupported uplinks, failed authentication, wrong-direction
frames and protocol-selected silence/rejection are `MessageProfilingV1`
outcomes. `QUEUE_FULL` and `PERSISTENCE_UNAVAILABLE` are `AdmissionResult`
values. Expected duplicate/conflict classification and poisoned-unit isolation
also remain in their existing contracts. None maps to `CORE`.

The valid error-code/operation combinations are closed:

| Error code | Allowed operations |
|---|---|
| `INVALID_ARGUMENT` | `VALIDATE` |
| `INVALID_STATE` | `INITIALIZE`, `VALIDATE`, `CLEANUP` |
| `REPRESENTATION_INVARIANT` | `ENCODE`, `DECODE`, `APPEND` |
| `PERSIST_QUEUE_CONTRACT` | `APPEND`, `CLEANUP` |
| `PERSISTENCE_CONTROL_CONTRACT` | `READ`, `WRITE`, `CLEANUP` |
| `MEMORY_EXHAUSTED` | `INITIALIZE`, `ENCODE`, `DECODE`, `APPEND`, `TRANSMIT`, `RECEIVE`, `CLEANUP` |
| `CODEC_BACKEND` | `ENCODE`, `DECODE` |
| `CRYPTO_BACKEND` | `ENCODE`, `DECODE` |
| `ARITHMETIC_RANGE` | `VALIDATE`, `ENCODE`, `DECODE` |
| `UNEXPECTED_EXCEPTION` | `INITIALIZE`, `VALIDATE`, `ENCODE`, `DECODE`, `APPEND`, `TRANSMIT`, `RECEIVE`, `RECOVER`, `CLEANUP` |

An expected OS/driver error must first be normalized by its owning adapter and
uses that domain. `UNEXPECTED_EXCEPTION` is reserved for a genuine code-path
escape; it must not be used to avoid defining a reachable operational result.
Exception class names, messages, tracebacks and object representations are not
stored in `DiagnosticV1`.

### Core context-schema assignments

| Value | Name | Length | Receiver use |
|---:|---|---:|---|
| `0` | `NONE` | `0` | Invalid for a nonzero core error |
| `1` | `RECEIVER_CORE_FAILURE_CONTEXT_V1` | `64` | Every receiver core diagnostic |

### Core context sub-enums

`CorePhase` identifies the stable communicator phase:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `STARTUP` |
| `2` | `IDLE` |
| `3` | `PACKET_PROCESSING` |
| `4` | `ACK_PREPARATION` |
| `5` | `POST_RESPONSE_FINALIZATION` |
| `6` | `PERIODIC_HEALTH` |
| `7` | `PERIODIC_TIME` |
| `8` | `AIRTIME_STATE` |
| `9` | `CONTROL_OPERATION` |
| `10` | `SHUTDOWN` |

`NONE` is invalid. `CoreFailureStage` locates the stable action without
recording a private function name:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `VALIDATE_ARGUMENT` |
| `2` | `TRANSITION_STATE` |
| `3` | `DERIVE_KEY` |
| `4` | `AUTHENTICATE_FRAME` |
| `5` | `DECODE_FRAME` |
| `6` | `VALIDATE_MESSAGE` |
| `7` | `CONSTRUCT_ACK` |
| `8` | `ENCRYPT_ACK` |
| `9` | `CONSTRUCT_ENTITY` |
| `10` | `VALIDATE_ENTITY` |
| `11` | `RESERVE_QUEUE` |
| `12` | `PUBLISH_QUEUE` |
| `13` | `CANCEL_QUEUE` |
| `14` | `LOAD_CONFIGURATION` |
| `15` | `LOAD_STATE` |
| `16` | `COMMIT_STATE` |
| `17` | `COMMIT_CLEAN_STOP` |
| `18` | `FINALIZE_EXCEPTION` |
| `19` | `ALLOCATE_MEMORY` |
| `20` | `INVOKE_ADAPTER` |

`NONE` is invalid. Time-policy arithmetic uses the `TIME` stages; normalized
radio failures use the `RADIO` stages. `INVOKE_ADAPTER` is only for an
unclassified exception that escaped a boundary whose expected failures should
already have been normalized.

`CoreDetailKind` selects the only two closed detail-code catalogues:

| Value | Name |
|---:|---|
| `0` | `NONE` |
| `1` | `PERSIST_QUEUE_VIOLATION` |
| `2` | `PERSISTENCE_CONTROL_VIOLATION` |

For `PERSIST_QUEUE_VIOLATION`, `detail_code` uses
`PersistQueueViolationDetailCode`:

| Value | Name |
|---:|---|
| `1` | `INVALID_SPEC` |
| `2` | `PRODUCER_CLOSED` |
| `3` | `INVALID_TOKEN` |
| `4` | `BUFFER_SPEC_MISMATCH` |
| `5` | `USE_AFTER_TRANSFER` |
| `6` | `DOUBLE_TRANSITION` |
| `7` | `INVALID_PUBLICATION_CONTENT` |

These codes cover only communicator-side calls. Overlapping consumer claims,
batch-disposition mismatch and stale-prefix acknowledgement occur in the
persistence thread; they cannot safely create a communicator diagnostic
through the same queue and instead follow persistence unavailability and
bounded service-log policy.

For `PERSISTENCE_CONTROL_VIOLATION`, `detail_code` uses
`PersistenceControlViolationDetailCode`:

| Value | Name |
|---:|---|
| `1` | `INVALID_ARGUMENT` |
| `2` | `INVALID_DEADLINE` |
| `3` | `WRONG_CALLER` |
| `4` | `INVALID_STATE` |
| `5` | `GENERATION_CONTENT_CONFLICT` |
| `6` | `STALE_GENERATION` |
| `7` | `GENERATION_GAP` |
| `8` | `CLEAN_STOP_PRECONDITION` |
| `9` | `CLEAN_STOP_CONFLICT` |

The assignments mirror the names in `PersistenceControlInterfaceViolation`
but are stable receiver-diagnostic values, independent from Python enum
numbers.

### `ReceiverCoreFailureContextV1`

Schema `RECEIVER_CORE_FAILURE_CONTEXT_V1 = 1` has this fixed 64-byte encoding:

| Offset | Field | Encoding |
|---:|---|---:|
| `0` | `validity_mask` | `u16` |
| `2` | `phase` | `u8` (`CorePhase`) |
| `3` | `stage` | `u8` (`CoreFailureStage`) |
| `4` | `detail_kind` | `u8` (`CoreDetailKind`) |
| `5` | `related_entity_kind` | `u8` (`PersistQueueEntityKind`) |
| `6` | `flags` | `u16` |
| `8` | `detail_code` | `i32` |
| `12` | `os_errno` | `i32` |
| `16` | `related_occurrence_sequence` | `u64` |
| `24` | `related_health_sequence` | `u64` |
| `32` | `related_clock_observation_sequence` | `u64` |
| `40` | `communicator_state_generation` | `u64` |
| `48` | `airtime_bucket_expiration_utc_us` | `i64` |
| `56` | `operation_duration_us` | `u64` |

Validity bits are:

| Bit | Field |
|---:|---|
| `0` | `detail_kind` and `detail_code` |
| `1` | `os_errno` |
| `2` | `related_occurrence_sequence` |
| `3` | `related_health_sequence` |
| `4` | `related_clock_observation_sequence` |
| `5` | `communicator_state_generation` |
| `6` | `airtime_bucket_expiration_utc_us` |
| `7`-`15` | Reserved; zero |

Flag bits are:

| Bit | Name |
|---:|---|
| `0` | `ENTITY_KIND_VALID` |
| `1` | `AIRTIME_GRANT_OUTSTANDING` |
| `2` | `OCCURRENCE_ACCEPTED` |
| `3` | `ACK_SELECTED` |
| `4` | `TX_MAY_HAVE_STARTED` |
| `5` | `PROFILE_PUBLISHED` |
| `6` | `QUEUE_KNOWN_SOUND` |
| `7` | `SAFE_RADIO_STATE_CONFIRMED` |
| `8`-`15` | Reserved; zero |

When `ENTITY_KIND_VALID` is clear, `related_entity_kind` is zero; when set, it
must be a defined `PersistQueueEntityKind`. When validity bit 0 is clear,
`detail_kind` and `detail_code` are zero. When set, both are nonzero and the
code belongs to the selected detail kind. All other absent fields are zero.
`operation_duration_us` is mandatory checked elapsed time from the first
observed failure to finalization and may be zero for adjacent monotonic reads.

The sequence fields are scoped by the base diagnostic's
`receiver_instance_id`; the airtime bucket correlation is its absolute durable
expiration. Flags describe facts already known when the diagnostic is
finalized; none permits retry, ACK or queue behavior that its owning interface
otherwise forbids.

## Core diagnostic emission contract

A core failure creates at most one best-effort diagnostic. If a packet unit is
already reserved and its required terminal profiling fields can still be
completed, the communicator first finalizes and publishes that unit as
`UNKNOWN_INTERRUPTED`, then attempts the separate diagnostic. It attempts the
documented safe radio state and terminates regardless of either admission
result.

A producer-side queue violation may use `PersistQueue` for its diagnostic only
when the rejected operation is known not to have mutated queue ownership and
the queue implementation explicitly reports that subsequent ordinary
reservations remain sound. The context then sets `QUEUE_KNOWN_SOUND`. Otherwise
the communicator makes no further queue call, records the bounded service log
when possible and terminates. It never probes queue soundness by attempting a
diagnostic reservation.

`MemoryError` and an exception in diagnostic construction may make even the
best-effort fixed diagnostic impossible. The communicator does not allocate a
second object or recursively diagnose that failure. Service logs may retain an
exception traceback for operators, but unrestricted text never enters the
fixed diagnostic entity.

When a core failure is observed while radio recovery would otherwise be
needed, the root remains `CORE`; the communicator does not also open a radio
episode. The safe-radio attempt and health counters still follow the radio
safety contract, while this diagnostic records the terminal implementation
failure.

## Core failure-scenario policy

| Scenario | Diagnostic policy |
|---|---|
| Malformed, unauthenticated, unsupported or wrong-direction radio frame | No core diagnostic; record the protocol/profile outcome |
| Queue returns `QUEUE_FULL` or `PERSISTENCE_UNAVAILABLE` | No core diagnostic; increment only the original admission-result cell |
| Communicator calls a queue method with an invalid spec/token/buffer or violates reservation ownership | Emit `APPEND` or `CLEANUP + PERSIST_QUEUE_CONTRACT` only if the queue is explicitly known sound, then terminate |
| Persistence consumer detects overlapping claims, invalid disposition count or stale-prefix acknowledgement | No communicator diagnostic through that queue; close admission, retain safe ownership and use bounded service evidence |
| Persistence control returns a caller interface violation | Emit `READ`, `WRITE` or `CLEANUP + PERSISTENCE_CONTROL_CONTRACT` with the exact normalized violation and terminate |
| Canonical fixed-size construction fails after its inputs passed validation | Emit `ENCODE` or `APPEND + REPRESENTATION_INVARIANT` and terminate before selecting another protocol outcome |
| Protocol decoding rejects malformed external bytes normally | No core diagnostic |
| Codec or crypto library fails outside its documented malformed/authentication result | Emit `DECODE` or `ENCODE + CODEC_BACKEND`/`CRYPTO_BACKEND` and terminate; never include key material or plaintext in context |
| Required checked non-time arithmetic exceeds its allowed representation | Emit the applicable operation plus `ARITHMETIC_RANGE` and terminate |
| Process-level memory allocation fails | Attempt one allocation-free/preallocated `MEMORY_EXHAUSTED` diagnostic only if feasible, then terminate; inability to admit it creates no recursion |
| An exception escapes an adapter or communicator phase with no defined operational mapping | Emit one `UNEXPECTED_EXCEPTION` with phase/stage only, perform exception finalization and terminate; implementation review must decide whether a later catalogue revision should normalize it |
| Top-level packet exception leaves ACK terminal outcome unknown | Complete the reserved profile as `UNKNOWN_INTERRUPTED` when possible, publish it before the diagnostic, establish a safe radio state and terminate |
| Diagnostic reservation or publication itself fails | Do not construct another diagnostic or make another capacity attempt; preserve the original failure through service logging when possible and terminate |

## Required core-diagnostic tests

At minimum, test:

- every allowed core operation/error/context combination and rejection of
  undefined phases, stages, detail kinds/codes, flags and reserved bits;
- exact 64-byte little-endian encoding and zeroing of all absent correlation
  and detail fields;
- one-to-one mapping of every producer-side queue violation and every
  `PersistenceControlInterfaceViolation`;
- ordinary packet validation, queue admission, duplicate classification and
  poison isolation producing no core diagnostic;
- queue-known-sound violation allowing at most one best-effort diagnostic and
  queue-uncertain violation making no further queue call;
- post-acceptance top-level exception publishing an `UNKNOWN_INTERRUPTED`
  profile before the diagnostic when publication remains safe;
- codec, crypto and arithmetic backend failures distinguished from malformed
  external input;
- unclassified adapter exception selecting `CORE` without also creating a
  radio/time diagnostic;
- every core path attempting bounded safe-state handling and terminating the
  receiver instance; and
- `MemoryError`, diagnostic-construction failure and diagnostic-admission
  failure producing no recursion.

## Cross-domain selection and completeness

The communicator selects exactly one root domain for one exceptional trigger:

1. use `RADIO` for a normalized radio/backend failure and its entire bounded
   recovery episode;
2. use `TIME` for an exceptional runtime time-adapter or time-policy failure;
3. use `PERSISTENCE_CONTROL` for an operational result returned synchronously
   by that channel; and
4. use `CORE` for communicator implementation failure or caller-contract
   violation.

An error in a later independent operation may create a second diagnostic and
carry the same correlation sequence, but a recovery stage, read-back or
reconciliation that belongs to the original logical episode updates that one
bounded record instead. The implementation must reject every combination not
explicitly admitted by its domain tables.

There is intentionally no pilot `PERSIST_QUEUE` or asynchronous
`PERSISTENCE` diagnostic domain. Admission results, profiles, admission state,
health counters, immutable canonical rows and quarantine already own those
facts, while an asynchronous persistence failure cannot reliably report
through the same failing queue/database path. The accepted limitation is
documented in `ARCHITECTURE.md` and does not weaken fail-closed behavior.

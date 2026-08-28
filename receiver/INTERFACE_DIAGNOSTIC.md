# Receiver diagnostic interface

Status: provisional pilot contract. This document defines the receiver's
diagnostic entity, stable common enums and domain-local diagnostic catalogues.
[`INTERFACE.md`](INTERFACE.md) remains authoritative for common encodings,
identities, `PersistQueue`, SQLite replay and receiver state enums.

Only the radio catalogue is defined here initially. A receiver diagnostic must
not use another domain until that domain's error codes, allowed operations and
every referenced context schema have been added to this document and to the
generated receiver enum source of truth.

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
on the change. Before the first deployed migration, assignments may be revised
directly. Once persisted assignments have been deployed, they remain
append-only and an incompatible encoding requires a new schema version rather
than reinterpretation of stored rows.

## Diagnostic role

Diagnostics record exceptional communicator failures and bounded recovery
episodes. They do not duplicate:

- ordinary protocol outcomes already present in `MessageProfilingV1`;
- duplicate/conflict classifications and canonical transport/reading evidence;
- aggregate rates already represented by `ReceiverHealthV1`; or
- exact poisoned-unit bytes and failure provenance in quarantine tables.

Only the communicator allocates diagnostic identity, constructs
`DiagnosticV1` and attempts its nonblocking `PersistQueue` admission. The
persistence thread may return a synchronous control failure for the
communicator to convert only after that failure's receiver-local catalogue has
been defined, but it never creates a diagnostic identity or inserts a
persistence-created diagnostic.

Diagnostics are immutable once published. Failure to admit a diagnostic never
recursively creates another diagnostic. The communicator increments its
bounded in-memory failure counter and may write one bounded best-effort message
to the service log.

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
the complete episode duration. The persistence thread adds `linux_boot_id` to
the stored row. The queued and stored representations remain monotonic-only;
analysis may derive UTC from `ClockObservationV1` without updating the row.

`(receiver_instance_id, diagnostic_sequence)` is the durable identity.
Ordinary replay compares every queued field plus the persistence-added
`linux_boot_id`; exact equality is no-op success and any difference is the
global identity-invariant failure defined by `INTERFACE.md`.

`context_schema = 0` if and only if `context_length = 0`. Every unused context
byte is zero. A nonempty context requires an operation other than `NONE`.
Context never contains keys, unrestricted plaintext, arbitrary exception text,
object dumps or private Python representations.

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

`NONE` is valid only when no operation is meaningful and therefore cannot be
used with the radio context below. New catalogues reuse these assignments
unless they genuinely introduce a new semantic action.

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
an unclassified implementation exception belongs to the future receiver-core
catalogue and the supervisor's service log.

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
| `48` | `airtime_reservation_sequence` | `u64` |
| `56` | `episode_duration_us` | `u64` |

Validity bits are:

| Bit | Field |
|---:|---|
| `0` | `last_recovery_failure_detail` and `last_recovery_error_code` |
| `1` | `related_occurrence_sequence` |
| `2` | `airtime_reservation_sequence` |
| `3`–`15` | Reserved; zero |

When bit 0 is clear, the complete second detail and its error code are zero.
When bit 1 or 2 is clear, its sequence is zero. The airtime-reservation owner is
the base diagnostic's `receiver_instance_id`, so only its per-instance sequence
is repeated. `episode_duration_us` is checked subtraction from the trigger time
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
TX is not a radio-recovery stage. It will use its future persistence/core
catalogue and may carry the same occurrence or airtime-reservation sequence for
analysis. It must not be folded into the radio diagnostic merely because both
were observed while handling the same packet.

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
- packet occurrence and airtime-reservation correlation present and absent;
- definite pre-`SetTx` failure versus uncertain command effect and their
  different airtime decisions;
- normal header/CRC/TX-timeout outcomes producing no redundant diagnostic;
- profile publication succeeding even when later diagnostic admission fails;
  and
- process crash or persistence unavailability losing the diagnostic without
  changing the required radio/airtime safety outcome.

# Receiver Architecture

## Purpose

This document describes the pilot architecture for the Raspberry Pi LoRa receiver connected to an SX1262.

The intended audience is an AI agent working inside the repository. The agent is expected to have access to:

- the node firmware;
- the existing LoRa protocol implementation and generated codecs;
- packet and ACK definitions;
- repository-specific entity and persistence code.

This document intentionally focuses on receiver architecture rather than protocol details. The repository should be treated as the source of truth for packet formats, authentication rules, counters, nonces, ACK codes, and retry behavior.

## Pilot goals

The pilot should:

- receive packets reliably from one or two field nodes;
- authenticate and decode packets;
- send an authenticated ACK with low and measurable latency;
- persist measurements, diagnostics, and timing data to SQLite;
- collect enough telemetry to guide later versions;
- remain simple enough to deploy quickly and debug in the field.

Performance is not the primary goal. The design favors clear ownership, observability, and simple failure semantics.

## High-level design

The receiver uses two Python threads:

1. **Communicator thread**
   - owns the SX1262, GPIO lines, SPI device, protocol state, duplicate state, and ACK generation;
   - owns the in-memory receiver state used for clock provenance and receiver TX-airtime admission;
   - performs only bounded in-memory work on the packet-to-ACK path;
   - never performs disk I/O or accesses SQLite.

2. **Persistence thread**
   - is the only thread allowed to access SQLite or the durable receiver-state file;
   - drains a bounded in-memory queue in batches;
   - records measurements, message statistics, and diagnostics;
   - services high-priority synchronous receiver-state commit requests.

The threads communicate through two paths with different guarantees:

- bounded `PersistQueue` admission is asynchronous and does not imply disk durability;
- `StateCommitRequest` is a high-priority synchronous request whose completion acknowledges a durable receiver-state generation.

```text
SX1262
  |
  | DIO1 / BUSY / SPI
  v
Communicator thread
  |
  |-- measurement entities / statistics / diagnostics --> PersistQueue --|
  |                                                                  |
  `-- receiver-state snapshot --> StateCommitRequest -----------------|
                                                                     v
Persistence thread
  |-- SQLite
  `-- durable receiver-state file
```

## Core ownership rules

These ownership rules should remain strict:

- Only the communicator thread touches the SX1262.
- Only the communicator thread owns `auth_node_map` and per-node message history.
- Only the communicator thread mutates the live clock-provenance and airtime-admission state.
- Only the persistence thread touches SQLite or the durable receiver-state file.
- Objects crossing `PersistQueue` must be complete, immutable enough for persistence, and independent from radio buffers.
- State snapshots crossing `StateCommitRequest` must be immutable and carry a monotonically increasing generation.
- Ordinary persistence work must never block the packet-to-ACK path.
- ACK transmission requires an already committed airtime reservation. If none is available, the communicator suppresses the ACK rather than waiting for disk I/O on the critical path.

These rules avoid locks around radio state and prevent SQLite latency from delaying ACK generation.

## Durable receiver state

The receiver has a small state file separate from asynchronous measurement and telemetry persistence. It contains the state needed to remain fail-conservative across process restart, Pi reboot and power loss:

- format version, generation and integrity information;
- last-observed `system_time_quality` and `rtc_health` for diagnosis;
- authoritative clock-provenance fields, including the last verified network-to-RTC synchronization;
- the persisted representation of the rolling receiver TX-airtime ledger;
- unresolved airtime reservations from earlier boots; and
- the active reservation, if one has been durably granted to the current boot.

The exact encoding and atomic-replacement mechanism are implementation details, but a commit is acknowledged only after the new generation is durably installed. A torn, malformed, unsupported or otherwise unverifiable state file is not partially recovered.

At startup, the persistence thread loads and validates the state before the communicator is allowed to transmit. A missing or invalid file means:

```text
airtime history = UNKNOWN_EXHAUSTED
RTC provenance  = UNTRUSTED
```

It must not be interpreted as an empty airtime ledger or as proof that the RTC is correct. An explicit receiver-initialization operation may create the first known-empty ledger for a new installation.

The communicator is the sole logical owner of the live state, while the persistence thread is the sole physical writer. The commit protocol is:

```text
communicator creates immutable state snapshot with generation N
  -> submit high-priority StateCommitRequest
  -> persistence thread durably installs generation N
  -> persistence thread acknowledges generation N
  -> communicator may rely on generation N
```

The persistence thread services state requests before beginning another ordinary batch, once any transaction already in progress has reached a safe boundary. A reported failure before installation leaves the preceding durable generation authoritative. A timeout or otherwise unknown commit outcome must not assume either generation; TX remains disabled until the persistence thread reconciles the installed generation. The communicator may continue receiving and accepting messages during either failure.

## Time model

The receiver uses the Pi system clocks and the battery-backed DS3231 for different purposes:

- Linux monotonic time drives GPIO-event timing, deadlines, latency metrics and the live airtime ledger.
- Linux system UTC supplies receiver reception time and protocol timestamp anchors when its quality is trusted.
- The DS3231 preserves UTC while the Pi is unpowered and seeds the system clock at boot. It is not read for each packet and is not used for elapsed-time measurements.

The DS3231 stores UTC, never local civil time.

### Time quality and RTC health

System-time quality and RTC health are separate axes:

```text
system_time_quality:
    NETWORK_SYNCED
    RTC_HOLDOVER
    UNTRUSTED

rtc_health:
    PRESENT
    MISSING
    INVALID
```

This permits states such as `NETWORK_SYNCED` with a missing RTC. `NETWORK_SYNCED` is established only from the current time-synchronization service. `RTC_HOLDOVER` requires all of the following:

- durable provenance that network-synchronized system UTC was successfully written to the RTC;
- a currently present and valid RTC; and
- an RTC value consistent with the last verified synchronization and the configured uncertainty policy.

The last observed quality and health may be recorded for diagnosis, but they are not restored as authoritative current-boot observations. On every boot the receiver probes the RTC and independently establishes current time quality.

### Boot and synchronization lifecycle

On an offline boot with valid RTC provenance, the DS3231 seeds Linux system UTC and the receiver enters `RTC_HOLDOVER`. With no valid provenance, system UTC remains `UNTRUSTED` even if the RTC provides a provisional value. A missing or invalid RTC is recorded independently in `rtc_health`.

When the time-synchronization service confirms network synchronization, the trust transition is ordered:

1. Mark current Linux system UTC as `NETWORK_SYNCED` in memory.
2. Write UTC to the DS3231.
3. Read back and verify the RTC update.
4. Durably commit the new RTC provenance through `StateCommitRequest`.
5. Only after acknowledgement may a later boot use that update for `RTC_HOLDOVER`.

A reset between the RTC update and the durable provenance commit leaves a good RTC conservatively classified as untrusted. The reverse ordering is forbidden because it could make a later boot trust an RTC that was never updated. The synchronized system time may be copied to the RTC periodically while network synchronization remains confirmed. Clean shutdown has no special clock-persistence role, and an unsynchronized Pi must never overwrite a credible RTC.

### Reception timestamps

The kernel-recorded DIO1 `RX_DONE` edge uses monotonic time. The communicator also captures or obtains a close correlation between monotonic time and system UTC, together with:

- a receiver boot identifier;
- `system_time_quality`;
- `rtc_health`; and
- the current clock-uncertainty estimate.

Live monotonic timestamps must not be persisted and reused as an absolute time base after reboot. NTP adjustment of system UTC must not advance or expire monotonic deadlines or airtime buckets.

When UTC is untrusted, the receiver still records the frame, boot identifier and monotonic reception time. Canonical UTC and the logical reading timestamp remain absent until a trustworthy same-boot correlation or a protocol-permitted timestamp anchor becomes available.

### Logical reading timestamps

The persistence thread owns logical reading-timestamp reconstruction because it exclusively accesses SQLite history. It applies the direct-anchor and extrapolation rules in the protocol, including:

- using the earliest authenticated current-reading reception as a direct anchor;
- preserving reception time at `RX_DONE`, not database-write time;
- crossing between consecutive samples only when the required deep-sleep, previous-cycle-metrics and identity-lifetime conditions hold;
- storing `timestamp_source` and `anchor_sample_id`; and
- never replacing an estimated timestamp after it has been durably assigned.

Reception records and measurements may therefore be committed before their canonical UTC or logical reading timestamp is known.

## Radio access

The pilot receiver runs entirely in userspace.

Use:

- `libgpiod` for DIO1 edge events and GPIO reads;
- `spidev` for SX1262 SPI commands;
- bounded polling of BUSY around SPI commands;
- SX1262 single-receive mode.

The communicator requests DIO1 as an input with rising-edge detection and blocks waiting for events.

BUSY may initially be polled rather than handled through a separate edge-event mechanism. Every BUSY wait must have a timeout.

## SX1262 state model

The minimum state model is:

```text
INITIALIZING
RX_SINGLE
RX_PACKET_PENDING
AUTHENTICATING
TX_PREPARING
TX_ACTIVE
RECOVERING
STOPPED
```

The exact representation is implementation-specific, but unexpected state/event combinations must produce structured diagnostics.

The normal path is:

```text
RX_SINGLE
  -> RxDone / DIO1
  -> read IRQ status
  -> read packet into Pi-owned RAM
  -> clear RX IRQ
  -> authenticate and validate
  -> decide duplicate/new/rejected/retry-later
  -> prepare ACK
  -> require committed airtime reservation
  -> SetTx or suppress ACK
  -> TxDone / DIO1 when TX starts
  -> clear TxDone
  -> SetRx
  -> RX_SINGLE
```

The radio should be returned to RX before non-critical housekeeping such as constructing detailed telemetry objects.

## Communicator thread

### Responsibilities

The communicator thread owns:

- `libgpiod` interaction;
- `spidev` interaction;
- SX1262 initialization and state transitions;
- IRQ inspection and clearing;
- BUSY polling;
- packet copying from SX1262 memory into Pi RAM;
- authentication and decoding;
- protocol-level validation;
- duplicate detection;
- per-node in-memory acceptance state;
- ACK selection, construction, and transmission;
- live time-quality, RTC-health and clock-correlation state;
- live rolling-airtime buckets, unresolved reservations and the active reservation;
- immutable receiver-state snapshot creation and generation tracking;
- timing measurements;
- structured diagnostic creation;
- enqueueing persistence entities.

It must not:

- write to SQLite;
- perform filesystem I/O;
- perform network requests;
- perform unbounded logging;
- block on ordinary `PersistQueue` persistence.

It may wait for acknowledgement of a high-priority `StateCommitRequest` outside the packet-to-ACK critical path. Reservations should be renewed proactively. If a packet requires an ACK while no committed allowance is usable, the communicator suppresses that ACK rather than waiting for a state commit.

### Node map

At startup, the communicator loads a static hardcoded node configuration into `auth_node_map`.

Conceptually:

```text
auth_node_map[node_id] -> NodeState
```

Each `NodeState` contains the material needed to authenticate the node and an in-memory recent-message map.

Repository code determines the exact key material and lookup rules.

### Recent-message map

Each node keeps approximately one day of accepted messages.

At the current sampling rate:

```text
24 hours * 4 samples/hour = 96 messages per node
```

Conceptually:

```text
messages_map[message_id] -> AcceptedMessage
```

An `AcceptedMessage` contains at least:

- the authenticated plaintext payload, or a representation sufficient for exact equality;
- the ACK previously selected for that message;
- acceptance metadata needed by the current protocol.

The current protocol identifies a message using `sample_id || domain`; repository definitions remain authoritative.

The map is an **acceptance cache**, not merely a packet cache.

### Acceptance invariant

A new message may be marked accepted only after its application entity has been successfully admitted to `PersistQueue`.

Required ordering:

```text
validate message
  -> confirm queue admission is possible
  -> enqueue measurement entity
  -> mark message accepted and cache ACK
  -> transmit ACK
```

This guarantees that an ACK transmission failure does not cause the same message to be enqueued repeatedly.

If TX fails after acceptance, a later identical retry reuses the cached ACK and must not enqueue the measurement again.

### Duplicate behavior

For the same authenticated `node_id` and `message_id`:

- same payload:
  - treat as a legitimate duplicate;
  - do not enqueue the measurement again;
  - resend the cached ACK.

- different payload:
  - treat as a protocol invariant violation;
  - create a high-priority diagnostic;
  - reject according to the repository protocol;
  - do not replace the accepted message silently.

### Retry-later behavior

`ACK_RETRY_LATER_DOWNLINK` means that the receiver has **not accepted ownership of the message**.

A message that receives `RETRY_LATER` must not be inserted into the normal accepted-message cache.

When the node retries later, the message must go through queue admission again.

Caching `RETRY_LATER` as a final ACK would risk rejecting the message forever after the queue recovers.

### ACK semantics in the pilot

A successful ACK means:

> The receiver authenticated, validated, and accepted the message into its in-memory processing pipeline.

It does **not** mean:

> The message has already been durably committed to SQLite.

Therefore, an acknowledged measurement can still be lost if the Pi loses power before persistence completes.

This weaker guarantee is accepted for the pilot and must be documented in protocol-facing code and tests.

A later durable-ACK design would require persistence or journaling before ACK transmission.

### Receiver TX-airtime budget

The communicator is the sole logical owner of receiver TX-airtime admission. Under the pilot protocol it enforces the configured charged-airtime limit over every continuous one-hour observation period, not over fixed clock hours. Every receiver transmission uses the protocol's modeled airtime plus its conservative charge.

The live ledger aggregates known charged transmissions into fixed-duration time buckets. The initial bucket width is one minute. The implementation chooses an array capacity sufficient to retain both partial boundary buckets and the complete rolling window; this document does not prescribe the exact number of entries.

The ring contains:

- a monotonic beginning timestamp for the oldest represented bucket;
- the physical array index corresponding to that bucket;
- charged airtime in integer microseconds for each bucket; and
- a cached `total_used` maintained from the buckets.

`total_used` is derived state: it is recomputed and validated when a ledger is loaded, then updated incrementally on insertion, expiration and definite reclamation. The communicator does not sum the complete ring on the receive-to-ACK path.

A bucket is discarded only after its complete interval lies outside the rolling window:

```text
while oldest_bucket_end <= now_monotonic - rolling_window:
    total_used -= oldest_bucket_charge
    clear oldest bucket
    advance starting index
    advance oldest bucket timestamp by one bucket width
```

An implementation may bulk-reset a fully expired ring after a long idle interval. It must bounds-check every computed logical and physical index and fail closed if the ring cannot represent the required interval.

Runtime buckets use monotonic time. A durable snapshot stores a conservative UTC representation or expiration for recovery; a monotonic timestamp from an earlier boot is never treated as comparable with the current boot. Restored buckets are aged only from `NETWORK_SYNCED` UTC or valid `RTC_HOLDOVER`. Otherwise the history remains `UNKNOWN_EXHAUSTED` until a complete verified interval has elapsed.

#### Durable airtime reservations

Known bucket charges alone cannot cover transmissions lost between state commits. Before the communicator is allowed to spend a tranche of airtime, the persistence thread durably commits an active reservation containing at least:

```text
reservation identity and owner boot
reserved charged airtime, no greater than Y
spend deadline
conservative expiration time
```

`X` is the maximum lifetime during which the reservation may be spent, and `Y` is the maximum charged airtime covered by the reservation. Their initial values are implementation and deployment parameters. The current boot enforces the spend deadline with monotonic time. Its durable form carries a conservative UTC deadline and expires no earlier than that deadline plus the continuous one-hour window, bucket-rounding allowance and clock-uncertainty guard.

Admission counts all of the following against the receiver budget:

```text
known unexpired bucket charges
+ unresolved reservations from earlier boots
+ complete active reservation for this boot
```

The active reservation is counted instead of, not in addition to, its tentative per-transmission charges. The communicator records actual charges in RAM and deducts them from the active spendable allowance so they can later replace the reservation with exact bucket totals.

Before the first use of a reservation, its durable generation must be acknowledged. Transmission using that reservation then follows this ordering:

```text
prune fully expired known buckets and reservations
  -> require active committed allowance for the complete ACK charge
  -> tentatively consume that allowance
  -> issue SetTx
  -> retain the charge if SetTx started or its effect is uncertain
  -> reclaim the tentative charge only after a definite pre-SetTx failure
```

When the active allowance is nearly exhausted or its spend deadline is reached, the communicator freezes further spending from it and requests one atomic state transition:

1. Replace the active reservation with the exact known bucket charges incurred under it.
2. Reclaim the unused portion.
3. Retain every uncertain transmission as charged.
4. Create the next active reservation when the remaining budget permits it.
5. Durably commit and acknowledge the new state before the new reservation is used.

No transmission may use the frozen reservation while its settlement outcome is pending. If persistence reports a definite pre-installation failure, the preceding full reservation remains authoritative and the communicator may resume its remaining in-memory allowance before the original deadline. If the outcome is unknown, it suppresses TX until the installed generation is reconciled. After the reservation is exhausted or reaches its deadline, TX remains suppressed until a new durable state generation succeeds.

After a restart, an active reservation owned by an earlier boot becomes unresolved and fully charged. It is never spendable again and remains charged until its conservative expiration. The new boot may create a new active reservation only after a durable state commit and only if known buckets plus all old and new reservations fit the budget. Therefore repeated crashes accumulate conservative reservations rather than losing possible transmissions:

```text
crash before reservation commit -> no TX was enabled under it
crash after reservation commit  -> its complete amount remains charged
```

When no committed reservation fits, an otherwise valid message is still authenticated, validated and admitted to `PersistQueue`; its ACK is suppressed. Lack of receiver TX budget never reverses message acceptance.

### Invalid and rejected packets

The protocol implementation in the repository determines the exact response rules.

At the architectural level:

- packets that cannot be authenticated must not be trusted;
- unauthenticated fields are untrusted input;
- authenticated but unsupported or malformed packets may receive the appropriate authenticated rejection ACK;
- failures before node authentication may require silent discard;
- after any non-TX outcome, the communicator returns the radio to `RX_SINGLE`.

## Critical receive-to-ACK flow

```text
RX_SINGLE
  |
  | packet completes
  v
RxDone set, DIO1 rises, SX1262 enters standby
  |
  v
T0 = kernel-recorded DIO1 edge timestamp
T1 = Python handler begins
  |
  v
GetIrqStatus
ReadBuffer into Pi-owned RAM
T2 = packet copy complete
Clear RX-related IRQs
  |
  v
Authenticate and decode
T3 = authentication complete
  |
  +--> unauthenticated/invalid
  |       -> diagnostic/statistics as appropriate
  |       -> SetRx
  |
  +--> authenticated duplicate, same payload
  |       -> reuse cached ACK
  |
  +--> authenticated duplicate, different payload
  |       -> invariant-violation diagnostic
  |       -> protocol rejection
  |
  +--> authenticated new message
          -> validate
          -> check queue admission
          -> if unavailable: RETRY_LATER, not accepted
          -> otherwise:
               construct measurement entity
               enqueue measurement entity
               mark accepted
               cache selected ACK
  |
  v
Construct or reuse authenticated ACK
Check committed airtime reservation
  |
  +--> unavailable/exhausted
  |       -> suppress ACK
  |       -> SetRx
  |       -> T6 = SetRx issued
  |
  `--> available
          -> tentatively consume charged allowance
          -> SetTx
          -> T4 = SetTx issued or attempted
          -> TxDone / DIO1
          -> T5 = TxDone edge
          -> Clear TxDone
          -> SetRx
          -> T6 = SetRx issued
  |
  v
Construct MessageStats
Enqueue MessageStats best-effort
Return to DIO1 wait
```

Every failure path must either:

- intentionally transition the radio to a known next state; or
- enter a bounded recovery path and create a diagnostic.

## PersistQueue

`PersistQueue` is a bounded in-memory handoff between the communicator and persistence threads.

Initial target capacity:

```text
approximately 50 MB
```

The exact accounting mechanism is implementation-specific, but capacity must be based on an estimated or serialized byte size, not only object count.

The queue may contain:

- measurement entities;
- `MessageStats`;
- structured diagnostics;
- persistence metrics, if represented as entities.

### Priority policy

Measurements are more important than telemetry.

Use conceptual priority classes:

1. measurement/application entities;
2. important diagnostics;
3. message statistics and performance telemetry.

When pressure increases:

- telemetry may be sampled or dropped first;
- diagnostics may be rate-limited;
- measurement capacity should be preserved;
- if a measurement cannot be admitted, reply with `ACK_RETRY_LATER_DOWNLINK`.

Failure to enqueue telemetry must not cause the message itself to be rejected after it has already been accepted.

### Non-recursive diagnostics

Failure of the diagnostic path must not recursively produce more diagnostics through the same full queue.

Fallback policy:

```text
normal structured diagnostic -> PersistQueue

if that fails:
    increment in-memory counters and/or write a bounded message to stderr
```

Do not generate a diagnostic about failure to enqueue a diagnostic.

## Persistence thread

### Responsibilities

The persistence thread is the exclusive owner of SQLite and the durable receiver-state file.

It:

- loads and validates durable receiver state during startup;
- services high-priority `StateCommitRequest` operations and acknowledges only durable generations;
- wakes approximately every five seconds, or when a configurable queue threshold is reached;
- checks whether the queue contains data;
- takes a logical batch of at most approximately 1 MB;
- writes the batch using one SQLite transaction;
- removes entries from the queue only after a successful commit;
- records persistence timing and batch metadata;
- repeats while work remains.

The five-second interval and one-megabyte batch limit are pilot defaults and should remain configurable.

### Receiver-state ownership

`StateCommitRequest` is separate from `PersistQueue` and has stronger semantics. A state request is never sampled, dropped, merged silently or acknowledged on enqueue. The persistence thread either durably installs the complete requested generation or reports failure while leaving the preceding generation authoritative.

The persistence thread does not independently edit clock or airtime policy. It validates the snapshot envelope and ordering information needed for safe replacement, writes the communicator-owned snapshot and returns the committed generation. This preserves a single policy owner without allowing two threads to write disk.

An ordinary SQLite batch and a receiver-state replacement need not share one transaction because they protect different guarantees. Their I/O ordering must nevertheless ensure that a state request cannot be starved indefinitely by telemetry batches.

### Batch ownership

Queue entries must remain logically owned by `PersistQueue` until commit succeeds.

Correct behavior:

```text
copy/reference batch
  -> begin SQLite transaction
  -> write all batch entries
  -> commit
  -> only then acknowledge/remove queue entries
```

If the transaction fails, the entries remain pending.

### Poisoned entities

One invalid or unserializable entity must not block all later persistence forever.

The persistence thread needs a bounded failure-isolation strategy:

```text
batch fails
  -> retry or narrow the batch
  -> identify the offending entity when possible
  -> quarantine or drop it according to pilot policy
  -> emit a diagnostic through a non-recursive path
  -> continue with later entities
```

The exact quarantine mechanism may be simple in the pilot, but infinite retry of the same failing batch is not acceptable.

## Telemetry

### MessageStats

For every handled message, collect when available:

- `T0`: kernel-recorded DIO1 edge timestamp;
- `T1`: Python handler begins;
- `T2`: packet copied from SX1262 into Pi RAM;
- `T3`: AES-CCM authentication completes;
- `T4`: `SetTx` command issued;
- `T5`: TxDone edge timestamp;
- `T6`: `SetRx` command issued.

Derived intervals should be computed during analysis or persistence rather than on the radio-critical path when practical:

```text
T1 - T0 : kernel/userspace scheduling delay
T2 - T1 : IRQ and SPI receive handling
T3 - T2 : authentication and decode time
T4 - T3 : decision and ACK preparation
T5 - T4 : TX transition and LoRa airtime
T6 - T5 : TX completion handling and RX restart
```

A missing timestamp must be representable explicitly. Error paths will not always produce all seven timestamps.

`T0` and `T5` are monotonic kernel-event timestamps. When available, `MessageStats` and the receiver packet record also carry the receiver boot identifier, canonical `RX_DONE` UTC, system-time quality, RTC health and clock uncertainty. UTC fields remain explicitly absent while time is untrusted.

### BUSY metrics

Collect at least:

- total BUSY wait time per handled radio event;
- maximum single BUSY wait;
- number of BUSY waits;
- BUSY timeout count;
- SX1262 command associated with a timeout, when known.

Do not log every tight polling iteration as a separate entity.

### Persistence metrics

For each persistence batch, record:

- batch start time;
- commit completion time;
- success or failure;
- number of entities;
- approximate bytes;
- count by entity type;
- queue depth/bytes before the batch;
- queue depth/bytes after the batch;
- retry or isolation activity.

Also record queue occupancy when admitting a new measurement. This is useful for deciding whether persistence is falling behind.

For each receiver-state commit, record or count when practical:

- requested and committed generation;
- request, write and acknowledgement times;
- success or failure;
- serialized byte size;
- whether the operation opened, settled or recovered an airtime reservation; and
- time-quality and RTC-health transitions, without treating telemetry as authoritative state.

## Diagnostics

Diagnostics should be structured and include enough context to reconstruct failures.

Useful fields include:

- timestamp;
- subsystem;
- current receiver state;
- event or command being handled;
- IRQ status;
- node ID and message ID only when safely available;
- exception/error category;
- bounded detail text;
- queue occupancy;
- relevant timing data;
- system-time quality and RTC health;
- receiver-state generation and airtime-reservation identity when relevant;
- recovery action selected.

Avoid unbounded payload dumps. Sensitive material, keys, nonces, and plaintext should follow repository logging rules.

Unexpected conditions worth diagnosing include:

- impossible state/event combinations;
- DIO1 edge with no expected IRQ;
- BUSY timeout;
- SPI failure;
- malformed radio response;
- RxDone plus radio error flags;
- authenticated duplicate ID with different plaintext;
- queue admission failure;
- missing, malformed or unsupported receiver-state file;
- state-generation mismatch or durable state-commit failure;
- RTC read, validation, write or read-back failure;
- loss or restoration of trusted time;
- exhausted, expired or unavailable airtime reservation;
- TX timeout or missing TxDone;
- persistence batch failure;
- dropped telemetry due to backpressure.

## Error recovery

Pilot recovery should be bounded and explicit.

Examples:

- RX packet rejected:
  - clear relevant IRQs;
  - return to `RX_SINGLE`.

- BUSY timeout or SPI failure:
  - create diagnostic;
  - enter `RECOVERING`;
  - perform only the minimal recovery currently implemented;
  - never continue under an assumed radio state.

- TX failure:
  - retain accepted-message state;
  - retain the airtime charge when `SetTx` started or its effect is uncertain;
  - reclaim tentative allowance only after a definite pre-`SetTx` failure;
  - attempt to restore RX;
  - allow a node retry to receive the cached ACK later.

- Receiver-state load or commit failure:
  - keep the preceding durable generation authoritative;
  - classify missing or invalid history as exhausted and RTC provenance as untrusted;
  - continue RX and application-queue admission;
  - suppress TX until a valid state and committed reservation are available.

- RTC missing or invalid:
  - update current `rtc_health`;
  - retain network-synchronized system time when available;
  - do not use RTC holdover or overwrite RTC provenance;
  - continue RX while timestamp and TX policies follow current system-time quality.

Full automatic radio reinitialization is deferred, but failures must not silently leave the state machine believing the radio is in RX when it is not.

## Concurrency model

The initial implementation should use one communicator thread and one persistence thread.

No additional worker is required unless measurements show a real need.

Python's GIL is not a practical concern for this workload because:

- the communicator mostly waits for GPIO events;
- persistence mostly waits or performs SQLite I/O;
- the packet-to-ACK CPU work is small;
- thread ownership is more important than parallel CPU execution.

Shared state should be minimized to:

- `PersistQueue`;
- the high-priority `StateCommitRequest` handoff and completion acknowledgement;
- shutdown signaling;
- read-only publication of current time quality for observation where required;
- optional bounded health counters.

The persistence thread must not mutate communicator-owned node, message, clock-policy or airtime-policy state. Its receiver-state acknowledgement reports only whether the exact requested generation became durable.

## Core invariants

- Only the communicator mutates live protocol, time-policy and airtime-policy state.
- Only the persistence thread performs disk I/O.
- `PersistQueue` admission establishes protocol acceptance but not SQLite durability.
- A receiver-state generation is usable only after durable acknowledgement or explicit startup reconciliation.
- Missing, corrupt or unsupported state never becomes an empty airtime ledger or trusted RTC provenance.
- Current `system_time_quality` and `rtc_health` are observed again on every boot; persisted observations are diagnostic only.
- Monotonic time controls live deadlines, event intervals and airtime aging. Wall-clock adjustment never expires live airtime early.
- Known bucket charges, unresolved reservations and the complete active reservation must fit the configured continuous-window budget before a new reservation is committed.
- No TX occurs without an acknowledged active reservation owned by the current boot.
- A reservation from an earlier boot is fully charged, unspendable and retained until conservative expiration.
- A started or uncertain `SetTx` consumes allowance. Only a definite pre-`SetTx` failure permits reclamation.
- A pending or unknown reservation-settlement outcome suppresses TX.
- Airtime suppression never reverses application-message acceptance.
- An untrusted receiver UTC never becomes an immutable direct timestamp anchor.

## Deferred work

The following are explicitly deferred from the first pilot:

- durable duplicate/replay state across Pi reboot;
- durable queue or write-ahead journal before ACK;
- systemd service configuration;
- automatic full radio reinitialization;
- watchdog and health-management subsystem;
- sophisticated node scheduling;
- multi-receiver coordination;
- a kernel SX1262 driver;
- optimization from Python to C;
- complex BUSY edge handling;
- durable configuration management.

Known consequences:

- a Pi reboot loses `messages_map`;
- a Pi reboot loses unpersisted queue contents;
- a successful ACK does not guarantee durable storage;
- manual restart or deployment scripts may initially be required;
- radio failures may require process or Pi restart until recovery is implemented.

## Repository checks and missing details

The agent modifying this document or implementing the receiver must verify the following against the repository.

### Protocol compliance

- Exact definition of node identity and key lookup.
- Exact packet framing and clear/authenticated fields.
- AES-CCM nonce construction and uniqueness requirements.
- Replay and counter rules.
- Exact definition of `message_id`, currently described as `sample_id || domain`.
- Which invalid packets require silence versus an authenticated rejection ACK.
- Exact ACK codes and retry semantics.
- Whether ACK bytes can be cached and retransmitted exactly, including nonce/counter constraints.
- Duplicate handling required by the protocol.
- Whether ACK generation changes any per-node protocol state.

### SX1262 behavior

- Correct single-RX command and timeout values.
- Required IRQ mapping to DIO1.
- Which IRQ bits must be cleared on every path.
- Handling of CRC, header, timeout, and unexpected IRQ combinations.
- Required BUSY checks before and after commands.
- RX buffer status and payload extraction.
- TX buffer setup and TxDone handling.
- Recovery behavior after command or TX failure.
- RF switch, reset, and module-specific GPIO requirements.

### Persistence model

- SQLite schema for measurements, diagnostics, and statistics.
- Idempotency or uniqueness constraints for `(node_id, message_id)`.
- Entity serialization and size accounting.
- Transaction boundaries.
- Poison-entity quarantine policy.
- Whether telemetry should be normalized or stored as structured blobs.
- Database corruption and disk-full behavior.

### Receiver state and time

- Durable state-file path, encoding, integrity check and atomic replacement primitive.
- Explicit initialization procedure for the first known-empty airtime ledger.
- Generation-reconciliation behavior after an unknown commit outcome.
- Concrete `StateCommitRequest` handoff, priority, timeout and acknowledgement mechanism.
- Initial values of `X` and `Y` and the proactive reservation-renewal threshold.
- Airtime-bucket capacity, UTC serialization and conservative bucket/clock guards.
- Per-band ledger representation if later RF configurations use more than one regulated sub-band.
- Time-synchronization service integration and the criterion for `NETWORK_SYNCED`.
- DS3231 discovery, health validation, UTC write/read-back and uncertainty policy.
- Monotonic-to-UTC correlation representation and same-boot timestamp completion.
- SQLite representation of nullable reception/logical timestamps, timestamp provenance and immutable completion.

### Queue and memory policy

- Concrete 50 MB accounting strategy.
- Queue priority implementation.
- Telemetry sampling/rate limiting under pressure.
- Exact admission point at which a message becomes accepted.
- Behavior when measurement enqueue succeeds but telemetry enqueue fails.
- Shutdown behavior with pending entries.

### Testing

At minimum, add tests or simulations for:

- new valid message;
- identical retry after successful ACK;
- identical retry after failed ACK transmission;
- same ID with different authenticated payload;
- unauthenticated packet;
- authenticated malformed packet;
- queue full and later recovery;
- telemetry dropped while measurements continue;
- BUSY timeout;
- unexpected IRQ combination;
- TxDone missing or delayed;
- SQLite transaction failure;
- one poisoned entity followed by valid entities;
- Pi process restart and documented state loss;
- timestamp fields missing on partial/error paths;
- first startup with an explicitly initialized empty ledger;
- missing, torn, malformed and unsupported receiver-state files fail closed;
- durable state generation success, reported failure and unknown completion;
- state requests take priority without starving ordinary persistence permanently;
- current `NETWORK_SYNCED` time with a missing RTC;
- valid offline `RTC_HOLDOVER` and rejected stale, invalid or unproven RTC values;
- network-to-RTC write, read-back and durable-provenance crash boundaries;
- NTP forward and backward corrections do not expire monotonic airtime early;
- rolling-window bucket boundaries, long idle intervals and cached-total reconstruction;
- active reservation opening, spending, exact settlement and unused reclamation;
- definite pre-`SetTx` failure versus started or uncertain `SetTx` charging;
- crash before and after reservation commit;
- repeated crashes accumulate unresolved reservations;
- expired old reservations and inability to open a new reservation;
- persistence failure or unknown settlement outcome suppresses TX without undoing acceptance;
- untrusted UTC delays canonical and logical timestamps;
- direct and extrapolated timestamp reconstruction, chain breaks and immutability.

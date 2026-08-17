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
   - is the only thread allowed to access SQLite, the durable receiver-state file, or the receiver configuration file;
   - drains a bounded in-memory queue in batches;
   - records measurements, message profiling, receiver health, and diagnostics;
   - loads and validates receiver configuration during startup;
   - services high-priority synchronous receiver-state commit requests.

The threads communicate through three paths with different guarantees:

- bounded `PersistQueue` admission is asynchronous and does not imply disk durability;
- `StateCommitRequest` is a high-priority synchronous request whose completion acknowledges a durable receiver-state generation.
- `ConfigurationLoadRequest` is a synchronous startup-only request that returns an immutable validated configuration snapshot.

```text
SX1262
  |
  | DIO1 / BUSY / SPI
  v
Communicator thread
  |
  |-- measurements / profiling / health / diagnostics --> PersistQueue ---|
  |                                                                  |
  |<-- configuration snapshot <-- ConfigurationLoadRequest -----------|
  |                                                                  |
  `-- receiver-state snapshot --> StateCommitRequest -----------------|
                                                                     v
Persistence thread
  |-- SQLite
  |-- durable receiver-state file
  `-- receiver configuration file
```

## Core ownership rules

These ownership rules should remain strict:

- Only the communicator thread touches the SX1262.
- Only the communicator thread owns `auth_node_map`, per-node transport-message history and optional reading-duplicate history.
- Only the communicator thread mutates the live clock-provenance and airtime-admission state.
- Only the persistence thread touches SQLite, the durable receiver-state file, or the receiver configuration file.
- Objects crossing `PersistQueue` must be complete, immutable enough for persistence, and independent from radio buffers.
- State snapshots crossing `StateCommitRequest` must be immutable and carry a monotonically increasing generation.
- Ordinary persistence work must never block the packet-to-ACK path.
- ACK transmission requires an already committed airtime reservation. If none is available, the communicator suppresses the ACK rather than waiting for disk I/O on the critical path.

These rules avoid locks around radio state and prevent SQLite latency from delaying ACK generation.

## Receiver configuration

Receiver configuration is operator-controlled input and is separate from the receiver-owned durable state. The development default is `receiver/receiver-group.json`; deployments may configure another path so that production does not depend on a repository working tree. The file uses the strict `receiver-group.json` format defined by the protocol provisioning tools.

The persistence thread is the sole disk owner and loads the configuration before the communicator enters RX. It must reject a configuration that:

- is missing, malformed or has an unsupported format version;
- is not a regular, non-symlink file;
- is accessible by group or other users;
- is not owned by the configured receiver service user;
- is reached through a parent directory writable by an untrusted user; or
- contains invalid group keys, node identifiers or overlapping active and retired node sets.

After validation, the persistence thread returns an immutable configuration snapshot through a synchronous startup request. The communicator uses that snapshot to construct its in-memory authentication map and never reads the file itself. Secret keys and derived key material must not be included in logs, diagnostics or profiling records.

Configuration failure prevents the receiver from entering normal radio operation because it cannot authenticate nodes safely. This differs from missing or corrupt durable receiver state, which permits RX and application admission while conservatively suppressing TX as described below.

The pilot loads configuration once at startup. Applying provisioning, revocation or key changes requires a receiver restart; hot reload and its associated in-flight-message semantics are deferred.

## Receiver instance and Linux boot identity

The receiver distinguishes the lifetime of the Python process from the lifetime of the Pi operating-system boot:

- `receiver_instance_id` is a random identifier created once whenever the receiver process starts. A clean service restart and a crash followed by automatic restart both create a new value.
- `linux_boot_id` is the kernel boot identifier. It remains stable across receiver-process restarts and changes when the Pi reboots or power-cycles.

The identifiers distinguish the important cases:

| Event | `receiver_instance_id` | `linux_boot_id` |
|---|---|---|
| Receiver process restart | changes | unchanged |
| Receiver crash and service restart | changes | unchanged |
| Pi reboot or power cycle | changes | changes |

The process creates `receiver_instance_id` before normal radio operation and provides it read-only to both threads. The persistence thread reads `linux_boot_id` and adds it to persisted records. Per-instance occurrence sequences, health sequences, radio-recovery counters and other process-local counters reset when `receiver_instance_id` changes.

The protocol packet-log schema currently calls the process-lifetime field `receiver_boot_id`. Until that protocol field is renamed, it maps exactly to `receiver_instance_id` and must never contain `linux_boot_id`.

Linux monotonic timestamps are meaningful only together with `linux_boot_id`. Receiver-local sequences and lifecycle observations additionally require `receiver_instance_id` so a process restart within one Linux boot remains visible.

The persistence thread records a receiver-started lifecycle marker as soon as startup information can be committed and attempts to record a receiver-stopped-cleanly marker during controlled shutdown. A new instance without a durable clean-stop marker for the preceding instance suggests a crash, kill, power loss or lost final write; it is not proof of any one cause. Lifecycle telemetry does not alter clock provenance, airtime state or clean-shutdown policy.

## Durable receiver state

The receiver has a small state file separate from asynchronous measurement and telemetry persistence. It contains the state needed to remain fail-conservative across process restart, Pi reboot and power loss:

- format version, generation and integrity information;
- last-observed `system_time_quality` and `rtc_health` for diagnosis;
- authoritative clock-provenance fields, including the last verified network-to-RTC synchronization;
- the persisted representation of the rolling receiver TX-airtime ledger;
- unresolved airtime reservations from earlier receiver instances; and
- the active reservation, if one has been durably granted to the current receiver instance.

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
- The DS3231 preserves UTC while the Pi is unpowered and seeds the system clock at Pi boot. It is not read for each packet and is not used for elapsed-time measurements.

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

The last observed quality and health may be recorded for diagnosis, but they are not restored as authoritative current-instance observations. On every receiver startup the receiver probes the RTC and independently establishes current time quality.

### Boot and synchronization lifecycle

On an offline Pi boot with valid RTC provenance, the DS3231 seeds Linux system UTC and the receiver enters `RTC_HOLDOVER`. With no valid provenance, system UTC remains `UNTRUSTED` even if the RTC provides a provisional value. A missing or invalid RTC is recorded independently in `rtc_health`.

When the time-synchronization service confirms network synchronization, the trust transition is ordered:

1. Mark current Linux system UTC as `NETWORK_SYNCED` in memory.
2. Write UTC to the DS3231.
3. Read back and verify the RTC update.
4. Durably commit the new RTC provenance through `StateCommitRequest`.
5. Only after acknowledgement may a later Pi boot use that update for `RTC_HOLDOVER`.

A reset between the RTC update and the durable provenance commit leaves a good RTC conservatively classified as untrusted. The reverse ordering is forbidden because it could make a later Pi boot trust an RTC that was never updated. The synchronized system time may be copied to the RTC periodically while network synchronization remains confirmed. Clean shutdown has no special clock-persistence role, and an unsynchronized Pi must never overwrite a credible RTC.

### Reception timestamps

The kernel-recorded DIO1 `RX_DONE` edge uses monotonic time. The communicator also captures or obtains a close correlation between monotonic time and system UTC, together with:

- `receiver_instance_id`;
- `linux_boot_id`;
- `system_time_quality`;
- `rtc_health`; and
- the current clock-uncertainty estimate.

Live monotonic timestamps must not be persisted and reused as an absolute time base after receiver restart or Pi reboot. NTP adjustment of system UTC must not advance or expire monotonic deadlines or airtime buckets.

When UTC is untrusted, the receiver still records the frame, both identity fields and monotonic reception time. Canonical UTC and the logical reading timestamp remain absent until a trustworthy same-Linux-boot correlation or a protocol-permitted timestamp anchor becomes available.

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

The state model describes the communicator's best-known operational state of the SX1262, not necessarily the chip's physical mode. After a command with an uncertain outcome, software may be unable to know the physical mode. `RECOVERING` represents that uncertainty until the communicator confirms a known-good receive state or enters a terminal state.

The minimum state model is:

```text
INITIALIZING
RX_SINGLE
RX_EVENT_PENDING
TX_ACTIVE
RECOVERING
SHUTDOWN
INITIALIZATION_FAILED
RECOVERY_EXHAUSTED
HARDWARE_MISSING
```

`RX_EVENT_PENDING` is used instead of a packet-specific state because DIO1 may report an RX packet, a header or CRC error, or an unexpected IRQ. Authentication, validation and ACK preparation are application-processing phases while the radio remains in this operational state; they are not SX1262 states.

`RX_SINGLE` means that the communicator has confirmed the required receive configuration, cleared or accounted for preceding IRQs and confirmed `SetRx`. A low DIO1 level alone does not prove that the radio is receiving.

`TX_ACTIVE` begins only after `SetTx` has a confirmed or uncertain outcome. A definite failure before `SetTx` can affect the radio does not enter `TX_ACTIVE`. When the outcome is uncertain, the communicator must treat TX as possibly active and retain its airtime charge.

The four terminal states are distinct enum values rather than `STOPPED` plus a separate reason:

- `SHUTDOWN`: intentional process or receiver shutdown;
- `INITIALIZATION_FAILED`: the hardware was reachable, but bounded startup initialization could not establish the configured receive state;
- `RECOVERY_EXHAUSTED`: bounded runtime recovery could not restore the configured receive state; and
- `HARDWARE_MISSING`: required SPI/GPIO resources or the SX1262 are absent or no longer reachable.

Terminal states have no outgoing transition within the current receiver process. A process restart begins again in `INITIALIZING`.

The state transitions are:

```text
INITIALIZING
  -> full initialization + confirmed SetRx -> RX_SINGLE
  -> required hardware absent or unreachable -> HARDWARE_MISSING
  -> bounded initialization attempts fail -> INITIALIZATION_FAILED

RX_SINGLE
  -> DIO1 edge -> RX_EVENT_PENDING

RX_EVENT_PENDING
  -> event handled without TX + confirmed SetRx -> RX_SINGLE
  -> SetTx confirmed or uncertain -> TX_ACTIVE

TX_ACTIVE
  -> terminal TX IRQ + confirmed SetRx -> RX_SINGLE

RX_SINGLE / RX_EVENT_PENDING / TX_ACTIVE
  -> radio mode or configuration uncertain, or RX re-arm fails -> RECOVERING

RECOVERING
  -> recovery + confirmed SetRx -> RX_SINGLE
  -> required hardware absent or unreachable -> HARDWARE_MISSING
  -> bounded recovery attempts exhausted -> RECOVERY_EXHAUSTED

Any non-terminal state
  -> intentional shutdown -> SHUTDOWN
```

If DIO1 reports a new RX event immediately after recovery confirms `SetRx`, the communicator transitions to `RX_EVENT_PENDING` and handles the event rather than discarding it as stale.

The normal path is:

```text
RX_SINGLE
  -> RxDone / DIO1
  -> RX_EVENT_PENDING
  -> read IRQ status
  -> read packet into Pi-owned RAM
  -> clear RX IRQ
  -> authenticate and validate
  -> decide duplicate/new/rejected/retry-later
  -> prepare ACK
  -> require committed airtime reservation
  -> suppress ACK + confirmed SetRx -> RX_SINGLE
  or
  -> SetTx confirmed or uncertain
  -> TX_ACTIVE
  -> TxDone or radio timeout / DIO1 when TX terminates
  -> clear terminal TX IRQ
  -> confirmed SetRx
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
- authenticated transport-retransmission and application-reading duplicate detection;
- per-node in-memory acceptance state;
- ACK selection, construction, and transmission;
- live time-quality, RTC-health and clock-correlation state;
- live rolling-airtime buckets, unresolved reservations and the active reservation;
- immutable receiver-state snapshot creation and generation tracking;
- timing measurements;
- periodic immutable `ReceiverHealthRequest` creation;
- per-instance health-sequence, time/RTC-transition and exceptional radio-recovery counters;
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

At startup, the communicator receives the immutable configuration snapshot loaded and validated by the persistence thread. It derives the active nodes' authentication material and constructs `auth_node_map` in RAM before entering RX.

Conceptually:

```text
auth_node_map[node_id] -> NodeState
```

Each `NodeState` contains the material needed to authenticate the node, an in-memory recent transport-message map and, when implemented, an application-reading duplicate map.

Repository protocol code determines the exact key derivation and lookup rules. Retired and unknown node identifiers are not inserted into the map and receive no response. The communicator does not access the configuration file after startup.

### Transport identity and CCM nonce

The fixed clear uplink header carries `node_id`, `message_id` and `domain`. These values are untrusted claims until AES-CCM authentication succeeds. The receiver authenticates the exact clear header as AAD and constructs the nonce exactly as defined by the protocol:

```text
node_id || message_id || domain
```

`message_id` is the persistent monotonic transport counter scoped to one node identity and key. It identifies a newly constructed logical LoRa message, not an individual RF attempt. Retransmissions of that logical message reuse the same `message_id`, domain, authenticated bytes and complete frame. Converting a current reading into a newly constructed backlog message allocates a new `message_id` even though the encrypted reading retains its original `sample_id`. The receiver must tolerate skipped IDs, but authenticated reuse under the same node identity is invalid; exhaustion or loss of the node counter requires a new node identity and key.

`sample_id` remains inside the authenticated reading body. It identifies the application reading, supports wake continuity and timestamp reconstruction, and is unavailable until authentication and decoding succeed. It is not part of the CCM nonce and does not identify RF retransmission episodes.

An ACK echoes the authenticated uplink's `message_id`; it does not carry `sample_id`. Its ACK domain selects the downlink nonce domain. Repeating the same ACK outcome reuses its exact authenticated frame, while a different outcome uses the protocol's distinct ACK domain under the same `message_id`.

### Transport and reading maps

Each node keeps a bounded recent transport-message map. The initial retention target is approximately one day, but its capacity must be based on logical transport messages rather than only on application samples. At four readings per hour there are 96 new `sample_id` values per day, while current-to-backlog conversion and later protocol domains can create additional `message_id` values.

Conceptually:

```text
transport_messages_map[message_id] -> AcceptedTransportMessage
```

An `AcceptedTransportMessage` contains at least:

- the exact authenticated uplink frame, or a representation sufficient for exact frame equality;
- its decoded `sample_id` and application contents when it is a reading;
- the ACK previously selected for that transport message; and
- acceptance metadata needed by the current protocol.

The transport map is an **acceptance cache**, not merely a packet cache. It detects retransmission of an already accepted logical message, prevents repeated measurement enqueue after ACK loss and permits reuse of the exact cached ACK.

An optional application-reading map is separate:

```text
readings_map[sample_id] -> AcceptedReading
```

It detects the same application reading arriving in distinct transport messages, such as current and backlog messages with different `message_id` values. Matching contents may be accepted without another measurement enqueue; conflicting contents for the same authenticated `(node_id, sample_id)` are malformed. Without this optional map, the communicator may enqueue the reading again and rely on idempotent persistence keyed by `(node_id, sample_id)`.

### Acceptance invariant

A new transport message whose reading requires persistence may be marked accepted only after its application entity and initial `MessageProfiling` record have been admitted to `PersistQueue` atomically. They have the same persistence priority, and the queue operation must insert both or neither.

Required ordering:

```text
validate message
  -> construct measurement entity, initial MessageProfiling and candidate success ACK
  -> atomically enqueue measurement entity and initial MessageProfiling
     -> failure: enqueue neither and select RETRY_LATER
     -> success: mark message accepted, cache ACK and transmit it when permitted
```

This guarantees that an ACK transmission failure does not cause the same transport message to be enqueued repeatedly.

If TX fails after acceptance, a later exact transport retransmission reuses the cached ACK and must not enqueue the measurement again.

### Duplicate behavior

For the same authenticated `node_id` and `message_id`:

- exact same authenticated frame:
  - treat as a legitimate transport retransmission;
  - do not enqueue the measurement again;
  - admit an initial `MessageProfiling` record for this packet occurrence;
  - resend the cached ACK only after that record is admitted;
  - select `ACK_RETRY_LATER_DOWNLINK` if the profiling record cannot be admitted.

- different authenticated frame bytes or domain:
  - treat as prohibited `message_id` reuse and a malformed protocol invariant violation;
  - create a high-priority diagnostic;
  - reject according to the repository protocol;
  - do not replace the accepted message silently.

After transport classification, two authenticated reading messages with different `message_id` values may still carry the same `sample_id`. If `readings_map` is present, identical application contents are a reading duplicate and conflicting contents are malformed. If it is absent, SQLite measurement persistence must remain idempotent on `(node_id, sample_id)`.

### Retry-later behavior

For a first-seen message, `ACK_RETRY_LATER_DOWNLINK` means that the receiver has **not accepted ownership of the message** because the measurement and its initial profiling record could not both be admitted.

A message that receives `RETRY_LATER` must not be inserted into the normal accepted-message cache.

When the node retries later, the message must go through queue admission again.

Caching `RETRY_LATER` as a final ACK would risk rejecting the message forever after the queue recovers.

An already accepted duplicate may also receive `RETRY_LATER` when the receiver cannot admit the duplicate occurrence's profiling record. This does not reverse ownership of the original measurement; it asks the node to retry so that the occurrence can be handled when persistence capacity is available.

### ACK semantics in the pilot

A successful ACK means:

> The receiver authenticated and validated the message, then admitted both its application entity and packet-occurrence profile into its in-memory processing pipeline.

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

Runtime buckets use monotonic time. A durable snapshot stores a conservative UTC representation or expiration for recovery; a persisted monotonic timestamp from an earlier receiver instance is never used as the new instance's time base, even when both instances ran during the same Linux boot. Restored buckets are aged only from `NETWORK_SYNCED` UTC or valid `RTC_HOLDOVER`. Otherwise the history remains `UNKNOWN_EXHAUSTED` until a complete verified interval has elapsed.

#### Durable airtime reservations

Known bucket charges alone cannot cover transmissions lost between state commits. Before the communicator is allowed to spend a tranche of airtime, the persistence thread durably commits an active reservation containing at least:

```text
reservation identity and owner receiver instance
reserved charged airtime, no greater than Y
spend deadline
conservative expiration time
```

`X` is the maximum lifetime during which the reservation may be spent, and `Y` is the maximum charged airtime covered by the reservation. Their initial values are implementation and deployment parameters. The current receiver instance enforces the spend deadline with monotonic time. Its durable form carries a conservative UTC deadline and expires no earlier than that deadline plus the continuous one-hour window, bucket-rounding allowance and clock-uncertainty guard.

Admission counts all of the following against the receiver budget:

```text
known unexpired bucket charges
+ unresolved reservations from earlier receiver instances
+ complete active reservation for this receiver instance
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

After a restart, an active reservation owned by an earlier receiver instance becomes unresolved and fully charged. It is never spendable again and remains charged until its conservative expiration. The new receiver instance may create a new active reservation only after a durable state commit and only if known buckets plus all old and new reservations fit the budget. Therefore repeated crashes accumulate conservative reservations rather than losing possible transmissions:

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
- authenticated but unsupported or malformed packets may receive the appropriate authenticated rejection ACK only after their occurrence profile is admitted;
- failure to admit a required occurrence profile changes an otherwise eligible authenticated response to `ACK_RETRY_LATER_DOWNLINK`;
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
Parse claimed node_id, message_id and domain
Authenticate exact header and frame with
nonce = node_id || message_id || domain
T3 = authentication complete
Decode sample_id only after authentication
  |
  +--> unauthenticated/unusable header
  |       -> no ACK
  |       -> SetRx, T6 = SetRx issued
  |       -> construct complete MessageProfiling and try to enqueue it
  |
  +--> authenticated unsupported/malformed
  |       -> construct initial MessageProfiling and rejection ACK
  |       -> enqueue profile or select RETRY_LATER
  |
  +--> authenticated transport retransmission, exact same frame
  |       -> construct initial MessageProfiling
  |       -> enqueue it or select RETRY_LATER
  |       -> otherwise reuse cached ACK
  |
  +--> authenticated message_id reuse, different frame/domain
  |       -> malformed invariant-violation diagnostic
  |       -> construct initial MessageProfiling and protocol rejection
  |       -> enqueue profile or select RETRY_LATER
  |
  +--> authenticated new transport message
          -> validate
          -> classify reading by authenticated sample_id when readings_map exists
          -> matching accepted reading: enqueue profile or select RETRY_LATER;
               on success mark transport accepted and cache/select ACCEPTED
          -> conflicting reading: enqueue profile or select RETRY_LATER;
               on success select malformed rejection
          -> reading requires persistence:
               construct measurement entity and initial MessageProfiling
               atomically enqueue both
               if either is unavailable: enqueue neither, RETRY_LATER, not accepted
               otherwise:
                   mark transport message accepted
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
If an admitted initial profile has pending radio-outcome fields,
enqueue its MessageProfiling completion update
Return to DIO1 wait
```

The initial `MessageProfiling` record is admitted before an ACK transmission and contains the exact selected ACK frame, but its TX-dependent fields are initially pending. After the TX or suppression path reaches a terminal outcome and RX recovery has been attempted, the communicator enqueues a small completion update for the same logical record. The initial record and completion update are separate immutable queue commands; the persistence thread inserts and then updates one SQLite row.

If initial profiling admission fails, the detailed packet-occurrence record cannot be retained without reserving queue capacity in advance. The pilot explicitly permits this overload exception, selects `ACK_RETRY_LATER_DOWNLINK` for an authenticated packet that is eligible for a response, and increments `message_profiling_admission_failures`. Packets that cannot be authenticated remain silent. The counter is reported later through the existing non-recursive persistence or diagnostic path so that gaps in the occurrence log are observable.

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
- initial `MessageProfiling` records;
- `MessageProfiling` TX-completion updates;
- `ReceiverHealthRequest` snapshots;
- structured diagnostics;
- persistence metrics, if represented as entities.

### Priority policy

Measurements and their packet-occurrence profiling are application data of equal importance.

Use conceptual priority classes:

1. measurement/application entities and `MessageProfiling` commands;
2. important diagnostics;
3. `ReceiverHealthRequest` and optional persistence telemetry.

When pressure increases:

- receiver-health and optional persistence telemetry may be sampled or rejected at admission first;
- diagnostics may be rate-limited;
- measurement and initial-profile admission must remain atomic;
- if either item cannot be admitted, enqueue neither and select `ACK_RETRY_LATER_DOWNLINK`.

Once admitted, measurement or profiling entries must not be sampled or dropped. Failure to enqueue a TX-completion update leaves the initial row pending and increments `message_profiling_admission_failures`; it never changes an ACK that has already been attempted.

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

The persistence thread is the exclusive owner of SQLite, the durable receiver-state file and the receiver configuration file.

It:

- loads, validates and publishes the immutable receiver configuration snapshot during startup;
- loads and validates durable receiver state during startup;
- services high-priority `StateCommitRequest` operations and acknowledges only durable generations;
- wakes approximately every five seconds, or when a configurable queue threshold is reached;
- checks whether the queue contains data;
- takes a logical batch of at most approximately 1 MB;
- writes the batch using one SQLite transaction;
- removes entries from the queue only after a successful commit;
- records persistence timing and batch metadata;
- inserts initial `MessageProfiling` rows and applies their ordered TX-completion updates;
- enriches `ReceiverHealthRequest` snapshots with persistence-owned and host-owned observations and inserts complete `ReceiverHealth` rows;
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

Initial `MessageProfiling` records and their completion updates have the same priority. Queue and batch processing must preserve their communicator enqueue order so that a completion update cannot overtake its initial insert, including when both appear in the same SQLite transaction.

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

### MessageProfiling

`MessageProfiling` is the single logical packet-occurrence record required by the protocol. It combines the former `MessageStats` timing data with the received frame, authentication and processing decisions, duplicate results, radio metadata, selected ACK and ACK-transmission result. The communicator copies the raw frame and radio metadata into Pi-owned memory before later radio activity can overwrite them.

Every admitted record is identified by:

- `receiver_instance_id`, serialized under the protocol's current `receiver_boot_id` field name; and
- a monotonically increasing per-instance occurrence sequence.

Together these fields identify the SQLite row and associate its later TX-completion update with the correct initial record.

The record carries the protocol-defined packet-occurrence fields and, when available:

- `T0`: kernel-recorded DIO1 edge timestamp;
- `T1`: Python handler begins;
- `T2`: packet copied from SX1262 into Pi RAM;
- `T3`: AES-CCM authentication completes;
- `T4`: `SetTx` command issued or attempted;
- `T5`: TxDone edge timestamp;
- `T6`: `SetRx` command issued.

It also records:

- `linux_boot_id`, which scopes its Linux monotonic timestamps;
- canonical reception UTC, which remains absent when system time is untrusted;
- the monotonic reception timestamp, with `T0` as its source;
- `system_time_quality` and `rtc_health` captured for that occurrence; and
- measurement-queue occupancy and configured capacity immediately before the admission attempt.

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

`T0` and `T5` are monotonic kernel-event timestamps. UTC fields remain explicitly absent while time is untrusted.

For a packet that may receive an ACK, the initial row is enqueued before TX with the selected ACK and exact ACK frame already fixed. `ack_tx_result` and TX-dependent timestamps remain pending. The completion update supplies the terminal TX result and `T4` through `T6` after the radio has been returned to RX or bounded recovery has been attempted. A packet that cannot receive an ACK may instead be returned to RX first and then enqueued once as a complete profile. A pending row from an earlier receiver instance is interpreted as an unknown interrupted outcome rather than as proof that TX failed or succeeded.

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

### ReceiverHealth

`ReceiverHealth` is a periodic application-aware snapshot. It is not sampled per packet and is not part of ACK acceptance. The initial pilot interval is one minute and remains configurable.

The communicator drives the interval because only that thread can take a coherent snapshot of its live state. Once per interval, and once immediately after successful communicator initialization, it creates an immutable `ReceiverHealthRequest` and attempts a nonblocking enqueue to `PersistQueue`. DIO1 and radio-deadline work take priority when they coincide with the health deadline.

The request contains only communicator-owned observations:

- `receiver_instance_id`;
- a monotonically increasing per-instance `health_sequence`;
- `communicator_sampled_at_monotonic_us`;
- current radio state;
- cumulative exceptional radio-recovery attempt, success and failure counts;
- bounded recovery counts by reason when practical;
- current `system_time_quality` and `rtc_health`;
- cumulative time-quality and RTC-health transition counts and their last transition times; and
- cumulative failed `ReceiverHealthRequest` enqueue attempts.

The request sequence and communicator sampling time are the communicator heartbeat: they prove that the communicator event loop reached the periodic health task. No separate high-frequency heartbeat is required. `health_sequence` advances before every enqueue attempt, so a gap in persisted sequence values exposes a missed or failed sample. The cumulative enqueue-failure count distinguishes known queue-pressure loss once a later request succeeds.

Radio recovery means an exceptional transition into `RECOVERING` after conditions such as a BUSY timeout, SPI failure, unexpected IRQ, missing or uncertain TX completion, or failed attempt to restore RX. A confirmed radio TX-timeout IRQ followed by successful RX re-arming, and the normal `SetRx` operation after packet handling or ACK transmission, are not counted as recovery. Recovery counters are cumulative within one `receiver_instance_id`; interval counts are derived during analysis.

When the persistence thread dequeues a request, it does not mutate the queued object. It samples its own and the Pi's current observations and constructs a new complete `ReceiverHealth` row containing the request fields plus:

- `linux_boot_id`;
- `persistence_sampled_at_monotonic_us`;
- CPU load and temperature;
- available memory;
- available bytes on the filesystem containing SQLite;
- SQLite database and WAL sizes;
- NTP offset when available through a bounded local observation;
- the last persistence-batch completion time and result; and
- persistence-owned cumulative failure or recovery counters relevant to the database writer.

Enrichment occurs once. If the SQLite transaction fails, the completed row and both sampling timestamps remain stable for retry; the persistence thread must not resample host fields and silently change the logical health record.

Unavailable host or time-service observations are represented explicitly as absent; they do not prevent the rest of the health row from being persisted. NTP telemetry is observational and must not independently change communicator-owned `system_time_quality` or clock policy.

`communicator_sampled_at_monotonic_us` and `persistence_sampled_at_monotonic_us` intentionally describe different moments. Communicator-owned fields belong to the former; persistence and Pi fields belong to the latter. Their difference measures how long the health request waited before enrichment and prevents a delayed request from appearing to be one simultaneous snapshot.

`ReceiverHealthRequest` has lower queue priority than measurements and `MessageProfiling`. Under pressure it may be delayed or rejected at admission and must never cause a reading to receive `ACK_RETRY_LATER_DOWNLINK`. Once admitted, it remains queue-owned until commit under the normal batch-ownership rule. The health sequence and cumulative enqueue-failure count make admission gaps visible. The existence of a committed health row proves that the persistence thread reached that request; an in-process health record cannot prove liveness while the persistence thread itself is stalled.

## Diagnostics

Diagnostics should be structured and include enough context to reconstruct failures.

Useful fields include:

- timestamp;
- subsystem;
- current receiver state;
- event or command being handled;
- IRQ status;
- claimed node ID and message ID, explicitly marked untrusted until authentication;
- decoded sample ID only after authentication and reading-body decoding;
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
- authenticated `message_id` reuse with different frame bytes or domain;
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

`RECOVERING` is entered when normal processing cannot confirm the radio's mode or configuration, particularly when it cannot prove that the radio has returned to `RX_SINGLE`. An ordinary packet or protocol error is not by itself a recovery event.

Examples:

- RX packet rejected:
  - clear relevant IRQs;
  - issue and confirm `SetRx`;
  - return directly to `RX_SINGLE` if re-arming succeeds;
  - enter `RECOVERING` only if the IRQ cannot be cleared or RX cannot be confirmed.

- BUSY timeout or SPI failure:
  - create diagnostic;
  - enter `RECOVERING`;
  - never continue under an assumed radio state.

- TX failure:
  - retain accepted-message state;
  - retain the airtime charge when `SetTx` started or its effect is uncertain;
  - reclaim tentative allowance only after a definite pre-`SetTx` failure;
  - return directly to `RX_SINGLE` after a known terminal TX IRQ and confirmed RX re-arm;
  - enter `RECOVERING` when the TX outcome or resulting radio mode is uncertain, or RX re-arming fails;
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

Authentication failure, malformed or unsupported protocol data, duplicate detection, persistence admission failure and airtime-based ACK suppression remain normal packet-processing outcomes. RX header or CRC errors and confirmed radio TX-timeout IRQs may also return directly to `RX_SINGLE` when IRQ clearing and `SetRx` both succeed.

### Radio recovery procedure

On entry to `RECOVERING`, the communicator must suppress new transmissions and record the preceding state, failed operation, command-outcome classification, known IRQ and device-error state, and selected recovery action. If a transmission may have started, its airtime charge remains reserved even when the final radio outcome is unknown.

The first bounded level is soft resynchronization:

1. Wait for BUSY to clear, subject to a bounded deadline.
2. Issue and confirm `SetStandby`.
3. Read and record IRQ and device-error information where possible.
4. Clear the relevant IRQs.
5. Restore the required receive packet, IRQ-routing and radio configuration.
6. Issue and confirm `SetRx`.

Successful confirmation of `SetRx` returns the state to `RX_SINGLE`, unless a new DIO1 event requires an immediate transition to `RX_EVENT_PENDING`.

If soft resynchronization fails, perform bounded hard recovery:

1. Toggle the SX1262 reset signal.
2. Repeat the complete initialization sequence, including standby selection, clock and regulator setup, device-error clearing, calibration, RF-switch configuration, buffer bases, packet type, frequency, modulation and packet parameters, PA/TX parameters, sync word, RX gain and fallback mode as required by the configured hardware profile.
3. Clear or account for pending IRQs.
4. Issue and confirm `SetRx`.

The pilot should attempt one soft resynchronization followed by one hardware reset and full reinitialization. These counts may become configuration values, but must remain finite. If required hardware becomes unavailable, enter `HARDWARE_MISSING`. If hard recovery completes without confirming `RX_SINGLE`, enter `RECOVERY_EXHAUSTED`. Recovery must never become an unbounded reset loop.

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
- the startup-only `ConfigurationLoadRequest` handoff and immutable result;
- shutdown signaling;
- read-only publication of current time quality for observation where required.

Receiver-health counters remain communicator-owned and cross the existing `PersistQueue` only inside immutable `ReceiverHealthRequest` snapshots; they do not create another shared mutable state path.

The persistence thread must not mutate communicator-owned node, message, clock-policy or airtime-policy state. Its receiver-state acknowledgement reports only whether the exact requested generation became durable.

## Core invariants

- Only the communicator mutates live protocol, time-policy and airtime-policy state.
- Only the persistence thread performs disk I/O and loads receiver configuration.
- `receiver_instance_id` changes on every receiver-process start; `linux_boot_id` changes only with the Pi's Linux boot.
- Atomic `PersistQueue` admission of a new measurement and its initial `MessageProfiling` record establishes protocol acceptance but not SQLite durability.
- A receiver-state generation is usable only after durable acknowledgement or explicit startup reconciliation.
- Missing, corrupt or unsupported state never becomes an empty airtime ledger or trusted RTC provenance.
- Current `system_time_quality` and `rtc_health` are observed again on every receiver startup; persisted observations are diagnostic only.
- Monotonic time controls live deadlines, event intervals and airtime aging. Wall-clock adjustment never expires live airtime early.
- Known bucket charges, unresolved reservations and the complete active reservation must fit the configured continuous-window budget before a new reservation is committed.
- No TX occurs without an acknowledged active reservation owned by the current receiver instance.
- A reservation from an earlier receiver instance is fully charged, unspendable and retained until conservative expiration.
- A started or uncertain `SetTx` consumes allowance. Only a definite pre-`SetTx` failure permits reclamation.
- A pending or unknown reservation-settlement outcome suppresses TX.
- Airtime suppression never reverses application-message acceptance.
- An untrusted receiver UTC never becomes an immutable direct timestamp anchor.

## Deferred work

The following are explicitly deferred from the first pilot:

- durable duplicate/replay state across Pi reboot;
- durable queue or write-ahead journal before ACK;
- systemd service configuration;
- watchdog and health-management subsystem;
- sophisticated node scheduling;
- multi-receiver coordination;
- a kernel SX1262 driver;
- optimization from Python to C;
- complex BUSY edge handling;
- durable configuration management.

Known consequences:

- any receiver-process restart loses `transport_messages_map` and the optional `readings_map`;
- any receiver-process restart loses unpersisted queue contents;
- `linux_boot_id` distinguishes a Pi reboot from a receiver-only restart, while a new `receiver_instance_id` exposes both;
- a successful ACK does not guarantee durable storage;
- manual restart or deployment scripts may initially be required.

## Repository checks and missing details

The agent modifying this document or implementing the receiver must verify the following against the repository.

### Protocol compliance

- Exact definition of node identity and key lookup.
- Exact packet framing and clear/authenticated fields.
- Exact AES-CCM nonce construction as `node_id || message_id || domain` and its uniqueness requirements.
- Persistent monotonic `message_id` claim, exhaustion and identity-lifetime rules.
- Separation of transport identity by `(node_id, message_id)` from application-reading identity by `(node_id, sample_id)`.
- Confirmation that `sample_id` remains outside the nonce and unavailable before authentication and reading-body decode.
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

- SQLite schema for measurements, diagnostics, `MessageProfiling`, `ReceiverHealth` and lifecycle markers.
- Measurement idempotency or uniqueness constraints for `(node_id, sample_id)`.
- Transport-message indexing or diagnostics keyed by `(node_id, message_id)` without conflating it with measurement identity.
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
- `receiver_instance_id` generation and the temporary mapping from the protocol's `receiver_boot_id` name.
- `linux_boot_id` acquisition and scoping of Linux monotonic timestamps.
- Monotonic-to-UTC correlation representation and same-Linux-boot timestamp completion.
- SQLite representation of nullable reception/logical timestamps, timestamp provenance and immutable completion.

### Queue and memory policy

- Concrete 50 MB accounting strategy.
- Queue priority implementation.
- Telemetry sampling/rate limiting under pressure.
- Exact admission point at which a transport message becomes accepted.
- Atomic measurement and initial-`MessageProfiling` admission.
- `ReceiverHealthRequest` loss, sequence gaps and cumulative enqueue-failure accounting.
- Shutdown behavior with pending entries.

### Testing

At minimum, add tests or simulations for:

- new valid message;
- identical retry after successful ACK;
- identical retry after failed ACK transmission;
- same `message_id` with different authenticated frame bytes or domain;
- same `sample_id` in distinct current/backlog transport messages with matching contents;
- same `sample_id` in distinct transport messages with conflicting contents;
- ACK echo of authenticated uplink `message_id` and exact retransmission for the same outcome;
- nonce construction from `node_id || message_id || domain`, never `sample_id`;
- unauthenticated packet;
- authenticated malformed packet;
- queue full and later recovery;
- telemetry dropped while measurements continue;
- BUSY timeout;
- unexpected IRQ combination;
- TxDone missing or delayed;
- SQLite transaction failure;
- one poisoned entity followed by valid entities;
- receiver-process restart within one Linux boot and documented state loss;
- Pi reboot changing both identity fields;
- receiver lifecycle start, best-effort clean stop and missing-stop ambiguity;
- periodic `ReceiverHealthRequest` enrichment with distinct communicator and persistence sampling times;
- health-request loss under pressure without affecting measurement admission;
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

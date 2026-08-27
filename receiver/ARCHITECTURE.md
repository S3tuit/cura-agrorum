# Receiver Architecture

## Purpose

This document describes the pilot architecture for the Raspberry Pi LoRa receiver connected to an SX1262.

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
   - owns the SX1262, GPIO lines, SPI device, live protocol processing, and ACK generation;
   - owns the in-memory receiver state used for clock provenance and receiver TX-airtime admission;
   - performs only bounded in-memory work on the packet-to-ACK path;
   - never performs disk I/O or accesses SQLite.

2. **Persistence thread**
   - is the only thread allowed to access SQLite or the receiver configuration file;
   - drains a bounded in-memory queue in batches;
   - records measurements, message profiling, receiver health, and communicator-created diagnostics;
   - loads and validates receiver configuration during startup;
   - services high-priority synchronous communicator-state and lifecycle requests.

The threads communicate through three paths with different guarantees:

- bounded `PersistQueue` reservation/publication is an in-memory handoff; successful reservation establishes acceptance but does not imply disk durability;
- the synchronous persistence control channel loads receiver configuration and communicator state, commits generation-numbered communicator state, and records a confirmed clean stop; its private commands are separate from `PersistQueue`, and only confirmed durable completion acknowledges a new state generation;
- `PersistenceAdmissionState` is a generation-numbered snapshot owned by the persistence thread and read by the communicator before application/profile reservation; it gates new acceptance but does not promise that a later commit will succeed.

```text
SX1262
  |
  | DIO1 / BUSY / SPI
  v
Communicator thread
  |
  |-- measurements / profiling / health / diagnostics --> PersistQueue ---|
  |                                                                  |
  |<-- configuration/state results <-- persistence control channel ---|
  |                                                                  |
  |<-- read-only admission state -------------------------------------|
  |                                                                  |
  `-- state/clean-stop calls --> persistence control channel ----------|
                                                                     v
Persistence thread
  |-- SQLite
  `-- receiver configuration file
```

## Core ownership rules

These ownership rules should remain strict:

- Only the communicator thread touches the SX1262.
- Only the communicator thread owns `auth_node_map`; it keeps no transport-message or reading-duplicate history.
- Only the communicator thread mutates the live clock-provenance and airtime-admission state.
- Only the persistence thread touches SQLite or the receiver configuration file.
- Objects crossing `PersistQueue` must be complete, immutable enough for persistence, and independent from radio buffers.
- Communicator-state values crossing the persistence control channel must be immutable and carry a monotonically increasing generation.
- Ordinary persistence work must never block the packet-to-ACK path.

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

Configuration failure prevents the receiver from entering normal radio operation because it cannot authenticate nodes safely. This differs from a missing or corrupt `communicator_state` singleton in an otherwise usable database, which permits RX and application admission while conservatively suppressing TX as described below.

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

The process creates `receiver_instance_id` before normal radio operation and provides it read-only to both threads. The persistence thread reads and validates `linux_boot_id`, returns it with the successful startup configuration result, and adds it to stored rows whose queue entity omits it. The communicator includes the returned immutable value in fixed-size profiling entities without reading `/proc` itself. Per-instance occurrence sequences, health sequences, radio-recovery counters and other process-local counters reset when `receiver_instance_id` changes.

Linux monotonic timestamps are meaningful only together with `linux_boot_id`. Receiver-local sequences and lifecycle observations additionally require `receiver_instance_id` so a process restart within one Linux boot remains visible.

The persistence thread inserts the current `receiver_instances` row after
SQLite startup validation and before ordinary persistence admission becomes
available. A database-local monotonic `instance_ordinal` supplies lifecycle
order; `receiver_instance_id` remains the public identity referenced by other
tables. Start fields are immutable. During controlled shutdown, the
communicator calls `commit_receiver_clean_stop()` after its radio, queue and
airtime preconditions hold; the persistence thread is the only component that
updates the lifecycle row. A new instance without a durable clean-stop marker
for the preceding instance suggests a crash, kill, power loss or lost final
write; it is not proof of any one cause. Lifecycle telemetry does not alter
clock provenance, airtime state or recovery policy. The exact row and control
contracts are defined in [`INTERFACE.md`](INTERFACE.md).

## Durable communicator state

The receiver keeps one authoritative `CommunicatorStateV1` value in the
singleton SQLite `communicator_state` table. It is distinct from asynchronous
measurement and telemetry records but shares the receiver database and its
WAL/`FULL` durability policy. It contains the state needed to remain
fail-conservative across process restart, Pi reboot and power loss:

- format version, generation and integrity information;
- last-observed `system_time_quality` and `rtc_health` for diagnosis;
- authoritative clock-provenance fields, including the last verified network-to-RTC synchronization;
- the persisted representation of the rolling receiver TX-airtime ledger;
- unresolved airtime reservations from earlier receiver instances; and
- the active reservation, if one has been durably granted to the current receiver instance.

The exact canonical encoding, SQL envelope, generation rules and result types
are defined in [`INTERFACE.md`](INTERFACE.md). A commit is acknowledged only
after the exact new generation is durably installed. A malformed, digest-invalid,
unsupported, policy-incompatible or otherwise unverifiable row is not partially
recovered.

At startup, the persistence thread loads and validates the singleton before the
communicator is allowed to transmit. A missing or corrupt row means:

```text
airtime history = UNKNOWN_EXHAUSTED
RTC provenance  = UNTRUSTED
```

It must not be interpreted as an empty airtime ledger or as proof that the RTC
is correct. A valid generation-one commit may create a missing singleton. When
the singleton relation is corrupt but SQLite itself is healthy, the
generation-one replacement transaction first preserves every observed invalid
row in `quarantined_communicator_states`; preservation and replacement commit
atomically. An unsupported-version or airtime-policy-mismatch row is never
overwritten automatically. Whole-database corruption follows the separate
database-corruption policy.

Unknown airtime history becomes a known-empty ledger only after the running
instance has suppressed every TX for the complete rolling window plus the
configured bucket and expiration guards, measured on Linux monotonic time, and
trusted UTC is available for the new durable snapshot. A process restart
restarts that interval. This supplies automatic conservative recovery without
assuming that a missing row means a new installation.

The communicator is the sole logical owner of the live state, while the persistence thread is the sole physical writer. The commit protocol is:

```text
communicator creates immutable state snapshot with generation N
  -> call high-priority commit_communicator_state
  -> persistence thread durably installs generation N
  -> persistence thread acknowledges generation N
  -> communicator may rely on generation N
```

The persistence thread services communicator-state control commands before beginning another ordinary batch, once any transaction already in progress has reached a safe boundary. Each state update uses its own SQLite transaction. A reported failure before installation leaves the preceding durable generation authoritative. A timeout or otherwise unknown commit outcome must not assume either generation; TX remains disabled until a serialized load reconciles the installed generation and exact bytes. The communicator may continue receiving and accepting messages during either failure when ordinary persistence admission remains available.

## Time model

The receiver uses the Pi system clocks and the battery-backed DS3231 for different purposes:

- Linux monotonic time drives GPIO-event timing, deadlines, latency metrics and the live airtime ledger.
- Linux system UTC supplies receiver reception time and protocol timestamp anchors when its quality is trusted.
- The DS3231 preserves UTC while the Pi is unpowered and seeds the system clock through the deployment's one-per-Linux-boot RTC-bootstrap stage. It is not read for each packet and is not used for elapsed-time measurements.

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
- a currently present and valid RTC;
- an RTC value consistent with the last verified synchronization and the configured uncertainty policy; and
- current Linux system UTC consistent with that RTC value after the boot-time bootstrap episode.

The last observed quality and health may be recorded for diagnosis, but they are not restored as authoritative current-instance observations. On every receiver startup the receiver probes the RTC and independently establishes current time quality.

### Boot and synchronization lifecycle

On an offline Pi boot, the deployment RTC-bootstrap stage may provisionally copy a hardware-valid DS3231 value into Linux system UTC. The receiver enters `RTC_HOLDOVER` only after it independently validates durable RTC provenance, current RTC health and agreement between RTC and system UTC. With no valid provenance, system UTC remains `UNTRUSTED` even if the bootstrap stage copied a provisional RTC value. A missing or invalid RTC is recorded independently in `rtc_health`.

When the time-synchronization service confirms network synchronization, the trust transition is ordered:

1. Mark current Linux system UTC as `NETWORK_SYNCED` in memory.
2. Write UTC to the DS3231.
3. Read back and verify the RTC update.
4. Durably commit the new RTC provenance through `commit_communicator_state()`.
5. Only after acknowledgement may a later Pi boot use that update for `RTC_HOLDOVER`.

A reset between the RTC update and the durable provenance commit leaves a good RTC conservatively classified as untrusted. The reverse ordering is forbidden because it could make a later Pi boot trust an RTC that was never updated. The synchronized system time may be copied to the RTC periodically while network synchronization remains confirmed. Clean shutdown has no special clock-persistence role, and an unsynchronized Pi must never overwrite a credible RTC.

### Reception timestamps

The kernel-recorded DIO1 `RX_DONE` edge uses monotonic time. The communicator also captures or obtains a close correlation between monotonic time and system UTC, together with:

- `receiver_instance_id`;
- `linux_boot_id`;
- `system_time_quality`;
- `rtc_health`.

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

## Deployment lifecycle

The pilot receiver is an unattended boot service, not a manually started application. A systemd unit or equivalent supervisor configuration is a first-pilot deliverable. It must enable the receiver at boot, restart it after abnormal process termination and enforce the local prerequisites below without waiting for Internet access.

The required boot ordering is:

```text
required local persistent storage mounted and writable
  +
DS3231 driver/device probe and bounded RTC-bootstrap episode completed
  |
  v
receiver service starts
  -> create receiver_instance_id
  -> persistence thread reads and validates linux_boot_id
  -> persistence thread validates storage, configuration, migrations,
     enum catalogues and durable communicator state
  -> persistence thread inserts receiver_instances start row
  -> probe RTC and local time-synchronization status
  -> establish NETWORK_SYNCED, RTC_HOLDOVER or UNTRUSTED
  -> reconcile durable airtime state and obtain usable allowance when possible
  -> initialize the SX1262
  -> enter RX_SINGLE
```

The storage prerequisite covers the filesystems containing the receiver
configuration, SQLite database, WAL and temporary SQLite files. The service
manager must order startup after those mounts. If the required storage is
unavailable or not writable, the receiver must not enter radio operation. A
missing or corrupt `communicator_state` row in an otherwise usable compatible
database instead retains the fail-conservative behavior: ordinary RX and
application admission may proceed while TX remains suppressed until a valid
state generation and reservation are durably established. An incompatible
database schema prevents ordinary persistence admission and therefore prevents
accepted ACK outcomes.

The DS3231 itself is not booted by Linux; it keeps counting from Pi power or its backup battery. RTC bootstrap means one RTC-to-Linux-system-clock bootstrap episode per `linux_boot_id`. The episode may make a small configured number of local read attempts, but it has a finite total deadline. It checks hardware-level validity, including any available invalid-time or oscillator-stop indication, before provisionally setting Linux UTC. It then terminates with success, missing, invalid or I/O/timeout status. Exact retry and deadline values are deployment parameters.

Completion of the RTC-bootstrap episode is an ordering prerequisite; success is not. A failed episode must not wait for a network or block receiver startup indefinitely. The receiver starts, reports the observed `rtc_health`, keeps `system_time_quality = UNTRUSTED` unless another trusted source is available, and continues RX under the existing fail-conservative timestamp and TX policies.

The bootstrap episode runs once per Linux boot, not once per receiver process. An automatic receiver restart under the same `linux_boot_id` must not copy the RTC into the system clock again. It probes current RTC health and time quality, creates a new `receiver_instance_id`, reconciles durable communicator state and proceeds from the clocks already running in that Linux boot. The bootstrap component does not read or write receiver configuration or SQLite; RTC provenance remains owned by the communicator-state model.

The RTC bootstrap must complete before network time discipline is allowed to establish synchronization; alternatively, the bootstrap must detect already-synchronized system time and skip the RTC-to-system copy. This prevents a late RTC bootstrap from overwriting NTP-disciplined time. The receiver service itself must not depend on a network-online target. The local time-synchronization service may establish `NETWORK_SYNCED` asynchronously after the local bootstrap prerequisite, and the receiver does not wait for it. Offline startup therefore produces either validated `RTC_HOLDOVER` or `UNTRUSTED`, never a wait for NTP.

The supervisor restarts crashes, uncaught-failure exits and terminal initialization or recovery failures with a nonzero restart delay and rate limiting so a permanent hardware fault cannot create a tight restart loop. A restarted process follows the normal receiver-instance and unresolved-airtime-reservation rules. Controlled service stop uses a bounded graceful-shutdown interval, but correctness never depends on a clean-stop marker or shutdown-time RTC write.

For systemd, the unit-level contract is:

- enable the receiver service for normal boot;
- order it after the required mounts and the boot-scoped RTC-bootstrap unit, while treating RTC-bootstrap completion rather than success as the prerequisite;
- give the RTC-bootstrap unit a finite operation timeout and ensure it runs no more than once in one Linux boot;
- do not add a `network-online.target` ordering requirement;
- run the receiver as the configured service user with access only to its data directories and required GPIO, SPI and RTC devices; and
- use restart-on-failure with a nonzero restart delay, rate limiting and a bounded graceful-stop timeout.

Exact unit names, paths, retry counts and timeouts are deployment configuration, but these semantics are not optional or deferred from the pilot.

### Controlled shutdown

Graceful shutdown is a bounded best-effort optimization for an orderly Pi reboot or power-off, `systemctl stop`, and the stop phase of `systemctl restart`. It is not used for correctness and cannot run after `SIGKILL`, process crash or sudden power loss.

On the supervisor's termination signal:

```text
stop admitting new radio events and suppress new TX
  -> finish or conservatively terminate the active radio operation
  -> place the SX1262 in the configured safe shutdown state
  -> settle the active airtime reservation when persistence permits;
     otherwise leave it conservatively unresolved
  -> let the persistence thread drain published FIFO units until
     the internal shutdown deadline
  -> if the queue drained and airtime-state handling reached a known outcome,
     call commit_receiver_clean_stop() through the control channel
  -> attempt a bounded WAL checkpoint and close SQLite
  -> exit
```

The application's internal deadline must expire before the supervisor's stop
timeout. The communicator invokes the clean-stop operation; the persistence
thread validates the database and queue preconditions and atomically completes
the current `receiver_instances` row. The marker asserts only that new
admission stopped, the radio reached its safe state, the published queue
drained, and airtime-state handling reached a known authoritative generation.
It does not assert that the best-effort WAL checkpoint ran successfully.
Repeating the identical request is idempotent; a deadline after SQLite may have
committed yields an unknown outcome and is reconciled by repeating that exact
request. If any prerequisite or commit cannot complete in time, the process
exits without claiming a clean stop; remaining volatile queue units are lost
and unresolved airtime state is recovered conservatively on the next instance.
A checkpoint or database-close failure is reported through the service log but
does not invalidate an already durable clean-stop marker. The supervisor may
then finish stopping or restarting the service. No shutdown-time RTC write is
required.

## Radio access

The pilot receiver runs entirely in userspace.

Use:

- `libgpiod` for DIO1 edge events and GPIO reads;
- `spidev` for SX1262 SPI commands;
- bounded polling of BUSY around SPI commands;
- SX1262 single-receive mode.

The communicator requests DIO1 as an input with rising-edge detection and blocks waiting for events.

BUSY may initially be polled rather than handled through a separate edge-event mechanism. Every BUSY wait must have a timeout.

### Required radio profiles and IQ transitions

The complete profile in [Protocol v2 LoRa: LoRa PHY framing](../protocol/protocol-v2-lora/README.md#lora-phy-framing) is normative for receiver interoperability. Receiver initialization, normal operation and recovery must apply that profile exactly; its frequency, modulation, packet, sync-word, CRC, gain, ramp and direction-specific IQ requirements are not deferred implementation choices. Hardware configuration may add module-specific regulator, TCXO, calibration, RF-switch and PA details, but it must not override the protocol PHY profile without a protocol revision.

The communicator treats the protocol configuration as two explicit operational profiles:

```text
UPLINK_RX_PROFILE = protocol PHY profile for normal-IQ, boosted uplink reception
ACK_TX_PROFILE    = protocol PHY profile for inverted-IQ ACK transmission
```

`INITIALIZING` and every recovery path must install `UPLINK_RX_PROFILE` before issuing `SetRx`. `RX_SINGLE` means both that this complete receive profile is known to be active and that `SetRx` was confirmed; `SetRx` alone is insufficient.

ACK frame construction changes only Pi-owned bytes. It does not configure the radio. After ACK admission and airtime admission, the communicator must explicitly:

```text
write the exact ACK frame to the SX1262 buffer
  -> install ACK_TX_PROFILE, including inverted IQ
  -> tentatively consume the charged airtime allowance
  -> issue SetTx
  -> handle the terminal TX outcome
  -> reinstall UPLINK_RX_PROFILE, including normal IQ and boosted RX
  -> issue and confirm SetRx
  -> enter RX_SINGLE
```

If ACK TX-profile installation definitely fails before `SetTx` can take effect, no transmission started; the communicator restores `UPLINK_RX_PROFILE` and re-arms RX, or enters `RECOVERING` if it cannot confirm that state. If any profile command or `SetTx` has an uncertain outcome, the communicator must not issue `SetRx` under a possibly inverted-IQ or otherwise partial configuration. It enters bounded recovery, retains any required conservative airtime charge, restores the complete receive profile and only then confirms `SetRx`.

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

`RX_SINGLE` means that the communicator has confirmed `UPLINK_RX_PROFILE`, cleared or accounted for preceding IRQs and confirmed `SetRx`. A low DIO1 level alone does not prove that the radio is receiving.

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
  -> full initialization + confirmed UPLINK_RX_PROFILE + SetRx -> RX_SINGLE
  -> required hardware absent or unreachable -> HARDWARE_MISSING
  -> bounded initialization attempts fail -> INITIALIZATION_FAILED

RX_SINGLE
  -> DIO1 edge -> RX_EVENT_PENDING

RX_EVENT_PENDING
  -> event handled without TX + confirmed UPLINK_RX_PROFILE + SetRx -> RX_SINGLE
  -> SetTx confirmed or uncertain -> TX_ACTIVE

TX_ACTIVE
  -> terminal TX IRQ + confirmed UPLINK_RX_PROFILE + SetRx -> RX_SINGLE

RX_SINGLE / RX_EVENT_PENDING / TX_ACTIVE
  -> radio mode or configuration uncertain, or RX re-arm fails -> RECOVERING

RECOVERING
  -> recovery + confirmed UPLINK_RX_PROFILE + SetRx -> RX_SINGLE
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
  -> decide accepted/rejected/retry-later
  -> prepare ACK
  -> require committed airtime reservation
  -> suppress ACK + confirmed UPLINK_RX_PROFILE + SetRx -> RX_SINGLE
  or
  -> write ACK frame + install ACK_TX_PROFILE with inverted IQ
  -> SetTx confirmed or uncertain
  -> TX_ACTIVE
  -> TxDone or radio timeout / DIO1 when TX terminates
  -> clear terminal TX IRQ
  -> reinstall UPLINK_RX_PROFILE with normal IQ and boosted RX
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
- ACK selection, construction, and transmission;
- ordinary `PersistQueue`-admission gating from the current read-only `PersistenceAdmissionState`;
- live time-quality, RTC-health and clock-correlation state;
- live rolling-airtime buckets, unresolved reservations and the active reservation;
- immutable communicator-state snapshot creation and generation tracking;
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

It may wait for acknowledgement of a high-priority `commit_communicator_state()` call outside the packet-to-ACK critical path. Reservations should be renewed proactively. If a packet requires an ACK while no committed allowance is usable, the communicator suppresses that ACK rather than waiting for a state commit.

### Node map

At startup, the communicator receives the immutable configuration snapshot loaded and validated by the persistence thread. It derives the active nodes' authentication material and constructs `auth_node_map` in RAM before entering RX.

Conceptually:

```text
auth_node_map[node_id] -> NodeState
```

Each `NodeState` contains only the provisioned material and immutable metadata needed to authenticate the node. It contains no accepted-message or application-reading history.

Repository protocol code determines the exact key derivation and lookup rules. Retired and unknown node identifiers are not inserted into the map and receive no response. The communicator does not access the configuration file after startup.

### Transport identity and CCM nonce

The fixed clear uplink header carries `node_id`, `message_id` and `domain`. These values are untrusted claims until AES-CCM authentication succeeds. The receiver authenticates the exact clear header as AAD and constructs the nonce exactly as defined by the protocol:

```text
node_id || message_id || domain
```

`message_id` is the persistent monotonic transport counter scoped to one node identity and key. It identifies a newly constructed logical LoRa message, not an individual RF attempt. Retransmissions of that logical message reuse the same `message_id`, domain, authenticated bytes and complete frame. Converting a current reading into a newly constructed backlog message allocates a new `message_id` even though the encrypted reading retains its original `sample_id`. The receiver must tolerate skipped IDs, but authenticated reuse under the same node identity is invalid; exhaustion or loss of the node counter requires a new node identity and key.

`sample_id` remains inside the authenticated reading body. It identifies the application reading, supports wake continuity and timestamp reconstruction, and is unavailable until authentication and decoding succeed. It is not part of the CCM nonce and does not identify RF retransmission episodes.

An ACK echoes the authenticated uplink's `message_id`; it does not carry `sample_id`. Its ACK domain selects the downlink nonce domain. ACK construction is deterministic: reconstructing an ACK for the same authenticated uplink and outcome produces the same exact frame without consulting receiver history. Different outcomes use distinct ACK domains and therefore distinct nonces under the same node key and `message_id`.

### Stateless communicator admission

The communicator keeps no `transport_messages_map`, `readings_map`, cached ACK or other per-node acceptance history. Every authenticated, structurally valid reading occurrence follows the same bounded admission path, including an exact RF retransmission after a lost ACK and a reading that SQLite has already stored. Duplicate and conflict classification belongs exclusively to the persistence thread.

This deliberately permits the communicator to acknowledge a candidate that persistence later classifies as conflicting. Such an ACK means only that the authenticated and structurally valid occurrence entered the bounded persistence pipeline. It does not mean that the candidate became the canonical transport message or measurement.

### Acceptance invariant

An authenticated, structurally valid reading occurrence may be marked accepted only after one atomic `PersistQueue` reservation exclusively owns the exact fixed-size application-candidate/`MessageProfiling` unit, including preallocated slots for all post-TX fields. The reservation counts against queue capacity immediately but is not visible to the persistence consumer until the communicator publishes it. The pair remains one logical persistence unit and batch selection must not split it across SQLite transactions.

Required ordering:

```text
validate message
  -> construct and validate the fixed-layout candidate/profile unit and candidate success ACK
  -> require persistence admission state AVAILABLE
     -> unavailable: reserve nothing and select RETRY_LATER
  -> atomically reserve the unit's exact bytes
     -> capacity failure: reserve nothing and select RETRY_LATER
     -> success: mark occurrence accepted and transmit the selected ACK when permitted
  -> reach a terminal TX/suppression/recovery outcome
  -> finalize one immutable pair and publish it against the reservation
```

Publication uses the existing reservation, performs no new capacity check and cannot fail because of queue pressure. If TX fails or its ACK is lost, a later retransmission repeats this complete admission path. This may publish another measurement candidate, but persistence retains only canonical data and stores a profile for every admitted radio occurrence.

### Retry-later behavior

`ACK_RETRY_LATER_DOWNLINK` means that the receiver did not accept ownership of this occurrence because durable persistence was currently unavailable or it could not reserve the exact capacity for the complete measurement/profile unit. When the node retries, the occurrence goes through the same availability and reservation path again. The communicator neither remembers nor caches `RETRY_LATER`.

### ACK semantics in the pilot

A successful ACK means:

> The receiver authenticated and validated the message while persistence admission was available, then reserved exclusive bounded-pipeline capacity for its fixed-size application-candidate/packet-occurrence unit.

It does **not** mean:

> The message has already been durably committed to SQLite.

Therefore, an acknowledged measurement can still be lost if the Pi loses power before persistence completes.

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

When no committed airtime reservation fits, an otherwise valid message is still authenticated, validated and accepted through a `PersistQueue` capacity reservation; its ACK is suppressed. Lack of receiver TX budget never reverses message acceptance.

### Invalid and rejected packets

The ordered procedure in [Protocol v2 LoRa: Receiver validation](../protocol/protocol-v2-lora/README.md#receiver-validation) is normative for the communicator. The communicator applies those checks in order. In particular, a protocol outcome that requires silence is terminal and must not be converted into a response by later validation or queue admission.

The architecture adds these execution requirements around that protocol procedure:

- unauthenticated fields are untrusted input;
- the protocol alone determines whether an authenticated packet is eligible for a rejection ACK;
- failure to admit a required occurrence profile changes an otherwise eligible authenticated response to `ACK_RETRY_LATER_DOWNLINK`;
- an authenticated downlink ACK domain that reaches the protocol's direction check is `WRONG_DIRECTION`: the receiver attempts to record it as reflected or replayed traffic, `ack_selected` is none, and no response is sent;
- failure to admit a `WRONG_DIRECTION` profile must not turn the outcome into `ACK_RETRY_LATER_DOWNLINK` or any other response;
- ACK selection and ACK transmission remain separate decisions, because the receiver TX-airtime budget can suppress a selected ACK; and
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
Apply the normative protocol Receiver validation procedure in order,
including fixed-header parsing, node lookup and authentication with
nonce = node_id || message_id || domain
T3 = authentication complete, when authentication was attempted
Decode sample_id only at the protocol's reading-body validation step
  |
  +--> protocol requires no response
  |       (a downlink ACK domain reaching the direction check is WRONG_DIRECTION)
  |       -> no ACK
  |       -> SetRx, T6 = SetRx issued
  |       -> construct complete MessageProfiling and try to enqueue it
  |       -> if admission fails, increment the profiling-admission counter;
  |          never change the outcome to RETRY_LATER
  |
  +--> protocol selects a response-eligible authenticated rejection
  |       -> construct the protocol-selected rejection ACK and pre-TX profiling fields
  |       -> require persistence AVAILABLE and reserve exact profile capacity,
  |          or select RETRY_LATER
  |
  +--> authenticated structurally valid reading
          -> validate
          -> construct measurement candidate, deterministic candidate ACCEPTED ACK
               and pre-TX MessageProfiling fields containing the exact ACK frame
          -> require persistence AVAILABLE and atomically reserve the exact
               fixed-size pair without consulting message history
          -> if unavailable: reserve nothing, RETRY_LATER, not accepted
          -> otherwise: mark occurrence accepted and select ACCEPTED
  |
  v
For a response-eligible outcome, use the selected deterministic ACK
Check committed airtime reservation
  |
  +--> unavailable/exhausted
  |       -> suppress ACK
  |       -> SetRx
  |       -> T6 = SetRx issued
  |
  `--> available
          -> write exact selected ACK frame to the SX1262 buffer
          -> install and confirm ACK_TX_PROFILE, including inverted IQ
          -> tentatively consume charged allowance
          -> SetTx
          -> T4 = SetTx issued or attempted
          -> TxDone / DIO1
          -> T5 = TxDone edge
          -> Clear TxDone
          -> reinstall and confirm UPLINK_RX_PROFILE,
               including normal IQ and boosted RX
          -> SetRx
          -> T6 = SetRx issued
  |
  v
Finalize ack_tx_result and T4 through T6
If a pre-TX reservation exists, publish one complete immutable
profile-only or measurement/profile unit without another capacity check;
otherwise record the profiling-admission failure counter only
Return to DIO1 wait
```

The communicator reserves the exact fixed-size unit before an ACK transmission but publishes no partial profile. It retains the preallocated unit locally while TX, suppression and any bounded recovery complete, fills only the fixed-width terminal TX result and timestamp slots, then freezes and publishes it. Publication cannot fail because of queue pressure, allocation or serialization.

If persistence admission is unavailable or profile reservation fails, the detailed packet-occurrence record cannot be retained. The pilot explicitly permits this exception, selects `ACK_RETRY_LATER_DOWNLINK` for an authenticated packet that is eligible for a response, and increments `message_profiling_admission_failures` plus the bounded counter for the observed admission-failure reason. Packets that cannot be authenticated remain silent. The counters are reported later through the existing non-recursive persistence or diagnostic path so that gaps in the occurrence log are observable.

If the communicator regains control after an exception but cannot determine the attempted ACK's terminal radio outcome, it finalizes the reserved profile as `UNKNOWN_INTERRUPTED` before entering a terminal receiver state. A hard process crash or power loss drops the volatile reservation and leaves no partial SQLite row. This deliberately gives up persistence of pre-TX partial records and does not weaken the pilot's existing non-durable ACK guarantee.

Every failure path must either:

- intentionally transition the radio to a known next state; or
- enter a bounded recovery path and create a diagnostic.

## PersistQueue

`PersistQueue` is a bounded in-memory handoff between the communicator and persistence threads.

The concrete shared values, enum assignments, optional-field rules and fixed
entity layouts are defined by [`INTERFACE.md`](INTERFACE.md).

A `PersistQueue` capacity reservation is volatile, process-local queue accounting. It is unrelated to the separately persisted receiver TX-airtime reservation described above.

Initial target capacity:

```text
approximately 50 MB
```

Every `PersistQueue` entity kind has a fixed serialized size chosen with its schema. Different entity kinds may have different fixed sizes, but an individual kind must not contain dynamically sized strings, byte arrays, containers or object graphs. Variable-length protocol values use fixed-capacity storage plus an explicit length; for example, a received frame uses a protocol-sized byte array and `received_frame_length`. Diagnostics use fixed fields and bounded buffers rather than arbitrary text or dictionaries.

The fixed sizes are implementation constants validated at startup. Byte-capacity accounting and every reservation use their exact sum, not an estimated Python-object size or object count. A separate configured entity-count limit additionally bounds Python object and queue-node overhead; it does not replace exact byte accounting.

The queue may contain:

- atomic measurement-candidate/complete-`MessageProfiling` units;
- complete profile-only `MessageProfiling` records for occurrences without an application candidate;
- `ReceiverHealthRequest` snapshots;
- structured diagnostics.

### Pilot queue policy

The pilot implements no `PersistQueue` priority policy. Every object published to the queue has the same importance regardless of whether it contains a measurement/profile pair, a profile-only occurrence, a diagnostic or a `ReceiverHealthRequest`.

The queue admits and processes published units in FIFO publication order. It does not reorder, evict, sample or drop an admitted unit based on entity type. New admission requires current `PersistenceAdmissionState = AVAILABLE`, sufficient exact byte capacity and space under the configured entity-count limit; any failure rejects the new object or complete atomic pair without removing existing entries. For an authenticated packet eligible for a response, either admission failure selects `ACK_RETRY_LATER_DOWNLINK`.

Outstanding reservations count against queue capacity but remain invisible to the persistence consumer until publication. Publishing a correctly sized unit against its reservation performs no capacity check and cannot be rejected due to pressure. Once published, every entity remains queue-owned until its persistence transaction commits, independent of its type.

The communicator is the only reservation owner and processes one radio event at a time, so it publishes or terminally finalizes the active reservation before accepting another packet. Reserved bytes are released only by successful publication or by cancellation before any response is attempted. Once an occurrence has been accepted, its reservation must be published with the best terminal outcome available rather than cancelled.

A reservation token is unique and single-use. Before reserving capacity and selecting an ACK, the communicator constructs and validates the complete fixed-layout measurement candidate and all stable `MessageProfiling` fields. The profiling layout preallocates fixed-width slots for `ack_tx_result` and `T4` through `T6`; after the terminal radio outcome, the communicator only fills those already validated slots and freezes the unit. No allocation, variable-sized serialization or fallible representation conversion is permitted after ACK selection.

Publication atomically transfers the complete immutable fixed-size unit against its exact reservation. A size mismatch or inability to fill a prevalidated slot is an implementation invariant violation, not a reason to perform a second admission attempt after an ACK may have been sent.

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

The persistence thread is the exclusive owner of SQLite and the receiver
configuration file.

It:

- loads, validates and publishes the immutable receiver configuration snapshot during startup;
- applies and validates the forward-only SQLite migrations and generated enum catalogues before admission;
- inserts the current `receiver_instances` start row;
- loads and validates durable communicator state during startup;
- opens and validates SQLite with the required WAL and synchronization settings;
- exclusively publishes `PersistenceAdmissionState` and closes ordinary admission on storage failure;
- services high-priority state and clean-stop control operations and acknowledges only confirmed durable outcomes;
- wakes approximately every five seconds, or when a configurable queue threshold is reached;
- checks whether the queue contains data;
- takes a logical batch of at most approximately N MB, where N is chosen from benchmarks on the deployed Pi and storage medium;
- writes the batch using one SQLite transaction;
- removes entries from the queue only after a successful commit;
- classifies transport retransmissions and identity conflicts against canonical SQLite records;
- applies idempotent measurement insertion without replacing canonical contents;
- accumulates persistence timing, throughput, failure, quarantine and checkpoint counters for `ReceiverHealthV1`;
- durably isolates item-specific poisoned units without silently dropping them;
- inserts each complete `MessageProfiling` row once;
- enriches `ReceiverHealthRequest` snapshots with persistence-owned and host-owned observations and inserts complete `ReceiverHealth` rows;
- repeats while work remains.

The five-second interval and batch limit are pilot defaults and should remain configurable.

### SQLite durability and retention

The schema is managed by immutable, forward-only SQL migrations under
`receiver/db/migrations/`. The migration runner owns each migration transaction
and records the filename and SHA-256 digest in `schema_migrations` while keeping
SQLite `user_version` synchronized. SQLite `application_id`, the complete
migration history and the generated persisted-enum catalogue must match this
receiver build before ordinary admission opens. The full migration, table,
identity and enum-generation contracts are defined in
[`INTERFACE.md`](INTERFACE.md) and [`db/README.md`](db/README.md).

The pilot database uses `PRAGMA journal_mode=WAL` and verifies that SQLite actually entered WAL mode. Every database connection sets `PRAGMA synchronous=FULL`; a connection that cannot establish the required journal or synchronization mode is not usable for persistence. The database, WAL and shared-memory files must remain together on the same local filesystem.

WAL checkpointing uses a configurable page or byte threshold and records checkpoint duration and result. Checkpoint work must not make the persistence thread ignore queue growth or state-commit requests indefinitely. A bounded checkpoint is also attempted during controlled shutdown before the database connection closes.

Implementation benchmarking must compare `synchronous=FULL` with `synchronous=NORMAL` on the deployed Pi and storage medium using realistic entity mixes, batch sizes and checkpoint behavior. Record at least transaction throughput, commit-latency percentiles, queue growth, WAL growth and checkpoint stalls. `FULL` remains the pilot deployment setting regardless of benchmark results; changing it requires an explicit architecture decision accepting weaker power-loss durability.

These settings protect transactions whose SQLite commit completed. They do not make a preceding radio ACK durable while its fixed-size queue unit remains only in RAM.

The pilot performs no automatic retention deletion. Canonical readings, transport messages, packet profiles, diagnostics, receiver health and quarantined entities are retained for the complete pilot. Storage must be provisioned from worst-case pilot rates with margin for the database, WAL, temporary SQLite files and quarantine. A configurable free-space low-water mark closes all new ordinary `PersistQueue` admission before actual exhaustion. Export, archive or pruning is an explicit maintenance operation outside active pilot collection; it must not silently remove canonical identity history needed for duplicate and conflict classification.

### Persistence availability and storage failures

The persistence thread owns a small read-only-to-the-communicator availability snapshot containing a generation, the current `PersistenceAdmissionState` value and its monotonic transition time:

```text
UNAVAILABLE_STARTING
AVAILABLE
UNAVAILABLE_LOW_SPACE
UNAVAILABLE_DISK_FULL
UNAVAILABLE_CORRUPT
UNAVAILABLE_IO
UNAVAILABLE_INCOMPATIBLE_SCHEMA
```

The state starts as `UNAVAILABLE_STARTING` and becomes `AVAILABLE` only after SQLite opens, migrations, enum catalogues and immutable database metadata are compatible, the required durability settings are confirmed and startup validation succeeds. A newer database, migration gap, changed migration digest, incompatible enum catalogue or configured-group/database-metadata mismatch publishes `UNAVAILABLE_INCOMPATIBLE_SCHEMA`; the receiver never guesses how to interpret that database. A current unavailable state closes all new ordinary `PersistQueue` admission even if RAM capacity remains; already published units stay queue-owned, and the synchronous persistence control channel retains its separate semantics. An authenticated packet otherwise eligible for a response is not accepted and selects `ACK_RETRY_LATER_DOWNLINK`; packets requiring silence remain silent. Periodic health, diagnostics and other ordinary entities increment bounded in-memory failure counters rather than entering the unavailable queue. The communicator retains counters by unavailable reason so the outage can be reported after persistence recovers. Availability is an admission gate, not a durability promise: a transaction can still fail immediately after the communicator observes `AVAILABLE`.

On a low-space or disk-full condition, the persistence thread:

1. rolls back the transaction and retains ownership of every affected queue unit;
2. publishes the corresponding unavailable state;
3. avoids a tight retry loop by using bounded retry backoff while observing free space;
4. retries the original immutable units without reconstruction when space is available; and
5. returns to `AVAILABLE` only after the required database checks and a transaction succeed.

Disk-full failure is not a poisoned-entity classification and must not cause a queued unit to be dropped or quarantined.

On SQLite corruption or a failed configured integrity check, the persistence thread rolls back when possible, publishes `UNAVAILABLE_CORRUPT`, closes the database and preserves the database, WAL and shared-memory files together. It must not automatically delete, replace, truncate or rebuild them. Radio RX may continue with all new ordinary `PersistQueue` admission closed so nodes retain their readings. Operator recovery must preserve the corrupt artifacts for diagnosis, restore or recover the database through an explicit maintenance procedure, and pass startup validation before persistence returns to `AVAILABLE`.

If the process crashes or loses power while an accepted unit remains only in the volatile queue during either outage, that unit can still be lost under the pilot's documented non-durable-ACK guarantee. Eliminating that window requires the deferred durable queue or pre-ACK journal.

### Communicator-state ownership

The persistence control channel is separate from `PersistQueue` and has stronger semantics. A private control command is never sampled, dropped, merged silently or acknowledged on submission. `commit_communicator_state()` either reports the requested generation and exact canonical bytes durably installed, reports that the preceding generation definitely remains authoritative, or returns an unknown outcome that requires a serialized state reload before TX resumes.

Servicing this separate synchronous control path is not a `PersistQueue` priority class and must not reorder already published queue units.

The persistence thread does not independently edit clock or airtime policy. It validates the state envelope, canonical encoding, digest, policy parameters and generation ordering, writes the communicator-owned singleton and returns the result. A corrupt application-level relation can be replaced by generation one only in the same transaction that archives every exact rejected row in `quarantined_communicator_states`. An unsupported state version or policy mismatch is not repaired automatically. This preserves a single policy owner without allowing two threads to write disk.

An ordinary SQLite batch and a communicator-state transaction remain separate because they protect different guarantees. Their I/O ordering must nevertheless ensure that a control command cannot be starved indefinitely by telemetry batches.

### Transport and reading classification

The persistence thread classifies accepted reading occurrences in queue order. SQLite maintains two independent canonical identities:

```text
transport_messages UNIQUE(node_id, message_id)
readings           UNIQUE(node_id, sample_id)
```

The canonical transport row stores the first exact authenticated frame, domain and decoded `sample_id`; the canonical reading row stores the first accepted application contents. These constraints are scoped to the node identity and key lifetime required by the protocol. The configured provisioning rules must not silently reuse a database identity with a reset counter.

For an existing transport key, persistence compares the candidate with the canonical transport row:

- same `message_id`, same `sample_id` and same exact frame is `RETRANSMISSION`;
- same `message_id`, same `sample_id` and a different frame is `DUPLICATE_CONFLICT`;
- same `message_id` and a different `sample_id` is `MESSAGE_ID_CONFLICT`.

A conflict does not replace the canonical transport row and its measurement candidate is not inserted into `readings`. The occurrence profile records the derived classification; together with the immutable canonical transport and reading rows, it is the durable conflict evidence. Persistence creates no conflict `DiagnosticV1`.

For a first-seen transport message, persistence next applies the reading key. A first-seen `(node_id, sample_id)` inserts the measurement. Existing identical application contents produce `DUPLICATE_SAME_CONTENT`; existing different contents produce `DUPLICATE_CONFLICT`. Neither case updates the canonical measurement.

Classification, canonical effects and the occurrence profile belong to one SQLite transaction. Conflict handling is successful processing and the corresponding queue pair is removed after commit. A SQLite or transaction failure is not a conflict: the queue retains ownership and retries the original immutable pair. The persistence thread constructs the stored profile with its derived `persistence_classification`; it does not mutate the communicator's queued object.

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

An atomic measurement/profile pair is one batch unit. A configured batch-size target may not split it; when a single pair exceeds that target but fits the queue's validated entity-size limit, persistence processes the complete pair in one transaction.

### Poisoned entities

All queue units have already passed fixed-layout construction and representation validation before admission. A poisoned unit is an admitted, representation-valid immutable unit that reproducibly fails while isolated because of an entity-specific decoder, binding, derivation, range or unexpected schema-constraint defect. Malformed radio input, duplicate classification, expected uniqueness conflicts, disk full, locking, database corruption and transient or global I/O failures are not poison.

The persistence thread needs a bounded failure-isolation strategy:

```text
batch fails
  -> roll back and exclude global or transient causes
  -> narrow the batch and reproduce the suspected unit alone
  -> durably quarantine its exact fixed-size bytes and minimal failure provenance
  -> only then remove it from PersistQueue
  -> continue with later entities
```

An admitted unit is never silently dropped, regardless of whether an ACK was transmitted for it. A `MeasurementProfileUnitV1` remains one indivisible 516-byte unit during isolation and quarantine. The generic append-only SQLite `quarantined_entities` table preserves the complete original canonical bytes, redundant kind/version/length metadata, receiver and Linux identities, quarantine monotonic time, database schema version, stable failure reason and operation, available SQLite/OS codes and isolation-attempt count. Its primary key is SHA-256 of the exact bytes; an identical existing row makes retry idempotent only after every stored value is verified.

After the normal batch transaction rolls back, quarantine uses a separate WAL/`FULL` transaction. Only a confirmed or reconciled quarantine commit permits the batch to acknowledge that unit as `QUARANTINED`. If quarantine cannot be durably committed, the complete batch remains queue-owned, persistence publishes `UNAVAILABLE_IO` and all new ordinary `PersistQueue` admission closes. Infinite retry of the same failing batch is not acceptable, but neither is removing an already-ACKed unit without a durable copy. Quarantining is successful failure isolation, not successful canonical measurement persistence; the retained bytes exist for diagnosis and later recovery.

Persistence does not invent a `DiagnosticV1` for an asynchronous poison. Only the communicator owns diagnostic identity. Poison remains observable through the quarantine row, bounded service logging and cumulative `ReceiverHealthV1` counters. The exact schema and allowed failure reasons are defined in [`INTERFACE.md`](INTERFACE.md).

## Telemetry

### MessageProfiling

`MessageProfiling` is the single logical packet-occurrence record required by the protocol. It combines timing data with the received frame, authentication and processing decisions, duplicate results, radio metadata, selected ACK and ACK-transmission result. The communicator copies the raw frame and radio metadata into Pi-owned memory before later radio activity can overwrite them.

Every admitted record is identified by:

- `receiver_instance_id`; and
- a monotonically increasing per-instance occurrence sequence.

Together these fields identify the single complete SQLite profiling row.

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

When the persistence thread writes the row, it adds the protocol-defined `persistence_classification`: `NOT_APPLICABLE`, `FIRST_SEEN`, `RETRANSMISSION`, `DUPLICATE_SAME_CONTENT`, `DUPLICATE_CONFLICT` or `MESSAGE_ID_CONFLICT`. This is derived from the canonical SQLite transport and reading rows and was not known to the communicator when it selected the ACK.

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

For a packet that may receive an ACK, capacity for the complete profile is reserved before TX with the selected ACK and exact ACK frame already fixed. After the radio has been returned to RX or bounded recovery has been attempted, the communicator fills `ack_tx_result` and `T4` through `T6`, freezes the profile and publishes it against the reservation. Persistence therefore inserts one complete row. A packet that cannot receive an ACK may instead be returned to RX first and then admitted once as a complete profile.

### BUSY metrics

Collect at least:

- total BUSY wait time per handled radio event;
- maximum single BUSY wait;
- number of BUSY waits;
- BUSY timeout count;
- SX1262 command associated with a timeout, when known.

Do not log every tight polling iteration as a separate entity.

### Persistence metrics

The pilot does not persist one row per batch or state operation. Persistence
instead maintains cumulative per-instance counters for transaction attempts,
commits and failures; committed entities and exact encoded bytes; total and
maximum commit duration; quarantine successes and failures; admission-state
transitions; and WAL-checkpoint attempts, successes and failures. Each
`ReceiverHealthV1` snapshots these aggregates.

The per-packet profile separately records queue occupancy when admitting a
measurement candidate. Together, queue occupancy and the health aggregates are
enough to determine whether persistence is falling behind without adding
`persistence_batches` or `receiver_state_operations` tables.

### ReceiverHealth

`ReceiverHealth` is a periodic application-aware snapshot. It is not sampled per packet and is not part of ACK acceptance. The initial pilot interval is one minute and remains configurable.

The communicator drives the interval because only that thread can take a coherent snapshot of its live state. Once per interval, and once immediately after successful communicator initialization, it creates an immutable `ReceiverHealthRequest` and attempts a nonblocking enqueue to `PersistQueue`. If the health deadline coincides with DIO1 or another radio deadline, the communicator completes the deadline-bound radio work before sampling health. This event-loop scheduling rule does not give the resulting queue object different FIFO importance after publication.

The request contains only communicator-owned observations:

- `receiver_instance_id`;
- a monotonically increasing per-instance `health_sequence`;
- `communicator_sampled_at_monotonic_us`;
- current radio state;
- cumulative exceptional radio-recovery attempt, success and failure counts;
- bounded recovery counts by reason when practical;
- current `system_time_quality` and `rtc_health`;
- cumulative time-quality and RTC-health transition counts and their last transition times;
- cumulative ordinary-admission rejections by observed `PersistenceAdmissionState` value; and
- cumulative failed `ReceiverHealthRequest` enqueue attempts.

The request sequence and communicator sampling time are the communicator heartbeat: they prove that the communicator event loop reached the periodic health task. No separate high-frequency heartbeat is required. `health_sequence` advances before every enqueue attempt, so a gap in persisted sequence values exposes a missed or failed sample. The cumulative enqueue-failure count distinguishes known queue-pressure loss once a later request succeeds.

Radio recovery means an exceptional transition into `RECOVERING` after conditions such as a BUSY timeout, SPI failure, unexpected IRQ, missing or uncertain TX completion, or failed attempt to restore RX. A confirmed radio TX-timeout IRQ followed by successful RX re-arming, and the normal `SetRx` operation after packet handling or ACK transmission, are not counted as recovery. Recovery counters are cumulative within one `receiver_instance_id`; interval counts are derived during analysis.

When the persistence thread dequeues a request, it does not mutate the queued
object. It samples its own and the Pi's current observations and constructs a
new complete `ReceiverHealthV1` row. In addition to the request fields, its
mandatory persistence-owned fields are:

- `linux_boot_id` and `persistence_sampled_at_monotonic_us`;
- current persistence-admission generation, state and transition time;
- cumulative admission transitions indexed by every
  `PersistenceAdmissionState`, including incompatible schema;
- cumulative durable-quarantine successes and failures;
- cumulative batch transaction attempts, commits and failures;
- cumulative entities and exact encoded bytes committed;
- cumulative and maximum batch-commit duration; and
- cumulative WAL-checkpoint attempts, successes and failures.

The optional bounded host observations are Linux one-minute load multiplied by
1,000, CPU temperature in milli-degrees Celsius, available memory, available
bytes on the SQLite filesystem, SQLite database and WAL sizes, and NTP offset
in microseconds. The exact encodings, sign convention and absence rules are
defined in [`INTERFACE.md`](INTERFACE.md).

Enrichment occurs once. If the SQLite transaction fails, the completed row and both sampling timestamps remain stable for retry; the persistence thread must not resample host fields and silently change the logical health record.

Unavailable host or time-service observations are represented explicitly as absent; they do not prevent the rest of the health row from being persisted. NTP telemetry is observational and must not independently change communicator-owned `system_time_quality` or clock policy.

`communicator_sampled_at_monotonic_us` and `persistence_sampled_at_monotonic_us` intentionally describe different moments. Communicator-owned fields belong to the former; persistence and Pi fields belong to the latter. Their difference measures how long the health request waited before enrichment and prevents a delayed request from appearing to be one simultaneous snapshot.

`ReceiverHealthRequest` has the same FIFO queue importance as every other published object. Its nonblocking admission may fail when persistence admission or queue capacity is unavailable, and an admitted health request consumes capacity that may cause a later measurement/profile reservation to fail and select `ACK_RETRY_LATER_DOWNLINK`. The pilot accepts this consequence instead of implementing prioritization or eviction. Once admitted, the request remains queue-owned until commit under the normal batch-ownership rule. The health sequence and cumulative enqueue-failure count make admission gaps visible. The existence of a committed health row proves that the persistence thread reached that request; an in-process health record cannot prove liveness while the persistence thread itself is stalled.

## Diagnostics

Diagnostics should be structured and include enough context to reconstruct failures.

Useful fields include:

- timestamp;
- subsystem;
- current receiver state;
- event or command being handled;
- IRQ status;
- exception/error category;
- fixed-layout bounded context fields appropriate to the diagnostic category.

The queued diagnostic representation has a fixed size; it contains no arbitrary exception text, dictionary or payload dump. Sensitive material, keys, nonces, and plaintext should follow repository logging rules.

Only the communicator allocates `diagnostic_sequence`, constructs
`DiagnosticV1` and admits it to `PersistQueue`. A synchronous persistence-control
failure can be returned to the communicator and converted into a diagnostic.
The persistence thread never creates a diagnostic identity or inserts a
persistence-created diagnostic row; its asynchronous failures remain visible
through admission state, `ReceiverHealthV1`, quarantine provenance and bounded
service logging.

Unexpected conditions worth diagnosing include:

- impossible state/event combinations;
- DIO1 edge with no expected IRQ;
- BUSY timeout;
- SPI failure;
- malformed radio response;
- RxDone plus radio error flags;
- `PersistQueue` capacity-reservation failure or publication invariant violation;
- missing, corrupt, unsupported or policy-incompatible communicator-state row;
- state-generation mismatch or durable state-commit failure;
- RTC read, validation, write or read-back failure;
- loss or restoration of trusted time;
- exhausted, expired or unavailable airtime reservation;
- TX timeout or missing TxDone;
- persistence admission becoming unavailable because of low space, disk full, corruption or I/O failure;
- persistence admission becoming unavailable because of an incompatible schema;
- failed telemetry admission due to queue capacity.

## Error recovery

Pilot recovery should be bounded and explicit.

`RECOVERING` is entered when normal processing cannot confirm the radio's mode or configuration, particularly when it cannot prove that the radio has returned to `RX_SINGLE`. An ordinary packet or protocol error is not by itself a recovery event.

Examples:

- RX packet rejected:
  - clear relevant IRQs;
  - restore `UPLINK_RX_PROFILE`, then issue and confirm `SetRx`;
  - return directly to `RX_SINGLE` if re-arming succeeds;
  - enter `RECOVERING` only if the IRQ cannot be cleared or RX cannot be confirmed.

- BUSY timeout or SPI failure:
  - create diagnostic;
  - enter `RECOVERING`;
  - never continue under an assumed radio state.

- TX failure:
  - do not retract the already admitted measurement candidate or occurrence profile;
  - retain the airtime charge when `SetTx` started or its effect is uncertain;
  - reclaim tentative allowance only after a definite pre-`SetTx` failure;
  - after a known terminal TX IRQ, restore `UPLINK_RX_PROFILE` and return directly to `RX_SINGLE` only after `SetRx` is confirmed;
  - enter `RECOVERING` when the TX outcome or resulting radio mode or profile is uncertain, or receive-profile restoration or RX re-arming fails;
  - allow a node retry to repeat the normal queue-reservation path and deterministic ACK construction later.

- Communicator-state load or commit failure:
  - keep the preceding durable generation authoritative;
  - classify missing or corrupt history as exhausted and RTC provenance as untrusted;
  - preserve a corrupt singleton before any atomic generation-one replacement;
  - never replace an unsupported-version or policy-mismatch singleton automatically;
  - continue RX and application-queue admission;
  - suppress TX until a valid state and committed reservation are available.

- RTC missing or invalid:
  - update current `rtc_health`;
  - retain network-synchronized system time when available;
  - do not use RTC holdover or overwrite RTC provenance;
  - continue RX while timestamp and TX policies follow current system-time quality.

Authentication failure, malformed or unsupported protocol data, persistence admission failure and airtime-based ACK suppression remain normal packet-processing outcomes. Asynchronously discovered duplicate and identity conflicts are persistence outcomes and do not affect radio state. RX header or CRC errors and confirmed radio TX-timeout IRQs may also return directly to `RX_SINGLE` when IRQ clearing, `UPLINK_RX_PROFILE` restoration and `SetRx` all succeed.

### Radio recovery procedure

On entry to `RECOVERING`, the communicator must suppress new transmissions and record the preceding state, failed operation, command-outcome classification, known IRQ and device-error state, and selected recovery action. If a transmission may have started, its airtime charge remains reserved even when the final radio outcome is unknown.

The first bounded level is soft resynchronization:

1. Wait for BUSY to clear, subject to a bounded deadline.
2. Issue and confirm `SetStandby`.
3. Read and record IRQ and device-error information where possible.
4. Clear the relevant IRQs.
5. Restore the complete protocol-defined `UPLINK_RX_PROFILE` and required IRQ routing.
6. Issue and confirm `SetRx`.

Successful restoration of `UPLINK_RX_PROFILE` followed by confirmation of `SetRx` returns the state to `RX_SINGLE`, unless a new DIO1 event requires an immediate transition to `RX_EVENT_PENDING`.

If soft resynchronization fails, perform bounded hard recovery:

1. Toggle the SX1262 reset signal.
2. Repeat the complete initialization sequence, including the module-specific setup and the protocol-defined `UPLINK_RX_PROFILE`.
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
- the high-priority synchronous persistence control channel and its immutable completion results;
- shutdown signaling;
- generation-numbered read-only publication of `PersistenceAdmissionState`;
- the immutable `linux_boot_id` returned during startup;
- read-only publication of current time quality for observation where required.

Receiver-health counters remain communicator-owned and cross the existing `PersistQueue` only inside immutable `ReceiverHealthRequest` snapshots; they do not create another shared mutable state path.

The persistence thread must not mutate communicator-owned node, message, clock-policy or airtime-policy state. Its communicator-state result reports only whether the exact requested generation and bytes became durable, definitely did not become durable, or require reconciliation.

## Core invariants

- Only the communicator mutates live protocol, time-policy and airtime-policy state.
- Only the persistence thread performs disk I/O and loads receiver configuration.
- Only the persistence thread publishes `PersistenceAdmissionState`; the communicator may accept an application/profile unit only while its current snapshot is `AVAILABLE` and exact queue capacity is reserved.
- SQLite application identity, migration history and generated enum catalogues must be compatible before ordinary persistence admission becomes available.
- The current `receiver_instances` start row is durable before ordinary persistence admission becomes available; only `commit_receiver_clean_stop()` may add its complete clean-stop marker.
- Radio operation begins only after required local storage is usable and the bounded boot-scoped RTC-bootstrap episode has completed, whether successfully or not.
- RTC-to-system-clock bootstrap occurs at most once per `linux_boot_id`; a receiver-process restart never repeats it and receiver startup never waits for network availability.
- `RX_SINGLE` is valid only while the complete protocol-defined `UPLINK_RX_PROFILE` is known to be active and `SetRx` has been confirmed.
- No ACK `SetTx` is issued before `ACK_TX_PROFILE`, including inverted IQ, is installed; after TX, `SetRx` is not issued until `UPLINK_RX_PROFILE`, including normal IQ and boosted RX, has been restored.
- `receiver_instance_id` changes on every receiver-process start; `linux_boot_id` changes only with the Pi's Linux boot.
- An atomic exact-size `PersistQueue` reservation for a prevalidated fixed-layout measurement/`MessageProfiling` unit while persistence admission is available establishes protocol acceptance of that occurrence but not SQLite durability or conflict-free canonical insertion.
- A successful pre-TX reservation must publish exactly one complete immutable pair after the terminal radio outcome; persistence accepts only complete profiling rows.
- No admitted queue unit is silently dropped; an item-specific poison is removed only after its exact fixed-size representation and minimal failure provenance are durably committed to `quarantined_entities`.
- Only the communicator creates `DiagnosticV1`; persistence-derived conflicts use profile classification and canonical rows, while asynchronous persistence failures use health counters, admission state, quarantine provenance and bounded service logs.
- The pilot database uses WAL with `synchronous=FULL`, retains all pilot records automatically, and closes all new ordinary `PersistQueue` admission on low space, disk full, corruption or persistence I/O failure.
- A communicator-state generation is usable only after durable acknowledgement or explicit startup reconciliation.
- `communicator_state` has at most one generation-bearing row; generation zero is conservative runtime state and is never stored.
- Missing, corrupt or unsupported state never becomes an empty airtime ledger or trusted RTC provenance.
- Current `system_time_quality` and `rtc_health` are observed again on every receiver startup; persisted observations are diagnostic only.
- Monotonic time controls live deadlines, event intervals and airtime aging. Wall-clock adjustment never expires live airtime early.
- Known bucket charges, unresolved reservations and the complete active reservation must fit the configured continuous-window budget before a new reservation is committed.
- No TX occurs without an acknowledged active reservation owned by the current receiver instance.
- A reservation from an earlier receiver instance is fully charged, unspendable and retained until conservative expiration.
- A started or uncertain `SetTx` consumes allowance. Only a definite pre-`SetTx` failure permits reclamation.
- A pending or unknown reservation-settlement outcome suppresses TX.
- Airtime suppression never reverses acceptance of an admitted occurrence.
- An untrusted receiver UTC never becomes an immutable direct timestamp anchor.

## Deferred work

The following are explicitly deferred from the first pilot:

- durable queue or write-ahead journal before ACK;
- watchdog and health-management subsystem;
- sophisticated node scheduling;
- multi-receiver coordination;
- a kernel SX1262 driver;
- optimization from Python to C;
- complex BUSY edge handling;
- durable configuration management.

Known consequences:

- any receiver-process restart loses unpersisted queue contents;
- `linux_boot_id` distinguishes a Pi reboot from a receiver-only restart, while a new `receiver_instance_id` exposes both;
- a successful ACK does not guarantee durable storage.

## Testing

At minimum, add tests or simulations for:

- new valid message;
- identical retry after successful ACK;
- identical retry after failed ACK transmission;
- same `message_id`, same `sample_id` and same exact frame classified as `RETRANSMISSION`;
- same `message_id`, same `sample_id` and different frame classified as `DUPLICATE_CONFLICT`;
- same `message_id` and different `sample_id` classified as `MESSAGE_ID_CONFLICT`;
- conflict classifications retain canonical rows and profile evidence without creating a persistence-owned diagnostic;
- same `sample_id` in distinct current/backlog transport messages with matching contents;
- same `sample_id` in distinct transport messages with conflicting contents;
- deterministic ACK reconstruction from the same uplink and outcome without cached receiver history;
- nonce construction from `node_id || message_id || domain`, never `sample_id`;
- unauthenticated packet;
- authenticated malformed packet;
- protocol validation-order cases select the exact processing result and response policy defined by the protocol;
- an authenticated downlink ACK domain reaching the direction check is classified as `WRONG_DIRECTION` and never produces a response, including when its profile cannot be admitted;
- initialization and both recovery levels install the exact protocol PHY profile before entering `RX_SINGLE`;
- the ACK path transitions from normal-IQ boosted RX to inverted-IQ TX and restores normal-IQ boosted RX before `SetRx`;
- failures and uncertain outcomes during either radio-profile transition cannot leave `RX_SINGLE` asserted under a partial or inverted-IQ configuration;
- queue full and later recovery;
- fixed-size entity construction rejects over-capacity fields before reservation and ACK selection;
- exact pre-TX pair reservation, fixed-width post-terminal field completion and publication without allocation or serialization;
- exception finalization as `UNKNOWN_INTERRUPTED` without a partial SQLite row;
- BUSY timeout;
- unexpected IRQ combination;
- TxDone missing or delayed;
- SQLite startup enforces WAL and `synchronous=FULL`, including failure to establish either setting;
- migration filename/digest history, `application_id`, `user_version` and generated enum catalogues are validated before admission;
- a newer, gapped or otherwise incompatible schema publishes `UNAVAILABLE_INCOMPATIBLE_SCHEMA` without modifying the database;
- a database bound to a different `group_id` is unavailable and never imports the configured master key;
- every queue-bound `u64` at `INT64_MAX` persists successfully and a larger value fails as an interface invariant;
- implementation benchmark compares `FULL` and `NORMAL` with realistic Pi batches and checkpoints without changing the pilot default;
- committed transactions recover after process crash and simulated power interruption under WAL/`FULL` assumptions;
- low-space and disk-full failures roll back, retain queue ownership, close admission and recover with bounded backoff;
- detected database corruption preserves the database, WAL and shared-memory files, closes admission and requires explicit recovery;
- an item-specific failure must reproduce in isolation before the exact complete unit is durably inserted into `quarantined_entities` and later valid units proceed;
- `MeasurementProfileUnitV1` is never split during poison isolation or quarantine;
- an ambiguous quarantine commit is reconciled by matching the complete frozen quarantine row;
- quarantine failure retains the poisoned entity, publishes persistence unavailability and closes new admission;
- no automatic retention deletion during active pilot collection and low-water admission closure before exhaustion;
- receiver-process restart within one Linux boot and documented state loss;
- automatic receiver-process restart within one Linux boot creates a new `receiver_instance_id` without repeating RTC-to-system-clock bootstrap;
- Pi reboot changing both identity fields;
- unavailable or read-only required storage prevents entry into radio operation;
- missing, invalid or unresponsive RTC completes bootstrap within its deadline and permits offline startup as `UNTRUSTED`;
- receiver lifecycle start, successful bounded queue drain and clean-stop marker;
- receiver-instance ordinal ordering across clean restart, crash restart and Pi reboot;
- repeated identical `commit_receiver_clean_stop()` is idempotent, while a conflicting marker or unmet precondition is rejected;
- an unknown clean-stop commit outcome is reconciled by repeating the exact request;
- controlled-shutdown queue-drain, airtime-state or marker-commit failure exits without a clean-stop marker and leaves conservative recovery state;
- periodic `ReceiverHealthRequest` enrichment with distinct communicator and persistence sampling times;
- equal FIFO treatment of health, diagnostic, profiling and measurement units;
- health-request capacity consumption causing a later measurement/profile reservation to fail without eviction or reordering;
- timestamp fields missing on partial/error paths;
- unknown airtime history becomes known-empty only after a complete guarded no-TX aging interval, and a process restart restarts that interval;
- missing and corrupt communicator-state rows produce conservative generation zero;
- every observed invalid communicator-state row is preserved and generation-one replacement commits atomically;
- unsupported-version and policy-mismatch communicator-state rows are never overwritten automatically;
- missing singleton creation, ordinary next-generation commit, exact idempotent replay, generation conflict, stale generation and generation gap;
- an unknown communicator-state commit outcome is reconciled by loading the exact installed generation and bytes;
- durable state generation success, reported failure and unknown completion;
- separate persistence-control servicing can precede an ordinary batch without reordering FIFO queue units or starving ordinary persistence permanently;
- current `NETWORK_SYNCED` time with a missing RTC;
- valid offline `RTC_HOLDOVER` without waiting for network availability, and rejected stale, invalid or unproven RTC values;
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

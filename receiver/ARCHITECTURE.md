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

The process creates `receiver_instance_id` before normal radio operation and provides it read-only to both threads. The persistence thread reads and validates `linux_boot_id` and returns it with the successful startup configuration result. It stores the mapping exactly once in `receiver_instances` before ordinary persistence admission becomes available. Every queue entity and every other receiver-instance-scoped persisted row carries only `receiver_instance_id`; persistence and analysis obtain its Linux boot by joining the immutable lifecycle row. Per-instance occurrence sequences, health sequences, radio-recovery, chrony-step, RTC-write/verification and other process-local counters reset when `receiver_instance_id` changes.

Linux monotonic timestamps are meaningful only together with `linux_boot_id`. Receiver entities carry `receiver_instance_id`, whose durable lifecycle row supplies that boot scope while also keeping a process restart within one Linux boot visible.

The persistence thread inserts the current `receiver_instances` row after
SQLite startup validation and before ordinary persistence admission becomes
available. A database-local monotonic `instance_ordinal` supplies lifecycle
order; `receiver_instance_id` remains the public identity referenced by other
tables. The durable start row is also an implicit ordinary clock-correlation
boundary at `started_at_monotonic_us`: it closes the preceding receiver
instance's correlation segment, and observations from one instance are never
assigned to events from another. Unlike an explicit step-discontinuity
boundary, the first later trusted observation from the new instance may be
extrapolated backward to its start when no explicit step boundary intervenes.
Start identity and monotonic time are immutable; optional start UTC may be
derived during analysis from that same-instance observation timeline without
mutating the lifecycle row. During controlled shutdown, the
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
fail-conservative across process restart, Pi reboot and power loss.

The exact canonical encoding, SQL envelope, generation rules and result types
are defined in [`INTERFACE.md`](INTERFACE.md). A commit is acknowledged only
after the exact new generation is durably installed. State-row classification
uses the interface's exclusive validation order: SQL envelope and digest,
digest-protected format version, complete supported-version structure, then
deployment policy. A malformed, digest-invalid, unsupported, policy-incompatible
or otherwise unverifiable row is not partially recovered.

At startup, the persistence thread loads and validates the singleton before the
communicator is allowed to transmit. A missing or corrupt row means:

```text
airtime history = unavailable until a synthetic worst-case ledger commits
RTC provenance  = UNTRUSTED
```

It must not be interpreted as an empty airtime ledger or as proof that the RTC
is correct. Once trusted UTC is available, missing or corrupt state is replaced
by a generation-one ledger that conservatively places the complete active
airtime budget in the most recent possible minute buckets. That synthetic
ledger must commit before a bucket grant or TX is permitted. When the singleton
relation is corrupt but SQLite itself is healthy, the replacement transaction
first preserves every observed invalid row in
`quarantined_communicator_states`; preservation and replacement commit
atomically.

An unsupported-version or airtime-policy-mismatch row is not decoded or
reinterpreted. Starting only after the transmitter is known unable to transmit,
the current process suppresses every TX for the active policy's complete
rolling window using the conservative minimum-wait monotonic conversion. A
process restart restarts that wait. After the wait and once trusted UTC is
available, persistence archives the exact rejected singleton and atomically
installs a generation-one state with an empty ledger under current policy.
Whole-database corruption follows the separate database-corruption policy and
is not eligible for this recovery.

For overlapping apparent defects, the earlier validation stage is authoritative.
A bad envelope or digest is `CORRUPT` even if the bytes resemble an unsupported
version. A digest-valid unknown version is `UNSUPPORTED_VERSION` and is not
decoded using a known layout. For a supported version, every canonical and
structural invariant is validated before policy comparison, so structural
failure is `CORRUPT` rather than `POLICY_MISMATCH`.

For the 36-second budget and eight-second bucket limit, missing or corrupt state
is synthesized as five chronological bucket charges of
`[4, 8, 8, 8, 8]` seconds. The newest possible bucket end is placed one complete
bucket width after the trusted recovery snapshot, conservatively covering an
unknown preceding bucket phase; older synthetic ends are spaced one bucket
width apart. The same checked construction generalizes to an oldest remainder
of `budget mod bucket limit` followed by `floor(budget / bucket limit)` full
buckets. This supplies automatic conservative recovery without pretending that
a missing row means a new installation.

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

- Linux `CLOCK_MONOTONIC` supplies every receiver event timestamp and drives
  GPIO-event timing, deadlines, latency metrics and the live airtime ledger.
  It never steps backwards, but Linux applies chrony's incremental frequency
  corrections to it, so its rate is not assumed to equal physical elapsed
  time exactly.
- Linux system UTC is sampled only while constructing a bounded
  `NETWORK_SYNCED` monotonic/UTC `ClockObservationV1`; analysis uses those
  observations to derive UTC for events from the same receiver instance within
  the same Linux boot without mutating event rows.
- The DS3231 preserves UTC while the Pi is unpowered and seeds the system clock
  through the deployment's one-per-Linux-boot RTC-bootstrap stage. The
  communicator also reads it at a bounded runtime cadence to construct direct
  `RTC_HOLDOVER` observations and to verify network-time writes. It is never
  read per packet and is not used for elapsed-time measurements.

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
- a direct current RTC observation consistent with the last verified
  synchronization; and
- verification, RTC-age/drift and read/whole-second uncertainty that leave a
  positive trusted interval under the configured receiver UTC budget.

The last observed quality and health may be recorded for diagnosis, but they
are not restored as authoritative current-instance observations. On every
receiver startup the receiver probes the RTC and independently establishes
current time quality. The communicator owns both live values and one runtime
`clock_state_generation`. It advances that generation when a completed chrony
tracking result is processed, on any other quality or RTC-health transition,
and before every intentional clock step. One atomic update advances it once;
a purely periodic observation with no new policy input may share a generation.
Every time-derived operation captures and later rechecks this generation, so
no second trust-snapshot identity is required.

### Boot and synchronization lifecycle

On an offline Pi boot, the deployment RTC-bootstrap stage may provisionally
copy a hardware-valid DS3231 value into Linux system UTC. The receiver enters
`RTC_HOLDOVER` only after it independently validates durable RTC provenance,
current RTC health and a new direct RTC observation under the uncertainty
policy. Linux `CLOCK_REALTIME` is not the holdover observation source and the
bootstrap copy is not itself proof of trust. With no valid provenance, time
remains `UNTRUSTED` even if the bootstrap stage copied a provisional RTC value.
A missing or invalid RTC is recorded independently in `rtc_health`.

When the time-synchronization service confirms network synchronization, the
trust transition is ordered:

1. Establish that the chrony status and the kernel clock state meet the
   configured network-synchronization bounds.
2. Process that tracking result as one atomic time-policy update, advance
   `clock_state_generation`, transition the live quality to `NETWORK_SYNCED`
   and publish the corresponding trusted
   `ClockObservationV1`.

Entering `NETWORK_SYNCED` does not by itself authorize an RTC write. On the
first stable network result after startup or a clock-step episode, and then at
the configured three-hour RTC-refresh period, the communicator starts a refresh
only when the fresh observation's complete network error is at or below the
separate five-second RTC-write threshold. The periodic online clock-observation
and RTC-refresh tasks share this scheduling episode when both are due. The
refresh is ordered:

1. Capture `clock_state_generation` as the episode generation and derive the
   target whole-second UTC from the fresh trusted observation plus bounded
   monotonic elapsed time.
2. Write that value to the DS3231.
3. Read back and verify the RTC update. The returned whole-second midpoint must
   differ from network UTC advanced to the same read midpoint by no more than
   the fixed one-second `time_sampling_margin_us`; the actual difference and
   the checked read uncertainty are charged to provenance.
4. Recheck the episode generation, current `NETWORK_SYNCED` quality and the
   stricter source-error condition, including that the supporting tracking
   result has not reached its next required poll deadline.
5. Durably commit the verification uncertainty, RTC drift bound and new RTC
   provenance through `commit_communicator_state()`.
6. Only after acknowledgement may a later Pi boot or runtime holdover
   observation use that update for `RTC_HOLDOVER`.

A clock-step command is not an RTC-refresh trigger: the communicator waits for
the first qualifying post-step `NETWORK_SYNCED` observation. A reset between
the RTC update and the durable provenance commit, or a generation change at
any fallible boundary, leaves a good RTC conservatively classified as
untrusted. The reverse ordering is forbidden because it could make a later Pi
boot trust an RTC that was never updated. Offline operation neither writes the
RTC from Linux nor repeatedly writes Linux system time from the RTC. Clean
shutdown has no special clock-persistence role, and an unsynchronized Pi must
never overwrite a credible RTC.

### Clock observations and event timestamps

All receiver entities record events only with `CLOCK_MONOTONIC`. Each entity's
`receiver_instance_id` resolves through `receiver_instances` to the
`linux_boot_id` that scopes its monotonic value. Monotonic time remains
comparable across receiver-process restarts in one Linux boot and is never
compared across different boots. The host must not suspend while the receiver
service is active because `CLOCK_MONOTONIC` excludes suspended time.

The communicator periodically creates a fixed-size `ClockObservationV1` that
contains:

- the source `receiver_instance_id`;
- a per-instance `observation_sequence` and `clock_state_generation`;
- one bounded monotonic/UTC correlation;
- the current `system_time_quality`; and
- the current `rtc_health`.

The communicator creates an observation after initial time and RTC state have
been established, on every quality or RTC-health transition, immediately
before every intentional clock step, and at a bounded periodic interval while
running. Three hours is the pilot's ordinary online period cap. Direct
`RTC_HOLDOVER` observations have a one-hour period cap. For either trusted
source, the communicator shortens the next interval when required by the
remaining UTC-error budget.
`UNTRUSTED` observations intentionally contain no UTC. A trusted observation
contains UTC only when quality is `RTC_HOLDOVER` or `NETWORK_SYNCED`.

The period caps limit the distance to a persisted correlation for recovery and
later analysis; they are not the proof that UTC is accurate. A
`NETWORK_SYNCED` observation obtains its initial error from the complete chrony
network-error calculation below. An `RTC_HOLDOVER` observation obtains it from
the durable verification uncertainty, RTC drift accumulated before the read,
and the direct read's bracket/whole-second uncertainty. After either
observation, only the configured monotonic rate bound grows the error used to
correlate a later or earlier event; RTC drift does not continue after the RTC
read because that RTC is no longer used in the extrapolation.

The pilot uses one shared `time_sampling_margin_us = 1_000_000`. Network
observations charge it for bounded tracking-response age and local sampling
effects. Direct RTC reads charge the same fixed margin in addition to the
500,000 us whole-second term and the conservatively converted measured
half-bracket. A verified RTC refresh stores the exact sum of its advanced
network error, actual accepted read-back difference and direct read uncertainty;
there is no separate unnamed device margin or read-back tolerance.

The communicator derives each trusted observation's maximum safe event
distance from the remaining receiver UTC budget and the monotonic rate bound,
using the checked formula in
[`INTERFACE.md`](INTERFACE.md#utc-error-growth-and-observation-deadlines). It
schedules a replacement no later than that deadline and the applicable period
cap. If the initial uncertainty exhausts the budget, or if the replacement
cannot be obtained and published in time, live quality becomes `UNTRUSTED` at
the calculated boundary and the pending-boundary FIFO rule applies. That expiry
does not itself authorize a time step: only a fresh valid chrony result strictly
above the separate step threshold may enter the ordered step procedure. The RTC
provenance age calculation separately uses the drift bound stored with that
provenance. Re-reading the RTC does not reset this age, and it is not combined
with monotonic drift over one fictitious common interval.

For the normative pilot example, 4,000,000 us of stored RTC verification
uncertainty and an illustrative zero-width read bracket produce 5,500,000 us of
initial direct-observation error. With the stored 10 ppm RTC drift bound, the
direct observation itself remains below the 40-second budget for just under
39.93 days. The 3,700 ppm monotonic bound adds 13,369,468 us over one observed
hour, so the ordinary one-hour RTC-read cadence is sufficient only until about
24.46 days; after that, replacement reads are scheduled progressively earlier.
Any nonzero read bracket shortens both limits. Only a verified network-to-RTC
refresh resets the provenance age.

For the pilot timestamp contract, the receiver UTC budget must not exceed the
same 40-second ceiling. The protocol's direct-anchor midpoint contributes less
than 15 additional seconds under its 30-second radio-cycle bound, leaving
approximately five seconds inside the one-minute target for other direct-anchor
error. Extrapolation across many sleep cycles remains explicitly best-effort
and is not justified by this direct-anchor budget.

A `NETWORK_SYNCED` observation is sampled without a receiver/daemon lock:

```text
generation_before = clock_state_generation
M_before = CLOCK_MONOTONIC
timex = adjtimex(modes = 0)
M_after = CLOCK_MONOTONIC
generation_after = clock_state_generation
```

The sample is accepted only if both generation reads are equal, checked
monotonic ordering holds from the supporting tracking-query start through
`M_after`, that complete span is no greater than the fixed one-second
`time_sampling_margin_us`, and `adjtimex()` reports no unexpected
clock-interference condition. A `NETWORK_SYNCED` observation also requires that
fresh acceptable `ChronyTrackingResult` obtained by the communicator before
this bracket. The observation monotonic value is the bracket midpoint and its
UTC is the system time returned by the read-only `adjtimex()` call. A bounded
number of failed sampling attempts leaves the observation pending; it never
guesses a correlation.

An `RTC_HOLDOVER` observation uses the same generation-before/after rule around
one bounded `Ds3231Control.read_time()` call. Its monotonic value is the read
bracket midpoint, while its UTC value is the midpoint of the returned whole
UTC second. The uncertainty calculation charges exactly half a second for that
representation, the conservatively converted measured half-bracket and the
fixed `time_sampling_margin_us`; the complete checked read bracket must also be
no greater than that one-second constant. It additionally charges the stored
verification uncertainty and RTC drift only from the durable network
verification to this read. Linux `CLOCK_REALTIME` is not sampled on this path.

The generation checks prevent an in-process time-quality ABA race, while the
single `adjtimex()` result supplies system time and kernel clock metadata from
one syscall. It cannot lock chronyd and does not try to: the deployment
prohibits automatic clock steps, and chronyd may continue bounded slewing while
the receiver samples. Linux applies that slew to `CLOCK_REALTIME` and
`CLOCK_MONOTONIC` together. Their offset therefore remains stable through a
slew, so same-instance correlation remains valid; only a clock step changes
the offset and requires the ordered observation boundary described here. The
pilot deliberately does not introduce `CLOCK_MONOTONIC_RAW`, because it would
require an additional continuously estimated correlation to system UTC.

The same simple `clock_state_generation` covers longer time-derived episodes.
All communicator time-policy tasks are serialized, but a blocking or yielding
chrony, RTC or persistence operation may let a newer task change the inputs.
The operation therefore captures the generation before deriving UTC and
rechecks it after each such call and immediately before publishing an
observation or committing RTC provenance. A mismatch caused by another task
discards the trusted result. After a successful check, the episode may apply
its own returned RTC-health/quality transition, advance the generation once
and continue with that new episode generation. A lock, if needed by a concrete
multi-threaded implementation, protects only the short in-RAM read/update of
this generation; it is never held across chrony, device or persistence I/O and
cannot lock chronyd itself.

Linux `adjtimex()` synchronization status is deliberately not the authority
for `NETWORK_SYNCED`. Chronyd clears the kernel `STA_UNSYNC` flag only when its
`rtcsync` mode is enabled, and that mode would also let the kernel copy system
time to the RTC every 11 minutes. Because the receiver must remain the sole
DS3231 writer, `rtcsync` stays disabled and `adjtimex()` may report
`TIME_ERROR` even while chrony tracking is synchronized. The communicator uses
fresh chrony tracking for network quality and treats the expected
`STA_UNSYNC`/`TIME_ERROR` pair as compatible with this deployment.

`ClockObservationV1` has the same FIFO importance as every other queue entity.
If a quality transition must be recorded while its queue admission is
temporarily impossible, the communicator changes unsafe live quality to
`UNTRUSTED` immediately, retains the transition boundary in RAM and attempts
that observation before any later ordinary queue admission. An intentional
clock step may be submitted once its complete `UNTRUSTED` observation carrying
`STEP_DISCONTINUITY_BOUNDARY` has been successfully published to the global
FIFO. Later ordinary publication may then continue: FIFO order guarantees that
no later ordinary FIFO entity becomes durable ahead of the boundary. The boundary is
non-quarantinable and non-bypassable, so an isolated persistence rejection
retains it and all following work, closes admission as incompatible and cannot
silently create a durable correlation gap.

Successful FIFO publication is intentionally not a durability acknowledgement.
If the process exits before the boundary commits, all later volatile FIFO work
from that process is lost with it. Before a replacement process can publish
ordinary work, persistence durably inserts its `receiver_instances` start row;
that implicit correlation boundary prevents any new-instance observation from
being extrapolated into the preceding instance. If the explicit boundary did
commit, it remains the stronger permanent step gap. These ordering properties
remove the need for a clock-observation durability callback or snapshot.

Analysis resolves the event and observation instance IDs through
`receiver_instances`, then derives an event's UTC from immutable trusted
observations carrying the same `receiver_instance_id`:

```text
event_utc_us = observation.sampled_at_utc_us
             + event_monotonic_us
             - observation.sampled_at_monotonic_us
```

This algebraic correlation remains continuous through bounded chrony slew, but
its UTC-error bound grows with the absolute monotonic distance from the source
observation. It does not claim that monotonic microseconds always equal
physical elapsed microseconds. Every
safety-sensitive physical duration is converted centrally using the
configured conservative monotonic elapsed-rate bound. Minimum waits, including
rolling-airtime retention and retry backoff, are lengthened; maximum lifetimes,
including radio/time-service deadlines and the remaining life of an active
airtime bucket grant, are shortened. Profiling and periodic-observation intervals are
observational and need no such conversion unless their individual contract
says otherwise. The exact conversion and pilot defaults are normative in
[`INTERFACE.md`](INTERFACE.md#elapsed-duration-policy).

Each durable receiver-instance start is an implicit ordinary boundary. Analysis
uses only observations whose `receiver_instance_id` equals the event's and
orders them by monotonic value and observation sequence. An ordinary
`UNTRUSTED` observation closes the preceding trusted segment. For an event in
an open trusted segment, analysis selects the latest preceding trusted
observation. For an event between its instance start or an ordinary
`UNTRUSTED` boundary and the first later trusted observation, it may select that
later same-instance observation only when no step-discontinuity boundary lies
between them.

A `STEP_DISCONTINUITY_BOUNDARY` is different. Events in the half-open interval
from that boundary through, but excluding, the first later trusted observation
are permanently ineligible for derived UTC. The later trusted observation
starts the new correlation segment; it never extrapolates backward across the
boundary, including to a pre-boundary event that lacks an eligible preceding
trusted observation. A boundary at the same monotonic microsecond is treated
as preceding the event. If the step was never submitted or its outcome is
unknown, retaining the gap is the deliberately conservative result. Stored
receiver-event rows remain monotonic-only and immutable; UTC and
source-observation identities are analysis results, not persistence enrichment.

The source `receiver_instance_id` is a correlation boundary as well as
provenance. No observation can correlate an event from another receiver
instance, even when both instances map to the same `linux_boot_id`, and no
monotonic value is compared across Linux boots. This deliberate loss of
cross-process backfill is what makes a durable process-start row close a step
boundary that was published but still volatile when the preceding process
exited.

The kernel-recorded DIO1 `RX_DONE` edge is therefore the only reception-time
input carried by `MessageProfilingV1`. Occurrence quality and RTC health are
not copied into each profile; they remain represented by the clock-observation
timeline and periodic receiver health. The same model applies to diagnostics
and receiver lifecycle events during analysis. Durable airtime state still contains UTC
expirations because it must survive Linux reboot, but the communicator derives
those values from its latest live trusted correlation rather than reading
`CLOCK_REALTIME` for the state entity.

An explicit realtime step cannot jump a monotonic deadline or bucket. Bounded
slew can change how quickly monotonic time advances relative to physical time,
so the conservative duration conversions prevent it from expiring a minimum
wait early or extending a maximum lifetime too long. When UTC is untrusted,
the receiver still records the frame, both identity fields and monotonic
reception time. Analysis produces UTC only when a trustworthy same-Linux-boot
observation segment permits it.

### Logical reading timestamps

Analysis owns logical reading-timestamp reconstruction from immutable SQLite
history. It applies the direct-anchor and extrapolation rules in the protocol,
including:

- using the earliest anchor-eligible accepted current-reading occurrence with
  analysis-derived UTC as a direct anchor;
- requiring `run_ms + Tair <= 30,000 ms` and a non-conflict persistence
  classification for direct-anchor eligibility;
- preserving reception time at `RX_DONE`, not database-write time;
- crossing between consecutive samples only when the required deep-sleep, previous-cycle-metrics and identity-lifetime conditions hold;
- storing `timestamp_source`, `anchor_sample_id` and the trusted clock
  observation behind the direct anchor when analysis materializes output; and
- never replacing an estimated timestamp after it has been written to that
  output.

The receiver database therefore stores reception records and measurements
without materialized UTC or logical reading timestamps.

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
  -> persistence thread validates storage, configuration, SQLite application
     identity, database schema metadata and durable communicator state
  -> persistence thread inserts receiver_instances start row
  -> probe RTC and local time-synchronization status
  -> establish NETWORK_SYNCED, RTC_HOLDOVER or UNTRUSTED
  -> publish the initial ClockObservationV1 before later ordinary queue work
  -> reconcile durable airtime state and obtain a durable bucket grant when possible
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
state generation and bucket grant are durably established. An incompatible
database schema prevents ordinary persistence admission and therefore prevents
accepted ACK outcomes.

The DS3231 itself is not booted by Linux; it keeps counting from Pi power or its backup battery. RTC bootstrap means one RTC-to-Linux-system-clock bootstrap episode per `linux_boot_id`. The episode may make a small configured number of local read attempts, but it has a finite total deadline. It checks hardware-level validity, including any available invalid-time or oscillator-stop indication, before provisionally setting Linux UTC. It then terminates with success, missing, invalid or I/O/timeout status. Exact retry and deadline values are deployment parameters.

Completion of the RTC-bootstrap episode is an ordering prerequisite; success is not. A failed episode must not wait for a network or block receiver startup indefinitely. The receiver starts, reports the observed `rtc_health`, keeps `system_time_quality = UNTRUSTED` unless another trusted source is available, and continues RX under the existing fail-conservative timestamp and TX policies.

The bootstrap episode runs once per Linux boot, not once per receiver process. An automatic receiver restart under the same `linux_boot_id` must not copy the RTC into the system clock again. It probes current RTC health and time quality, creates a new `receiver_instance_id`, reconciles durable communicator state and proceeds from the clocks already running in that Linux boot. The bootstrap component does not read or write receiver configuration or SQLite; RTC provenance remains owned by the communicator-state model.

The RTC bootstrap must complete before network time discipline is allowed to establish synchronization; alternatively, the bootstrap must detect already-synchronized system time and skip the RTC-to-system copy. This prevents a late RTC bootstrap from overwriting NTP-disciplined time. The receiver service itself must not depend on a network-online target. The local time-synchronization service may establish `NETWORK_SYNCED` asynchronously after the local bootstrap prerequisite, and the receiver does not wait for it. Offline startup therefore produces either validated `RTC_HOLDOVER` or `UNTRUSTED`, never a wait for NTP.

### Chrony integration

The pilot uses chronyd as the sole normal writer of Linux system UTC. Its
deployment configuration must:

- omit `makestep` and `initstepslew` directives so chronyd never steps the
  clock autonomously;
- use `leapsecmode slew` and, for the pilot, configure
  `maxslewrate 3500`, limiting chrony's phase-correction contribution to
  3,500 ppm;
- omit `rtcsync` and `rtcfile`, leaving DS3231 reads, writes, verification and
  provenance entirely to the receiver lifecycle;
- set `cmdport 0` and use only the local Unix command socket; and
- disable or mask `systemd-timesyncd` and every other competing system-clock or
  RTC writer.

`chronyc tracking` does not expose the configured `maxslewrate`. The receiver
therefore validates the relationship between its declared expected chrony
ceiling and its 3,700 ppm elapsed-rate bound, while installation/deployment
verification checks that the effective chronyd configuration actually contains
the declared `maxslewrate 3500`, `leapsecmode slew` and no automatic-step
directive. A mismatch fails deployment validation; the runtime adapter must not
pretend it discovered that daemon setting from tracking output.

The receiver has no `CAP_SYS_TIME` and never invokes `clock_settime()`. For the
pilot, its service account receives narrowly scoped filesystem permission to
chronyd's local Unix command socket, normally through membership in the
socket-directory group. The receiver invokes chronyc with the explicit socket
path so failure cannot fall back to its localhost UDP command port. It accesses
that socket in normal code only through the fixed `ChronyControl` adapter in
[`INTERFACE.md`](INTERFACE.md#chrony-control-interface). The adapter exposes a
read-only tracking operation and one privileged operation equivalent to
`chronyc makestep`; it accepts no arbitrary chronyc command, server, UTC value
or offset. A future separately privileged local coordinator can implement the
same adapter without changing communicator policy.

This is modularity, not a complete OS privilege boundary: chronyd intentionally
grants broad control to a client that can access its Unix command socket. A
compromised pilot receiver process could bypass the adapter and send another
chrony command. The pilot accepts that local risk to avoid another service. A
future coordinator must run under a separate identity, own the chrony-socket
permission and expose only tracking plus one policy-checked step operation if
command-level least privilege becomes a requirement.

Chronyd does not announce that it wants to step. With automatic stepping
disabled it normally slews every correction, so the communicator alone decides
whether an explicit step is necessary. A fresh successful tracking result is
usable for this decision only when chrony reports a selected, synchronized,
reliable source and the response passes the interface freshness, value and
configured skew bounds. The communicator computes the conservative total
network error:

```text
network_error_bound_us =
    abs(remaining_correction_us)
    + root_distance_us
    + time_sampling_margin_us
```

The remaining correction is the current displacement between chrony's
software NTP clock and Linux system UTC that still has to be removed by slew;
direction does not reduce error, so it is absolute. Root distance is the NTP
source's own uncertainty, conservatively formed from half the root delay plus
root dispersion. The fixed one-second sampling margin covers bounded
tracking-response age, subprocess latency and the observation bracket. A simple
NTP offset omits source and local sampling uncertainty, so every tracking poll
evaluates the complete expression again.

All arithmetic is checked and rounded conservatively. The pilot defaults use
a 35-second network-trust threshold and a 40-second step threshold:

- at or below 35 seconds, a qualifying result may establish or retain
  `NETWORK_SYNCED`;
- above 40 seconds, quality first becomes `UNTRUSTED`, its boundary
  observation is completely published to the global FIFO, and the explicit
  step procedure may run; and
- between those thresholds, including exactly 40 seconds, current quality is
  retained. In particular, an `UNTRUSTED` or `RTC_HOLDOVER` clock does not
  enter `NETWORK_SYNCED` in this hysteresis band, while an already
  `NETWORK_SYNCED` clock is not made to flap.

An unavailable, stale, unsynchronized or otherwise invalid chrony result can
neither establish `NETWORK_SYNCED` nor authorize a step. Loss of the current
source also removes `NETWORK_SYNCED` according to the bounded status-poll
policy; the receiver does not continue claiming network trust from a stale
sample. The 40-second decision applies to the total bound above, not merely to
chrony's remaining correction.

The communicator performs this tracking poll at most one minute after the
previous poll initially, with a short deadline and outside the RX-to-ACK
critical path. It polls earlier when the current observation's calculated UTC
error horizon requires a replacement first. Processing every completed result,
including an unavailable, expired or invalid result, is one atomic time-policy
update and advances `clock_state_generation`. A result may support
`NETWORK_SYNCED` only until the next required poll deadline. Independent UTC
budget expiry can therefore remove `NETWORK_SYNCED` within the 35-to-40-second
hysteresis band without authorizing a step.

RTC writes use a distinct stricter source-error threshold, initially five
seconds. The 35-second threshold may establish network time but cannot
authorize overwriting durable RTC provenance. The stricter condition must hold
when a refresh starts and still hold, with the same
`clock_state_generation`, immediately before provenance commit.

Subject to those rules:

- from `RTC_HOLDOVER`, a plausible correction is slewed and quality becomes
  `NETWORK_SYNCED` only after the configured synchronization bounds hold;
- from `UNTRUSTED`, a large pending correction may be applied by one explicit
  step while quality remains untrusted; and
- an implausible correction observed while `RTC_HOLDOVER` or
  `NETWORK_SYNCED` first causes an `UNTRUSTED` transition and published clock
  boundary; it may be stepped only after that exact boundary has been
  completely published to the global FIFO.

The communicator drives the step without blocking radio reception through this
small runtime-only state machine:

```text
IDLE
  -> STEP_COMMAND_PENDING
  -> WAITING_FOR_STABLE_TIME
  -> IDLE
       or RETRY_BACKOFF -> STEP_COMMAND_PENDING
```

It keeps an operation generation, step start and deadline, next status-poll
time and retry-not-before time. Before leaving `IDLE` it sets quality to
`UNTRUSTED`, advances `clock_state_generation` and publishes a boundary
observation carrying `STEP_DISCONTINUITY_BOUNDARY`. A successful complete FIFO
publication authorizes command submission and allows later ordinary entities
to be published behind the boundary. If publication cannot be established, it
keeps later ordinary admission blocked, stays untrusted, submits no chrony
command and retries the retained boundary. A definite command rejection enters
bounded backoff; the published analysis gap remains conservatively valid even
though no step occurred. Confirmed
submission and an unknown command outcome both enter
`WAITING_FOR_STABLE_TIME`; an unknown result is never retried blindly because
the step may already have happened. The communicator polls status with short
deadlines outside the RX-to-ACK critical path. Once chrony and the kernel meet
the network-entry bounds, including total error at or below 35 seconds, it
publishes the first post-boundary trusted observation, which ends the
analysis-only discontinuity gap, and enters `NETWORK_SYNCED`. Failure or total
deadline expiry retains `UNTRUSTED` and enters bounded backoff.

The receiver uses 3,700 ppm as its conservative bound on the possible rate
difference between `CLOCK_MONOTONIC` and physical elapsed time. This exceeds
chrony's configured 3,500 ppm ceiling to cover integer rounding and residual
frequency behavior. At the receiver bound, three physical hours can differ by
at most 39.96 seconds in monotonic elapsed time; chrony's configured slew
contribution is at most 37.8 seconds over the same interval. These are pilot
defaults, not protocol constants. They may be changed after pilot evidence is
reviewed, but the deployment configuration, receiver validation and duration
conversion policy must be changed together.

This step state is not durable. After a receiver-process restart the
communicator reconstructs policy from current chrony/kernel status, RTC
provenance and fresh observations and does not assume whether a prior
in-process command ran. The new instance's already-durable start row is an
ordinary correlation boundary, so its first same-instance trusted observation
may backfill only to that start and never into the preceding process. It never
trusts an unfinished operation merely because the Linux boot did not change.

The supervisor restarts crashes, uncaught-failure exits and terminal initialization or recovery failures with a nonzero restart delay and rate limiting so a permanent hardware fault cannot create a tight restart loop. A restarted process follows the normal receiver-instance and durable-airtime-bucket rules. Controlled service stop uses a bounded graceful-shutdown interval, but correctness never depends on a clean-stop marker or shutdown-time RTC write.

For systemd, the unit-level contract is:

- enable the receiver service for normal boot;
- order it after the required mounts and the boot-scoped RTC-bootstrap unit, while treating RTC-bootstrap completion rather than success as the prerequisite;
- order chronyd after that same bootstrap completion so network discipline
  cannot race a late RTC-to-system copy;
- give the RTC-bootstrap unit a finite operation timeout and ensure it runs no more than once in one Linux boot;
- do not add a `network-online.target` ordering requirement;
- run the receiver as the configured service user with access only to its data
  directories, required GPIO, SPI and RTC devices, and the single local chrony
  command socket;
- prevent host suspend while the receiver service is active; and
- use restart-on-failure with a nonzero restart delay, rate limiting and a bounded graceful-stop timeout.

Exact unit names, paths, retry counts and timeouts are deployment configuration, but these semantics are not optional or deferred from the pilot.

### Controlled shutdown

Graceful shutdown is a bounded best-effort optimization for an orderly Pi reboot or power-off, `systemctl stop`, and the stop phase of `systemctl restart`. It is not used for correctness and cannot run after `SIGKILL`, process crash or sudden power loss.

On the supervisor's termination signal:

```text
stop admitting new radio events and suppress new TX
  -> finish or conservatively terminate the active radio operation
  -> place the SX1262 in the configured safe shutdown state
  -> settle the current process's active bucket grant when persistence permits;
     otherwise leave its complete durable precharge in the bucket
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
A checkpoint or database-close failure does not invalidate an already durable
clean-stop marker. The supervisor may then finish stopping or restarting the
service. No shutdown-time RTC write is required.

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
  -> require acknowledged spendable airtime bucket grant
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
- bounded runtime DS3231 access through the fixed `Ds3231Control` adapter;
- live time-quality, RTC-health and clock-correlation state;
- live rolling-airtime buckets and the current process's active bucket grant;
- immutable communicator-state snapshot creation and generation tracking;
- timing measurements;
- periodic immutable `ReceiverHealthRequest` creation;
- per-instance health-sequence, time/RTC-transition, chrony-step,
  RTC-write/verification and exceptional radio-recovery counters;
- structured diagnostic creation;
- enqueueing persistence entities.

It must not:

- write to SQLite;
- perform configuration, database or other persistent filesystem I/O;
- perform network requests;
- perform unbounded logging;
- block on ordinary `PersistQueue` persistence.

It may wait for acknowledgement of a high-priority `commit_communicator_state()` call outside the packet-to-ACK critical path. The next bucket should be precharged proactively at the safe boundary. If a packet requires an ACK while no acknowledged bucket allowance is usable, the communicator suppresses that ACK rather than waiting for a state commit.

The persistent-filesystem prohibition does not include bounded access to local
device APIs or fixed local IPC/helper adapters. In particular, the communicator
uses `Ds3231Control` and `ChronyControl` only through the runtime contracts in
[`INTERFACE.md`](INTERFACE.md); it does not open configuration or state files,
use raw I2C alongside a bound kernel RTC driver, or grant itself filesystem
ownership.

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

Known pilot limitation: the receiver provides no recovery path for erased or
rolled-back node counters. Same-credential counter erasure or restoration is
prohibited, and the identity and key must be rotated before that node resumes
transmission. A future protocol revision may add a pre-traffic recovery
handshake in which a fresh challenge authenticates the exchange and the
receiver durably allocates either a never-reused epoch that scopes both
transport and application identities or disjoint ranges for both counters; it
must not resume merely from the greatest received message ID because
transmissions may have been lost before reception and sample IDs may collide
with stored readings.

`sample_id` remains inside the authenticated reading body. It identifies the application reading, supports wake continuity and timestamp reconstruction, and is unavailable until authentication and decoding succeed. It is not part of the CCM nonce and does not identify RF retransmission episodes.

An ACK echoes the authenticated uplink's `message_id`; it does not carry `sample_id`. Its ACK domain selects the downlink nonce domain. ACK construction is deterministic: reconstructing an ACK for the same authenticated uplink and outcome produces the same exact frame without consulting receiver history. Different outcomes use distinct ACK domains and therefore distinct nonces under the same node key and `message_id`.

### Stateless communicator admission

The communicator keeps no `reading_messages_map`, cached ACK or other per-node
acceptance history. Every authenticated, structurally valid reading occurrence
follows the same bounded admission path, including an exact RF retransmission
after a lost ACK and a reading that SQLite has already stored. Duplicate and
conflict classification belongs exclusively to the persistence thread.

This deliberately permits the communicator to acknowledge a candidate that
persistence later classifies as conflicting. Such an ACK means only that the
authenticated and structurally valid occurrence entered the bounded
persistence pipeline. It does not mean that the candidate became canonical for
its sample.

### Acceptance invariant

An authenticated, structurally valid reading occurrence may be marked accepted
only after one atomic `PersistQueue` reservation exclusively owns one entity
slot for the eventual `AuthenticatedReadingCandidateV1`/`MessageProfilingV1`
unit. The reservation counts against the 500-entity queue capacity immediately
but is not visible to the persistence consumer until the communicator publishes
the complete immutable pair. The pair remains one logical persistence unit and
batch selection must not split it across SQLite transactions.

Required ordering:

```text
validate message
  -> construct and validate the stable candidate/profile inputs and candidate success ACK
  -> require persistence admission state AVAILABLE
     -> unavailable: reserve nothing and select RETRY_LATER
  -> atomically reserve one entity slot
     -> capacity failure: reserve nothing and select RETRY_LATER
     -> success: mark occurrence accepted and transmit the selected ACK when permitted
  -> reach a terminal TX/suppression/recovery outcome
  -> construct one complete immutable pair and publish its existing object reference against the reservation
```

Publication uses the existing reservation, performs no new capacity check and cannot fail because of queue pressure. If TX fails or its ACK is lost, a later retransmission repeats this complete admission path. This may publish another `MeasurementProfileUnitV1`, but persistence retains only canonical data and stores a profile for every admitted radio occurrence.

### Retry-later behavior

`ACK_RETRY_LATER_DOWNLINK` means that the receiver did not accept ownership of
this occurrence because durable persistence was currently unavailable or it
could not reserve an entity slot for the complete measurement/profile unit.
When the node retries, the occurrence goes through the same availability and
reservation path again. The communicator neither remembers nor caches
`RETRY_LATER`.

### ACK semantics in the pilot

A successful ACK means:

> The receiver authenticated and validated the message while persistence admission was available, then reserved one exclusive slot in the bounded persistence pipeline for its immutable `AuthenticatedReadingCandidateV1`/`MessageProfilingV1` unit.

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

A bucket is discarded only after its complete interval lies outside the
physical rolling window. `minimum_wait_monotonic_us()` is the normative
conservative conversion in `INTERFACE.md`:

```text
rolling_retention_monotonic_us =
    minimum_wait_monotonic_us(rolling_window_us)

while now_monotonic - oldest_bucket_end >= rolling_retention_monotonic_us:
    total_used -= oldest_bucket_charge
    clear oldest bucket
    advance starting index
    advance oldest bucket timestamp by one bucket width
```

An implementation may bulk-reset a fully expired ring after a long idle interval. It must bounds-check every computed logical and physical index and fail closed if the ring cannot represent the required interval.

The durable bucket expiration identifies the bucket across receiver instances;
persisted monotonic timestamps are never reused. At grant time the communicator
maps the selected bucket end to a shortened maximum-lifetime monotonic deadline
and thereafter uses monotonic time for live spending and aging. A change that
invalidates that correlation freezes the grant until trusted time is
re-established. Restored buckets are reconstructed only from `NETWORK_SYNCED`
UTC or valid `RTC_HOLDOVER`.

#### Durable airtime bucket grants

Each persisted bucket charge is both historical accounting and the write-ahead
ceiling for transmissions that may be lost between state commits. There is no
separate durable reservation list. The pilot defaults are:

```text
rolling window W       = 3,600 seconds
bucket width X         = 60 seconds
bucket charge limit Y  = 8 seconds
total budget B         = 36 seconds
UTC expiration guard G = 1 second
```

Bucket spend intervals are consecutive and non-overlapping. A new process
continues the latest unexpired durable bucket grid; after a fully expired or
empty history it may begin a new grid. A grant ends at the selected bucket
boundary, never one nominal minute after an arbitrary commit, so two grants
cannot authorize more than `Y` in one logical bucket. The durable expiration is
exactly the checked bucket end plus `W` and the configured UTC-expiration guard.

Every persisted charge loaded by another receiver instance is an unspendable
baseline, regardless of whether it represents actual airtime or an unused
precharge. To obtain allowance, the current process computes for the current
logical bucket:

```text
bucket_headroom = Y - persisted_current_bucket_charge
global_headroom = B - sum(all unexpired bucket charges)
grant            = min(bucket_headroom, global_headroom)
```

All arithmetic is checked. The process submits a complete next-generation state
that increases the current bucket by `grant`; only after durable acknowledgement
is that increment spendable. If the latest persisted bucket is still the current
time bucket, it is topped up in place. Otherwise its old charge remains where it
was incurred and a new current bucket is precharged. A charge is never relocated
merely because the receiver process restarted. In particular, the write-ahead
increment belongs to the logical bucket selected by the instance that committed
it; restart does not relabel that increment as charge in the new instance's
current bucket.

Runtime state retains the bucket expiration, unspendable baseline, acknowledged
increment, remaining spendable increment, state generation and monotonic bucket
deadline. Transmission follows this ordering:

```text
prune fully expired buckets
  -> require acknowledged current-process allowance for the complete ACK charge
  -> tentatively consume that allowance
  -> issue SetTx
  -> retain the charge if SetTx started or its effect is uncertain
  -> reclaim the tentative charge only after a definite pre-SetTx failure
```

At the bucket boundary the communicator freezes the grant and requests one
atomic state transition:

1. Retain the loaded baseline unchanged.
2. Replace the current process's provisional increment with the exact charge
   spent under it, reclaiming only definitely unused allowance.
3. Retain every uncertain transmission as charged.
4. Precharge the next bucket when the remaining rolling budget permits it.
5. Durably commit and acknowledge the new state before the next increment is
   used.

No transmission may use a frozen grant while settlement is pending. If
persistence reports a definite pre-installation failure, the preceding full
bucket precharge remains authoritative and the process may resume only an
allowance that is still within the original monotonic bucket deadline. If the
outcome is unknown, TX is suppressed until the exact installed generation is
reconciled. After the increment is exhausted or the bucket ends, TX remains
suppressed until another durable bucket increment succeeds.

After restart, no loaded bucket charge is spendable. A precharge committed by
the preceding process remains fully charged until normal expiration, and the
new process must durably add its own increment. Therefore repeated crashes
accumulate conservative bucket charges rather than losing possible
transmissions:

```text
crash before bucket-increment commit -> no TX was enabled under it
crash after bucket-increment commit  -> its complete increment remains charged
```

When no bucket increment fits, an otherwise valid message is still
authenticated, validated and accepted through a `PersistQueue` capacity
reservation; its ACK is suppressed. Lack of receiver TX budget never reverses
message acceptance.

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
  |       -> construct complete MessageProfilingV1 and try to enqueue it
  |       -> if admission fails, increment the profiling-admission counter;
  |          never change the outcome to RETRY_LATER
  |
  +--> protocol selects a response-eligible authenticated rejection
  |       -> construct the protocol-selected rejection ACK and pre-TX profiling fields
  |       -> require persistence AVAILABLE and reserve one profile slot,
  |          or select RETRY_LATER
  |
  +--> authenticated structurally valid reading
          -> validate
          -> construct AuthenticatedReadingCandidateV1, deterministic candidate ACCEPTED ACK
               and pre-TX MessageProfilingV1 fields containing the exact ACK frame
          -> require persistence AVAILABLE and atomically reserve one slot for
               the complete pair without consulting message history
          -> if unavailable: reserve nothing, RETRY_LATER, not accepted
          -> otherwise: mark occurrence accepted and select ACCEPTED
  |
  v
For a response-eligible outcome, use the selected deterministic ACK
Check acknowledged airtime bucket grant
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
otherwise retain the already-recorded reservation result only
Return to DIO1 wait
```

The communicator reserves one queue slot before an ACK transmission but
publishes no partial profile. It retains its local construction state while TX,
suppression and any bounded recovery complete, then creates one complete frozen
typed unit and publishes that existing object reference. Publication performs
no queue-capacity check, payload copy or serialization and cannot fail because
of queue pressure. Construction of the Python value remains ordinary process
work; the pilot does not claim that CPython performs no allocation after ACK
selection.

If persistence admission is unavailable or profile reservation fails, the detailed packet-occurrence record cannot be retained. The pilot explicitly permits this exception and selects `ACK_RETRY_LATER_DOWNLINK` for an authenticated packet that is eligible for a response. The reservation attempt has already incremented exactly one `persist_queue_admission_counts` cell for the selected profile-unit kind and returned `AdmissionResult`; no overlapping profiling-failure counter is maintained. Packets that cannot be authenticated remain silent. The communicator offers the cumulative matrix in a later `ReceiverHealthRequest`; a crash before successful admission may lose the increments under the documented persistence-unavailable observability limitation.

If the communicator regains control after an exception but cannot determine the attempted ACK's terminal radio outcome, it finalizes the reserved profile as `UNKNOWN_INTERRUPTED` before entering a terminal receiver state. A hard process crash or power loss drops the volatile reservation and leaves no partial SQLite row. This deliberately gives up persistence of pre-TX partial records and does not weaken the pilot's existing non-durable ACK guarantee.

Every radio failure path must either intentionally establish a known safe next
state or enter bounded recovery. An exceptional path constructs the one direct
or episode-level radio diagnostic required by
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md), but inability to admit
that best-effort diagnostic never changes radio-state or airtime safety.

## PersistQueue

`PersistQueue` is a bounded in-memory handoff between the communicator and persistence threads.

The concrete shared values, enum assignments, optional-field rules and typed
entity contracts are defined by [`INTERFACE.md`](INTERFACE.md).

A `PersistQueue` capacity reservation is volatile, process-local queue accounting. It is unrelated to the durable receiver TX-airtime bucket precharge described above.

Pilot production capacity:

```text
500 entities
```

`PersistQueue` preallocates a circular array of 500 generic slots. Each slot
holds an opaque immutable Python object reference plus queue metadata. Smaller
positive capacities may be constructed by focused tests, but production does
not exceed 500. Admission counts occupied slots only: every entity kind costs
one slot. The queue neither estimates recursive Python-object size nor assigns
logical byte charges. Measuring actual object/RSS cost and revisiting the cap
are deferred until receiver workload evidence justifies them.

The queue does not encode, decode, copy or construct normal entities. The
communicator creates a complete immutable typed value and publication stores
that existing reference; the persistence thread receives the same object.
Variable-length logical values such as received frames remain bounded by their
own entity contracts, not by queue storage policy. Only quarantine of a
poisoned typed entity uses the separate canonical tagged-JSON evidence codec.

The queue may contain:

- atomic `AuthenticatedReadingCandidateV1`/complete-`MessageProfilingV1` units;
- complete profile-only `MessageProfilingV1` records for occurrences without an authenticated reading candidate;
- complete `ClockObservationV1` time-state boundaries;
- `ReceiverHealthRequest` snapshots;
- structured diagnostics.

### Pilot queue policy

The pilot implements no `PersistQueue` priority policy. Every object published to the queue has the same importance regardless of whether it contains a measurement/profile unit, a profile-only occurrence, a diagnostic, a clock observation or a `ReceiverHealthRequest`.

The queue admits and processes published units in FIFO publication order. It
does not reorder, evict, sample or drop an admitted unit based on entity type.
New admission requires current `PersistenceAdmissionState = AVAILABLE` and one
free entity slot; either failure rejects the new object or complete atomic pair
without removing existing entries. For an authenticated packet eligible for a
response, either admission failure selects `ACK_RETRY_LATER_DOWNLINK`.

Outstanding reservations count against queue capacity but remain invisible to
the persistence consumer until publication. Publishing a complete object
against its reservation performs no capacity check and cannot be rejected due
to pressure. Once published, every entity remains queue-owned until its durable
disposition is acknowledged, independent of its type.

The communicator is the only reservation owner and processes one radio event
at a time, so it publishes or terminally finalizes the active reservation
before accepting another packet. Cancellation before any response is attempted
releases the reserved slot; publication retains it until durable
acknowledgement. Once an occurrence has been accepted, its reservation must be
published with the best terminal outcome available rather than cancelled.

A reservation token is unique and single-use. Before reserving capacity and
selecting an ACK, the communicator validates the authenticated candidate and
all stable profile inputs. After the terminal radio outcome it constructs one
frozen typed unit containing `ack_tx_result` and `T4` through `T6`, then gives
the queue that object reference. No normal-path entity serialization or kind-
specific queue builder exists.

Publication atomically transfers logical ownership of the complete immutable
unit against its reservation. Python aliases cannot be revoked, so deep
immutability and producer non-use after publication are interface obligations.
An inability to construct the promised unit is an implementation failure, not
a reason to perform a second admission attempt after an ACK may have been sent.

### Non-recursive diagnostics

An ordinary queue-admission failure is not a diagnostic trigger. If any
`PersistQueue` reservation returns `PERSISTENCE_UNAVAILABLE` or `QUEUE_FULL`,
the communicator records exactly the original entity-kind/admission-result
matrix increment. It does not allocate a diagnostic sequence, construct a
`DiagnosticV1` about that result or attempt another reservation for that
failure. If the original entity was itself a diagnostic, its existing identity
is simply the failed original attempt; no additional diagnostic identity or
entity is created. This applies equally to measurement/profile, profile-only,
clock-observation, health and diagnostic admission attempts.

Failure of the diagnostic path must not recursively produce more diagnostics
through the same unavailable or full queue.

Fallback policy:

```text
normal structured diagnostic -> PersistQueue

if that fails:
    retain the DIAGNOSTIC admission-result matrix increment
```

Do not generate a diagnostic about failure to enqueue a diagnostic.

## Persistence thread

### Responsibilities

The persistence thread is the exclusive owner of SQLite and the receiver
configuration file.

It:

- loads, validates and publishes the immutable receiver configuration snapshot during startup;
- validates SQLite application identity and the immutable schema metadata
  singleton before admission;
- inserts the current `receiver_instances` start row;
- loads and validates durable communicator state during startup;
- opens and validates SQLite with the required WAL and synchronization settings;
- exclusively publishes `PersistenceAdmissionState` and closes ordinary admission on storage failure;
- services high-priority state and clean-stop control operations and acknowledges only confirmed durable outcomes;
- wakes approximately every five seconds, or when a configurable entity-count
  queue threshold is reached;
- checks whether the queue contains data;
- claims a FIFO batch of at most the configured number of complete queue
  entities;
- writes the batch using one SQLite transaction;
- removes entries from the queue only after a successful commit;
- classifies transport retransmissions and identity conflicts against canonical SQLite records;
- applies idempotent measurement insertion without replacing canonical contents;
- accumulates persistence timing, throughput, failure, quarantine and checkpoint counters for `ReceiverHealthV1`;
- durably isolates item-specific poisoned units without silently dropping them;
- inserts each complete `MessageProfileRowV1` once;
- enriches `ReceiverHealthRequest` snapshots with persistence-owned and host-owned observations and inserts complete `ReceiverHealth` rows;
- repeats while work remains.

The five-second interval, entity-count wake threshold and entity-count batch
limit are pilot defaults and should remain configurable. Choosing their runtime
values belongs to the later persistence worker rather than `PersistQueue`.

### SQLite durability and retention

The host build generates one exact `receiver/db/schema.sql` from the handwritten
schema source and the receiver enum and entity manifests. It then hashes the
exact generated bytes and emits that fingerprint, the schema version and SQLite
`application_id` into the generated Python interface module. The generated
catalogue tables are seeded once and protected by triggers against every later
insert, update or delete.

SQLite `application_id = 0x43555252` is the separate file-type marker in the
SQLite header. The immutable `database_metadata` singleton binds the configured
`group_id` to the sole `database_schema_version` and the SHA-256 fingerprint of
the exact packaged `schema.sql`. The pilot does not use `PRAGMA user_version`,
a migration ledger or in-place upgrades. Startup checks the application ID and
one metadata row rather than querying every generated catalogue. The complete
generation, initialization and enum contracts are defined in
[`INTERFACE.md`](INTERFACE.md) and [`db/README.md`](db/README.md).

Any pilot schema change starts a new database schema epoch. Deployment is an
explicit offline procedure that stops the receiver, preserves the preceding
database together with consistent WAL/shared-memory state, creates and
validates a fresh database from the new packaged schema, and only then installs
it. Receiver startup never drops, rewrites, migrates or silently replaces an
incompatible database.

The pilot database uses `PRAGMA journal_mode=WAL` and verifies that SQLite actually entered WAL mode. Every database connection sets `PRAGMA synchronous=FULL`; a connection that cannot establish the required journal or synchronization mode is not usable for persistence. The database, WAL and shared-memory files must remain together on the same local filesystem.

WAL checkpointing uses a configurable page or byte threshold and records checkpoint duration and result. Checkpoint work must not make the persistence thread ignore queue growth or state-commit requests indefinitely. A bounded checkpoint is also attempted during controlled shutdown before the database connection closes.

Implementation benchmarking must compare `synchronous=FULL` with `synchronous=NORMAL` on the deployed Pi and storage medium using realistic entity mixes, batch sizes and checkpoint behavior. Record at least transaction throughput, commit-latency percentiles, queue growth, WAL growth and checkpoint stalls. `FULL` remains the pilot deployment setting regardless of benchmark results; changing it requires an explicit architecture decision accepting weaker power-loss durability.

These settings protect transactions whose SQLite commit completed. They do not make a preceding radio ACK durable while its typed queue unit remains only in RAM.

The pilot performs no automatic retention deletion within the active schema
epoch. Reading-message rows, packet profiles, diagnostics, receiver health and
quarantined entities are retained for that complete epoch.
Preceding epoch databases remain read-only archives rather than being discarded
or silently imported into the new active database. Storage must be provisioned
from worst-case pilot rates with margin for the database, WAL, temporary SQLite
files, quarantine and required archives. Export, archive or pruning is an
explicit maintenance operation outside active pilot collection; it must not
silently remove canonical identity history needed for duplicate and conflict
classification.

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

The state starts as `UNAVAILABLE_STARTING` and becomes `AVAILABLE` only after
SQLite opens, the header `application_id` and immutable
`database_metadata` version/fingerprint/group binding are exact, the required
durability settings are confirmed and startup validation succeeds. A newer or
older schema version, changed schema fingerprint, missing/malformed metadata
singleton or configured-group mismatch publishes
`UNAVAILABLE_INCOMPATIBLE_SCHEMA`; the receiver never guesses how to interpret
or upgrade that database. Catalogue contents do not require separate startup
queries because the exact schema fingerprint covers their generated seed SQL
and database triggers make every catalogue immutable after creation. A current
unavailable state closes all new ordinary `PersistQueue` admission even if RAM
capacity remains; already published units stay queue-owned, and the synchronous
persistence control channel retains its separate semantics. An authenticated
packet otherwise eligible for a response is not accepted and selects
`ACK_RETRY_LATER_DOWNLINK`; packets requiring silence remain silent. Periodic
health, diagnostics and other ordinary entities increment bounded in-memory
failure counters rather than entering the unavailable queue. Queue admission
counts record `PERSISTENCE_UNAVAILABLE` without duplicating its reason;
persistence-owned admission-state transition counters retain aggregate reason
information for a later health row. Availability is an admission gate, not a
durability promise: a transaction can still fail immediately after the
communicator observes `AVAILABLE`.

On a low-space or disk-full condition, the persistence thread:

1. rolls back the transaction and retains ownership of every affected queue unit;
2. publishes the corresponding unavailable state;
3. avoids a tight retry loop by using bounded retry backoff while observing free space;
4. retries the original immutable units without reconstruction when space is available; and
5. returns to `AVAILABLE` only after the required database checks and a transaction succeed.

Disk-full failure is not a poisoned-entity classification and must not cause a queued unit to be dropped or quarantined.

A lock/contention result, temporary access failure or other non-capacity,
non-corruption global SQLite or storage failure uses `UNAVAILABLE_IO`. Once an
ordinary transaction has such a failure, the persistence thread:

1. distinguishes a definite pre-commit failure from an outcome requiring
   reconciliation;
2. rolls back when the failure is definitely pre-commit, retains the original
   immutable queue units and every frozen persistence-derived value, and never
   classifies them as poison;
3. publishes `UNAVAILABLE_IO` on the first classified global failure, closing
   new ordinary admission;
4. bounds each SQLite lock wait and schedules recovery with an interruptible
   monotonic backoff starting at 250 milliseconds and doubling to a five-second
   cap;
5. services a pending persistence-control command at the next safe boundary
   before an ordinary retry;
6. reopens and revalidates SQLite when required, then retries or reconciles the
   original pending batch without reconstruction; and
7. resets the backoff and returns to `AVAILABLE` only after the required checks
   and the pending transaction commit or exact reconciliation succeed.

There is no maximum retry count for this recoverable state. At the cap the
persistence thread continues low-frequency attempts so an operator-corrected
mount, access or contention condition can recover without restarting the
process and losing the volatile queue. The shared wakeup may interrupt the
backoff for control work or shutdown, but an unrelated wakeup does not make an
ordinary retry run before its retry deadline. Corruption and incompatible
schema remain operator-recovery states under their separate policies.
The same unavailable state and retry scheduler pace a transient/global failure
while reconciling an unknown ordinary outcome or committing quarantine; those
paths retain the active lease and frozen intended rows required by their
existing reconciliation contracts.

On SQLite corruption or a failed configured integrity check, the persistence thread rolls back when possible, publishes `UNAVAILABLE_CORRUPT`, closes the database and preserves the database, WAL and shared-memory files together. It must not automatically delete, replace, truncate or rebuild them. Radio RX may continue with all new ordinary `PersistQueue` admission closed so nodes retain their readings. Operator recovery must preserve the corrupt artifacts for diagnosis, restore or recover the database through an explicit maintenance procedure, and pass startup validation before persistence returns to `AVAILABLE`.

If the process crashes or loses power while an accepted unit remains only in the volatile queue during either outage, that unit can still be lost under the pilot's documented non-durable-ACK guarantee. Eliminating that window requires the deferred durable queue or pre-ACK journal.

### Communicator-state ownership

The persistence control channel is separate from `PersistQueue` and has stronger semantics. A private control command is never sampled, dropped, merged silently or acknowledged on submission. `commit_communicator_state()` either reports the requested generation and exact canonical bytes durably installed, reports that the preceding generation definitely remains authoritative, or returns an unknown outcome that requires a serialized state reload before TX resumes.

Servicing this separate synchronous control path is not a `PersistQueue` priority class and must not reorder already published queue units.

The persistence thread does not independently edit clock or airtime policy. It applies the normative state-validation order—SQL envelope and digest, digest-protected version, complete supported-version canonical structure and generation invariants, then policy parameters—writes the communicator-owned singleton and returns the result. A corrupt application-level relation can be replaced by generation one only in the same transaction that archives every exact rejected row in `quarantined_communicator_states`. An unsupported state version or policy mismatch is not repaired automatically. This preserves a single policy owner without allowing two threads to write disk.

An ordinary SQLite batch and a communicator-state transaction remain separate because they protect different guarantees. Their I/O ordering must nevertheless ensure that a control command cannot be starved indefinitely by telemetry batches.

Control priority is cooperative rather than preemptive. Submitting a
synchronous command places its immutable request in the in-memory control
mailbox and sets the shared persistence wakeup. The
communicator waits on that command's private completion event until its
absolute monotonic deadline; it does not poll. An interruptible idle or retry
wait wakes immediately, but the persistence thread never interrupts or enters
another SQLite operation from inside an open transaction. It services the
mailbox before dispatching an ordinary batch and after each commit, rollback or
reconciliation boundary, before any ordinary retry.

Mailbox insertion and ordinary-batch dispatch share a short scheduler lock. If
submission wins that ordering point and the control bypass is available, the
control command runs first. If batch dispatch wins, that one bounded attempt
reaches its safe boundary before the control command runs. The scheduler lock
is never held during SQLite or other filesystem I/O. A caller timeout
atomically cancels a command whose durable effect is still definitely
impossible; if a mutating command may have crossed `COMMIT`, it remains
serialized and reports an unknown outcome for later reconciliation. At most one
control command may bypass an already-due ordinary attempt at one boundary;
after completing it, the scheduler gives one due ordinary attempt a dispatch
turn before a later control command so neither path can starve the other.

### Reading-message classification

The persistence thread classifies accepted reading occurrences in queue order.
SQLite stores one immutable row for each new authenticated logical message and
one distinguished canonical row for each application sample:

```text
reading_messages PRIMARY KEY (node_id, message_id)
UNIQUE (node_id, sample_id) WHERE is_canonical_for_sample = 1
```

Every `reading_messages` row stores the clear 32-byte reading body, decoded
columns, `sample_id`, whether that logical message is canonical for the sample,
and the first occurrence identity. The referenced `message_profiles` row stores
the exact authenticated frame and domain. `is_canonical_for_sample = 0` does
not mean that the packet or reading body is malformed: it identifies a valid
logical message whose sample was already canonical or conflicted with an
existing canonical body. These identities are scoped to the node identity and
key lifetime required by the protocol. Provisioning must not silently reuse a
database identity with a reset counter.

For an existing `(node_id, message_id)`, persistence compares the candidate
with the immutable reading-message row and its first occurrence profile:

- the same `sample_id` and exact authenticated frame is `RETRANSMISSION`;
- the same `sample_id` and a different exact frame is `DUPLICATE_CONFLICT`;
- a different `sample_id` is `MESSAGE_ID_CONFLICT`.

No existing reading-message row is replaced. A message-ID conflict creates
only the current occurrence profile; that profile and the earlier immutable row
plus its first profile are the durable conflict evidence. Persistence creates
no conflict `DiagnosticV1`.

For a new message ID, persistence looks up the one row for the same
`(node_id, sample_id)` whose `is_canonical_for_sample` is true. If none exists,
it inserts the new row with `is_canonical_for_sample = 1` and classifies
`FIRST_SEEN`. If a canonical row has the same exact reading body, it inserts the
new logical-message row with `is_canonical_for_sample = 0` and classifies
`DUPLICATE_SAME_CONTENT`; this includes an expected current-to-backlog
conversion with a new message ID. Different contents likewise insert a
noncanonical row but classify `DUPLICATE_CONFLICT`. The partial unique index is
the relational backstop that prevents two canonical rows for one sample; code
still owns classification and the Boolean it supplies.

Classification, the reading-message effect and the occurrence profile belong
to one SQLite transaction. Conflict handling is successful processing and the
corresponding queue pair is removed after commit. A SQLite or transaction
failure is not a conflict: the queue retains ownership and retries the original
immutable pair. The persistence thread constructs the stored profile with its
derived `persistence_classification`; it does not mutate the communicator's
queued object.

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

An ordinary batch attempt has one internal outcome:

```text
COMMITTED
NOT_COMMITTED
OUTCOME_UNKNOWN
```

`COMMITTED` permits the final per-entry durable disposition.
`NOT_COMMITTED` is used only when failure is known to precede durable commit;
the entries remain pending for retry. Once `COMMIT` may have executed without
confirmation, the outcome is `OUTCOME_UNKNOWN`: the persistence thread retains
the lease and every frozen persistence-derived value and reconciles before
acknowledging or reconstructing work.

Every queue entity has one primary durable identity:

```text
ClockObservationV1       (receiver_instance_id, observation_sequence)
ProfileOnlyUnitV1        (receiver_instance_id, occurrence_sequence)
MeasurementProfileUnitV1 (receiver_instance_id, occurrence_sequence)
ReceiverHealthRequestV1  (receiver_instance_id, health_sequence)
DiagnosticV1             (receiver_instance_id, diagnostic_sequence)
```

Reconciliation applies the same rule to every identity. If its row is absent,
the normal insertion path may run. If it is present, every stored column must
exactly equal the frozen intended value, using byte equality for BLOBs and
null-safe equality for optional values; exact equality is already-committed
success and the row is kept unchanged. A differing row is a correctness-critical
identity collision: it is not overwritten, updated, classified as poison or
acknowledged. Persistence retains the batch and publishes
`UNAVAILABLE_INCOMPATIBLE_SCHEMA` pending operator or implementation recovery.

Queue-backed rows have no UTC-enrichment exception. Profiles and diagnostics
store only their monotonic timestamps; analysis derives UTC without modifying
them. `ClockObservationV1` stores its own sampled UTC when trusted, and that
complete observation row is also immutable.

For `ProfileOnlyUnitV1`, exact equality covers every stored profiling field and
`persistence_classification = NOT_APPLICABLE`. For
`ReceiverHealthRequestV1`, the persistence thread freezes one complete
`ReceiverHealthV1` before the first commit attempt; replay compares every
communicator, persistence and optional host-observation column and never
resamples it. For `DiagnosticV1`, exact equality covers every queued field;
persistence adds no boot-identity column.

For a `MeasurementProfileUnitV1`, exact replay additionally validates the
immutable canonical side effects against the stored profile classification:

- `FIRST_SEEN`: the current occurrence owns a matching reading-message row with
  `is_canonical_for_sample = 1`;
- `RETRANSMISSION`: an earlier occurrence owns the reading-message key and its
  first profile has the same exact authenticated frame;
- `DUPLICATE_SAME_CONTENT`: the current occurrence owns a new noncanonical
  reading-message row whose body matches the earlier canonical sample row;
- `DUPLICATE_CONFLICT`: either an earlier occurrence owns the same message key
  and sample with a different exact frame, or the current occurrence owns a new
  noncanonical row whose body differs from the earlier canonical sample row;
  and
- `MESSAGE_ID_CONFLICT`: an earlier reading-message row has the same message key
  and a different sample ID.

The existing profile's stored classification is authoritative during replay;
persistence validates it and never reclassifies the occurrence from the now
populated tables. A reading-message row that claims the current occurrence as
its first owner while the corresponding profile row is absent is an impossible
partial effect of the required atomic transaction and therefore the same global
invariant failure.

One retry transaction may treat matching rows as no-ops and insert identities
that are still absent. Only confirmed commit, or later reconciliation showing
every unit exact and complete, permits queue acknowledgement.

An atomic measurement/profile pair is one queue entity. Entity-count batching
therefore cannot split it: persistence either includes the complete pair in the
claimed FIFO prefix or leaves it at the head for a later transaction.

### Poisoned entities

All queue units are complete immutable typed values when published. A poisoned
unit is an admitted unit that reproducibly fails while isolated because of an
entity-specific binding, derivation, range, schema-version or unexpected
schema-constraint defect. Malformed radio input, duplicate classification,
expected uniqueness conflicts, disk full, locking, database corruption and
transient or global I/O failures are not poison.

`ClockObservationV1` is deliberately excluded from item quarantine. Losing a
time-state boundary while allowing later entities to commit could make
persistence correlate them across an undocumented clock step. If a clock
observation reproducibly fails in isolation, persistence retains it and the
complete claimed batch, publishes `UNAVAILABLE_INCOMPATIBLE_SCHEMA` and admits
no later ordinary work. Recovery requires a compatible implementation or
schema; it never converts the observation into a gap and continues.

The persistence thread needs a bounded failure-isolation strategy:

```text
batch fails
  -> roll back and exclude global or transient causes
  -> narrow the batch and reproduce the suspected unit alone
  -> encode and durably quarantine its exact tagged logical evidence and minimal failure provenance
  -> only then remove it from PersistQueue
  -> continue with later entities
```

An admitted unit is never silently dropped, regardless of whether an ACK was
transmitted for it. A `MeasurementProfileUnitV1` remains one indivisible logical
unit during isolation and quarantine. The generic append-only SQLite
`quarantined_entities` table preserves a bounded canonical `QuarantineEvidenceV1`
tagged-JSON snapshot, redundant kind/version/length metadata, receiver-instance
identity, quarantine monotonic time, database schema version, stable failure
reason and operation, available SQLite/OS codes and isolation-attempt count.
The lifecycle join supplies the Linux boot identity. Its primary key is SHA-256
of the exact evidence bytes; an identical existing row makes retry idempotent
only after every stored value is verified.

After the normal batch transaction rolls back, quarantine uses a separate
WAL/`FULL` transaction. Only a confirmed or reconciled quarantine commit
permits the batch to acknowledge that unit as `QUARANTINED`. If evidence cannot
be encoded or quarantine cannot be durably committed, the complete batch
remains queue-owned, persistence closes new ordinary admission and reports the
applicable incompatible or I/O state. Infinite retry of the same failing batch
is not acceptable, but neither is removing an already-ACKed unit without
durable evidence. Quarantining is successful failure isolation, not successful
canonical measurement persistence; the retained evidence exists for diagnosis
and later recovery.

Persistence does not invent a `DiagnosticV1` for an asynchronous poison. Only
the communicator owns diagnostic identity. A successfully committed quarantine
row preserves exact poison evidence. A later `ReceiverHealthV1` may add
aggregate evidence, but it is not a guaranteed durable record while
persistence remains unavailable. The exact quarantine schema and allowed
failure reasons are defined in
[`INTERFACE.md`](INTERFACE.md).

## Telemetry

### Packet-occurrence profiling lifecycle

One received radio occurrence produces one communicator-owned
`MessageProfilingV1`. It is evidence about that physical occurrence, not the
canonical reading and not a persistence decision. Its durable identity is
`(receiver_instance_id, occurrence_sequence)`. The communicator copies the
received frame and radio metadata into Pi-owned memory before later radio work
can overwrite them, then freezes every terminal processing, ACK and recovery
field before queue publication.

The profile carries the exact received frame, authentication and processing
facts, radio metadata, selected ACK and terminal ACK-transmission result. Its
event timeline contains, when applicable:

- `T0`: kernel-recorded DIO1 edge timestamp and `received_at_monotonic_us`;
- `T1`: Python handler begins;
- `T2`: the packet is copied from SX1262 into Pi RAM;
- `T3`: AES-CCM authentication completes;
- `T4`: `SetTx` is issued or attempted;
- `T5`: the TxDone edge occurs; and
- `T6`: `SetRx` is issued.

`T0` and `T5` are kernel-event monotonic timestamps. A path that does not
produce one of the optional events stores that timestamp as absent rather than
reconstructing it.

It does not record queue occupancy or capacity; queue admission attempts are
instead accumulated by entity kind and result in the communicator-owned health
snapshot. The profile also does not contain `PersistenceClassification`, UTC,
`SystemTimeQuality` or `RtcHealth`.

The communicator publishes the completed profile in exactly one of two queue
forms:

1. `ProfileOnlyUnitV1(profile)` records an occurrence with no accepted
   application candidate.
2. `MeasurementProfileUnitV1(candidate, profile)` atomically pairs the profile
   with one `AuthenticatedReadingCandidateV1`. The candidate contains only the
   authenticated transport/application identity, domain and exact reading body
   needed by persistence; it is not a prospective `reading_messages` row.

Persistence consumes either queue unit without mutating it. It creates one
`MessageProfileRowV1(profile, persistence_classification)` for every durable
occurrence. A profile-only occurrence uses `NOT_APPLICABLE`. For a measurement
unit, persistence derives the classification from the existing
`(node_id, message_id)` row and canonical `(node_id, sample_id)` relation.

A measurement unit has one mandatory profile-row effect and one conditional
reading-message effect. A new transport identity creates an immutable
`ReadingMessageRowV1`; a retransmission or an existing message-ID conflict does
not. When a reading row is created, its
`(first_receiver_instance_id, first_occurrence_sequence)` points to the exact
profile that first established that transport identity. The applicable profile
and reading-row effects commit atomically, and ambiguous replay retains the
already-derived classification rather than reclassifying against newer rows.

Analysis treats the two tables as distinct evidence layers. `message_profiles`
answers what happened to each physical packet occurrence; `reading_messages`
answers which authenticated logical messages and canonical samples exist. A
profile remains monotonic-only. Analysis may resolve its Linux boot through
`receiver_instances` and derive UTC through a qualifying `ClockObservationV1`
without updating either table.

Derived intervals should be computed during analysis or persistence rather
than on the radio-critical path when practical:

```text
T1 - T0 : kernel/userspace scheduling delay
T2 - T1 : IRQ and SPI receive handling
T3 - T2 : authentication and decode time
T4 - T3 : decision and ACK preparation
T5 - T4 : TX transition and LoRa airtime
T6 - T5 : TX completion handling and RX restart
```

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
commits and failures; committed entities; total and maximum commit duration;
quarantine successes and failures; admission-state transitions; and WAL-
checkpoint attempts, successes and failures. Each `ReceiverHealthV1` snapshots
these aggregates.

The communicator-owned admission-result matrix reports queue-full pressure by
entity kind. Together with the cumulative commit and duration aggregates, it
provides bounded evidence that persistence is falling behind without inventing
byte occupancy or adding `persistence_batches` or
`receiver_state_operations` tables.

### ReceiverHealth

`ReceiverHealth` is a periodic application-aware snapshot. It is not sampled per packet and is not part of ACK acceptance. The initial pilot interval is one minute and remains configurable.

The communicator drives the interval because only that thread can take a coherent snapshot of its live state. Once per interval, and once immediately after successful communicator initialization, it advances `health_sequence` and attempts a nonblocking `ReceiverHealthRequest` reservation. It increments the returned admission-matrix cell and, on `RESERVED`, constructs the immutable request with the updated matrix and its other coherent observations before publishing it. If the health deadline coincides with DIO1 or another radio deadline, the communicator completes the deadline-bound radio work before starting the health attempt. This event-loop scheduling rule does not give the resulting queue object different FIFO importance after publication.

The request contains only communicator-owned observations:

- `receiver_instance_id`;
- a monotonically increasing per-instance `health_sequence`;
- `communicator_sampled_at_monotonic_us`;
- current radio state;
- cumulative exceptional radio-recovery attempt, success and failure counts;
- bounded recovery counts by reason when practical;
- current `system_time_quality` and `rtc_health`;
- cumulative time-quality and RTC-health transition counts and their last transition times;
- cumulative chrony step-command results by `SUBMITTED`, `NOT_SUBMITTED` and
  `OUTCOME_UNKNOWN` disposition;
- cumulative DS3231 write results by `COMPLETED`, `NOT_APPLIED` and
  `OUTCOME_UNKNOWN` disposition, plus verified-readback and
  post-write-trust-invalidation counts;
- a cumulative matrix containing exactly one result for every queue reservation
  attempt, indexed by `PersistQueueEntityKind` and `AdmissionResult`.

Each returned chrony step result and each returned RTC write result increments
exactly one disposition cell. Their sums derive the corresponding command or
write attempt totals; no overlapping `chrony_step_required_count` or RTC-write
attempt counter is stored. A step policy episode that cannot publish its
boundary and therefore never calls chrony is not a command attempt.
The RTC verification counters distinguish a possibly successful device write
from one that established usable provenance under a still-valid episode
`clock_state_generation`. These are process-local operational observations,
not correctness state in `CommunicatorStateV1`.

The request sequence and communicator sampling time are the communicator heartbeat: they prove that the communicator event loop reached the periodic health task. No separate high-frequency heartbeat is required. `health_sequence` advances before every reservation attempt, so a gap in persisted sequence values exposes a missed or failed sample. The `RECEIVER_HEALTH_REQUEST` failure cells distinguish known queue-full and persistence-unavailable losses once a later request succeeds. A successfully admitted request includes its own `RESERVED` attempt because the communicator updates the matrix before constructing and publishing the immutable request.

The matrix counts reservation calls, not packets, SQL rows, bytes or durable
entities. `RESERVED` means capacity was obtained even if the process later
dies or the communicator legally cancels before publication. Aggregate totals
are derived by summing cells; overlapping stored counters are forbidden. The
matrix deliberately collapses every unavailable reason into
`PERSISTENCE_UNAVAILABLE`. Persistence-owned current-state and transition
counters provide bounded cause evidence, but the pilot does not promise the
exact unavailable reason for each rejected attempt.

Radio recovery means an exceptional transition into `RECOVERING` after conditions such as a BUSY timeout, SPI failure, unexpected IRQ, missing or uncertain TX completion, or failed attempt to restore RX. A confirmed radio TX-timeout IRQ followed by successful RX re-arming, and the normal `SetRx` operation after packet handling or ACK transmission, are not counted as recovery. Recovery counters are cumulative within one `receiver_instance_id`; interval counts are derived during analysis.

When the persistence thread dequeues a request, it does not mutate the queued
object. It samples its own and the Pi's current observations and constructs a
new complete `ReceiverHealthV1` row. In addition to the request fields, its
mandatory persistence-owned fields are:

- `persistence_sampled_at_monotonic_us`;
- current persistence-admission generation, state and transition time;
- cumulative admission transitions indexed by every
  `PersistenceAdmissionState`, including incompatible schema;
- cumulative durable-quarantine successes and failures;
- cumulative batch transaction attempts, commits and failures;
- cumulative entities committed;
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

`ReceiverHealthRequest` has the same FIFO queue importance as every other published object. Its nonblocking admission may fail when persistence admission or queue capacity is unavailable, and an admitted health request consumes capacity that may cause a later measurement/profile reservation to fail and select `ACK_RETRY_LATER_DOWNLINK`. The pilot accepts this consequence instead of implementing prioritization or eviction. Once admitted, the request remains queue-owned until commit under the normal batch-ownership rule. The health sequence and the `RECEIVER_HEALTH_REQUEST` admission-result row make known admission gaps visible. The existence of a committed health row proves that the persistence thread reached that request; an in-process health record cannot prove liveness while the persistence thread itself is stalled.

## Diagnostics

[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md) is normative for the
fixed-size `DiagnosticV1`, common diagnostic enums, allowed domain/operation/
error combinations and every domain-local context encoding. An undefined
domain, error code or context schema must never be emitted. The stored row
remains monotonic-only; analysis may derive UTC from the clock-observation
timeline without updating it. Diagnostics contain no arbitrary exception text,
dictionary or payload dump. Sensitive material, keys, nonces and plaintext
follow repository logging rules.

Only the communicator allocates `diagnostic_sequence`, constructs
`DiagnosticV1` and admits it to `PersistQueue`. A synchronous
persistence-control failure can be returned to the communicator and converted
under the defined `PERSISTENCE_CONTROL` catalogue; a caller-side interface
violation uses `CORE`. Neither may be encoded using an improvised radio or
opaque context.
The persistence thread never creates a diagnostic identity or inserts a
persistence-created diagnostic row; its asynchronous failures remain visible
only to the extent that admission-state transitions, a later
`ReceiverHealthV1` or quarantine provenance can be retained.

One transition into `RECOVERING` defines exactly one logical radio diagnostic
and at most one queue-admission attempt. The communicator creates a bounded
in-RAM episode builder when recovery starts, updates it as soft and hard
recovery progress, and finalizes the one immutable value only when the episode
returns to confirmed `RX_SINGLE` or reaches a terminal radio state. Individual
recovery-stage failures do not create more diagnostics. The original operation
and error code remain the episode's root cause; the context separately records
command-effect certainty, recovery results and the last recovery-stage failure.
The exact radio catalogue, context and scenario policy are defined in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md#radio-diagnostic-catalogue).
During implementation, either the human implementer/reviewer or an AI agent may
propose a correction under that document's
[implementation-feedback contract](INTERFACE_DIAGNOSTIC.md#implementation-feedback-and-revision),
but implementation must not silently diverge from the currently agreed
encoding or recovery semantics.

The other closed catalogues and their scenario policies are:

- [`TIME`](INTERFACE_DIAGNOSTIC.md#time-diagnostic-catalogue) for exceptional
  chrony, Linux-clock, DS3231 and checked time-policy failures;
- [`PERSISTENCE_CONTROL`](INTERFACE_DIAGNOSTIC.md#persistence-control-diagnostic-catalogue)
  for operational failures returned synchronously by that channel; and
- [`CORE`](INTERFACE_DIAGNOSTIC.md#core-diagnostic-catalogue) for communicator
  implementation failures and caller-side interface violations.

Those policies deliberately exclude ordinary loss/restoration of time trust,
RTC `MISSING`/`INVALID`, queue admission results, airtime suppression or
bucket-grant expiration, normal protocol outcomes, duplicate/conflict
classification, poisoned-unit isolation and a confirmed TX-timeout IRQ that
restores RX normally. Their existing profile, clock-observation, health,
admission-state, communicator-state or quarantine records remain authoritative.

Ordinary `PersistQueue` results `PERSISTENCE_UNAVAILABLE` and `QUEUE_FULL`, and
the corresponding `PersistenceAdmissionState` transitions, are represented by
their existing counters and state snapshot. They never cause a
`DiagnosticV1` construction or a second queue-admission attempt.

### Persistence-unavailable observability limitation

For the pilot there is no second durable diagnostic store outside the normal
SQLite path. Low space, disk full, an incompatible schema, database corruption,
failed quarantine and asynchronous SQLite failures may therefore leave no
durable structured record when persistence cannot write its normal rows.
Admission state and in-memory counters remain available to the running process;
a later `ReceiverHealthV1` can preserve aggregate evidence if persistence
recovers. Neither is guaranteed to survive a process crash or power loss during
the outage.

This is an accepted observability limitation, not permission to continue
ordinary admission or discard queue ownership. The normal fail-closed
persistence, quarantine and recovery rules still apply. A future durable
fallback must define its own bounded format, ownership and failure policy; the
pilot does not add an append-only side file or a second disk writer.

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
  - start one bounded radio-diagnostic episode builder;
  - enter `RECOVERING`;
  - finalize the one diagnostic after bounded recovery succeeds or reaches a terminal state;
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
  - use generation zero for missing or corrupt history and keep RTC provenance untrusted until the synthetic worst-case generation-one ledger commits;
  - preserve a corrupt singleton in the same atomic transaction as its synthetic generation-one replacement;
  - for unsupported-version or policy-mismatch state, suppress TX for the complete conservative active-policy rolling-window wait, then atomically archive the exact old singleton and install an empty generation-one ledger;
  - continue RX and application-queue admission;
  - suppress TX until a valid state and acknowledged bucket grant are available.

- RTC missing or invalid:
  - update current `rtc_health`;
  - retain network-synchronized system time when available;
  - do not use RTC holdover or overwrite RTC provenance;
  - continue RX while timestamp and TX policies follow current system-time quality.

Authentication failure, malformed or unsupported protocol data, persistence admission failure and airtime-based ACK suppression remain normal packet-processing outcomes. Asynchronously discovered duplicate and identity conflicts are persistence outcomes and do not affect radio state. RX header or CRC errors and confirmed radio TX-timeout IRQs may also return directly to `RX_SINGLE` when IRQ clearing, `UPLINK_RX_PROFILE` restoration and `SetRx` all succeed.

### Radio recovery procedure

On entry to `RECOVERING`, the communicator must suppress new transmissions and
create the single in-RAM radio-diagnostic episode builder defined by
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md#radio-diagnostic-emission-contract).
It records the preceding state, failed operation, command-outcome
classification, known IRQ and device-error state, and selected recovery action.
If a transmission may have started, its airtime charge remains reserved even
when the final radio outcome is unknown.

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

Returning to confirmed `RX_SINGLE`, entering `HARDWARE_MISSING` or entering
`RECOVERY_EXHAUSTED` finalizes exactly one diagnostic for the complete recovery
episode. Recovery-stage failures update the builder and health counters rather
than emitting separate diagnostics.

## Concurrency model

### Why the pilot uses `threading.Thread`

The pilot runs the communicator and persistence owners in two ordinary
`threading.Thread` instances within one Python process. These are operating-
system threads: the Linux scheduler may place them on different logical cores,
and one can make progress while the other is blocked in GPIO, SPI, SQLite or
filesystem I/O. The design does not pin either thread to a core and does not
require or promise simultaneous execution of Python bytecode. On the standard
CPython build the GIL still serializes ordinary Python-bytecode execution; many
blocking I/O operations release it. This is sufficient because the workload is
I/O-oriented and the bounded packet-to-ACK CPU work is small. It is not a hard-
real-time scheduling claim.

A single `asyncio` event loop would normally remain one OS thread. It would not
put persistence on another logical core, and blocking GPIO/SPI/SQLite APIs
would still need thread executors or async-specific adapters. That would retain
threads while spreading ownership and cancellation semantics across two
programming models. Multiprocessing would permit parallel Python-bytecode
execution, but would replace direct immutable-object handoff with serialization
and IPC and add process-lifecycle, failure-reconciliation and secret/configuration
ownership machinery. The pilot has no measured CPU-bound need that justifies
those costs.

`PersistQueue` therefore bridges the two owner threads directly. One short
`threading.Lock` protects each compound reservation, publication, claim,
acknowledgement, admission-snapshot and closure transition. Correctness does not
rely on the GIL or on an individual list/integer operation being atomic, and the
lock is never held across caller work, radio operations, evidence encoding,
SQLite or filesystem I/O. No additional worker is introduced unless later
measurements establish a concrete need.

Shared state should be minimized to:

- `PersistQueue`;
- the high-priority synchronous persistence control channel and its immutable completion results;
- shutdown signaling;
- generation-numbered read-only publication of `PersistenceAdmissionState`;
- the immutable `linux_boot_id` returned during startup.

Receiver-health counters remain communicator-owned and cross the existing `PersistQueue` only inside immutable `ReceiverHealthRequest` snapshots; they do not create another shared mutable state path.

The persistence thread must not mutate communicator-owned node, message, clock-policy or airtime-policy state. Its communicator-state result reports only whether the exact requested generation and bytes became durable, definitely did not become durable, or require reconciliation.

Clean shutdown coordinates and bounds draining but is not part of the
correctness argument. Process termination or power loss may discard the
volatile queue under the pilot's documented ACK semantics; neither thread
depends on a finalizer running.

## Core invariants

- Only the communicator mutates live protocol, time-policy and airtime-policy state.
- Only the persistence thread performs disk I/O and loads receiver configuration.
- Only the persistence thread publishes `PersistenceAdmissionState`; the communicator may accept an application/profile unit only while its current snapshot is `AVAILABLE` and one queue slot is reserved.
- SQLite application identity and the immutable database schema
  version/fingerprint/group metadata must exactly match the build before
  ordinary persistence admission becomes available.
- The current `receiver_instances` start row is durable before ordinary persistence admission becomes available; only `commit_receiver_clean_stop()` may add its complete clean-stop marker.
- Each durable receiver-instance start is an implicit ordinary correlation
  boundary. Analysis never assigns an observation to an event from another
  receiver instance; the first later trusted same-instance observation may
  backfill to the start only when no explicit step boundary intervenes.
- Radio operation begins only after required local storage is usable and the bounded boot-scoped RTC-bootstrap episode has completed, whether successfully or not.
- RTC-to-system-clock bootstrap occurs at most once per `linux_boot_id`; a receiver-process restart never repeats it and receiver startup never waits for network availability.
- `NETWORK_SYNCED` clock observations sample Linux system UTC;
  `RTC_HOLDOVER` observations sample the proven DS3231 directly. Offline
  operation reads the RTC only at its bounded observation cadence, never per
  packet, and writes neither clock from the other.
- Any time-derived trusted publication or RTC-provenance commit requires the
  same episode `clock_state_generation` before and after every blocking or
  yielding boundary. Only the episode's own checked atomic health/quality
  update may advance and replace that value; another change invalidates the
  result without requiring a long-held lock.
- `RX_SINGLE` is valid only while the complete protocol-defined `UPLINK_RX_PROFILE` is known to be active and `SetRx` has been confirmed.
- No ACK `SetTx` is issued before `ACK_TX_PROFILE`, including inverted IQ, is installed; after TX, `SetRx` is not issued until `UPLINK_RX_PROFILE`, including normal IQ and boosted RX, has been restored.
- `receiver_instance_id` changes on every receiver-process start; `linux_boot_id` changes only with the Pi's Linux boot.
- `receiver_instances` is the sole persisted `receiver_instance_id` to
  `linux_boot_id` mapping; every other queue or durable entity stores only its
  receiver instance and obtains boot scope through that immutable row.
- An atomic one-slot `PersistQueue` reservation for a prevalidated `AuthenticatedReadingCandidateV1`/`MessageProfilingV1` unit while persistence admission is available establishes protocol acceptance of that occurrence but not SQLite durability or conflict-free canonical insertion.
- A successful pre-TX reservation must publish exactly one complete immutable pair after the terminal radio outcome; persistence accepts only complete profiling rows.
- No admitted queue unit is silently dropped; an item-specific poison is removed only after its canonical tagged logical evidence and minimal failure provenance are durably committed to `quarantined_entities`.
- Only the communicator creates `DiagnosticV1`; persistence-derived conflicts use profile classification and canonical rows, while asynchronous persistence failures may be exposed through health counters, admission state and quarantine provenance but are not guaranteed to leave durable evidence while SQLite is unavailable.
- The pilot database uses WAL with `synchronous=FULL`, retains all pilot records automatically, and closes all new ordinary `PersistQueue` admission on low space, disk full, corruption or persistence I/O failure.
- A communicator-state generation is usable only after durable acknowledgement or explicit startup reconciliation.
- `communicator_state` has at most one generation-bearing row; generation zero is conservative runtime state and is never stored.
- Missing, corrupt or unsupported state never becomes an empty airtime ledger or trusted RTC provenance.
- Current `system_time_quality` and `rtc_health` are observed again on every
  receiver startup. The last-observed copies in `CommunicatorStateV1` are
  diagnostic only; immutable `ClockObservationV1` rows remain authoritative
  provenance for UTC values derived during analysis.
- Receiver event entities carry Linux monotonic time, never directly sampled
  wall-clock time; analysis resolves their `receiver_instance_id` values
  through `receiver_instances` and derives UTC from a trusted
  `ClockObservationV1` from that same receiver instance without updating the
  event row.
- A pending time-state boundary is offered to `PersistQueue` before later
  ordinary admissions, and no intentional clock step is requested before its
  exact `UNTRUSTED` boundary observation is completely published to the global
  FIFO. Later ordinary entities may be published only behind it.
- `CLOCK_MONOTONIC` controls live deadlines, event intervals and airtime
  aging. Safety-sensitive physical durations use the configured conservative
  elapsed-rate conversions, so bounded slew cannot expire a minimum retention
  early or extend a maximum lifetime too long; a realtime step does not jump
  the clock.
- All unexpired bucket charges, including every process's unused precharge, must fit the configured continuous-window budget before a bucket increment is committed.
- No TX occurs without an acknowledged, unspent bucket increment owned by the current receiver instance.
- Every bucket charge loaded from an earlier receiver instance is fully charged, unspendable and retained until conservative expiration.
- A started or uncertain `SetTx` consumes allowance. Only a definite pre-`SetTx` failure permits reclamation.
- A pending or unknown bucket-settlement outcome suppresses TX.
- Airtime suppression never reverses acceptance of an admitted occurrence.
- The earliest anchor-eligible accepted current-reading occurrence becomes the
  immutable direct timestamp anchor only when analysis can derive its
  `RX_DONE` UTC from a trusted same-Linux-boot clock observation, its
  `run_ms + Tair` is at most 30 seconds and its persistence classification is
  not a conflict.

## Deferred work

The following are explicitly deferred from the first pilot:

- durable queue or write-ahead journal before ACK;
- watchdog and health-management subsystem;
- sophisticated node scheduling;
- multi-receiver coordination;
- a kernel SX1262 driver;
- optimization from Python to C;
- complex BUSY edge handling;
- durable configuration management;
- authenticated node counter recovery with durable, never-reused receiver
  allocation for both transport and application identities;
- a durable structured diagnostic fallback for intervals in which normal SQLite
  persistence is unavailable.

Known consequences:

- any receiver-process restart loses unpersisted queue contents;
- the next durable receiver-instance start closes the preceding correlation
  segment, so loss of a volatile step boundary also sacrifices cross-instance
  UTC backfill rather than allowing an unsafe correlation;
- `linux_boot_id` distinguishes a Pi reboot from a receiver-only restart, while a new `receiver_instance_id` exposes both;
- a successful ACK does not guarantee durable storage;
- a persistence outage may leave no durable structured account of the outage's
  own low-space, disk-full, corruption, schema, quarantine or SQLite failure.

## Testing

At minimum, add tests or simulations for:

- new valid message;
- identical retry after successful ACK;
- identical retry after failed ACK transmission;
- same `message_id`, same `sample_id` and same exact frame classified as `RETRANSMISSION`;
- same `message_id`, same `sample_id` and different frame classified as `DUPLICATE_CONFLICT`;
- same `message_id` and different `sample_id` classified as `MESSAGE_ID_CONFLICT`;
- conflict classifications retain canonical rows and profile evidence without creating a persistence-owned diagnostic;
- same `sample_id` in distinct current/backlog transport messages with matching
  contents classified as `DUPLICATE_SAME_CONTENT`;
- same `sample_id` in distinct transport messages with conflicting contents
  classified as `DUPLICATE_CONFLICT`;
- deterministic ACK reconstruction from the same uplink and outcome without cached receiver history;
- nonce construction from `node_id || message_id || domain`, never `sample_id`;
- unauthenticated packet;
- authenticated malformed packet;
- protocol validation-order cases select the exact processing result and response policy defined by the protocol;
- an authenticated downlink ACK domain reaching the direction check is classified as `WRONG_DIRECTION` and never produces a response, including when its profile cannot be admitted;
- initialization and both recovery levels install the exact protocol PHY profile before entering `RX_SINGLE`;
- the ACK path transitions from normal-IQ boosted RX to inverted-IQ TX and restores normal-IQ boosted RX before `SetRx`;
- failures and uncertain outcomes during either radio-profile transition cannot leave `RX_SINGLE` asserted under a partial or inverted-IQ configuration;
- queue full and later recovery without allocating a diagnostic identity or
  making a second queue-admission attempt;
- persistence-unavailable admission for every entity kind increments only the
  original admission-result matrix cell and never constructs a diagnostic
  about that result or makes a second reservation attempt;
- stable entity inputs reject invalid lengths or values before reservation and ACK selection;
- one-slot pre-TX pair reservation followed by complete frozen-object construction and reference publication without queue serialization;
- exception finalization as `UNKNOWN_INTERRUPTED` without a partial SQLite row;
- BUSY timeout;
- unexpected IRQ combination;
- TxDone missing or delayed;
- radio diagnostics reject undefined domain/code/context combinations and use
  the exact fixed context encoding in `INTERFACE_DIAGNOSTIC.md`;
- a radio anomaly handled without `RECOVERING` constructs exactly one direct
  diagnostic and attempts admission at most once;
- soft-recovery success, soft-failure/hard-success and recovery exhaustion each
  finalize exactly one diagnostic, retain the original operation/error code and
  record the last recovery-stage failure without per-stage diagnostic rows;
- a packet/profile publication remains successful when the subsequent
  best-effort radio-diagnostic admission fails;
- SQLite startup enforces WAL and `synchronous=FULL`, including failure to establish either setting;
- exact packaged-schema fingerprint generation, `application_id` and the
  immutable metadata schema version/fingerprint/group binding are validated
  before admission without per-catalogue startup queries;
- a newer, gapped or otherwise incompatible schema publishes `UNAVAILABLE_INCOMPATIBLE_SCHEMA` without modifying the database;
- a database bound to a different `group_id` is unavailable and never imports the configured master key;
- every queue-bound `u64` at `INT64_MAX` persists successfully and a larger value fails as an interface invariant;
- implementation benchmark compares `FULL` and `NORMAL` with realistic Pi batches and checkpoints without changing the pilot default;
- committed transactions recover after process crash and simulated power interruption under WAL/`FULL` assumptions;
- low-space and disk-full failures roll back, retain queue ownership, close admission and recover with bounded backoff;
- lock/contention, temporary access and other global I/O failures close
  admission as `UNAVAILABLE_IO` on the first classified failure, retain frozen
  queue work without quarantine, and follow the 250-millisecond-to-five-second
  interruptible backoff without a maximum attempt count;
- the closed SQLite/host failure classifier maps capacity, corruption,
  compatibility, transient/global, entity-specific and unknown results to the
  required distinct paths, with every unrecognized result failing closed as
  `UNAVAILABLE_IO` rather than poison;
- an unrelated wakeup during persistence recovery does not advance the
  ordinary retry deadline, and admission returns to `AVAILABLE` only after the
  pending transaction commits or reconciles exactly;
- detected database corruption preserves the database, WAL and shared-memory files, closes admission and requires explicit recovery;
- an item-specific failure must reproduce in isolation before the exact complete unit is durably inserted into `quarantined_entities` and later valid units proceed;
- an isolated `ClockObservationV1` failure is never quarantined or bypassed,
  retains the queue head and closes admission as incompatible;
- `MeasurementProfileUnitV1` is never split during poison isolation or quarantine;
- an ambiguous quarantine commit is reconciled by matching the complete frozen quarantine row;
- definite and ambiguous transient quarantine failures retain the active lease,
  frozen isolation result and exact intended quarantine row while using the
  same paced recovery scheduler;
- quarantine failure retains the poisoned entity, publishes persistence unavailability and closes new admission;
- an ambiguous ordinary batch commit reconciles an exact complete row as a
  no-op success without resampling health fields or reclassifying a measurement;
- an absent ordinary durable identity is inserted on retry, while a differing
  row under the same identity retains the lease and closes admission as
  incompatible without update or quarantine;
- every measurement classification is replay-validated against its exact
  reading-message effect and canonical-sample relation, including impossible
  partial ownership;
- no automatic retention deletion during active pilot collection and low-water admission closure before exhaustion;
- receiver-process restart within one Linux boot and documented state loss;
- automatic receiver-process restart within one Linux boot creates a new `receiver_instance_id` without repeating RTC-to-system-clock bootstrap;
- Pi reboot changing both identity fields;
- queue entities and every receiver-instance-scoped table other than
  `receiver_instances` omit `linux_boot_id`, and their instance references
  resolve to exactly one lifecycle row;
- analysis never assigns a clock observation across receiver-instance or Linux-
  boot boundaries, while the first qualifying later same-instance observation
  may backfill to the durable instance start;
- valid RTC provenance resolves its verification receiver instance through
  `receiver_instances`, while a missing referenced lifecycle row makes the
  communicator state invalid;
- unavailable or read-only required storage prevents entry into radio operation;
- missing, invalid or unresponsive RTC completes bootstrap within its deadline and permits offline startup as `UNTRUSTED`;
- receiver lifecycle start, successful bounded queue drain and clean-stop marker;
- receiver-instance ordinal ordering across clean restart, crash restart and Pi reboot;
- repeated identical `commit_receiver_clean_stop()` is idempotent, while a conflicting marker or unmet precondition is rejected;
- an unknown clean-stop commit outcome is reconciled by repeating the exact request;
- controlled-shutdown queue-drain, airtime-state or marker-commit failure exits without a clean-stop marker and leaves conservative recovery state;
- periodic `ReceiverHealthRequest` enrichment with distinct communicator and persistence sampling times;
- equal FIFO treatment of clock observations, health, diagnostic, profiling
  and measurement units;
- health-request capacity consumption causing a later measurement/profile reservation to fail without eviction or reordering;
- monotonic timestamp fields missing on partial/error paths and analysis
  returning no UTC without a qualifying clock observation;
- missing and corrupt communicator-state rows produce conservative generation zero, then atomically install the configured synthetic worst-case generation-one ledger before TX;
- every observed corrupt communicator-state row is preserved and its synthetic generation-one replacement commits atomically;
- unsupported-version and policy-mismatch state suppresses TX for a complete conservative rolling-window wait, a process restart restarts that wait, and the exact rejected row is archived with an empty generation-one replacement in one transaction;
- an unknown embedded version together with a bad digest is `CORRUPT`, while an
  equal SQL/blob unknown version with a valid envelope and digest is
  `UNSUPPORTED_VERSION`;
- a supported state with both a structural defect and mismatched airtime policy
  is `CORRUPT`, while a fully valid supported state with only mismatched policy
  is `POLICY_MISMATCH`;
- missing singleton creation, ordinary next-generation commit, exact idempotent replay, generation conflict, stale generation and generation gap;
- an unknown communicator-state commit outcome is reconciled by loading the exact installed generation and bytes;
- durable state generation success, reported failure and unknown completion;
- separate persistence-control servicing can precede an ordinary batch without reordering FIFO queue units or starving ordinary persistence permanently;
- a control submission wakes idle and retry-backoff waits, while submission
  racing ordinary dispatch runs either before that attempt or immediately
  after its safe boundary according to the scheduler-lock ordering point;
- repeated sequential control submissions cannot starve an already-due
  ordinary batch, and sustained ordinary work cannot starve a pending control
  command;
- a queued control command cancelled at its deadline has no effect, a
  pre-commit cancellation cannot cross `COMMIT`, and a timeout after `COMMIT`
  may have run remains ordered for exact reconciliation;
- clearing and rechecking the shared wakeup cannot lose concurrent queue,
  control or shutdown work;
- current `NETWORK_SYNCED` time with a missing RTC;
- valid offline `RTC_HOLDOVER` without waiting for network availability, and rejected stale, invalid or unproven RTC values;
- network-to-RTC write, read-back and durable-provenance crash boundaries;
- an RTC refresh starts and commits provenance only below the stricter
  five-second source-error threshold, runs after post-step stability rather
  than step submission, and is invalidated by any intervening
  `clock_state_generation` change;
- the one-minute chrony-poll cap, three-hour online
  observation/RTC-refresh caps and one-hour holdover-observation cap are
  shortened when the calculated UTC error deadline is earlier;
- offline operation samples the DS3231 directly for holdover observations but
  never periodically writes Linux UTC from it or writes it from an
  unsynchronized Linux clock;
- chronyd has no automatic step path, no competing RTC writer and no competing
  system time service;
- deployment verification confirms the 3,500 ppm chrony slew ceiling, and the
  receiver rejects inconsistent declared-ceiling/rate-bound constants;
- target-Pi testing compares disciplined `CLOCK_MONOTONIC` with an independent
  elapsed-time reference during maximum positive and negative slew and
  validates the 3,700 ppm receiver bound;
- network error at and below 35 seconds can enter trust, the 35-to-40-second
  hysteresis band retains current quality, and total error above 40 seconds
  follows the ordered untrusted-step path;
- RTC holdover becomes or remains trusted only while its conservative
  verification, age/drift and direct-read uncertainty stays within the
  40-second pilot ceiling;
- remaining correction, root distance and sampling margin use checked,
  conservative arithmetic, and an invalid, stale or unreliable chrony source
  cannot establish trust or authorize a step;
- explicit chrony-step success, definite rejection, unknown outcome, stability
  polling, bounded backoff and receiver restart during every state;
- no explicit step before the exact `UNTRUSTED` clock-boundary observation is
  completely published to the global FIFO, and no ordinary post-boundary queue
  admission overtakes a pending boundary;
- later ordinary entities may be published immediately behind a successful
  step boundary but cannot commit ahead of it; isolated boundary failure closes
  admission without quarantine or bypass;
- trusted `adjtimex()` observation sampling accepts only a stable generation,
  bounded monotonic bracket and acceptable kernel metadata, while the expected
  no-`rtcsync` `STA_UNSYNC`/`TIME_ERROR` pair does not reject a chrony-confirmed
  network sample;
- quality ABA during observation sampling is rejected;
- direct RTC observations use the bounded read bracket and whole-second
  midpoint/uncertainty rule; RTC drift is charged before that read and only
  monotonic rate error is charged from the observation to an event;
- the fixed one-second sampling margin, read-back comparison boundary, exact
  advanced-network/read-back/direct-read provenance sum and every checked
  equality/one-unit boundary around the 40-second UTC budget;
- stored RTC verification uncertainty of four seconds yields the documented
  approximately 24.46-day one-hour-cadence limit and 39.93-day absolute
  direct-observation limit before accounting for a nonzero read bracket;
- a trusted observation whose error budget expires without a publishable
  replacement produces an `UNTRUSTED` boundary before later ordinary queue
  admission, but never authorizes a step without a separate fresh qualifying
  chrony result;
- periodic and transition `ClockObservationV1` persistence, including a
  receiver-process restart in the same Linux boot;
- deterministic preceding- and later-observation UTC correlation, absence when
  no trusted observation exists, rejection across receiver instances and Linux
  boots, no event-row mutation and permanent non-assignment inside a step-
  discontinuity gap;
- a clean same-boot process restart makes the durable instance start an
  ordinary boundary and permits the first later trusted same-instance
  observation to backfill only to that start;
- a process crash after step-boundary publication but before its commit loses
  all following volatile FIFO work, and the replacement instance's durable
  start prevents its observations from backfilling into the preceding process;
- a pre-boundary untrusted event is never correlated from a post-step trusted
  observation across the discontinuity;
- step success, rejection and unknown outcome keep the half-open interval from
  a durable step boundary to the first later trusted observation permanently
  without derived UTC;
- the 3,700 ppm elapsed-rate conversions lengthen minimum waits and shorten
  maximum lifetimes with checked integer rounding, including 3,613.32 seconds
  of monotonic retention for a one-hour physical rolling window and 29.889
  seconds for a 30-second maximum lifetime;
- bounded chrony slewing preserves same-instance UTC correlation; an explicit
  forward or backward step creates the required gap and correlation resumes
  only at the first later trusted observation; neither expires rolling airtime
  early;
- chrony-step and RTC-write outcome arrays increment exactly once per returned
  adapter result, derive their attempt totals by summation, and distinguish RTC
  read-back verification from post-write trust invalidation;
- rolling-window bucket boundaries, long idle intervals and cached-total reconstruction;
- current-bucket increment opening, spending, exact settlement and unused reclamation without reducing a loaded baseline;
- definite pre-`SetTx` failure versus started or uncertain `SetTx` charging;
- crash before and after bucket-increment commit;
- repeated crashes accumulate conservative bucket charges;
- expired old bucket charges and inability to obtain a new bucket increment;
- persistence failure or unknown settlement outcome suppresses TX without undoing acceptance;
- untrusted time causes analysis to return no UTC until a later eligible
  same-instance observation permits backward correlation, except across a step
  gap;
- direct anchoring rejects authenticated-but-unadmitted current occurrences,
  chooses the earliest anchor-eligible accepted occurrence and ignores a later
  eligible retransmission for that sample;
- direct anchoring accepts the largest representable `run_ms` for which
  `run_ms + Tair <= 30,000 ms` and rejects the next `run_ms` value without
  changing the stored occurrence;
- direct and extrapolated timestamp reconstruction, chain breaks, immutable
  analysis output and no mutation of receiver event rows.

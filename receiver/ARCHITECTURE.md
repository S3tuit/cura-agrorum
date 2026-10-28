# Receiver architecture

The Raspberry Pi receiver authenticates LoRa readings, responds within the node's
ACK window, and records readings and operational history in SQLite. The design
prioritizes explicit ownership and bounded packet processing over throughput.
Wire formats and node behavior belong to the
[LoRa protocol](../protocol/protocol-v2-lora/README.md); exact receiver values and
interfaces live in [code and schema sources](README.md#directory-map).

## Owners and data flow

There are two owners in one Python process:

- The communicator owns the radio, protocol processing, live time trust, and TX
  airtime policy. Its packet-to-ACK path makes no configuration-file, SQLite, or
  synchronous persistence call.
- The persistence worker owns configuration loading, SQLite, batching, checkpoints,
  and durable lifecycle/state changes. It returns immutable results and never
  invokes communicator callbacks.

```text
SX1262 <--> communicator: radio / ingress / time / airtime
                  |
                  +-- immutable readings, receptions, observations, diagnostics
                  |                          |
                  |                     PersistQueue
                  |                          |
                  +-- synchronous state --> persistence worker --> SQLite
                  |      and lifecycle       |
                  +<-- results/admission ----+-- receiver-group configuration
```

[`application.py`](cura_receiver/application.py) composes the owners;
[`communicator.py`](cura_receiver/communicator.py) owns an exchange;
[`communicator_scheduler.py`](cura_receiver/communicator_scheduler.py) schedules
housekeeping between exchanges. [`persistence_worker.py`](cura_receiver/persistence_worker.py)
services controls, ordinary batches, and checkpoint maintenance at transaction
boundaries, with bounded alternation so eligible work does not starve.

Ordinary OS threads fit the blocking GPIO/SPI/SQLite APIs without adding IPC,
serialization, or a second asynchronous cancellation model. Blocking I/O can
release the GIL; this design makes no promise of parallel Python execution or
hard real-time scheduling. Short locks protect queue/mailbox transitions and
are never held across device or storage I/O.

One process-lifetime [ownership guard](cura_receiver/platform/linux_process_ownership.py)
is acquired before configuration/state loading or hardware access. Otherwise two
instances could load the same airtime state before GPIO exclusivity detected the
conflict. Supported launches share a Linux network namespace and use the installed
entry point; independent namespaces and unrelated hardware tools do not share
this guard.

## Acceptance and durability

[`ProtocolIngress`](cura_receiver/protocol_ingress.py) authenticates and validates
copied packet bytes, then reserves one bounded queue slot for a reading and its
reception record. An accepted ACK means that this RAM reservation succeeded
while persistence admission was available. **It does not confirm a SQLite commit.**
A crash or power loss can therefore lose an acknowledged reading before persistence.
Waiting for a journal or database commit before ACK would require a different
latency/durability design.

The queue transfers complete immutable objects by reference. Reservation secures
capacity; publication makes the object visible. The producer publishes terminal
facts after acceptance or a response attempt even when later radio work fails.
The persistence owner frees a FIFO prefix only after durable completion or exact
reconciliation. A separate admission snapshot closes new acceptance during storage
failures without invalidating already owned work.

The communicator keeps no duplicate-message history. Every admitted reading
reception, including repeats, gets its own immutable record. Only authenticated
packets can produce reception records; pre-authentication failures remain radio
observations or health counts. [Reading persistence](cura_receiver/reading_persistence.py)
classifies against the first transport domain and complete decoded payload, then
the canonical sample. Neither reception history nor raw exception bytes are
classification authority. ACK suppression by airtime policy is independent of
valid reading acceptance.

Transport identity is `(node_id, message_id)` in `received_messages`.
`reading_messages` extends it with the first complete decoded reading, even when
that candidate conflicts with an existing sample. Its `is_canonical` partial
unique index selects at most one message per `(node_id, sample_id)`. This keeps
a later reuse of a conflicting message ID detectable without copying payloads
into a separate canonical table. Other message types can have other subtypes.

`packet_receptions` owns reception identity, monotonic provenance, radio/timing
facts and processing outcomes. Its one-to-one exception extension retains raw
authenticated rejection/conflict evidence. Malformed, unsupported, wrong-direction
and retry-later receptions do not establish transport authority; in particular,
a reflected ACK cannot occupy an uplink ID. Normal repeats retain reception facts
without raw copies. Field definitions and constraints belong to the
[schema manifests](schemas/receiver_entities.json).

## Persistence and recovery

SQLite uses WAL and `synchronous=FULL`, exact metadata/schema-fingerprint/group
compatibility checks, and a single owner. Batches amortize write cost; explicit checkpoints avoid surprise checkpoint
work at connection close or arbitrary commit boundaries. See
[`checkpoint_maintenance.py`](cura_receiver/checkpoint_maintenance.py) for commit
coverage and scheduling, and [database maintenance](deploy/README.md#database-maintenance) for creation and
restore procedures. Runtime never creates, migrates, or silently replaces a database.

Readiness prepares required SQL operations, checks durability/FK settings, inspects
only bounded operational state and necessary provenance, and commits the actual
receiver-instance start. Historical integrity/FK scans are explicit offline
maintenance, not startup or transient-recovery gates. Fresh database initialization
still performs full validation. Indexed lifecycle ordering permits historical
ordinal gaps. This bounds the set of readiness operations, not B-tree access,
WAL recovery or kernel I/O latency.

Usability is not a certificate of historical integrity. Queries must not hide an
existing transport, reception or exception through an inner join to missing
related history. Missing authority needed for the current decision is an explicit
failure; an unrelated old orphan is not an admission failure. Detected corruption
and contradictory current effects remain failures, never permission to repair or
overwrite history. Bounded state inspection must not be mistaken for the complete
set of rejected rows that a later recovery transaction must preserve.

[Ordinary persistence](cura_receiver/ordinary_persistence.py) freezes derived values
before attempting a commit. A lost commit result retains the lease until exact rows
can be compared. Replaying the same identity with different content is a correctness
failure, not permission to overwrite it. Item-local poison can be isolated and
quarantined; global I/O faults retain the original work and close admission.

[Shared recovery](cura_receiver/persistence_recovery.py) uses paced retries and
requires the same storage-usability checks, resolution of pending effects, and any required
checkpoint before reopening admission. Corrupt/incompatible storage needs operator
recovery. Same-process reopen remains bound to the original database file: restoring
older history requires stopping the receiver and restarting through normal validation.

The [control channel](cura_receiver/persistence_control_channel.py) serializes state
and lifecycle operations separately from the ordinary queue. A caller timeout cannot
undo a possibly issued commit or preempt kernel I/O. Reconciliation observes the exact
request through the same channel before another mutation can proceed.

## Airtime across restarts

[`TxAirtimePolicy`](cura_receiver/tx_airtime.py) uses a bounded
[fixed-charge ledger](cura_receiver/airtime_ledger.py) with monotonic expiration.
It charges before TX, refunds only definite non-starts, and retains uncertain TX
charges. Required saves run after RX rearm and gate further ACKs until confirmed.
Conservative grouping reduces disk writes on the radio path at the cost of unused
capacity after small saves and repeated restarts.

RTC provenance and airtime share one complete-state
[owner](cura_receiver/communicator_state_owner.py). Immutable receipts identify
individual save outcomes across later generations; RTC and airtime must not
independently overwrite each other's snapshots. Unknown commits block mutations
until exact-byte reconciliation, rather than generation equality alone, resolves them.

Startup restores only elapsed time it can justify and reserves capacity for possible
unsaved transmissions. Missing/corrupt history or unusable time gets a full-budget
conservative ledger; absence is never interpreted as an empty transmission history.
Unsupported formats or policy changes require a complete uninterrupted no-TX window
before atomic archive/replacement. A restart loses that silence proof. Clean shutdown
does not exempt the next startup from recovery.

[Known-empty commissioning](cura_receiver/airtime_commissioning.py) is the explicit
exception: database creation can authorize empty history with a single-use token.
Startup consumes it atomically with the first state. The authorization asserts
knowledge of actual transmitter history; a fresh database alone cannot establish it.
It establishes no RTC trust. The [deployment guide](deploy/README.md) owns preparation.

## Time and timestamps

Receiver instance identity changes on every process start; Linux boot identity
scopes monotonic time. Event rows store monotonic timestamps and stay immutable.
[Clock observations](cura_receiver/time_observations.py) provide trusted UTC samples;
[correlation](cura_receiver/clock_correlation.py) derives UTC within one receiver
instance, retaining the source observation. Clock steps fence correlation rather than
retroactively changing stored events. [Logical reading analysis](cura_receiver/logical_timestamps.py)
applies protocol anchor and sample-chain rules afterward.

[Runtime time](cura_receiver/runtime_time.py) separates Linux clock quality from RTC
health. Chrony-disciplined system time may be trusted when its estimated error fits
the policy; a recent NTP packet alone is neither necessary nor sufficient. RTC
holdover samples the proven RTC directly. Offline operation never repeatedly copies
one clock into the other to manufacture trust.

RTC refresh first durably invalidates old provenance, performs one write and verified
read-back, then commits new provenance. Live clock generations fence trust across
blocking/yielding actions. An uncertain write or state commit cannot publish trust.
The privileged [native helper](native/ds3231-set.c) is a production dependency with
fixed operations, not a test utility. [Chrony setup](deploy/CHRONY.md) owns deployment
privileges and the boot-scoped provisional RTC-to-Linux copy; receiver startup does
not wait for a network connection.

## Radio, diagnostics, and stopping

The [radio owner](cura_receiver/radio.py) separates protocol policy from the
[SX1262 command backend](cura_receiver/sx1262.py) and
[Linux device adapter](cura_receiver/platform/linux_radio.py). RX is usable only with
a confirmed complete uplink profile; ACK transmission installs its complete downlink
profile, then restores uplink RX. Bounded recovery retains command certainty and
reports a single episode rather than hiding uncertain effects behind retries.

[Telemetry](cura_receiver/communicator_telemetry.py) publishes authenticated receptions before their
separate diagnostic attempt. A shared telemetry gate fences clock expiry before
publication, including after blocking health probes and terminal cleanup. Accepted
packet completion retains its existing ownership and bypasses that gate.
Domain codecs retain bounded context, not arbitrary exception text. A recovery or
reconciliation episode retains its original failure; queue rejection never
recursively creates a diagnostic. Persistence cannot reliably
report through its own failed queue/database, so some failures leave only health,
quarantine, or best-effort service output. Diagnostics do not control radio safety.

[Health collection](cura_receiver/communicator_telemetry.py) samples hardware once
at a safe housekeeping boundary, before queue reservation. It preserves software
expectations at their original sampling time independently of the reads; probes
cannot change operational state, clock provenance or recovery policy. Normal clock
expiry at the publication gate does not relabel the prepared sample. The health row
and its referenced radio observation commit as one immutable unit, including on
retry. An outstanding packet reservation still belongs exclusively to that
exchange or its cleanup;
nested incident capture retains bounded RAM evidence and never reserves.

Radio [snapshots](cura_receiver/radio_observations.py) precede recovery mutations;
periodic reads also observe silence. Pilot rejection capture and bounded GPIO/command
history support investigation; the rejection counters remain permanent. Deferred
observations may be lost under pressure, with a cumulative loss count in health.
An allocated sequence or successful enqueue does not establish durability, and a
process loss can remove evidence before any later health reports it.

[Shutdown](cura_receiver/application_shutdown.py) stops new TX, attempts radio safety,
settles state, closes/drains the queue, and requests a clean marker only when its
preconditions hold. Signal callbacks record intent and wake the owner; they perform
no cleanup or lock acquisition. Repeated requests preserve the first deadline.
Queue closure also ends diagnostic admission. Clean-stop reconciliation remains
bounded, and correctness never depends on shutdown completing before process loss.

## Limits that matter when changing the design

- A host deadline cannot prevent an arbitrarily stalled SPI `SetTx` from reaching
  hardware late. The uncertain TX remains charged, but its reservation can then
  expire too early relative to the actual transmission. Rolling-window guarantees
  under arbitrarily delayed SPI require a physical completion/quiet-accounting fix.
- Safety conversions assume the configured monotonic rate envelope. Chrony's slew
  ceiling alone does not prove the entire platform envelope; the broader
  qualification/enforcement remains incomplete.
- RTC operation bounds depend on the controller, wiring, kernel/driver, and helper.
  A kill request is not proof that an ioctl or helper has finished. The
  [DS3231 procedure](hardware/ds3231/tests/i2c_faults/README.md) owns the adopted bound,
  recovery limits, and restoration checks.
- ACK timing and receive availability remain affected by scheduling, device I/O,
  and blocking housekeeping. Host tests and modeled delays do not measure physical
  RF timing, electrical power-loss durability, or deployed storage latency.

[Testing](tests/README.md), [hardware](HARDWARE.md), and
[deployment](deploy/README.md) each own their procedures; this overview does not
repeat their commands or individual assertions.

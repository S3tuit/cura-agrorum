# Receiver test support

This directory contains reusable test-only support for the receiver suites. It
is not part of the deployed receiver package. Production modules must never
import `tests` or anything below this directory.

Only add a shared helper when a production component and its tests expose a
concrete repeated need. Keep a helper local to one test module until sharing it
removes real duplication without hiding the behavior under test. Do not create
placeholder interfaces for later receiver components.

## Organization

The support tree grows by responsibility when its first real consumer exists:

```text
support/
  builders/       reviewed input construction grouped by production component
  coordination/   named thread barriers and bounded subprocess control
  fakes/          low-level implementations of existing production ports
  models/         small independent behavioral oracles
```

At this stage `fakes/os_clock.py` implements the already defined monotonic and
realtime clock capabilities, `models/persist_queue.py` is the independent
list-based oracle for the production queue's observable state machine, and
`coordination/threads.py` provides named checked workers plus bounded deadlock
detection for the queue's real-thread tests. `fakes/kernel_clock.py`,
`fakes/chrony.py` and `fakes/ds3231.py` implement the existing production time
ports. Each was introduced with its first runtime state-machine test. They
supply explicit queued outcomes and named call hooks; they never infer policy,
advance time or acknowledge persistence. SX1262 and other fakes still wait for
their production interfaces and component tests.

The ordinary-persistence suites now share reviewed literal entity inputs in
`builders/persistence.py`. Their real-file host fixture is local to the host
suite. Fault cases subclass the concrete production `SqliteTransactions`
begin/commit/rollback/checkpoint boundary locally; they do not emulate SQL.
The host and Pi crash families share
`coordination/persistence_crash.py` for named child readiness, bounded
termination/join and pre-recovery database/WAL/SHM preservation. Only the
parent-written test input file is deserialized; production never restores a
volatile queue from it.
`models/ordinary_persistence.py` is shared by reviewed model examples and the
generated state-machine suite. It uses primitive inputs, copied dictionaries
and FIFO lists, with no production classifier, binder, codec or builder.
The model covers mixed clock/profile/reading work, commit uncertainty, poison,
identity collisions and volatile-state loss on restart; exhaustive health and
quarantine byte contracts remain in their deterministic suites.

The state validator and control-transaction tests share reviewed immutable
inputs in `builders/persistence_control.py`; semantic expectations remain in
their tests. `coordination/persistence_worker.py` prepares dedicated component
files and propagates failures from actual worker threads with bounded joins.
`coordination/worker_crash.py` serves the host and Pi worker SIGKILL families,
preserving the file set before restart validation. Scheduling/race barriers
remain local to the test that names their safe boundary. The worker schedule
oracle remains local to `host/test_persistence_worker_schedules.py`, where two
reviewed primitive examples precede deterministic Hypothesis sequences.

`coordination/state_commit.py` loses a reply from an actual persistence control
call, after either installation or queued expiry. It is shared by complete-state
owner and airtime tests; it never emulates SQL or acknowledges a fabricated
state. The airtime component fixture is local to the host suite. Its SIGKILL
milestones run the actual policy, complete-state owner and persistence worker.
`models/tx_airtime.py` uses primitive list entries, rational duration bounds and
an independent history of possible transmission times. Reviewed literal examples
precede generated comparisons of grants, full-budget bursts, settlements, UTC
offset changes, trust loss, failed/unknown commits and repeated process loss.
Production codecs decode observed results only; they never supply model inputs
or expectations. The existing model import guard covers this module.
Each model entry retains its own rational-duration deadline. A separate
eight-hour real-SQLite test asserts the approved 14-second deferral and 74-second
ACK-gap bounds without calling that model, so shared availability mistakes do
not pass merely through differential agreement (review F-001).
The Pi lifetime/restart and two-phase reboot families share
`hardware/airtime_component.py` only after both needed the same real Linux time
adapters and SQLite worker fixture. Existing-history rows are explicit test
inputs. The reboot controller lives in `tools/test_airtime_reboot.py`, outside
the deployed package, and uses the existing hardware/destructive interlocks.

Pure time analysis keeps its merged-stream correlation oracle local to
`host/test_time_analysis_properties.py` and its independent per-anchor walk
local to `host/test_logical_timestamps.py`. Each has reviewed primitive examples
before generated comparisons. Observation/reading constructors also remain
local, while realtime-step tests reuse the existing `FakeOsClock`. This stage
added no shared time model or helper. The runtime extension keeps its primitive
network oracle, RTC episode builders and SIGKILL milestones local to
`host/test_runtime_time.py`, using the existing real queue and SQLite worker.
The laptop reference bridge remains in `hardware/time_reference.py`; its
independent numerical examples are host tests, not target slew evidence.

## Builders

A shared builder creates valid test input with reviewed literal defaults and
explicit keyword-only overrides. It must not read a clock, generate random
identity, consult mutable global state or infer values from the implementation
under test. Builders construct inputs only; expected encodings, SQL rows,
transitions and policy decisions remain explicit in the test or an independent
model.

Exact generation and schema tests should prefer direct or file-local
construction. A shared default must not conceal a newly required generated
field or make a schema test pass after an unreviewed interface change.

## Reference models

A reference model represents only the observable state needed by one family of
model-based tests. It favors direct, slow logic and test-local primitive values
over the production representation. It must not import or call production
algorithms, generated binders or codecs to calculate an expected result, and it
must not use a builder to derive expectations.

Add reviewed examples that establish the model's boundaries before comparing it
with production behavior or using it with Hypothesis. Add an automated import
boundary for `support/models` when the first model is introduced.

The `PersistQueue` model may reuse the stable `AdmissionResult` enum solely as
an output label. It must not import the production queue, quarantine-evidence
codec, entity binders or another production algorithm; its ordinary list and
primitive counters remain an independently implemented oracle.

## Thread and subprocess coordination

Thread tests coordinate at named, documented safe boundaries using explicit
events or barriers. A wall-clock timeout is only a bounded deadlock detector; it
does not represent receiver time and must not be used to choose a winner in a
race. Host timing behavior uses an injected manual clock, and tests do not use
`sleep()` to manufacture an interleaving.

The queue thread helper captures worker exceptions and performs bounded joins;
each test still declares its own named `Event` or `Barrier` arrival, inspection
and release boundaries. It never sleeps or chooses an interleaving.

Create subprocess support only
with the first crash test, using an explicit child-readiness boundary, a bounded
join, exact child-process termination and preservation of database evidence.
A simulated process crash must not be described as physical power-loss proof.

## Fakes

A fake implements an existing narrow production port, records or controls only
that boundary and contains no receiver policy. Production code receives the
port; tests instantiate the fake. A fake must not broaden the production
interface merely to make a test convenient.

Normal persistence tests use real temporary SQLite files. Fake SQL behavior or
a fake filesystem must not replace constraints and durability behavior that the
real schema can exercise.

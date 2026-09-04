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
detection for the queue's real-thread tests. Chrony, DS3231, SX1262, host-
health, filesystem and persistence fakes wait for their production interfaces
and component tests.

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

# Receiver testing

Status: this document defines the pilot receiver test suite. The generated
interface, SQLite-schema, database-initializer and low-level clock-boundary host
tests exist; most runtime host and Raspberry Pi tests described here will be
implemented with their production components.

## Purpose and authority

The receiver test suite verifies the behavior defined by
[`ARCHITECTURE.md`](ARCHITECTURE.md), the exact shared values and persistence
contracts in [`INTERFACE.md`](INTERFACE.md), the closed diagnostic catalogues in
[`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md), and the governing LoRa
protocol under [`../protocol/protocol-v2-lora/`](../protocol/protocol-v2-lora/).
Tests must exercise those contracts rather than infer a second behavior from
the current implementation.

Every test obligation below belongs to one of two suites:

- **Host tests** run on a developer laptop and in an ordinary Linux container.
  They contain the exhaustive policy, state-machine, fault-injection and SQLite
  matrices. They must not require Raspberry Pi devices, systemd, chronyd,
  privileged clock control or real elapsed-time sleeps.
- **Hardware tests** run locally on the target Raspberry Pi and verify facts
  that host fakes cannot establish: the deployed Python/SQLite/kernel stack,
  SPI and GPIO behavior, the DS3231, chronyd integration, systemd lifecycle,
  target storage and physical timing. They complement rather than repeat the
  exhaustive host matrices.

The complete end-to-end RF suite is deliberately deferred until all receiver
production code and every other host and hardware test in this document have
been implemented and pass. Component-level radio hardware tests may establish
the SX1262 fixture before then, but they must not be presented as an
end-to-end receiver result.

## Framework and organization

Both suites use `pytest`. Host property tests and model-based state-machine
tests use Hypothesis where generated sequences add coverage beyond reviewed
examples. The Raspberry Pi runs ordinary pytest directly because the receiver
is a native Python program on that host. `pytest-embedded` is optional only
when a later radio test also controls an ESP32 peer over serial; it is not the
receiver test runner.

The implemented layout is:

```text
receiver/tests/
  host/       exhaustive deterministic and fault-injection tests
  hardware/   target-Pi adapters, devices, deployment and timing tests
  support/    reviewed builders, reference models, fakes and subprocess helpers
```

The generation and schema tests are part of the host suite. Test
support may import production public interfaces, but production code must not
import test support. Shared golden inputs must be reviewed constants or the
checked-in protocol vectors; expected values must not be calculated by the
same implementation being tested.

The repository entry points are:

```text
make test-receiver-host
make test-receiver-hardware
make test-receiver-hardware-slow
make test-receiver-hardware-all
make test-receiver-hardware-destructive
```

`make test-receiver` remains an alias for the complete host suite. The ordinary
hardware target selects non-destructive fast cases. The slow target selects
long-running but non-destructive cases. The all target combines both safe
sets. Destructive cases require their own target, an explicit confirmation
option, dedicated test paths and any additional fixture-specific interlock.

From the repository root, install the receiver test dependencies and run the
ordinary validation loop with:

```sh
.venv/bin/python -m pip install -r receiver/requirements-test.txt
make test-receiver
```

Receiver Make targets disable third-party pytest plugin autoload and explicitly
load Hypothesis. `pytest-embedded` is not loaded by ordinary receiver tests. The
receiver-local `pytest.ini` also makes direct discovery from `receiver/`
host-only and enforces strict configuration, markers and expected-failure
behavior:

```sh
. .venv/bin/activate
cd receiver
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest
```

From the repository root, use the Make targets or pass an explicit receiver test
path; `-c receiver/pytest.ini` alone does not constrain pytest's initial search
root.

Register and enforce these pytest markers strictly:

- `hardware`: requires the target Raspberry Pi or its deployed Linux stack;
- `slow`: deliberately unsuitable for the ordinary validation loop;
- `destructive`: may change system time, write the RTC, reboot the Pi, disturb
  a service or exercise a bounded failure filesystem;
- `rf_peer`: requires the separately controlled radio peer and RF fixture.

Hardware tests run serially. They must fail clearly when an explicitly selected
fixture is missing rather than silently turn a requested hardware run into a
host-only success. They use a dedicated receiver-group configuration, service
name, database and storage root and must never modify pilot data. A hardware
fixture that changes device or host state must restore it in teardown; an
uncertain restoration is a test failure and stops dependent tests.

The hardware entry points pass the required explicit hardware option and verify
that pytest is running on a Raspberry Pi. The hardware package initially
contains only these safety and collection rules; component tests arrive with
their production adapters.

Destructive tests additionally require a dedicated absolute directory containing
`.cura-receiver-test-root` with the exact line
`CURA AGRORUM RECEIVER TEST ROOT`, followed by this explicit invocation:

```sh
receiver_test_root=/absolute/path/to/dedicated/receiver-test-root
mkdir -p "$receiver_test_root"
printf '%s\n' 'CURA AGRORUM RECEIVER TEST ROOT' \
  > "$receiver_test_root/.cura-receiver-test-root"
make test-receiver-hardware-destructive \
  CONFIRM_RECEIVER_DESTRUCTIVE=YES \
  RECEIVER_TEST_ROOT="$receiver_test_root"
```

Individual destructive fixtures add their own configuration, service, device
and restoration interlocks when they are implemented. RF-peer execution remains
deferred and has no Make target yet.

## Test implementation rules

- Each bullet in this document is a required test or a cohesive parameterized
  test family. During implementation, split a bullet when separate failures
  need clearer identities, but do not hide unrelated behavior in one large
  orchestration test.
- New tests may be added whenever implementation, review or a discovered bug
  exposes an uncovered contract or regression. Record the reason close to the
  new test or in this document.
- Deleting a listed or implemented test, permanently skipping it, weakening its
  assertions or replacing it with materially narrower coverage must be
  carefully evaluated and explicitly agreed with the user first. Passing code
  is not by itself a reason to remove a test.
- Each test must have a concise description in the form of a comment right
  above its implementing code.
- Host tests use injected monotonic clocks, explicit events, barriers and
  bounded subprocesses instead of real sleeps. Thread tests control ordering at
  documented safe boundaries and assert externally relevant state rather than
  relying on a fortunate scheduler interleaving.
- Use real SQLite temporary files for normal persistence semantics. Inject
  faults only at a narrow persistence/backend seam for outcomes that cannot be
  produced reliably, such as an unknown commit result. Never mock SQL into
  behavior that the real schema would reject.
- Crash tests run the production component in a child process and terminate
  that process at a named boundary. A simulated crash or injected torn value
  must not be described as proof of physical power-loss behavior.
- Timing assertions on the host use exact virtual boundaries. Hardware timing
  assertions use documented asymmetric tolerance and record the raw samples;
  they do not assume laboratory-grade determinism from Linux scheduling.
- A failed persistence test preserves the database, WAL and shared-memory files
  together when they are evidence. Hardware failures additionally retain
  relevant configuration, service journal, device trace, test seed and host
  metadata without retaining secret keys.

The current low-level `FakeOsClock` is manually controlled. Reading monotonic or
realtime never advances either value. Tests call `advance_elapsed_us()` to move
both clocks together or `step_realtime_us()` to move realtime independently;
monotonic time cannot move backward. It contains no time policy, sleeps or
automatic scheduling behavior.

Reusable builders, reference models, named barriers and bounded subprocess
helpers are intentionally not implemented in advance. Add each with the first
production component and test family that establishes its actual input,
observable-state or coordination boundary. The organization and independence
rules are in [`tests/support/README.md`](tests/support/README.md). Chrony,
DS3231, SX1262, host-health and persistence fakes likewise wait for their
production ports.

## Protocol ingress

### Host tests

- **Reviewed valid current reading:** Feed a checked-in authenticated current-reading frame through the production ingress path and verify the decoded transport and application identities, processing result, selected ACK and immutable pre-TX profile fields.
- **Reviewed valid backlog reading:** Repeat the valid path with a backlog domain and prove that domain selection changes only the protocol-defined behavior while `message_id` and `sample_id` retain their distinct meanings.
- **Validation-order matrix:** Parameterize failures at PHY/header length, node lookup, authentication, direction/control and exact body-structure stages and assert the first applicable protocol result and silence/response policy.
- **Untrusted clear header:** Supply unknown node IDs, invalid clear-header encodings and tampered authenticated-data bytes and verify that no unauthenticated value reaches node state, queue admission or ACK construction.
- **Authentication failure:** Exercise wrong keys, modified ciphertext and modified tags and require silence, the exact processing result and no application-candidate reservation.
- **Authenticated malformed reading:** Authenticate structurally invalid bodies at every constrained field or flag and verify the protocol-selected malformed response without decoding absent or invalid application values as accepted data.
- **Wrong-direction packet:** Authenticate a downlink ACK domain received as uplink, require `WRONG_DIRECTION`, no response under every queue state, and at most the permitted profile-only admission attempt.
- **Unsupported authenticated input:** Exercise supported framing with an unsupported control, domain or version and verify the exact rejection ACK domain/status and complete profile content.
- **Deterministic ACK reconstruction:** Reconstruct an ACK repeatedly from the same authenticated uplink and outcome with no receiver history and require byte-for-byte equality, including after unrelated messages have been handled.
- **Nonce identity:** Verify every ingress authentication and ACK construction uses `node_id || message_id || domain` and never incorporates `sample_id`; use deliberately different transport and sample values.
- **Acceptance ordering:** Prove an authenticated valid reading becomes accepted only after `PersistenceAdmissionState.AVAILABLE` and successful exact pair reservation, and that airtime suppression cannot reverse that acceptance.
- **Retry-later selection:** Cover persistence unavailable and queue-full results for every response-eligible authenticated packet, requiring no acceptance, no cached outcome and the deterministic `ACK_RETRY_LATER_DOWNLINK` response when airtime permits.
- **Profile completeness:** Verify all stable profile inputs and the exact selected ACK frame are fixed before ACK selection, then require one complete immutable typed profile containing the terminal TX result and optional timestamps to be published against the already reserved queue slot.
- **Ingress properties:** Use Hypothesis to generate valid and invalid headers, bodies and authenticated frames around every integer and length boundary, comparing the result with a small independent validation-order model.

### Hardware tests

- **Target runtime compatibility:** On the Pi, import the installed production package and process the reviewed current, backlog and invalid frames without radio hardware, proving compatibility with the deployed Python, cryptography and generated-codec versions.
- **Target crypto latency characterization:** Repeatedly authenticate and construct ACKs from reviewed frames on an otherwise idle Pi, record latency percentiles and assert a generous configured watchdog bound; this measures the platform without redefining protocol correctness.
- **Target load characterization:** Repeat the in-memory ingress benchmark under a documented bounded CPU and I/O load and record whether the receiver still leaves adequate margin for the packet-to-ACK deadline; exact policy boundaries remain host tests.

## Radio

### Host tests

- **Initialization profile:** Use a transcript-recording fake SX1262 backend and verify initialization installs the complete protocol `UPLINK_RX_PROFILE`, clears or accounts for IRQs, confirms `SetRx` and only then enters `RX_SINGLE`.
- **Initialization terminals:** Distinguish absent/unreachable resources as `HARDWARE_MISSING` from reachable hardware whose bounded initialization exhausts as `INITIALIZATION_FAILED`; neither state may transmit or transition again in the same process.
- **RX event classification:** Parameterize RxDone, header error, CRC error, TX-timeout and unexpected IRQ combinations and verify exact clearing, packet-copy and recovery behavior without treating ordinary IRQ outcomes as diagnostics.
- **Pi-owned packet snapshot:** Mutate the fake radio buffer immediately after `ReadBuffer` and prove authentication, profiling and persistence use the independent bytes copied before later radio commands.
- **Response-free RX rearm:** For every silent or airtime-suppressed outcome, verify the complete receive profile is restored and `SetRx` confirmed before `RX_SINGLE` is asserted.
- **ACK profile transition:** Verify the exact order `WriteBuffer`, complete inverted-IQ `ACK_TX_PROFILE`, allowance consumption, `SetTx`, terminal IRQ handling, complete normal-IQ boosted `UPLINK_RX_PROFILE`, and confirmed `SetRx`.
- **Definite pre-SetTx failure:** Fail every operation before `SetTx` can take effect and require no `TX_ACTIVE`, reclaimed tentative allowance when permitted, no false transmitted result and bounded restoration or recovery.
- **Started or uncertain SetTx:** Return confirmed-start and uncertain command outcomes and require `TX_ACTIVE` semantics, retained airtime charge and recovery before any new `SetRx` under a possibly partial profile.
- **Missing or delayed TxDone:** Exercise no terminal IRQ, a terminal IRQ at the deadline and one after it; verify terminal profiling, charge retention, recovery reason and bounded exit without an unbounded wait.
- **BUSY and SPI failures:** Inject BUSY timeouts and SPI failures at every semantic operation and assert command-effect certainty, one recovery episode, exact diagnostic root cause and no continuation under an assumed mode.
- **Soft and hard recovery:** Cover soft success, soft failure followed by reset/full-initialization success, absent hardware during recovery and final exhaustion; require exactly one episode diagnostic and a confirmed receive profile before success.
- **Event immediately after recovery:** Raise DIO1 as recovery confirms `SetRx` and prove the event becomes `RX_EVENT_PENDING` instead of being cleared as stale.
- **Diagnostic catalogue enforcement:** Attempt every allowed radio operation/error/context family and representative undefined combinations, requiring exact fixed context bytes for allowed cases and construction failure for undefined cases.
- **Controlled radio shutdown:** From each non-terminal state, request shutdown and verify new TX suppression, conservative active-operation termination, safe configured radio state and terminal `SHUTDOWN` without relying on later cleanup for correctness.
- **Radio state-machine properties:** Generate valid and faulted operation sequences against a small reference model and assert that `RX_SINGLE` always implies a confirmed complete receive profile and that terminal states have no outgoing transition.

### Hardware tests

- **Device and permission probe:** Open the configured SPI device and GPIO lines as the receiver service user, verify direction/edge configuration and report missing or inaccessible resources within the configured startup deadline.
- **Real initialization:** Reset and initialize the attached SX1262, install the complete uplink profile and confirm bounded entry into receive mode using production adapters.
- **Real BUSY behavior:** Measure BUSY assertion and release around representative commands, prove every wait is bounded and retain the command trace when a timeout occurs.
- **DIO1 timestamp path:** Trigger controlled radio IRQs and verify libgpiod delivers rising edges with ordered kernel monotonic timestamps that populate the expected profiling fields.
- **Normal-IQ uplink reception:** With the component RF peer, receive a reviewed payload under the exact protocol sync word/profile and verify length, bytes, IRQs and plausible RSSI/SNR without imposing exact RF-strength assertions.
- **Inverted-IQ ACK transmission:** Transmit a reviewed ACK and require the peer to receive its exact bytes only under the expected inverted-IQ profile.
- **Profile restoration:** Alternate bounded receive and transmit operations and prove normal-IQ boosted RX is restored after every ACK before the next uplink is accepted.
- **Radio timing characterization:** Measure SetTx-to-TxDone and receive-deadline behavior against calculated airtime and the documented asymmetric hardware tolerance, recording raw monotonic samples.
- **Hardware reset recovery:** Force a safe recoverable fault with a controllable fixture, require soft recovery or reset/full initialization as appropriate, and prove the final known state rather than only checking a return code.
- **Safe-state teardown:** End tests from RX, TX-adjacent, recovery and ordinary idle conditions and verify the module reaches the configured safe shutdown state; an uncertain state aborts later radio cases.

## PersistQueue

### Host tests

- **Constructor and slot bounds:** Reject capacities below one or above the pilot maximum of 500, construct both boundary values, and prove all backing arrays have fixed identity and length for the queue lifetime.
- **Exact entity-count limit:** Reserve at empty, one-below, exact and full boundaries with mixed entity kinds and verify every reservation consumes exactly one slot without byte charges or Python-object-size inspection.
- **Atomic measurement/profile reservation:** Require one token to own one slot for the complete typed pair and prove neither batch selection nor cancellation can expose or split a partial measurement/profile unit.
- **Profile-only reservation:** Verify response-eligible rejections and silent occurrences reserve one ordinary slot and preserve their distinct response policies outside the queue.
- **Reservation lifecycle:** Cover reserve, publish an existing immutable object, permitted pre-response cancel, double publish, double cancel, stale and foreign tokens, with invariant violations rejected deterministically.
- **Outstanding reservation visibility:** Prove a reservation counts immediately but the persistence consumer cannot claim it until publication.
- **Opaque publication:** Publish under full queue pressure and require the consumer to receive the identical object reference without a second capacity check, copying, serialization, field validation or kind-specific builder.
- **FIFO across entity kinds:** Interleave measurement/profile, profile-only, clock observation, health and diagnostic units and require global publication order with no priority, sampling, eviction or reordering.
- **Persistence admission gate:** Parameterize every unavailable state and prove new reservations return `PERSISTENCE_UNAVAILABLE`, already published work remains owned and synchronous control commands retain their separate path.
- **Admission-count matrix:** For every entity kind and result, verify exactly one matrix cell increments per reservation call, including a successful health request containing its own updated `RESERVED` count.
- **Non-recursive diagnostics:** Fail an ordinary entity admission and a diagnostic admission with queue-full and persistence-unavailable outcomes and require no diagnostic sequence allocation, no diagnostic construction and no second reservation.
- **Claim and disposition ownership:** Verify claimed units remain queue-owned through attempt, rollback, replay and quarantine work and are released only by confirmed or reconciled durable disposition.
- **Quarantine evidence:** Round-trip every allowed neutral value kind and queue-unit shape through canonical tagged JSON; preserve invalid field types, lengths and arbitrary-size integers exactly; reject unsupported/cyclic values, duplicate or unknown keys/tags, noncanonical JSON/base64/integers, excessive depth/nodes/output, and any attempt to interpret decoded evidence as a production entity.
- **Poison retention:** Make evidence encoding fail for an unsupported or over-limit poisoned value and prove the active FIFO head and following entries remain claimed and owned rather than being acknowledged or bypassed.
- **Close and wake semantics:** Cover empty and nonempty closure, blocked producer/consumer wakeup, publication racing closure and shutdown drain without lost items or indefinite waits.
- **Queue state-machine properties:** Generate reservation, publication, claim, rollback, durable acknowledgement and close sequences against an independent entity-count/FIFO model and assert count, token and ownership invariants after every step.
- **Deterministic SPSC interleavings:** Use named barriers/events and bounded joins with real `threading.Thread` producer and consumer calls to cover admission-state publication versus reservation, publication versus claim, acknowledgement versus a new reservation, retry, closure and clear/recheck wakeup ordering without timing sleeps.

### Hardware tests

- **Configured-capacity construction:** Construct the production 500-slot queue on the Pi, verify its fixed backing lengths and exercise every slot without recursive object-size or RSS characterization.
- **Sustained handoff:** Run communicator-style publication and persistence-style batch claims concurrently for a bounded interval and verify FIFO identities, stable accounting and no deadlock on the target interpreter.
- **Queue-pressure recovery:** Pause the consumer until the 500-slot entity bound rejects new work, resume it and prove admission recovers without eviction, reordering or a recursive diagnostic storm.
- **Target queue close/drain:** Close a populated queue during sustained handoff and verify existing reservations may finish, every published entity drains and both threads terminate without hanging. Process-signal and owning-service shutdown policy belong to later lifecycle integration tests.
- **Target lock latency:** Measure reservation, publication, claim and acknowledgement latency under sustained SPSC contention on the Pi and record percentiles plus maxima. The bounded run must finish without deadlock, but Linux scheduling supplies no hard per-call latency ceiling; representative external CPU and SQLite load belong to later integration tests.

The `PersistQueue` implementation stage owns the ring, typed handoff values,
evidence codec and queue-only coordination assertions above. Protocol response
selection, admission-matrix updates and non-recursive diagnostic policy remain
future communicator work; SQLite dispositions, the persistence worker and
process-signal lifecycle behavior remain future persistence/integration work.
Recursive heap sizing and RSS-based admission are deliberately not test
requirements for the 500-slot pilot unless later measurements reopen that
decision.

## Persistence

### Host tests

- **Generated schema freshness:** Run receiver generation in validate and check modes, verify checked-in outputs are current and verify the exact `schema.sql` SHA-256, schema version and SQLite application ID.
- **Database initialization:** Create a file database from packaged `schema.sql`, bind the configured group metadata and verify strict tables, immutable catalogues, foreign keys, integrity checks and singleton constraints.
- **Startup identity validation:** Parameterize missing, malformed, newer, older, gapped and fingerprint-mismatched metadata plus a different `group_id`; require `UNAVAILABLE_INCOMPATIBLE_SCHEMA` without modifying or importing secrets into the database.
- **WAL/FULL enforcement:** Verify a usable connection actually enters WAL mode and reports `synchronous=FULL`; injected refusal of either setting prevents ordinary admission.
- **Receiver-instance start:** Insert exactly one durable lifecycle row with increasing database-local ordinal before publishing `AVAILABLE`, and reject identity collision or malformed Linux boot identity.
- **Reading classification matrix:** Cover `FIRST_SEEN`, `RETRANSMISSION`, `DUPLICATE_SAME_CONTENT`, `DUPLICATE_CONFLICT` and `MESSAGE_ID_CONFLICT` with exact canonical/noncanonical rows and occurrence profiles.
- **Conflict evidence:** Prove canonical rows are never replaced, every admitted occurrence keeps its profile evidence and persistence creates no conflict `DiagnosticV1`.
- **Atomic pair transaction:** Fail after each reading-message/profile write boundary and verify the complete measurement/profile unit is either wholly committed or remains queue-owned with no partial durable effect.
- **Batch commit ownership:** Exercise successful commit, definite pre-commit failure and `OUTCOME_UNKNOWN`; only confirmed commit or exact reconciliation may acknowledge units.
- **Ordinary identity replay:** For every queue entity identity, reconcile absent, exactly equal and differing stored rows; insert absent work, accept exact rows unchanged and close as incompatible on collision without update or quarantine.
- **Measurement replay side effects:** Validate each stored persistence classification against its required reading-message and canonical-sample relation, including impossible partial ownership.
- **Frozen health enrichment:** Sample a complete `ReceiverHealthV1` once, force retry and ambiguous commit, and prove host fields and both sampling timestamps are never resampled.
- **Communicator-state precedence:** Cover missing row, SQL envelope/digest corruption, valid-digest unknown version, supported structural corruption and pure policy mismatch in the normative exclusive order.
- **Communicator-state generations:** Test generation-one creation, next generation, exact idempotent replay, stale generation, generation gap, conflicting bytes and unknown commit reconciliation.
- **Corrupt-state recovery:** Preserve every exact rejected singleton row and install the synthetic worst-case generation-one ledger atomically before TX can become eligible.
- **Unsupported/policy recovery:** Enforce the complete conservative rolling-window wait, restart-reset wait and atomic archive plus empty generation-one replacement without decoding or repairing the rejected state.
- **Closed storage-failure classifier:** Map low space, disk full, corruption, incompatibility, global/transient I/O, entity-specific reproducible faults and unrecognized errors to their exact distinct paths, with unknown results failing closed as `UNAVAILABLE_IO`.
- **Retry scheduler:** Verify finite per-attempt SQLite waits and interruptible 250 ms doubling backoff capped at 5 seconds, no maximum retry count, no early retry after unrelated wakeup and recovery only after validation plus commit/reconciliation.
- **Retained frozen batch:** On global failure, retain original immutable units and every derived classification/enrichment value without reconstruction or poison classification.
- **Corruption preservation:** On corruption, close admission and preserve database, WAL and shared-memory files together without delete, truncate, rebuild or silent epoch replacement.
- **Poison isolation:** Require an entity-specific failure to reproduce alone before quarantine, then store exact canonical `QuarantineEvidenceV1` bytes and provenance durably before later valid units proceed.
- **Non-quarantinable clock boundary:** Reproduce an isolated `ClockObservationV1` failure and require retained queue head plus incompatible admission, with no bypass or quarantine.
- **Quarantine reconciliation:** Cover confirmed, definite-failure and ambiguous quarantine commits, accepting only an exactly matching frozen row and retaining the active lease otherwise.
- **No automatic retention:** Populate every retained table and prove normal operation and checkpointing never delete active-epoch canonical, profile, health, diagnostic or quarantine history.
- **Integer storage boundaries:** Persist every queue-bound unsigned value at `INT64_MAX` and reject the next value before SQL silently changes its meaning.
- **Subprocess crash matrix:** Terminate a child before transaction, during writes, before commit, after commit may have run and after acknowledgement and verify startup recovery/reconciliation for each durable outcome.
- **Persistence state-machine properties:** Generate batches, failures, retries, identity collisions and restarts against a small durable reference model and compare queue ownership, admission state and committed identities after each action.

### Hardware tests

- **Deployed SQLite capabilities:** On the Pi, record Python and SQLite versions/compile options and verify the production connection establishes all required pragmas and integrity checks on the target filesystem.
- **Target transaction workload:** Persist realistic mixtures of every entity kind with production encodings and batch sizes, then verify exact rows, foreign keys, integrity and queue acknowledgements.
- **FULL versus NORMAL benchmark:** On an isolated test database on the deployed storage, measure throughput, commit-latency percentiles, queue growth, WAL growth and checkpoint stalls for both modes while retaining `FULL` as the pilot correctness setting.
- **Checkpoint behavior:** Grow WAL beyond configured thresholds under concurrent publication, run bounded checkpoints and prove control work and queue growth are not ignored indefinitely.
- **Process-kill recovery:** Kill the persistence test process at named transaction and acknowledgement boundaries, restart against the same files and require exact replay or reconciliation without duplicate mutation.
- **Bounded full-filesystem recovery:** Use a dedicated bounded test filesystem or loop device, never the pilot filesystem, to produce low-space and disk-full outcomes and verify rollback, retained work, closed admission, paced recovery and later success.
- **Read-only and transient storage recovery:** Change only the isolated fixture's access or mount state, verify `UNAVAILABLE_IO` and continued retained retries, then restore it and require validation plus successful commit before `AVAILABLE`.
- **Corrupt-artifact handling:** Corrupt a copied test database or WAL, verify the service preserves all artifacts and refuses ordinary admission, then recover only through the explicit test maintenance procedure.
- **Physical interruption:** When a safe externally controlled power fixture exists, interrupt the Pi during selected WAL/`FULL` operations and compare recovery with the documented SQLite assumptions; until then this remains a destructive manual test and subprocess/reboot cases must not claim equivalent proof.

## Concurrency

### Host tests

- **Exclusive resource ownership:** Instrument radio, configuration and SQLite adapters with thread identity checks and prove only the communicator touches radio/live policy while only persistence performs filesystem and database I/O.
- **Radio path independence:** Block an ordinary SQLite transaction at controlled barriers and verify the communicator can continue bounded RX processing, reserve available queue capacity and suppress or select ACKs without waiting for that transaction.
- **Control wakes idle persistence:** Submit a synchronous control command while persistence is idle and while it is in interruptible backoff and require immediate wakeup without polling.
- **Submission versus batch dispatch:** Control both sides of the scheduler-lock ordering point and prove submission either runs before the ordinary attempt or immediately after that one attempt reaches its safe boundary.
- **No transaction preemption:** Submit control and shutdown work during every open SQLite phase and prove execution waits for commit, rollback or reconciliation rather than entering filesystem I/O concurrently.
- **Fairness in both directions:** Sustain sequential control commands and ordinary batches and require at most the documented one-command bypass followed by one due ordinary attempt, with neither path starved.
- **Cancellation before effect:** Expire a queued and a running-precommit control command and prove atomic cancellation prevents it from crossing `COMMIT`.
- **Unknown post-commit outcome:** Expire a mutating command after `COMMIT` may have run and require ordered `COMMIT_MAY_HAVE_RUN` handling plus exact serialized reconciliation before later conflicting work.
- **Lost-wakeup races:** Exhaustively gate clearing/rechecking the shared wakeup against concurrent queue publication, control insertion and shutdown and require every kind of work to be observed.
- **Shutdown races:** Signal shutdown during idle, radio handling, queue reservation, batch transaction, backoff, control command and reconciliation and verify bounded ownership-preserving termination.
- **Immutable cross-thread values:** Attempt to reuse or mutate published entities, state snapshots, admission snapshots and control requests and verify the receiving thread observes frozen values or rejects the violation.
- **Deterministic stress schedules:** Generate long operation schedules with explicit barriers and repeatable seeds, asserting safety invariants continuously and emitting the minimal recorded schedule on failure.

### Hardware tests

- **Pi scheduling under load:** Run both production threads with controlled CPU and storage load and verify progress counters, bounded control completion and absence of deadlock for a documented duration.
- **Persistence stall isolation:** Stall a real target-storage transaction or checkpoint and measure that communicator in-memory work continues while queue capacity remains, without claiming hard real-time scheduling.
- **Control latency at safe boundaries:** Measure control-command completion when submitted during idle, an ordinary commit and retry backoff, proving observed ordering matches the cooperative contract.
- **Signal-under-load behavior:** Send the real supervisor termination signal during active queue and persistence work and require bounded exit, correct clean-stop eligibility and preserved failure artifacts.
- **Long-run race soak:** Repeatedly publish mixed entities, request state commits and trigger bounded recoverable storage failures on the Pi, recording seeds and thread stacks for any timeout; mark the case slow and keep it serial.

## Time policy and timestamp analysis

### Host tests

- **Independent quality axes:** Exercise every meaningful `SystemTimeQuality`/`RtcHealth` combination, including `NETWORK_SYNCED + MISSING`, and prove persisted last-observed values never become current startup authority.
- **Conservative duration conversions:** Check integer equality and one-unit boundaries for minimum-wait lengthening and maximum-lifetime shortening, including the documented 3,613.32-second and 29.889-second examples.
- **Chrony result validation:** Reject unavailable, stale, unsynchronized, unreliable, structurally invalid and arithmetic-overflow results and accept only a fresh bounded result from the configured local socket contract.
- **Network-error arithmetic:** Verify checked conservative formation of absolute remaining correction, half root delay, root dispersion and sampling margin without cancellation from sign.
- **Trust hysteresis:** Cover at/below 35 seconds, the open 35-to-40-second band, exactly 40 seconds and above 40 seconds from each current quality.
- **Poll and observation deadlines:** Verify one-minute chrony, three-hour online, one-hour holdover and three-hour RTC-refresh caps shorten whenever the calculated UTC-error horizon expires first.
- **Network observation bracket:** Parameterize generation equality, monotonic ordering, bracket width, tracking freshness and kernel metadata around `adjtimex()`; accept the expected no-`rtcsync` `TIME_ERROR`/`STA_UNSYNC` pair when chrony is otherwise valid.
- **Quality ABA rejection:** Change time quality away and back during a sampled operation and prove the changed `clock_state_generation` invalidates the result despite equal final enum values.
- **Direct RTC observation:** Verify whole-second midpoint, half-second representation term, converted half-bracket, fixed margin, durable verification uncertainty and pre-read RTC drift are combined exactly.
- **Holdover age limits:** Reproduce the documented approximately 24.46-day cadence and 39.93-day absolute examples, then check equality, next-unit and nonzero-read-bracket boundaries.
- **RTC refresh ordering:** Exercise derive, write, read-back, generation recheck and durable provenance commit, with failures/crashes after every step and no usable provenance before acknowledged commit.
- **RTC source threshold:** Require the stricter five-second source-error bound both at refresh start and before commit, independently of the broader network-trust threshold.
- **Clock-step state machine:** Cover boundary publication failure, command rejection, confirmed submission, unknown command outcome, stable-time polling, deadline and bounded retry without blind resubmission after an unknown result.
- **Step-boundary FIFO:** Prove no explicit step precedes complete boundary publication, later ordinary entities cannot overtake it, and an isolated boundary failure closes admission without quarantine.
- **UTC correlation segments:** Select the latest preceding or permitted first later trusted observation within one receiver instance, reject cross-instance/boot correlation and permanently exclude the half-open step-discontinuity gap.
- **Process-start boundary:** Verify a durable new instance start closes the previous instance and permits later-observation backfill only to its own start when no explicit step boundary intervenes.
- **Realtime-step immunity:** Step the fake realtime clock forward and backward while monotonic deadlines, retry waits and live event intervals continue unchanged; bounded slew affects only conservative elapsed-rate calculations.
- **Logical direct anchors:** Choose the earliest anchor-eligible accepted current occurrence with derived `RX_DONE` UTC, enforce `run_ms + Tair <= 30,000 ms` at the exact boundary and reject conflict classifications.
- **Logical extrapolation:** Exercise consecutive sample, deep-sleep, previous-metric and identity-lifetime requirements in both directions, break invalid chains and never replace already materialized analysis output.
- **Time-policy state-machine properties:** Generate quality changes, observations, steps, RTC operations, process starts and events against an independent correlation model and shrink any unsafe UTC assignment.

### Hardware tests

- **Linux boot and monotonic identity:** Verify `/proc` boot identity is stable across receiver-process restarts, changes across Pi reboot and scopes observed monotonic values through durable receiver instances.
- **Real chrony tracking adapter:** Query only the configured local Unix socket, parse real tracking output and compare supported fields with independent `chronyc` evidence without granting arbitrary command construction.
- **Real adjtimex sampling:** Capture production monotonic brackets and `adjtimex()` metadata under synchronized chronyd with `rtcsync` disabled, including the expected kernel unsynchronized status pair.
- **DS3231 read contract:** Read the bound RTC through the production adapter, verify bounded bracket and whole-second representation, and classify missing, invalid and I/O/deadline outcomes with a safe fixture.
- **Offline holdover startup:** Start the isolated test service without network synchronization after valid durable RTC provenance and require direct RTC-based holdover without waiting for the network or periodically copying RTC time into Linux.
- **Unproven RTC startup:** Remove or invalidate only the test provenance and prove a plausible RTC bootstrap value does not establish `RTC_HOLDOVER` authority.
- **RTC write/read-back:** Under destructive opt-in, save the fixture state, perform a network-qualified RTC refresh, verify read-back and durable provenance ordering, then restore and independently verify host/device state.
- **Maximum slew-rate validation:** Compare disciplined `CLOCK_MONOTONIC` with an independent elapsed-time reference during configured maximum positive and negative slew and validate the 3,700 ppm receiver bound; this is slow and destructive.
- **Explicit step integration:** Under destructive isolation, execute forward and backward chrony-step episodes and prove complete boundary publication before the command, FIFO persistence ordering, permanent correlation gaps and recovery only at the first later trusted observation.
- **Deployment time-writer audit:** Verify effective chrony configuration has the declared slew/leap policy, no automatic-step directive, no `rtcsync`/`rtcfile`, command port disabled and no competing enabled system-clock or RTC writer.

## Receiver TX-airtime policy

### Host tests

- **Bucket boundary aging:** Exercise exact start/end, partially overlapping oldest bucket, complete expiration and long-idle bulk reset while retaining a bucket until its full interval is conservatively outside the rolling window.
- **Cached-total reconstruction:** Load valid and inconsistent ledgers, recompute `total_used`, reject checked overflow or mismatch and prove ACK admission updates totals without scanning the full ring.
- **Grid continuation:** Continue the latest unexpired durable logical bucket across process restart, top up only that bucket when eligible and never relabel an earlier process's charge.
- **Grant headroom:** Check bucket and global headroom equality/one-unit boundaries and require a durable current-process increment before any allowance becomes spendable.
- **Spend and settlement:** Tentatively spend an ACK charge, settle exact used airtime, reclaim only definitely unused allowance and atomically precharge a later bucket when budget permits.
- **SetTx certainty charging:** Parameterize definite pre-SetTx failure, confirmed start, uncertain command and missing terminal outcome; reclaim only the first and retain every possible transmission.
- **Crash before and after grant commit:** Prove a pre-commit crash enables no TX and a post-commit crash leaves the complete increment charged and unspendable to the replacement process.
- **Repeated crashes:** Generate consecutive process failures and show conservative precharges accumulate without exceeding bucket/global budget or reopening spent allowance.
- **Settlement failures:** Cover definite commit failure and unknown outcome, retaining the preceding authoritative generation and suppressing TX until exact reconciliation.
- **Time-trust loss:** Invalidate the grant's UTC/monotonic correlation and require frozen allowance until a new trusted correlation and state transition establish safe expiration.
- **Missing/corrupt history:** Start from generation zero, synthesize `[4, 8, 8, 8, 8]` seconds for pilot defaults and forbid TX until the checked generation-one state commits.
- **Unsupported/policy mismatch wait:** Start the complete conservative rolling-window wait only after TX is known disabled, restart the wait on process restart and permit empty-ledger replacement only after trusted UTC and the full wait.
- **Budget exhaustion semantics:** Accept and publish an otherwise valid reading when no ACK allowance exists, record airtime suppression and never change acceptance to retry-later.
- **Reference-model properties:** Generate bucket charges, grants, spends, time advances, trust changes, settlements and crashes and compare every decision with an independent continuous-window model.

### Hardware tests

- **Target monotonic grant lifetime:** Commit a test grant on the Pi, measure its shortened monotonic lifetime and prove it freezes at the logical bucket boundary rather than one arbitrary minute after commit.
- **Process-restart baseline:** Restart the isolated receiver process within one Linux boot and verify loaded charge is unspendable, a new increment is durably required and no persisted monotonic timestamp is reused.
- **Pi-reboot reconstruction:** Reboot with trusted RTC or network time and reconstruct unexpired charges from UTC; repeat without trusted time and require TX suppression.
- **Real-radio charge result:** With the component RF fixture, exercise one definite pre-SetTx failure and one confirmed/uncertain attempted transmission and verify durable settlement follows command certainty, not whether the peer observed the packet.
- **Continuous-window observation:** Run a slow, legally bounded sequence spanning bucket edges and confirm no set of attempted ACK transmissions exceeds the configured 36 seconds in any continuous 3,600-second observation period.

## Deployment and lifecycle

### Host tests

- **Configuration loading boundary:** Verify only persistence invokes the strict receiver-group loader, returns an immutable secret-bearing snapshot and never exposes keys through logs, health, diagnostics or profiles.
- **Startup ordering:** Gate every startup stage and prove storage/configuration/database validation and durable receiver-instance insertion precede ordinary admission, while initial time observation and communicator-state reconciliation precede radio TX.
- **Storage prerequisite:** Simulate missing, read-only and unusable required storage and require no radio operation; distinguish this from an otherwise usable database with missing communicator state, where RX may continue but TX fails closed.
- **Instance identities:** Generate a new receiver instance for clean restart, crash restart and Pi-reboot simulation, preserve or change Linux boot identity as appropriate and reset every process-local sequence.
- **Initial clock observation:** Require exactly one initial observation after current time/RTC state is established and before later ordinary queue work, including an explicit untrusted observation when no trusted source exists.
- **Controlled clean stop:** Exercise radio safe-state, grant settlement, bounded queue drain, exact clean-stop commit, checkpoint and close ordering and mark clean only when all stated preconditions hold.
- **Failed controlled stop:** Fail queue drain, airtime settlement, marker commit, checkpoint and database close independently and verify which failures forbid the marker and which occur after its already durable meaning.
- **Clean-stop idempotency:** Repeat the identical request, reconcile unknown commit and reject conflicting markers or unmet database/queue preconditions.
- **Crash semantics:** Terminate without shutdown hooks at every lifecycle phase and verify correctness comes from durable startup/state/replay rules rather than an assumed cleanup callback.
- **Terminal initialization/recovery diagnostic:** Enter each terminal radio failure, finalize at most one best-effort fatal diagnostic when admission permits and never claim that diagnostic durability is guaranteed.
- **Static systemd/chrony contract:** Validate packaged unit/config artifacts for required mounts, RTC-bootstrap completion ordering, no network-online dependency, restart delay/rate limit, bounded stop timeout, service identity and suspend prevention.
- **No pilot-data dependency:** Verify every test configuration points at dedicated test paths and refuses an accidentally supplied production database, receiver-group file or service name.

### Hardware tests

- **Boot service startup:** Boot the Pi with the isolated test deployment and verify systemd ordering after required storage and RTC-bootstrap completion without waiting for network-online.
- **Bootstrap failure continuation:** Make the test RTC missing, invalid or unresponsive and prove the boot-scoped bootstrap completes within its deadline and the receiver starts offline as untrusted.
- **Bootstrap once per Linux boot:** Restart the receiver service repeatedly under one boot and prove RTC-to-system bootstrap is not repeated; reboot and verify one new bootstrap episode.
- **Service-user privileges:** Run as the configured unprivileged user and verify only required storage, GPIO, SPI, RTC and chrony-socket access, with no `CAP_SYS_TIME` or unintended writable paths.
- **Automatic crash restart:** Kill the process, verify nonzero delayed/rate-limited restart, a new receiver-instance identity and conservative recovery under the unchanged Linux boot identity.
- **Clean service restart:** Stop and restart after a successful bounded drain and verify the exact clean-stop marker, lifecycle ordinal order and new process identity.
- **Unclean service stop:** Exceed a controlled stop precondition or kill during it and verify no false clean marker and correct next-start recovery.
- **Pi reboot lifecycle:** Reboot after clean and unclean instances and verify both identities, lifecycle ordering, clock scope and durable queue/state consequences.
- **Suspend prevention:** While the isolated service is active, verify the deployment prevents host suspend as required; after stop, verify the test did not leave a persistent inhibitor.
- **Missing hardware restart policy:** Remove or deny only the test fixture's required device, verify bounded terminal failure and supervisor rate limiting, then restore it and prove normal recovery.

## End-to-end receiver behavior

End-to-end host tests use the complete production communicator and persistence
components with injected platform adapters. They are implemented before the RF
suite because they can establish the complete application contract
deterministically.

### Host tests

- **Valid reading to durable row:** Inject a reviewed authenticated frame through the fake radio, obtain the deterministic accepted ACK transcript, drain persistence and verify the exact canonical reading and complete occurrence profile.
- **Lost-ACK retransmission:** Complete persistence but hide TX completion from the simulated node, inject the identical retry and verify another admitted profile plus `RETRANSMISSION` without a second canonical reading.
- **Failed ACK transmission:** Fail after `SetTx` may have started, require retained charge and published `UNKNOWN_INTERRUPTED` or exact terminal result, then retry and persist normally.
- **Current-to-backlog conversion:** Inject distinct current and backlog transport messages carrying one exact sample and require one canonical row, one noncanonical matching row and `DUPLICATE_SAME_CONTENT` evidence.
- **Persistence unavailable:** Stall or fail SQLite, verify admission closes, response-eligible packets select retry-later without reservations, retained work recovers and later health exposes aggregate outage evidence when possible.
- **Airtime-suppressed acceptance:** Exhaust the durable ACK allowance, inject a valid reading and prove it is accepted, published and persisted without radio transmission.
- **Clock-boundary pipeline:** Publish events around ordinary untrusted and step-discontinuity observations, persist them through the FIFO and verify analysis derives UTC only for permitted segments.
- **Graceful and crash restart:** Run the complete process through clean stop and child-process kill, then restart from the same database and verify identities, state reconciliation, queue-loss semantics and continued correct admission.

### Hardware tests

All tests in this section are **deferred until every receiver production
component and every other host and hardware test in this document have been
implemented and pass**. Starting this section earlier requires explicit user
agreement. The later suite runs the production receiver service on the Pi and a
separately controlled real node/ESP32 peer; pytest runs on the Pi and may use
`pytest-embedded` only to control that peer.

- **First valid RF reading:** Transmit one reviewed current reading over the pilot PHY, require the exact authenticated accepted ACK at the node and verify the receiver's canonical SQLite row and complete profiling timestamps.
- **Lost accepted ACK:** Suppress or miss the first downlink at the node, retransmit the identical uplink and verify deterministic ACK bytes, one canonical reading and two occurrence profiles with retransmission classification.
- **Failed receiver TX outcome:** Force a controllable post-SetTx failure, verify conservative charge/profile/recovery behavior and prove the node's later retry succeeds without cached receiver history.
- **Current then backlog RF identity:** Send the same sample first as current and later as a newly constructed backlog message and verify transport IDs differ while application classification and stored bodies remain correct.
- **Authenticated rejection and silence matrix:** Send representative authenticated malformed, unsupported and wrong-direction packets plus unauthenticated traffic and verify exact rejection ACKs or required silence over RF and in persistence evidence.
- **Queue and persistence backpressure:** Stall the isolated persistence path, fill the bounded queue, require retry-later responses only for eligible packets and verify nodes retain/retry readings after recovery.
- **Airtime-suppressed RF acceptance:** Exhaust receiver ACK allowance legally, transmit a valid reading, observe no downlink and prove persistence still accepts the occurrence; retry later and verify duplicate classification.
- **Profile transition interoperability:** Alternate normal-IQ node uplinks and inverted-IQ receiver ACKs across multiple episodes and prove neither side receives the wrong direction profile or stale IRQ/buffer contents.
- **Clock and timestamp evidence:** Run online and approved offline-holdover episodes and verify stored monotonic events, trusted observations and derived direct/logical timestamps against independent test timing evidence.
- **Receiver process restart:** Restart or crash the service between node attempts and verify new receiver identity, unchanged Linux boot identity, conservative airtime state and correct durable duplicate handling.
- **Pi reboot:** Reboot between node attempts, verify both receiver and boot identities change, restore time/state conservatively and accept or suppress ACK exactly as the durable contracts require.
- **Complete pilot soak:** Run a legally airtime-bounded mixed current/backlog workload with health sampling, checkpoints and controlled recoverable faults, then reconcile every transmitted logical message, ACK observation, SQLite identity, profile and diagnostic/health aggregate.

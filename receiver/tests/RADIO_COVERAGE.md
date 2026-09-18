# Radio component coverage map

This maps the required [Radio families](../TESTING.md#radio) to their
implementation boundaries and recorded validation.
The production physical port, Linux adapter, SX1262 command backend, radio
owner and diagnostic codec/factory are implemented. Host coverage lives in:

- `host/test_linux_radio.py` and `host/test_sx1262.py`: physical dependency
  failures, exact transactions/profiles, bounded BUSY/reset and certainty;
- `host/test_radio.py`: state, IRQ, ACK, recovery, shutdown, every reviewed TX
  setup/confirmation transaction, and real airtime settlement;
- `host/test_radio_diagnostics.py`: the complete catalogue matrix, literal
  context bytes, validation and bounded episode completion;
- `host/test_radio_ingress_contract.py`: copied bytes through real ingress,
  queue and SQLite, plus approved failure-result persistence;
- `host/test_radio_model.py`: reviewed independent examples and generated
  valid/faulted histories; and
- `host/test_radio_fixture.py`: fixture input and collection interlocks.

These are component tests; communicator orchestration, RF interoperability
and independent physical timing remain separate obligations. Non-peer cases
are in `hardware/test_radio.py`, with the procedure in
[hardware/RADIO_TESTS.md](hardware/RADIO_TESTS.md).

On 2026-09-17, all three nominal Pi cases passed in 2.09 s: device/GPIO/SPI
configuration, initialization/profile readback, and finite RX-timeout IRQ
handling with direct RX restoration. All three teardowns confirmed safe
shutdown; no SetTx was submitted. Source archive and manifests were verified.
The fixture records operator wiring/power confirmations. Later operator DC
measurements are recorded below. IRQ delivery is functional evidence, not an independent
pin/kernel timestamp or waveform qualification.

The [curated radio report](hardware/evidence/radio/README.md) is the permanent
record. It keeps one successful-source snapshot (235 files, 189 manifest entries),
selected physical traces, fixture/target metadata and dependency versions.
The initial startup and timeout-handling failures are preserved as lessons and
host regressions; their complete session bundles have been deleted.

The same source subsequently observed held BUSY: INITIALIZE and CLEANUP /
BUSY_TIMEOUT in 277,477 us, 809 HIGH samples, no SPI/RX/TX and handle release.
Its assertions passed, but safe shutdown remained false and the session aborted.
After operator-confirmed power removal and nominal selector restoration, all
three nominal cases passed in 2.28 s with safe shutdown and no SetTx. Boot IDs
changed; power removal itself is operator-confirmed. Missing-sentinel collection
ran zero cases and remains a setup lesson.

The [offline verification](hardware/evidence/radio/README.md#verification) checks
the retained source, outcomes and traces without Pi or temporary storage.
Held/restored exact commands and numeric process exits were not captured;
curation does not reconstruct them or turn the held run into a safe-teardown pass.

The operator also reported the following multimeter readings in nominal wiring
after boot, before pytest: RESET 2.796 V, CS 3.276 V, BUSY 0 V, DIO1 0 V,
473.9 mV across R_RESET and 1.3 mV across R_CS. With the specified 10 kohm
resistors, these imply approximately 47.39 uA and 0.13 uA respectively.
RESET is consistent with a roughly 59 kohm path to ground, within the Pi 3's
[documented internal pull range](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#voltage-specifications).
This is a pull-down hypothesis, not a measured bias setting or independent
logic-threshold/timing qualification. The Linux adapter currently preserves
GPIO bias with AS_IS; no pull configuration was changed during this audit.

Host validation on 2026-09-17 after the approved runtime status/IRQ
correction: `make test-receiver-host` passed 3,142 cases; the focused
backend/owner/model/ingress/diagnostic/Linux/fixture selection passed 657. Diff checks passed.
Earlier backend validation passed 258 protocol cases and both generators'
validation/freshness checks; the manual fixture revision changes no protocol
or schema production code. These results do not satisfy any physical acceptance
row.

The subsequent implementation-review corrections have host validation only:
3,229 receiver tests and 744 focused radio cases pass. Reset timing now starts
after GPIO assertion; resource lifecycle errors preserve acquisition and every
release failure; soft/hard recovery explicitly resynchronizes the Linux event
stream before rearm. Tests compose the real adapter/backend/owner and demonstrate
later packet delivery after a sequence gap, with strict malformed-evidence and
bounded-failure behavior. The retained Pi snapshot predates these corrections;
its results remain evidence for that recorded source, not physical qualification
of the revised implementation. Existing RF, waveform and gated-recovery deferrals
remain unchanged.

## Host families

| Required family | Implementation boundary |
|---|---|
| Initialization profile | Production command/profile backend transcript and owner state |
| Initialization terminals | Normalized device errors and bounded owner initialization |
| Post-reset status | Captured 0x2A, fresh command confirmation and exact startup/post-calibration error masks |
| Failed-startup cleanup | Retained safety/release result and distinct original/cleanup diagnostics through terminal shutdown |
| Resource lifecycle evidence | Immutable primary/all-release failures through the production Linux/backend/owner; single release attempts and CORE exception propagation |
| RX event classification | IRQ snapshot, packet-copy result, clear/rearm transcript |
| Correlated event confirmation | Captured 0x26/0x0200, immutable status/IRQ/errors, immediate-edge retention, chronology/mode/error conflicts and strict subsequent commands |
| Pi-owned packet snapshot | Copied backend bytes through real ingress and queue publication |
| Response-free RX rearm | Owner rearm operation after caller-selected silent/suppressed handling |
| ACK profile transition | Prepared bytes, external allowance handoff, backend profile/SetTx transcript |
| Definite pre-SetTx failure | Command certainty, optional T4, external refund facts, RX restoration |
| Started or uncertain SetTx | TX_ACTIVE only after SetTx; charge retention and TX_UNCONFIRMED |
| Missing or delayed TxDone | Injected monotonic deadline and timestamped IRQ events |
| BUSY and SPI failures | Existing production port with precise primitive fault injection |
| Soft and hard recovery | Owner recovery transitions, counters, one immutable episode |
| Event immediately after recovery | New DIO1 event preserved across confirmed SetRx |
| GPIO stream recovery | Production adapter sequence-gap recovery and subsequent packet delivery; explicit bounded resynchronization, untrusted metadata, flood and shutdown-boundary cases |
| Diagnostic catalogue enforcement | Handwritten context codec/factory and generated receiver enums |
| Controlled radio shutdown | Owner shutdown and production backend resource lifecycle |
| Interrupted recovery accounting | Owner shutdown at soft/hard primitive boundaries and diagnostic builder tests |
| Radio state-machine properties | Independent primitive model after reviewed deterministic examples |

The physical-port fake is shared by backend and owner tests after both
established its real boundary.
No reference model may derive its expected behavior from production profile
builders, command encoders, or state transitions.

## Hardware families

| Required family | Evidence boundary |
|---|---|
| Device and permission probe | Nominal PASS: configured devices opened as the intended service user |
| Real initialization | Nominal PASS: attached board, production SPI/GPIO, confirmed profile/mode |
| Finite RX timeout/rearm | Nominal PASS: real timeout IRQ and complete receive restoration; no independent timing claim |
| Manual held-BUSY startup | Expected fault observed on Pi in 277,477 us; no SPI/RX/TX, unconfirmed cleanup retained; subsequent nominal restoration PASS |
| Real BUSY behavior | Raw radio BUSY waveform and bounded production waits |
| DIO1 timestamp path | Controlled radio IRQ, kernel monotonic edge, matching observations |
| Normal-IQ uplink reception | Independent component peer and reviewed exact payload |
| Inverted-IQ ACK transmission | Independent peer with matching and opposite IQ settings |
| Profile restoration | Alternating peer RX/TX evidence after each rearm |
| Radio timing characterization | Raw command/edge samples and independent timing reference |
| Hardware reset recovery | Controllable physical fault, real reset/profile restoration |
| Safe-state teardown | Nominal PASS; TX-adjacent/recovery conditions still require their deferred fixtures |

The [carrier proposal](../hardware/TEST_CARRIER.md#proposed-sx1262-extension)
defines connections and fault states. Selected missing hardware fails; there
is no runtime skip or host result that satisfies a physical obligation.
The approved peer is the real C6 radio application, coordinated from laptop
tests/rf/ with a separate Pi component process in
receiver/test_apps/radio_peer/. Reuse the production Pi radio components where
their fixed profile and state contract applies; deliberate alternative-profile
cases identify the lower layer they exercise. Peer implementation and RF
execution are still pending. Independent waveform/timestamp qualification and
controlled BUSY-gate recovery retain their separate deferred status.
Waveform and timing acceptance await an instrument beyond the available
multimeter. No peer placeholder or RF acceptance is provided.
The unavailable SN74LVC1G32 gate and its synchronized physical soft/hard
recovery execution are also operator-deferred. Nominal non-peer cases remain
independent of that gate. Manual held-BUSY startup and subsequent nominal
restoration have been observed on hardware; these separate boots cannot
satisfy the runtime recovery obligation. The held run's unconfirmed cleanup
remains retained despite the later nominal success. Future held runs still
require powered-off selector restoration and a fresh nominal run.
The source manifest includes the shared rail/selector
schematic and operator procedure.

The implementation stage was closed at the operator's direction on 2026-09-17,
with RF tests explicitly deferred. The independent waveform/timestamp/timing
checks and gate-driven runtime recovery remain separate, previously deferred
non-RF obligations. RESET-bias investigation is operator-deferred to future
hardware work; the DC readings do not establish a defect or change bias policy.
Missing command/exit records for the two operator runs remain historical
evidence limitations. Closure neither marks these items as passing nor removes
the evidence requirements for future runs.

## Radio diagnostics and orchestration boundary

The [required diagnostic tests](../INTERFACE_DIAGNOSTIC.md#required-radio-diagnostic-tests)
map to the context/factory tests for exact bytes and catalogue rejection, and
to owner tests for direct anomalies, recovery episode cardinality, severity,
original/last failure evidence, correlations, certainty and normal IRQ outcomes.
Generator freshness and receiver/firmware independence remain host checks.

The radio returns an immutable completed episode. Host tests cover safe behavior
after the caller discards that result without performing diagnostic admission. Its safe-state result,
transmission facts and counters are fixed before the caller can discard the
episode, assign a diagnostic identity or attempt admission. Component tests
cover that independence and do not manufacture a communicator to claim the
following later orchestration obligations:

- publishing an already reserved profile before attempting diagnostic admission;
- preserving that publication when the diagnostic reservation fails;
- top-level communicator crash/exception and persistence-unavailability handling.

These obligations remain with the production communicator's tests. The
packet-snapshot test may compose existing real ingress and queue components to
verify byte ownership; that limited composition is not receiver end-to-end
coverage. Durable grant acquisition/settlement and protocol selection retain
their existing production owners.

# Receiver application coverage

Run `make test-receiver-host` to verify the production application on the host.
Installed service, Pi timing/permissions and RF behavior have separate tests.
The governing assertions remain in [TESTING.md](../TESTING.md),
[INTERFACE.md](../INTERFACE.md) and [INTERFACE_DIAGNOSTIC.md](../INTERFACE_DIAGNOSTIC.md).

Signal stop separates main-thread intent from Linux wakeup notification and
owner-controlled cleanup. Actual SIGTERM regressions cover the application wait
and the radio's held Event lock; repeated/nested requests retain the first
deadline. The signal callback takes no synchronization lock or cleanup action.

| Production responsibility | Host assertions and owning tests |
|---|---|
| Packet lifecycle | `test_communicator.py`: exact reviewed accepted/rejected ACKs, malformed/unauthenticated silence, repeated reading after lost ACK, current/backlog, one canonical SQLite reading with every occurrence retained, failed RX copied-byte evidence and TX terminal facts. `test_protocol_ingress_*`: full decision, identity, payload, reservation and immutable-completion matrices. |
| Scheduler | `test_communicator.py`: ready radio before health/RTC, precharge outside the exchange, paced clock-boundary retry, initial health, completion before independent stream failures and original event timestamps. Radio/backend suites retain active-TX and pre-SetRx synchronization checks. |
| Admission and pressure | `test_producer_admission.py`, `test_communicator.py`: shared exact reservation matrix, multiple/cancelled attempts, own health attempt, queue-full RETRY_LATER followed by acceptance after capacity returns, no acceptance/ACK for failed ingress. Existing queue/worker concurrency and schedule tests use the real queue and SQLite. |
| Airtime and complete state | `test_tx_airtime_*`, `test_communicator_state_owner.py`, `test_communicator.py`: shared authority, acknowledged grants, conservative uncertain TX, unused-charge settlement, exact unknown-commit reconciliation, retained precharge and durable crash boundaries. Caller-retained receipts preserve exact outcomes across later commits; a scheduler/SQLite regression completes RTC verification after airtime reconciles its lost reply and commits a later settlement. RTC complete-state handoff preserves the airtime owner's snapshot. |
| Time | `test_runtime_time.py`, `test_rtc_refresh_episode.py`, application tests: initial boundary, independent network time, offline/untrusted behavior, generation/expiry/step boundaries, provenance validation and source freshness. Incremental refresh permits one adapter/control action per turn and retains the original retry window. Installed receipts still require usable current expected proof and fresh matching sources; continuation does not reconcile an unrelated later pending request. |
| RTC shutdown | `test_rtc_refresh_episode.py`, `test_application_shutdown.py`: stop before/after invalidation, during a slow read or possibly applied write, between retries and during unknown commit; no further device write, retained root/results/counters, conservative provenance and exact state reconciliation. Garbage-collection checks prove cancelled episodes leave unresolved receipts with the state owner, which releases them after resolution. Real application stop during the reproduced 2.9-second pre-read completes within its original deadline. Device timings are scripted host inputs, not Pi measurements. |
| Profiles and health | `test_communicator.py`, `test_application_diagnostics.py`, `test_producer_admission.py`, existing value/enrichment tests: timestamp/frame presence, sequences, initial/periodic scheduling, admission counters and worker enrichment. Failed-receive entity construction precedes reservation; allocation/constructor failures retain no reservation or admission count, preserve the existing CORE allocation policy and leave the closed queue drainable. |
| Diagnostics | `test_application_diagnostics.py` and radio/time diagnostic suites: closed CORE/control/RADIO/TIME contexts and padding, root selection, operation catalogues and disposition counters. `test_communicator.py`: runtime emission, independent roots, clock/reservation ordering, best-effort loss and non-recursion. |
| Fatal packet completion | `test_communicator.py` and ingress completion tests: bounded terminal cleanup before publication; definite pre-SetTx failure, UNKNOWN_INTERRUPTED, retained TxDone/timeout; no second diagnostic reservation while an unpublishable accepted occurrence still owns the slot. |
| Startup/package | `test_application.py`, `test_application_settings.py`, `test_storage_preflight.py`: real worker/configuration/state startup, public HKDF vector, configuration rejection before radio, missing RTC/state with RX but no TX allowance, required paths and reserve. Isolated bundle imports both entry points and initializes its exact packaged schema without test modules or checkout imports. |
| Controlled stop/restart | `test_application_shutdown.py`: shared deadline, TX inhibition/radio cleanup, state handling, bounded drain, exact marker retries, unknown state/unsafe radio/undrained queue forbidding a marker, marker transport exception, idempotent stop, checkpoint/close failure preserving a durable marker. `test_application_signals.py`: actual SIGTERM before wait, after predicate check, during an observed kernel poll and with the radio Event lock held; second SIGINT during cleanup, real SQLite clean marker and original deadline. `test_stop_intent.py`, `test_linux_signal_wait.py`: nested intent, notification races/full pipe, absolute waits, installation failure and handler/descriptor restoration. Real SIGKILL after application startup and after marker commit, followed by a new receiver UUID. |
| Bootstrap/service | `test_rtc_bootstrap.py`: boot-scoped atomic claim across process reopen, synchronized-kernel protection, bounded attempts and late-read rejection; copied UTC remains provisional. Unit files encode mounts, UID, restart bounds, bootstrap ordering, capability boundary and sleep inhibition. Isolated Pi transient systemd execution qualifies the signal-stop software path as described below; packaged-unit boot/suspend ordering and installed hardware/privilege boundaries remain separate gates. |

Tests use production policy owners, real temporary SQLite files, reviewed
protocol vectors and platform/control boundary fakes. They do not substitute a
test-only communicator. SIGKILL establishes process-crash behavior, not electrical
power-loss durability. Simulated peer loss establishes receiver retry handling,
not over-the-air ACK delivery.

The application source bundle contains no tests, credentials, populated database
or development environment. It is a source artifact, not a completed Pi install.
Helper digest and validated target RTC/kernel bound remain required installation
inputs. No service was installed, device accessed, clock changed or RF emitted
for this host verification.

## Implementation file map

| File | Change |
|---|---|
| [stop_intent.py](../cura_receiver/stop_intent.py) | Retains one coherent first request/deadline pair, including nested handler re-entry; exposes read-only intent. |
| [linux_signal_wait.py](../cura_receiver/platform/linux_signal_wait.py) | Owns nonblocking wakeup pipe, absolute wait, SIGTERM/SIGINT installation, rollback and restoration before descriptor close. |
| [application.py](../cura_receiver/application.py) | Reads shared intent, exposes its retained deadline and delegates waiting to an injected adapter. |
| [__main__.py](../cura_receiver/__main__.py) | Wires one intent to application/radio and scopes Linux signal ownership across startup, run and shutdown. |
| [radio.py](../cura_receiver/radio.py) | Reads application intent at existing operation/primitive boundaries; retains the independent thread-safe shutdown API. |
| [communicator.py](../cura_receiver/communicator.py) | constructs the complete failed-receive profile unit before reserving queue capacity. |
| [test_stop_intent.py](host/test_stop_intent.py) | Repeated/nested requests and coherent deadline arithmetic. |
| [test_linux_signal_wait.py](host/test_linux_signal_wait.py) | Notification hints/full pipe, EINTR/absolute deadlines, partial installation, foreign-thread rejection and restoration/reuse. |
| [test_application_signals.py](host/test_application_signals.py) | Named actual-signal process interleavings, observed kernel blocking, held radio lock, owner cleanup and real durable marker. |
| [test_application.py](host/test_application.py) | Shares intent in the production fixture and permits actual Linux clocks/waits. |
| [test_application_entrypoint.py](host/test_application_entrypoint.py) | Checks exact intent wiring and waiter/context lifecycle. |
| [test_application_shutdown.py](host/test_application_shutdown.py) | Migrates stop assertions to shared intent and the explicit wait input. |
| [test_radio.py](host/test_radio.py) | Adds application-stop checkpoints before TX, after profile transfer and after actual SetTx submission. |
| [test_communicator.py](host/test_communicator.py) | allocation/constructor failures cannot strand a failed-receive reservation; preserve CORE policy/drain. |
| [ARCHITECTURE.md](../ARCHITECTURE.md), [INTERFACE.md](../INTERFACE.md), [TESTING.md](../TESTING.md) | Define the approved ownership boundary and regression obligations. |
| APPLICATION_COVERAGE.md | Closes the signal gap, records actual validation scope and maps the changed files. |

# Receiver application coverage

The production receiver application is implemented and host-verified. This closes
DEP-003's host phase; installed service, Pi timing/permissions and RF acceptance
remain separate gates. The exact source manifest, commands and results are in
[evidence/application_host/README.md](evidence/application_host/README.md).
The governing assertions remain in [TESTING.md](../TESTING.md),
[INTERFACE.md](../INTERFACE.md) and [INTERFACE_DIAGNOSTIC.md](../INTERFACE_DIAGNOSTIC.md).

| Production responsibility | Host assertions and owning tests |
|---|---|
| Packet lifecycle | `test_communicator.py`: exact reviewed accepted/rejected ACKs, malformed/unauthenticated silence, repeated reading after lost ACK, current/backlog, one canonical SQLite reading with every occurrence retained, failed RX copied-byte evidence and TX terminal facts. `test_protocol_ingress_*`: full decision, identity, payload, reservation and immutable-completion matrices. |
| Scheduler | `test_communicator.py`: ready radio before health/RTC, precharge outside the exchange, paced clock-boundary retry, initial health, completion before independent stream failures and original event timestamps. Radio/backend suites retain active-TX and pre-SetRx synchronization checks. |
| Admission and pressure | `test_producer_admission.py`, `test_communicator.py`: shared exact reservation matrix, multiple/cancelled attempts, own health attempt, queue-full RETRY_LATER followed by acceptance after capacity returns, no acceptance/ACK for failed ingress. Existing queue/worker concurrency and schedule tests use the real queue and SQLite. |
| Airtime and complete state | `test_tx_airtime_*`, `test_communicator_state_owner.py`, `test_communicator.py`: shared authority, acknowledged grants, conservative uncertain TX, unused-charge settlement, exact unknown-commit reconciliation, retained precharge and durable crash boundaries. RTC complete-state handoff preserves the airtime owner's snapshot. |
| Time | `test_runtime_time.py`, `test_rtc_refresh_episode.py`, application tests: initial boundary, independent network time, offline/untrusted behavior, generation/expiry/step boundaries, provenance validation and source freshness. Incremental refresh permits one adapter/control action per turn and retains the original retry window. |
| RTC shutdown | `test_rtc_refresh_episode.py`, `test_application_shutdown.py`: stop before/after invalidation, during a slow read or possibly applied write, between retries and during unknown commit; no further device write, retained root/results/counters, conservative provenance and exact state reconciliation. Real application stop during the reproduced 2.9-second pre-read completes within its original deadline. Device timings are scripted host inputs, not Pi measurements. |
| Profiles and health | `test_communicator.py`, `test_application_diagnostics.py`, `test_producer_admission.py`, existing value/enrichment tests: timestamp/frame presence, sequences, initial/periodic scheduling, admission counters and worker enrichment. |
| Diagnostics | `test_application_diagnostics.py` and radio/time diagnostic suites: closed CORE/control/RADIO/TIME contexts and padding, root selection, operation catalogues and disposition counters. `test_communicator.py`: runtime emission, independent roots, clock/reservation ordering, best-effort loss and non-recursion. |
| Fatal packet completion | `test_communicator.py` and ingress completion tests: bounded terminal cleanup before publication; definite pre-SetTx failure, UNKNOWN_INTERRUPTED, retained TxDone/timeout; no second diagnostic reservation while an unpublishable accepted occurrence still owns the slot. |
| Startup/package | `test_application.py`, `test_application_settings.py`, `test_storage_preflight.py`: real worker/configuration/state startup, public HKDF vector, configuration rejection before radio, missing RTC/state with RX but no TX allowance, required paths and reserve. Isolated bundle imports both entry points and initializes its exact packaged schema without test modules or checkout imports. |
| Controlled stop/restart | `test_application_shutdown.py`: shared deadline, TX inhibition/radio cleanup, state handling, bounded drain, exact marker retries, unknown state/unsafe radio/undrained queue forbidding a marker, marker transport exception, idempotent stop, checkpoint/close failure preserving a durable marker. Real SIGKILL after application startup and after marker commit, followed by a new receiver UUID. Existing worker/airtime crash suites cover additional named durable boundaries. |
| Bootstrap/service | `test_rtc_bootstrap.py`: boot-scoped atomic claim across process reopen, synchronized-kernel protection, bounded attempts and late-read rejection; copied UTC remains provisional. Unit files encode mounts, UID, restart bounds, bootstrap ordering, capability boundary and sleep inhibition. Actual systemd execution and installed privileges are NOT RUN. |

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

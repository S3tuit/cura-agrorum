# Firmware testing

Status: the `node_core`, `node_persistence`, `node_sensors`, `sx1262_radio` and
platform-port host matrices are implemented. The on-device implementation now
includes the `node_persistence` matrix, the complete bare-board RTC suite, the
receiver-free `node_core` integration suite, and platform clock, randomness,
software-reset-reason and timer-deep-sleep cases. A sibling sensor-carrier app
implements real sensor component and core-reading integration cases, including
operator-guided fixtures. Its [coverage/evidence index](test_apps/sensor_carrier/COVERAGE.md)
distinguishes implementation, prior component evidence and pending mapping runs.

## Philosophy and build

Host tests use the production `node_core` with fake persistence, sensor, radio,
clock, randomness and system ports. Component tests use the real component with
a private fake backend only where meaningful policy exists. Assertions cover
results, state and relevant ordering constraints. Only a few orchestration
tests assert a complete operation sequence so harmless refactoring does not
break unrelated tests.

Native firmware tests are separate CMake targets compiled with strict warnings,
ASan and UBSan. CTest runs the executables. The top-level `make test-host`
target configures, builds and invokes CTest. The `node_persistence` executable
uses the production component with a private POSIX/NVS fake backend and splits
its scenarios by NVS, record behavior, recovery, fault handling and retention.
The existing pytest suite remains responsible for protocol generation, shared
Python/C vectors and Hypothesis. `pytest-embedded` provides hardware
orchestration as described below.

The `test_node_core` executable links the production controller, protocol codec
and OpenSSL-backed production crypto layer to deterministic link-time fakes for
persistence, sensors, radio and the injected platform ports. Each named CTest
scenario is a fresh process. The fake clock advances only through scripted
component behavior, so retry, deadline, awake-time and terminal-sleep ordering
are tested without real waiting.

## `node_core`

### Wake initialization and reading construction

- A current reading accepted on its first attempt is claimed, sampled,
  persisted, transmitted, removed, followed by a backlog query, and reflected
  in outgoing RTC metrics.
- `claim_sample_id` failure invalidates RTC before the claim, attempts a
  best-effort diagnostic, and performs no sampling, radio initialization, TX or
  RX. Idempotent `force_power_off`, `radio.sleep`, `sync_all` and deep sleep
  still run once.
- Valid incoming RTC state is copied into the reading and invalidated before
  any fallible operation.
- One parameterized test rejects incoming RTC state for a wrong marker,
  non-deep-sleep reset, nonconsecutive sample ID or invalid metric invariants;
  previous metrics become zero and both previous flags are clear.
- Each soil and DS18B20 validity group maps its exact value and flag while
  invalid groups stay zero. A BME280 failure zeroes all three enclosure fields
  and clears their three protocol validity bits together.
- Failure of all sensors still produces a structurally valid reading.
- `DEEP_SLEEP_BOOT` is set exactly when reset reason is 8.
- `run_ms` uses the application-start and body-finalization clock samples.
- Pending-read append failure attempts a diagnostic and permits RAM-only
  current delivery.
- RAM-only `ACCEPTED` does not attempt to remove a nonexistent pending copy.
- Sampling that consumes the radio deadline produces no TX.
- `run_ms` above `UINT16_MAX` saturates, preserves the reading and emits
  `ETIME_RANGE`; `run_time_and_sampling_deadline_boundaries` tests this alongside
  truncated millisecond conversion and the sampling-consumed radio deadline.

### ACKs and retries

- Silence retransmits the identical 54-byte authenticated frame. The test uses
  different `message_id` and `sample_id` values and verifies that only the
  former occupies clear-header bytes 10-13.
- Every `TX_DONE` consumes the next scripted random value and produces the
  expected `retry_at`.
- Multiple invalid ACKs followed by a valid ACK stay in one RX interval and do
  not restart that interval.
- Parameterized invalid ACKs cover bad length, bad tag, foreign node, wrong
  message ID, unsupported control, uplink domain and domain/status mismatch.
- An authenticated unknown ACK status is isolated from domain/status mismatch
  and reports `CURAG_ECORE_EACK_STATUS`.
- `ACCEPTED`, `RETRY_LATER`, `REJECTED_UNSUPPORTED` and
  `REJECTED_MALFORMED` are each exercised for current and backlog delivery.
- Failure before `SetTx` can take effect neither increments attempts nor
  charges airtime; an uncertain result after the command crosses SPI does both.
- Failure after `SetTx` starts but before `TX_DONE`, including radio deadline,
  increments attempts, charges airtime, records a local error and does not
  retry.
- An RX local error after `TX_DONE` terminates delivery rather than being
  treated as silence.
- An authenticated ACK whose `RX_DONE` timestamp is at or before `retry_at`
  wins; a later ACK does not prevent retry.

### Airtime and wall-clock budgets

- The fixed-profile integer model returns 102,656 us for the 54-byte reading
  frame, and core applies the independent 10% charge of 112,922 us. The radio
  reports a 112,704 us minimum TX window.
- Exactly one more reading-airtime charge fits.
- One microsecond more than the available airtime does not fit.
- Exactly one radio-reported minimum TX window fits the wall-clock deadline;
  one microsecond less and the reported 100,000 us regression window do not.
- A full cycle whose completed TX reaches the wall-clock deadline reports
  `RADIO_CYCLE_DEADLINE`; a radio deadline after `SetTx` reports a charged
  `LOCAL_RADIO_ERROR` instead.
- Airtime can exhaust while wall-clock time remains, and wall-clock time can
  exhaust while airtime remains.
- Current and backlog deliveries share both limits; neither budget resets per
  reading.
- After accepted current and backlog traffic consumes the shared airtime, a
  final silent backlog delivery ends as `AIRTIME_BUDGET_END` before retry.
- A started TX remains charged when `TX_DONE` never arrives.
- An attempt that cannot fit either limit is never passed to the radio.

### Current and backlog transitions

- Current `RETRY_LATER` stops before backlog lookup or transmission.
- Backlog `RETRY_LATER` stops further drainage.
- Permanent current rejection quarantines and stops; quarantine failure logs
  and still stops.
- Accepted-current removal failure logs and prevents backlog selection.
- Accepted current begins backlog drainage only after successful removal, or
  directly for RAM-only current data when backlog storage remains available.
- `node_core` consumes backlog entries in persistence-provided order; actual
  newest-first reconstruction is a persistence responsibility.
- A current reading converted to backlog retains its `sample_id` and exact
  32-byte body but receives a newly committed `message_id` and backlog-domain
  frame before its first backlog TX.
- Current retries reuse one RAM-resident frame. Backlog retries in the same or
  later wakes reuse the exact durably bound frame; a binding failure permits no
  first backlog transmission.
- Accepted-backlog removal failure stops drainage.
- Permanent backlog rejection quarantines and continues.
- A lost quarantine copy with successful pending removal logs and continues.
- Backlog quarantine failure to remove the pending tail logs and stops to
  prevent immediate reselection.
- Empty backlog completes normally, while backlog lookup failure logs and
  stops.
- Backlog silence retries while both shared limits allow another attempt.

### Delivery and diagnostic events

- An entered delivery operation appends durable `DELIVERY_STARTED` before its
  first `transmit_uplink` call and one durable `DELIVERY_FINISHED` after its
  terminal result.
- One start/finish pair brackets the whole operation, not each retry.
- The pair uses `cycle_sample_id`, reading `sample_id`, transport `message_id`
  and domain, allowing one logical backlog message to be recognized across
  wakes independently from the reading it carries.
- First-attempt acceptance produces a matched start and finish.
- Silence followed by acceptance produces one pair whose finish contains all
  attempts and offsets.
- Terminal failures and invalid ACK frames appear in the finish event.
- Start-event failure does not prevent TX; finish-event failure does not alter
  the delivery result or final cleanup.
- An ordinary diagnostic append failure never changes wake behavior and never
  triggers recursive logging.

### Metrics, RTC and finalization

- Current acceptance on attempt two produces current attempts 2, cycle
  attempts 2, accepted count 1, and delivery time measured from immediately
  before the first `SetTx`.
- Backlog attempts affect cycle attempts but not current attempts.
- Accepted-reading count increments once per distinct reading, not per attempt.
- Unaccepted current clears `PREVIOUS_CURRENT_ACCEPTED` and stores zero current
  delivery time.
- A completed cycle with zero TX attempts can still produce valid outgoing
  metrics.
- RTC helpers cover exact wire maxima and semantic-invalid combinations.
  Full-cycle tests cover reachable metric boundaries; awake-time overflow known
  before `sync_all` emits a diagnostic and invalidates RTC.
- Awake-time overflow caused only by `sync_all` invalidates RTC without a
  diagnostic append after the single final synchronization.
- `previous_awake_ms` includes final logging and `sync_all`, but excludes RTC
  commit and the sleep call.
- Outgoing metrics never alter an already constructed body or frame.
- RTC fields are stored before the commit marker, which is stored last.
- Every full-cycle scenario calls `force_power_off`, `radio.sleep`, `sync_all`
  and deep sleep exactly once, with no operation after deep sleep.

## `node_persistence`

Tests that do not depend on the pending/log record encoding cover:

- Fresh storage independently claims sample ID 0 and message ID 0;
  consecutive claims commit before returning and never reuse IDs.
- Sample and message counters advance independently. Their `UINT32_MAX`
  exhaustion and NVS commit failures return no claimed ID.
- An ambiguous message-counter commit may skip an ID, but the next successful
  claim never reuses the possibly committed value.
- NVS and LittleFS initialize independently and at most once per wake.
- Initialization failure is cached only for the affected backend.
- Failed NVS initialization does not prevent a LittleFS diagnostic attempt.
- `sync_all` skips unused backends, does not initialize them, and does not
  retry a backend whose initialization was cached as failed. It synchronizes
  and closes owned handles without unregistering LittleFS.
- Empty backlog is distinguishable from a storage error.
- Successful delivery start and finish events are durable before returning.
- Every injected public-operation failure returns the documented
  `err_curag_t`, operation and exact seven-byte persistence context.
- A successful operation clears a supplied `diagn_context_t`; passing a null
  diagnostic output does not change behavior or the returned error.
- Backend status values are canonically encoded as signed little-endian `i32`;
  semantic failures use `NO_ERROR` and a zero status.

After choosing the record encoding, tests also cover:

- Round-trip of every record type and boundary value.
- Truncated final records at every meaningful boundary, trailing bytes,
  semantically invalid tails and unsupported record types or versions.
- A safely delimited unusable tail is truncated and synchronized, the current
  operation returns the original error without proceeding, and the next call
  can use the repaired preceding tail.
- An append never writes after an unvalidated or unrecoverable tail; corruption
  is therefore not buried in the middle of the log.
- An unprovable boundary preserves the file and repeatedly reports corruption
  rather than guessing.
- Newest-first backlog reconstruction, including exact 54-byte binding
  round-trip and compaction that preserves a reading/binding pair as one item.
- `remove_newest_reading` removes only a supported pending tail with the
  expected sample ID; empty, different and recovered tails are not mistaken for
  that reading.
- Reset between quarantine append and pending removal permits duplicates but
  never loses the pending copy before the quarantine attempt.
- Matched and unmatched delivery events, including the same packet retried in
  different cycle IDs.
- Host fault injection models interruption between each append and
  synchronization step. Physical power-loss testing is deferred.

A torn application append is tested even though general filesystem corruption
is delegated to LittleFS; a torn record does not necessarily imply filesystem
corruption.

Controller behavior is tested only under `node_core`: it owns suppression of
recursive diagnostic logging, copying component diagnostic context, ordering a
quarantine append before pending removal, attempting removal after quarantine
failure, and the four combined quarantine/removal outcomes. Those expectations
are intentionally not duplicated in the `node_persistence` component suite.

## `sx1262_radio`

The implemented `test_sx1262_radio` executable links the production portable
state machine to a deterministic private fake backend. Every CTest scenario is
a separate process, which restores the hidden singleton through ordinary BSS
initialization without adding test controls to the public component. It uses
the same strict warnings, ASan and UBSan as the other native component tests.

- Initialization applies the complete pilot profile: 868.1 MHz, SF7, BW125,
  CR4/5, +14 dBm, preamble 8, explicit header, payload CRC, private sync word,
  40 us ramp, boosted RX and the selected regulator mode.
- The host fake validates portable initialization policy and profile values,
  but does not execute ESP-IDF backend register work such as the SX1262
  TX-clamp workaround. The firmware build compile-links that production path;
  its electrical effect remains part of the deliberately deferred radio
  hardware validation.
- The first transmit initializes once; repeated TX does not repeat full
  initialization, and initialization failure is cached.
- Uplink selects normal IQ; downlink selects inverted IQ; TX after downlink RX
  restores normal IQ.
- `RX_DONE` returns exact bytes, length, RSSI, SNR and completion time; timeout
  and local radio error remain distinct.
- An oversized received length cannot overflow the caller's buffer.
- Confirmed and uncertain `SetTx` outcomes and later IRQ outcomes are reported
  accurately.
- A post-`SetTx` `GetStatus` failure and an IRQ-read failure both report
  `tx_started = true` and `tx_done = false`; a definitive failure before
  `SetTx` reports `tx_started = false`.
- An uncertain configuration command before `SetTx` terminates transmission
  without reporting or charging an attempt.
- A captured `TX_DONE` remains visible when IRQ clearing fails or a simultaneous
  timeout makes the overall operation fail.
- Zero, oversized and null TX inputs are rejected without touching hardware;
  1- and 255-byte payloads are transmitted without truncation.
- The airtime model covers invalid, 1-, 50-, 54- and 255-byte inputs with exact
  fixed-profile values, including watchdog quantization in each corresponding
  minimum TX window.
- A 50-byte regression call with only 100,000 us remaining completes packet
  setup but is rejected before `SetTx`; it reports no started or charged
  attempt because the five-millisecond watchdog margin would undercut modeled
  airtime.
- Expired and boundary deadlines, an already-pending IRQ at the inclusive
  boundary, reused absolute RX deadlines, late packets and the maximum
  TX-watchdog conversion are covered without sleeping in real time.
- RX packet data is snapshotted before IRQ clearing and single-shot RX is
  rearmed before returning. Header/CRC-error IRQs are discarded and rearmed;
  unexpected IRQs, device errors, buffer overlength and backend failures remain
  distinguishable and bounded.
- Failures populate the stable 14-byte diagnostic context, including state,
  operation, stage, backend status and valid raw IRQ/device fields.
- Sleep before initialization is a no-op and does not initialize the radio.
- Sleep after initialization stops active RX with `SetStandby(STDBY_RC)` and
  then issues `SetSleep(COLD_START)`.
- A standby or sleep-command failure is returned to the controller and does not
  prevent system deep sleep.

## `node_sensors`

The implemented `test_node_sensors` executable links production policy code to
a small private fake hardware seam. It is built with the same strict warnings,
ASan and UBSan as persistence tests. It verifies the successful sequence and
200 ms delay, null arguments, isolated and complete sensor failures, atomic
BME280 invalidation, simultaneous diagnostic slots, shared DS18B20 failures,
power-on blocking, power-off precedence, mandatory immediate cleanup, optional
diagnostics and idempotent final power-off.

The hardening matrix additionally verifies duplicate configured ROM rejection
before bus access; accurate validation, initialization, read and cleanup
operations; preservation of successful temperatures after cleanup failure;
cleanup-error precedence; combined channel/cleanup and shared/power failures;
valid zero-valued groups; and the ordering constraint that the switched rail is
released before independent BME280 acquisition. Exact full traces are limited
to a few orchestration tests so harmless refactoring does not break the suite.

`test_node_sensors_identity` links the exact production ROM parser/resolver and
verifies canonical numeric parsing, independent invalid identities, mixed
valid/invalid provisioning and the duplicate result that forbids bus access.

A second private seam links the production power-gate implementation to fake
ESP-IDF GPIO calls. It verifies that power-on loads level 1 before selecting
open-drain output, uses no internal pulls, and only then asserts level 0. It
also verifies that power-off releases with level 1 before selecting floating
input, every later power-on reconfigures output mode, setup failures never
progress to the enabling low assertion, and shutdown attempts every safety
step while returning the first failure. The fakes do not model ADC, I2C or
1-Wire electrically; those behaviors remain in the on-device plan below.

The GPIO boundary tests also exercise production DS18B20 pad release: disable
input/output and both internal pulls before switched-rail shutdown, and
propagate a GPIO configuration failure. `gpio_reset_pin` alone does not satisfy
this condition because ESP-IDF enables the internal pull-up.

### BME280 driver and adapter host boundary

Driver and adapter regression sources belong in firmware/tests/host; corrected
Bosch driver sources belong in the separately hosted S3tuit/BME280_SensorAPI
fork. Both target firmware and native tests resolve the same full commit pin.
The fork documentation points back to these tests. The fork does not contain a
second test harness. A standalone reproducer is deferred until the first
upstream bug report is prepared; local regressions remain required now.

Compile the real driver with scripted read/write/delay callbacks for native
initialization, first/later polling failures, NVM limits, combined data read,
packed signed calibration and compensation cases. Compile the
actual production adapter plus that driver with ESP-IDF/time boundary fakes for
bus/device allocation failures, callback error preservation, monotonic budgets,
freshness, failure latching and sleep recovery. Exercise disabled and other
non-x1 settings on each channel after configuration, before triggering and at
completed conversion; assert phase-specific invalid-response errors, atomic
output invalidation, recovery and latching. Verify that enabled raw T/P 0x80000
and H 0x8000 can succeed, with independent compensation expectations, including
the measured DUT temperature calibration near 25.185 C. Raw code equality alone
is not evidence of a skipped channel. Verify expected numbers from
datasheet calculations or independent vectors. Do not use fake Bosch functions
or repeat the node_sensors fake-backend policy matrix as driver evidence.

For the installed IDF v6.1-dev-4182-g47faecc3e4, synchronous register reads have
six internal I2C operations and writes three. This adapter limits register reads
to 26 bytes and complete writes to 20 bytes, fitting the C6's 32-byte FIFO.
Thus each command advances without a multi-FIFO loop. A conservative wait
ledger per transfer is one bus semaphore wait, up to six command semaphore
waits, one completion-queue wait and one NACK bus-busy wait. Each uses the 20 ms
argument; the strict `>` NACK tick check can add one 10 ms tick. At most two
hardware bus-clear attempts add up to 60 ms each (50 ms with strict `>` at
100 Hz). The conservative sum is 310 ms of explicit waiting per call; writes
have fewer command waits. This is a source-derived wait allowance, not a hard
wall-time guarantee including scheduling, logging or heap/critical-section
latency. Bus/device setup and teardown have no device polling and operate under
exclusive ownership; their portMAX_DELAY mutex acquisitions assume no competing
owner. Async paths and their unrelated waits are not enabled. Reinspect this
ledger when upgrading IDF or changing transfer sizes/ownership. Adapter phase
admission checks cannot preempt an in-flight SDK call; reject its late success
and do not admit another call after the budget. Retain target and host evidence
separately; no host fake establishes the electrical bus behavior.

## On-device hardware tests

### Strategy and harness

On-device tests complement the host suite; they do not repeat its exhaustive
state-machine and malformed-input matrices. Assertions that need the ESP32 run
as C tests using ESP-IDF's Unity integration. `pytest-embedded` is the outer
runner: it selects the test image and serial port, flashes the board, controls
reset and serial interaction, coordinates multi-stage cases and parses Unity
results. ESP-IDF CMake and `idf.py` remain the build layer rather than a separate
hardware-test runner.

The repository contains a dedicated ESP32-C6 test application under
`firmware/test_apps/on_device`. Its implemented bare-board suites link
production `node_core`, `node_persistence`, `node_platform_esp` and protocol
codec/crypto code and use:

- a 24 KiB `nvs_test` partition and a 2,944 KiB `storage_test` LittleFS
  partition, separate from production labels;
- deterministic reading and event values;
- reduced logical log quotas so retention behavior can be exercised quickly;
- the real ESP-IDF NVS, LittleFS and software-reset paths; and
- a test-only physical-log inspector and raw-record injector; and
- a fixed non-production identity plus deterministic sensor, receiver-free
  radio, clock and randomness adapters that never access sensor or radio GPIOs.

New RTC and `node_core` terminal sleeps are 250 ms only in this test build. The
production radio-cycle, airtime, ACK-wait and retry-jitter constants remain
unchanged; the deterministic clock advances them without wall-clock waiting.

Test state is erased before and after a scenario, but is preserved between the
reset/deep-sleep stages of that scenario. A test-only storage inspector may read
and decode physical logs for assertions; it is never exposed as a production
interface. A compile-time-only `node_core` hook may force software restart
immediately after `node_rtc_record_take` has copied and invalidated RTC state and
before the sample-ID claim. Neither facility is present in production builds.

Connect the ESP32-C6-DEVKITM-1 through the USB-C connector labelled `UART`, not
the connector labelled `USB`. The runner requires the external USB-to-UART
bridge to remain enumerated while the ESP32 resets and enters deep sleep; the
native USB device disconnects during those transitions and is not a supported
test ingress.

Install the runner in the repository virtual environment and activate ESP-IDF
before invoking the entry points:

```text
.venv/bin/pip install -r firmware/tests/requirements-hardware.txt
source ~/esp/esp-idf/export.sh
make test-host
make test-hardware PORT=/dev/serial/by-id/...
make test-hardware-slow PORT=/dev/serial/by-id/...
make test-hardware-all PORT=/dev/serial/by-id/...
```

`PORT` defaults to `/dev/ttyUSB0`. `test-hardware` runs only cases without the
Unity `[slow]` tag. `test-hardware-slow` runs only `[slow]` cases, while
`test-hardware-all` runs both sets. Every hardware target builds and flashes the
test image before executing it, replacing the application previously on the
board. Fast hardware tests run during ordinary validation. Persistence
compaction/quota/churn, the one-minute platform timer test and the exactly-20
RTC round trips form the explicitly selected slow set.

Physical power-loss testing is deliberately deferred. Software restart is used
only after an operation has returned success. Torn or corrupted records are
injected explicitly to exercise recovery on real LittleFS; these tests do not
claim to reproduce interruption during an in-progress flash operation.

### `node_core` integration

The following receiver-free integration cases are implemented in the bare-board
test application.

- **Current accepted across deep sleep:** start with erased state, use a fixed
  sensor snapshot and return first-attempt `ACCEPTED`. On the next test stage,
  before starting another wake cycle, verify sample ID `0`, the committed NVS
  successor, empty pending storage, one delivery start/finish pair and RTC
  metrics for one accepted attempt.
- **Unacknowledged current becomes backlog:** let cycle 0 exhaust the production
  limits in deterministic-clock silence. After deep sleep, verify sample `0`
  remains pending and previous-current acceptance is false. In cycle 1 accept
  current sample `1` and then backlog sample `0`; verify `(1, CURRENT)` precedes
  `(0, BACKLOG)` and pending storage is empty after the following wake.
- **Permanent current rejection is quarantined:** return authenticated
  `REJECTED_MALFORMED`. After deep sleep, verify the reading is in
  `quarantine.log`, absent from `pending.log`, no backlog delivery was attempted
  and the delivery finish result is `MALFORMED`.
- **Retry later retains current and stops:** preload one older backlog reading,
  return `RETRY_LATER` for the current reading, and verify both remain pending
  while the older reading was never transmitted.
- **Previous metrics reach the next reading:** accept a current reading after
  two attempts. Capture the next wake's finalized plaintext and verify its
  previous-current attempts, delivery time, total attempts, accepted count and
  validity flags match the preceding RTC record.
- **Unexpected restart after RTC consumption:** begin with valid previous-cycle
  state, let the controller copy and invalidate it, then use the test hook to
  call `esp_restart()` before final RTC commit. Verify the following execution
  does not reuse those old metrics.

These cases assert externally visible frames and durable state, not the complete
internal call sequence already covered by host tests.

### RTC memory

The complete RTC suite below is implemented. RTC cases use Unity multi-stage
tests so the assertion stage executes after the required boot transition.

- **Committed record survives deep sleep:** write a committed record with
  distinctive values, enter timer deep sleep and verify every field, the commit
  marker, reset reason and wake cause after boot.
- **Software reset is not a valid RTC transition:** write a committed record and
  call `esp_restart()` without deep sleep. Even if its bytes remain, validation
  must reject it because the reset reason is not `ESP_RST_DEEPSLEEP`.
- **Consumed record stays invalid after restart:** commit, deep-sleep, copy and
  invalidate the record, then software-reset. Verify the RTC-resident marker is
  still invalid.
- **New commit replaces the previous commit:** commit A and deep-sleep; consume
  A, commit B and deep-sleep again; verify only B is exposed and A cannot
  reappear.
- **RTC and NVS sample continuity:** commit RTC state for completed sample `N`,
  deep-sleep and claim `N + 1` from real NVS; validation must succeed. Repeat
  with a deliberately nonconsecutive NVS value and verify rejection.
- **Repeated deep-sleep round trip (slow):** run exactly 20 short cycles
  with a changing counter and bit pattern. Every wake must observe exactly the
  immediately preceding committed record.

Wrong markers, malformed metric combinations and every representability
boundary remain host tests.

### `node_persistence`: NVS

- **Sample IDs are monotonic across restarts:** claim `0`, restart, claim `1`,
  restart and claim `2`; no ID may be reused.
- **Message IDs mirror counter correctness:** independently claim `0`, restart,
  claim `1`, restart and claim `2`; no transport ID may be reused.
- **A successful claim is already committed:** run this scenario for both
  counters by claiming once, immediately restarting after the successful return
  and verifying the next claim or stored value is its successor.
- **Exhaustion preserves state:** seed each counter at the exhaustion boundary
  and verify claiming fails repeatedly across restart without wrapping or
  changing the stored value.
- **Counter independence:** claim one sample ID and one message ID and verify
  that each stored successor advances without disturbing the other.
- **NVS and LittleFS initialize independently:** claiming an ID must not mount
  LittleFS; the first log operation mounts it without disturbing NVS state.

### `node_persistence`: pending readings

- **Pending round trip survives restart:** append one reading, restart, peek and
  compare its sample ID and canonical 32-byte body.
- **Bound backlog frame survives restart:** append a reading, bind a distinct
  message ID and exact 54-byte backlog frame, restart, and verify the ID and
  every frame byte are returned unchanged.
- **Pending selection is newest-first:** append samples 0, 1 and 2, then verify
  peek/removal order 2, 1, 0.
- **Removal requires the expected ID:** request removal with a different ID and
  verify `CURAG_ERECORD_MISMATCH` while the physical file remains unchanged.
- **Removed data stays removed:** remove the newest reading, immediately restart
  and verify it does not reappear.

### `node_persistence`: other record families

- **Quarantine survives restart:** append a quarantined reading, restart and
  verify its exact physical record.
- **Delivery events are immediately durable:** append start and finish events,
  restart without relying on final `sync_all`, and verify both records.
- **Diagnostics become durable after `sync_all`:** append a diagnostic, call
  `sync_all`, restart and verify its exact encoding.
- **`sync_all` leaves LittleFS registered:** verify buffered state is
  synchronized and owned handles are closed while the LittleFS VFS remains
  registered until reset/deep sleep.

### `node_persistence`: tail recovery

Host tests cover every structural boundary. Hardware tests use representative
records on real LittleFS:

- **Torn tail is removed:** append a valid record and inject a partial suffix.
  The first semantic operation must return `CURAG_ECORRUPT_RECORD`, synchronize
  the truncation and not execute its original request; the next call must expose
  the preceding valid tail.
- **CRC-invalid tail is removed:** inject a complete record with an invalid CRC
  and verify the same two-call recovery behavior.
- **Unsupported complete tail is removed:** inject a CRC-valid record with an
  unsupported type or version, then verify `CURAG_EUNSUPPORTED_RECORD`, exact
  tail removal and successful access on the next call.
- **Semantically invalid tail is removed:** inject a CRC-valid known record with
  an invalid payload and verify `CURAG_ECORRUPT_RECORD` and exact-tail removal.
- **Unprovable boundary is preserved:** create a file with no trustworthy
  boundary, save its size and checksum, then invoke recovery twice. Both calls
  must fail and the complete file must remain byte-for-byte unchanged.
- **Append does not bury corruption:** place a recoverable invalid tail and
  request an append. Recovery must happen, but the requested record is appended
  only by a later explicit call.

### `node_persistence`: retention and stress

- **Pending compaction retains the newest half (slow):** with a reduced test
  quota, trigger compaction and restart. Verify the expected newest complete
  records remain in their original order and `pending.compact` is absent.
- **Interrupted compact is not promoted:** construct an authoritative
  `pending.log` beside a stale test-created `pending.compact`; verify the
  temporary file is never promoted and is removed only after validation of the
  authoritative log.
- **Full non-pending logs reject new records (slow):** use reduced test quotas
  for quarantine, diagnostic and delivery logs. Existing records remain
  unchanged and further appends return `CURAG_ELOG_FULL`.
- **Persistence churn (slow):** perform several hundred deterministic
  append/peek/remove operations with periodic software restarts and compare the
  recovered backlog against a small reference model after every restart.

### `node_sensors`: hardware strategy

Sensor hardware tests link the production `node_sensors`, low-level drivers and
board configuration. They complement the private fake-backend tests by checking
real ADC conversion, I2C and 1-Wire behavior, configured sensor identity and the
physical switched rail. They do not impose agricultural plausibility ranges or
calibration policy on production sampling.

The approved electrical implementation is documented in
[`test_apps/on_device/SENSOR_CARRIER.md`](test_apps/on_device/SENSOR_CARRIER.md).
The sensor carrier has one authoritative schematic and separate annotated
connection diagrams for the declared fixture states below. A state is created
only by fitting or removing the specified sensor connectors and reference-input
jumpers; no active fault-injection circuit is part of the pilot fixture. Power
is removed from the carrier before changing state. The selected state, carrier
schematic revision, DUT identity and configured DS18B20 ROM identities are
recorded with every run.

Missing-device and reference-input cases are selected explicitly from pytest.
A state mismatch is a failed precondition rather than a skipped or reclassified
test. The sibling
[`test_apps/sensor_carrier`](test_apps/sensor_carrier/README.md) Unity app selects
only implemented operations explicitly. `nominal` supports discovery-independent
acquisition, repetition, BME sleep observation, final cleanup, DS identity and electrical/reset/sleep
checks; `adc_reference` selects guided A/B conversion/mapping. `missing_ds0` and
`missing_ds1` and `missing_bme280` select acquisition plus their respective preflight and
sample-return hold using the [missing-probe commands](test_apps/sensor_carrier/README.md#missing-ds-probes-and-nominal-restoration).
The separate `reading` operation supports all five declared fixtures through
production core; `adc_reference` still requires guided A/B measurements. See
the [reading commands](test_apps/sensor_carrier/README.md#production-core-reading-mapping).
Discovery remains
setup and permits unknown identities. Acquisition preflight requires both
distinct configured identities in the build and checks the declared inventory:
exactly both ROMs for `nominal`, `adc_reference` and `missing_bme280`, exactly ROM1 for
`missing_ds0`, or exactly ROM0 for `missing_ds1`. The declared missing ROM must
be absent; additional/replacement devices and incomplete enumeration fail
preflight. BME280 identification at 0x76 is required for the connected fixtures.
`missing_bme280` instead requires a NACK-specific probe result of NOT_FOUND
(261); responding wrong IDs, timeouts, generic errors and failed cleanup cannot
establish absence. Preflight releases
resources and resets before sampling. Full ELF/configuration and factory-MAC
checks apply on every boot.
Missing/ignored/zero cases, an incomplete repetition count, fixture mismatch or
missing operator measurements cannot become a passing requested case.

The sample-return and separate final-cleanup holds remain untouched after their
respective operations. Automatic holds last 60 seconds. All guided awake
electrical holds, including separate final cleanup and the on/off stages of
reset/sleep operations, allow 180 seconds or early acknowledgement after valid
readings. Held-reset measurements also allow 180 seconds and prompt EN release
immediately after valid readings. Guided acceptance records independent
meter observations; paired A/B procedures cannot accept only one position. A
600-second maximum deep-sleep observation allowance permits all four off-state
readings after settling and a separate YES attestation, then the operator ends
sleep by pressing EN/reset when prompted. No timer wake is configured for this case.
Host deadlines are orchestration policies, not BME recovery guarantees. App-only
forwarding observers monitor the real production driver calls without supplying
values or adding hardware operations. See the app README for exact implemented
commands, measurement prompts and incomplete-result handling.

`--exploration` is an explicitly non-accepting alternative to `--sensor-guided`
for gate-on/off, acquire, final-cleanup, reset, held-reset and deep-sleep. It
records a description of actual wiring and arbitrary live text with entry UTC,
phase and elapsed time, saving every entry immediately. `/done` advances the
current observation or finishes the final one. Observation waits have no time
limit; active acquisition, boot and command handshakes retain their deadlines.
Awake waits yield to the idle task. Unexpected reboot, disconnect or test failure
ends the observation; subsequent notes cannot be attributed to the old state.
Interrupted records remain distinct from explicitly completed exploration.

Pure electrical exploration permits declared disconnected sensor branches and
does not run nominal inventory preflight. Build/configuration/DUT checks remain
mandatory. Acquire/final-cleanup exploration still requires both configured
identities and nominal preflight, production sampling and its software assertions. Exploration
introduces no missing-device sampling cases. Free text is not interpreted as
validated voltages or stable-display attestations and cannot grant acceptance
or serve as an A/B acceptance prerequisite. Completed exploration remains a
non-passing pytest orchestration result labeled `exploration_complete_not_acceptance`.
Exploration deep sleep has no configured timer wakeup: after `/done` the operator
presses EN/reset, and the runner verifies that reset rather than timer wakeup.
Guided deep sleep uses the same real sleep/reset path, with a bounded observation,
validated meter readings and explicit attestation required for acceptance.

The two soil probes are sampled in air with the production 200 ms switched-rail
stabilization and ADC averaging path. Whenever a sensor hardware case expects a
connected air-exposed soil probe to be valid, every acquired value for that
group must be in the inclusive range 2,000 through 2,700 mV. This applies to
every iteration of the repeated-acquisition case and to unaffected soil groups
in missing-device states. The deliberately lower `adc_reference` voltages are
exempt because they verify conversion and channel mapping rather than an air
reading. This is a test-fixture acceptance condition for the connected probes;
it does not change the production meaning of a validity bit or add an agronomic
plausibility policy to `node_sensors`.

### `node_sensors`: declared fixture states and test mapping

| Fixture state | Physical configuration | Automated mapping | Manual or host-guided mapping |
|---|---|---|---|
| `nominal` | Both soil probes, both externally powered DS18B20 probes with their configured ROM identities, and the BME280 are connected. | All groups acquired; every soil reading is 2,000–2,700 mV inclusive; atomic enclosure group; at least 100 repeated switched-rail acquisitions; idempotent final cleanup; successful diagnostic slots remain empty; the carrier `reading` case verifies all seven sensor protocol validity bits. | DS18B20 identity mapping; sampling-owned successful shutdown; stable active-low gate states; final cleanup; intentional restart cleanup; held-reset and deep-sleep default-off behavior; post-sampling and deep-sleep back-power check. |
| `missing_ds0` | The complete power, ground and data connector for the probe whose ROM is configured as logical channel 0 is removed. Channel 1 remains connected; both soil probes and the BME280 remain connected. | Channel 0 temperature is zero and invalid; channel 1 and the other independent groups remain usable; both soil readings are 2,000–2,700 mV inclusive; the result is partial and the channel-0 diagnostic pair contains the exact backend kind and status while unaffected pairs are empty. | Verify the switched rail is off after the failed acquisition path. |
| `missing_ds1` | The complete power, ground and data connector for the probe whose ROM is configured as logical channel 1 is removed. Channel 0 remains connected; both soil probes and the BME280 remain connected. | Channel 1 temperature is zero and invalid; channel 0 and the other independent groups remain usable; both soil readings are 2,000–2,700 mV inclusive; the result is partial and the channel-1 diagnostic pair contains the exact backend kind and status while unaffected pairs are empty. | Verify the switched rail is off after the failed acquisition path. |
| `missing_bme280` | The complete BME280 power, ground, SDA and SCL connector is removed. Both soil probes and both DS18B20 probes remain connected. | Enclosure temperature, pressure and humidity are all zero and the single enclosure group is invalid; the other four groups remain usable; both soil readings are 2,000–2,700 mV inclusive; the result is partial and the enclosure diagnostic pair contains the exact backend kind and status while unaffected pairs are empty. | Verify that the already-disabled switched rail remains off after the independent BME280 failure. |
| `adc_reference` | Both soil-probe connectors are removed from the ADC inputs and two distinct, safe reference voltages share DUT ground and are selected by removable jumpers. Both DS18B20 probes and the BME280 remain connected. | No unattended pass/fail case is assigned: the applied voltages are external measured quantities. Both ADC conversions succeed, both soil groups are valid, and their converted millivolt values are exposed for comparison; the air-probe range does not apply. | Measure both ADC test points immediately before acquisition, compare each reported value with the corresponding measurement, then exchange the two reference-input jumper positions and verify that only the intended logical channel follows each voltage. |

`adc_reference` uses two distinguishable voltages deliberately below the
2,000–2,700 mV air-probe range so reference injection cannot be mistaken for a
connected probe reading. The carrier provides nominal 1.185 V and 1.650 V
divider outputs through 1 kOhm ADC series resistors and requires each converted
result to agree with its immediately measured test-point voltage within 75 mV.
Measurements are recorded rather than replaced by nominal resistor values.
The 75 mV fixture allowance is deliberately wider than Espressif's specified
40 mV total error for a calibrated ADC at the attenuation used by the firmware;
it leaves margin for the meter, wiring and residual noise without accepting a
channel swap. The two freshly measured reference intervals must remain
disjoint, separated by more than 150 mV, with VREF_A lower than VREF_B.

The DS18B20 identity-mapping procedure remains in the `nominal` state because
both configured devices stay present: create a clear temperature difference,
record the logical-channel result, exchange their physical connector positions
and repeat. Logical identity must continue to follow the configured ROM rather
than connector position or enumeration order.

The BME280 low-power case uses `nominal` and observes real status/mode registers
after sampling; it does not write a mode to obtain a passing observation. Exhaustive simultaneous backend failures and power-gate fault
injection remain host-test responsibilities and have no physical fixture state.

`node_sensors_sample_all` owns immediate rail cleanup: after any path that may
enable the shared soil/DS18B20 rail, it attempts to disable it before beginning
independent BME280 acquisition. The final `node_sensors_force_power_off` call is
an independent, idempotent safety attempt. If acquisition and rail cleanup both
fail, power control takes precedence in `err_curag_t`, operation and the single
component slot. Blocked group slots preserve the occurrence, but not
necessarily the exact earlier shared backend status. Both public operations use
the same private gate-off primitive.

### `node_sensors`: automated cases

It links real sensors/backends, core, codec/crypto and test-partition
NVS/LittleFS persistence. An app-only forwarding observer retains the exact
sample returned to core; a local radio adapter captures the constructed frame
and reports a non-started local error, leaving the reading pending. The terminal
sleep port records the unchanged requested duration and returns for assertions.
No RF or actual sleep behavior is claimed by this integration operation.

The opened frame plaintext must equal the actual pending record's canonical
32-byte body. Each decoded sensor field must equal the same acquisition, with
invalid fields zero. Expected component validity and protocol sensor flags
(`flags & 0x00fe`) are:

| Fixture | Component validity | Protocol sensor flags |
|---|---:|---:|
| `nominal` | `0x1f` | `0x00fe` |
| `missing_ds0` | `0x1b` | `0x00f6` |
| `missing_ds1` | `0x17` | `0x00ee` |
| `missing_bme280` | `0x0f` | `0x001e` |
| `adc_reference`, positions A/B | `0x1f` | `0x00fe` |

The three enclosure flags must agree on both success and failure. Partial
sensor results preserve every independent valid field in the reading. Exact
missing-fixture diagnostics and sensor timing remain as specified below.
Reference integration requires fresh meter observations and the existing
guided A/B acceptance criteria; no unattended ADC-reference pass is assigned.
Each independent scenario uses a fresh test identity/key lifetime and isolated
storage. Erasing its counters retires that identity. Existing electrical holds
remain separate: core's final cleanup cannot establish sampling-owned shutdown.

- **All groups acquired:** with the complete fixture, one call sets all five
  component validity bits and returns values from the expected physical
  channels. The corresponding node reading sets all seven sensor protocol bits.
- **Atomic enclosure group:** every successful BME280 acquisition validates
  temperature, pressure and humidity together. It is impossible for a
  node-generated sample to set only a subset of the three enclosure protocol
  bits.
- **Repeated switched-rail acquisition:** perform at least 100 consecutive
  calls in one boot and require the complete requested count. Both configured
  DS18B20 identities remain stable. App-only observers forward real driver
  calls unchanged and verify a successful bus-wide Skip ROM/Convert T command,
  at least 750 ms before temperature scratchpad reads, and successful reads
  addressed to both configured ROMs. The 750 ms criterion comes from the
  DS18B20 datasheet maximum 12-bit conversion time; production retains its
  existing driver wait. Equal temperatures do not establish or refute freshness.
  Record every acquisition duration and enforce a 30-second host deadline per
  iteration. The target also checks the actual returned whole-sample duration
  against 30 seconds in every declared fixture. Stage-08 acceptance comprises
  bounded BME behavior under injected BME faults and actual completion on the
  declared finite fixtures. It is not a universal whole-call deadline for
  changing 1-Wire inventories or stalled SDK resources: production enumeration
  has no explicit attempt cap. Host timeout means failure, not target recovery.
- **BME280 returns to low power:** the nominal `bme-sleep` case and each repeated
  nominal sample read real 0xF3 status and 0xF4 mode without a reset, mode write
  or new conversion. Require measuring/im_update clear and mode 0, and compare
  the returned sample with its preserved copy. A read failure fails the case.
  Do not insert even a BME read into the untouched sample-return electrical hold.
- **Final cleanup is idempotent:** call `node_sensors_force_power_off` more than
  once after successful sampling. Every call succeeds without initializing a
  sensor bus, enabling the rail or changing the collected sample.
- **Missing DS18B20 channel 0:** with only its configured ROM identity absent,
  channel 0 is zero and invalid, its diagnostic pair reports the driver result,
  and channel 1 plus independent groups remain usable.
- **Missing DS18B20 channel 1:** verify the symmetric channel-1 behavior.
- **Missing BME280:** all three enclosure values are zero, the single enclosure
  group is invalid, its diagnostic pair reports the failure and gated groups
  remain usable. Require partial result `0x00030002`, validity `0x0f`, INITIALIZE,
  V1 length 48 and only the enclosure pair at offset 40 equal to `(ESP_ERR,264)`
  for the selected IDF chip-ID NACK path. Require the untouched post-sample OFF
  observation; preflight's NOT_FOUND is a separate fixture check.
- **Diagnostic slot mapping:** for each detectable hardware failure above,
  unaffected pairs are `(NONE, 0)` and the failing fixed pair contains the
  exact backend kind and status. Host tests remain responsible for exhaustive
  simultaneous failures and power-gate fault injection.

Disconnecting an analog soil probe normally still leaves a readable ADC input
and is not necessarily a detectable acquisition error. Soil hardware tests
therefore validate voltage conversion and channel mapping rather than requiring
an unplugged probe to clear a validity bit.

For the selected onewire_bus 1.1.1 / ds18b20 0.4.0 path, enumeration ends with
`ESP_ERR_NOT_FOUND`. The production backend marks a configured ROM without a
device handle as `DRIVER_STATUS`, `ESP_ERR_NOT_FOUND` (`0x105`, decimal 261),
operation `INITIALIZE`; it still converts and reads the surviving ROM.
Consequently either single-missing fixture must return sensor
`EPARTIAL_SAMPLE` (`0x00030002`), context V1 of length 48, with validity `0x1b`
for `missing_ds0` or `0x17` for `missing_ds1`. Only the missing channel pair
(offset 24 or 32) is `(2, 261)`; all other pairs, including the component pair,
are zero. The encodings remain owned by `INTERFACE.md`. Timeout, CRC, shared
bus and cleanup errors are failures of these requested cases, not alternative
acceptable missing-probe outcomes.

Common app-only forwarding observations check nominal and all three missing states
against the existing production soil acquisition conditions: the configured
200 ms stabilization request, GPIO0/1 ADC channels, 12 dB attenuation, 16 reads
per channel separated by 2000 us delay requests, and calibrated conversion of
the rounded average. These checks observe the unchanged driver path without
supplying values, adding delay or measuring the electrical rail-rise waveform.
The DS observer requires the real broadcast conversion and existing freshness
criterion, with reads addressed only to the configured ROMs expected present.
The rail-off attempt must complete successfully before BME acquisition.

### `node_sensors`: manual electrical cases

These tests use the available AN8008 multimeter for stable DC observations.
This section owns the wiring preflight, measurement procedure, settling times
and voltage limits. The schematic and fixture connections are defined in
[SENSOR_CARRIER.md](test_apps/on_device/SENSOR_CARRIER.md). Commands are in the
[sensor-carrier README](test_apps/sensor_carrier/README.md); recorded runs are in
[sensor-carrier evidences/](test_apps/sensor_carrier/evidences/).
Measure DC voltage relative to an adjacent carrier ground point. The meter's
hold function does not provide transient capture or a min/max measurement.

After readiness, wait at least 5 seconds for on-state readings or 10 seconds
for off-state readings. Record each point only after three consecutive stable
display updates.The test application provides separate power-on and power-off holds through the
production gate-control implementation. Power-on readiness follows at least
the production 200 ms stabilization interval. Each hold remains stable for at least
60 seconds or until host acknowledgement. Every guided awake electrical hold
allows up to 180 seconds, with a 195-second host observation/result deadline,
and finishes as soon as complete valid operator readings trigger an explicit
host acknowledgement. This includes `acquire`, `gate-on`, `gate-off`,
`final-cleanup`, transition-on before reset/sleep, and reset-off after restart.
Held-reset observations allow 180 seconds, with release prompted immediately
after valid readings and the same 195-second budget for release/boot. Guided
deep sleep allows at most 600 seconds for readings and a separate YES attestation
that they were measured while the MCU remained asleep. Save readings immediately;
only complete valid readings plus YES permit the EN/reset prompt. Keep USB
connected and press/release EN only after that prompt. Reject a boot, failure or
UART loss observed before attestation completes. Verify the same DUT/image after
reset and the C6 EN/POWERON reset reason. Reset/boot has the existing 30-second
active deadline, capped by the 615-second overall observation/boot deadline.
Missing readings, attestation or reset remain incomplete. This replaces the
carrier's earlier compulsory 600-second dwell and timer-wakeup assertions;
the separate bare-C6 one-minute timer-deep-sleep case remains unchanged. It
establishes the measured electrical observation, not a ten-minute dwell or
timer wakeup. ADC reference meter input before sampling allows
180 seconds and starts acquisition on complete input. DS/ADC post-sample holds
have no live meter prompt and retain their automatic 60 seconds, as do unguided
runs. Preparation/wiring/thermal confirmations are not observation windows. Missing or
invalid measurements/acknowledgement remain incomplete or failed. Settling and
voltage criteria are unchanged. A separate sample-return hold calls
the unchanged `node_sensors_sample_all`, reports its result only after the call
returns, and then keeps the CPU awake without another sensor or gate-control
operation. In particular, it does not call `node_sensors_force_power_off`
before the observation. No hold is inserted into the production sampling path.

- **Soil ADC conversion:** apply or measure safe known voltages at both soil
  inputs and compare `soil_0_mv` and `soil_1_mv` against the freshly measured test points within
  the carrier's 75 mV fixture comparison allowance.
- **Soil channel mapping:** use the carrier's reference positions A and B,
  remove power before exchanging the two leads, measure both inputs again and
  verify each logical channel follows its connected reference within 75 mV.
- **DS18B20 identity mapping:** create a clear temperature difference between
  the probes and verify configured ROM identity, rather than 1-Wire enumeration
  order, determines channel 0 and channel 1. Confirm the textual ROM byte order
  against the library's enumerated `uint64_t`, then repeat after reconnecting or
  reversing their physical order on the bus with power removed. Predeclare the
  warmed physical ROM and require its logical result to be at least 2 C above
  the other probe in both arrangements. This is an identity discrimination
  criterion, not a thermal-response or accuracy measurement.
- **Sampling-owned successful shutdown:** in the sample-return hold, verify the
  gate is released and the shared rail has settled to off. Because no operation
  runs between the sampling return and the held observation, this proves that
  cleanup occurred inside sampling rather than being deferred until final wake
  cleanup. It does not measure the exact shutdown instant.
- **Stable active-low gate states:** in the dedicated production gate-on hold,
  verify the GPIO/P-MOSFET gate is low and the switched rail is on. In the
  dedicated gate-off hold, verify the external 47 kOhm source-to-gate resistor
  has pulled the released gate to the always-on rail and the switched rail has
  settled off. These holds do not alter production sampling timing.
- **Shutdown after acquisition failure:** repeat with a detectable DS18B20 or
  BME280 failure using the corresponding declared fixture state and verify in
  the sample-return hold that the shared rail still settles off without an
  additional cleanup call.
- **Final cleanup:** after repeated `node_sensors_force_power_off` calls, the
  shared rail remains off.
- **Intentional restart cleanup:** enable the rail, then call production
  `node_platform_esp_restart`, which attempts unconditional sensor force-off
  before ESP-IDF restart. Verify the successful off attempt and empty diagnostic,
  no sensor initialization or gate-on during that cleanup, the software reset
  reason and an untouched post-restart electrical off observation. This tests
  restart-owned cleanup, not hardware default-off for a CPU-only restart.
- **Hardware reset and deep-sleep default:** enable the rail in a dedicated test stage,
  then assert EN/reset or enter deep sleep and verify that the board returns the rail to
  off when the MCU no longer actively enables it. Sensor-carrier deep sleep
  has no timer wake; within 600 seconds enter all four readings and attest YES,
  then press EN/reset when prompted to finish early. The 615-second overall
  observation/boot and 30-second active boot ceilings above apply. Repeat while holding the MCU
  in reset to exercise the external 47 kOhm gate pull-up and R12 rail discharge
  without a firmware shutdown path.
- **Back-power check:** in the real post-sampling and deep-sleep pin states,
  measure the disabled rail for voltage fed through ADC, 1-Wire, pull-up or
  protection-diode paths with permanent R12 fitted and no additional test load.
  If it remains above 0.1 V after 10 seconds, retain the failure and investigate
  using separately declared exploratory wiring. Do not add another 100 kOhm
  resistor in parallel or remove R12 for acceptance. The earlier no-resistor
  versus temporary-load comparison remains diagnostic evidence only. A low
  voltage with R12 fitted can mask weak back-power; it does not establish the
  sleep-current budget. A gate-control voltage alone is not sufficient
  evidence of the switched-rail voltage.

The devkit and multimeter cannot establish the 200 ms rail-rise waveform, the
exact shutdown instant, brief boot/reset/deep-sleep glitches, MOSFET switching
edges or inrush, digital-bus waveforms, or the final microamp sleep budget.
Those require an oscilloscope or current instrumentation on the custom board;
these pilot tests establish functional switching, stable channel identity and
absence of obvious steady-state back-power. Successful acquisition after the
unchanged production delay is indirect functional evidence, not a waveform
measurement.

### Platform ports

The bare-board monotonic-clock, random-range, software-reset-reason and
one-minute timer-deep-sleep cases are implemented. The shared-radio-clock case
remains deferred until SX1262 hardware is available. These tests exercise the
concrete ESP-IDF adapters; they do not add statistical randomness tests or
duplicate controller policy already covered by host fakes.

- **Monotonic clock:** take many immediate `monotonic_us` readings and readings
  before and after short blocking delays. Every value must be greater than or
  equal to the preceding value within the wake.
- **Shared radio clock domain:** extend the existing radio timing cases by
  capturing `monotonic_us` immediately before and after a transmission and
  asserting:

  ```text
  before <= set_tx_at_us <= tx_done_at_us <= after
  ```

  Apply equivalent enclosing bounds to RX/DIO1 timestamps. This verifies that
  `node_core` and the private radio backend use the same clock domain without a
  separate radio exchange.
- **Software-reset reason:** in a multi-stage test, call `esp_restart()` and
  assert after boot that `get_reset_reason() == ESP_RST_SW`. This assertion may
  be incorporated into the existing RTC software-reset case.
- **One-minute deep-sleep round trip (slow):** emit a final pre-sleep serial
  marker, request 60 seconds of timer deep sleep and assert after boot that
  `get_reset_reason() == ESP_RST_DEEPSLEEP`. `pytest-embedded` measures elapsed
  time with the host monotonic clock from the pre-sleep marker to the first
  post-boot marker. Accept when:

  ```text
  abs(observed_duration - 60 s) <= 15% of 60 s + 2 s
  ```

  The extra two seconds accommodate boot and harness latency. This physical
  slow test is run manually when desired and may share its stages with the RTC
  deep-sleep round-trip test.
- **Random-range smoke:** generate several thousand values for each inclusive
  range `[100000, 500000]`, `[0, 1]` and `[0, UINT32_MAX]`. Every result must be
  inside its requested range. This checks range and overflow behavior only; it
  makes no statistical claim about uniformity.

### `sx1262_radio`: hardware strategy

The radio circuit and manual connection states are documented in
[SENSOR_CARRIER.md](test_apps/on_device/SENSOR_CARRIER.md#sx1262-radio-fixture).
Assembly/static preflight and the first RF operating envelope are specified
below. All executable tests in this radio hardware section remain deferred:
there is no radio test application, receiver-peer harness or retained RF
acceptance run in the current implementation.

SX1262 hardware tests exercise behavior that the fake backend cannot prove:
real SPI/BUSY/DIO1 operation, RF interoperability, IRQ timestamps,
direction-specific IQ, physical state transitions and bounded deadlines. They
do not repeat every invalid argument, backend error or exact deadline boundary
already covered on the host.

The node DUT runs the production `sx1262_radio` component and ESP backend through
ESP-IDF Unity, with pytest-embedded as host orchestration. The selected pilot
peer is the actual Pi 3B receiver hardware and its second Waveshare module,
running separate component test code. Full receiver application readiness is
not required for these component tests. The peer can:

- receive node uplinks with normal IQ and report exact payload bytes;
- transmit arbitrary downlink payloads with inverted or normal IQ;
- select the pilot or an intentionally different sync word; and
- schedule transmissions relative to a pytest command and report its own
  observations.

If the receiver is a Raspberry Pi, ordinary pytest fixtures control its test
process while `pytest-embedded` controls the ESP32. If the receiver is another
MCU, `pytest-embedded` may run it as a second DUT. The peer implementation is
separate from the node component so the exchange tests two real ends rather
than a node radio loopback.

Tests use antennas in fixed, repeatable positions at least a few metres apart;
the +14 dBm radios are not placed immediately beside one another. Exact RSSI
and SNR repeatability is not assumed.

Each independent case reboots the ESP32 so ordinary BSS restores the hidden
radio singleton to `UNTOUCHED`. No test-only singleton setter, getter or reset
operation is required by this plan.

Timing assertions are asymmetric. For calculated airtime `T` from
`sx1262_radio_airtime_us`:

```text
T <= tx_done_at_us - set_tx_at_us <= ceil(T * 1.10)
```

The current 54-byte reading profile therefore permits 102.656 through 112.922
ms. For a receive call made at `S` with absolute deadline `D`, normal silence
must return within:

```text
D <= returned_at_us <= D + ceil((D - S) * 0.15)
```

Hardware packets are scheduled with generous separation from the deadline;
the exact equality boundary remains a deterministic host test.

### `sx1262_radio`: static preflight

The [radio sections of SENSOR_CARRIER.md](test_apps/on_device/SENSOR_CARRIER.md#sx1262-radio-fixture)
own the node circuit, connector numbering, parts and three declared radio
states. This procedure establishes node static assembly readiness only. It
neither sends SPI commands nor validates radio initialization, RF, interrupt
timing or sleep current. Wiring and multimeter observations are operator-owned.
Receiver wiring/preflight belongs to receiver work; a node preflight record
cannot establish peer readiness for the later RF exchange.

1. **Identify and isolate.** Record the C6 and node module identities, antenna
   kit, radio state and sensor state. Check the documented HF layout and
   `ANT_SW A (3V3)` population. Disconnect UART USB power and any other
   power/back-power path. Verify the carrier rail is below 0.1 V before
   changing wiring. Leave battery/Pico/VBUS/VSYS connections absent and the
   module battery switch off. Attach the antenna unpowered.
2. **Check the unpowered circuit.** Trace every C6 GPIO through the carrier
   and module contact against the C6/Pico numbering. Check intended ground
   continuity, capacitor polarity, RN5's 10 kohm value and its connection from
   MCU-side DIO1 to GND. Check for unintended shorts between supply, ground
   and signals. Verify both absence jumpers are open whenever the radio is
   installed. To select `radio_absent`, remove the whole board before fitting
   BUSY-to-3.3 V and MISO-to-GND ties; confirm that all nine module connections
   are absent and the ties reach GPIO19/GPIO14 respectively. RN5 stays on the
   carrier in every state. A capacitor charging on the resistance range is not
   a steady short; retain unresolved readings. With the relevant loom detached,
   verify DIO1 continuity when its link is fitted and no direct continuity
   across the open link, accounting for semiconductor paths through a module.
3. **Make the powered check incapable of issuing radio commands.** With
   power removed, disconnect the seven-signal `J_HOST_N`; leave carrier supply
   and ground intact. When a radio is fitted, connect a temporary lead from
   carrier `TP_N_CS` to `TP_N_3V3` before powering it. This holds CS inactive
   while every C6 signal driver is isolated. The radio's internal NRESET
   pull-up releases reset; no external reset resistor is fitted. Hold the C6
   EN/reset button before connecting UART USB power and keep it held until
   USB power is removed again. Do not access/flash the C6. In `radio_absent`,
   fit the two absence jumpers and omit the temporary CS lead.
4. **Record stable DC readings.** Use the AN8008 in DC-voltage mode, black
   lead at `TP_N_GND`. Check the C6's 3V3 header and `TP_N_3V3`; when a module
   is fitted, also measure its U1.36 supply contact. Perform resistance and
   continuity checks only unpowered. The assembly admission band remains
   3.2-3.4 V at the supply points, centred on the specified 3.3 V. It is a
   conservative bench check, not a new module operating range. Wait at least
   five seconds after power application and record three stable display
   updates, consistent with the existing DC observation convention. This wait
   makes no claim about the production 5 ms TCXO startup or supply transients.
   Never put the current-range leads across the supply.
5. **Check the static levels in the table below.** Low means at most
   `0.2 * measured_local_3V3`; high means at least `0.8 * measured_local_3V3`
   without exceeding that supply beyond meter resolution. These bands check
   defined wiring levels, not edge timing. Remove power before every state
   change and keep `J_HOST_N` isolated during readings. Remove the temporary
   CS lead before removing the radio, and remove both absence jumpers before
   reinstalling it. Repeat the supply check in each state. An intermediate
   or unstable level where the table requires high/low, a missing required
   measurement or a wiring mismatch leaves preflight unresolved; investigate
   without widening limits.
6. **Restore and retain.** Remove power. Remove both absence jumpers before
   reinstalling the Waveshare, restore `nominal` with its DIO1 link fitted,
   and remove the temporary CS lead before reconnecting `J_HOST_N`. Verify
   the absence ties are open, no C6 output is tied to a supply rail, and the
   sensor reservations are intact. Leave the node unpowered. Retain readings
   and setup photographs tied to the C6, module, connections and date. Before
   executable use, confirm a reviewed image that waits without transmitting
   for an explicit selected case, its resolved pin configuration and operator
   DUT readiness. A separate peer configuration/readiness record is required
   for RF tests; its component process must exclusively own its radio and
   other automatic transmitters must be disabled.

| Node power-only observation (`J_HOST_N` isolated) | CS | RESET | BUSY | MISO | MCU-side DIO1 |
|---|---|---|---|---|---|
| `nominal`, temporary CS-high lead fitted | High | High, internal radio pull-up | Low after power-on settles | High impedance; no voltage criterion | Low, no IRQ armed |
| `radio_absent`, both absence jumpers fitted | Unconnected; no voltage criterion | Unconnected; no voltage criterion | High, direct 3.3 V tie | Low, direct GND tie | Low from RN5 |
| `dio1_disconnected`, temporary CS-high lead fitted | High | High, internal radio pull-up | Low after power-on settles | High impedance; no voltage criterion | Low from RN5; open link independently checked |

Record each required reading individually. MISO is not sampled by a host in
this power-only check; its deselected high-impedance state is specified in
[Semtech section 8.2](https://files.waveshare.com/wiki/SX1262-XXXM-LoRaWAN-GNSS-HAT/DS_SX1261-2_V1.2.pdf).
Removing its pull-down removes the former fitted-module static-low expectation;
exact received SPI/RF bytes remain required in executable tests. With the
module absent, all three MCU inputs instead have defined levels from the two
jumpers and RN5. Confirm their continuity to the MCU pins unpowered after
refitting `J_HOST_N`.

The low open-DIO1 measurement alone does not establish disconnection; the
unpowered link check does. A BUSY-low observation does not establish successful
SPI/oscillator calibration. Later functional runs must also watch for
supply/reset errors under TX load; this check cannot establish dynamic margin.

For the first RF run, mark two repeatable antenna positions **at least 3 m
apart**, implementing the hardware strategy's few-metres separation. Use the
supplied antennas in the same vertical orientation, at fixed heights, with a
clear path and away from immediate metal obstructions. Record separation,
height, orientation and nearby objects, and photograph both locations. Keep
them fixed through a run and its restoration comparison. Do not bring the
+14 dBm ends next to one another to remedy a failing exchange or interpret
RSSI/SNR as a calibrated power measurement.

If assembly disturbed sensor wiring, repeat the affected existing
[sensor preflight and measurements](#node_sensors-manual-electrical-cases)
using the implemented carrier workflow before claiming continued sensor
acceptance. Changes to the shared supply branch require a nominal sensor
acquisition and gate-on/gate-off DC recheck with the radio quiet. For that
check, disconnect `J_HOST_N` and fit the temporary radio CS-high lead with
power removed; remove the lead unpowered before reconnecting `J_HOST_N`.
Preserve the configured DS identities, BME address and reference-state rules.
Do not repeat every sensor absence case for an untouched connector or treat
the radio design as new sensor evidence.

### `sx1262_radio`: first RF operating envelope

This envelope is for the later first component exchange using the assembled
C6/Waveshare node and Pi/Waveshare peer. Static preflight itself emits nothing.
Use the [protocol's pilot PHY](../protocol/protocol-v2-lora/README.md#pilot-airtime-constants)
unchanged: 868.1 MHz, SF7, 125 kHz bandwidth, CR4/5, LDRO off, explicit header,
eight-symbol preamble, payload CRC, private sync word and configured +14 dBm.
The node sends normal-IQ uplinks and the peer sends inverted-IQ downlinks.
First component packets are known raw byte patterns, not authenticated live
protocol traffic; no identity provisioning or persistent-counter erasure is
needed for this stage. Authenticated tests introduced later must follow the
protocol's test-identity and counter rules.

**Regulatory basis and power evidence.** Checked on 2026-09-15,
[Commission Implementing Decision (EU) 2025/105, annex row 48 and duty-cycle definition](https://www.boe.es/buscar/doc.php?id=DOUE-L-2025-80100)
(official EU text reproduced by BOE) specifies 25 mW ERP in 868.0-868.6 MHz,
with a duty-cycle alternative of at most 1%. Its observation period is a
continuous hour for each transmitter over the applicable band. Use that
alternative here; do not claim LBT/equivalent channel-access compliance or
borrow another sub-band's allowance. The configured 125 kHz channel spans
868.0375-868.1625 MHz nominally; this calculation is not a measured emission mask.

The field-pilot-v2 kit record supplies the nominal 2 dBi antenna gain. With
`P_port` the radio's output in dBm, `G` gain in dBi and `L` total RF-path loss
in dB, the source-based estimate is:

```text
ERP_dBm = P_port + G - L - 2.15
nominal ERP = 14 + 2 - 0 - 2.15 = 13.85 dBm = 24.27 mW
25 mW = 13.9794 dBm; nominal margin is only about 0.13 dB
```

The [ITU short-range-device report](https://www.itu.int/dms_pub/itu-r/opb/rep/R-REP-SM.2153-8-2021-PDF-E.pdf)
explains the 2.15 dB ERP/EIRP reference conversion. Zero loss above deliberately
credits no unmeasured pigtail attenuation. The nominal PA output follows the
[selected implementation](INTERFACE.md#selected-radio-implementation), including
its optimal +14 dBm PA row: the associated `SetTxParams(+22)` register value
does not mean +22 dBm emitted power in that configuration. Apply the same
intended +14 dBm output at the peer through its own driver configuration.

This is the first run's nominal source-based power assessment, **not measured
RF compliance**. Neither actual output tolerance nor antenna gain/path loss
has been measured, and the small nominal margin does not bound their combined
uncertainty. The AN8008 and a successful packet exchange cannot establish ERP,
carrier accuracy or unwanted emissions. Retain the specified kit antennas and
PA settings; a different antenna or conflicting RF evidence requires renewed
assessment. Lower duty cycle does not cure excessive instantaneous ERP.

**Airtime and initial pacing.** The protocol's independently specified values
are the expected results, not values copied from DUT output:

| Transmitter / initial packet | Payload bytes | On-air time | Reserved charge, `ceil(us * 1.10)` | Ten attempted packets |
|---|---:|---:|---:|---:|
| C6 / known uplink | 54 | 102,656 us | 112,922 us | 1,129,220 us |
| Pi / known downlink | 23 | 61,696 us | 67,866 us | 678,660 us |

The independent LoRa symbol calculation gives `Tsym = 2^7 / 125000 = 1.024 ms`.
At explicit-header/CRC-on/LDRO-off, payload symbols are
`8 + 5 * ceil((8 * PL - 4 * 7 + 28 + 16) / (4 * 7))`: 88 and 48 respectively.
Adding the 12.25 preamble symbols gives the two times above. The 10% charge is
accounting margin, not additional emitted airtime or a changed timing tolerance.

Allow at most **ten attempted exchanges per initial session**, with one
54-byte node TX and at most one 23-byte peer TX per exchange. Keep at least
**60 seconds between successive TX starts by the same transmitter**, across
case boundaries and reruns too. The downlink follows within the selected RX
window; the minute is between exchanges, not an added ACK delay. Do not add
automatic retries or other transmissions to this envelope. After ten attempts,
stop and retain the results before another session. Even counting 61 boundary
packets conservatively in an hour gives only 6.889 charged seconds at the node
and 4.140 at the peer; the independent rolling ledger below still controls
admission and includes any other recorded activity.

**Per-transmitter accounting before emission.** For every session, the operator
and later host orchestration must retain a separate journal for each physical
transmitter, covering all its activity in this sub-band, including both test
ends' earlier runs:

- Before any trigger capable of TX, durably reserve that end's full packet
  charge. Admit it only if the retained charges plus the new reservation are
  at most **36,000,000 us**. Retain a charge until a full 3,600 seconds after
  the latest possible end of its transmission, conservatively covering packets
  that straddle an observation-window boundary. Record the physical end,
  run/case, PHY/payload length, charge, trigger time, latest possible TX end,
  outcome and running total. Update completion information without refunding
  an attempted packet merely because its result failed.
- Reserve both ends before an exchange if its command could cause both to
  transmit. Reserve every attempted or uncertain TX, including missing DIO1,
  no peer reception, lost host acknowledgement, test failure or interrupted
  orchestration. A fault result is not evidence that the channel was unused.
  A pending reservation with no trustworthy completion must remain held until
  its possible emission interval has been bounded or the quiet-hour fallback
  has completed.
- Boot/reset, reflashing, peer-process restart, session restart and changing
  fixture state do not clear the journals or restart the minute spacing. Keep
  history outside DUT storage and preserve it across host process restarts.
  Use a time basis whose continuity is established across those records; a
  discontinuous clock cannot age entries out early. Before use, confirm no
  other process can transmit through either module.
- If either end's prior activity, clock continuity or journal is unknown,
  keep that radio physically unpowered for a verified uninterrupted **3,600
  seconds** before starting its empty history. A known quiet hour is evidence;
  merely rebooting or starting an empty file is not. If RF duration may have
  exceeded the modeled packet, stop, remove power and retain the failure;
  bound the possible emission interval conservatively or use this fallback.

This host-controlled component-test envelope does not claim that the node's
production per-wake eight-second allowance enforces a rolling hour across
resets. It does not replace the receiver's eventual production ledger either.
Larger payloads, filtering bursts, transition cases and stress runs retain all
their required coverage; their later run plans must account for every emission
and establish suitable pacing before expanding beyond this initial envelope.
Missing hardware, unresolved preflight, missing peer configuration, zero
selected tests or a mismatched fixture cannot be reported as a passing run.

### `sx1262_radio`: fast automated cases

- **First transmit initializes and sends exact bytes:** reboot the node,
  transmit a known 54-byte pattern and verify the peer receives those exact
  bytes with normal IQ. Require success, `tx_started`, `tx_done`, ordered
  nonzero timestamps and an empty diagnostic.
- **Payload-length boundaries:** transmit distinctive 1-, 54- and 255-byte
  payloads and verify exact length and contents at the peer. Exhaustive invalid
  input lengths remain host tests.
- **TX airtime matches the pilot profile:** for the 54-byte reading frame,
  require the measured pre-`SetTx` to `TX_DONE` duration to satisfy the 10%
  asymmetric airtime bound above. This detects major PHY-profile
  misconfiguration without claiming laboratory-grade RF timing.
- **Inverted-IQ downlink reception:** initialize with an uplink, have the peer
  send a known 23-byte inverted-IQ downlink, and require exact payload,
  `RX_PACKET`, `rx_done_at_us` no later than the deadline and plausibly encoded
  RSSI/SNR.
- **RX deadline with silence:** keep the peer silent and require
  `RX_DEADLINE`, success, zero packet fields and return within the 15% late-only
  bound above.
- **Normal-IQ traffic is filtered during downlink RX:** send a normal-IQ packet
  while the node waits with inverted IQ and require that it is not returned;
  then send an inverted-IQ packet before the same absolute deadline and require
  that packet.
- **Sync-word filtering:** send with a different sync word and require no
  packet; restore the private sync word and send before the same deadline, then
  require the second packet.
- **One absolute deadline survives invalid packets:** send several
  inverted-IQ, PHY-valid but application-invalid payloads. After the caller
  rejects each one, call `receive_downlink_until` again with the original
  deadline and verify the eventual deadline outcome is not extended.
- **Late downlink does not become stale:** transmit after the receive deadline
  using a generous timing guard. Require the first call to return deadline,
  then begin a new TX/RX episode and verify stale IRQ or buffer contents are not
  returned as its downlink.
- **RX-to-TX transition:** transmit, remain in downlink RX until deadline and
  retransmit. The peer must receive both normal-IQ uplinks exactly, proving the
  armed-RX to `STDBY_RC` to TX transition.
- **TX-to-RX-to-TX transition with ACK:** send uplink A, receive an inverted-IQ
  downlink and send uplink B without reinitialization. Verify all payloads and
  direction-specific IQ settings.
- **Sleep is idempotent and terminal for the wake:** initialize, call `sleep`
  twice and require success. A later TX in the same boot must return invalid
  state and emit no packet at the peer.
- **Cold initialization after ESP32 deep sleep:** transmit successfully, put
  the radio into cold-start sleep and enter short ESP32 deep sleep. The first
  transmit after boot must fully initialize and succeed again.

### `sx1262_radio`: manual hardware-fault cases

- **Missing DIO1 after `SetTx`:** disconnect or deliberately mask DIO1 for this
  test. Verify the peer receives the RF payload while the node returns within
  its deadline with `tx_started = true`, `tx_done = false` and diagnostic
  context identifying `TRANSMIT`, `WAIT_IRQ` and `HARDWARE_TOUCHED`.
- **Radio absent:** disconnect the module and verify lazy initialization returns
  a bounded I/O or BUSY error without starting TX or hanging. Final radio
  cleanup must not prevent ESP32 deep sleep.

Use the [declared radio states](test_apps/on_device/SENSOR_CARRIER.md#declared-radio-connection-states):
the removable node DIO1 link for `dio1_disconnected`; for `radio_absent`,
remove the whole module before fitting the MCU-side BUSY-to-3.3 V and
MISO-to-GND jumpers. Remove both jumpers before restoring the module and
`nominal`. RN5 remains on MCU-side DIO1 in all three states. Change all
connections with power removed. These fault cases are manual for the pilot; no automated DIO1 switching or radio-disconnect
fixture is planned yet. Broken-SPI, forced-BUSY and unexpected IRQ injection
remain deferred until a safe controllable fixture exists.

The slow 50-100-exchange RF transition stress test is also deferred. The test
suite must account for all RF airtime and remain within the applicable EU868
duty-cycle policy.

### `sx1262_radio`: current instrumentation limits

The available multimeter may be used for coarse supply-voltage and current
checks. The current setup cannot claim validation of configured +14 dBm RF
power, exact carrier frequency, DIO1 timestamp accuracy against the electrical
edge or SX1262 microamp sleep current while attached to a development board.
Those require suitable RF equipment, a logic analyser or a dedicated low-current
measurement setup and remain deferred.

## Deferred or unresolved hardware-test details

- Physical power-loss injection and its external power-switching rig are
  deferred.
- Hardware-runner provisioning, including permanent serial-port assignment and
  whether slow tests run automatically, remains to be defined when a dedicated
  runner exists.
- The SX1262 host fake is private, and fresh CTest processes reset the hidden
  singleton; the production radio interface intentionally exposes no test-only
  setter, getter, reset or snapshot operation.
- Receiver-peer command transport and scheduling details remain an
  implementation choice; the peer hardware, separate-code requirement and
  observable behaviors are fixed above.

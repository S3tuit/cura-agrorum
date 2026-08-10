# Firmware testing

Status: the `node_core`, `node_persistence`, `node_sensors`, `sx1262_radio` and
platform-port host matrices and the `node_persistence` on-device matrix are
implemented. The remaining RTC, radio and sensor on-device, platform-port, and
`node_core` integration suites below are agreed but not yet implemented.

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
- Each soil or DS18B20 failure zeroes only its corresponding field and clears
  only its validity bit. A BME280 failure zeroes all three enclosure fields and
  clears their three protocol validity bits together.
- Failure of all sensors still produces a structurally valid reading.
- `DEEP_SLEEP_BOOT` is set exactly when reset reason is 8.
- `run_ms` uses the application-start and body-finalization clock samples.
- Pending-read append failure attempts a diagnostic and permits RAM-only
  current delivery.
- RAM-only `ACCEPTED` does not attempt to remove a nonexistent pending copy.
- Sampling that consumes the radio deadline produces no TX.
- Behavior when `run_ms` exceeds `UINT16_MAX` is tested after its policy is
  defined.

### ACKs and retries

- Silence retransmits the identical 50-byte frame.
- Every `TX_DONE` consumes the next scripted random value and produces the
  expected `retry_at`.
- Multiple invalid ACKs followed by a valid ACK stay in one RX interval and do
  not restart that interval.
- Parameterized invalid ACKs cover bad length, bad tag, foreign node, wrong
  sample ID, unsupported control, uplink domain and domain/status mismatch.
- `ACCEPTED`, `RETRY_LATER`, `REJECTED_UNSUPPORTED` and
  `REJECTED_MALFORMED` are each exercised for current and backlog delivery.
- Failure before `SetTx` can take effect neither increments attempts nor
  charges airtime; an uncertain result after the command crosses SPI does both.
- Failure after `SetTx` starts but before `TX_DONE` increments attempts,
  charges airtime, records a local error and does not retry.
- An RX local error after `TX_DONE` terminates delivery rather than being
  treated as silence.
- An authenticated ACK whose `RX_DONE` timestamp is at or before `retry_at`
  wins; a later ACK does not prevent retry.

### Airtime and wall-clock budgets

- Exactly one more reading-airtime charge fits.
- One microsecond more than the available airtime does not fit.
- TX finishing exactly at the wall-clock deadline and just after it exercise
  opposite boundary outcomes.
- Airtime can exhaust while wall-clock time remains, and wall-clock time can
  exhaust while airtime remains.
- Current and backlog deliveries share both limits; neither budget resets per
  reading.
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
- A backlog body remains unchanged; only its domain, header and resulting
  authenticated frame change.
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
- The pair uses `cycle_sample_id`, packet `sample_id` and domain, allowing the
  same backlog packet to be distinguished across wakes.
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
- Every metric is tested at its exact encoded maximum and one above it;
  overflow emits a diagnostic and leaves the outgoing RTC marker invalid.
- `previous_awake_ms` includes final logging and `sync_all`, but excludes RTC
  commit and the sleep call.
- Outgoing metrics never alter an already constructed body or frame.
- RTC fields are stored before the commit marker, which is stored last.
- Every full-cycle scenario calls `force_power_off`, `radio.sleep`, `sync_all`
  and deep sleep exactly once, with no operation after deep sleep.

## `node_persistence`

Tests that do not depend on the pending/log record encoding cover:

- Fresh storage claims sample ID 0; consecutive claims commit and never reuse
  IDs.
- `UINT32_MAX` exhaustion and NVS commit failure return no claimed ID.
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
- Newest-first backlog reconstruction.
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
`firmware/test_apps/on_device`. Its implemented persistence suite links the
production `node_persistence` component and uses:

- a 24 KiB `nvs_test` partition and a 2,944 KiB `storage_test` LittleFS
  partition, separate from production labels;
- deterministic reading and event values;
- reduced logical log quotas so retention behavior can be exercised quickly;
- the real ESP-IDF NVS, LittleFS and software-reset paths; and
- a test-only physical-log inspector and raw-record injector.

The planned `node_core` integration suite will add the fixed node identity,
short deep-sleep/radio durations and scripted sensor, radio, clock and
randomness adapters described below.

Test state is erased before and after a scenario, but is preserved between the
reset/deep-sleep stages of that scenario. A test-only storage inspector may read
and decode physical logs for assertions; it is never exposed as a production
interface. A narrowly placed, compile-time-only `node_core` test hook may force
software restart at the RTC-consumption boundary. Neither facility is present
in production builds.

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
board. Fast hardware tests run during ordinary validation; compaction, quota
and churn cases remain in the explicitly selected slow set.

Physical power-loss testing is deliberately deferred. Software restart is used
only after an operation has returned success. Torn or corrupted records are
injected explicitly to exercise recovery on real LittleFS; these tests do not
claim to reproduce interruption during an in-progress flash operation.

### `node_core` integration

- **Current accepted across deep sleep:** start with erased state, use a fixed
  sensor snapshot and return first-attempt `ACCEPTED`. On the next test stage,
  before starting another wake cycle, verify sample ID `0`, the committed NVS
  successor, empty pending storage, one delivery start/finish pair and RTC
  metrics for one accepted attempt.
- **Unacknowledged current becomes backlog:** let cycle 0 exhaust its shortened
  limits in silence. After deep sleep, verify sample `0` remains pending and
  previous-current acceptance is false. In cycle 1 accept current sample `1`
  and then backlog sample `0`; verify `(1, CURRENT)` precedes `(0, BACKLOG)` and
  pending storage is empty after the following wake.
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

RTC cases use Unity multi-stage tests so the assertion stage executes after the
required boot transition.

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
- **Repeated deep-sleep round trip (slow):** run approximately 20 short cycles
  with a changing counter and bit pattern. Every wake must observe exactly the
  immediately preceding committed record.

Wrong markers, malformed metric combinations and every representability
boundary remain host tests.

### `node_persistence`: NVS

- **Sample IDs are monotonic across restarts:** claim `0`, restart, claim `1`,
  restart and claim `2`; no ID may be reused.
- **A successful claim is already committed:** claim once, immediately restart
  after the successful return and verify the next claim returns its successor.
- **Exhaustion preserves state:** seed the test namespace at the exhaustion
  boundary and verify claiming fails repeatedly across restart without wrapping
  or changing the stored value.
- **NVS and LittleFS initialize independently:** claiming an ID must not mount
  LittleFS; the first log operation mounts it without disturbing NVS state.

### `node_persistence`: pending readings

- **Pending round trip survives restart:** append one reading, restart, peek and
  compare its sample ID and canonical 28-byte body.
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

The normal automated fixture has both soil sensors, both configured DS18B20 ROM
identities and the BME280 attached. Missing-device and channel-mapping cases are
separate physical fixture configurations. They may be selected explicitly from
pytest rather than pretending that unplugging hardware is automated.

`node_sensors_sample_all` owns immediate rail cleanup: after any path that may
enable the shared soil/DS18B20 rail, it attempts to disable it before beginning
independent BME280 acquisition. The final `node_sensors_force_power_off` call is
an independent, idempotent safety attempt. If acquisition and rail cleanup both
fail, power control takes precedence in `err_curag_t`, operation and the single
component slot. Blocked group slots preserve the occurrence, but not
necessarily the exact earlier shared backend status. Both public operations use
the same private gate-off primitive.

### `node_sensors`: automated cases

- **All groups acquired:** with the complete fixture, one call sets all five
  component validity bits and returns values from the expected physical
  channels. The corresponding node reading sets all seven sensor protocol bits.
- **Atomic enclosure group:** every successful BME280 acquisition validates
  temperature, pressure and humidity together. It is impossible for a
  node-generated sample to set only a subset of the three enclosure protocol
  bits.
- **Repeated switched-rail acquisition:** perform at least 100 consecutive
  calls. Both DS18B20 identities remain stable, no previous conversion is
  mistaken for the current one and every call completes within the configured
  sensor timing bound.
- **BME280 returns to low power (deferred):** after the BME280 driver path is
  chosen and hardened, inspect its mode bits after sampling and verify that it
  is back in sleep mode without changing the already returned enclosure values.
- **Final cleanup is idempotent:** call `node_sensors_force_power_off` more than
  once after successful sampling. Every call succeeds without initializing a
  sensor bus, enabling the rail or changing the collected sample.
- **Missing DS18B20 channel 0:** with only its configured ROM identity absent,
  channel 0 is zero and invalid, its diagnostic pair reports the driver result,
  and channel 1 plus independent groups remain usable.
- **Missing DS18B20 channel 1:** verify the symmetric channel-1 behavior.
- **Missing BME280:** all three enclosure values are zero, the single enclosure
  group is invalid, its diagnostic pair reports the failure and gated groups
  remain usable.
- **Diagnostic slot mapping:** for each detectable hardware failure above,
  unaffected pairs are `(NONE, 0)` and the failing fixed pair contains the
  exact backend kind and status. Host tests remain responsible for exhaustive
  simultaneous failures and power-gate fault injection.

Disconnecting an analog soil probe normally still leaves a readable ADC input
and is not necessarily a detectable acquisition error. Soil hardware tests
therefore validate voltage conversion and channel mapping rather than requiring
an unplugged probe to clear a validity bit.

### `node_sensors`: manual electrical cases

These tests use the available multimeter and firmware stages that hold the CPU
awake after the operation so the resulting electrical state can be measured:

- **Soil ADC conversion:** apply or measure safe known voltages at both soil
  inputs and compare `soil_0_mv` and `soil_1_mv` against the multimeter within
  the configured ESP32 ADC calibration tolerance.
- **Soil channel mapping:** change one input voltage at a time and verify that
  only the expected logical channel follows it.
- **DS18B20 identity mapping:** create a clear temperature difference between
  the probes and verify configured ROM identity, rather than 1-Wire enumeration
  order, determines channel 0 and channel 1. Confirm the textual ROM byte order
  against the library's enumerated `uint64_t`, then repeat after reconnecting or
  reversing their physical order on the bus.
- **Immediate successful shutdown:** after `node_sensors_sample_all` returns
  while the ESP32 remains awake, the shared rail measures off. This proves that
  cleanup occurs inside sampling rather than being deferred until final wake
  cleanup.
- **Active-low gate states:** while sampling is deliberately paused, verify the
  GPIO/P-MOSFET gate is low and the switched rail is on. After cleanup, verify
  the external 47 kOhm source-to-gate resistor has pulled the released gate to
  3.3 V and the switched rail is off.
- **Shutdown after acquisition failure:** repeat with a detectable DS18B20 or
  BME280 failure and verify that the shared rail still measures off.
- **Final cleanup:** after repeated `node_sensors_force_power_off` calls, the
  shared rail remains off.
- **Reset and deep-sleep default:** enable the rail in a dedicated test stage,
  then reset or enter deep sleep and verify that the board returns the rail to
  off when the MCU no longer actively enables it. Repeat while holding the MCU
  in reset to exercise the external 47 kOhm default rather than a firmware
  shutdown path.
- **Back-power check:** in the real post-sampling and deep-sleep pin states,
  measure the disabled rail for voltage fed through ADC, 1-Wire, pull-up or
  protection-diode paths. A low gate-control GPIO alone is not sufficient
  evidence that the sensors are unpowered.

The devkit and multimeter cannot establish the final microamp sleep budget or
capture short rail transients. Those require current instrumentation on the
custom board; these pilot tests establish functional switching, stable channel
identity and absence of obvious steady-state back-power.

### Platform ports

The platform ports receive a small on-device smoke suite. These tests exercise
the concrete ESP-IDF adapters; they do not add statistical randomness tests or
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

All tests in this radio hardware section are deliberately deferred. No radio
hardware test application, receiver-peer harness or RF exchange is part of the
current implementation.

SX1262 hardware tests exercise behavior that the fake backend cannot prove:
real SPI/BUSY/DIO1 operation, RF interoperability, IRQ timestamps,
direction-specific IQ, physical state transitions and bounded deadlines. They
do not repeat every invalid argument, backend error or exact deadline boundary
already covered on the host.

The node DUT runs the production `sx1262_radio` component. The peer is the
actual receiver hardware running separate test code that can:

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

Timing assertions are asymmetric. For calculated airtime `T`:

```text
T <= tx_done_at_us - set_tx_at_us <= ceil(T * 1.10)
```

The current 50-byte profile therefore permits 97.536 through 107.290 ms. For a
receive call made at `S` with absolute deadline `D`, normal silence must return
within:

```text
D <= returned_at_us <= D + ceil((D - S) * 0.15)
```

Hardware packets are scheduled with generous separation from the deadline;
the exact equality boundary remains a deterministic host test.

### `sx1262_radio`: fast automated cases

- **First transmit initializes and sends exact bytes:** reboot the node,
  transmit a known 50-byte pattern and verify the peer receives those exact
  bytes with normal IQ. Require success, `tx_started`, `tx_done`, ordered
  nonzero timestamps and an empty diagnostic.
- **Payload-length boundaries:** transmit distinctive 1-, 50- and 255-byte
  payloads and verify exact length and contents at the peer. Exhaustive invalid
  input lengths remain host tests.
- **TX airtime matches the pilot profile:** for the 50-byte frame, require the
  measured pre-`SetTx` to `TX_DONE` duration to satisfy the 10% asymmetric
  airtime bound above. This detects major PHY-profile misconfiguration without
  claiming laboratory-grade RF timing.
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

These fault cases are manual for the pilot; no automated DIO1 switching or
radio-disconnect fixture is planned yet. Broken-SPI, forced-BUSY and unexpected
IRQ injection remain deferred until a safe controllable fixture exists.

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
- Short `node_core` and radio timing values remain to be chosen when those
  hardware tests are implemented.
- The exact placement of the RTC-consumption restart hook remains an
  implementation decision; its permitted scope and production exclusion are
  fixed above.
- Hardware-runner provisioning, including permanent serial-port assignment and
  whether slow tests run automatically, remains to be defined when a dedicated
  runner exists.
- The SX1262 host fake is private, and fresh CTest processes reset the hidden
  singleton; the production radio interface intentionally exposes no test-only
  setter, getter, reset or snapshot operation.
- Receiver-peer command transport and scheduling details remain an
  implementation choice; the peer hardware, separate-code requirement and
  observable behaviors are fixed above.

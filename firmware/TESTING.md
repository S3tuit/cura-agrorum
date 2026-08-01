# Firmware testing

Status: agreed host-test plan for the LoRa v2 node firmware. Exact persistence
record encodings and their encoding-specific tests remain deferred.

## Philosophy and build

Host tests use the production `node_core` with fake persistence, sensor, radio,
clock, randomness and system ports. Component tests use the real component with
a private fake backend only where meaningful policy exists. Assertions cover
results, state and relevant ordering constraints. Only a few orchestration
tests assert a complete operation sequence so harmless refactoring does not
break unrelated tests.

Native firmware tests are separate CMake targets compiled with strict warnings,
ASan and UBSan. CTest runs the executables. A top-level `make test-host` target
will configure, build and invoke CTest. The existing pytest suite remains
responsible for protocol generation, shared Python/C vectors, Hypothesis and
future hardware orchestration.

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
- Each individual sensor failure zeroes only its corresponding field and clears
  only its validity bit.
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
- Failure before `SetTx` starts neither increments attempts nor charges
  airtime.
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
- Backlog quarantine failure logs and stops to prevent immediate reselection.
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
  retry a backend whose initialization was cached as failed.
- Empty backlog is distinguishable from a storage error.
- Diagnostic failure is discarded without recursive logging.
- Successful delivery start and finish events are durable before returning.

After choosing the record encoding, tests also cover:

- Round-trip of every record type and boundary value.
- Truncated final records at every meaningful boundary, trailing bytes, and
  unsupported record types or versions.
- Newest-first backlog reconstruction.
- Accepted and quarantined tombstones, duplicate records and duplicate
  tombstones.
- Matched and unmatched delivery events, including the same packet retried in
  different cycle IDs.
- Recovery after power loss between each append and synchronization step.

A torn application append is tested even though general filesystem corruption
is delegated to LittleFS; a torn record does not necessarily imply filesystem
corruption.

## `sx1262_radio`

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
- Successful `SetTx` and IRQ outcomes are reported accurately.
- IRQ-read failure after `SetTx` reports `tx_started = true` and
  `tx_done = false`; failure before successful `SetTx` reports
  `tx_started = false`.
- Sleep before initialization is a no-op and does not initialize the radio.
- Sleep after initialization stops active RX with `SetStandby(STDBY_RC)` and
  then issues `SetSleep(COLD_START)`.
- A standby or sleep-command failure is returned to the controller and does not
  prevent system deep sleep.

## `node_sensors`

A small private fake hardware seam tests sequencing and guarantees that the
shared soil/DS18B20 power gate is disabled after every success and failure
point. It need not model ADC, I2C or 1-Wire electrically; those behaviors need
on-hardware tests.

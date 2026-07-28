# Firmware architecture

Status: pilot design for the LoRa v2 node firmware. The wire contract remains
defined by `protocol/protocol-v2-lora/README.md`.

## Structure

The firmware is split into:

- a platform-independent wake-cycle controller that owns application policy;
- small pure helpers for reading construction, delivery decisions, budgets and
  metrics;
- injected interfaces for sensors, radio, storage, time, randomness,
  diagnostics and sleep; and
- ESP-IDF adapters that implement those interfaces.

`app_main` constructs the ESP32 implementations and invokes one wake cycle. It
does not contain the wake policy. Interfaces are groups of typed function
pointers with implementation context pointers, not unrelated callbacks passed
to every function. Protocol encoding and cryptography are deterministic
components and are called directly rather than injected.

The controller must not depend directly on GPIO, NVS, LittleFS, the SX1262 or
ESP-IDF sleep APIs. It uses fixed-capacity data where practical and does not
require heap allocation.

An expected initial layout is:

```text
firmware/
  components/
    protocol_v2_lora/   wire codec and authenticated frame layer
    soil_sensor/        soil ADC implementation
    node_core/          wake controller and platform-independent policy
    node_storage/       NVS, LittleFS, RTC state and diagnostic adapters
    node_sensors/       sensor sequencing and power control
    sx1262_radio/       SX1262 adapter
  main/
    app_main.c          composition root
    board_config.h      board-specific pins and options
```

The exact components and C types are defined after the algorithms and
persistence boundaries are stable.

## Persistence boundaries

- The generated node-identity header contains `node_id` and `node_key`.
- NVS contains the next monotonic `sample_id`.
- RTC memory carries metrics from one completed wake to the immediately
  following deep-sleep wake.
- LittleFS contains pending readings, quarantined readings and append-only
  diagnostic logs.
- RAM contains the active delivery state and exact encrypted frame used by all
  attempts for one `(node_id, sample_id, domain)`.

The current reading is persisted before its first transmission. If the node
resets afterward, the reading remains available as backlog. A LittleFS failure
puts the wake into a degraded RAM-only mode: the current reading may still be
transmitted, but it may be lost after sleep if it is not accepted.

No storage is automatically erased or reformatted after a failure during the
pilot.

## RTC-state lifecycle

Incoming and outgoing metrics are separate:

- incoming metrics describe the preceding completed wake and are copied into
  the current reading; and
- outgoing metrics are accumulated during the current wake and become incoming
  metrics only on the next valid deep-sleep wake.

At application start, the controller copies the RTC record into RAM and
immediately invalidates the RTC-resident copy. After claiming the current
`sample_id`, the copied record is accepted only when:

- its magic, format version and CRC are valid;
- `reset_reason` is `ESP_RST_DEEPSLEEP`; and
- its completed `sample_id` immediately precedes the current `sample_id`.

Otherwise all previous-cycle wire fields are zero and both previous-cycle
flags are clear.

The early invalidation prevents a partially executed wake from deep-sleeping
and causing an older record to be reported as the immediately preceding wake.
At normal finalization, logs are flushed first, total awake time is measured,
and a new RTC record is written. Its validity marker is written last.

Metric accumulators use wider internal types. They are serialized to the
protocol's `u8` and `u16` fields only if all values are representable. Overflow
is logged and produces an invalid outgoing RTC metrics record instead of
wrapping or clamping.

## Wake cycle

### Start and persistence

1. Capture `reset_reason` and the monotonic application-start time.
2. Copy and invalidate the RTC record.
3. Initialize diagnostics, NVS and LittleFS independently.
4. Claim and commit a new `sample_id` in NVS.
5. If claiming the ID fails, write a best-effort diagnostic, do not activate
   sensors or radio, leave outgoing RTC state invalid, and enter deep sleep.
6. Validate the copied RTC record now that the current ID is known.

NVS is initialized before sensor activation because a reading must never be
created or transmitted with an uncommitted ID. Gaps caused by a reset after
claiming an ID are allowed; reuse is not.

### Sampling and reading construction

1. The sensor adapter powers the gated sensors, collects all channels and
   guarantees power-off on both success and failure.
2. Failure of one sensor sets its value to zero and clears only its validity
   bit. Other sensors continue.
3. The controller combines the sensor snapshot, reset reason and incoming RTC
   metrics into the 28-byte reading body.
4. `run_ms` is captured when the body is finalized, immediately before
   persistence and frame construction.
5. The exact body is appended to pending-reading storage. If this fails, a
   best-effort diagnostic is written and delivery continues from the RAM copy.
6. The SX1262 is initialized only after the reading exists.

### Delivery operation

One delivery operation owns the retry loop for either a current or backlog
reading. It:

- constructs one authenticated frame and reuses its exact 50 bytes;
- records the time immediately before the first `SetTx`;
- charges estimated airtime and increments attempt metrics when `SetTx` is
  successfully started;
- after `TX_DONE`, listens continuously and calculates
  `retry_at = TX_DONE + 500 ms + uniform_random(100 ms, 500 ms)`;
- logs and ignores invalid ACKs without closing the RX window;
- retransmits at `retry_at` while another complete attempt fits both the
  eight-second charged-TX budget and 30-second radio-cycle deadline; and
- returns a terminal ACK, exhausted-limit or local-error result.

When a started transmission does not produce `TX_DONE`, its estimated airtime
remains charged and the delivery ends as a local radio error. Attempts that
fail before `SetTx` starts are not counted or charged.

The delivery result is one of:

```text
ACCEPTED
RETRY_LATER
REJECTED_UNSUPPORTED
REJECTED_MALFORMED
NO_ACK_RADIO_DEADLINE
NO_ACK_AIRTIME_LIMIT
LOCAL_RADIO_ERROR
```

### Current reading

The current reading is always delivered first with
`CURRENT_READING_UPLINK`.

- `ACCEPTED`: mark the outgoing current as accepted, store its delivery time
  and increment the distinct accepted-reading count. If the reading was
  persisted, remove its pending copy; a removal failure is logged and stops
  radio work so the same entry cannot be selected again. If no removal is
  needed, or it succeeds, begin backlog drainage when storage is available.
- `RETRY_LATER`: retain the reading and stop all radio work for the wake.
- `REJECTED_UNSUPPORTED` or `REJECTED_MALFORMED`: quarantine the reading and
  stop all radio work. A rejection of the current firmware's frame may be
  systematic, so backlog is not attempted.
- Silence: retransmit the exact frame while both limits permit. When no further
  attempt fits, retain the reading and stop radio work.
- Local codec, cryptographic or radio error: log it, retain the reading when it
  was persisted, and stop radio work.

### Backlog drainage

Backlog drainage selects the most recent pending reading and transmits its
unchanged body with `BACKLOG_READING_UPLINK`.

- `ACCEPTED`: increment the distinct accepted-reading count, remove the entry
  and continue with the next most recent entry.
- `RETRY_LATER`: retain the entry and stop all radio work.
- `REJECTED_UNSUPPORTED` or `REJECTED_MALFORMED`: quarantine that entry and
  continue with the next one while both budgets permit.
- Silence: retransmit the exact frame while both limits permit. When no further
  attempt fits, retain the entry and stop drainage for the wake.
- Local error: log it, retain the entry and stop drainage.

If removal or quarantine fails, the controller logs the failure and stops
drainage. It must not immediately select the same entry again.

### Finalization

1. Put the radio into standby.
2. Finalize delivery episodes and append diagnostic records.
3. Flush best-effort persistent state.
4. Measure total awake time.
5. Validate and commit outgoing RTC metrics.
6. Enter deep sleep.

Final persistence is outside the 30-second radio-cycle deadline. The awake-time
measurement includes final logging but excludes only the small RTC commit and
sleep-call overhead.

## Metric accounting

All metric updates occur in the shared delivery operation:

- current-reading and whole-cycle attempt counts increment for every current
  `SetTx` that starts;
- the whole-cycle attempt count also increments for every backlog `SetTx` that
  starts;
- the accepted-reading count increments once for each distinct authenticated
  `ACCEPTED`;
- current delivery time spans immediately before the first current `SetTx` to
  its authenticated `ACCEPTED`; and
- `PREVIOUS_CURRENT_ACCEPTED` is false and its delivery time is zero unless the
  current reading was accepted.

Outgoing metrics stay in RAM during the wake and are committed to RTC only
during finalization. They never modify the already finalized or transmitted
current reading.

## Pilot failure policy

Unexpected failures are not automatically repaired during the pilot. They get
a deterministic safe transition and a best-effort diagnostic:

| Failure | Action |
|---|---|
| NVS initialization or `sample_id` claim | Log, skip sampling and radio, sleep |
| Invalid incoming RTC state | Ignore it and continue with previous metrics invalid |
| Individual sensor failure | Zero that field, clear its validity bit and continue |
| LittleFS initialization or write | Continue current delivery from RAM without unavailable persistence features |
| Codec or cryptographic failure | Log and end the radio phase |
| Radio initialization or local TX/RX failure | Log, retain persisted data and end the radio phase |
| Invalid or unauthenticated ACK | Log it and continue waiting |
| Pending-reading removal or quarantine failure | Log and stop backlog drainage |

The diagnostic sink is append-only and best-effort. Records identify the
operation, error code, application-time offset and `sample_id` when available,
but never contain node or group keys. A logging failure is discarded and is
never recursively logged.

Every early-exit path powers off sensors and leaves the radio in standby when
those devices were initialized.

## Core invariants

- No reading is transmitted before its `sample_id` increment is committed.
- A `sample_id` may be skipped but never reused during one identity lifetime.
- All attempts for one `(node_id, sample_id, domain)` use identical frame
  bytes.
- An invalid ACK never changes delivery or persistence state.
- A pending reading is removed only after authenticated `ACCEPTED`.
- `RETRY_LATER` stops all transmissions for the wake.
- Current delivery always precedes backlog drainage.
- The charged-TX budget and radio-cycle deadline are independent limits.
- Final persistence may exceed the radio deadline but completes before sleep.
- No protocol key is written to diagnostics.

Host tests inject fake ports into the controller; ESP32 tests exercise the real
PSA, NVS, LittleFS, RTC, sensor and radio adapters. Tests are added with each
implemented vertical slice rather than after the complete rewrite.

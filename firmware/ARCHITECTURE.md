# Firmware architecture

Status: pilot design for the LoRa v2 node firmware. The wire contract remains
defined by `protocol/protocol-v2-lora/README.md`.

## Wake cycle

### Start and persistence

1. Capture `reset_reason` and the monotonic application-start time.
2. Copy and invalidate the RTC record.
3. Claim and commit a new `sample_id` in NVS. This first storage operation
   lazily initializes NVS.
4. If claiming the ID fails, write a best-effort diagnostic, do not activate
   sensors or radio, leave outgoing RTC state invalid, and enter deep sleep.
   Writing the diagnostic independently attempts lazy LittleFS initialization.
5. Validate the copied RTC record now that the current ID is known.

NVS is first used before sensor activation because a reading must never be
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
6. The first radio operation lazily initializes the SX1262, so it is not
   initialized before a reading exists.

### Delivery operation

One delivery operation owns the retry loop for either a current or backlog
reading. It:

- constructs one authenticated frame and reuses its exact 50 bytes;
- appends one durable `DELIVERY_STARTED` event before its first call to
  `transmit_uplink`;
- records the time immediately before the first `SetTx`;
- charges estimated airtime and increments attempt metrics when `SetTx` is
  successfully started;
- after `TX_DONE`, listens continuously and calculates
  `retry_at = TX_DONE + 500 ms + uniform_random(100 ms, 500 ms)`;
- logs and ignores invalid ACKs without closing the RX window;
- retransmits at `retry_at` while another complete attempt fits both the
  eight-second charged-TX budget and 30-second radio-cycle deadline; and
- returns a terminal ACK, exhausted-limit or local-error result.

When the operation reaches that result, it appends one durable
`DELIVERY_FINISHED` event. The events bracket the whole delivery operation,
including all retries; they do not bracket each transmission attempt. They are
written by `node_core`, not by the radio component. `DELIVERY_STARTED` means
that the controller entered delivery, not that the SX1262 certainly executed
`SetTx`.

Each pair is identified by the wake's current `cycle_sample_id`, the delivered
packet's `sample_id` and its domain. This distinguishes separate wakes that
retry the same backlog packet without adding another persistent counter. An
unmatched start means only that no durable finish was recorded: reset may have
occurred before TX, during TX or RX, during retry waiting, or while persisting
the finish. Failure to append either event never blocks radio work.

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

1. Put an initialized radio into cold-start sleep.
2. Append remaining structured diagnostic records.
3. Call the persistence component's single `sync_all` operation.
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

Every early-exit path powers off sensors and calls `radio.sleep` exactly once.
That call is a no-op when the radio was never initialized; otherwise it leaves
the SX1262 in cold-start sleep. A radio-sleep failure is logged when possible
but never prevents the ESP32 from entering deep sleep.

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
- A delivery event failure never changes the delivery result or radio policy.

Host tests inject fake persistence, sensor, radio, clock, randomness and system
ports into the controller and pass an ordinary RTC record. ESP32 tests exercise
the real PSA, NVS, LittleFS, RTC memory, sensor and radio implementations. Tests
are added with each implemented vertical slice rather than after the complete
rewrite. The agreed host-test catalogue and build strategy are in
`firmware/TESTING.md`.

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

- its combined commit, magic and format marker is
  `NODE_RTC_COMMITTED_V1`;
- `reset_reason` is `ESP_RST_DEEPSLEEP`; and
- its completed `sample_id` immediately precedes the current `sample_id`; and
- its metrics satisfy their semantic and representability invariants.

Otherwise all previous-cycle wire fields are zero and both previous-cycle
flags are clear.

The early invalidation prevents a partially executed wake from deep-sleeping
and causing an older record to be reported as the immediately preceding wake.
At normal finalization, logs are flushed first, total awake time is measured,
and a new RTC record is written. Its marker is set to
`NODE_RTC_COMMITTED_V1` last, after enforcing the required compiler and memory
store ordering. No RTC CRC is used during the pilot.

Metric accumulators use wider internal types. They are serialized to the
protocol's `u8` and `u16` fields only if all values are representable. Overflow
is logged and produces an invalid outgoing RTC metrics record instead of
wrapping or clamping.

## Components and responsibilities

### `node_core`

`node_core` is platform-independent and contains:

- the wake-cycle controller, which owns operation ordering and failure policy;
- the shared current/backlog delivery operation and retry state;
- reading construction and structural validation;
- airtime and wall-clock budget accounting;
- current-wake metric accumulation;
- pure RTC-record consume, validation, invalidation and commit helpers; and
- small pure helpers rather than one monolithic controller function.

The controller calls the generated protocol codec and AES-CCM frame component
directly. Those deterministic components are not injected. The controller does
not call GPIO, NVS, LittleFS, SX1262 or ESP-IDF sleep functions directly.

### `node_persistence`

`node_persistence` is the single owner of NVS, LittleFS, pending readings,
quarantined readings and diagnostic logs. It exposes semantic operations:

```text
claim_sample_id
append_pending_reading
peek_most_recent_pending
mark_reading_accepted
quarantine_reading
append_diagnostic_event
append_delivery_event
sync_all
```

`append_diagnostic_event` receives a structured warning or error record.
It is the controller's general best-effort interface for persistent warnings
and errors. `append_delivery_event` receives either `DELIVERY_STARTED` or
`DELIVERY_FINISHED` and implements the delivery-log semantics defined in the
protocol document. There is no separate diagnostic component and no raw append
operation exposed to the wake controller.

A start is appended once before the first `transmit_uplink` call. Its matching
finish is appended once after a terminal ACK, exhausted limit or local error
and contains the episode's attempt information and result. Both event types
are durable when their operation returns successfully. Ordinary diagnostics
may remain buffered until `sync_all`. Logging failures are never recursively
logged and never prevent transmission or deep sleep.

NVS and LittleFS have independent lazy-initialization states:

```text
UNINITIALIZED -> READY
UNINITIALIZED -> FAILED
```

The first operation needing a backend initializes it. A failure is cached for
the remainder of the wake and returned by later operations; ordinary RAM is
reinitialized at the next boot, so initialization is attempted again then.
Failure results distinguish initialization from the requested operation.

Operations affecting delivery correctness are durable when they return:
claiming an ID includes the NVS commit, and pending, accepted and quarantine
transitions commit their LittleFS state. Delivery events are also durable on
successful return because an unmatched start must survive a reset.
`sync_all` performs the one final synchronization for remaining buffered state,
including ordinary diagnostics. It skips backends that were never initialized
and does not initialize them merely to finalize a wake.

The LittleFS record representation, recovery framing and compaction policy are
internal and remain to be designed. Private helper functions may share
mounting, appending and synchronization code without becoming injected
architectural layers.

### `node_sensors`

`node_sensors` owns sensor sequencing and exposes:

```text
sample_all
force_power_off
```

`sample_all` lazily initializes required buses, enables the shared gated sensor
rail, samples both soil channels, both DS18B20 channels and the BME280, and
disables the rail through a common cleanup path. It returns individual values,
validity indicators and failure details so one failed sensor does not suppress
the others. `force_power_off` is idempotent and does not initialize anything.

One power gate supplies both soil sensors and both DS18B20 sensors. The BME280
remains on the always-powered rail and uses its low-power operating mode. The
board must default the switched sensor rail to off whenever the MCU does not
actively enable it. The circuit used to enforce that default is intentionally
not yet specified.

### `sx1262_radio`

The radio component owns SX1262 commands, IRQ handling and all pilot PHY
configuration. It exposes protocol-oriented operations:

```text
transmit_uplink
receive_downlink_until
sleep
```

The first transmit lazily initializes the radio. Uplink transmission uses
normal IQ and downlink reception uses inverted IQ as defined by the protocol.
The transmit result reports whether `SetTx` started and the actual transmit
start and `TX_DONE` monotonic times, so initialization latency is not counted
as delivery time. Reception returns packet bytes, length and radio metadata, or
a timeout or local error.

Within an active wake the radio uses `STDBY_RC` as the intermediate state for
configuration and transitions between TX and RX. It is not put to sleep while
delivery or retries remain possible. Final `sleep` behaves as follows:

1. If the radio was never initialized, return successfully without issuing a
   radio command or initializing it.
2. Stop continuous RX or any other active mode with `SetStandby(STDBY_RC)`.
3. Issue `SetSleep(COLD_START)`; the next wake performs complete lazy
   initialization rather than trusting retained radio configuration.
4. Return any failure for best-effort diagnostics, but do not prevent ESP32
   deep sleep.

The controller owns retries, deadlines, ACK validation and delivery policy; the
radio component only executes radio operations and reports their outcomes.

### Existing protocol and sensor components

`protocol_v2_lora` remains the generated wire codec and authenticated frame
layer. `soil_sensor` remains the low-level calibrated ADC reader used by
`node_sensors`. External DS18B20 and BME280 drivers remain hardware adapters;
they do not own wake policy.

### Platform services

Small injected ports provide:

```text
clock:
    monotonic_us

randomness:
    uniform_u32_inclusive

system:
    get_reset_reason
    enter_deep_sleep_for
```

Only monotonic time is needed; the node does not maintain UTC. The clock drives
`run_ms`, radio deadlines, retry scheduling, delivery and awake durations and
diagnostic offsets. A fake clock allows host tests to advance without real
waiting. Retry randomness does not need cryptographic guarantees.

The system implementation performs the final board-safe transition and enters
deep sleep. `enter_deep_sleep_for` does not return during normal operation.

### `app_main` and RTC memory

`app_main` is the composition root. It owns the concrete component state,
constructs the injected port table and invokes one wake cycle. It contains no
wake policy.

RTC memory is direct retained data rather than an injected interface:

```c
RTC_DATA_ATTR static node_rtc_record_t rtc_record;

void app_main(void) {
  node_cycle_run(&platform, &rtc_record);
}
```

The pilot record contains:

```text
commit_marker
completed_sample_id
cycle_metrics
```

`commit_marker` is zero while invalid and
`NODE_RTC_COMMITTED_V1` when committed. That one constant combines validity,
magic and record-format version. At wake start, `node_core` copies the record
into ordinary RAM and immediately invalidates the retained marker. At normal
finalization it writes the completed ID and metrics, then commits the marker
last.

Host tests pass an ordinary `node_rtc_record_t` variable to the same controller
and RTC helper functions; only the production declaration uses
`RTC_DATA_ATTR`.

Interfaces are typed function tables with implementation context pointers.
Fixed-capacity data is used where practical, and `node_core` does not require
heap allocation.

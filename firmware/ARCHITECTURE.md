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
3. The controller combines `sample_id`, the sensor snapshot, reset reason and
   incoming RTC metrics into the 32-byte reading body.
4. `run_ms` is captured when the body is finalized, immediately before
   persistence and frame construction.
5. The exact body is appended to pending-reading storage. If this fails, a
   best-effort diagnostic is written and delivery continues from the RAM copy.
   Backlog storage is not accessed again during that wake because append
   durability may be uncertain.
6. Claim and commit a new transport `message_id` for the current logical
   uplink. If that fails, leave any pending reading intact, log best-effort and
   perform no radio work. A successful claim may skip but is never reused.
7. Seal one 54-byte current-domain frame. The first radio operation then lazily
   initializes the SX1262, so it is not initialized before a complete logical
   message exists.

### Delivery operation

One delivery operation owns the retry loop for either a current or backlog
reading. It:

- receives one already constructed authenticated frame and reuses its exact 54
  bytes;
- appends one durable `DELIVERY_STARTED` event before its first call to
  `transmit_uplink`;
- records the time immediately before the first `SetTx`;
- charges the radio's modeled airtime plus a controller-owned 10% allowance
  and increments attempt metrics when `SetTx` is confirmed or its effect
  becomes uncertain after crossing SPI;
- after `TX_DONE`, keeps single-shot RX armed and calculates
  `retry_at = TX_DONE + 500 ms + uniform_random(100 ms, 500 ms)`;
- logs and ignores invalid ACKs without closing the RX window;
- retransmits at `retry_at` while another complete attempt's 10%-padded
  airtime charge and radio-reported minimum TX window fit the independent
  eight-second charged-TX budget and 30-second radio-cycle deadline; and
- returns a terminal ACK, exhausted-limit or local-error result.

When the operation reaches that result, it appends one durable
`DELIVERY_FINISHED` event. The events bracket the whole delivery operation,
including all retries; they do not bracket each transmission attempt. They are
written by `node_core`, not by the radio component. `DELIVERY_STARTED` means
that the controller entered delivery, not that the SX1262 certainly executed
`SetTx`.

Each pair records the wake's current `cycle_sample_id`, the reading's
`sample_id`, the logical transport `message_id` and its domain. Transport
identity is `(node_id, message_id)`; reading identity remains
`(node_id, sample_id)`. An unmatched start means only that no durable finish
was recorded: reset may have occurred before TX, during TX or RX, during retry
waiting, or while persisting the finish. Failure to append either event never
blocks radio work.

When a started or uncertain transmission does not produce `TX_DONE`, its
estimated airtime remains charged and the delivery ends as a local radio error.
Attempts that definitively fail before `SetTx` can take effect are not counted
or charged.

The two limits intentionally use different radio timing values. The charged-TX
limit counts only modeled RF airtime plus 10%; watchdog, command and setup time
are not RF airtime. Wall-clock admission uses
`sx1262_radio_min_tx_window_us(54)`, which includes modeled airtime, TX ramp,
watchdog margin and quantization, and a conservative pre-`SetTx` setup
allowance. The radio checks the remaining watchdog-capable window again after
setup and immediately before `SetTx`, so an unexpectedly slow preparation can
fail without starting or charging a doomed attempt.

#### Continuous-hour duty-cycle accounting

For the 868.1 MHz pilot profile, when relying on the EU 1% duty-cycle
alternative, transmitter on-time is limited to 36 seconds in every continuous
one-hour observation period. This is a rolling window, not a wall-clock-hour
allowance: for example, 30 seconds transmitted at 03:58 leaves only six seconds
available at 04:00. The earlier airtime becomes available again only as it ages
out of the continuous one-hour window.

The implemented eight-second charged-TX budget is a per-wake retry and energy
guard, not an hour-spanning regulatory ledger. It starts from zero in each
`node_cycle_run` invocation and is absent from retained RTC metrics. Under the
normal lifecycle, which enters 15 minutes of deep sleep only after completing a
wake, at most four such budgets occur in a continuous hour, limiting
conservatively charged airtime to at most 32 seconds. A restart, brownout or
other reset can create another fresh per-wake budget without first completing
that sleep. The current pilot therefore depends on uninterrupted normal cadence
for its intended hourly margin and does not independently enforce the 1% limit
across resets.

Before production compliance is claimed, add a durable per-band rolling ledger
with these fail-conservative properties:

- retain charged transmissions until they age out of the continuous one-hour
  window rather than resetting at a clock-hour boundary;
- use an elapsed-time basis that survives deep sleep, and do not grant elapsed
  time after a reset unless it can be established conservatively;
- durably reserve the padded airtime before `SetTx`, retain that reservation
  when command effect is uncertain or reset interrupts the result, and reclaim
  it only after a definite pre-`SetTx` failure; and
- treat unavailable or invalid regulatory state as exhausted until sufficient
  verified time has elapsed. RTC retention may cache the ledger, but durable
  storage must remain authoritative across brownouts and cold restarts.

The eight-second per-wake budget remains as an independent operational limit
after this ledger is introduced.

The delivery result and its `DELIVERY_FINISHED.final_result` encoding are:

| Value | Result | Meaning |
|---:|---|---|
| `0` | `INVALID` | Reserved uninitialized value; never written as a completed result |
| `1` | `ACCEPTED` | Authenticated `ACCEPTED` ACK |
| `2` | `RETRY_LATER` | Authenticated `RETRY_LATER` ACK |
| `3` | `UNSUPPORTED` | Authenticated `REJECTED_UNSUPPORTED` ACK |
| `4` | `MALFORMED` | Authenticated `REJECTED_MALFORMED` ACK |
| `5` | `AIRTIME_BUDGET_END` | No terminal ACK before another attempt ceased to fit the charged-TX budget |
| `6` | `RADIO_CYCLE_DEADLINE` | No terminal ACK before another attempt ceased to fit the 30-second radio-cycle deadline |
| `7` | `LOCAL_RADIO_ERROR` | Local initialization, TX, IRQ or RX operation failed |

`RADIO_CYCLE_DEADLINE` does not mean the complete wake ended: final logging,
synchronization, RTC commit and the sleep call occur afterward.

### Current reading

The current reading is always delivered first with
`CURRENT_READING_UPLINK`.

Its committed `message_id` and exact frame live in RAM for the wake. Every RF
retry in that delivery episode reuses the same ID, domain and frame bytes. A
reset abandons that current logical message; the persisted application reading
later receives a new ID and backlog domain when it is converted to backlog.

- `ACCEPTED`: mark the outgoing current as accepted, store its delivery time
  and increment the distinct accepted-reading count. If the reading was
  persisted, remove its pending copy; a removal failure is logged and stops
  radio work so the same entry cannot be selected again. If no removal is
  needed, or it succeeds, begin backlog drainage when storage is available.
- `RETRY_LATER`: retain the reading and stop all radio work for the wake.
- `REJECTED_UNSUPPORTED` or `REJECTED_MALFORMED`: quarantine the reading and
  then, when its pending append succeeded, attempt to remove its pending copy
  even if quarantine failed. Stop all radio work afterward. A rejection of the
  current firmware's frame may be systematic, so backlog is not attempted.
- Silence: retransmit the exact frame while both limits permit. When no further
  attempt fits, retain the reading and stop radio work.
- Local codec, cryptographic or radio error: log it, retain the reading when it
  was persisted, and stop radio work.

### Backlog drainage

Backlog drainage selects the most recent pending reading. If it is unbound, the
controller commits a new `message_id`, seals one backlog-domain frame and
durably appends a binding containing the ID and exact frame before the first
TX. Failure before the binding becomes durable transmits nothing and leaves the
reading unbound; the claimed ID may be skipped. Once bound, all retries in the
same or later wakes load and transmit the persisted frame bytes verbatim.

- `ACCEPTED`: increment the distinct accepted-reading count, remove the entry
  and continue with the next most recent entry.
- `RETRY_LATER`: retain the entry and stop all radio work.
- `REJECTED_UNSUPPORTED` or `REJECTED_MALFORMED`: quarantine that entry, then
  attempt to remove its pending copy even if quarantine failed. Continue with
  the next one only when removal succeeds and both budgets permit.
- Silence: retransmit the exact frame while both limits permit. When no further
  attempt fits, retain the entry and stop drainage for the wake.
- Local error: log it, retain the entry and stop drainage.

If removal after acceptance or quarantine fails, the controller logs the
failure and stops drainage. If the quarantine copy is lost but the separate
pending removal succeeds, the controller logs that loss and may continue; it
cannot immediately reselect the same entry.

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
| `message_id` claim/commit failure or exhaustion | Log, construct no message, retain any reading, skip radio, sleep; loss or exhaustion requires a new identity/key |
| Durable backlog-frame binding failure | Log, transmit no backlog frame, retain the unbound reading and stop drainage |
| Invalid incoming RTC state | Ignore it and continue with previous metrics invalid |
| Individual sensor failure | Zero that field, clear its validity bit and continue |
| LittleFS initialization or write | Continue current delivery from RAM without unavailable persistence features |
| Codec or cryptographic failure | Log and end the radio phase |
| Radio initialization or local TX/RX failure | Log, retain persisted data and end the radio phase |
| Invalid or unauthenticated ACK | Log it and continue waiting |
| Pending-reading removal failure | Log and stop backlog drainage |
| Lost quarantine copy after successful pending removal | Log and continue; do not obstruct backlog progress |

The diagnostic sink is append-only and best-effort. Records identify the
operation, error code, application-time offset, wake `sample_id` and active
transport `message_id` when available, but never contain node or group keys. A
logging failure is discarded and is never recursively logged.

### Stable diagnostic model

Diagnostics separate four responsibilities:

- `error_domain` identifies the subsystem responsible for the failure;
- `operation` describes the stable action being attempted, not a C function or
  private helper name;
- `error_code` gives the domain-specific reason for failure;
- `context_schema` selects a domain-local context encoding; and
- `context` identifies the affected backend, resource and internal failure
  stage according to that encoding.

The packed in-memory error type is `err_curag_t`, a `u32` whose upper 16 bits
contain `error_domain` and lower 16 bits contain `error_code`. Zero means
success. The diagnostic record stores the two halves separately. Domain and
error values are append-only protocol constants: once assigned, a value is
never changed or reused for another meaning.

The assigned component domains are:

| Value | Domain |
|---:|---|
| `0` | `NONE`, used only for success |
| `1` | `CURAG_EDOM_PERSISTENCE` |
| `2` | `CURAG_EDOM_RADIO` |
| `3` | `CURAG_EDOM_SENSORS` |
| `4` | `CURAG_EDOM_CORE` |

Firmware C identifiers use the `CURAG_` prefix. Unprefixed identifiers that
begin with `E` followed by an uppercase letter are reserved by the C standard
for implementation error macros, so names such as bare `EDOM_PERSISTENCE` and
`ENVS_INIT` are not used in headers.

The shared operation values are deliberately coarse and stable:

| Value | Operation |
|---:|---|
| `0` | `NONE` |
| `1` | `INITIALIZE` |
| `2` | `VALIDATE` |
| `3` | `READ` |
| `4` | `WRITE` |
| `5` | `APPEND` |
| `6` | `REMOVE` |
| `7` | `SYNC` |
| `8` | `RECOVER` |
| `9` | `COMPACT` |
| `10` | `POWER_ON` |
| `11` | `POWER_OFF` |
| `12` | `ENCODE` |
| `13` | `DECODE` |
| `14` | `ENCRYPT` |
| `15` | `DECRYPT` |
| `16` | `TRANSMIT` |
| `17` | `RECEIVE` |
| `18` | `SLEEP` |
| `19` | `CLEANUP` |

`NONE` is used when no operation is meaningful, so operation does not need a
validity flag. New component methods normally reuse one of these actions; an
operation value is added only for a genuinely new kind of action.

Every fallible component operation returns `err_curag_t` and may receive a
caller-owned diagnostic output:

```c
#define CURAG_DIAGNOSTIC_CONTEXT_MAX 252

typedef struct {
    uint16_t operation;
    uint8_t context_length;
    uint8_t context_schema;
    uint8_t context[CURAG_DIAGNOSTIC_CONTEXT_MAX];
} diagn_context_t;

_Static_assert(sizeof(diagn_context_t) == 256,
               "unexpected diagn_context_t layout");
```

The component clears this output on entry and, at the actual failure site,
populates the operation and a canonical domain-specific context. A null output
pointer discards diagnostic detail without changing the component operation.
The structure is never serialized or copied as a native memory dump.

Context-schema values are scoped by `error_domain`: two domains may assign the
same numeric value to unrelated schemas. Zero means no context and requires a
zero length. Values and their encodings are append-only within their domain.
The pilot begins with one schema per component, but this is not a permanent
restriction.

Components never persist diagnostics directly. `node_core` treats the
component-produced operation and context as opaque, extracts domain and code
from `err_curag_t`, adds application timing and the cycle ID when available,
and invokes `append_diagnostic_event` best-effort.

Every early-exit path powers off sensors and calls `radio.sleep` exactly once.
That call is a no-op when the radio was never initialized; otherwise it leaves
the SX1262 in cold-start sleep. A radio-sleep failure is logged when possible
but never prevents the ESP32 from entering deep sleep.

## Core invariants

- No reading is transmitted before its `sample_id` increment is committed.
- A `sample_id` may be skipped but never reused during one identity lifetime.
- No logical uplink frame is constructed before its `message_id` successor is
  committed. A message ID may skip but never wrap or be reused while the same
  identity/key is active; counter loss or exhaustion requires a new identity
  and key.
- All attempts for one `(node_id, message_id)` reuse the same domain and exact
  authenticated frame bytes.
- Converting a current reading into backlog allocates a new `message_id` and
  durably binds its exact backlog frame before TX; later-wake retries reuse that
  binding.
- An invalid ACK never changes delivery or persistence state.
- A pending reading is removed after authenticated `ACCEPTED`, after a
  permanent rejection has triggered a quarantine attempt, or when bounded
  pending-log pressure compaction evicts it under the retention policy below.
- `RETRY_LATER` stops all transmissions for the wake.
- Current delivery always precedes backlog drainage.
- The charged-TX budget and radio-cycle deadline are independent limits.
- Core admission uses radio timing queries rather than duplicating PHY timing
  constants.
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
- NVS contains independent next monotonic `sample_id` and `message_id` values.
- RTC memory carries metrics from one completed wake to the immediately
  following deep-sleep wake.
- LittleFS contains pending readings, optional exact backlog-frame bindings,
  quarantined readings and append-only diagnostic logs.
- RAM contains the active delivery state and exact encrypted frame used by all
  attempts for one `(node_id, message_id)`.

The current reading is persisted before its first transmission. If the node
resets afterward, the reading remains available as backlog. A LittleFS failure
puts the wake into a degraded RAM-only mode: the current reading may still be
transmitted, but it may be lost after sleep if it is not accepted.

No storage is automatically erased or reformatted after a failure during the
pilot.

Known pilot limitation: absence of an NVS counter key is interpreted as fresh
provisioning, so firmware cannot distinguish first use from counter erasure or
rollback. Erasing or restoring NVS while retaining the same identity and key is
therefore prohibited; recovery requires rotating both. The repository
maintenance application formats only LittleFS and deliberately preserves NVS.
An automatic counter-recovery handshake is deferred to a coordinated protocol,
node and receiver revision; the protocol document defines the required safety
properties of that future work.

Every operator-initiated identity rotation is a destructive boundary for
node-local state. Before production firmware using the new identity may
transmit, the operator erases NVS, all LittleFS logs and retained RTC state.
Pending readings and exact backlog-frame bindings from the retired identity are
discarded rather than migrated; receiver-side historical records remain. This
full-device rotation procedure is distinct from the ordinary LittleFS-only
maintenance application.

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

Known limitation: the native-layout record is protected only by its marker and
semantic checks. A retained-memory bit error that preserves those checks can
therefore be reported as valid previous-cycle metrics; a future version should
CRC a deterministic field encoding rather than raw struct bytes and padding.

Metric accumulators use wider internal types. They are serialized to the
protocol's `u8` and `u16` fields only if all values are representable. Overflow
known before final synchronization is logged best-effort. Overflow caused only
while `sync_all` advances awake time invalidates outgoing RTC without a late
persistent diagnostic; persistence is not reopened after the single sync.

## Components and responsibilities

The provisional callable contracts, ownership rules, failure results and
`node_core` diagnostic responses are defined in
[`INTERFACE.md`](INTERFACE.md).

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
claim_message_id
append_pending_reading
bind_newest_backlog_frame
peek_most_recent_pending
remove_newest_reading
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
claiming either ID includes the corresponding NVS commit, appends and
newest-pending removals commit their LittleFS state, and quarantine appends are
durable independently from pending removal. Backlog binding commits the
`message_id` and exact authenticated frame before first transmission. Delivery
events are also durable on successful return because an unmatched start must
survive a reset.
`sync_all` performs the one final synchronization for remaining buffered state,
including ordinary diagnostics. It skips backends that were never initialized
and does not initialize them merely to finalize a wake. It synchronizes and
closes persistence-owned file handles but does not unregister or unmount
LittleFS; deep sleep resets the in-memory mount state.

#### LittleFS record framing

`append_pending_reading`, `bind_newest_backlog_frame`, `quarantine_reading`,
`append_diagnostic_event` and `append_delivery_event` use one private record codec and the same single-file
append/truncate machinery. This sharing is internal to `node_persistence`; the
controller continues to see only semantic operations. It does not make the
operations share durability policy: pending, quarantine and delivery records
are durable when their semantic operation succeeds, while ordinary diagnostics
may remain buffered until `sync_all`.

Every record has this envelope:

| Field | Encoding | Meaning |
|---|---:|---|
| `magic` | `u32` | Storage-record marker |
| `format_version` | `u8` | Storage framing version, independent of the LoRa protocol version |
| `record_type` | `u8` | Selects the payload schema and semantic record kind |
| `payload_length` | `u16` | Number of payload bytes |
| `payload` | byte array | Type-specific canonical encoding |
| `total_length` | `u16` | Bytes from `magic` through the end of `crc32` |
| `crc32` | `u32` | CRC of every preceding field, including `total_length` |

All integer fields are little-endian. No implicit compiler padding or record
alignment is present. For a payload of `N` bytes, `total_length` is `N + 14`.
The maximum payload is 498 bytes and the maximum complete record is 512 bytes.
The decoder rejects larger values before allocating or reading the payload.
The duplicated lengths permit both forward traversal from the header and
newest-first traversal from the footer:

```text
magic | format_version | record_type | payload_length | payload
      | total_length | crc32
```

The storage framing constants are:

| Constant | Value |
|---|---:|
| `magic` | `0x756fec23` (`23 ec 6f 75` on disk) |
| `format_version` | `0x02` |
| `PENDING_READING` | `0x01` |
| `QUARANTINED_READING` | `0x02` |
| `DIAGNOSTIC_EVENT` | `0x03` |
| `DELIVERY_STARTED` | `0x04` |
| `DELIVERY_FINISHED` | `0x05` |
| `PENDING_BACKLOG_BINDING` | `0x06` |

Pending and quarantined readings may share a payload schema while retaining
different semantic record types. Unassigned record-type values are reserved.

#### Type-specific payloads

Every payload is encoded field by field in the order below. Multi-byte integers
are little-endian; no payload is a native C structure dump.

`PENDING_READING` and `QUARANTINED_READING` have the same 32-byte payload:

| Field | Encoding |
|---|---:|
| `reading_body` | canonical 32-byte LoRa v2 reading plaintext, beginning with `sample_id` |

The reading body is the output of the generated protocol codec. An unbound
pending reading ends with this record. A bound pending item immediately follows
it with `PENDING_BACKLOG_BINDING`, whose 62-byte payload is:

| Offset | Field | Encoding |
|---:|---|---:|
| 0 | `sample_id` | `u32` |
| 4 | `message_id` | `u32` |
| 8 | `backlog_frame` | exact 54-byte authenticated frame |

The binding's sample ID must match the decoded preceding reading. Its frame
must have the backlog domain and the same message ID in clear-header bytes
10-13. Pending recovery, newest-first selection, removal and compaction treat
the reading plus optional binding as one item and never retain a binding
without its reading.

`DELIVERY_STARTED` has a 17-byte payload:

| Field | Encoding |
|---|---:|
| `cycle_sample_id` | `u32` |
| `sample_id` | `u32` |
| `message_id` | `u32` |
| `domain` | `u8`, exact LoRa v2 domain byte |
| `start_offset_ms` | `u32`, relative to application start |

`DELIVERY_FINISHED` has a 15-byte payload:

| Field | Encoding |
|---|---:|
| `cycle_sample_id` | `u32` |
| `sample_id` | `u32` |
| `message_id` | `u32` |
| `domain` | `u8`, exact LoRa v2 domain byte |
| `attempt_count` | `u8`, attempts in this wake |
| `final_result` | `u8` |

The numeric `final_result` mapping is the delivery-result table in the wake
cycle section above.

For the pilot, `DIAGNOSTIC_EVENT` has this variable-size payload:

| Field | Encoding |
|---|---:|
| `error_domain` | `u16` |
| `error_code` | `u16` |
| `flags` | `u16` |
| `application_offset_ms` | `u32` |
| `cycle_sample_id` | `u32` |
| `message_id` | `u32`, zero unless its validity flag is set |
| `operation` | `u16` |
| `context_length` | `u8` |
| `context_schema` | `u8`, scoped by `error_domain` |
| `context` | `context_length` bytes |

The fixed diagnostic prefix is 22 bytes and context is limited to 252 bytes,
so a diagnostic payload is at most 274 bytes and its complete storage record is
at most 288 bytes.
The initial flags are:

| Bit | Flag |
|---:|---|
| `0` | `APPLICATION_OFFSET_VALID` |
| `1` | `CYCLE_SAMPLE_ID_VALID` |
| `2` | `MESSAGE_ID_VALID` |
| `3`–`15` | Reserved; must be zero |

When any validity flag is clear, its corresponding numeric field must be zero.
When set, every representable value is permitted, including zero.
`context_schema = 0` if and only if `context_length = 0`; a nonzero known
schema requires its exact defined length. Context is bounded and canonically
encoded, never truncated dynamically and never copied from arbitrary memory.
Unknown schema values are preserved for offline inspection but are not decoded
by the current firmware. Additional error domains, domain-specific error codes
and context schemas remain to be assigned before this provisional payload is
frozen.

#### Physical files

Record families are isolated so they can have independent durability,
retention and failure policies:

| File | Permitted record types |
|---|---|
| `pending.log` | `PENDING_READING`, `PENDING_BACKLOG_BINDING` |
| `quarantine.log` | `QUARANTINED_READING` |
| `diagnostic.log` | `DIAGNOSTIC_EVENT` |
| `delivery.log` | `DELIVERY_STARTED`, `DELIVERY_FINISHED` |

Each is one append-only logical file. Pending records are additionally removed
from its tail after acceptance or quarantine because backlog selection is
newest-first.

The checksum is CRC-32/ISO-HDLC, calculated on the serialized bytes from
`magic` through `total_length`:

```text
polynomial  0x04c11db7
init        0xffffffff
refin       true
refout      true
xorout      0xffffffff
check       CRC32("123456789") = 0xcbf43926
```

Firmware uses `esp_crc32_le(0, bytes, length)` and stores the result as a
little-endian `u32`; host tools use the same specified algorithm. CRC32 detects
accidental storage corruption but is not an authentication mechanism. It is
still required despite LoRa AES-CCM: a corrupted plaintext reading would
otherwise be encrypted afterward with a valid CCM tag and accepted by the
receiver.

Before trusting either length, the decoder applies a compile-time maximum,
checks all arithmetic for overflow and requires:

```text
total_length == payload_length + 14
```

It then validates magic, matching lengths and CRC before checking whether the
format and type are supported and decoding the payload. An unsupported record
away from the tail is preserved and reported. The pilot may discard an
unsupported or semantically invalid newest record only under the bounded tail
recovery policy below.

#### Append and tail recovery

Before any append, the component must establish or repair a trustworthy tail;
otherwise torn bytes could be buried as unrecoverable middle corruption. An
append then constructs the complete canonical record, appends all bytes and
applies the semantic operation's synchronization policy. A successful durable
append therefore includes both the footer and CRC.

`remove_newest_reading` first reconstructs the newest complete pending item,
validates that its decoded reading has the caller's expected `sample_id`, then
truncates the reading plus its optional immediately following binding in one
operation and synchronizes before returning success. It never removes an
orphan binding, an unidentified item or a different supported reading.

An operation that needs a log tail applies this policy:

1. If the file is empty, recovery succeeds with no record.
2. Read the final six bytes containing `total_length` and `crc32`. Treat a
   short footer or a length outside the configured bounds as an invalid tail
   candidate and continue with the bounded scan in step 6.
3. Calculate the candidate start, read the complete record, validate all
   framing and CRC fields, check that its type is permitted in that physical
   file, and decode its type-specific payload.
4. If all checks succeed, the tail is usable and the requested operation may
   proceed.
5. If framing and CRC establish an exact complete record but its version or
   type is unsupported, or its known payload is semantically invalid, truncate
   that one record and synchronize. Return `CURAG_EUNSUPPORTED_RECORD` or
   `CURAG_ECORRUPT_RECORD` respectively without executing the requested
   operation.
6. For a torn or CRC-invalid tail, examine candidate record ends backwards over
   at most one maximum record length. A candidate is accepted only when its
   footer, header, lengths and CRC all agree. If the entire file is no larger
   than one maximum record, offset zero is also a valid preceding boundary.
7. If a preceding boundary is proven, truncate the unusable suffix,
   synchronize, and return `CURAG_ECORRUPT_RECORD` without executing the
   requested operation.
8. If no boundary can be established without discarding more than one maximum
   record, preserve the file and return `CURAG_ECORRUPT_RECORD` instead of
   guessing or formatting LittleFS.

Returning an error after successful destructive recovery ensures `node_core`
can record the lost tail. The next wake or later explicit call starts from the
repaired boundary. Recovery that successfully discarded bytes uses operation
`RECOVER`, stage `TRUNCATE` and backend status `NO_ERROR`; failure to establish
a boundary uses stage `TAIL_SCAN`; a failed truncation or synchronization uses
the exact primitive stage and backend status.

Caching a per-file `UNVALIDATED -> READY/FAILED` state so the tail is checked
only once per wake is a possible future optimization, not an interface
requirement. An implementation may revalidate whenever an operation needs the
tail, provided it always validates before appending.

Forward iteration starts at offset zero, reads the fixed header, bounds-checks
`payload_length`, and validates the complete record before advancing by
`total_length`. Corruption discovered away from the tail is reported and is not
automatically truncated because later records may still be valid. Recovery
never formats LittleFS automatically.

#### Quarantine transition

Quarantining a pending reading is not atomic across its two files. The
controller uses two independent persistence calls in loss-averse order:

1. Call `quarantine_reading(reading)` to append and synchronize the
   `QUARANTINED_READING` record in `quarantine.log`.
2. Call `remove_newest_reading(sample_id)` even if the quarantine append
   failed. It validates and durably truncates the matching tail from
   `pending.log`.

A reset between those steps can leave the reading in both files. This is
allowed: loss is less acceptable than duplication, and offline tooling must
tolerate duplicates identified by `sample_id` and reading body. There is no
idempotent recovery in the pilot, so retrying the transition may also leave
duplicate records permanently in `quarantine.log`.

For a current RAM-only reading whose pending append did not succeed, the
controller calls only `quarantine_reading`; an ambiguously durable pending tail
is left for later recovery. If the quarantine append cannot be retained because
its file is at its quota or storage is unavailable, the controller still
attempts to remove a known persisted rejected reading. Losing that diagnostic
copy is preferable to repeatedly selecting the same rejected record and
obstructing sensing and backlog progress. Each call returns and populates its
own diagnostic independently.

#### Retention and compaction

For the at-most-one-month pilot, the logical file limits are deliberately much
smaller than the 2,944 KiB LittleFS partition:

| File | Logical limit |
|---|---:|
| `pending.log` | 512 KiB |
| `quarantine.log` | 192 KiB |
| `diagnostic.log` | 256 KiB |
| `delivery.log` | 256 KiB |

These limits total 1,216 KiB. A file reaching its logical limit does not imply
that LittleFS is physically full; the unallocated capacity is intentional
workspace for copy-on-write, garbage collection and compaction.

When the next pending append would exceed 512 KiB, compact `pending.log` by
retaining the newest complete pending items whose combined record size is at
most 50% of the limit. A bound item is retained or discarded only as its
reading-plus-binding pair. Write retained records in their original order to
`pending.compact`, synchronize the complete temporary file, and atomically
rename it over `pending.log` in the same directory. Until that rename commits,
`pending.log` remains authoritative;
on initialization, an extra `pending.compact` beside it is an interrupted
pre-rename copy and may be removed after validating the authoritative file.
The largest compacted copy is therefore 256 KiB. Even with all logical files
at their quotas, the temporary copy leaves substantial space for LittleFS
metadata and garbage collection.

Compaction always retains whole structurally valid records. Failure to create,
synchronize or replace the temporary file leaves the original pending log
authoritative and makes the requested append fail; the current reading can
still be transmitted from RAM under the ordinary persistence-failure policy.
`pending.compact` is never promoted to recover an absent, corrupt or otherwise
unrecoverable `pending.log` during the pilot. In that situation both files are
preserved when present and the persistence failure is reported; automatically
choosing the temporary file would require generation or manifest metadata that
the pilot deliberately omits.

No recovery or reclamation strategy has yet been chosen for a full
`quarantine.log`, `diagnostic.log` or `delivery.log`. At present they reject new
records after reaching their limits. Such logging failures never block sensing,
radio work or deep sleep, and the special quarantine rule above still removes
the rejected pending reading. Defining how those files resume recording after
physical collection or autonomous reclamation is an open production-design
item.

#### Open pilot recovery decisions

The pilot deliberately leaves these recovery procedures undefined:

- reclaiming space after `quarantine.log`, `diagnostic.log` or `delivery.log`
  reaches its logical limit; and
- physically recovering a log whose tail has no trustworthy preceding record
  boundary.

Their safe runtime behavior is already fixed. A full non-pending log rejects
new records according to its best-effort failure policy. When bounded tail
recovery cannot prove a boundary, persistence preserves every byte and returns
`CURAG_ECORRUPT_RECORD`; it never guesses, reformats LittleFS or promotes
`pending.compact`. An unrecoverable `pending.log` disables backlog persistence,
but the node may continue transmitting each current reading from RAM. Restoring
either condition requires a physical recovery workflow that is intentionally
outside the pilot design.

### `node_sensors`

`node_sensors` owns sensor sequencing and exposes:

```text
sample_all
force_power_off
```

`sample_all` lazily initializes required buses, enables the shared gated sensor
rail, samples both soil channels and both DS18B20 channels, releases that rail,
then samples the independent always-powered BME280. Its caller-owned snapshot
has five validity bits: one for each soil channel, one for each DS18B20 channel
and one atomic BME280 enclosure group. The enclosure bit validates temperature,
pressure and humidity together; all three are zero when it is clear. Invalid
individual channels are zero, while other channels continue to be sampled.
`node_core` maps the enclosure bit to all three enclosure validity bits in the
LoRa reading.

A validity bit means that acquisition and structural conversion succeeded. It
does not assert that a connected sensor is healthy or that the value is
physically or agronomically plausible. The pilot retains representable
outliers for receiver-side analysis. Under correct wiring, the DFRobot soil
sensor's documented 3 V maximum output is the accepted ADC-voltage assumption.

Sensor diagnostics use one fixed context with six backend-status pairs: one
component-wide pair followed by the five sensor groups. The fixed group pairs
preserve independent simultaneous failures without making `node_core`
interpret driver results. The single component-wide pair describes the
selected shared initialization, power-gate or cleanup failure, so a later
higher-precedence shared failure may replace an earlier exact shared status.
Affected group slots still record that they were blocked. `err_curag_t` remains
a summary result, while the context carries the selected exact ESP-IDF,
sensor-driver or component-internal statuses.

The DS18B20 backend reports shared acquisition, both channel acquisitions and
resource cleanup separately. Cleanup failure never invalidates temperatures
already acquired successfully. It is still returned as a nonzero component
result because leaked or incompletely released bus resources may affect later
work in the same wake.

`force_power_off` is idempotent, initializes no sensor bus, never powers a
device on and reports failure only when it cannot enforce the gated rail's off
state.

One power gate supplies both soil sensors and both DS18B20 sensors. The BME280
remains on the always-powered rail and is intended to use its low-power
operating mode. The board must default the switched sensor rail to off whenever
the MCU does not actively enable it.

Hardening of the current generated Espressif BME280 dependency is deferred.
The eventual implementation must either pin a corrected fork while submitting
the fixes upstream, or replace the dependency after evaluating another driver,
starting with Bosch's official SensorAPI. Required fixes include bounded
initialization and forced measurement, allocation failure handling, polling
error propagation and reliable low-power transitions. Generated
`managed_components` are never patched directly.

The pilot gate is a high-side P-MOSFET: source at 3.3 V, drain at the switched
sensor rail and gate pulled to its source by 47 kOhm. A non-strapping ESP32-C6
GPIO controls the gate as an open-drain output with both internal pulls
disabled. Driving it low turns the rail on; releasing it lets the resistor turn
the rail off. Power-on loads the inactive high/released latch before enabling
open-drain output, then asserts low. Power-off releases the output first and
then returns the GPIO to a floating input. It does not use GPIO hold. Because
off changes the direction, every later power-on configures open-drain output
again. Reset, deep sleep or an unconfigured MCU therefore leaves the rail off
without relying on firmware execution.

The backend waits 200 ms after enabling the rail. The two DS18B20 probes are
externally powered and use one bus-wide 12-bit conversion. Their 1-Wire pull-up
must connect to the switched rail rather than the always-on 3.3 V rail; the
ESP32 internal pull-up is disabled and the bus is released before the rail is
switched off. This avoids back-powering the probes through the data line.

### `sx1262_radio`

The radio component owns SX1262 commands, IRQ handling and all pilot PHY
configuration. It exposes protocol-oriented operations:

```text
transmit_uplink
receive_downlink_until
sleep
```

The component owns one file-static singleton; no state pointer or public
initialization function is exposed. Ordinary BSS initialization places it in
`UNTOUCHED` at each ESP32 boot. The first transmit lazily initializes the radio.
Uplink transmission uses normal IQ and downlink reception uses inverted IQ as
defined by the protocol.

TX and RX operations receive an absolute deadline expressed in the same
monotonic-microsecond domain used by `node_core`. The component has a private
`monotonic_us` dependency so it can enforce those deadlines and timestamp IRQ
events. The transmit result reports whether `SetTx` started and the actual
pre-`SetTx` and `TX_DONE` monotonic times, so initialization latency is not
counted as delivery time. Reception returns SX1262 payload bytes, length and
radio metadata, or a normal deadline outcome or local error. It never exposes
the LoRa preamble, PHY header or modem-generated CRC, and it never parses the
Cura frame.

The radio exposes fixed-profile timing queries for modeled LoRa time-on-air and
a conservative controller-admission TX window. Time-on-air is calculated with
quarter-symbol arithmetic from SF, bandwidth, coding rate, low-data-rate
optimization, preamble length, header mode, payload length and CRC. Carrier
frequency remains a profile setting but does not change symbol duration. For
the 54-byte reading frame the model returns 102,656 us; `node_core`
independently charges 112,922 us after its 10% allowance. The conservative
minimum TX window for that frame is 112,704 us.

Within an active wake the radio uses `STDBY_RC` as the intermediate state for
configuration and transitions between TX and RX. It is not put to sleep while
delivery or retries remain possible. Final `sleep` behaves as follows:

1. If the radio is `UNTOUCHED`, or initialization failed before any hardware
   access, return successfully without issuing a radio command or initializing
   it.
2. Otherwise stop an armed receive or any other active mode with
   `SetStandby(STDBY_RC)`.
3. Issue `SetSleep(COLD_START)`; the next wake performs complete lazy
   initialization rather than trusting retained radio configuration.
4. Return any failure for best-effort diagnostics, but do not prevent ESP32
   deep sleep.

The controller owns retries, deadlines, ACK validation and delivery policy; the
radio component only executes radio operations and reports their outcomes.
The detailed callable contract and implemented diagnostic/state model are in
[`INTERFACE.md`](INTERFACE.md).

The production backend vendors Semtech's Clear-BSD `sx126x_driver` v2.5.0 at a
pinned upstream commit. It owns SPI2, GPIO setup, reset/BUSY handling and a
DIO1 rising-edge ISR. `esp_timer_get_time()` is used both by ordinary deadline
checks and by the ISR, while a static binary semaphore wakes the waiting task.
The selected Waveshare Pico-LoRa-SX1262-868M uses its onboard DIO2 RF switch,
DIO3 1.7 V TCXO and the SX1262 DC-DC regulator. SPI runs at 8 MHz; the seven
ESP32-C6 pins are provisional component Kconfig values until the board is
assembled.

BUSY waits are bounded to 10 ms, reset startup to 20 ms and the radio TX
watchdog to five milliseconds before the caller deadline when representable.
After packet setup, transmission is rejected before `SetTx` unless the
programmed watchdog can contain the modeled airtime and TX ramp after its
five-millisecond margin and 64 kHz quantization.
After each non-sleep Semtech command, the backend reads chip status so a driver
or HAL success cannot hide command timeout, processing or execution failure.
The +14 dBm profile uses the SX1262 datasheet's lower-current PA configuration
for that radiated power.

### Existing protocol and sensor components

`protocol_v2_lora` remains the generated wire codec and authenticated frame
layer. The former standalone `soil_sensor` component has been folded into
`node_sensors`; its public `soil_sensor.h` compatibility API remains available
to calibration and maintenance applications and assumes the caller already
powered the probe. The production backend uses pinned Espressif `ds18b20`,
`onewire_bus` and `bme280` components. These hardware adapters do not own wake
policy.

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
waiting. It is nondecreasing within a wake and shares its concrete source with
the radio backend. Retry randomness is inclusive, unbiased and does not need
cryptographic guarantees; it is never used for identity or key material.

`get_reset_reason` exposes the `u8` numeric value of `esp_reset_reason_t`; sleep
wakeup causes are not collected. The system implementation performs the final
board-safe transition and enters timer deep sleep for a relative duration.
From the controller's perspective `enter_deep_sleep_for` is terminal and cannot
fail. Production configures timer wakeup and enters deep sleep; if wakeup
configuration fails, the system adapter reports to the development console,
waits 60 seconds without a tight busy-spin and restarts instead of returning.
A host fake may return only to terminate the test invocation, after which
`node_core` performs no more operations. The complete callable contracts are in
[`INTERFACE.md`](INTERFACE.md#platform-ports).

### `app_main` and RTC memory

`app_main` is the composition root. It owns the concrete component state,
constructs the injected port table and invokes one wake cycle. It contains no
wake policy.

RTC memory is direct retained data rather than an injected interface:

```c
RTC_DATA_ATTR static node_rtc_record_t rtc_record;

void app_main(void) {
  node_cycle_run(&platform, &identity, &rtc_record);
}
```

The composition root constructs `identity` from the generated private node
identity header. Passing it into the reusable controller keeps that secret
header out of the `node_core` component and lets host tests use a fixed test
identity.

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

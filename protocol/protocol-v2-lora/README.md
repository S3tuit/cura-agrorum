# Cura Agrorum LoRa Protocol v2

Status: early design draft. This records current decisions, not a complete
specification.

## Model

- Custom raw-LoRa, sessionless node-to-receiver protocol.
- Each receiver group has a secret master key.
- Each node has a random 64-bit `node_id` and a unique 128-bit node key derived
  from the group master key and `node_id` using HKDF-SHA-256.
- `node_id` is public. A node stores only `node_id` and its derived node key,
  never the group master key.
- The receiver derives node keys as needed. It still persists replay state,
  node metadata, revocation state and readings.
- No UUID and no identity/session handshake are used on the LoRa link.
- Each receiver group contains exactly one receiver during the pilot.
- A node belongs to exactly one receiver group during an identity lifetime.
  Roaming and redundant receivers are not supported.
- Moving a node to another receiver group requires physical reprovisioning with
  a fresh `node_id` and a node key derived from the destination group's master
  key.

## Keys and provisioning

Each receiver group has one `group_master_key`, generated once as 32 random
bytes with Python's `secrets.token_bytes(32)`. Each node identity gets an
independent eight-byte `node_id` from `secrets.token_bytes(8)`. Zero, an active
collision and a previously retired ID are rejected and regenerated. `node_id`
is public; the master key and derived node keys are secret.

An identity lifetime is the period during which a node uses one `node_id`,
its corresponding `node_key` and one monotonic `sample_id` sequence. There is
no explicit epoch field or counter. Replacing both `node_id` and `node_key`
starts a new identity lifetime.

The node key is derived exactly as follows:

```text
KDF     = HKDF-SHA-256 (extract and expand)
IKM     = group_master_key                                   32 raw bytes
salt    = node_id                                             8 raw bytes
info    = ASCII("cura-agrorum/protocol-v2-lora/aes-128-ccm/node-key/v1")
L       = 16 bytes
output  = node_key
```

The salt is the same opaque eight-byte `node_id` transmitted on the wire. The
`info` string has no terminating NUL. Hexadecimal text is used only for display
and storage formats where explicitly required; it is not passed to HKDF.

Provisioning installs only `node_id` and `node_key` on a node. A node never
receives the group master key. The receiver stores the group master key and a
persistent allowlist of accepted node IDs and derives node keys as needed.
Production keys must not appear in source control, test fixtures or logs.

The pilot provisioning tools use fixed filenames within overridable
directories:

```sh
python protocol/protocol-v2-lora/tools/init_receiver_group.py \
  [--recv-dir PATH]
python protocol/protocol-v2-lora/tools/provision_node.py \
  [--recv-dir PATH] [--node-dir PATH]
```

The first command exclusively creates the ignored `receiver-group.json`. A
master-key rotation requires both `--rotate-master-key` and
`--acknowledge-all-nodes-require-reprovisioning`; it gives the group a new ID
and key, retires every active node ID and leaves the active set empty.

The second command creates the ignored `protocol_v2_lora_identity.h` and adds
its random ID to the receiver's active set. It refuses an existing header.
`--replace-staged-identity` creates another active identity without retiring
the header's previous ID, while `--rotate-node-identity` retires the previous
ID. Both secret files are written with mode `0600`; existing secret files
accessible by group or others are rejected. The tools print public IDs and
paths but never keys.

The following public, non-production test vector is used by both Python and
firmware tests to detect HKDF parameter or byte-encoding differences:

```text
group_master_key = 000102030405060708090a0b0c0d0e0f
                   101112131415161718191a1b1c1d1e1f
node_id          = 0102030405060708
node_key         = c0f9a1a0f386692e01028082be92330e
```

### Revocation and identity replacement

To revoke a node, the receiver disables its `node_id` in the persistent
allowlist and removes its derived key, authentication state and replay state
from RAM. The ID remains in a persistent retired-ID set and is never reused.
Unknown or revoked IDs receive no response. Historical readings are not erased
by protocol-level revocation.

When an identity must be replaced, use a new random `node_id` and its newly
derived node key:

1. Add the new ID to the receiver allowlist.
2. Provision the node with the new ID and key and verify communication.
3. Revoke the old ID.

If compromise or theft is suspected, revoke the old ID before provisioning the
replacement, accepting the resulting downtime. Compromise of one node key
requires replacing only that node's identity. Compromise of the group master
key requires a new group master key and reprovisioning every node in the group;
changing only their public node IDs is insufficient.

## Routine packet

```text
control       1 byte
domain        1 byte
node_id       8 bytes
sample_id     4 bytes
ciphertext    N bytes
CCM tag       8 bytes
```

- AES-CCM-128 provides encryption and authentication.
- The clear header is authenticated as associated data:

  ```text
  AAD = control || domain || node_id || sample_id
  ```

### Control byte

```text
bits 7–4   protocol version
bits 3–0   protocol flags
```

- Protocol version `0` is reserved as invalid.
- The pilot uses protocol version `2`, with all flags reserved and zero:
  `control = 0x20`.
- A version-2 packet with nonzero protocol flags is unsupported.
- `control` is authenticated as AAD and must not change between repeated uses of
  the same `(node_id, sample_id, domain)` nonce.

### Byte order and serialization

- All multi-byte integers, including `sample_id` and reading fields, use
  little-endian byte order.
- Signed integers use two's-complement representation.
- `node_id` is an opaque sequence of eight bytes, not a numeric field requiring
  byte-order conversion.
- Implementations explicitly encode and decode fields at their defined offsets.
  They must not transmit a native C structure or include compiler padding.

## Reading payload

The encrypted application body for a reading is 28 bytes:

| Offset | Field | Type | Meaning |
|---:|---|---|---|
| 0 | `run_ms` | `u16` | Milliseconds from application start until the current reading body is finalized, immediately before persistence and frame construction. |
| 2 | `soil_0_mv` | `u16` | Soil-sensor channel 0 output in millivolts. |
| 4 | `soil_1_mv` | `u16` | Soil-sensor channel 1 output in millivolts. |
| 6 | `soil_temp_0_centi_c` | `i16` | Soil-temperature channel 0 in 0.01 degrees Celsius. |
| 8 | `soil_temp_1_centi_c` | `i16` | Soil-temperature channel 1 in 0.01 degrees Celsius. |
| 10 | `enclosure_centi_c` | `i16` | Enclosure temperature in 0.01 degrees Celsius. |
| 12 | `enclosure_pressure_pa` | `u32` | Enclosure pressure in pascals. |
| 16 | `enclosure_humidity_centi_pct` | `u16` | Enclosure relative humidity in 0.01 percent. |
| 18 | `reset_reason` | `u8` | Reason for the current boot, using the pilot mapping below. |
| 19 | `previous_current_tx_attempts` | `u8` | All transmission attempts for the previous cycle's current reading, including the first. |
| 20 | `previous_awake_ms` | `u16` | Previous cycle's total awake time, including transmission and ACK waiting. |
| 22 | `previous_current_delivery_ms` | `u16` | Time immediately before the previous current reading's first `SetTx` until its authenticated `ACCEPTED`. |
| 24 | `previous_cycle_tx_attempts` | `u8` | All current and backlog reading transmissions in the previous wake. |
| 25 | `previous_cycle_accepted_readings` | `u8` | Distinct readings for which the node received authenticated `ACCEPTED` in the previous wake. |
| 26 | `flags` | `u16` | Validity and diagnostic bitmap. |

Channel numbers identify stable node connectors or configured sensor identities;
they do not encode depth. The receiver stores each channel's depth, calibration,
probe identity and other installation metadata. DS18B20 ROM-address assignment
must not depend on bus enumeration order.

The `flags` bitmap is:

| Bit | Name | Meaning when set |
|---:|---|---|
| 0 | `DEEP_SLEEP_BOOT` | The current boot was caused by the expected deep-sleep wake. |
| 1 | `SOIL_0_VALID` | `soil_0_mv` is valid. |
| 2 | `SOIL_1_VALID` | `soil_1_mv` is valid. |
| 3 | `SOIL_TEMP_0_VALID` | `soil_temp_0_centi_c` is valid. |
| 4 | `SOIL_TEMP_1_VALID` | `soil_temp_1_centi_c` is valid. |
| 5 | `ENCLOSURE_TEMP_VALID` | `enclosure_centi_c` is valid. |
| 6 | `ENCLOSURE_PRESSURE_VALID` | `enclosure_pressure_pa` is valid. |
| 7 | `ENCLOSURE_HUMIDITY_VALID` | `enclosure_humidity_centi_pct` is valid. |
| 8 | `PREVIOUS_CYCLE_METRICS_VALID` | All previous-cycle metrics refer to the immediately preceding wake. |
| 9 | `PREVIOUS_CURRENT_ACCEPTED` | The previous cycle's current reading received authenticated `ACCEPTED`. |
| 10–15 | Reserved | Must be zero. |

For the pilot, `reset_reason` uses the numeric values of ESP-IDF's
`esp_reset_reason_t`, frozen into this protocol as follows:

| Value | ESP-IDF name | Meaning |
|---:|---|---|
| 0 | `ESP_RST_UNKNOWN` | Reset reason could not be determined. |
| 1 | `ESP_RST_POWERON` | Power-on reset. |
| 2 | `ESP_RST_EXT` | External-pin reset. |
| 3 | `ESP_RST_SW` | Software reset through `esp_restart`. |
| 4 | `ESP_RST_PANIC` | Exception or panic reset. |
| 5 | `ESP_RST_INT_WDT` | Interrupt-watchdog reset. |
| 6 | `ESP_RST_TASK_WDT` | Task-watchdog reset. |
| 7 | `ESP_RST_WDT` | Other watchdog reset. |
| 8 | `ESP_RST_DEEPSLEEP` | Reset after exiting deep sleep. |
| 9 | `ESP_RST_BROWNOUT` | Brownout reset. |
| 10 | `ESP_RST_SDIO` | SDIO reset. |
| 11 | `ESP_RST_USB` | USB-peripheral reset. |
| 12 | `ESP_RST_JTAG` | JTAG reset. |
| 13 | `ESP_RST_EFUSE` | eFuse-error reset. |
| 14 | `ESP_RST_PWR_GLITCH` | Power-glitch reset. |
| 15 | `ESP_RST_CPU_LOCKUP` | CPU-lockup reset. |

The node explicitly converts the returned enum value to `u8`; it does not
serialize the C enum representation. Values 16–255 are unassigned but are
accepted and retained numerically so a newer ESP-IDF diagnostic is not lost.
`DEEP_SLEEP_BOOT` must be set if and only if `reset_reason` is 8
(`ESP_RST_DEEPSLEEP`). Sleep wakeup sources are not carried in this pilot.

An invalid measurement field is ignored by the receiver and must be encoded as
zero. After a cold boot, both previous-cycle flags are clear and all
previous-cycle metrics are zero. `previous_current_delivery_ms` is zero unless
`PREVIOUS_CURRENT_ACCEPTED` is set. The 30-second radio-cycle deadline keeps
`previous_current_delivery_ms` within `u16`. Final persistence may extend
`previous_awake_ms` beyond 30 seconds but must still finish while the total is
representable by `u16`.

The wire encoding has no alignment requirement and contains no implicit C
padding.

This reading makes the SX1262 payload `14 + 28 + 8 = 50` bytes, excluding the
modem-generated payload CRC.

## LoRa PHY framing

The complete transmitted frame is layered as follows:

```text
LoRa RF packet
|-- LoRa PHY framing overhead
|   |-- PHY preamble: 8 configured symbols + 4.25 fixed symbols
|   `-- Explicit PHY header: 12-bit PHDR + 8-bit PHDR_CRC
|-- Cura application frame (the payload passed to the SX1262)
|   |-- Clear authenticated header / AAD       14 bytes
|   |-- Encrypted application body              N bytes
|   `-- AES-CCM authentication tag              8 bytes
`-- LoRa PHY payload CRC                       2 bytes
```

Current radio decisions:

- Use an SX1262 at a carrier frequency of 868.1 MHz and configured output power
  of +14 dBm. Antenna gain and RF-path losses must still be included when
  checking the permitted e.r.p.
- Use SF7, BW 125 kHz, coding rate 4/5 and low-data-rate optimization off.
- Use RX boosted/high-sensitivity mode and a 40 us PA ramp time.
- Use the standard private LoRa sync word. An API accepting the legacy
  one-byte notation receives `0x12`; an API accepting the raw SX1262 register
  value receives `0x1424` (`0x14` at address `0x0740`, then `0x24` at
  `0x0741`). Confirm the driver API width rather than truncating `0x1424`.
- Use explicit-header mode.
- Configure an 8-symbol preamble. The complete PHY preamble therefore occupies
  `8 + 4.25 = 12.25` symbols.
- Enable the 16-bit LoRa payload CRC. It is generated and checked by the modem
  and is separate from the CCM authentication tag.
- The SX1262 payload length is `PL = 14 + N + 8 = N + 22` bytes. It excludes
  the preamble, PHY header and modem-generated payload CRC.

Preamble, header, payload, CRC and FEC cannot be added as a simple byte count;
the complete transmission size is expressed in LoRa symbols and depends on SF,
bandwidth and low-data-rate optimization.

The sync word must match at both ends. It rejects unrelated LoRa networks
earlier in reception but provides neither authentication nor encryption and
does not prevent RF collisions. It does not change packet airtime; AES-CCM
provides protocol security.

### Pilot airtime constants

The fixed pilot profile above gives a symbol time of 1.024 ms. Exact airtime
and the conservative charge applied to the corresponding transmitter's rolling
budget are:

| Packet | SX1262 payload | On-air symbols | Airtime | Charged airtime |
|---|---:|---:|---:|---:|
| Reading | 50 bytes | 95.25 | 97.536 ms | 107.290 ms |
| ACK | 23 bytes | 60.25 | 61.696 ms | 67.866 ms |

Charged airtime is `ceil(actual_airtime_us * 1.10)` and is maintained in integer
microseconds. The 10% margin is internal conservative accounting; it does not
represent additional RF transmission. Every attempted transmission is charged,
irrespective of reception or authentication outcome.

## CCM nonce

```text
node_id       8 bytes
sample_id     4 bytes
domain        1 byte
             --------
             13 bytes
```

`domain` separates directions and logical packet types. Under one node key,
each `(node_id, sample_id, domain)` combination identifies at most one logical
packet.

`control` is separate from `domain` to leave room for protocol versioning,
format selection and flags without consuming nonce-domain values.

The domain byte is an opaque enum without internal bit semantics. Value `0x00`
is reserved. Pilot values are permanent and must not be reassigned:

| Value | Domain |
|---:|---|
| `0x01` | `CURRENT_READING_UPLINK` |
| `0x02` | `BACKLOG_READING_UPLINK` |
| `0x03` | `ACK_ACCEPTED_DOWNLINK` |
| `0x04` | `ACK_RETRY_LATER_DOWNLINK` |
| `0x05` | `ACK_REJECTED_UNSUPPORTED_DOWNLINK` |
| `0x06` | `ACK_REJECTED_MALFORMED_DOWNLINK` |

A reading starts in `CURRENT_READING_UPLINK`. If it is not accepted and is
stored locally, a later wake sends it using `BACKLOG_READING_UPLINK`. The
different domain produces a different nonce. Repeated attempts within one
domain use identical AAD, ciphertext and tag.

## Counters

- `sample_id` is an unsigned persistent 32-bit monotonic counter.
- It must never restart while the same node key remains in use. Losing the
  counter requires provisioning a new identity/key or another specified
  rekeying procedure.

## ACK behaviour

- ACKs use the `sample_id` of the reading being acknowledged and the domain that
  corresponds to their status.
- The ACK encrypted application body is a one-byte `status` value:

  | Domain | Status | Meaning |
  |---|---:|---|
  | `ACK_ACCEPTED_DOWNLINK` | `0` | `ACCEPTED` |
  | `ACK_RETRY_LATER_DOWNLINK` | `1` | `RETRY_LATER` |
  | `ACK_REJECTED_UNSUPPORTED_DOWNLINK` | `2` | `REJECTED_UNSUPPORTED` |
  | `ACK_REJECTED_MALFORMED_DOWNLINK` | `3` | `REJECTED_MALFORMED` |

- The node accepts an ACK only after CCM authentication and only when its domain
  and decrypted status match the table. A mismatch is an invalid ACK.

- A reading is accepted after it has been authenticated, decoded, checked and
  inserted into the receiver's bounded in-memory queue.
- An ACK does not guarantee durable disk persistence. The receiver may persist
  an accepted reading asynchronously after sending the ACK.
- A reading may be timestamped after being persisted to disk.
- A recognized authenticated duplicate receives `ACCEPTED` again.
- A full receiver queue produces `RETRY_LATER`.
- Unsupported and malformed authenticated readings receive their corresponding
  permanent rejection. The node retains them for diagnosis but does not keep
  sending them unchanged.
- Implausible sensor values are accepted and recorded as anomalous when the
  packet is otherwise valid.
- Invalid authentication or an unusable header receives no response.

## Receiver validation

The pilot performs structural validation but does not reject representable
sensor or diagnostic values for being physically implausible. Outliers such as
`soil_0_mv = 3000` are retained without requiring the receiver to classify them
as good or bad.

Reading-value rules are:

- Every `u8`, `u16`, `u32` and `i16` bit pattern is accepted for its field unless
  a structural rule below says otherwise. Assigned and unassigned
  `reset_reason` values are retained numerically.
- When a sensor `*_VALID` flag is clear, its field must be zero.
- When a sensor `*_VALID` flag is set, every representable value is accepted,
  including zero.
- Reserved reading-flag bits must be zero.
- `DEEP_SLEEP_BOOT` must be set if and only if `reset_reason` is 8; a mismatch
  is malformed.
- If `PREVIOUS_CYCLE_METRICS_VALID` is clear, all five previous-cycle metrics
  must be zero and `PREVIOUS_CURRENT_ACCEPTED` must be clear.
- `PREVIOUS_CURRENT_ACCEPTED` may be set only with
  `PREVIOUS_CYCLE_METRICS_VALID` and, when clear,
  `previous_current_delivery_ms` must be zero.
- Relationships that are suspicious but structurally valid are retained for
  diagnosis rather than rejected as malformed.

The receiver processes a packet in this order:

1. Let the SX1262 discard invalid PHY headers and payload CRCs; send no response.
2. Parse only the fixed clear header, enforce the protocol maximum length and
   look up `node_id`. An unknown, revoked or unprovisioned node receives no
   response.
3. Authenticate and decrypt with AES-CCM using the received `control` and
   `domain` as AAD. Authentication failure receives no response.
4. Require authenticated `control = 0x20`; otherwise select
   `REJECTED_UNSUPPORTED`.
5. Accept the two uplink reading domains for parsing. An authenticated unknown
   domain selects `REJECTED_UNSUPPORTED`. A downlink ACK domain received by the
   receiver is logged as reflected or replayed traffic and receives no response.
6. Require the exact 28-byte reading body and the structural encoding rules
   above; otherwise select `REJECTED_MALFORMED`.
7. Atomically reserve space in the bounded in-memory queue. If unavailable,
   select `RETRY_LATER`.
8. Enqueue the reading, then select `ACCEPTED`.
9. Send the selected authenticated ACK only if the receiver TX-airtime budget
   permits it. Lack of ACK budget does not undo validation or queue insertion.

An in-memory map from `(node_id, sample_id)` to accepted or queued contents is a
proposed optimization, not a pilot protocol requirement. When present, a
matching authenticated duplicate selects `ACCEPTED` without another queue
insertion, while conflicting contents select `REJECTED_MALFORMED`. Without the
map, the receiver may enqueue a duplicate and rely on idempotent asynchronous
storage keyed by `(node_id, sample_id)`.

### Receiver TX-airtime budget

Under the pilot's assumed 1% operating regime, the receiver permits at most 36
charged TX seconds in any rolling 3,600-second window. Every response is charged
its estimated time-on-air plus a 10% safety margin. The ACK is sent only if its
charge keeps the total within 36 seconds.

The ledger covers every receiver transmission and should survive receiver
software restarts. If its recent history is unavailable, the receiver must use a
conservative no-transmit interval. When budget is unavailable, a valid reading
is still processed and queued but receives no ACK; a later copy can then be
acknowledged.

## Pilot transmission policy

The pilot sends one reading per packet. After each `TX_DONE`, the node switches
to continuous RX and schedules:

```text
retry_interval = 500 ms + uniform_random(100 ms, 500 ms)
retry_at       = TX_DONE + retry_interval
```

The interval is therefore uniformly distributed from 600 to 1,000 ms. A valid
authenticated ACK cancels `retry_at`. Otherwise, when `retry_at` is reached, the
node transmits the identical packet again if both global limits permit it. An
invalid ACK is logged and ignored while RX remains open. This pilot does not
adapt the interval from measured latency.

Each wake's radio phase has two limits:

```text
node charged-TX budget   8 seconds
radio-cycle deadline    30 seconds
```

The 8-second allocation assumes the current 15-minute wake interval and a 1%
duty-cycle operating regime: four scheduled wakes use at most 32 of the 36
available TX seconds per hour. It is an additional pilot limit, not a
replacement for the regulatory airtime accounting required by the final RF
configuration. The 30-second deadline uses the same application-start reference
as `run_ms`.

Reading and ACK time-on-air are deterministic from packet length and the RF
parameters and are calculated or validated with the Semtech time-on-air model.

The wake sequence is:

1. Attempt the current reading.
2. On `ACCEPTED`, remove its stored copy, if any, and start draining the backlog
   from the most recent reading.
3. Continue sending backlog readings until it is empty or either wake limit
   prevents another complete attempt.
4. On `RETRY_LATER`, stop all transmission for this wake. Retain the reading.
5. On a permanent rejection, quarantine the reading for diagnosis and do not
   send it unchanged again. A rejected current reading ends the wake without
   backlog processing; a rejected backlog reading does not prevent trying the
   next backlog entry while both budgets allow it.
6. On silence at `retry_at`, retain the reading and make another attempt if both
   limits allow it.
7. At the radio-cycle deadline, or when no further attempt can fit, stop radio
   activity and finalize the diagnostic and backlog state.
8. Flush persistent logs and state, then enter deep sleep. This final work may
   extend the total awake duration beyond 30 seconds.

There is no protocol-level backlog maximum or expiration policy during the
pilot. Storage must be provisioned for the worst-case pilot duration. A stored
reading is removed from the normal backlog only after authenticated `ACCEPTED`;
permanently rejected readings are retained separately for diagnosis.

## Pilot diagnostic logs

The pilot assumes enough local storage for append-only diagnostic logs. Log
records contain no node key or group master key.

### Node delivery log

The node records each `(sample_id, domain)` delivery episode for a wake when at
least one attempt was unanswered, the final result was not `ACCEPTED`, or a
previously failed backlog reading was finally accepted. A current reading
accepted on its first attempt need not produce a diagnostic record.

Each record contains:

```text
log_sequence
sample_id
domain
plaintext_body                 28 bytes
transmitted_frame              50 bytes
attempt_count                  relative to this wake cycle
attempt_tx_offsets_ms[]        relative to application start
final_result
invalid_ack_frames[]           received length, bytes and offset
```

`final_result` is one of:

```text
ACCEPTED
RETRY_LATER
REJECTED_UNSUPPORTED
REJECTED_MALFORMED
NO_ACK_RADIO_DEADLINE
NO_ACK_AIRTIME_LIMIT
LOCAL_RADIO_ERROR
```

All attempts within one `(node_id, sample_id, domain)` use identical frame
bytes, so the 50-byte transmitted frame is stored once. A PHY-valid ACK that
fails protocol parsing or authentication is retained in
`invalid_ack_frames`; an ACK discarded by the modem before delivery to the MCU
cannot be retained.

Before the first attempt, the node persists a small delivery-started marker.
It finalizes the diagnostic record when the delivery reaches a terminal ACK
result or when the radio phase ends. An unresolved marker after reboot records
that the previous wake ended before final logging. Log flushing occurs before
deep sleep and is outside the 30-second radio-cycle deadline.

### Receiver packet log

The receiver records every packet occurrence delivered by the radio. The
reception timestamps are captured at `RX_DONE`, rather than when the record is
written.

Each record contains:

```text
received_at_utc
received_at_monotonic_us
received_frame_length
received_frame
claimed_node_id
claimed_sample_id
claimed_domain
header_authenticated
rssi
snr
processing_result
duplicate_status
ack_selected
ack_tx_result
ack_frame
```

The clear-header identity fields are untrusted claims unless
`header_authenticated` is true. `processing_result` is one of:

```text
ACCEPTED
RETRY_LATER_QUEUE_FULL
REJECTED_UNSUPPORTED_CONTROL
REJECTED_UNSUPPORTED_DOMAIN
REJECTED_MALFORMED_LENGTH
REJECTED_MALFORMED_BODY
UNKNOWN_NODE
AUTHENTICATION_FAILED
WRONG_DIRECTION
RADIO_ERROR
```

`duplicate_status` is one of:

```text
NOT_CHECKED
FIRST_SEEN
DUPLICATE_SAME_CONTENT
DUPLICATE_CONFLICT
```

Duplicate status is trusted only for an authenticated packet. A receiver
without the proposed in-memory accepted/queued map uses `NOT_CHECKED`; duplicate
relationships can then be reconstructed from the append-only log.

`ack_selected` is either none or one of the four ACK domains.
`ack_tx_result` independently records what happened after selection:

```text
NOT_APPLICABLE
SUPPRESSED_AIRTIME_BUDGET
SET_TX_FAILED
TX_TIMEOUT
TX_DONE
```

When an ACK is constructed, its exact 23-byte frame is stored in `ack_frame`
whether or not transmission succeeds. Separating `processing_result`,
`ack_selected` and `ack_tx_result` preserves cases such as an accepted reading
whose ACK was suppressed by the receiver airtime budget.

A PHY-header, payload-CRC or radio failure may not expose a usable frame or
identity. The receiver stores it as a separate radio-event record containing
the UTC and monotonic timestamps, IRQ/error state, and RSSI/SNR when available;
identity and frame fields are absent.

## Deferred delivery approaches

These are possible successors to the one-reading-per-packet pilot, not current
protocol decisions.

### Atomic reading batches

- Put at most five readings in one encrypted packet to share the PHY framing,
  AAD, CCM tag and ACK overhead.
- Keep `sample_id` in each reading and add a persistent `packet_id` to the outer
  header and CCM nonce. Retain distinct uplink and ACK domains.
- The receiver authenticates and validates the entire batch, deduplicates its
  readings and reserves queue space for every new reading before inserting any.
- Return the existing one-byte `ACCEPTED` only when the complete batch is
  accepted atomically; otherwise accept none and return one packet-level error.
- A lost batch requires sending the complete batch again, so its airtime saving
  must be weighed against the higher cost of losing a longer packet.

### Selective bitmap synchronization

- Send a configurable window of readings without opening an ACK wait after each
  one, while retaining every reading locally.
- Periodically send a synchronization request. The receiver replies with a
  `base_sample_id` and bitmap indicating which consecutive sample IDs it has.
- Remove only bitmap-confirmed readings and send the missing readings again.
- This can greatly reduce receive-window energy but requires persistent window
  state, wraparound rules, receiver-restart recovery and bounded synchronization
  intervals.

## Timestamp reconstruction

The logical timestamp of a reading is the node application-start time of the
cycle in which its sensors were read. All sensor fields share it. Their actual
acquisition occurs between application start and reading finalization, as
bounded by `run_ms`; this within-cycle difference is intentionally ignored
because the pilot requires only minute-level precision.

### Direct anchors

The earliest authenticated `CURRENT_READING_UPLINK` reception for a sample is a
direct receiver-time anchor, whether or not the node received its ACK. Let:

```text
R     = receiver UTC timestamp captured at RX_DONE
Tair  = reading airtime, 97.536 ms
A     = node application-start time
```

Because the receiver cannot know whether earlier attempts were lost, the
30-second radio-cycle deadline gives:

```text
A_min = R - 30,000 ms
A_max = R - Tair - run_ms
A_est = midpoint(A_min, A_max)
```

The receiver persists `A_est` at second precision and rounds only for
minute-level presentation. Each authenticated current reading creates an
independent direct anchor; it is not extrapolated from the previous anchor.

### Extrapolation

The pilot schedule is:

```text
next_application_start =
    previous_application_start
    + previous_awake_ms
    + 900,000 ms deep sleep
    + boot delay
```

Boot delay and deep-sleep timer error are treated as zero by the pilot
estimator. For consecutive readings, the backward rule is therefore:

```text
timestamp[i - 1] =
    timestamp[i]
    - previous_awake_ms[i]
    - 900,000 ms
```

The equivalent forward rule is:

```text
timestamp[i] =
    timestamp[i - 1]
    + previous_awake_ms[i]
    + 900,000 ms
```

`previous_awake_ms[i]` already includes the previous cycle's uplinks, retries,
ACK waiting, backlog work and final logging. ACK airtime and processing time
must not be added separately.

Backward extrapolation may cross from sample `i` to sample `i - 1` only when:

- their `sample_id` values are consecutive;
- sample `i` has `DEEP_SLEEP_BOOT = 1`;
- sample `i` has `PREVIOUS_CYCLE_METRICS_VALID = 1`; and
- both samples belong to the same identity lifetime.

Otherwise the chain stops. A reading with `DEEP_SLEEP_BOOT = 0` may itself be
timestamped from a newer anchor, but the receiver must not cross from it to its
predecessor because the intervening reset or power-off duration is unknown.

Once an estimated timestamp is written to non-volatile storage, it is
immutable. The receiver also stores:

```text
timestamp_source    DIRECT or EXTRAPOLATED
anchor_sample_id    sample that supplied the direct anchor
```

An extrapolation-hop count is not stored. It can be reconstructed from ordered
sample IDs, packet domains, continuity flags and receiver packet logs.
`PREVIOUS_CURRENT_ACCEPTED` is node-side evidence that the preceding current
reading received an ACK; it is not by itself proof that the receiver lacks or
has a direct anchor because the ACK may have been lost.

The ignored boot and sleep-clock errors can accumulate across long chains.
Pilot timestamps remain best-effort in that case; the retained anchor and
packet logs allow the accumulated error to be measured before defining a
production uncertainty policy.

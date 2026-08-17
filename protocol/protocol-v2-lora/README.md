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

An identity lifetime is the period during which a node uses one `node_id`, its
corresponding `node_key`, one monotonic transport `message_id` sequence and one
monotonic application `sample_id` sequence. There is no explicit epoch field.
Replacing both `node_id` and `node_key` starts a new identity lifetime.

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

The receiver runtime treats `receiver-group.json` as operator-controlled,
read-only configuration distinct from receiver-owned runtime state. Its sole
disk-owning thread loads the file before radio operation, enforces the same
strict schema and secret-file protections as the provisioning tools, and
publishes an immutable configuration snapshot from which the communication
thread constructs its in-memory authentication map. Missing or invalid
configuration prevents normal receiver startup. The pilot applies
configuration changes by restart rather than hot reload.

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
message_id    4 bytes
ciphertext    N bytes
CCM tag       8 bytes
```

The fixed clear-header offsets are `control` 0, `domain` 1, `node_id` 2-9 and
little-endian `message_id` 10-13. In a reading frame the 32-byte ciphertext is
at offsets 14-45 and the tag is at 46-53. In an ACK frame the encrypted
one-byte status is at offset 14 and the tag is at 15-22.

- AES-CCM-128 provides encryption and authentication.
- The clear header is authenticated as associated data:

  ```text
  AAD = control || domain || node_id || message_id
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
  the same `(node_id, message_id, domain)` nonce.

### Byte order and serialization

- All multi-byte integers, including `message_id`, `sample_id` and reading fields, use
  little-endian byte order.
- Signed integers use two's-complement representation.
- `node_id` is an opaque sequence of eight bytes, not a numeric field requiring
  byte-order conversion.
- Implementations explicitly encode and decode fields at their defined offsets.
  They must not transmit a native C structure or include compiler padding.

## Reading payload

The encrypted application body for a reading is 32 bytes:

| Offset | Field | Type | Meaning |
|---:|---|---|---|
| 0 | `sample_id` | `u32` | Persistent application-reading ID used for reading/wake continuity and timestamp reconstruction. |
| 4 | `run_ms` | `u16` | Milliseconds from application start until the current reading body is finalized, immediately before persistence and frame construction. |
| 6 | `soil_0_mv` | `u16` | Soil-sensor channel 0 output in millivolts. |
| 8 | `soil_1_mv` | `u16` | Soil-sensor channel 1 output in millivolts. |
| 10 | `soil_temp_0_centi_c` | `i16` | Soil-temperature channel 0 in 0.01 degrees Celsius. |
| 12 | `soil_temp_1_centi_c` | `i16` | Soil-temperature channel 1 in 0.01 degrees Celsius. |
| 14 | `enclosure_centi_c` | `i16` | Enclosure temperature in 0.01 degrees Celsius. |
| 16 | `enclosure_pressure_pa` | `u32` | Enclosure pressure in pascals. |
| 20 | `enclosure_humidity_centi_pct` | `u16` | Enclosure relative humidity in 0.01 percent. |
| 22 | `reset_reason` | `u8` | Reason for the current boot, using the pilot mapping below. |
| 23 | `previous_current_tx_attempts` | `u8` | All transmission attempts for the previous cycle's current reading, including the first. |
| 24 | `previous_awake_ms` | `u16` | Previous cycle's total awake time, including transmission and ACK waiting. |
| 26 | `previous_current_delivery_ms` | `u16` | Time immediately before the previous current reading's first `SetTx` until its authenticated `ACCEPTED`. |
| 28 | `previous_cycle_tx_attempts` | `u8` | All current and backlog reading transmissions in the previous wake. |
| 29 | `previous_cycle_accepted_readings` | `u8` | Distinct readings for which the node received authenticated `ACCEPTED` in the previous wake. |
| 30 | `flags` | `u16` | Validity and diagnostic bitmap. |

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

This reading makes the SX1262 payload `14 + 32 + 8 = 54` bytes, excluding the
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
- Use direction-specific IQ polarity while retaining that sync word in both
  directions:

  | Direction | Domains | IQ polarity |
  |---|---|---|
  | Node to receiver | `CURRENT_READING_UPLINK`, `BACKLOG_READING_UPLINK` | Normal |
  | Receiver to node | All ACK downlink domains | Inverted |

  After uplink `TX_DONE`, a node configures inverted-IQ RX before waiting for
  its ACK. The receiver normally listens with normal IQ; it configures
  inverted-IQ TX for an ACK and returns to normal-IQ RX afterward.
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

IQ polarity must likewise match the direction at both ends. Direction-specific
polarity prevents a node waiting for a downlink from normally demodulating
another node's uplink. It does not identify the destination: a node can still
receive a downlink for another node and must reject it using the authenticated
`node_id`, `message_id` and ACK semantics. Opposite-IQ transmissions still
occupy the same RF channel and can interfere when they overlap. IQ polarity
adds no packet bytes and does not change airtime.

### Pilot airtime constants

The fixed pilot profile above gives a symbol time of 1.024 ms. Exact airtime
and the conservative charge applied to the corresponding transmitter's rolling
budget are:

| Packet | SX1262 payload | On-air symbols | Airtime | Charged airtime |
|---|---:|---:|---:|---:|
| Reading | 54 bytes | 100.25 | 102.656 ms | 112.922 ms |
| ACK | 23 bytes | 60.25 | 61.696 ms | 67.866 ms |

Charged airtime is `ceil(actual_airtime_us * 1.10)` and is maintained in integer
microseconds. The 10% margin is internal conservative accounting; it does not
represent additional RF transmission. Every attempted transmission is charged,
irrespective of reception or authentication outcome.

## CCM nonce

```text
node_id       8 bytes
message_id    4 bytes
domain        1 byte
             --------
             13 bytes
```

`domain` separates directions and logical packet types. Under one node key,
each `(node_id, message_id, domain)` combination identifies at most one logical
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

A reading starts in a newly constructed `CURRENT_READING_UPLINK` logical
message. If it remains stored locally and is later converted to backlog, the
node allocates a new `message_id`, persists the exact authenticated
`BACKLOG_READING_UPLINK` frame and only then transmits it. Repeated RF attempts
for either logical message reuse its exact `message_id`, domain, AAD,
ciphertext, tag and complete frame bytes. Later-wake backlog retries load those
persisted bytes rather than reconstructing the frame.

## Counters

- `message_id` is an unsigned persistent monotonic `u32` counter scoped to one
  node identity/key. It is claimed and durably advanced before every newly
  constructed logical uplink message, not before each RF attempt.
- A `message_id` may skip but must never be reused while the same node key is
  active. Losing or exhausting the counter requires a new node identity/key.
  Failure to claim and commit it prevents frame construction and transmission.
- `sample_id` remains the persistent application-reading and wake-continuity
  counter. It does not identify RF retry episodes and is not part of the nonce.

## ACK behaviour

- ACKs echo the pending uplink's `message_id`; they do not carry `sample_id`.
  Their nonce uses that message ID and the domain corresponding to the status.
- Repeating an ACK with the same outcome reuses identical frame bytes. Different
  outcomes use their existing distinct ACK domains and therefore distinct
  nonces under the same node key and `message_id`.
- The ACK encrypted application body is a one-byte `status` value:

  | Domain | Status | Meaning |
  |---|---:|---|
  | `ACK_ACCEPTED_DOWNLINK` | `0` | `ACCEPTED` |
  | `ACK_RETRY_LATER_DOWNLINK` | `1` | `RETRY_LATER` |
  | `ACK_REJECTED_UNSUPPORTED_DOWNLINK` | `2` | `REJECTED_UNSUPPORTED` |
  | `ACK_REJECTED_MALFORMED_DOWNLINK` | `3` | `REJECTED_MALFORMED` |

- The node accepts an ACK only after CCM authentication and only when its domain
  and decrypted status match the table. A mismatch is an invalid ACK.

- A reading is accepted after it has been authenticated, decoded and checked,
  and after both its application entity and initial packet-occurrence profile
  have been inserted atomically into the receiver's bounded in-memory queue.
- An ACK does not guarantee durable disk persistence. The receiver may persist
  an accepted reading asynchronously after sending the ACK.
- A reading may be timestamped after being persisted to disk.
- A recognized authenticated duplicate receives `ACCEPTED` again only after
  the receiver admits the duplicate occurrence's initial profile.
- Failure to admit either required item for a new reading, or the occurrence
  profile for a duplicate or other authenticated response, produces
  `RETRY_LATER`. Partial admission is forbidden.
- Unsupported and malformed authenticated readings receive their corresponding
  permanent rejection after their occurrence profile is admitted. If profiling
  admission fails, they receive `RETRY_LATER` instead. The node retains a
  permanent rejection for diagnosis but does not keep sending the rejected
  message unchanged.
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
2. Parse only the fixed clear header, expose its `node_id`, `message_id` and
   `domain` as untrusted claims, enforce the protocol maximum length and look
   up `node_id`. An unknown, revoked or unprovisioned node receives no response.
3. Authenticate and decrypt with AES-CCM using the exact received 14-byte clear
   header as AAD and `node_id || message_id || domain` as the nonce.
   Authentication failure receives no response; `sample_id` is not available
   at this stage.
4. Require authenticated `control = 0x20`; otherwise select
   `REJECTED_UNSUPPORTED`.
5. Accept the two uplink reading domains for parsing. An authenticated unknown
   domain selects `REJECTED_UNSUPPORTED`. A downlink ACK domain received by the
   receiver is logged as reflected or replayed traffic and receives no response.
6. Require the exact 32-byte reading body, decode authenticated `sample_id`, and
   enforce the structural encoding rules above; otherwise select
   `REJECTED_MALFORMED`.
7. Classify authenticated transport retransmissions by `(node_id, message_id)`
   separately from duplicate readings by `(node_id, sample_id)`. Reuse of one
   message ID with different authenticated frame bytes is malformed.
8. Atomically reserve space in the bounded in-memory queue. If unavailable,
   select `RETRY_LATER`.
9. Enqueue the reading, then select `ACCEPTED`.
10. Construct an authenticated ACK that echoes the uplink `message_id`, and send
    it only if the receiver TX-airtime budget permits it. Lack of ACK budget
    does not undo validation or queue insertion.

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
adapt the interval from measured latency. An authenticated ACK with an
`RX_DONE` timestamp at or before `retry_at` wins the boundary; an ACK completed
after it does not prevent the retry.

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

The node represents every entered delivery episode with two append-only events.
They carry the wake's current `cycle_sample_id`, the delivered reading's
`sample_id`, its transport `message_id` and its domain. Transport identity is
`(node_id, message_id)`; the cycle and sample IDs preserve wake and reading
context without changing that identity.

Before the first call to the radio's `transmit_uplink`, `node_core` appends:

```text
event_type                       DELIVERY_STARTED
cycle_sample_id
sample_id
message_id
domain
start_offset_ms                relative to application start
```

When the episode reaches a terminal ACK result, exhausted limit or local error,
`node_core` appends:

```text
event_type                       DELIVERY_FINISHED
cycle_sample_id
sample_id
message_id
domain
attempt_count                  relative to this wake cycle
final_result
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

The start and finish bracket the complete delivery episode, including all
retries; they do not bracket individual attempts. `DELIVERY_STARTED` means that
the controller entered delivery, not that the SX1262 necessarily executed
`SetTx`. Both events are durable when their append operation succeeds. Logging
failure never blocks transmission or deep sleep.

An unmatched start after reboot means that no durable finish was recorded. It
may result from reset before TX, during TX or RX, during retry waiting, or while
persisting the finish; it is not proof that reset occurred during RF emission.
Ordinary diagnostic records may remain buffered until final `sync_all`, which
occurs before deep sleep and outside the 30-second radio-cycle deadline.

### Receiver packet log

The receiver attempts to record every packet occurrence delivered by the
radio. Packet-occurrence profiling has the same persistence priority as
application measurements. The reception timestamps are captured at `RX_DONE`,
rather than when the record is written.

For a new accepted reading, admission of the application entity and the
initial packet-occurrence profile is one atomic queue operation: both are
inserted or neither is inserted. Any authenticated packet eligible for a
response must have its initial occurrence profile admitted before TX. If the
required admission fails, the receiver selects
`ACK_RETRY_LATER_DOWNLINK`; packets that cannot be authenticated remain silent.

Queue exhaustion can therefore prevent the receiver from retaining the very
profile that reports the exhaustion. This is an explicit pilot exception to
the per-occurrence recording requirement. Each failed initial-profile or
TX-completion admission increments the bounded
`message_profiling_admission_failures` counter, which is reported later through
a non-recursive persistence or diagnostic path so that gaps remain observable.

Each record contains:

```text
receiver_boot_id
occurrence_sequence
received_at_utc
received_at_monotonic_us
system_time_quality
rtc_health
persist_queue_used_bytes_before_admission
persist_queue_capacity_bytes
received_frame_length
received_frame
claimed_node_id
claimed_message_id
claimed_domain
header_authenticated
decoded_sample_id
rssi
snr
processing_result
transport_duplicate_status
reading_duplicate_status
ack_selected
ack_tx_result
ack_frame
t1_handler_started_monotonic_us
t2_packet_copied_monotonic_us
t3_authentication_completed_monotonic_us
t4_set_tx_attempted_monotonic_us
t5_tx_done_monotonic_us
t6_set_rx_issued_monotonic_us
```

`receiver_boot_id` and the monotonically increasing per-boot
`occurrence_sequence` together identify one logical profiling row.
`received_at_monotonic_us` is the former `T0` kernel-recorded DIO1 timestamp.
The queue occupancy and capacity use the same byte-accounting rules as the
bounded persistence queue and are sampled immediately before admission. UTC is
null while receiver system time is untrusted; the monotonic timestamp, time
quality and RTC health are still recorded.

`system_time_quality` is one of:

```text
NETWORK_SYNCED
RTC_HOLDOVER
UNTRUSTED
```

`rtc_health` is recorded independently and is one of:

```text
PRESENT
MISSING
INVALID
```

`received_at_utc` is present for `NETWORK_SYNCED` and `RTC_HOLDOVER`, and null
for `UNTRUSTED`. This permits trusted network time to coexist with a missing or
invalid RTC without discarding the reception timestamp.

The clear-header identity fields are untrusted claims unless
`header_authenticated` is true. `decoded_sample_id` is null unless the frame
authenticated and the exact reading body decoded successfully.
`processing_result` is one of:

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

`transport_duplicate_status` is one of:

```text
NOT_CHECKED
FIRST_SEEN
RETRANSMISSION_SAME_FRAME
MESSAGE_ID_CONFLICT
```

It describes occurrences keyed by `(node_id, message_id)`. A retransmission is
the same logical transport message and exact authenticated frame; it is not a
second application reading.

`reading_duplicate_status` is one of:

```text
NOT_CHECKED
FIRST_SEEN
DUPLICATE_SAME_CONTENT
DUPLICATE_CONFLICT
```

Transport duplicate status is trusted only after authentication. Reading
duplicate status additionally requires successful reading-body decoding. A
receiver without the corresponding in-memory maps uses `NOT_CHECKED`; both
relationships can then be reconstructed from the append-only log.

`ack_selected` is either none or one of the four ACK domains. The exact ACK
frame is constructed before initial-profile admission and therefore remains
known even if later transmission is suppressed or fails.

One logical profiling row is persisted in two stages when an ACK is selected.
The immutable initial record is queued before TX with `ack_tx_result` set to
`PENDING` and TX-dependent timestamps absent. After TX, suppression or bounded
radio recovery reaches a terminal outcome, an immutable completion command for
the same `(receiver_boot_id, occurrence_sequence)` supplies the final result
and `T4` through `T6`. The persistence thread applies that command as an update
to the existing row. A packet that cannot receive an ACK may be returned to RX
first and then queued once as a complete row with `ack_tx_result` set to
`NOT_APPLICABLE`.

`ack_tx_result` independently records what happened after selection:

```text
PENDING
NOT_APPLICABLE
SUPPRESSED_AIRTIME_BUDGET
SET_TX_FAILED
TX_TIMEOUT
TX_DONE
UNKNOWN_INTERRUPTED
```

`PENDING` is a transitional persisted value, not evidence that transmission
started. A pending row belonging to an earlier receiver boot is interpreted as
`UNKNOWN_INTERRUPTED`: durable evidence cannot establish whether the ACK was
transmitted. A completion-admission failure may leave a row pending during the
current boot and increments `message_profiling_admission_failures`.

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
- Keep `sample_id` in each reading and use the existing persistent `message_id`
  in the outer header and CCM nonce. Retain distinct uplink and ACK domains.
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
Tair  = reading airtime, 102.656 ms
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

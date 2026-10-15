# Firmware architecture

The ESP32-C6 node samples sensors, persists a reading, delivers it over LoRa v2,
drains recent backlog when delivery succeeds, and sleeps. Start with
[README.md](README.md) for commands and navigation, [HARDWARE.md](HARDWARE.md) for
wiring, and the [protocol](../protocol/protocol-v2-lora/README.md) for wire behavior.

## Components and ownership

| Owner | Responsibility |
|---|---|
| [app_main](main/app_main.c) | Compose private identity, ESP platform ports and retained RTC record |
| [node_core](components/node_core/include/node_core.h) | Wake ordering, reading construction, retries, ACK validation, budgets and metrics |
| [node_persistence](components/node_persistence/include/node_persistence.h) | NVS counters, LittleFS records, recovery, backlog bindings and log retention |
| [node_sensors](components/node_sensors/include/node_sensors.h) | Sensor acquisition, independent validity and switched-rail cleanup |
| [sx1262_radio](components/sx1262_radio/include/sx1262_radio.h) | Radio commands, IRQs, fixed PHY, absolute deadlines and TX/RX outcomes |
| [node_platform_esp](components/node_platform_esp/include/node_platform_esp.h) | Monotonic clock, retry randomness, reset reason and terminal sleep/restart |
| [protocol_v2_lora](components/protocol_v2_lora/) | Generated codec and authenticated frames |

Core calls semantic operations, not GPIO, NVS or radio registers. Components own
private state and link to ESP-IDF backends in production or deterministic fakes
in [host tests](tests/README.md). Clock/randomness/system callbacks are borrowed
for one wake. Identity is passed into core so its reusable code never includes
the generated secret header. Ordinary BSS resets wake-local component state;
no public singleton reset API is needed for production or tests.

## One wake

1. Capture application-start time and reset reason, then copy and immediately
   invalidate retained RTC metrics. Commit the next sample ID before activating
   sensors or radio. Failure skips acquisition/transmission and proceeds to cleanup.
2. Acquire sensor groups independently. Validity bits distinguish a valid zero
   from failure; the enclosure temperature/pressure/humidity group is atomic.
   The switched soil/DS rail is released before independent BME acquisition.
3. Construct the reading with available sensor values and valid previous-cycle
   metrics. Persist its canonical body. If append fails, attempt current delivery
   from RAM and suppress backlog access for this wake: an ambiguous append could
   otherwise cause the same reading to be selected again immediately.
4. Commit a message ID and seal the current-domain frame. The first transmit
   lazily initializes the radio. Reuse the exact authenticated frame for retries.
5. On current acceptance, remove its persisted copy before selecting backlog.
   Removal failure stops radio work. Drain newest pending readings while limits
   allow; an unbound reading receives a new message ID and durable backlog-frame
   binding before TX. Later wakes reuse a bound frame verbatim.
6. Release sensor power, put touched radio hardware into cold sleep, write
   remaining diagnostics and perform one final persistence sync. Cleanup failures
   are best-effort diagnostics and do not prevent terminal MCU sleep.
7. Measure total awake time and commit representable RTC metrics, writing their
   marker last. Enter the configured relative deep sleep, normally 900 seconds.

A sample ID names an application reading; a message ID names one logical uplink.
Converting an old current reading to backlog creates a new message/domain binding.
Both counters commit their successors before use. Gaps are safe; reuse within
one identity/key lifetime is not. Firmware cannot distinguish erased counters
from new provisioning: counter loss, rollback or exhaustion requires a new
identity/key and complete node-local state reset. Follow
[deployment](deploy/README.md), preserving valid counters during ordinary rebuilds.

## Delivery and failure decisions

Each current/backlog delivery permits two attempts. After first TX_DONE, core
selects one 100–400 ms jitter and listens continuously for 300 ms plus that jitter.
An eligible ACK ends delivery immediately. Silence can trigger one identical-frame
retry, followed by a 300 ms receive interval. Invalid ACKs never extend a deadline.
Captured IRQ time decides timeliness, including equality, even if task processing
is delayed. A local radio error ends delivery without an in-wake retry.

All deliveries share an eight-second charged-TX budget and a 30-second deadline
from application start, including acquisition time. Core charges modeled airtime
plus 10% only when SetTx started or its effect is uncertain. Radio setup/watchdog
margins affect wall-clock admission, not RF airtime; the driver checks admission
again immediately before SetTx. Cleanup and final persistence run after the radio
deadline. Constants and boundary logic live in core/radio source.

An authenticated ACCEPTED result permits removal and progress. RETRY_LATER,
exhausted retries or local failure retain the reading and stop radio work.
Permanent current rejection stops the radio phase because it may indicate a
systematic problem. Permanent backlog rejection can continue after successful
removal. In both cases core first attempts quarantine, then removes a known
persisted rejected entry even if quarantine failed, avoiding repeated obstruction.
Reset between those independent operations can leave duplicate quarantine data.

Delivery start/finish records are durable boundaries around the controller
operation. A start does not prove TX, and an unmatched start leaves the outcome
unknown. Timing and PHY-rejection observations accumulate in RAM during delivery
and reach persistence afterward, avoiding storage work inside ACK windows.
Logging failure never changes an authenticated result or recursively logs itself.

## Persistence and recovery

NVS and LittleFS initialize independently on first use and cache failures for the
wake. Identity claims, pending transitions, quarantine and delivery events are
synchronized before success. Ordinary diagnostics can remain buffered until the
single final sync, which skips unused backends and closes owned handles without
unmounting LittleFS.

The private [record codec](components/node_persistence/private_include/node_persistence_record.h)
owns canonical framing, version/type assignments and payload encodings. Generated
protocol code owns reading/frame layouts. Records use bounded lengths and CRC
before decoding; CRC protects stored plaintext that will later be authenticated.

Every append validates the tail so torn bytes cannot be buried. Bounded recovery
may remove one unusable suffix and then reports its loss without executing the
requested operation. An unprovable boundary preserves all bytes. Pending
compaction retains complete recent reading/binding items, syncs a temporary copy,
then renames it over the authoritative log. A leftover temporary copy is never
promoted as guessed recovery. Production never formats storage automatically;
[erase_storage](maintenance/erase_storage/README.md) is an explicit maintenance tool.

## Hardware and platform boundaries

Sensor identities and pins are build configuration. DS channels follow configured
ROMs, not enumeration order. The external P-MOSFET pull-up defaults the rail off;
firmware releases its open-drain gate and the 1-Wire pad to avoid back-power.
BME280 remains powered, uses forced conversion, and verifies actual channel
settings, completion and sleep. Failure is latched for the wake with its original
status. Admission budgets reject late SDK results but cannot preempt them.

The radio owns SPI, BUSY/DIO1 and a fixed normal-IQ uplink/inverted-IQ downlink
profile. Core and its IRQ backend share `esp_timer_get_time()`; no UTC clock is
needed. The backend applies the Semtech TX-clamp workaround and checks command
status. Driver pins/licenses remain with the [Semtech vendor sources](components/sx1262_radio/vendor/semtech_sx126x_driver/README.cura.md)
and [Bosch source resolver](components/bosch_bme280/README.md).

Sleep is terminal to core, including when a host fake returns. If timer setup
fails, the ESP adapter logs to the console, waits without spinning and restarts;
it does not reopen persistence after final sync. Intentional restart attempts
sensor force-off and proceeds even if that attempt fails. The production main
task stack includes the nested LittleFS call path; its rationale is beside
[sdkconfig.defaults](sdkconfig.defaults).

## Current limits

- The charged-TX budget resets each wake. Normal sleep cadence supplies its
  hourly margin; unexpected resets can create additional budgets. The node
  does not independently enforce a rolling-hour airtime limit across resets.
- RTC metrics require a committed marker, deep-sleep reset, the immediately
  preceding sample ID and valid metric relationships. They have no CRC; a
  retained-memory error that preserves those checks can escape detection.
- Pending storage compacts; full quarantine/diagnostic/delivery logs reject new
  records without autonomous reclamation. Unrecoverable logs need an explicit
  physical recovery procedure. Current readings may still be sent from RAM.
- Sensor validity checks acquisition and representability, not plausibility.
  Finite BME budgets do not impose a universal deadline on all SDK or changing
  1-Wire behavior. Physical timing, power-loss and current observations have
  their own [test scope](tests/README.md#scope-of-observations).

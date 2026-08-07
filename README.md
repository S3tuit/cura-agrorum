# Cura Agrorum

Cura Agrorum is a low-cost field-monitoring project for collecting soil and
enclosure measurements from a remote sensor node. The current design uses an
ESP32-C6 node and a Raspberry Pi receiver connected over authenticated EU868
LoRa.

## Current goal

The project is working toward the one-week pilot described in
[`deployment/field_deployment_v2.md`](deployment/field_deployment_v2.md). The
pilot will use development hardware to:

- measure soil moisture and temperature at two depths;
- validate LoRa reliability, retries, RSSI and SNR in the intended field;
- verify receiver timestamping, offline retention and later forwarding; and
- measure operating-state energy to guide the battery, radio, antenna,
  enclosure and custom-PCB design.

This is a communications and measurement pilot, not yet a battery-life or
solar-autonomy test.

## Repository map

- `deployment/` — field plans; `field_deployment_v2.md` is the active target.
- `firmware/` — ESP-IDF node firmware, reusable components, maintenance apps,
  architecture notes and host/on-device tests.
- `protocol/` — the current LoRa v2 wire contract, schemas, generated codecs,
  provisioning tools and cross-language tests; v1 documents the earlier Wi-Fi
  protocol.
- `receiver/` — Python v2 codec and authenticated-frame building blocks for
  the Raspberry Pi receiver.
- `hardware/` — notes and measurements for candidate hardware.
- `logbook/` — soil characterization and moisture-sensor experiments.
- `server/` — legacy code retained temporarily and scheduled for removal.

The v2 protocol and the persistence, sensor and SX1262 firmware components are
implemented. Full node wake-cycle orchestration and the receiver application
remain in progress; see [`firmware/ARCHITECTURE.md`](firmware/ARCHITECTURE.md)
and [`firmware/TESTING.md`](firmware/TESTING.md) for the current contracts and
test status.

## Tests

Run native firmware host tests with `make test-host`. Protocol tests and their
dependencies are documented in
[`protocol/protocol-v2-lora/tests/README.md`](protocol/protocol-v2-lora/tests/README.md);
ESP32-C6 hardware-test commands are documented in
[`firmware/test_apps/on_device/README.md`](firmware/test_apps/on_device/README.md).

# Cura Agrorum

Cura Agrorum is a low-cost field-monitoring project for collecting soil and
enclosure measurements from a remote sensor node. The current design uses an
ESP32-C6 node and a Raspberry Pi receiver connected over authenticated EU868
LoRa.

This repository owns the production firmware, receiver, wire protocol, and
server code. Experiments and their firmware, acquisition tools, datasets,
analysis, findings, reference material, and deployment records live in the
separate
[`cura-agrorum-logbook`](https://github.com/S3tuit/cura-agrorum-logbook)
repository. A local checkout of that repository is expected at
`../cura-agrorum-logbook` so contributors and agents can inspect the evidence
behind production decisions.

## Current goal

The project is working toward the one-week pilot described in the logbook's
[`field-pilot-v2`](https://github.com/S3tuit/cura-agrorum-logbook/blob/main/deployments/field-pilot-v2/README.md)
record. The pilot will use development hardware to:

- measure soil moisture and temperature at two depths;
- validate LoRa reliability, retries, RSSI and SNR in the intended field;
- verify receiver timestamping, offline retention and later forwarding; and
- measure operating-state energy to guide the battery, radio, antenna,
  enclosure and custom-PCB design.

This is a communications and measurement pilot, not yet a battery-life or
solar-autonomy test.

## Repository map

- `firmware/` — ESP-IDF node firmware, reusable production components,
  architecture notes and host/on-device tests.
- `protocol/` — the current LoRa v2 wire contract, schemas, generated codecs,
  provisioning tools and cross-language tests; v1 documents the earlier Wi-Fi
  protocol.
- `receiver/` — Python v2 codec and authenticated-frame building blocks for
  the Raspberry Pi receiver.
- `server/` — legacy code retained temporarily and scheduled for removal.

The logbook may depend on public components exposed from
`firmware/components/`. Production code in this repository must not depend on
the logbook. During the repository split, old experiment, deployment, or
reference files may remain here temporarily until their migrated copies have
been reviewed; the logbook versions are the authoritative records.

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

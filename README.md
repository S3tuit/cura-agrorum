# Cura Agrorum

Cura Agrorum collects soil and enclosure measurements using an ESP32-C6 sensor
node and a Raspberry Pi receiver connected over authenticated EU868 LoRa.
This repository owns the production firmware, receiver, wire protocol and their
test procedures. Testing policy is in the firmware and receiver testing guides.

Experiments, datasets, analysis and deployment plans/results belong in
[`cura-agrorum-logbook`](https://github.com/S3tuit/cura-agrorum-logbook), normally
checked out at `../cura-agrorum-logbook`. The logbook may use public firmware
components; production code here must not depend on the logbook.

## Repository map

| Path | Responsibility |
| --- | --- |
| [`firmware/`](firmware/) | ESP-IDF node application, wake-cycle controller, persistence, sensors, radio and platform components. |
| [`receiver/`](receiver/README.md) | Raspberry Pi receiver application, radio/clock integration, authenticated ingestion and durable SQLite storage. |
| [`protocol/protocol-v2-lora/`](protocol/protocol-v2-lora/README.md) | LoRa wire contract, schemas, generated C/Python codecs, provisioning and cross-language tests. |
| [`tests/rf/`](tests/rf/README.md) | Joint node/receiver RF procedures, runners, host verification and offline capture analysis. |

## Where the rules live

Start with the documents for the component you are changing:

| Subject | Authoritative documents |
| --- | --- |
| Runtime behavior and component ownership | [Firmware architecture](firmware/ARCHITECTURE.md), [receiver architecture](receiver/ARCHITECTURE.md) |
| Component interfaces and stored data | [Firmware interfaces](firmware/INTERFACE.md), [receiver interfaces](receiver/INTERFACE.md) |
| Wire encoding and authentication | [LoRa protocol](protocol/protocol-v2-lora/README.md) |
| Receiver diagnostic definitions | [Diagnostic interface](receiver/INTERFACE_DIAGNOSTIC.md) |
| Test scope, procedures and coverage | [Firmware testing](firmware/TESTING.md), [receiver testing](receiver/TESTING.md), [joint RF verification](tests/rf/README.md) |

These documents own their rules; local READMEs explain navigation, setup and
editing instructions. Generated files implement their schema/generator inputs.

## Development and tests

Run host checks from the repository root after preparing the linked prerequisites:

| Scope | Entry point | Setup and details |
| --- | --- | --- |
| Native firmware | `make test-host` | [Firmware host tests](firmware/TESTING.md#philosophy-and-build) |
| Receiver | `make test-receiver-host` | [Python environment](receiver/README.md#python-setup) |
| RF tooling | `make test-rf-host` | [RF host checks](tests/rf/README.md#host-checks) |
| Protocol | `.venv/bin/python -m pytest protocol/protocol-v2-lora/tests` | [Protocol test prerequisites](protocol/protocol-v2-lora/tests/README.md#running) |

Hardware execution has separate fixture and device requirements; follow the
linked testing procedures. Install and configure production systems using the
[firmware deployment guide](firmware/deploy/README.md) and
[receiver deployment guide](receiver/deploy/README.md).

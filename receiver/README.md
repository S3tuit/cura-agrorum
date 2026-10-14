# Receiver

The Raspberry Pi receiver authenticates LoRa readings through an SX1262, sends
ACKs, and stores readings and operational history in SQLite.

Start with the [architecture](ARCHITECTURE.md) for ownership and design tradeoffs,
[hardware](HARDWARE.md) for the production schematic, [testing](tests/README.md)
for test commands, or [deployment](deploy/README.md) for installation and maintenance.
[Chrony setup](deploy/CHRONY.md) owns system time configuration.
The [LoRa protocol](../protocol/protocol-v2-lora/README.md) owns packet formats,
authentication, and node/receiver interoperability.

## Directory map

| Path | Responsibility |
| --- | --- |
| [`cura_receiver/`](cura_receiver/) | Handwritten runtime: `application.py` composes communicator and persistence; component modules document non-obvious behavior. |
| [`cura_receiver/ports/`](cura_receiver/ports/) and [`platform/`](cura_receiver/platform/) | Narrow device/OS capabilities and their Linux adapters. |
| [`schemas/`](schemas/) | Editable enum assignments, entity layouts, and relational mappings. |
| [`db/`](db/) | Handwritten `schema_source.sql`, generated `schema.sql`, and database maintenance instructions. |
| [`tools/`](tools/) | Generation, database initialization, packaging, and test controllers. |
| [`cura_receiver/generated/`](cura_receiver/generated/) | Checked-in generated Python projections; edit their sources and regenerate. |
| [`native/`](native/) | Production capability-limited RTC write helper. |
| [`tests/`](tests/) | Host tests, target-Pi integration tests, and shared support. |
| [`hardware/`](hardware/) | Component-specific physical tests with preparation and restoration beside their code. |
| [`test_apps/radio_peer/`](test_apps/radio_peer/) | Pi component peer for [joint RF tests](../tests/rf/README.md). |
| [`benchmarks/`](benchmarks/) | Reusable performance characterization tools. |

`receiver-group.json` is the development configuration. Installed deployments use
explicit private configuration and database paths. Schema versions and fingerprints
come from the generated sources; runtime requires an exact match and performs no
automatic migration. Follow [database maintenance](db/README.md) for a fresh schema
or restoration.

## Python setup

Receiver database access requires Python 3.12 or newer and SQLite support for
`SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE`. From the repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install \
  -r receiver/requirements-test.txt \
  -r protocol/protocol-v2-lora/requirements-test.txt \
  ./protocol/protocol-v2-lora/python
make test-receiver
```

The shared configuration/protocol loader is a separate installed package;
production does not depend on provisioning tools or test imports. Pytest uses the
current protocol source checkout. See [tests/README.md](tests/README.md) for focused
runs and target/component selection.

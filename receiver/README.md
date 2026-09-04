# Receiver

This directory contains the provisional Raspberry Pi receiver architecture,
interfaces, Python package, schema sources, generators, and tests.

## Documentation

- [`ARCHITECTURE.md`](ARCHITECTURE.md) defines component ownership, runtime
  behavior, persistence lifecycle, durability, and recovery policy.
- [`INTERFACE.md`](INTERFACE.md) defines shared receiver values, typed queue
  entities, persistence-control interfaces, and the SQLite contract.
- [`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md) defines diagnostic
  catalogues and fixed diagnostic contexts.
- [`TESTING.md`](TESTING.md) defines the pilot host and Raspberry Pi test
  suites, their framework, fixtures, safety boundaries, and deferred
  end-to-end RF coverage.

The protocol contract remains under
[`../protocol/protocol-v2-lora/`](../protocol/protocol-v2-lora/).

## Directory map

- [`cura_receiver/`](cura_receiver/) contains handwritten receiver Python code.
  `persist_queue.py`, `persist_queue_entities.py`, and
  `quarantine_evidence.py` implement the fixed object-reference ring, its
  immutable handoff values, and the poison-evidence codec respectively.
  Its [`generated/`](cura_receiver/generated/) subdirectory contains checked-in
  generated Python modules. `database_initializer.py` installs a fresh receiver
  database from the exact packaged schema without overwriting an existing one.
  [`ports/`](cura_receiver/ports/) defines narrow production capabilities, and
  [`platform/`](cura_receiver/platform/) contains their deployed Linux adapters.
- [`schemas/`](schemas/) contains machine-readable receiver sources of truth for
  stable enums and persisted entity layouts.
- [`db/`](db/) contains the handwritten SQLite schema input and generated
  `schema.sql`.
- [`tools/`](tools/) contains host-side receiver generation tools.
- [`tests/`](tests/) contains the deterministic host suite, target-Pi hardware
  suite, and test-only support. The generation/schema tests live under
  [`tests/host/`](tests/host/); [`tests/support/README.md`](tests/support/README.md)
  defines when shared fakes, builders, models and coordination helpers are added.
- `receiver-group.json` is the development receiver-group configuration.

Each subdirectory README explains only the files and editing rules local to
that directory. Follow links to the governing documents above for behavioral
and compatibility contracts rather than repeating those contracts in local
READMEs.

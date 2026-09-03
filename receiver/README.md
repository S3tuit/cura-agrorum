# Receiver

This directory contains the provisional Raspberry Pi receiver architecture,
interfaces, Python package, schema sources, generators, and tests.

## Documentation

- [`ARCHITECTURE.md`](ARCHITECTURE.md) defines component ownership, runtime
  behavior, persistence lifecycle, durability, and recovery policy.
- [`INTERFACE.md`](INTERFACE.md) defines shared receiver values, fixed entities,
  persistence-control interfaces, and the SQLite contract.
- [`INTERFACE_DIAGNOSTIC.md`](INTERFACE_DIAGNOSTIC.md) defines diagnostic
  catalogues and fixed diagnostic contexts.
- [`TESTING.md`](TESTING.md) defines the pilot host and Raspberry Pi test
  suites, their framework, fixtures, safety boundaries, and deferred
  end-to-end RF coverage.

The protocol contract remains under
[`../protocol/protocol-v2-lora/`](../protocol/protocol-v2-lora/).

## Directory map

- [`cura_receiver/`](cura_receiver/) contains handwritten receiver Python code.
  Its [`generated/`](cura_receiver/generated/) subdirectory contains checked-in
  generated Python modules. `database_initializer.py` installs a fresh receiver
  database from the exact packaged schema without overwriting an existing one.
- [`schemas/`](schemas/) contains machine-readable receiver sources of truth for
  stable enums and persisted entity layouts.
- [`db/`](db/) contains the handwritten SQLite schema input and generated
  `schema.sql`.
- [`tools/`](tools/) contains host-side receiver generation tools.
- [`tests/`](tests/) contains receiver-side generation and schema tests.
- `receiver-group.json` is the development receiver-group configuration.

Each subdirectory README explains only the files and editing rules local to
that directory. Follow links to the governing documents above for behavioral
and compatibility contracts rather than repeating those contracts in local
READMEs.

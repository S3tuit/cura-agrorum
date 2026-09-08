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
  `receiver_configuration.py` runs the shared protocol loader on its persistence
  owner, while `receiver_startup.py` creates process identities and commits
  lifecycle start rows. `sqlite_database.py` opens and validates existing
  databases, and `sqlite_repository.py` supplies explicit row operations within
  caller-owned transactions. `ordinary_persistence.py` adds caller-driven queue
  transactions, immutable health enrichment, exact replay, poison quarantine,
  and exact reconciliation; `reading_persistence.py` owns reading classification
  and canonical evidence checks. `persistence_recovery.py` owns the shared
  admission, retry and checkpoint policy. `persistence_worker.py` supplies the
  sole disk-owning thread, startup, queue/control scheduling and bounded final
  shutdown handoff. `persistence_control_channel.py` implements the four
  synchronous operations; handwritten state validation and transactions live
  in `communicator_state_persistence.py` and `persistence_control_operations.py`.
  `sqlite_transactions.py` remains the concrete SQLite transaction/checkpoint
  fault boundary. The communicator, live time policy and radio integration
  remain later components.
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

## Python setup

The database opener requires Python 3.12 or newer with SQLite
`SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE` support. The deployed Pi component tests
verify that capability and the required connection settings on its storage.
The shared configuration loader is a separate protocol-owned package under
`protocol/protocol-v2-lora/python`; installed receiver deployments must include
it, without needing the provisioning tools directory.

For a fresh development/test environment, from the repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install \
  -r receiver/requirements-test.txt \
  -r protocol/protocol-v2-lora/requirements-test.txt \
  ./protocol/protocol-v2-lora/python
make test-receiver
```

The protocol requirements include the cryptography dependency used by receiver
ingress. Pytest also adds the shared protocol source directory to its import
path so tests exercise the current checkout. Production imports the installed
`cura_protocol_v2_lora` package directly. Deployment supplies explicit private
configuration and database paths; the default configuration path is only the
development `receiver/receiver-group.json`.

The worker takes an already-created immutable `ReceiverInstanceStart` and
explicit database/configuration paths. Call `start()` before using its
`control` channel; `wait_started()` exposes immutable startup facts without a
database handle. The caller closes `queue`, waits for closed-and-drained within
its budget and requests the clean-stop marker after its own shutdown
preconditions hold. `request_stop(deadline_monotonic_us=...)` is the final
handoff: it closes submissions, drains within the remaining budget and attempts
checkpoint/close when eligible. It cannot preempt kernel I/O or manufacture a
clean marker.

The raw communicator-state envelope advances the pilot schema to epoch 10.
Existing epoch databases require the documented offline fresh-database
deployment boundary; there is no automatic migration or restoration.
Worker validation evidence is recorded under
[`tests/hardware/evidence/persistence_worker/`](tests/hardware/evidence/persistence_worker/README.md).

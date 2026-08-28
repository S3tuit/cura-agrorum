# Receiver database migrations

This directory contains the immutable, forward-only SQLite schema history for
the receiver. [`../INTERFACE.md`](../INTERFACE.md) defines the logical schemas,
identities, general enum values and persistence semantics, while
[`../INTERFACE_DIAGNOSTIC.md`](../INTERFACE_DIAGNOSTIC.md) defines the diagnostic
enum catalogues and context encodings. Migrations are their applied database
representation.

## Layout

Migration files live in `migrations/` and use four-digit monotonically
increasing versions:

```text
0001_initial.sql
0002_<description>.sql
...
```

There is one migration for each database schema change, not for each software
release. An applied filename, version or byte content is immutable. Fixes are
new migrations; the pilot has no downgrade migrations.

The initial schema migration will be added with the persistence implementation.
Until then, no database schema in this directory is deployable.

## Runner contract

The application migration runner, not an SQL script:

1. opens the database with foreign keys enabled;
2. sets SQLite `application_id = 0x43555252` (`CURR`) when creating an empty
   receiver database, or validates that value for an existing database;
3. validates every applied migration filename and SHA-256 digest in
   `schema_migrations`;
4. rejects a database newer than the binary, a version gap, a changed digest,
   an incompatible generated enum catalogue or a `database_metadata.group_id`
   different from the loaded receiver group;
5. applies each pending migration in filename order inside one runner-owned
   transaction that also inserts its ledger row; and
6. updates `PRAGMA user_version` to the committed migration version.

Migration SQL must not contain `BEGIN`, `COMMIT`, `ROLLBACK`, `journal_mode` or
`synchronous` statements. Runtime database setup separately establishes and
verifies WAL mode and `synchronous=FULL` before ordinary persistence admission
becomes available.

Migration resources are resolved relative to the installed receiver package,
never the process working directory. A large future table rebuild requires an
explicit maintenance procedure rather than an unbounded startup migration.

## Enum catalogues

Receiver-local persisted enum assignments, including the diagnostic catalogues,
will come from the checked-in `receiver/schemas/receiver_interface.json`
required by the interfaces before the initial migration is implemented.
Generation emits static Python enums, migration insert blocks and the expected
startup catalogue. SQLite is not used to define Python enums at runtime.

Each persisted enum has a strict reference table with numeric `id` and unique
uppercase `code`. Before the first deployable migration, provisional receiver
assignments may be revised through the interface-review process. Once an
assignment appears in a deployed migration it is append-only; startup
validation treats a changed or incomplete catalogue as incompatible.
Protocol-owned enum assignments continue to come from the protocol schema.

## Durability and compatibility

All schema tables use the constraints required by the interface: strict typing,
foreign keys, fixed-length BLOB checks, explicit integer ranges and SQL `NULL`
for absent optional values. Canonical data is never written with `INSERT OR
REPLACE`.

An incompatible schema publishes
`PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA`; the receiver does
not guess, downgrade or overwrite it. Whole-database corruption follows the
artifact-preservation policy in
[`../ARCHITECTURE.md`](../ARCHITECTURE.md), while a structurally healthy
database with a corrupt communicator-state singleton uses the atomic
`quarantined_communicator_states` preservation path.

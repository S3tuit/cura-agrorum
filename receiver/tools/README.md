# Receiver tools

`check_chrony.py` is the read-only deployment policy check installed as Chrony's
`ExecStartPre`. It checks expanded configuration without waiting for network
time; see the [trusted-operator procedure](../hardware/ds3231/README.md#3-configure-time-ownership).

`generate.py` is a host-side build tool. It validates the receiver enum and
entity manifests plus the handwritten SQL source, then generates the checked-in
SQLite schema, Python enum/schema-identity module, and Python entity/binding
module. Reusable logical records may be flattened into relational rows without
creating standalone tables. Canonical-BLOB declarations additionally produce
immutable logical entities, deterministic encoders, structural decoders,
exact-blob SHA-256 binders, and their controlled SQLite envelope tables. The
generator validates foreign-key closure against the fully assembled schema.
Run it from any working directory; use `--check` in tests or CI.

The offline initializer lives in `cura_receiver/database_initializer.py`. It
has different authority: it verifies and executes the exact packaged
`schema.sql`, inserts deployment-specific metadata, and installs only a fresh
database. It never regenerates schemas or enum assignments on the Pi.

`capture_database_evidence.py` is an operator-only best-effort raw copy tool.
Use it after stopping all database users, as described in the
[last-resort restoration workflow](../db/README.md#last-resort-operator-restoration).
It preserves the full database/WAL/SHM context in a new incident directory and
reports capture errors; it never opens SQLite, repairs storage or restores a backup.

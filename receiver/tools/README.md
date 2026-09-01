# Receiver tools

`generate.py` is a host-side build tool. It validates the receiver enum and
entity manifests plus the handwritten SQL source, then generates the checked-in
SQLite schema, Python enum/schema-identity module, and Python entity/binding
module. Canonical-BLOB declarations additionally produce immutable logical
entities, deterministic encoders, structural decoders, exact-blob SHA-256
binders, and their controlled SQLite envelope tables.
Run it from any working directory; use `--check` in tests or CI.

This directory does not yet contain the Pi database initializer. That future
deployment tool has different authority: it will verify and execute the exact
packaged `schema.sql`, insert deployment-specific metadata, and install a fresh
database. It must not regenerate schemas or enum assignments on the Pi.

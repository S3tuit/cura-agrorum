# Generated receiver modules

Files in this directory are checked-in build artifacts. Do not edit them by
hand.

- `protocol_v2_lora_generated.py` comes from the protocol-v2 LoRa schema and
  generator.
- `receiver_enums_generated.py` comes from
  `receiver/schemas/receiver_enums.json` through
  `receiver/tools/generate.py`. It contains stable receiver-local Python enums
  and the expected SQLite application ID, schema version, and exact
  `schema.sql` fingerprint.
- `receiver_entities_generated.py` comes from
  `receiver/schemas/receiver_entities.json` through the same generator. It
  contains immutable logical entities for direct mappings, explicit
  persistence rows for derived or multi-source targets, row-specific SQLite
  table strings and column tuples, pure parameter binders, and canonical-BLOB
  codecs. Relational binders project values only. Canonical encoders derive
  representation fields; decoders enforce the binary grammar needed for safe
  parsing. Neither layer performs policy validation or SQLite I/O.

Generation is static: receiver startup imports these files and never needs a
usable database merely to define an enum. The generator's `--check` mode
verifies that checked-in artifacts are current.

# Editing receiver schemas

[receiver_enums.json](receiver_enums.json) owns stable receiver-local assignments
that cross persistence boundaries. Keep runtime-only enums in code and
protocol-owned assignments in the [protocol schema](../../protocol/protocol-v2-lora/schemas/protocol_v2_lora.json).
[receiver_entities.json](receiver_entities.json) owns logical records, relational
mappings and canonical binary layouts. Use its existing declarations as examples;
[generate.py](../tools/generate.py) validates the supported grammar.

When editing these manifests:

- A `logical_record:<NAME>` with `sql.flatten: true` keeps Python composition
  while flattening SQL columns. Logical records cannot nest. A nullable record
  needs at least one non-nullable member to distinguish absence from presence;
  its required members must be all NULL or all present. The binder writes all
  member columns as NULL for `None`.
- Relational `u64` fields must fit SQLite's signed integer range, `0..2^63-1`.
  Primary/foreign keys and indexes reference SQL column names, which can differ
  from Python field names.
- Array axis order determines relational binding order. Canonical array `length`
  is exact; without it, a derived length field supplies the sequence count.
  Changing these declarations changes stored representation.
- Keep validity masks in binary encodings and optional values in logical/SQL
  fields. `constant` and `derived` encoding fields are representation details,
  not Python constructor arguments. An absent fixed structure occupies its
  all-zero space and is identified through its `presence_bit`.
- `raw_envelope: true` deliberately permits malformed stored values so handwritten
  persistence can classify and archive them. Declared types constrain canonical
  output, not those stored rows. Do not add SQL constraints that prevent this
  recovery path. Lifecycle/metadata and malformed-row archival tables remain in
  [schema_source.sql](../db/schema_source.sql).

Generated relational binders project values; canonical codecs check binary
structure. Handwritten code still owns domain validation, transaction boundaries
and state-transition policy. A generated class or successful decode does not
establish that a value is acceptable for persistence or runtime use.

Regenerate after changing a manifest or the handwritten SQL source, then check
that the checked-in artifacts match. From the repository root:

```sh
.venv/bin/python receiver/tools/generate.py
.venv/bin/python receiver/tools/generate.py --check
```

Generated files identify their inputs; edit those inputs rather than the output.

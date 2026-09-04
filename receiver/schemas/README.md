# Receiver schemas

This directory contains machine-readable receiver contracts used by host-side
generators.

## `receiver_enums.json`

This is the source of truth for stable receiver-local enum assignments that
cross a persistence boundary. Its persistence modes are
`scalar_foreign_key`, `scoped_foreign_key`, and `encoded_only`.

Runtime-only result enums and bit flags are intentionally absent. Protocol-owned
assignments remain in the protocol schema.

## `receiver_entities.json`

This is the source of truth for generated logical Python entities, explicit
persistence-only rows, entity-table DDL, array projection, and transaction
target layouts. Its top-level `logical_records` define reusable immutable
Python values that have no table or binder of their own. A relational field
may reference one as `logical_record:<NAME>` with `sql.flatten: true`; the
generated row keeps that Python composition while SQL columns and binding stay
flat. Logical records do not nest.

Persisted entities have four mapping modes:

- `direct` maps every declared logical field to one SQL column;
- `array_expansion` keeps arrays in the logical Python entity and binds one
  scalar SQL column for each declared axis combination; and
- `multi_table_transaction` declares persistence-row targets that handwritten
  code may handle in one transaction; and
- `canonical_blob` binds a generated logical entity to a named canonical
  binary encoding and a controlled SQLite envelope.

`MessageProfilingV1` is a generated logical record shared by queue-facing code
and `MessageProfileRowV1`; the latter adds persistence's per-occurrence
classification. `ClockObservationV1` and `DiagnosticV1` are generated logical entities with
direct binders. `ReceiverHealthV1` is also a logical entity; its arrays remain
arrays in Python and its binder consumes their exact shapes in deterministic
axis order. Runtime-only chrony and RTC result names used only as array axes
remain layout labels, while stable status bytes stored inside time diagnostic
contexts are encoded-only enums. `QuarantinedEntityRowV1` remains an
explicit persistence row because persistence constructs it from an isolated
queue entity and frozen failure provenance.

The packet transaction declares `MessageProfileRowV1` and
`ReadingMessageRowV1` persistence targets because their values come from
multiple inputs or handwritten derived decisions. `ReadingMessageRowV1` is
keyed by `(node_id, message_id)` and includes the clear reading candidate plus
`is_canonical_for_sample`; a generated partial unique index permits only one
canonical row for each `(node_id, sample_id)`.

`AuthenticatedReadingCandidateV1`, the composed queue-unit types and diagnostic
contexts are not relational entities. The queue-unit types and semantic
validators reuse generated logical records and enums but remain outside this
manifest. Normal queue handoff has no entity codec; only the separate poisoned-
entity evidence format is handwritten.

The top-level `encodings` array owns canonical binary layouts. Each encoding
has a stable name, one endianness, ordered root `fields`, and reusable
fixed-size `structs`. `struct:<NAME>` embeds one structure;
`array:<NAME>` embeds a structure sequence. An array `length` is exact, so the
current communicator state requires exactly 62 `TX_AIRTIME_BUCKET_V1`
elements. Omitting `length` means that a derived integer field supplies the
encoded sequence length; the current communicator-state encoding has no such
variable-length array.

The canonical SQL row uses `write_policy: controlled_singleton`. Generation
therefore emits neither append-only nor immutable triggers for that table;
handwritten persistence may atomically install or replace its one constant-key
row under the communicator-state control contract.

Every other generated SQL row uses `write_policy: append_only`. Generation
emits update/delete rejection plus a pre-insert conflict guard for its primary
key and every declared partial unique identity. The guard prevents
statement-level `INSERT OR REPLACE` from deleting immutable evidence even when
SQLite recursive triggers are disabled.

Encoding fields with `constant` or `derived` are representation details and do
not become Python constructor arguments. The supported derived operations are
`{"encoded_length": true}`, `{"presence_mask": true}`, and
`{"length": "<array_field>"}`. A nullable fixed structure identifies both its
mask and bit through `presence_bit`; an absent structure occupies its fixed
all-zero representation. The communicator-state SQL fields additionally use
`constant`, `encoding`, and `derived.sha256` to produce the singleton row
without duplicating the binary layout. A field without one of those strategies
maps to the same-named logical encoding field.

Generated binders perform projection only. They preserve `None` as SQL `NULL`,
extract `.value` only for fields declared as enums, and expand arrays in the
manifest's fixed order. They do not validate numeric bounds, byte lengths,
validity semantics, cross-field relationships or policy; they perform no
SQLite I/O, transaction selection, classification, replay or reconciliation.
Exact array-shape rejection prevents a projection from silently ignoring
values and is part of consuming the declared mapping, not domain validation.
Generated canonical codecs necessarily check the binary grammar: constants,
encoded length, exact fixed-array sizes, enum assignments, masks, reserved
zeros, collection counts and trailing bytes. They do not validate policy,
ordering, time relationships, lifecycle references or state transitions.

Validity masks belong to canonical binary encoding definitions, not logical
Python entities or relational layouts. A packet or canonical-BLOB codec
translates between encoded mask bits and Python optional values; SQLite stores
those optional values as nullable columns and does not duplicate the mask.

An optional `indexes` array declares generated structured indexes. Its current
form permits a unique index over non-null identity columns with one non-null
Boolean equality predicate. It is used only for the relational
canonical-sample uniqueness rule; code remains responsible for assigning the
Boolean consistently with classification.

For a field, omitted `column` means that the SQL column has the field's name,
and omitted `nullable` means `false`. `enum:<NAME>` refers to an assignment in
`receiver_enums.json`; optional `minimum`, `maximum`, `minimum_length`, and
`maximum_length` values are structural bounds. A non-null variable `bytes`
field may name a non-null integer sibling through `length_field`; generation
then emits an exact SQLite length-equality check. Primary- and foreign-key
lists use SQL column names. Persistence-row field names identify the values
that handwritten code must provide after validated protocol decoding or
persistence classification.

Receiver lifecycle rows, metadata, and exact malformed communicator-state
preservation deliberately remain outside the regular entity mappings and are
defined in `db/schema_source.sql`.

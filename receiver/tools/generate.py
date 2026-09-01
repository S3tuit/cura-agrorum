#!/usr/bin/env python3
"""Generate receiver enums, persisted entities, SQLite schema, and constants.

The receiver enum manifest owns every stable receiver-local numeric enum that
crosses a persistence boundary. The entity manifest owns generated relational
layouts, logical single-source entities, explicit array projection, and
multi-table transaction targets, plus canonical binary encodings. This tool
validates both manifests, appends their generated SQL to the handwritten schema
source, fingerprints the exact generated ``schema.sql`` bytes, and emits
ordinary Python ``Enum`` classes, immutable entity or persistence-row classes,
canonical codecs, row-specific table-name strings and column tuples, pure
SQLite parameter binders, and the expected database identity constants.

The generated SQL deliberately contains no transaction, ``application_id``
assignment, or ``database_metadata`` row. Database initialization owns those
dynamic steps and must compare the packaged SQL fingerprint before executing
the file.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import os
import re
import struct
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any


RECEIVER_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = RECEIVER_ROOT.parent
GENERATOR_PATH = Path(__file__).resolve()

DEFAULT_MANIFEST = RECEIVER_ROOT / "schemas" / "receiver_enums.json"
DEFAULT_ENTITY_MANIFEST = RECEIVER_ROOT / "schemas" / "receiver_entities.json"
DEFAULT_SCHEMA_SOURCE = RECEIVER_ROOT / "db" / "schema_source.sql"
DEFAULT_SCHEMA_OUTPUT = RECEIVER_ROOT / "db" / "schema.sql"
DEFAULT_PYTHON_OUTPUT = (
    RECEIVER_ROOT
    / "cura_receiver"
    / "generated"
    / "receiver_enums_generated.py"
)
DEFAULT_ENTITY_PYTHON_OUTPUT = (
    RECEIVER_ROOT
    / "cura_receiver"
    / "generated"
    / "receiver_entities_generated.py"
)

UPPER_NAME_RE = re.compile(r"^[A-Z][A-Z0-9]*(?:_[A-Z0-9]+)*$")
SQL_IDENTIFIER_RE = re.compile(r"^[a-z][a-z0-9]*(?:_[a-z0-9]+)*$")
STORAGE_RANGES = {
    "u8": (0, (1 << 8) - 1),
    "u16": (0, (1 << 16) - 1),
    "u32": (0, (1 << 32) - 1),
    "i8": (-(1 << 7), (1 << 7) - 1),
    "i16": (-(1 << 15), (1 << 15) - 1),
    "i32": (-(1 << 31), (1 << 31) - 1),
}
SQLITE_INTEGER_RANGES = {
    "u8": (0, (1 << 8) - 1),
    "u16": (0, (1 << 16) - 1),
    "u32": (0, (1 << 32) - 1),
    "u64": (0, (1 << 63) - 1),
    "i8": (-(1 << 7), (1 << 7) - 1),
    "i16": (-(1 << 15), (1 << 15) - 1),
    "i32": (-(1 << 31), (1 << 31) - 1),
    "i64": (-(1 << 63), (1 << 63) - 1),
}
ENTITY_MODES = {
    "array_expansion",
    "canonical_blob",
    "direct",
    "multi_table_transaction",
}
FIELD_TYPE_RE = re.compile(r"^bytes(?:\[([1-9][0-9]*)\])?$")
ENCODING_REFERENCE_RE = re.compile(r"^(struct|array):([A-Z][A-Z0-9]*(?:_[A-Z0-9]+)*)$")
PERSISTENCE_MODES = {
    "scalar_foreign_key",
    "scoped_foreign_key",
    "encoded_only",
}
BINARY_FORMATS = {
    "u8": "B",
    "u16": "H",
    "u32": "I",
    "u64": "Q",
    "i8": "b",
    "i16": "h",
    "i32": "i",
    "i64": "q",
    "bool8": "B",
}


class ManifestError(ValueError):
    """Raised when the receiver enum manifest is inconsistent."""


@dataclass(frozen=True)
class EnumValue:
    name: str
    value: int


@dataclass(frozen=True)
class EnumSpec:
    name: str
    python_name: str
    storage: str
    mode: str
    table: str | None
    scope_enum: str | None
    scope_member: str | None
    scope_column: str | None
    values: tuple[EnumValue, ...]

    def value_named(self, name: str) -> EnumValue:
        for value in self.values:
            if value.name == name:
                return value
        raise ManifestError(f"{self.name} has no member {name}")


@dataclass(frozen=True)
class Manifest:
    schema_format_version: int
    application_id: int
    database_schema_version: int
    enums: tuple[EnumSpec, ...]


@dataclass(frozen=True)
class ArrayAxisMember:
    name: str
    source_index: int


@dataclass(frozen=True)
class ArrayAxis:
    name: str
    enum_name: str | None
    members: tuple[ArrayAxisMember, ...]


@dataclass(frozen=True)
class EntityManifest:
    schema_format_version: int
    enum_source: str
    array_axes: tuple[ArrayAxis, ...]
    encodings: tuple[dict[str, Any], ...]
    entities: tuple[dict[str, Any], ...]

    def axis_named(self, name: str) -> ArrayAxis:
        for axis in self.array_axes:
            if axis.name == name:
                return axis
        raise ManifestError(f"entity manifest has no array axis {name}")

    def encoding_named(self, name: str) -> dict[str, Any]:
        for encoding in self.encodings:
            if encoding["name"] == name:
                return encoding
        raise ManifestError(f"entity manifest has no encoding {name}")


@dataclass(frozen=True)
class Output:
    path: Path
    content: str


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "generate receiver Python enums and entities, SQLite schema, and "
            "schema fingerprint constants"
        )
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=DEFAULT_MANIFEST,
        help="read this enum manifest instead of the default one",
    )
    parser.add_argument(
        "--entities",
        type=Path,
        default=DEFAULT_ENTITY_MANIFEST,
        help="read this entity manifest instead of the default one",
    )
    parser.add_argument(
        "--schema-source",
        type=Path,
        default=DEFAULT_SCHEMA_SOURCE,
        help="read this handwritten SQL source instead of the default one",
    )
    parser.add_argument(
        "--schema-output",
        type=Path,
        default=DEFAULT_SCHEMA_OUTPUT,
        help="write the complete generated SQL schema to this file",
    )
    parser.add_argument(
        "--python-output",
        type=Path,
        default=DEFAULT_PYTHON_OUTPUT,
        help="write generated Python enums and constants to this file",
    )
    parser.add_argument(
        "--entity-python-output",
        type=Path,
        default=DEFAULT_ENTITY_PYTHON_OUTPUT,
        help="write generated Python row layouts to this file",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if a generated output is absent or out of date",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate inputs without reading or writing generated outputs",
    )
    args = parser.parse_args(argv)

    if args.check and args.validate_only:
        parser.error("--check and --validate-only cannot be used together")

    try:
        manifest_bytes = args.manifest.read_bytes()
        manifest_data = json.loads(manifest_bytes.decode("utf-8"))
        manifest = validate_manifest(manifest_data)

        entity_manifest_bytes = args.entities.read_bytes()
        entity_manifest_data = json.loads(entity_manifest_bytes.decode("utf-8"))
        entity_manifest = validate_entity_manifest(
            entity_manifest_data,
            manifest,
            args.manifest.name,
        )

        schema_source = args.schema_source.read_text(encoding="utf-8")
        validate_schema_source(schema_source, manifest, entity_manifest)
        if args.validate_only:
            return 0

        manifest_sha256 = hashlib.sha256(manifest_bytes).hexdigest()
        entity_manifest_sha256 = hashlib.sha256(entity_manifest_bytes).hexdigest()
        schema_source_sha256 = hashlib.sha256(
            schema_source.encode("utf-8")
        ).hexdigest()
        schema_sql = generate_schema(
            manifest,
            entity_manifest,
            schema_source,
            args.manifest,
            args.entities,
            args.schema_source,
            manifest_sha256,
            entity_manifest_sha256,
            schema_source_sha256,
        )
        schema_fingerprint = hashlib.sha256(schema_sql.encode("utf-8")).hexdigest()

        outputs = (
            Output(args.schema_output, schema_sql),
            Output(
                args.python_output,
                generate_python(
                    manifest,
                    args.manifest,
                    args.entities,
                    args.schema_output,
                    manifest_sha256,
                    entity_manifest_sha256,
                    schema_fingerprint,
                ),
            ),
            Output(
                args.entity_python_output,
                generate_entity_python(
                    manifest,
                    entity_manifest,
                    args.entities,
                    entity_manifest_sha256,
                ),
            ),
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ManifestError) as exc:
        print(f"receiver interface generation failed: {exc}", file=sys.stderr)
        return 2

    if args.check:
        stale = [
            output.path
            for output in outputs
            if read_text_if_present(output.path) != output.content
        ]
        for path in stale:
            print(f"{display_path(path)} is absent or out of date")
        return 1 if stale else 0

    for output in outputs:
        write_text_atomic(output.path, output.content)
    return 0


def validate_manifest(data: Any) -> Manifest:
    if not isinstance(data, dict):
        raise ManifestError("manifest root must be an object")
    require_exact_keys(
        data,
        {"schema_format_version", "application_id", "database_schema_version", "enums"},
        "manifest",
    )

    schema_format_version = require_int(
        data, "schema_format_version", "manifest", 1, 1
    )
    application_id = require_int(
        data, "application_id", "manifest", 1, 0x7FFFFFFF
    )
    database_schema_version = require_int(
        data, "database_schema_version", "manifest", 1, (1 << 32) - 1
    )

    enum_data = data.get("enums")
    if not isinstance(enum_data, list) or not enum_data:
        raise ManifestError("manifest.enums must be a nonempty array")

    enums = tuple(validate_enum(item, index) for index, item in enumerate(enum_data))
    validate_enum_relationships(enums)
    return Manifest(
        schema_format_version=schema_format_version,
        application_id=application_id,
        database_schema_version=database_schema_version,
        enums=enums,
    )


def validate_enum(data: Any, index: int) -> EnumSpec:
    context = f"manifest.enums[{index}]"
    if not isinstance(data, dict):
        raise ManifestError(f"{context} must be an object")
    require_exact_keys(data, {"name", "storage", "persistence", "values"}, context)

    name = require_upper_name(data, "name", context)
    storage = data.get("storage")
    if storage not in STORAGE_RANGES:
        raise ManifestError(
            f"{context}.storage must be one of {', '.join(STORAGE_RANGES)}"
        )

    persistence = data.get("persistence")
    if not isinstance(persistence, dict):
        raise ManifestError(f"{context}.persistence must be an object")
    mode = persistence.get("mode")
    if mode not in PERSISTENCE_MODES:
        raise ManifestError(
            f"{context}.persistence.mode must be one of "
            + ", ".join(sorted(PERSISTENCE_MODES))
        )

    table: str | None = None
    scope_enum: str | None = None
    scope_member: str | None = None
    scope_column: str | None = None
    if mode == "scalar_foreign_key":
        require_exact_keys(persistence, {"mode", "table"}, f"{context}.persistence")
        table = require_sql_identifier(persistence, "table", f"{context}.persistence")
    elif mode == "scoped_foreign_key":
        require_exact_keys(
            persistence,
            {"mode", "table", "scope_enum", "scope_member", "scope_column"},
            f"{context}.persistence",
        )
        table = require_sql_identifier(persistence, "table", f"{context}.persistence")
        scope_enum = require_upper_name(
            persistence, "scope_enum", f"{context}.persistence"
        )
        scope_member = require_upper_name(
            persistence, "scope_member", f"{context}.persistence"
        )
        scope_column = require_sql_identifier(
            persistence, "scope_column", f"{context}.persistence"
        )
    else:
        require_exact_keys(persistence, {"mode"}, f"{context}.persistence")

    values_data = data.get("values")
    if not isinstance(values_data, list) or not values_data:
        raise ManifestError(f"{context}.values must be a nonempty array")

    minimum, maximum = STORAGE_RANGES[storage]
    values: list[EnumValue] = []
    names: set[str] = set()
    numbers: set[int] = set()
    for value_index, value_data in enumerate(values_data):
        value_context = f"{context}.values[{value_index}]"
        if not isinstance(value_data, dict):
            raise ManifestError(f"{value_context} must be an object")
        require_exact_keys(value_data, {"name", "value"}, value_context)
        value_name = require_upper_name(value_data, "name", value_context)
        value_number = require_int(
            value_data, "value", value_context, minimum, maximum
        )
        if value_name in names:
            raise ManifestError(f"{context} repeats member name {value_name}")
        if value_number in numbers:
            raise ManifestError(f"{context} repeats numeric value {value_number}")
        names.add(value_name)
        numbers.add(value_number)
        values.append(EnumValue(value_name, value_number))

    return EnumSpec(
        name=name,
        python_name=pascal_case(name),
        storage=storage,
        mode=mode,
        table=table,
        scope_enum=scope_enum,
        scope_member=scope_member,
        scope_column=scope_column,
        values=tuple(values),
    )


def validate_enum_relationships(enums: tuple[EnumSpec, ...]) -> None:
    by_name: dict[str, EnumSpec] = {}
    python_names: set[str] = set()
    scalar_tables: dict[str, str] = {}
    scoped_groups: dict[str, tuple[str, str, str]] = {}
    scoped_rows: dict[str, set[tuple[int, int]]] = {}
    scoped_codes: dict[str, set[tuple[int, str]]] = {}

    for enum in enums:
        if enum.name in by_name:
            raise ManifestError(f"manifest repeats enum name {enum.name}")
        if enum.python_name in python_names:
            raise ManifestError(f"manifest repeats Python class {enum.python_name}")
        by_name[enum.name] = enum
        python_names.add(enum.python_name)

        if enum.mode == "scalar_foreign_key":
            assert enum.table is not None
            if enum.table in scalar_tables:
                raise ManifestError(
                    f"scalar table {enum.table} is shared by {scalar_tables[enum.table]} "
                    f"and {enum.name}"
                )
            scalar_tables[enum.table] = enum.name

    for enum in enums:
        if enum.mode != "scoped_foreign_key":
            continue
        assert enum.table is not None
        assert enum.scope_enum is not None
        assert enum.scope_member is not None
        assert enum.scope_column is not None

        if enum.table in scalar_tables:
            raise ManifestError(f"table {enum.table} is both scalar and scoped")
        scope = by_name.get(enum.scope_enum)
        if scope is None:
            raise ManifestError(
                f"{enum.name} references unknown scope enum {enum.scope_enum}"
            )
        if scope.mode != "scalar_foreign_key" or scope.table is None:
            raise ManifestError(
                f"{enum.name} scope enum {scope.name} must use scalar_foreign_key"
            )
        scope_value = scope.value_named(enum.scope_member).value
        group = (enum.scope_enum, scope.table, enum.scope_column)
        previous_group = scoped_groups.setdefault(enum.table, group)
        if previous_group != group:
            raise ManifestError(
                f"scoped table {enum.table} has inconsistent scope configuration"
            )

        rows = scoped_rows.setdefault(enum.table, set())
        codes = scoped_codes.setdefault(enum.table, set())
        for value in enum.values:
            row = (scope_value, value.value)
            code = (scope_value, value.name)
            if row in rows:
                raise ManifestError(
                    f"scoped table {enum.table} repeats key {row}"
                )
            if code in codes:
                raise ManifestError(
                    f"scoped table {enum.table} repeats code {code}"
                )
            rows.add(row)
            codes.add(code)


def validate_encodings(
    data: Any,
    enum_by_name: dict[str, EnumSpec],
) -> tuple[dict[str, Any], ...]:
    if not isinstance(data, list):
        raise ManifestError("entity manifest.encodings must be an array")

    encodings: list[dict[str, Any]] = []
    encoding_names: set[str] = set()
    struct_names: set[str] = set()
    python_names: set[str] = set()
    for encoding_index, encoding in enumerate(data):
        context = f"entity manifest.encodings[{encoding_index}]"
        if not isinstance(encoding, dict):
            raise ManifestError(f"{context} must be an object")
        require_exact_keys(
            encoding,
            {"name", "endianness", "fields", "structs"},
            context,
        )
        name = require_upper_name(encoding, "name", context)
        if name in encoding_names:
            raise ManifestError(f"entity manifest repeats encoding {name}")
        encoding_names.add(name)
        if encoding.get("endianness") != "little":
            raise ManifestError(f"{context}.endianness must be little")

        structs_data = encoding.get("structs")
        if not isinstance(structs_data, list):
            raise ManifestError(f"{context}.structs must be an array")
        local_structs: dict[str, dict[str, Any]] = {}
        for struct_index, struct_spec in enumerate(structs_data):
            struct_context = f"{context}.structs[{struct_index}]"
            if not isinstance(struct_spec, dict):
                raise ManifestError(f"{struct_context} must be an object")
            require_exact_keys(
                struct_spec,
                {"name", "python_name", "fields"},
                struct_context,
            )
            struct_name = require_upper_name(struct_spec, "name", struct_context)
            python_name = require_python_class_name(
                struct_spec, "python_name", struct_context
            )
            if struct_name in struct_names:
                raise ManifestError(
                    f"entity manifest repeats encoding struct {struct_name}"
                )
            if python_name in python_names:
                raise ManifestError(
                    f"entity manifest repeats encoding Python class {python_name}"
                )
            struct_names.add(struct_name)
            python_names.add(python_name)
            local_structs[struct_name] = struct_spec

        for struct_index, struct_spec in enumerate(structs_data):
            validate_encoding_fields(
                struct_spec.get("fields"),
                f"{context}.structs[{struct_index}].fields",
                enum_by_name,
                local_structs,
                is_root=False,
            )
        validate_encoding_fields(
            encoding.get("fields"),
            context + ".fields",
            enum_by_name,
            local_structs,
            is_root=True,
        )
        for struct_spec in structs_data:
            encoding_struct_size(struct_spec, enum_by_name, local_structs, set())
        encodings.append(encoding)
    return tuple(encodings)


def validate_encoding_fields(
    fields: Any,
    context: str,
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
    *,
    is_root: bool,
) -> None:
    if not isinstance(fields, list) or not fields:
        raise ManifestError(f"{context} must be a nonempty array")
    field_by_name: dict[str, dict[str, Any]] = {}
    for index, field in enumerate(fields):
        field_context = f"{context}[{index}]"
        if not isinstance(field, dict):
            raise ManifestError(f"{field_context} must be an object")
        require_allowed_keys(
            field,
            {
                "name",
                "type",
                "nullable",
                "constant",
                "derived",
                "presence_bit",
                "length",
            },
            {"name", "type"},
            field_context,
        )
        name = require_sql_identifier(field, "name", field_context)
        if name in field_by_name:
            raise ManifestError(f"{context} repeats field {name}")
        field_by_name[name] = field

        type_name = field.get("type")
        if not isinstance(type_name, str):
            raise ManifestError(f"{field_context}.type must be a string")
        reference = ENCODING_REFERENCE_RE.fullmatch(type_name)
        if type_name.startswith("enum:"):
            enum_name = type_name.removeprefix("enum:")
            if enum_name not in enum_by_name:
                raise ManifestError(
                    f"{field_context}.type references unknown enum {enum_name}"
                )
        elif reference is not None:
            reference_kind, reference_name = reference.groups()
            if reference_name not in struct_by_name:
                raise ManifestError(
                    f"{field_context}.type references unknown struct {reference_name}"
                )
            if reference_kind == "array" and not is_root:
                raise ManifestError(
                    f"{field_context} arrays are supported only in encoding roots"
                )
        elif type_name not in BINARY_FORMATS and FIELD_TYPE_RE.fullmatch(type_name) is None:
            raise ManifestError(f"{field_context}.type is unsupported: {type_name}")
        elif type_name == "bytes":
            raise ManifestError(
                f"{field_context}.type must have a fixed byte length in an encoding"
            )

        nullable = field.get("nullable", False)
        if not isinstance(nullable, bool):
            raise ManifestError(f"{field_context}.nullable must be Boolean")
        if nullable and (reference is None or reference.group(1) != "struct"):
            raise ManifestError(
                f"{field_context}.nullable is supported only for fixed structs"
            )
        if nullable != ("presence_bit" in field):
            raise ManifestError(
                f"{field_context} nullable fields require one presence_bit and vice versa"
            )
        if not is_root and nullable:
            raise ManifestError(
                f"{field_context} nested nullable fields are not yet supported"
            )

        strategies = [key for key in ("constant", "derived") if key in field]
        if len(strategies) > 1:
            raise ManifestError(
                f"{field_context} cannot be both constant and derived"
            )
        if strategies and nullable:
            raise ManifestError(
                f"{field_context} nullable fields cannot be constant or derived"
            )
        if "constant" in field:
            if type_name not in BINARY_FORMATS:
                raise ManifestError(
                    f"{field_context}.constant requires an integer storage type"
                )
            constant = field["constant"]
            if isinstance(constant, bool) or not isinstance(constant, int):
                raise ManifestError(f"{field_context}.constant must be an integer")
            validate_binary_integer(constant, type_name, field_context + ".constant")

        if "derived" in field:
            if not is_root:
                raise ManifestError(
                    f"{field_context} nested derived fields are not yet supported"
                )
            if type_name not in {"u8", "u16", "u32", "u64"}:
                raise ManifestError(
                    f"{field_context}.derived requires an unsigned integer storage type"
                )
            derived = field["derived"]
            if not isinstance(derived, dict) or len(derived) != 1:
                raise ManifestError(
                    f"{field_context}.derived must contain exactly one operation"
                )
            operation, operand = next(iter(derived.items()))
            if operation in {"encoded_length", "presence_mask"}:
                if operand is not True:
                    raise ManifestError(
                        f"{field_context}.derived.{operation} must be true"
                    )
            elif operation == "length":
                if not isinstance(operand, str) or SQL_IDENTIFIER_RE.fullmatch(operand) is None:
                    raise ManifestError(
                        f"{field_context}.derived.length must name an array field"
                    )
            else:
                raise ManifestError(
                    f"{field_context}.derived has unsupported operation {operation}"
                )

        if "length" in field:
            if reference is None or reference.group(1) != "array":
                raise ManifestError(f"{field_context}.length requires an array type")
            require_int(field, "length", field_context, 1, (1 << 31) - 1)

    field_positions = {field["name"]: index for index, field in enumerate(fields)}
    length_fields: dict[str, str] = {}
    encoded_lengths = 0
    mask_fields: dict[str, int] = {}
    for field in fields:
        derived = field.get("derived")
        if derived == {"encoded_length": True}:
            encoded_lengths += 1
        elif derived == {"presence_mask": True}:
            mask_fields[field["name"]] = binary_integer_bits(field["type"])
        elif isinstance(derived, dict) and "length" in derived:
            target = derived["length"]
            target_field = field_by_name.get(target)
            target_type = None if target_field is None else target_field["type"]
            target_reference = (
                None
                if target_type is None
                else ENCODING_REFERENCE_RE.fullmatch(target_type)
            )
            if target_reference is None or target_reference.group(1) != "array":
                raise ManifestError(
                    f"{context} length field {field['name']} references non-array {target}"
                )
            if target in length_fields:
                raise ManifestError(f"{context} has two lengths for array {target}")
            if field_positions[field["name"]] > field_positions[target]:
                raise ManifestError(
                    f"{context} length field {field['name']} must precede {target}"
                )
            if "length" in target_field:
                validate_binary_integer(
                    target_field["length"],
                    field["type"],
                    f"{context}.{field['name']}",
                )
            length_fields[target] = field["name"]

    used_bits: set[tuple[str, int]] = set()
    for field in fields:
        presence = field.get("presence_bit")
        if presence is None:
            continue
        field_context = f"{context}.{field['name']}"
        if not isinstance(presence, dict):
            raise ManifestError(f"{field_context}.presence_bit must be an object")
        require_exact_keys(presence, {"mask", "bit"}, field_context + ".presence_bit")
        mask = require_sql_identifier(
            presence, "mask", field_context + ".presence_bit"
        )
        if mask not in mask_fields:
            raise ManifestError(
                f"{field_context}.presence_bit.mask must reference a presence_mask field"
            )
        if field_positions[mask] > field_positions[field["name"]]:
            raise ManifestError(
                f"{field_context}.presence_bit.mask must precede the nullable field"
            )
        bit = require_int(
            presence,
            "bit",
            field_context + ".presence_bit",
            0,
            mask_fields[mask] - 1,
        )
        if (mask, bit) in used_bits:
            raise ManifestError(f"{context} reuses {mask} bit {bit}")
        used_bits.add((mask, bit))

    array_names = {
        field["name"]
        for field in fields
        if (reference := ENCODING_REFERENCE_RE.fullmatch(field["type"])) is not None
        and reference.group(1) == "array"
    }
    missing_lengths = sorted(array_names - set(length_fields))
    if missing_lengths:
        raise ManifestError(
            f"{context} arrays require derived length fields: "
            + ", ".join(missing_lengths)
        )
    if is_root and encoded_lengths != 1:
        raise ManifestError(f"{context} must contain one derived encoded_length")
    if not is_root and encoded_lengths:
        raise ManifestError(f"{context} nested structs cannot derive encoded_length")
    unreferenced_masks = sorted(set(mask_fields) - {mask for mask, _ in used_bits})
    if unreferenced_masks:
        raise ManifestError(
            f"{context} presence masks have no nullable fields: "
            + ", ".join(unreferenced_masks)
        )


def validate_binary_integer(value: int, type_name: str, context: str) -> None:
    try:
        struct.pack("<" + BINARY_FORMATS[type_name], value)
    except struct.error as exc:
        raise ManifestError(f"{context} does not fit {type_name}") from exc


def binary_integer_bits(type_name: str) -> int:
    return struct.calcsize("<" + BINARY_FORMATS[type_name]) * 8


def encoding_struct_size(
    struct_spec: dict[str, Any],
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
    visiting: set[str],
) -> int:
    name = struct_spec["name"]
    if name in visiting:
        raise ManifestError(f"encoding structs contain a cycle at {name}")
    visiting.add(name)
    size = sum(
        encoding_field_fixed_size(field, enum_by_name, struct_by_name, visiting)
        for field in struct_spec["fields"]
    )
    visiting.remove(name)
    return size


def encoding_field_fixed_size(
    field: dict[str, Any],
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
    visiting: set[str] | None = None,
) -> int:
    type_name = field["type"]
    if type_name.startswith("enum:"):
        storage = enum_by_name[type_name.removeprefix("enum:")].storage
        return struct.calcsize("<" + BINARY_FORMATS[storage])
    if type_name in BINARY_FORMATS:
        return struct.calcsize("<" + BINARY_FORMATS[type_name])
    bytes_match = FIELD_TYPE_RE.fullmatch(type_name)
    if bytes_match is not None and bytes_match.group(1) is not None:
        return int(bytes_match.group(1))
    reference = ENCODING_REFERENCE_RE.fullmatch(type_name)
    assert reference is not None
    kind, reference_name = reference.groups()
    item_size = encoding_struct_size(
        struct_by_name[reference_name],
        enum_by_name,
        struct_by_name,
        set() if visiting is None else visiting,
    )
    if kind == "struct":
        return item_size
    if "length" not in field:
        raise ManifestError(f"variable array {field['name']} has no fixed size")
    return field["length"] * item_size


def validate_entity_manifest(
    data: Any,
    enum_manifest: Manifest,
    enum_manifest_filename: str,
) -> EntityManifest:
    if not isinstance(data, dict):
        raise ManifestError("entity manifest root must be an object")
    require_exact_keys(
        data,
        {
            "schema_format_version",
            "enum_source",
            "array_axes",
            "encodings",
            "entities",
        },
        "entity manifest",
    )
    schema_format_version = require_int(
        data, "schema_format_version", "entity manifest", 4, 4
    )
    enum_source = data.get("enum_source")
    if enum_source != enum_manifest_filename:
        raise ManifestError(
            "entity manifest.enum_source must name the selected enum manifest "
            f"({enum_manifest_filename})"
        )

    enum_by_name = {enum.name: enum for enum in enum_manifest.enums}
    axes_data = data.get("array_axes")
    if not isinstance(axes_data, list):
        raise ManifestError("entity manifest.array_axes must be an array")
    axes: list[ArrayAxis] = []
    axis_names: set[str] = set()
    for index, axis_data in enumerate(axes_data):
        context = f"entity manifest.array_axes[{index}]"
        if not isinstance(axis_data, dict):
            raise ManifestError(f"{context} must be an object")
        allowed = {"name", "enum", "members"}
        require_allowed_keys(axis_data, allowed, {"name", "members"}, context)
        name = require_sql_identifier(axis_data, "name", context)
        if name in axis_names:
            raise ManifestError(f"entity manifest repeats array axis {name}")
        axis_names.add(name)
        enum_name = axis_data.get("enum")
        enum_spec: EnumSpec | None = None
        if enum_name is not None:
            if not isinstance(enum_name, str) or UPPER_NAME_RE.fullmatch(enum_name) is None:
                raise ManifestError(f"{context}.enum must be UPPER_SNAKE_CASE")
            enum_spec = enum_by_name.get(enum_name)
            if enum_spec is None:
                raise ManifestError(f"{context}.enum references unknown enum {enum_name}")

        members_data = axis_data.get("members")
        if not isinstance(members_data, list) or not members_data:
            raise ManifestError(f"{context}.members must be a nonempty array")
        members: list[ArrayAxisMember] = []
        member_names: set[str] = set()
        source_indices: set[int] = set()
        for member_index, member_data in enumerate(members_data):
            member_context = f"{context}.members[{member_index}]"
            if not isinstance(member_data, dict):
                raise ManifestError(f"{member_context} must be an object")
            require_exact_keys(member_data, {"name", "source_index"}, member_context)
            member_name = require_upper_name(member_data, "name", member_context)
            source_index = require_int(
                member_data, "source_index", member_context, 0, (1 << 31) - 1
            )
            if member_name in member_names:
                raise ManifestError(f"{context} repeats member {member_name}")
            if source_index in source_indices:
                raise ManifestError(
                    f"{context} repeats source_index {source_index}"
                )
            if enum_spec is not None:
                enum_spec.value_named(member_name)
            member_names.add(member_name)
            source_indices.add(source_index)
            members.append(ArrayAxisMember(member_name, source_index))
        if source_indices != set(range(len(members))):
            raise ManifestError(
                f"{context}.source_index values must be contiguous from zero"
            )
        axes.append(ArrayAxis(name, enum_name, tuple(members)))

    encodings = validate_encodings(data.get("encodings"), enum_by_name)

    entities_data = data.get("entities")
    if not isinstance(entities_data, list) or not entities_data:
        raise ManifestError("entity manifest.entities must be a nonempty array")
    entity_names: set[str] = set()
    python_names = {
        struct_spec["python_name"]
        for encoding in encodings
        for struct_spec in encoding["structs"]
    }
    table_names: set[str] = set()
    entities: list[dict[str, Any]] = []
    provisional = EntityManifest(
        schema_format_version,
        enum_source,
        tuple(axes),
        encodings,
        (),
    )
    for index, entity_data in enumerate(entities_data):
        context = f"entity manifest.entities[{index}]"
        if not isinstance(entity_data, dict):
            raise ManifestError(f"{context} must be an object")
        name = require_upper_name(entity_data, "name", context)
        if name in entity_names:
            raise ManifestError(f"entity manifest repeats entity {name}")
        entity_names.add(name)
        persistence = entity_data.get("persistence")
        if not isinstance(persistence, dict):
            raise ManifestError(f"{context}.persistence must be an object")
        mode = persistence.get("mode")
        if mode not in ENTITY_MODES:
            raise ManifestError(
                f"{context}.persistence.mode must be one of "
                + ", ".join(sorted(ENTITY_MODES))
            )
        if mode == "multi_table_transaction":
            require_exact_keys(entity_data, {"name", "persistence"}, context)
            validate_transaction_mapping(
                persistence,
                context + ".persistence",
                enum_by_name,
                provisional,
                python_names,
                table_names,
            )
        else:
            require_allowed_keys(
                entity_data,
                {
                    "name",
                    "python_name",
                    "persistence",
                    "fields",
                    "indexes",
                    "foreign_keys",
                },
                {"name", "python_name", "persistence", "fields", "foreign_keys"},
                context,
            )
            python_name = require_python_class_name(entity_data, "python_name", context)
            if python_name in python_names:
                raise ManifestError(f"entity manifest repeats Python class {python_name}")
            python_names.add(python_name)
            validate_row_layout(
                entity_data,
                persistence,
                mode,
                context,
                enum_by_name,
                provisional,
                table_names,
            )
            if mode == "canonical_blob":
                validate_canonical_mapping(
                    entity_data,
                    context,
                    provisional,
                )
        entities.append(entity_data)

    return EntityManifest(
        schema_format_version,
        enum_source,
        tuple(axes),
        encodings,
        tuple(entities),
    )


def validate_transaction_mapping(
    persistence: dict[str, Any],
    context: str,
    enum_by_name: dict[str, EnumSpec],
    entity_manifest: EntityManifest,
    python_names: set[str],
    table_names: set[str],
) -> None:
    require_exact_keys(
        persistence,
        {"mode", "atomic", "targets"},
        context,
    )
    if persistence.get("atomic") is not True:
        raise ManifestError(f"{context}.atomic must be true")

    targets_data = persistence.get("targets")
    if not isinstance(targets_data, list) or not targets_data:
        raise ManifestError(f"{context}.targets must be a nonempty array")
    target_names: set[str] = set()
    for target_index, target in enumerate(targets_data):
        target_context = f"{context}.targets[{target_index}]"
        if not isinstance(target, dict):
            raise ManifestError(f"{target_context} must be an object")
        require_allowed_keys(
            target,
            {
                "name",
                "python_name",
                "table",
                "write_policy",
                "primary_key",
                "without_rowid",
                "fields",
                "indexes",
                "foreign_keys",
            },
            {
                "name",
                "python_name",
                "table",
                "write_policy",
                "primary_key",
                "without_rowid",
                "fields",
                "foreign_keys",
            },
            target_context,
        )
        target_name = require_upper_name(target, "name", target_context)
        if target_name in target_names:
            raise ManifestError(f"{context} repeats target {target_name}")
        target_names.add(target_name)
        python_name = require_python_class_name(target, "python_name", target_context)
        if python_name in python_names:
            raise ManifestError(f"entity manifest repeats Python class {python_name}")
        python_names.add(python_name)
        row_persistence = {
            "mode": "transaction_target",
            "table": target["table"],
            "write_policy": target["write_policy"],
            "primary_key": target["primary_key"],
            "without_rowid": target["without_rowid"],
        }
        validate_row_layout(
            target,
            row_persistence,
            "transaction_target",
            target_context,
            enum_by_name,
            entity_manifest,
            table_names,
        )


def validate_row_layout(
    row: dict[str, Any],
    persistence: dict[str, Any],
    mode: str,
    context: str,
    enum_by_name: dict[str, EnumSpec],
    entity_manifest: EntityManifest,
    table_names: set[str],
) -> None:
    if mode != "transaction_target":
        require_exact_keys(
            persistence,
            {"mode", "table", "write_policy", "primary_key", "without_rowid"},
            context + ".persistence",
        )
    table = require_sql_identifier(persistence, "table", context + ".persistence")
    if table in table_names:
        raise ManifestError(f"entity manifest repeats table {table}")
    table_names.add(table)
    expected_write_policy = (
        "controlled_singleton" if mode == "canonical_blob" else "append_only"
    )
    if persistence.get("write_policy") != expected_write_policy:
        raise ManifestError(
            f"{context} write_policy must be {expected_write_policy}"
        )
    if not isinstance(persistence.get("without_rowid"), bool):
        raise ManifestError(f"{context} without_rowid must be Boolean")

    fields = row.get("fields")
    if not isinstance(fields, list) or not fields:
        raise ManifestError(f"{context}.fields must be a nonempty array")
    field_names: set[str] = set()
    columns: dict[str, bool] = {}
    column_types: dict[str, str] = {}
    for field_index, field in enumerate(fields):
        field_context = f"{context}.fields[{field_index}]"
        validate_entity_field(
            field,
            field_context,
            mode,
            enum_by_name,
            entity_manifest,
        )
        assert isinstance(field, dict)
        name = field["name"]
        if name in field_names:
            raise ManifestError(f"{context} repeats field {name}")
        field_names.add(name)
        for _, column in expand_field(field, entity_manifest):
            if column in columns:
                raise ManifestError(f"{context} repeats SQL column {column}")
            columns[column] = bool(field.get("nullable", False))
            column_types[column] = field["type"]

    primary_key = persistence.get("primary_key")
    validate_identifier_list(primary_key, context + ".primary_key")
    assert isinstance(primary_key, list)
    for column in primary_key:
        if column not in columns:
            raise ManifestError(f"{context}.primary_key references unknown column {column}")
        if columns[column]:
            raise ManifestError(f"{context}.primary_key column {column} is nullable")

    indexes = row.get("indexes", [])
    if not isinstance(indexes, list):
        raise ManifestError(f"{context}.indexes must be an array")
    local_index_names: set[str] = set()
    for index_number, index in enumerate(indexes):
        index_context = f"{context}.indexes[{index_number}]"
        if not isinstance(index, dict):
            raise ManifestError(f"{index_context} must be an object")
        require_exact_keys(
            index,
            {"name", "columns", "unique", "where"},
            index_context,
        )
        index_name = require_sql_identifier(index, "name", index_context)
        if index_name in local_index_names or index_name in table_names:
            raise ManifestError(
                f"entity manifest repeats SQLite schema object {index_name}"
            )
        local_index_names.add(index_name)
        table_names.add(index_name)
        index_columns = index.get("columns")
        validate_identifier_list(index_columns, index_context + ".columns")
        assert isinstance(index_columns, list)
        for column in index_columns:
            if column not in columns:
                raise ManifestError(
                    f"{index_context} references unknown column {column}"
                )
            if columns[column]:
                raise ManifestError(
                    f"{index_context} indexed identity column {column} is nullable"
                )
        if index.get("unique") is not True:
            raise ManifestError(f"{index_context}.unique must be true")
        where = index.get("where")
        if not isinstance(where, dict):
            raise ManifestError(f"{index_context}.where must be an object")
        require_exact_keys(where, {"column", "equals"}, index_context + ".where")
        where_column = require_sql_identifier(
            where, "column", index_context + ".where"
        )
        if where_column not in columns:
            raise ManifestError(
                f"{index_context}.where references unknown column {where_column}"
            )
        if column_types[where_column] != "bool8" or columns[where_column]:
            raise ManifestError(
                f"{index_context}.where column must be a non-null bool8"
            )
        require_int(where, "equals", index_context + ".where", 0, 1)

    foreign_keys = row.get("foreign_keys")
    if not isinstance(foreign_keys, list):
        raise ManifestError(f"{context}.foreign_keys must be an array")
    foreign_key_shapes: set[tuple[tuple[str, ...], str, tuple[str, ...]]] = set()
    for fk_index, foreign_key in enumerate(foreign_keys):
        fk_context = f"{context}.foreign_keys[{fk_index}]"
        if not isinstance(foreign_key, dict):
            raise ManifestError(f"{fk_context} must be an object")
        require_exact_keys(foreign_key, {"columns", "references"}, fk_context)
        local_columns = foreign_key.get("columns")
        validate_identifier_list(local_columns, fk_context + ".columns")
        assert isinstance(local_columns, list)
        for column in local_columns:
            if column not in columns:
                raise ManifestError(f"{fk_context} references unknown column {column}")
        references = foreign_key.get("references")
        if not isinstance(references, dict):
            raise ManifestError(f"{fk_context}.references must be an object")
        require_exact_keys(references, {"table", "columns"}, fk_context + ".references")
        require_sql_identifier(references, "table", fk_context + ".references")
        referenced_columns = references.get("columns")
        validate_identifier_list(referenced_columns, fk_context + ".references.columns")
        assert isinstance(referenced_columns, list)
        if len(local_columns) != len(referenced_columns):
            raise ManifestError(f"{fk_context} local and referenced arity differ")
        foreign_key_shapes.add(
            (
                tuple(local_columns),
                references["table"],
                tuple(referenced_columns),
            )
        )

    for field in fields:
        type_name = field["type"]
        if not type_name.startswith("enum:"):
            continue
        enum_spec = enum_by_name[type_name.removeprefix("enum:")]
        assert enum_spec.table is not None
        column = field.get("column", field["name"])
        expected_shape = ((column,), enum_spec.table, ("id",))
        if expected_shape not in foreign_key_shapes:
            raise ManifestError(
                f"{context} enum column {column} requires a foreign key to "
                f"{enum_spec.table}(id)"
            )


def validate_entity_field(
    field: Any,
    context: str,
    mode: str,
    enum_by_name: dict[str, EnumSpec],
    entity_manifest: EntityManifest,
) -> None:
    if not isinstance(field, dict):
        raise ManifestError(f"{context} must be an object")
    allowed = {
        "name",
        "type",
        "column",
        "nullable",
        "minimum",
        "maximum",
        "minimum_length",
        "maximum_length",
        "array_axes",
        "column_pattern",
        "constant",
        "derived",
        "encoding",
    }
    required = {"name", "type"}
    require_allowed_keys(field, allowed, required, context)
    require_sql_identifier(field, "name", context)
    type_name = field.get("type")
    if not isinstance(type_name, str):
        raise ManifestError(f"{context}.type must be a string")
    enum_name: str | None = None
    if type_name.startswith("enum:"):
        enum_name = type_name.removeprefix("enum:")
        if enum_name not in enum_by_name:
            raise ManifestError(f"{context}.type references unknown enum {enum_name}")
        if enum_by_name[enum_name].mode != "scalar_foreign_key":
            raise ManifestError(
                f"{context}.type can store only scalar_foreign_key enums; "
                f"{enum_name} uses {enum_by_name[enum_name].mode}"
            )
    elif type_name == "blob":
        if mode != "canonical_blob":
            raise ManifestError(f"{context}.type blob requires canonical_blob mode")
    elif (
        type_name not in SQLITE_INTEGER_RANGES
        and type_name != "bool8"
        and FIELD_TYPE_RE.fullmatch(type_name) is None
    ):
        raise ManifestError(f"{context}.type is unsupported: {type_name}")

    if "column" in field:
        require_sql_identifier(field, "column", context)
    if "nullable" in field and not isinstance(field["nullable"], bool):
        raise ManifestError(f"{context}.nullable must be Boolean")
    for key in ("minimum", "maximum", "minimum_length", "maximum_length"):
        if key in field and (isinstance(field[key], bool) or not isinstance(field[key], int)):
            raise ManifestError(f"{context}.{key} must be an integer")

    if type_name in SQLITE_INTEGER_RANGES:
        storage_minimum, storage_maximum = SQLITE_INTEGER_RANGES[type_name]
        minimum = field.get("minimum", storage_minimum)
        maximum = field.get("maximum", storage_maximum)
        if not storage_minimum <= minimum <= maximum <= storage_maximum:
            raise ManifestError(f"{context} numeric bounds exceed {type_name}")
        if "minimum_length" in field or "maximum_length" in field:
            raise ManifestError(f"{context} integer field cannot have length bounds")
    elif type_name == "bool8" or enum_name is not None:
        if any(key in field for key in ("minimum", "maximum", "minimum_length", "maximum_length")):
            raise ManifestError(f"{context} type cannot have explicit bounds")
    elif type_name != "blob":
        fixed_match = FIELD_TYPE_RE.fullmatch(type_name)
        assert fixed_match is not None
        fixed_length = fixed_match.group(1)
        if "minimum" in field or "maximum" in field:
            raise ManifestError(f"{context} byte field cannot have numeric bounds")
        if fixed_length is not None and (
            "minimum_length" in field or "maximum_length" in field
        ):
            raise ManifestError(f"{context} fixed byte field cannot have length bounds")
        if fixed_length is None:
            minimum_length = field.get("minimum_length", 0)
            maximum_length = field.get("maximum_length")
            if minimum_length < 0 or maximum_length is None or maximum_length < minimum_length:
                raise ManifestError(
                    f"{context} variable bytes require valid minimum_length and maximum_length"
                )

    special_keys = {"constant", "derived", "encoding"} & set(field)
    if mode != "canonical_blob" and special_keys:
        raise ManifestError(
            f"{context} canonical mapping keys require canonical_blob mode"
        )
    if mode == "canonical_blob":
        strategies = {"constant", "derived", "encoding"} & set(field)
        if len(strategies) > 1:
            raise ManifestError(
                f"{context} has multiple canonical mapping strategies"
            )
        if "constant" in field:
            value = field["constant"]
            if (
                type_name not in SQLITE_INTEGER_RANGES
                or isinstance(value, bool)
                or not isinstance(value, int)
            ):
                raise ManifestError(
                    f"{context}.constant requires an integer SQL field"
                )
            minimum = field.get("minimum", SQLITE_INTEGER_RANGES[type_name][0])
            maximum = field.get("maximum", SQLITE_INTEGER_RANGES[type_name][1])
            if not minimum <= value <= maximum:
                raise ManifestError(f"{context}.constant is outside its SQL bounds")
        if "encoding" in field:
            encoding_name = field["encoding"]
            if type_name != "blob" or not isinstance(encoding_name, str):
                raise ManifestError(
                    f"{context}.encoding requires a blob SQL field"
                )
            entity_manifest.encoding_named(encoding_name)

    axes = field.get("array_axes")
    if axes is None:
        if "column_pattern" in field:
            raise ManifestError(f"{context}.column_pattern requires array_axes")
    else:
        if mode != "array_expansion":
            raise ManifestError(f"{context}.array_axes requires array_expansion mode")
        if enum_name is not None:
            raise ManifestError(f"{context} expanded arrays must contain counters")
        if "column" in field or field.get("nullable", False):
            raise ManifestError(f"{context} expanded arrays cannot set column or nullable")
        if not isinstance(axes, list) or not 1 <= len(axes) <= 2:
            raise ManifestError(f"{context}.array_axes must contain one or two axes")
        if len(set(axes)) != len(axes):
            raise ManifestError(f"{context}.array_axes repeats an axis")
        pattern = field.get("column_pattern")
        if not isinstance(pattern, str):
            raise ManifestError(f"{context}.column_pattern must be a string")
        for axis_name in axes:
            if not isinstance(axis_name, str):
                raise ManifestError(f"{context}.array_axes entries must be strings")
            entity_manifest.axis_named(axis_name)
            if "{" + axis_name + "}" not in pattern:
                raise ManifestError(
                    f"{context}.column_pattern must contain {{{axis_name}}}"
                )


def validate_canonical_mapping(
    entity: dict[str, Any],
    context: str,
    entity_manifest: EntityManifest,
) -> None:
    fields = entity["fields"]
    encoded_fields = [field for field in fields if "encoding" in field]
    if len(encoded_fields) != 1:
        raise ManifestError(f"{context} must contain exactly one encoded blob field")
    encoded_field = encoded_fields[0]
    encoding = entity_manifest.encoding_named(encoded_field["encoding"])
    encoding_fields = {field["name"]: field for field in encoding["fields"]}
    logical_names = {
        name
        for name, field in encoding_fields.items()
        if "constant" not in field and "derived" not in field
    }

    hash_fields: list[dict[str, Any]] = []
    for field in fields:
        strategies = {"constant", "derived", "encoding"} & set(field)
        if not strategies:
            name = field["name"]
            if name not in logical_names:
                raise ManifestError(
                    f"{context} SQL field {name} is not a logical field of "
                    f"{encoding['name']}"
                )
            encoding_field = encoding_fields[name]
            if field["type"] != encoding_field["type"]:
                raise ManifestError(
                    f"{context} SQL field {name} and its encoding field must "
                    "have the same type"
                )
        if "constant" in field:
            encoded_constant = encoding_fields.get(field["name"])
            if encoded_constant is not None and encoded_constant.get("constant") != field["constant"]:
                raise ManifestError(
                    f"{context} SQL and encoding constants differ for {field['name']}"
                )
        if "derived" not in field:
            continue
        derived = field["derived"]
        if not isinstance(derived, dict) or set(derived) != {"sha256"}:
            raise ManifestError(
                f"{context}.{field['name']}.derived must be sha256"
            )
        if derived["sha256"] != encoded_field["name"]:
            raise ManifestError(
                f"{context}.{field['name']} must hash {encoded_field['name']}"
            )
        if field["type"] != "bytes[32]":
            raise ManifestError(
                f"{context}.{field['name']} SHA-256 field must be bytes[32]"
            )
        hash_fields.append(field)
    if len(hash_fields) != 1:
        raise ManifestError(f"{context} must contain exactly one SHA-256 field")

    if entity.get("indexes"):
        raise ManifestError(f"{context} canonical singleton cannot declare indexes")
    if entity["foreign_keys"]:
        raise ManifestError(
            f"{context} canonical blob SQL envelope cannot declare foreign keys"
        )


def validate_identifier_list(value: Any, context: str) -> None:
    if not isinstance(value, list) or not value:
        raise ManifestError(f"{context} must be a nonempty array")
    seen: set[str] = set()
    for index, item in enumerate(value):
        if not isinstance(item, str) or SQL_IDENTIFIER_RE.fullmatch(item) is None:
            raise ManifestError(f"{context}[{index}] must be a snake_case SQL identifier")
        if item in seen:
            raise ManifestError(f"{context} repeats {item}")
        seen.add(item)


def require_python_class_name(
    data: dict[str, Any], key: str, context: str
) -> str:
    value = data.get(key)
    if not isinstance(value, str) or re.fullmatch(r"^[A-Z][A-Za-z0-9]*$", value) is None:
        raise ManifestError(f"{context}.{key} must be a Python class name")
    return value


def require_allowed_keys(
    data: dict[str, Any],
    allowed: set[str],
    required: set[str],
    context: str,
) -> None:
    missing = sorted(required - set(data))
    unexpected = sorted(set(data) - allowed)
    if not missing and not unexpected:
        return
    parts: list[str] = []
    if missing:
        parts.append("missing " + ", ".join(missing))
    if unexpected:
        parts.append("unexpected " + ", ".join(unexpected))
    raise ManifestError(f"{context} has " + "; ".join(parts))


def validate_schema_source(
    source: str,
    manifest: Manifest,
    entity_manifest: EntityManifest,
) -> None:
    if not source.strip():
        raise ManifestError("schema source must not be empty")
    executable_lines = "\n".join(
        line.split("--", 1)[0] for line in source.splitlines()
    )
    if not re.search(
        r"\bCREATE\s+TABLE\s+database_metadata\b",
        executable_lines,
        flags=re.IGNORECASE,
    ):
        raise ManifestError("schema source must create database_metadata")

    forbidden = (
        (
            r"(?m)^\s*(?:BEGIN\s+(?:DEFERRED|IMMEDIATE|EXCLUSIVE|TRANSACTION)"
            r"|COMMIT(?:\s+TRANSACTION)?|ROLLBACK(?:\s+TRANSACTION)?)\s*;",
            "transaction boundary",
        ),
        (r"\bPRAGMA\s+(?:application_id|user_version)\b", "identity/version PRAGMA"),
        (r"\bINSERT\s+INTO\s+database_metadata\b", "database_metadata row"),
        (r"\bschema_migrations\b", "schema_migrations reference"),
    )
    for pattern, description in forbidden:
        if re.search(pattern, executable_lines, flags=re.IGNORECASE):
            raise ManifestError(f"schema source must not contain a {description}")

    generated_tables = {
        enum.table
        for enum in manifest.enums
        if enum.table is not None
    }
    generated_tables.update(
        row["table"] for row in iter_row_layouts(entity_manifest)
    )
    for table in sorted(generated_tables):
        if re.search(
            rf"\bCREATE\s+TABLE\s+{re.escape(table)}\b",
            executable_lines,
            flags=re.IGNORECASE,
        ):
            raise ManifestError(
                f"schema source creates generated catalogue table {table}"
            )


def generate_schema(
    manifest: Manifest,
    entity_manifest: EntityManifest,
    schema_source: str,
    manifest_path: Path,
    entity_manifest_path: Path,
    schema_source_path: Path,
    manifest_sha256: str,
    entity_manifest_sha256: str,
    schema_source_sha256: str,
) -> str:
    lines = [
        f"-- Generated from {display_path(manifest_path)},",
        f"-- {display_path(entity_manifest_path)}, and",
        f"-- {display_path(schema_source_path)} by {display_path(GENERATOR_PATH)}.",
        "-- Do not edit this file by hand.",
        f"-- Receiver enum manifest SHA-256: {manifest_sha256}",
        f"-- Receiver entity manifest SHA-256: {entity_manifest_sha256}",
        f"-- Handwritten schema source SHA-256: {schema_source_sha256}",
        "",
        schema_source.rstrip(),
        "",
        "-- Generated receiver enum catalogues.",
        "",
    ]

    for enum in manifest.enums:
        if enum.mode != "scalar_foreign_key":
            continue
        assert enum.table is not None
        lines.extend(render_scalar_catalogue(enum))

    emitted_scoped_tables: set[str] = set()
    for enum in manifest.enums:
        if enum.mode != "scoped_foreign_key":
            continue
        assert enum.table is not None
        if enum.table in emitted_scoped_tables:
            continue
        emitted_scoped_tables.add(enum.table)
        group = tuple(
            candidate
            for candidate in manifest.enums
            if candidate.mode == "scoped_foreign_key"
            and candidate.table == enum.table
        )
        lines.extend(render_scoped_catalogue(manifest, group))

    lines.extend(["-- Generated receiver entity tables.", ""])
    for row in iter_row_layouts(entity_manifest):
        lines.extend(render_entity_table(row, entity_manifest))

    return "\n".join(lines).rstrip() + "\n"


def render_scalar_catalogue(enum: EnumSpec) -> list[str]:
    assert enum.table is not None
    lines = [
        f"CREATE TABLE {enum.table} (",
        "    id INTEGER PRIMARY KEY,",
        "    code TEXT NOT NULL UNIQUE",
        ") STRICT;",
        "",
        f"INSERT INTO {enum.table} (id, code) VALUES",
    ]
    lines.extend(render_insert_rows([(value.value, value.name) for value in enum.values]))
    lines.extend(["", *render_immutable_triggers(enum.table), ""])
    return lines


def render_scoped_catalogue(
    manifest: Manifest,
    group: tuple[EnumSpec, ...],
) -> list[str]:
    first = group[0]
    assert first.table is not None
    assert first.scope_enum is not None
    assert first.scope_column is not None
    scope = next(enum for enum in manifest.enums if enum.name == first.scope_enum)
    assert scope.table is not None

    lines = [
        f"CREATE TABLE {first.table} (",
        f"    {first.scope_column} INTEGER NOT NULL",
        f"        REFERENCES {scope.table}(id),",
        "    id INTEGER NOT NULL,",
        "    code TEXT NOT NULL,",
        f"    PRIMARY KEY ({first.scope_column}, id),",
        f"    UNIQUE ({first.scope_column}, code)",
        ") STRICT, WITHOUT ROWID;",
        "",
        f"INSERT INTO {first.table} ({first.scope_column}, id, code) VALUES",
    ]
    rows: list[tuple[int, int, str]] = []
    for enum in group:
        assert enum.scope_member is not None
        scope_value = scope.value_named(enum.scope_member).value
        rows.extend((scope_value, value.value, value.name) for value in enum.values)
    lines.extend(render_insert_rows(rows))
    lines.extend(["", *render_immutable_triggers(first.table), ""])
    return lines


def iter_row_layouts(entity_manifest: EntityManifest) -> tuple[dict[str, Any], ...]:
    rows: list[dict[str, Any]] = []
    for entity in entity_manifest.entities:
        persistence = entity["persistence"]
        mode = persistence["mode"]
        if mode == "multi_table_transaction":
            for target in persistence["targets"]:
                rows.append(
                    {
                        **target,
                        "mode": mode,
                        "transaction": entity["name"],
                    }
                )
            continue
        rows.append(
            {
                "name": entity["name"],
                "python_name": entity["python_name"],
                "mode": mode,
                "table": persistence["table"],
                "write_policy": persistence["write_policy"],
                "primary_key": persistence["primary_key"],
                "without_rowid": persistence["without_rowid"],
                "fields": entity["fields"],
                "indexes": entity.get("indexes", []),
                "foreign_keys": entity["foreign_keys"],
            }
        )
    return tuple(rows)


def expand_field(
    field: dict[str, Any], entity_manifest: EntityManifest
) -> tuple[tuple[str, str], ...]:
    axes = field.get("array_axes")
    if axes is None:
        return ((field["name"], field.get("column", field["name"])),)
    axis_specs = [entity_manifest.axis_named(name) for name in axes]
    expanded: list[tuple[str, str]] = []
    for members in itertools.product(*(axis.members for axis in axis_specs)):
        substitutions = {
            axis.name: member.name.lower()
            for axis, member in zip(axis_specs, members, strict=True)
        }
        column = field["column_pattern"].format(**substitutions)
        if SQL_IDENTIFIER_RE.fullmatch(column) is None:
            raise ManifestError(
                f"expanded column {column!r} is not a snake_case SQL identifier"
            )
        expanded.append((column, column))
    return tuple(expanded)


def render_entity_table(
    row: dict[str, Any], entity_manifest: EntityManifest
) -> list[str]:
    definitions: list[str] = []
    for field in row["fields"]:
        for _, column in expand_field(field, entity_manifest):
            definitions.append(render_entity_column(field, column))
    primary_key = ", ".join(row["primary_key"])
    definitions.append(f"PRIMARY KEY ({primary_key})")
    for foreign_key in row["foreign_keys"]:
        local_columns = ", ".join(foreign_key["columns"])
        references = foreign_key["references"]
        remote_columns = ", ".join(references["columns"])
        definitions.append(
            f"FOREIGN KEY ({local_columns}) REFERENCES "
            f"{references['table']}({remote_columns})"
        )

    lines = [f"CREATE TABLE {row['table']} ("]
    for index, definition in enumerate(definitions):
        suffix = "," if index < len(definitions) - 1 else ""
        lines.append(f"    {definition}{suffix}")
    table_suffix = ") STRICT"
    if row["without_rowid"]:
        table_suffix += ", WITHOUT ROWID"
    lines.extend([table_suffix + ";", ""])
    for index in row.get("indexes", []):
        lines.extend([*render_entity_index(row["table"], index), ""])
    if row["write_policy"] == "append_only":
        lines.extend([*render_append_only_triggers(row["table"]), ""])
    return lines


def render_entity_index(table: str, index: dict[str, Any]) -> list[str]:
    unique = "UNIQUE " if index["unique"] else ""
    columns = ", ".join(index["columns"])
    where = index["where"]
    return [
        f"CREATE {unique}INDEX {index['name']}",
        f"ON {table} ({columns})",
        f"WHERE {where['column']} = {where['equals']};",
    ]


def render_entity_column(field: dict[str, Any], column: str) -> str:
    type_name = field["type"]
    nullable = bool(field.get("nullable", False))
    affinity = (
        "BLOB"
        if type_name == "blob" or FIELD_TYPE_RE.fullmatch(type_name)
        else "INTEGER"
    )
    parts = [column, affinity]
    if not nullable:
        parts.append("NOT NULL")

    condition: str | None = None
    if "constant" in field:
        condition = f"{column} = {field['constant']}"
    elif type_name in SQLITE_INTEGER_RANGES:
        default_minimum, default_maximum = SQLITE_INTEGER_RANGES[type_name]
        minimum = field.get("minimum", default_minimum)
        maximum = field.get("maximum", default_maximum)
        condition = f"{column} BETWEEN {minimum} AND {maximum}"
    elif type_name == "bool8":
        condition = f"{column} IN (0, 1)"
    else:
        bytes_match = FIELD_TYPE_RE.fullmatch(type_name)
        if bytes_match is not None:
            fixed_length = bytes_match.group(1)
            if fixed_length is not None:
                condition = f"length({column}) = {fixed_length}"
            else:
                minimum_length = field.get("minimum_length", 0)
                maximum_length = field["maximum_length"]
                condition = (
                    f"length({column}) BETWEEN {minimum_length} AND {maximum_length}"
                )
    if condition is not None:
        if nullable:
            condition = f"{column} IS NULL OR {condition}"
        parts.append(f"CHECK ({condition})")
    return " ".join(parts)


def render_append_only_triggers(table: str) -> list[str]:
    lines: list[str] = []
    for action in ("UPDATE", "DELETE"):
        action_lower = action.lower()
        lines.extend(
            [
                f"CREATE TRIGGER {table}_no_{action_lower}",
                f"BEFORE {action} ON {table}",
                "BEGIN",
                f"    SELECT RAISE(ABORT, '{table} is append-only');",
                "END;",
                "",
            ]
        )
    if lines:
        lines.pop()
    return lines


def render_insert_rows(rows: list[tuple[Any, ...]]) -> list[str]:
    rendered: list[str] = []
    for index, row in enumerate(rows):
        suffix = ";" if index == len(rows) - 1 else ","
        values = ", ".join(sql_literal(value) for value in row)
        rendered.append(f"    ({values}){suffix}")
    return rendered


def render_immutable_triggers(table: str) -> list[str]:
    lines: list[str] = []
    for action in ("INSERT", "UPDATE", "DELETE"):
        action_lower = action.lower()
        lines.extend(
            [
                f"CREATE TRIGGER {table}_no_{action_lower}",
                f"BEFORE {action} ON {table}",
                "BEGIN",
                f"    SELECT RAISE(ABORT, '{table} is immutable');",
                "END;",
                "" if action != "DELETE" else "",
            ]
        )
    if lines and lines[-1] == "":
        lines.pop()
    return lines


def generate_python(
    manifest: Manifest,
    manifest_path: Path,
    entity_manifest_path: Path,
    schema_path: Path,
    manifest_sha256: str,
    entity_manifest_sha256: str,
    schema_fingerprint: str,
) -> str:
    enum_names = [enum.python_name for enum in manifest.enums]
    lines = [
        f"# Generated from {display_path(manifest_path)} by {display_path(GENERATOR_PATH)}.",
        f"# Entity layout: {display_path(entity_manifest_path)}.",
        f"# Database schema: {display_path(schema_path)}. Do not edit by hand.",
        "from __future__ import annotations",
        "",
        "from enum import Enum, unique",
        "",
        f"RECEIVER_ENUM_MANIFEST_SHA256 = {manifest_sha256!r}",
        f"RECEIVER_ENTITY_MANIFEST_SHA256 = {entity_manifest_sha256!r}",
        f"SQLITE_APPLICATION_ID = 0x{manifest.application_id:08X}",
        f"DATABASE_SCHEMA_VERSION = {manifest.database_schema_version}",
        f"DATABASE_SCHEMA_SHA256 = {schema_fingerprint!r}",
        "DATABASE_SCHEMA_FINGERPRINT = bytes.fromhex(DATABASE_SCHEMA_SHA256)",
        "",
        "__all__ = [",
        '    "RECEIVER_ENUM_MANIFEST_SHA256",',
        '    "RECEIVER_ENTITY_MANIFEST_SHA256",',
        '    "SQLITE_APPLICATION_ID",',
        '    "DATABASE_SCHEMA_VERSION",',
        '    "DATABASE_SCHEMA_SHA256",',
        '    "DATABASE_SCHEMA_FINGERPRINT",',
    ]
    lines.extend(f'    "{name}",' for name in enum_names)
    lines.extend(["]", ""])

    for enum in manifest.enums:
        lines.extend(["@unique", f"class {enum.python_name}(Enum):"])
        for value in enum.values:
            lines.append(f"    {value.name} = {value.value}")
        lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def generate_entity_python(
    enum_manifest: Manifest,
    entity_manifest: EntityManifest,
    entity_manifest_path: Path,
    entity_manifest_sha256: str,
) -> str:
    rows = iter_row_layouts(entity_manifest)
    relational_rows = tuple(row for row in rows if row["mode"] != "canonical_blob")
    canonical_entities = tuple(
        entity
        for entity in entity_manifest.entities
        if entity["persistence"]["mode"] == "canonical_blob"
    )
    enum_by_name = {enum.name: enum for enum in enum_manifest.enums}
    used_enum_names = sorted(
        {
            field["type"].removeprefix("enum:")
            for field in (
                [
                    field
                    for row in relational_rows
                    for field in row["fields"]
                ]
                + [
                    field
                    for encoding in entity_manifest.encodings
                    for container in [encoding, *encoding["structs"]]
                    for field in container["fields"]
                ]
            )
            if field["type"].startswith("enum:")
        }
    )
    imported_python_enums = [enum_by_name[name].python_name for name in used_enum_names]
    struct_class_names = [
        struct_spec["python_name"]
        for encoding in entity_manifest.encodings
        for struct_spec in encoding["structs"]
    ]
    class_names = (
        [row["python_name"] for row in relational_rows]
        + struct_class_names
        + [entity["python_name"] for entity in canonical_entities]
    )
    layout_constant_names = [
        name
        for row in rows
        for name in (row["name"] + "_TABLE", row["name"] + "_COLUMNS")
    ]
    binder_names = [snake_case(row["python_name"]) + "_parameters" for row in rows]
    codec_names = [
        name
        for entity in canonical_entities
        for name in (
            "encode_" + snake_case(entity["python_name"]),
            "decode_" + snake_case(entity["python_name"]),
        )
    ]

    lines = [
        f"# Generated from {display_path(entity_manifest_path)} by {display_path(GENERATOR_PATH)}.",
        "# Do not edit by hand.",
        "from __future__ import annotations",
        "",
    ]
    if canonical_entities:
        lines.extend(["import hashlib", "import struct", ""])
    lines.extend(
        [
        "from dataclasses import dataclass",
        "",
        ]
    )
    if imported_python_enums:
        lines.append("from .receiver_enums_generated import (")
        lines.extend(f"    {name}," for name in imported_python_enums)
        lines.extend([")", ""])
    lines.extend(
        [
            f"RECEIVER_ENTITY_MANIFEST_SHA256 = {entity_manifest_sha256!r}",
            "",
            "__all__ = [",
            '    "RECEIVER_ENTITY_MANIFEST_SHA256",',
        ]
    )
    lines.extend(f'    "{name}",' for name in layout_constant_names)
    lines.extend(f'    "{name}",' for name in class_names)
    lines.extend(f'    "{name}",' for name in codec_names)
    lines.extend(f'    "{name}",' for name in binder_names)
    lines.extend(["]", ""])

    for row in rows:
        emitted_fields = sqlite_bound_fields(row, entity_manifest)
        table_constant_name = row["name"] + "_TABLE"
        column_constant_name = row["name"] + "_COLUMNS"
        columns = tuple(column for _, column, _ in emitted_fields)
        lines.append(f"{table_constant_name} = {row['table']!r}")
        lines.append("")
        lines.append(f"{column_constant_name} = (")
        lines.extend(f"    {column!r}," for column in columns)
        lines.extend([")", ""])

    if canonical_entities:
        lines.extend(render_canonical_runtime_helpers())

    for row in relational_rows:
        lines.extend(
            [
                "@dataclass(frozen=True, slots=True)",
                f"class {row['python_name']}:",
            ]
        )
        for field in row["fields"]:
            attribute = field["name"]
            annotation = python_type_for_field(field, enum_by_name)
            for _ in field.get("array_axes", []):
                annotation = f"tuple[{annotation}, ...]"
            if field.get("nullable", False):
                annotation += " | None"
            lines.append(f"    {attribute}: {annotation}")
        lines.append("")

        binder_name = snake_case(row["python_name"]) + "_parameters"
        lines.extend(
            [
                f"def {binder_name}(entity: {row['python_name']}) -> tuple[object, ...]:",
            ]
        )
        lines.extend(render_array_shape_guards(row, entity_manifest))
        lines.extend(
            [
                "    return (",
            ]
        )
        for field in row["fields"]:
            axes_data = field.get("array_axes")
            if axes_data is None:
                expression = f"entity.{field['name']}"
                lines.append(
                    f"        {sqlite_binding_expression(expression, field)},"
                )
                continue
            axes = [entity_manifest.axis_named(name) for name in axes_data]
            for members in itertools.product(*(axis.members for axis in axes)):
                indexes = "".join(
                    f"[{member.source_index}]" for member in members
                )
                expression = f"entity.{field['name']}{indexes}"
                lines.append(
                    f"        {sqlite_binding_expression(expression, field)},"
                )
        lines.extend(["    )", ""])

    for encoding in entity_manifest.encodings:
        struct_by_name = {
            struct_spec["name"]: struct_spec
            for struct_spec in encoding["structs"]
        }
        for struct_spec in encoding["structs"]:
            lines.extend(
                render_encoding_dataclass(
                    struct_spec["python_name"],
                    struct_spec["fields"],
                    enum_by_name,
                    struct_by_name,
                )
            )

    for entity in canonical_entities:
        encoded_field = next(field for field in entity["fields"] if "encoding" in field)
        encoding = entity_manifest.encoding_named(encoded_field["encoding"])
        struct_by_name = {
            struct_spec["name"]: struct_spec
            for struct_spec in encoding["structs"]
        }
        lines.extend(
            render_encoding_dataclass(
                entity["python_name"],
                encoding["fields"],
                enum_by_name,
                struct_by_name,
            )
        )
        for struct_spec in encoding["structs"]:
            lines.extend(
                render_encoding_struct_codec(
                    struct_spec,
                    encoding,
                    enum_by_name,
                    struct_by_name,
                )
            )
        lines.extend(
            render_canonical_codec(
                entity,
                encoding,
                enum_by_name,
                struct_by_name,
            )
        )
        lines.extend(render_canonical_binder(entity))

    return "\n".join(lines).rstrip() + "\n"


def render_canonical_runtime_helpers() -> list[str]:
    return [
        "def _pack(format_code: str, value: object, field: str) -> bytes:",
        "    try:",
        "        return struct.pack(format_code, value)",
        "    except struct.error as exc:",
        "        raise ValueError(f'{field} does not fit its encoded type') from exc",
        "",
        "def _unpack(",
        "    blob: memoryview, offset: int, format_code: str, field: str",
        ") -> tuple[object, int]:",
        "    size = struct.calcsize(format_code)",
        "    if offset + size > len(blob):",
        "        raise ValueError(f'{field} is truncated')",
        "    return struct.unpack_from(format_code, blob, offset)[0], offset + size",
        "",
        "def _read_exact(",
        "    blob: memoryview, offset: int, length: int, field: str",
        ") -> tuple[bytes, int]:",
        "    end = offset + length",
        "    if end > len(blob):",
        "        raise ValueError(f'{field} is truncated')",
        "    return bytes(blob[offset:end]), end",
        "",
        "def _require_bytes_length(value: bytes, length: int, field: str) -> bytes:",
        "    if len(value) != length:",
        "        raise ValueError(f'{field} must contain exactly {length} bytes')",
        "    return value",
        "",
    ]


def render_encoding_dataclass(
    python_name: str,
    fields: list[dict[str, Any]],
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
) -> list[str]:
    lines = ["@dataclass(frozen=True, slots=True)", f"class {python_name}:"]
    logical_fields = [
        field for field in fields if "constant" not in field and "derived" not in field
    ]
    if not logical_fields:
        lines.append("    pass")
    for field in logical_fields:
        annotation = python_type_for_encoding_field(
            field, enum_by_name, struct_by_name
        )
        if field.get("nullable", False):
            annotation += " | None"
        lines.append(f"    {field['name']}: {annotation}")
    lines.append("")
    return lines


def python_type_for_encoding_field(
    field: dict[str, Any],
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
) -> str:
    type_name = field["type"]
    if type_name.startswith("enum:"):
        return enum_by_name[type_name.removeprefix("enum:")].python_name
    reference = ENCODING_REFERENCE_RE.fullmatch(type_name)
    if reference is not None:
        kind, reference_name = reference.groups()
        item_type = struct_by_name[reference_name]["python_name"]
        return item_type if kind == "struct" else f"tuple[{item_type}, ...]"
    if FIELD_TYPE_RE.fullmatch(type_name) is not None:
        return "bytes"
    if type_name == "bool8":
        return "bool"
    return "int"


def render_encoding_struct_codec(
    struct_spec: dict[str, Any],
    encoding: dict[str, Any],
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
) -> list[str]:
    python_name = struct_spec["python_name"]
    function_suffix = snake_case(python_name)
    endian = encoding_endian_prefix(encoding)
    size = encoding_struct_size(struct_spec, enum_by_name, struct_by_name, set())
    lines = [
        f"def _encode_{function_suffix}(value: {python_name}) -> bytes:",
        "    chunks: list[bytes] = []",
    ]
    for field in struct_spec["fields"]:
        expression = (
            repr(field["constant"])
            if "constant" in field
            else f"value.{field['name']}"
        )
        lines.extend(
            render_encoded_field_append(
                field,
                expression,
                f"{python_name}.{field['name']}",
                endian,
                enum_by_name,
                struct_by_name,
                indent="    ",
            )
        )
    lines.extend(
        [
            "    encoded = b''.join(chunks)",
            f"    if len(encoded) != {size}:",
            f"        raise RuntimeError('{struct_spec['name']} generated an invalid size')",
            "    return encoded",
            "",
            f"def _decode_{function_suffix}(",
            "    blob: memoryview, offset: int",
            f") -> tuple[{python_name}, int]:",
        ]
    )
    for field in struct_spec["fields"]:
        lines.extend(
            render_decoded_field_read(
                field,
                f"{python_name}.{field['name']}",
                endian,
                enum_by_name,
                struct_by_name,
                indent="    ",
            )
        )
    logical_fields = [
        field
        for field in struct_spec["fields"]
        if "constant" not in field and "derived" not in field
    ]
    lines.append(f"    return {python_name}(")
    lines.extend(
        f"        {field['name']}={field['name']}," for field in logical_fields
    )
    lines.extend(["    ), offset", ""])
    return lines


def render_encoded_field_append(
    field: dict[str, Any],
    expression: str,
    label: str,
    endian: str,
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
    *,
    indent: str,
) -> list[str]:
    type_name = field["type"]
    if type_name.startswith("enum:"):
        storage = enum_by_name[type_name.removeprefix("enum:")].storage
        return [
            f"{indent}chunks.append(_pack({(endian + BINARY_FORMATS[storage])!r}, "
            f"{expression}.value, {label!r}))"
        ]
    if type_name in BINARY_FORMATS:
        return [
            f"{indent}chunks.append(_pack({(endian + BINARY_FORMATS[type_name])!r}, "
            f"{expression}, {label!r}))"
        ]
    bytes_match = FIELD_TYPE_RE.fullmatch(type_name)
    if bytes_match is not None:
        length = int(bytes_match.group(1))
        return [
            f"{indent}chunks.append(_require_bytes_length(",
            f"{indent}    {expression}, {length}, {label!r}",
            f"{indent}))",
        ]
    reference = ENCODING_REFERENCE_RE.fullmatch(type_name)
    assert reference is not None and reference.group(1) == "struct"
    helper = snake_case(struct_by_name[reference.group(2)]["python_name"])
    return [f"{indent}chunks.append(_encode_{helper}({expression}))"]


def render_decoded_field_read(
    field: dict[str, Any],
    label: str,
    endian: str,
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
    *,
    indent: str,
) -> list[str]:
    name = field["name"]
    type_name = field["type"]
    lines: list[str]
    if type_name.startswith("enum:"):
        enum_spec = enum_by_name[type_name.removeprefix("enum:")]
        lines = [
            f"{indent}{name}_value, offset = _unpack(",
            f"{indent}    blob, offset, {(endian + BINARY_FORMATS[enum_spec.storage])!r}, {label!r}",
            f"{indent})",
            f"{indent}try:",
            f"{indent}    {name} = {enum_spec.python_name}({name}_value)",
            f"{indent}except ValueError as exc:",
            f"{indent}    raise ValueError({label!r} + ' has an unknown enum value') from exc",
        ]
    elif type_name in BINARY_FORMATS:
        lines = [
            f"{indent}{name}, offset = _unpack(",
            f"{indent}    blob, offset, {(endian + BINARY_FORMATS[type_name])!r}, {label!r}",
            f"{indent})",
        ]
        if type_name == "bool8":
            lines.extend(
                [
                    f"{indent}if {name} not in (0, 1):",
                    f"{indent}    raise ValueError({label!r} + ' is not Boolean')",
                    f"{indent}{name} = bool({name})",
                ]
            )
    else:
        bytes_match = FIELD_TYPE_RE.fullmatch(type_name)
        if bytes_match is not None:
            length = int(bytes_match.group(1))
            lines = [
                f"{indent}{name}, offset = _read_exact(",
                f"{indent}    blob, offset, {length}, {label!r}",
                f"{indent})",
            ]
        else:
            reference = ENCODING_REFERENCE_RE.fullmatch(type_name)
            assert reference is not None and reference.group(1) == "struct"
            helper = snake_case(struct_by_name[reference.group(2)]["python_name"])
            lines = [f"{indent}{name}, offset = _decode_{helper}(blob, offset)"]
    if "constant" in field:
        lines.extend(
            [
                f"{indent}if {name} != {field['constant']!r}:",
                f"{indent}    raise ValueError({label!r} + ' has an invalid constant value')",
            ]
        )
    return lines


def render_canonical_codec(
    entity: dict[str, Any],
    encoding: dict[str, Any],
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
) -> list[str]:
    python_name = entity["python_name"]
    function_suffix = snake_case(python_name)
    endian = encoding_endian_prefix(encoding)
    fields = encoding["fields"]
    length_fields = {
        field["derived"]["length"]: field["name"]
        for field in fields
        if isinstance(field.get("derived"), dict) and "length" in field["derived"]
    }
    presence_fields: dict[str, list[dict[str, Any]]] = {}
    for field in fields:
        presence = field.get("presence_bit")
        if presence is not None:
            presence_fields.setdefault(presence["mask"], []).append(field)

    lines = [f"def encode_{function_suffix}(entity: {python_name}) -> bytes:"]
    for field in fields:
        reference = ENCODING_REFERENCE_RE.fullmatch(field["type"])
        if reference is None or reference.group(1) != "array" or "length" not in field:
            continue
        lines.extend(
            [
                f"    if len(entity.{field['name']}) != {field['length']}:",
                "        raise ValueError(",
                f"            {field['name']!r} + ' must have length {field['length']}'",
                "        )",
            ]
        )
    for field in fields:
        derived = field.get("derived")
        if derived == {"encoded_length": True}:
            lines.append(
                f"    {field['name']} = {encoding_size_expression(encoding, enum_by_name, struct_by_name)}"
            )
        elif derived == {"presence_mask": True}:
            contributors = presence_fields.get(field["name"], [])
            expression = " | ".join(
                f"((1 << {item['presence_bit']['bit']}) if "
                f"entity.{item['name']} is not None else 0)"
                for item in contributors
            ) or "0"
            lines.append(f"    {field['name']} = {expression}")
        elif isinstance(derived, dict) and "length" in derived:
            lines.append(
                f"    {field['name']} = len(entity.{derived['length']})"
            )
    lines.append("    chunks: list[bytes] = []")
    for field in fields:
        type_name = field["type"]
        reference = ENCODING_REFERENCE_RE.fullmatch(type_name)
        if reference is not None and reference.group(1) == "array":
            helper = snake_case(struct_by_name[reference.group(2)]["python_name"])
            lines.extend(
                [
                    f"    for item in entity.{field['name']}:",
                    f"        chunks.append(_encode_{helper}(item))",
                ]
            )
            continue
        if reference is not None and reference.group(1) == "struct" and field.get("nullable", False):
            size = encoding_field_fixed_size(field, enum_by_name, struct_by_name)
            helper = snake_case(struct_by_name[reference.group(2)]["python_name"])
            lines.extend(
                [
                    f"    if entity.{field['name']} is None:",
                    f"        chunks.append(bytes({size}))",
                    "    else:",
                    f"        chunks.append(_encode_{helper}(entity.{field['name']}))",
                ]
            )
            continue
        if "constant" in field:
            expression = repr(field["constant"])
        elif "derived" in field:
            expression = field["name"]
        else:
            expression = f"entity.{field['name']}"
        lines.extend(
            render_encoded_field_append(
                field,
                expression,
                f"{python_name}.{field['name']}",
                endian,
                enum_by_name,
                struct_by_name,
                indent="    ",
            )
        )
    encoded_length_field = next(
        field["name"]
        for field in fields
        if field.get("derived") == {"encoded_length": True}
    )
    lines.extend(
        [
            "    encoded = b''.join(chunks)",
            f"    if len(encoded) != {encoded_length_field}:",
            f"        raise RuntimeError('{encoding['name']} generated an invalid length')",
            "    return encoded",
            "",
            f"def decode_{function_suffix}(blob: bytes) -> {python_name}:",
            "    try:",
            "        view = memoryview(blob)",
            "    except TypeError as exc:",
            "        raise ValueError('canonical blob must be bytes-like') from exc",
            "    blob = view",
            "    offset = 0",
        ]
    )
    for field in fields:
        name = field["name"]
        type_name = field["type"]
        reference = ENCODING_REFERENCE_RE.fullmatch(type_name)
        label = f"{python_name}.{name}"
        if reference is not None and reference.group(1) == "array":
            count_name = length_fields[name]
            if "length" in field:
                lines.extend(
                    [
                        f"    if {count_name} != {field['length']}:",
                        f"        raise ValueError({label!r} + ' has an invalid fixed length')",
                        f"    {name}_items: list[{struct_by_name[reference.group(2)]['python_name']}] = []",
                        f"    for _ in range({field['length']}):",
                    ]
                )
            else:
                lines.extend(
                    [
                        f"    {name}_items: list[{struct_by_name[reference.group(2)]['python_name']}] = []",
                        f"    for _ in range({count_name}):",
                    ]
                )
            helper = snake_case(struct_by_name[reference.group(2)]["python_name"])
            lines.extend(
                [
                    f"        item, offset = _decode_{helper}(view, offset)",
                    f"        {name}_items.append(item)",
                    f"    {name} = tuple({name}_items)",
                ]
            )
            continue
        if reference is not None and reference.group(1) == "struct" and field.get("nullable", False):
            presence = field["presence_bit"]
            size = encoding_field_fixed_size(field, enum_by_name, struct_by_name)
            helper = snake_case(struct_by_name[reference.group(2)]["python_name"])
            lines.extend(
                [
                    f"    if {presence['mask']} & (1 << {presence['bit']}):",
                    f"        {name}, offset = _decode_{helper}(view, offset)",
                    "    else:",
                    f"        {name}_absent, offset = _read_exact(",
                    f"            view, offset, {size}, {label!r}",
                    "        )",
                    f"        if any({name}_absent):",
                    f"            raise ValueError({label!r} + ' absent representation is not zero')",
                    f"        {name} = None",
                ]
            )
            continue
        lines.extend(
            render_decoded_field_read(
                field,
                label,
                endian,
                enum_by_name,
                struct_by_name,
                indent="    ",
            )
        )
        if field.get("derived") == {"encoded_length": True}:
            lines.extend(
                [
                    f"    if {name} != len(view):",
                    f"        raise ValueError({label!r} + ' does not match the blob length')",
                ]
            )
        if field.get("derived") == {"presence_mask": True}:
            known_mask = sum(
                1 << contributor["presence_bit"]["bit"]
                for contributor in presence_fields.get(name, [])
            )
            lines.extend(
                [
                    f"    if {name} & ~{known_mask}:",
                    f"        raise ValueError({label!r} + ' has reserved bits set')",
                ]
            )
    lines.extend(
        [
            "    if offset != len(view):",
            "        raise ValueError('canonical blob has trailing bytes')",
            f"    return {python_name}(",
        ]
    )
    for field in fields:
        if "constant" not in field and "derived" not in field:
            lines.append(f"        {field['name']}={field['name']},")
    lines.extend(["    )", ""])
    return lines


def encoding_size_expression(
    encoding: dict[str, Any],
    enum_by_name: dict[str, EnumSpec],
    struct_by_name: dict[str, dict[str, Any]],
) -> str:
    fixed_size = 0
    dynamic_terms: list[str] = []
    for field in encoding["fields"]:
        reference = ENCODING_REFERENCE_RE.fullmatch(field["type"])
        if reference is not None and reference.group(1) == "array" and "length" not in field:
            item_size = encoding_struct_size(
                struct_by_name[reference.group(2)],
                enum_by_name,
                struct_by_name,
                set(),
            )
            dynamic_terms.append(f"len(entity.{field['name']}) * {item_size}")
        else:
            fixed_size += encoding_field_fixed_size(
                field, enum_by_name, struct_by_name
            )
    return " + ".join([str(fixed_size), *dynamic_terms])


def encoding_endian_prefix(encoding: dict[str, Any]) -> str:
    assert encoding["endianness"] == "little"
    return "<"


def render_canonical_binder(entity: dict[str, Any]) -> list[str]:
    python_name = entity["python_name"]
    function_suffix = snake_case(python_name)
    encoded_fields = [field for field in entity["fields"] if "encoding" in field]
    lines = [
        f"def {function_suffix}_parameters(entity: {python_name}) -> tuple[object, ...]:"
    ]
    for field in encoded_fields:
        lines.append(
            f"    {field['name']} = encode_{function_suffix}(entity)"
        )
    lines.append("    return (")
    for field in entity["fields"]:
        if "constant" in field:
            expression = repr(field["constant"])
        elif "encoding" in field:
            expression = field["name"]
        elif "derived" in field:
            source_name = field["derived"]["sha256"]
            expression = f"hashlib.sha256({source_name}).digest()"
        else:
            expression = f"entity.{field['name']}"
            expression = sqlite_binding_expression(expression, field)
        lines.append(f"        {expression},")
    lines.extend(["    )", ""])
    return lines


def sqlite_bound_fields(
    row: dict[str, Any], entity_manifest: EntityManifest
) -> tuple[tuple[str, str, dict[str, Any]], ...]:
    fields: list[tuple[str, str, dict[str, Any]]] = []
    for field in row["fields"]:
        expanded = expand_field(field, entity_manifest)
        for attribute, column in expanded:
            fields.append((attribute, column, field))
    return tuple(fields)


def python_type_for_field(
    field: dict[str, Any], enum_by_name: dict[str, EnumSpec]
) -> str:
    type_name = field["type"]
    if type_name.startswith("enum:"):
        return enum_by_name[type_name.removeprefix("enum:")].python_name
    if FIELD_TYPE_RE.fullmatch(type_name) is not None:
        return "bytes"
    if type_name == "bool8":
        return "bool"
    return "int"


def sqlite_binding_expression(expression: str, field: dict[str, Any]) -> str:
    if not field["type"].startswith("enum:"):
        return expression
    if field.get("nullable", False):
        return f"None if {expression} is None else {expression}.value"
    return expression + ".value"


def render_array_shape_guards(
    row: dict[str, Any], entity_manifest: EntityManifest
) -> list[str]:
    lines: list[str] = []
    array_fields = [field for field in row["fields"] if "array_axes" in field]
    for field in array_fields:
        expression = "entity." + field["name"]
        axes = [entity_manifest.axis_named(name) for name in field["array_axes"]]
        lines.extend(
            [
                f"    if len({expression}) != {len(axes[0].members)}:",
                "        raise ValueError(",
                f"            {field['name']!r} + ' must have length {len(axes[0].members)}'",
                "        )",
            ]
        )
        if len(axes) == 2:
            lines.extend(
                [
                    f"    if any(len(item) != {len(axes[1].members)} for item in {expression}):",
                    "        raise ValueError(",
                    f"            {field['name']!r} + "
                    f"' inner arrays must have length {len(axes[1].members)}'",
                    "        )",
                ]
            )
    return lines


def require_exact_keys(data: dict[str, Any], expected: set[str], context: str) -> None:
    actual = set(data)
    if actual == expected:
        return
    missing = sorted(expected - actual)
    unexpected = sorted(actual - expected)
    parts: list[str] = []
    if missing:
        parts.append("missing " + ", ".join(missing))
    if unexpected:
        parts.append("unexpected " + ", ".join(unexpected))
    raise ManifestError(f"{context} has " + "; ".join(parts))


def require_int(
    data: dict[str, Any],
    key: str,
    context: str,
    minimum: int,
    maximum: int,
) -> int:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ManifestError(f"{context}.{key} must be an integer")
    if not minimum <= value <= maximum:
        raise ManifestError(
            f"{context}.{key} must be in [{minimum}, {maximum}]"
        )
    return value


def require_upper_name(data: dict[str, Any], key: str, context: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or UPPER_NAME_RE.fullmatch(value) is None:
        raise ManifestError(f"{context}.{key} must be UPPER_SNAKE_CASE")
    return value


def require_sql_identifier(data: dict[str, Any], key: str, context: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or SQL_IDENTIFIER_RE.fullmatch(value) is None:
        raise ManifestError(f"{context}.{key} must be a snake_case SQL identifier")
    return value


def pascal_case(name: str) -> str:
    return "".join(part.lower().capitalize() for part in name.split("_"))


def snake_case(name: str) -> str:
    first_pass = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", first_pass).lower()


def sql_literal(value: Any) -> str:
    if isinstance(value, int) and not isinstance(value, bool):
        return str(value)
    if isinstance(value, str) and UPPER_NAME_RE.fullmatch(value) is not None:
        return "'" + value + "'"
    raise AssertionError(f"unsupported generated SQL literal: {value!r}")


def display_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(path)


def read_text_if_present(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None


def write_text_atomic(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        text=True,
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as file:
            file.write(content)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary_path, path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


if __name__ == "__main__":
    raise SystemExit(main())

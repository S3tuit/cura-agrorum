#!/usr/bin/env python3
# Usage (from the repository root):
#   protocol/protocol-v2-lora/tools/generate.py [OPTIONS]
# With no options, reads the default schema and writes all default outputs.
#   --schema PATH         Read this schema instead of the default one.
#   --c-header PATH       Write the generated C declarations to this file.
#   --c-source PATH       Write the generated C implementation to this file.
#   --python-output PATH  Write the generated Python codec to this file.
#   --validate-only       Validate the schema without reading/writing outputs.
#   --check               Fail if an output is missing or differs from generation.
"""Generate deterministic C and Python codecs for LoRa protocol v2.

The JSON schema is the wire-format source of truth. This tool validates that
schema before emitting:

* a C header containing logical structures, constants and codec declarations;
* a C source file containing explicit byte-wise encoders and decoders; and
* a Python module containing matching dataclasses, constants and codecs.

The generated C code never casts byte storage to a C structure and does not
depend on compiler packing or host byte order. Cryptography, radio operation,
key provisioning, retransmission and persistence remain handwritten code.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROTOCOL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
GENERATOR_PATH = Path(__file__).resolve()

DEFAULT_SCHEMA = PROTOCOL_ROOT / "schemas" / "protocol_v2_lora.json"
DEFAULT_C_HEADER = (
    REPO_ROOT
    / "firmware"
    / "components"
    / "protocol_v2_lora"
    / "include"
    / "protocol_v2_lora_schema_generated.h"
)
DEFAULT_C_SOURCE = (
    REPO_ROOT
    / "firmware"
    / "components"
    / "protocol_v2_lora"
    / "protocol_v2_lora_schema_generated.c"
)
DEFAULT_PYTHON = (
    REPO_ROOT
    / "receiver"
    / "cura_receiver"
    / "generated"
    / "protocol_v2_lora_generated.py"
)

IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
BYTES_TYPE_RE = re.compile(r"^bytes\[(\d+)]$")

TYPE_INFO: dict[str, dict[str, Any]] = {
    "u8": {"size": 1, "c": "uint8_t", "python": "int", "struct": "B"},
    "u16": {"size": 2, "c": "uint16_t", "python": "int", "struct": "H"},
    "u32": {"size": 4, "c": "uint32_t", "python": "int", "struct": "I"},
    "i16": {"size": 2, "c": "int16_t", "python": "int", "struct": "h"},
}

SUPPORTED_CONSTRAINTS = {
    "all_zero_unless_flag",
    "flag_iff_field_equals",
    "flag_requires_flag",
    "reserved_bits_zero",
    "zero_unless_flag",
}

C_PREFIX = "CURA_LORA_V2"
C_TYPE_PREFIX = "cura_lora_v2"


class SchemaError(ValueError):
    """Raised when the protocol schema is inconsistent or unsupported."""


@dataclass(frozen=True)
class Output:
    path: Path
    content: str


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate Cura Agrorum LoRa protocol v2 codecs"
    )
    parser.add_argument(
        "--schema",
        type=Path,
        default=DEFAULT_SCHEMA,
        help="read this schema instead of the default one",
    )
    parser.add_argument(
        "--c-header",
        type=Path,
        default=DEFAULT_C_HEADER,
        help="write the generated C declarations to this file",
    )
    parser.add_argument(
        "--c-source",
        type=Path,
        default=DEFAULT_C_SOURCE,
        help="write the generated C implementation to this file",
    )
    parser.add_argument(
        "--python-output",
        type=Path,
        default=DEFAULT_PYTHON,
        help="write the generated Python codec to this file",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if any generated output is absent or out of date",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate the schema without reading or writing generated outputs",
    )
    args = parser.parse_args(argv)

    if args.check and args.validate_only:
        parser.error("--check and --validate-only cannot be used together")

    try:
        schema = load_schema(args.schema)
        validate_schema(schema)
        if args.validate_only:
            return 0

        schema_hash = schema_sha256(schema)
        outputs = (
            Output(
                args.c_header,
                generate_c_header(schema, args.schema, schema_hash),
            ),
            Output(
                args.c_source,
                generate_c_source(schema, args.schema, args.c_header, schema_hash),
            ),
            Output(
                args.python_output,
                generate_python(schema, args.schema, schema_hash),
            ),
        )
    except (OSError, json.JSONDecodeError, SchemaError) as exc:
        print(f"protocol generation failed: {exc}", file=sys.stderr)
        return 2

    if args.check:
        stale = [
            output.path
            for output in outputs
            if read_text(output.path) != output.content
        ]
        for path in stale:
            print(f"{display_path(path)} is absent or out of date")
        return 1 if stale else 0

    for output in outputs:
        write_text_atomic(output.path, output.content)
    return 0


def load_schema(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SchemaError("schema root must be an object")
    return data


def schema_sha256(schema: dict[str, Any]) -> str:
    canonical = json.dumps(
        schema,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def validate_schema(schema: dict[str, Any]) -> None:
    require_equal(schema, "schema_format_version", 1, "schema")
    require_identifier(schema, "protocol_name", "schema")
    protocol_version = require_int(schema, "protocol_version", "schema", 1, 15)
    require_equal(schema, "endianness", "little", "schema")
    require_equal(schema, "signed_integer_encoding", "twos_complement", "schema")

    control = require_object(schema, "control", "schema")
    validate_control(control, protocol_version)

    clear_header = require_object(schema, "clear_header", "schema")
    header_fields = validate_record(clear_header, "clear_header")
    require_exact_fields(
        header_fields,
        (
            ("control", "u8"),
            ("domain", "u8"),
            ("node_id", "bytes[8]"),
            ("message_id", "u32"),
        ),
        "clear_header",
    )

    domains = validate_domains(require_object(schema, "domains", "schema"))

    reset_reason_table = require_object(schema, "reset_reasons", "schema")
    ack_status_table = require_object(schema, "ack_statuses", "schema")
    reset_reasons = validate_named_values(reset_reason_table, "reset_reasons")
    ack_statuses = validate_named_values(ack_status_table, "ack_statuses")
    named_value_tables = {
        "reset_reasons": reset_reason_table,
        "ack_statuses": ack_status_table,
    }

    reading_body = require_object(schema, "reading_body", "schema")
    reading_fields = validate_record(reading_body, "reading_body")
    require_exact_fields(
        reading_fields,
        (
            ("sample_id", "u32"),
            ("run_ms", "u16"),
            ("soil_0_mv", "u16"),
            ("soil_1_mv", "u16"),
            ("soil_temp_0_centi_c", "i16"),
            ("soil_temp_1_centi_c", "i16"),
            ("enclosure_centi_c", "i16"),
            ("enclosure_pressure_pa", "u32"),
            ("enclosure_humidity_centi_pct", "u16"),
            ("reset_reason", "u8"),
            ("previous_current_tx_attempts", "u8"),
            ("previous_awake_ms", "u16"),
            ("previous_current_delivery_ms", "u16"),
            ("previous_cycle_tx_attempts", "u8"),
            ("previous_cycle_accepted_readings", "u8"),
            ("flags", "u16"),
        ),
        "reading_body",
    )
    reading_flags = validate_flags(reading_body, reading_fields)
    validate_field_metadata(
        reading_fields, reading_flags, named_value_tables, "reading_body"
    )
    validate_constraints(reading_body, reading_fields, reading_flags)

    ack_body = require_object(schema, "ack_body", "schema")
    ack_fields = validate_record(ack_body, "ack_body")
    require_exact_fields(ack_fields, (("status", "u8"),), "ack_body")
    validate_field_metadata(ack_fields, {}, named_value_tables, "ack_body")
    validate_ack_pairs(ack_body, domains, ack_statuses)

    ccm = require_object(schema, "ccm", "schema")
    validate_ccm(ccm, header_fields)
    validate_frames(
        require_object(schema, "frames", "schema"),
        domains,
        ccm,
        clear_header,
        reading_body,
        ack_body,
    )


def validate_control(control: dict[str, Any], protocol_version: int) -> None:
    require_equal(control, "type", "u8", "control")
    validate_bit_range(
        require_object(control, "protocol_version_bits", "control"),
        7,
        4,
        "control.protocol_version_bits",
    )
    validate_bit_range(
        require_object(control, "protocol_flags_bits", "control"),
        3,
        0,
        "control.protocol_flags_bits",
    )
    require_equal(control, "protocol_version_zero_reserved", True, "control")
    flags = require_int(control, "pilot_protocol_flags", "control", 0, 0x0F)
    value = require_int(control, "pilot_value", "control", 0, 0xFF)
    expected = (protocol_version << 4) | flags
    if value != expected:
        raise SchemaError(
            f"control.pilot_value is {value}, expected {expected} from version and flags"
        )
    require_hex_matches(control, "pilot_value_hex", value, "control")


def validate_bit_range(
    bit_range: dict[str, Any], expected_msb: int, expected_lsb: int, context: str
) -> None:
    require_equal(bit_range, "most_significant_bit", expected_msb, context)
    require_equal(bit_range, "least_significant_bit", expected_lsb, context)


def validate_record(
    record: dict[str, Any], context: str, *, descriptions_required: bool = True
) -> list[dict[str, Any]]:
    wire_size = require_int(record, "wire_size_bytes", context, 1, 65535)
    fields = require_list(record, "fields", context)
    if not fields:
        raise SchemaError(f"{context}.fields must not be empty")

    result: list[dict[str, Any]] = []
    seen_names: set[str] = set()
    expected_offset = 0
    for index, raw_field in enumerate(fields):
        field_context = f"{context}.fields[{index}]"
        if not isinstance(raw_field, dict):
            raise SchemaError(f"{field_context} must be an object")
        name = require_identifier(raw_field, "name", field_context)
        if name in seen_names:
            raise SchemaError(f"{context} contains duplicate field {name}")
        seen_names.add(name)
        field_type = require_string(raw_field, "type", field_context)
        size = type_info(field_type)["size"]
        offset = require_int(raw_field, "offset", field_context, 0, wire_size)
        if offset != expected_offset:
            raise SchemaError(
                f"{field_context}.offset is {offset}, expected contiguous offset "
                f"{expected_offset}"
            )
        if descriptions_required:
            description = require_string(raw_field, "description", field_context)
            if not description.strip():
                raise SchemaError(f"{field_context}.description must not be empty")
        expected_offset += size
        result.append(raw_field)

    if expected_offset != wire_size:
        raise SchemaError(
            f"{context}.wire_size_bytes is {wire_size}, fields occupy {expected_offset}"
        )
    return result


def require_exact_fields(
    fields: list[dict[str, Any]],
    expected: tuple[tuple[str, str], ...],
    context: str,
) -> None:
    actual = tuple((field["name"], field["type"]) for field in fields)
    if actual != expected:
        raise SchemaError(f"{context} fields are {actual!r}, expected {expected!r}")


def validate_domains(domains: dict[str, Any]) -> dict[str, dict[str, Any]]:
    require_equal(domains, "zero_reserved", True, "domains")
    values = require_list(domains, "values", "domains")
    if not values:
        raise SchemaError("domains.values must not be empty")

    by_name: dict[str, dict[str, Any]] = {}
    seen_values: set[int] = set()
    for index, raw_domain in enumerate(values):
        context = f"domains.values[{index}]"
        if not isinstance(raw_domain, dict):
            raise SchemaError(f"{context} must be an object")
        name = require_identifier(raw_domain, "name", context)
        value = require_int(raw_domain, "value", context, 1, 255)
        direction = require_string(raw_domain, "direction", context)
        if direction not in {"uplink", "downlink"}:
            raise SchemaError(f"{context}.direction must be uplink or downlink")
        body = require_string(raw_domain, "body", context)
        if body not in {"reading_body", "ack_body"}:
            raise SchemaError(f"{context}.body is unsupported: {body}")
        if name in by_name:
            raise SchemaError(f"duplicate domain name: {name}")
        if value in seen_values:
            raise SchemaError(f"duplicate domain value: {value}")
        by_name[name] = raw_domain
        seen_values.add(value)

    return by_name


def validate_named_values(
    table: dict[str, Any], context: str
) -> dict[str, dict[str, Any]]:
    value_type = require_string(table, "type", context)
    info = type_info(value_type)
    if value_type.startswith("i"):
        minimum = -(1 << (info["size"] * 8 - 1))
        maximum = (1 << (info["size"] * 8 - 1)) - 1
    else:
        minimum = 0
        maximum = (1 << (info["size"] * 8)) - 1
    require_bool(table, "allow_unassigned_values", context)

    values = require_list(table, "values", context)
    if not values:
        raise SchemaError(f"{context}.values must not be empty")

    by_name: dict[str, dict[str, Any]] = {}
    seen_values: set[int] = set()
    for index, raw_value in enumerate(values):
        value_context = f"{context}.values[{index}]"
        if not isinstance(raw_value, dict):
            raise SchemaError(f"{value_context} must be an object")
        name = require_identifier(raw_value, "name", value_context)
        value = require_int(raw_value, "value", value_context, minimum, maximum)
        if name in by_name:
            raise SchemaError(f"{context} contains duplicate name {name}")
        if value in seen_values:
            raise SchemaError(f"{context} contains duplicate value {value}")
        by_name[name] = raw_value
        seen_values.add(value)
    return by_name


def validate_flags(
    reading_body: dict[str, Any], fields: list[dict[str, Any]]
) -> dict[str, dict[str, Any]]:
    flags = require_object(reading_body, "flags", "reading_body")
    flag_field_name = require_string(flags, "field", "reading_body.flags")
    fields_by_name = {field["name"]: field for field in fields}
    if flag_field_name not in fields_by_name:
        raise SchemaError(f"reading_body.flags.field is unknown: {flag_field_name}")
    flag_type = require_string(flags, "type", "reading_body.flags")
    if fields_by_name[flag_field_name]["type"] != flag_type:
        raise SchemaError("reading_body.flags.type does not match its field")
    bit_count = type_info(flag_type)["size"] * 8

    values = require_list(flags, "values", "reading_body.flags")
    by_name: dict[str, dict[str, Any]] = {}
    assigned_bits: set[int] = set()
    for index, raw_flag in enumerate(values):
        context = f"reading_body.flags.values[{index}]"
        if not isinstance(raw_flag, dict):
            raise SchemaError(f"{context} must be an object")
        name = require_identifier(raw_flag, "name", context)
        bit = require_int(raw_flag, "bit", context, 0, bit_count - 1)
        require_string(raw_flag, "description", context)
        if name in by_name:
            raise SchemaError(f"duplicate reading flag name: {name}")
        if bit in assigned_bits:
            raise SchemaError(f"duplicate reading flag bit: {bit}")
        by_name[name] = raw_flag
        assigned_bits.add(bit)

    reserved_bits_raw = require_list(flags, "reserved_bits", "reading_body.flags")
    reserved_bits: set[int] = set()
    for index, raw_bit in enumerate(reserved_bits_raw):
        if not isinstance(raw_bit, int) or isinstance(raw_bit, bool):
            raise SchemaError(f"reading_body.flags.reserved_bits[{index}] is not an int")
        if raw_bit < 0 or raw_bit >= bit_count:
            raise SchemaError(f"reserved flag bit is out of range: {raw_bit}")
        if raw_bit in reserved_bits:
            raise SchemaError(f"duplicate reserved flag bit: {raw_bit}")
        reserved_bits.add(raw_bit)

    if assigned_bits & reserved_bits:
        raise SchemaError("assigned and reserved flag bits overlap")
    if assigned_bits | reserved_bits != set(range(bit_count)):
        raise SchemaError("assigned and reserved flag bits must cover the flag field")

    expected_mask = sum(1 << bit for bit in reserved_bits)
    mask = require_int(
        flags, "reserved_mask", "reading_body.flags", 0, (1 << bit_count) - 1
    )
    if mask != expected_mask:
        raise SchemaError(
            f"reading_body.flags.reserved_mask is {mask}, expected {expected_mask}"
        )
    require_hex_matches(flags, "reserved_mask_hex", mask, "reading_body.flags")
    return by_name


def validate_field_metadata(
    fields: list[dict[str, Any]],
    flags: dict[str, dict[str, Any]],
    named_value_tables: dict[str, dict[str, Any]],
    context: str,
) -> None:
    for field in fields:
        field_context = f"{context}.{field['name']}"
        if "valid_flag" in field:
            flag = field["valid_flag"]
            if not isinstance(flag, str) or flag not in flags:
                raise SchemaError(f"{field_context}.valid_flag is unknown: {flag!r}")
        if "named_values" in field:
            table_name = field["named_values"]
            if (
                not isinstance(table_name, str)
                or table_name not in named_value_tables
            ):
                raise SchemaError(
                    f"{field_context}.named_values is unknown: {table_name!r}"
                )
            table = named_value_tables[table_name]
            if table["type"] != field["type"]:
                raise SchemaError(
                    f"{field_context} type does not match {table_name}"
                )
            allow = field.get("allow_unassigned_values")
            if not isinstance(allow, bool):
                raise SchemaError(
                    f"{field_context}.allow_unassigned_values must be a bool"
                )
            if allow != table["allow_unassigned_values"]:
                raise SchemaError(
                    f"{field_context}.allow_unassigned_values does not match "
                    f"{table_name}"
                )


def validate_constraints(
    reading_body: dict[str, Any],
    fields: list[dict[str, Any]],
    flags: dict[str, dict[str, Any]],
) -> None:
    constraints = require_list(
        reading_body, "structural_constraints", "reading_body"
    )
    fields_by_name = {field["name"]: field for field in fields}

    for index, raw_constraint in enumerate(constraints):
        context = f"reading_body.structural_constraints[{index}]"
        if not isinstance(raw_constraint, dict):
            raise SchemaError(f"{context} must be an object")
        kind = require_string(raw_constraint, "kind", context)
        if kind not in SUPPORTED_CONSTRAINTS:
            raise SchemaError(f"{context}.kind is unsupported: {kind}")

        if kind == "zero_unless_flag":
            validate_field_ref(raw_constraint, "field", fields_by_name, context)
            validate_flag_ref(raw_constraint, "flag", flags, context)
        elif kind == "all_zero_unless_flag":
            referenced_fields = require_list(raw_constraint, "fields", context)
            if not referenced_fields:
                raise SchemaError(f"{context}.fields must not be empty")
            for field_name in referenced_fields:
                if not isinstance(field_name, str) or field_name not in fields_by_name:
                    raise SchemaError(f"{context} references unknown field {field_name!r}")
            validate_flag_ref(raw_constraint, "flag", flags, context)
        elif kind == "flag_iff_field_equals":
            field = validate_field_ref(
                raw_constraint, "field", fields_by_name, context
            )
            validate_flag_ref(raw_constraint, "flag", flags, context)
            info = type_info(field["type"])
            maximum = (1 << (info["size"] * 8)) - 1
            require_int(raw_constraint, "value", context, 0, maximum)
        elif kind == "flag_requires_flag":
            validate_flag_ref(raw_constraint, "flag", flags, context)
            validate_flag_ref(raw_constraint, "required_flag", flags, context)
        elif kind == "reserved_bits_zero":
            field = validate_field_ref(
                raw_constraint, "field", fields_by_name, context
            )
            info = type_info(field["type"])
            mask = require_int(
                raw_constraint,
                "mask",
                context,
                0,
                (1 << (info["size"] * 8)) - 1,
            )
            require_hex_matches(raw_constraint, "mask_hex", mask, context)


def validate_field_ref(
    value: dict[str, Any],
    key: str,
    fields: dict[str, dict[str, Any]],
    context: str,
) -> dict[str, Any]:
    field_name = require_string(value, key, context)
    if field_name not in fields:
        raise SchemaError(f"{context}.{key} is unknown: {field_name}")
    return fields[field_name]


def validate_flag_ref(
    value: dict[str, Any],
    key: str,
    flags: dict[str, dict[str, Any]],
    context: str,
) -> dict[str, Any]:
    flag_name = require_string(value, key, context)
    if flag_name not in flags:
        raise SchemaError(f"{context}.{key} is unknown: {flag_name}")
    return flags[flag_name]


def validate_ack_pairs(
    ack_body: dict[str, Any],
    domains: dict[str, dict[str, Any]],
    statuses: dict[str, dict[str, Any]],
) -> None:
    raw_pairs = require_list(ack_body, "domain_status_pairs", "ack_body")
    pairs: dict[str, str] = {}
    used_statuses: set[str] = set()
    for index, raw_pair in enumerate(raw_pairs):
        context = f"ack_body.domain_status_pairs[{index}]"
        if not isinstance(raw_pair, dict):
            raise SchemaError(f"{context} must be an object")
        domain = require_string(raw_pair, "domain", context)
        status = require_string(raw_pair, "status", context)
        if domain not in domains or domains[domain]["body"] != "ack_body":
            raise SchemaError(f"{context}.domain is not an ACK domain: {domain}")
        if status not in statuses:
            raise SchemaError(f"{context}.status is unknown: {status}")
        if domain in pairs:
            raise SchemaError(f"ACK domain is mapped more than once: {domain}")
        if status in used_statuses:
            raise SchemaError(f"ACK status is mapped more than once: {status}")
        pairs[domain] = status
        used_statuses.add(status)

    expected_domains = {
        name for name, domain in domains.items() if domain["body"] == "ack_body"
    }
    if set(pairs) != expected_domains:
        raise SchemaError("ACK domain/status pairs must cover every ACK domain")
    if used_statuses != set(statuses):
        raise SchemaError("ACK domain/status pairs must cover every ACK status")


def validate_ccm(
    ccm: dict[str, Any], header_fields: list[dict[str, Any]]
) -> None:
    require_equal(ccm, "algorithm", "AES-CCM-128", "ccm")
    require_equal(ccm, "key_size_bytes", 16, "ccm")
    nonce_size = require_int(ccm, "nonce_size_bytes", "ccm", 7, 13)
    tag_size = require_int(ccm, "tag_size_bytes", "ccm", 4, 16)
    if tag_size not in {4, 6, 8, 10, 12, 14, 16}:
        raise SchemaError("ccm.tag_size_bytes is not an AES-CCM tag size")

    aad = require_object(ccm, "aad", "ccm")
    require_equal(aad, "source", "clear_header", "ccm.aad")
    header_size = sum(type_info(field["type"])["size"] for field in header_fields)
    require_equal(aad, "size_bytes", header_size, "ccm.aad")
    aad_fields = require_list(aad, "fields", "ccm.aad")
    if aad_fields != [field["name"] for field in header_fields]:
        raise SchemaError("ccm.aad.fields must contain the complete clear header")

    nonce = require_object(ccm, "nonce", "ccm")
    require_equal(nonce, "size_bytes", nonce_size, "ccm.nonce")
    nonce_fields = validate_record(
        {
            "wire_size_bytes": nonce["size_bytes"],
            "fields": nonce.get("fields"),
        },
        "ccm.nonce",
        descriptions_required=False,
    )
    header_by_name = {field["name"]: field for field in header_fields}
    for field in nonce_fields:
        require_equal(field, "source", "clear_header", f"ccm.nonce.{field['name']}")
        header_field = header_by_name.get(field["name"])
        if header_field is None or header_field["type"] != field["type"]:
            raise SchemaError(
                f"ccm.nonce field {field['name']} does not match clear_header"
            )


def validate_frames(
    frames: dict[str, Any],
    domains: dict[str, dict[str, Any]],
    ccm: dict[str, Any],
    clear_header: dict[str, Any],
    reading_body: dict[str, Any],
    ack_body: dict[str, Any],
) -> None:
    header_size = clear_header["wire_size_bytes"]
    tag_size = ccm["tag_size_bytes"]
    for frame_name, body_name, body in (
        ("reading", "reading_body", reading_body),
        ("ack", "ack_body", ack_body),
    ):
        frame = require_object(frames, frame_name, "frames")
        frame_context = f"frames.{frame_name}"
        frame_domains = require_list(frame, "domains", frame_context)
        expected_domains = [
            name for name, domain in domains.items() if domain["body"] == body_name
        ]
        if frame_domains != expected_domains:
            raise SchemaError(
                f"{frame_context}.domains is {frame_domains}, expected {expected_domains}"
            )
        require_equal(
            frame, "clear_header_size_bytes", header_size, frame_context
        )
        require_equal(
            frame, "plaintext_body_size_bytes", body["wire_size_bytes"], frame_context
        )
        require_equal(
            frame, "ciphertext_body_size_bytes", body["wire_size_bytes"], frame_context
        )
        require_equal(frame, "ccm_tag_size_bytes", tag_size, frame_context)
        expected_frame_size = header_size + body["wire_size_bytes"] + tag_size
        require_equal(
            frame, "sx1262_payload_size_bytes", expected_frame_size, frame_context
        )


def type_info(type_name: str) -> dict[str, Any]:
    if type_name in TYPE_INFO:
        return TYPE_INFO[type_name]
    match = BYTES_TYPE_RE.fullmatch(type_name)
    if match is None:
        raise SchemaError(f"unsupported field type: {type_name}")
    size = int(match.group(1))
    if size <= 0:
        raise SchemaError(f"byte-array size must be positive: {type_name}")
    return {
        "size": size,
        "c": "uint8_t",
        "python": "bytes",
        "struct": f"{size}s",
    }


def generate_c_header(
    schema: dict[str, Any], schema_path: Path, schema_hash: str
) -> str:
    header = schema["clear_header"]
    reading = schema["reading_body"]
    ack = schema["ack_body"]
    domains = schema["domains"]["values"]
    flags = reading["flags"]["values"]
    reset_reasons = schema["reset_reasons"]["values"]
    ack_statuses = schema["ack_statuses"]["values"]
    frames = schema["frames"]

    lines = [
        c_banner(schema_path, schema_hash),
        "#pragma once",
        "",
        "#include <stdbool.h>",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        "#ifdef __cplusplus",
        'extern "C" {',
        "#endif",
        "",
        f'#define {C_PREFIX}_SCHEMA_SHA256 "{schema_hash}"',
        f"#define {C_PREFIX}_PROTOCOL_VERSION UINT8_C({schema['protocol_version']})",
        f"#define {C_PREFIX}_CONTROL UINT8_C({c_hex(schema['control']['pilot_value'], 2)})",
        f"#define {C_PREFIX}_KEY_SIZE {schema['ccm']['key_size_bytes']}u",
        f"#define {C_PREFIX}_NONCE_SIZE {schema['ccm']['nonce_size_bytes']}u",
        f"#define {C_PREFIX}_TAG_SIZE {schema['ccm']['tag_size_bytes']}u",
        f"#define {C_PREFIX}_CLEAR_HEADER_SIZE {header['wire_size_bytes']}u",
        f"#define {C_PREFIX}_READING_BODY_SIZE {reading['wire_size_bytes']}u",
        f"#define {C_PREFIX}_ACK_BODY_SIZE {ack['wire_size_bytes']}u",
        f"#define {C_PREFIX}_READING_FRAME_SIZE "
        f"{frames['reading']['sx1262_payload_size_bytes']}u",
        f"#define {C_PREFIX}_ACK_FRAME_SIZE "
        f"{frames['ack']['sx1262_payload_size_bytes']}u",
        "",
    ]

    lines.extend(c_offset_macros("CLEAR_HEADER", header["fields"]))
    lines.append("")
    lines.extend(c_offset_macros("NONCE", schema["ccm"]["nonce"]["fields"]))
    lines.append("")
    lines.extend(c_offset_macros("READING", reading["fields"]))
    lines.append("")
    lines.extend(c_offset_macros("ACK", ack["fields"]))
    lines.extend(
        [
            "",
            f"typedef uint8_t {C_TYPE_PREFIX}_domain_t;",
        ]
    )
    for domain in domains:
        lines.append(
            f"#define {C_PREFIX}_DOMAIN_{domain['name']} "
            f"UINT8_C({c_hex(domain['value'], 2)})"
        )

    lines.extend(["", f"typedef uint8_t {C_TYPE_PREFIX}_ack_status_t;"])
    for status in ack_statuses:
        lines.append(
            f"#define {C_PREFIX}_ACK_STATUS_{status['name']} "
            f"UINT8_C({status['value']})"
        )

    lines.extend(["", f"typedef uint8_t {C_TYPE_PREFIX}_reset_reason_t;"])
    for reason in reset_reasons:
        lines.append(
            f"#define {C_PREFIX}_RESET_REASON_{reason['name']} "
            f"UINT8_C({reason['value']})"
        )

    lines.extend(["", f"typedef uint16_t {C_TYPE_PREFIX}_reading_flags_t;"])
    for flag in flags:
        lines.append(
            f"#define {C_PREFIX}_FLAG_{flag['name']} "
            f"UINT16_C({c_hex(1 << flag['bit'], 4)})"
        )
    lines.append(
        f"#define {C_PREFIX}_READING_RESERVED_FLAGS_MASK "
        f"UINT16_C({c_hex(reading['flags']['reserved_mask'], 4)})"
    )

    lines.extend(
        [
            "",
            "typedef enum {",
            f"  {C_PREFIX}_CODEC_OK = 0,",
            f"  {C_PREFIX}_CODEC_INVALID_ARGUMENT,",
            f"  {C_PREFIX}_CODEC_BUFFER_TOO_SMALL,",
            f"  {C_PREFIX}_CODEC_INVALID_LENGTH,",
            f"  {C_PREFIX}_CODEC_MALFORMED,",
            f"}} {C_TYPE_PREFIX}_codec_result_t;",
            "",
            "typedef struct {",
            f"  uint8_t bytes[{C_PREFIX}_READING_FRAME_SIZE];",
            f"}} {C_TYPE_PREFIX}_authenticated_reading_frame_t;",
            "",
            "#if defined(__cplusplus)",
            f"static_assert(sizeof({C_TYPE_PREFIX}_authenticated_reading_frame_t) ==",
            f"                  {C_PREFIX}_READING_FRAME_SIZE,",
            '              "authenticated reading frame size mismatch");',
            "#else",
            f"_Static_assert(sizeof({C_TYPE_PREFIX}_authenticated_reading_frame_t) ==",
            f"                   {C_PREFIX}_READING_FRAME_SIZE,",
            '               "authenticated reading frame size mismatch");',
            "#endif",
            "",
            "typedef struct {",
        ]
    )
    lines.extend(c_struct_fields(header["fields"]))
    lines.extend(
        [
            f"}} {C_TYPE_PREFIX}_clear_header_t;",
            "",
            "typedef struct {",
        ]
    )
    lines.extend(c_struct_fields(reading["fields"]))
    lines.extend(
        [
            f"}} {C_TYPE_PREFIX}_reading_t;",
            "",
            "typedef struct {",
        ]
    )
    lines.extend(c_struct_fields(ack["fields"]))
    lines.extend(
        [
            f"}} {C_TYPE_PREFIX}_ack_t;",
            "",
            f"bool {C_TYPE_PREFIX}_is_supported_control(uint8_t control);",
            f"bool {C_TYPE_PREFIX}_domain_is_reading(uint8_t domain);",
            f"bool {C_TYPE_PREFIX}_domain_is_ack(uint8_t domain);",
            f"bool {C_TYPE_PREFIX}_ack_status_matches_domain(",
            "    uint8_t domain, uint8_t status);",
            "",
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_validate_reading(",
            f"    const {C_TYPE_PREFIX}_reading_t *reading);",
            "",
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_encode_clear_header(",
            "    uint8_t *output, size_t output_size,",
            f"    const {C_TYPE_PREFIX}_clear_header_t *header);",
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_decode_clear_header(",
            f"    {C_TYPE_PREFIX}_clear_header_t *header,",
            "    const uint8_t *input, size_t input_size);",
            "",
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_build_nonce(",
            "    uint8_t *output, size_t output_size,",
            f"    const {C_TYPE_PREFIX}_clear_header_t *header);",
            "",
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_encode_reading(",
            "    uint8_t *output, size_t output_size,",
            f"    const {C_TYPE_PREFIX}_reading_t *reading);",
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_decode_reading(",
            f"    {C_TYPE_PREFIX}_reading_t *reading,",
            "    const uint8_t *input, size_t input_size);",
            "",
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_encode_ack(",
            "    uint8_t *output, size_t output_size,",
            f"    const {C_TYPE_PREFIX}_ack_t *ack);",
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_decode_ack(",
            f"    {C_TYPE_PREFIX}_ack_t *ack,",
            "    const uint8_t *input, size_t input_size);",
            "",
            "#ifdef __cplusplus",
            "}",
            "#endif",
            "",
        ]
    )
    return "\n".join(lines)


def generate_c_source(
    schema: dict[str, Any],
    schema_path: Path,
    c_header_path: Path,
    schema_hash: str,
) -> str:
    header_fields = schema["clear_header"]["fields"]
    nonce_fields = schema["ccm"]["nonce"]["fields"]
    reading_fields = schema["reading_body"]["fields"]
    ack_fields = schema["ack_body"]["fields"]
    reading_domains = [
        domain
        for domain in schema["domains"]["values"]
        if domain["body"] == "reading_body"
    ]
    ack_domains = [
        domain
        for domain in schema["domains"]["values"]
        if domain["body"] == "ack_body"
    ]
    ack_pairs = schema["ack_body"]["domain_status_pairs"]
    ack_status_by_name = {
        value["name"]: value for value in schema["ack_statuses"]["values"]
    }

    lines = [
        c_banner(schema_path, schema_hash),
        f'#include "{c_header_path.name}"',
        "",
        "#include <limits.h>",
        "#include <string.h>",
        "",
        '_Static_assert(CHAR_BIT == 8, "protocol requires 8-bit bytes");',
        '_Static_assert(sizeof(uint16_t) == 2, "protocol requires 16-bit uint16_t");',
        '_Static_assert(sizeof(uint32_t) == 4, "protocol requires 32-bit uint32_t");',
        '_Static_assert(INT16_MIN == -32768, "protocol requires 16-bit int16_t");',
        '_Static_assert(INT16_MAX == 32767, "protocol requires 16-bit int16_t");',
        "",
        "static void write_u16_le(uint8_t *output, uint16_t value) {",
        "  output[0] = (uint8_t)value;",
        "  output[1] = (uint8_t)(value >> 8);",
        "}",
        "",
        "static void write_u32_le(uint8_t *output, uint32_t value) {",
        "  output[0] = (uint8_t)value;",
        "  output[1] = (uint8_t)(value >> 8);",
        "  output[2] = (uint8_t)(value >> 16);",
        "  output[3] = (uint8_t)(value >> 24);",
        "}",
        "",
        "static uint16_t read_u16_le(const uint8_t *input) {",
        "  return (uint16_t)input[0] | ((uint16_t)input[1] << 8);",
        "}",
        "",
        "static uint32_t read_u32_le(const uint8_t *input) {",
        "  return (uint32_t)input[0] | ((uint32_t)input[1] << 8) |",
        "         ((uint32_t)input[2] << 16) | ((uint32_t)input[3] << 24);",
        "}",
        "",
        "static int16_t read_i16_le(const uint8_t *input) {",
        "  const uint16_t raw = read_u16_le(input);",
        "  if (raw <= (uint16_t)INT16_MAX) {",
        "    return (int16_t)raw;",
        "  }",
        "  return (int16_t)(-1 - (int32_t)(UINT16_MAX - raw));",
        "}",
        "",
        f"bool {C_TYPE_PREFIX}_is_supported_control(uint8_t control) {{",
        f"  return control == {C_PREFIX}_CONTROL;",
        "}",
        "",
        f"bool {C_TYPE_PREFIX}_domain_is_reading(uint8_t domain) {{",
        "  switch (domain) {",
    ]
    for domain in reading_domains:
        lines.append(f"  case {C_PREFIX}_DOMAIN_{domain['name']}:")
    lines.extend(
        [
            "    return true;",
            "  default:",
            "    return false;",
            "  }",
            "}",
            "",
            f"bool {C_TYPE_PREFIX}_domain_is_ack(uint8_t domain) {{",
            "  switch (domain) {",
        ]
    )
    for domain in ack_domains:
        lines.append(f"  case {C_PREFIX}_DOMAIN_{domain['name']}:")
    lines.extend(
        [
            "    return true;",
            "  default:",
            "    return false;",
            "  }",
            "}",
            "",
            f"bool {C_TYPE_PREFIX}_ack_status_matches_domain(",
            "    uint8_t domain, uint8_t status) {",
            "  switch (domain) {",
        ]
    )
    for pair in ack_pairs:
        lines.extend(
            [
                f"  case {C_PREFIX}_DOMAIN_{pair['domain']}:",
                f"    return status == {C_PREFIX}_ACK_STATUS_{pair['status']};",
            ]
        )
    lines.extend(
        [
            "  default:",
            "    return false;",
            "  }",
            "}",
            "",
            "static bool ack_status_is_known(uint8_t status) {",
            "  switch (status) {",
        ]
    )
    for status in ack_status_by_name.values():
        lines.append(f"  case {C_PREFIX}_ACK_STATUS_{status['name']}:")
    lines.extend(
        [
            "    return true;",
            "  default:",
            "    return false;",
            "  }",
            "}",
            "",
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_validate_reading(",
            f"    const {C_TYPE_PREFIX}_reading_t *reading) {{",
            "  if (reading == NULL) {",
            f"    return {C_PREFIX}_CODEC_INVALID_ARGUMENT;",
            "  }",
        ]
    )
    lines.extend(c_validation_lines(schema["reading_body"]))
    lines.extend(
        [
            f"  return {C_PREFIX}_CODEC_OK;",
            "}",
            "",
        ]
    )

    lines.extend(
        c_encode_function(
            "clear_header",
            f"{C_TYPE_PREFIX}_clear_header_t",
            f"{C_PREFIX}_CLEAR_HEADER_SIZE",
            header_fields,
            validate_expression=None,
        )
    )
    lines.extend(
        c_decode_function(
            "clear_header",
            f"{C_TYPE_PREFIX}_clear_header_t",
            f"{C_PREFIX}_CLEAR_HEADER_SIZE",
            header_fields,
            validate_expression=None,
        )
    )

    lines.extend(
        [
            f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_build_nonce(",
            "    uint8_t *output, size_t output_size,",
            f"    const {C_TYPE_PREFIX}_clear_header_t *header) {{",
            "  if (output == NULL || header == NULL) {",
            f"    return {C_PREFIX}_CODEC_INVALID_ARGUMENT;",
            "  }",
            f"  if (output_size < {C_PREFIX}_NONCE_SIZE) {{",
            f"    return {C_PREFIX}_CODEC_BUFFER_TOO_SMALL;",
            "  }",
        ]
    )
    lines.extend(c_encode_fields(nonce_fields, "header", "NONCE"))
    lines.extend(
        [
            f"  return {C_PREFIX}_CODEC_OK;",
            "}",
            "",
        ]
    )

    lines.extend(
        c_encode_function(
            "reading",
            f"{C_TYPE_PREFIX}_reading_t",
            f"{C_PREFIX}_READING_BODY_SIZE",
            reading_fields,
            validate_expression=f"{C_TYPE_PREFIX}_validate_reading(value)",
        )
    )
    lines.extend(
        c_decode_function(
            "reading",
            f"{C_TYPE_PREFIX}_reading_t",
            f"{C_PREFIX}_READING_BODY_SIZE",
            reading_fields,
            validate_expression=f"{C_TYPE_PREFIX}_validate_reading(value)",
        )
    )
    lines.extend(
        c_encode_function(
            "ack",
            f"{C_TYPE_PREFIX}_ack_t",
            f"{C_PREFIX}_ACK_BODY_SIZE",
            ack_fields,
            validate_expression=(
                f"ack_status_is_known(value->status) ? {C_PREFIX}_CODEC_OK "
                f": {C_PREFIX}_CODEC_MALFORMED"
            ),
        )
    )
    lines.extend(
        c_decode_function(
            "ack",
            f"{C_TYPE_PREFIX}_ack_t",
            f"{C_PREFIX}_ACK_BODY_SIZE",
            ack_fields,
            validate_expression=(
                f"ack_status_is_known(value->status) ? {C_PREFIX}_CODEC_OK "
                f": {C_PREFIX}_CODEC_MALFORMED"
            ),
        )
    )
    return "\n".join(lines)


def c_offset_macros(scope: str, fields: list[dict[str, Any]]) -> list[str]:
    return [
        f"#define {C_PREFIX}_{scope}_{field['name'].upper()}_OFFSET "
        f"{field['offset']}u"
        for field in fields
    ]


def c_struct_fields(fields: list[dict[str, Any]]) -> list[str]:
    lines: list[str] = []
    for field in fields:
        info = type_info(field["type"])
        if field["type"].startswith("bytes["):
            lines.append(f"  uint8_t {field['name']}[{info['size']}];")
        else:
            lines.append(f"  {info['c']} {field['name']};")
    return lines


def c_encode_function(
    name: str,
    c_type: str,
    size_macro: str,
    fields: list[dict[str, Any]],
    validate_expression: str | None,
) -> list[str]:
    lines = [
        f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_encode_{name}(",
        "    uint8_t *output, size_t output_size,",
        f"    const {c_type} *value) {{",
        "  if (output == NULL || value == NULL) {",
        f"    return {C_PREFIX}_CODEC_INVALID_ARGUMENT;",
        "  }",
        f"  if (output_size < {size_macro}) {{",
        f"    return {C_PREFIX}_CODEC_BUFFER_TOO_SMALL;",
        "  }",
    ]
    if validate_expression is not None:
        lines.extend(
            [
                f"  const {C_TYPE_PREFIX}_codec_result_t validation = "
                f"{validate_expression};",
                f"  if (validation != {C_PREFIX}_CODEC_OK) {{",
                "    return validation;",
                "  }",
            ]
        )
    lines.extend(c_encode_fields(fields, "value", name.upper()))
    lines.extend([f"  return {C_PREFIX}_CODEC_OK;", "}", ""])
    return lines


def c_decode_function(
    name: str,
    c_type: str,
    size_macro: str,
    fields: list[dict[str, Any]],
    validate_expression: str | None,
) -> list[str]:
    lines = [
        f"{C_TYPE_PREFIX}_codec_result_t {C_TYPE_PREFIX}_decode_{name}(",
        f"    {c_type} *value,",
        "    const uint8_t *input, size_t input_size) {",
        "  if (value == NULL || input == NULL) {",
        f"    return {C_PREFIX}_CODEC_INVALID_ARGUMENT;",
        "  }",
        f"  if (input_size != {size_macro}) {{",
        f"    return {C_PREFIX}_CODEC_INVALID_LENGTH;",
        "  }",
    ]
    lines.extend(c_decode_fields(fields, "value", name.upper()))
    if validate_expression is not None:
        lines.append(f"  return {validate_expression};")
    else:
        lines.append(f"  return {C_PREFIX}_CODEC_OK;")
    lines.extend(["}", ""])
    return lines


def c_encode_fields(
    fields: list[dict[str, Any]], value_name: str, scope: str
) -> list[str]:
    lines: list[str] = []
    for field in fields:
        offset = f"{C_PREFIX}_{scope}_{field['name'].upper()}_OFFSET"
        access = f"{value_name}->{field['name']}"
        field_type = field["type"]
        if field_type == "u8":
            lines.append(f"  output[{offset}] = {access};")
        elif field_type == "u16":
            lines.append(f"  write_u16_le(&output[{offset}], {access});")
        elif field_type == "u32":
            lines.append(f"  write_u32_le(&output[{offset}], {access});")
        elif field_type == "i16":
            lines.append(
                f"  write_u16_le(&output[{offset}], (uint16_t){access});"
            )
        elif field_type.startswith("bytes["):
            lines.append(
                f"  memcpy(&output[{offset}], {access}, sizeof({access}));"
            )
        else:
            raise AssertionError(f"unsupported type after validation: {field_type}")
    return lines


def c_decode_fields(
    fields: list[dict[str, Any]], value_name: str, scope: str
) -> list[str]:
    lines: list[str] = []
    for field in fields:
        offset = f"{C_PREFIX}_{scope}_{field['name'].upper()}_OFFSET"
        access = f"{value_name}->{field['name']}"
        field_type = field["type"]
        if field_type == "u8":
            lines.append(f"  {access} = input[{offset}];")
        elif field_type == "u16":
            lines.append(f"  {access} = read_u16_le(&input[{offset}]);")
        elif field_type == "u32":
            lines.append(f"  {access} = read_u32_le(&input[{offset}]);")
        elif field_type == "i16":
            lines.append(f"  {access} = read_i16_le(&input[{offset}]);")
        elif field_type.startswith("bytes["):
            lines.append(
                f"  memcpy({access}, &input[{offset}], sizeof({access}));"
            )
        else:
            raise AssertionError(f"unsupported type after validation: {field_type}")
    return lines


def c_validation_lines(reading_body: dict[str, Any]) -> list[str]:
    lines: list[str] = []
    for constraint in reading_body["structural_constraints"]:
        kind = constraint["kind"]
        if kind == "zero_unless_flag":
            flag = f"{C_PREFIX}_FLAG_{constraint['flag']}"
            lines.extend(
                [
                    f"  if ((reading->flags & {flag}) == 0u &&",
                    f"      reading->{constraint['field']} != 0) {{",
                    f"    return {C_PREFIX}_CODEC_MALFORMED;",
                    "  }",
                ]
            )
        elif kind == "all_zero_unless_flag":
            flag = f"{C_PREFIX}_FLAG_{constraint['flag']}"
            tests = [f"reading->{field} != 0" for field in constraint["fields"]]
            lines.append(f"  if ((reading->flags & {flag}) == 0u &&")
            for index, test in enumerate(tests):
                prefix = "      (" if index == 0 else "       "
                suffix = " ||" if index + 1 < len(tests) else ")) {"
                lines.append(f"{prefix}{test}{suffix}")
            lines.extend(
                [
                    f"    return {C_PREFIX}_CODEC_MALFORMED;",
                    "  }",
                ]
            )
        elif kind == "flag_iff_field_equals":
            flag = f"{C_PREFIX}_FLAG_{constraint['flag']}"
            lines.extend(
                [
                    f"  if (((reading->flags & {flag}) != 0u) !=",
                    f"      (reading->{constraint['field']} == {constraint['value']}u)) {{",
                    f"    return {C_PREFIX}_CODEC_MALFORMED;",
                    "  }",
                ]
            )
        elif kind == "flag_requires_flag":
            flag = f"{C_PREFIX}_FLAG_{constraint['flag']}"
            required = f"{C_PREFIX}_FLAG_{constraint['required_flag']}"
            lines.extend(
                [
                    f"  if ((reading->flags & {flag}) != 0u &&",
                    f"      (reading->flags & {required}) == 0u) {{",
                    f"    return {C_PREFIX}_CODEC_MALFORMED;",
                    "  }",
                ]
            )
        elif kind == "reserved_bits_zero":
            lines.extend(
                [
                    f"  if ((reading->{constraint['field']} &",
                    f"       {C_PREFIX}_READING_RESERVED_FLAGS_MASK) != 0u) {{",
                    f"    return {C_PREFIX}_CODEC_MALFORMED;",
                    "  }",
                ]
            )
        else:
            raise AssertionError(f"unsupported constraint after validation: {kind}")
    return lines


def generate_python(
    schema: dict[str, Any], schema_path: Path, schema_hash: str
) -> str:
    header = schema["clear_header"]
    reading = schema["reading_body"]
    ack = schema["ack_body"]
    domains = schema["domains"]["values"]
    flags = reading["flags"]["values"]
    reset_reasons = schema["reset_reasons"]["values"]
    ack_statuses = schema["ack_statuses"]["values"]

    header_format = python_struct_format(header["fields"])
    reading_format = python_struct_format(reading["fields"])
    ack_format = python_struct_format(ack["fields"])
    nonce_format = python_struct_format(schema["ccm"]["nonce"]["fields"])

    lines = [
        python_banner(schema_path, schema_hash),
        "from __future__ import annotations",
        "",
        "from dataclasses import dataclass",
        "from enum import IntEnum, IntFlag",
        "import struct",
        "",
        f'SCHEMA_SHA256 = "{schema_hash}"',
        f"PROTOCOL_VERSION = {schema['protocol_version']}",
        f"CONTROL = {schema['control']['pilot_value']}",
        f"KEY_SIZE = {schema['ccm']['key_size_bytes']}",
        f"NONCE_SIZE = {schema['ccm']['nonce_size_bytes']}",
        f"TAG_SIZE = {schema['ccm']['tag_size_bytes']}",
        f"CLEAR_HEADER_SIZE = {header['wire_size_bytes']}",
        f"READING_BODY_SIZE = {reading['wire_size_bytes']}",
        f"ACK_BODY_SIZE = {ack['wire_size_bytes']}",
        f"READING_FRAME_SIZE = {schema['frames']['reading']['sx1262_payload_size_bytes']}",
        f"ACK_FRAME_SIZE = {schema['frames']['ack']['sx1262_payload_size_bytes']}",
        "",
        f'CLEAR_HEADER_STRUCT = struct.Struct("{header_format}")',
        f'READING_BODY_STRUCT = struct.Struct("{reading_format}")',
        f'ACK_BODY_STRUCT = struct.Struct("{ack_format}")',
        f'NONCE_STRUCT = struct.Struct("{nonce_format}")',
        "",
        "assert CLEAR_HEADER_STRUCT.size == CLEAR_HEADER_SIZE",
        "assert READING_BODY_STRUCT.size == READING_BODY_SIZE",
        "assert ACK_BODY_STRUCT.size == ACK_BODY_SIZE",
        "assert NONCE_STRUCT.size == NONCE_SIZE",
        "",
        "",
        "class CodecError(ValueError):",
        "  \"\"\"The supplied value or byte sequence is not valid protocol v2 data.\"\"\"",
        "",
        "",
        "class Domain(IntEnum):",
    ]
    for domain in domains:
        lines.append(f"  {domain['name']} = {domain['value']}")

    lines.extend(["", "", "class AckStatus(IntEnum):"])
    for status in ack_statuses:
        lines.append(f"  {status['name']} = {status['value']}")

    lines.extend(["", "", "class ResetReason(IntEnum):"])
    for reason in reset_reasons:
        lines.append(f"  {reason['name']} = {reason['value']}")

    lines.extend(["", "", "class ReadingFlag(IntFlag):"])
    for flag in flags:
        lines.append(f"  {flag['name']} = 1 << {flag['bit']}")

    lines.extend(
        [
            "",
            "",
            f"READING_RESERVED_FLAGS_MASK = {reading['flags']['reserved_mask']}",
            "",
            "ACK_STATUS_BY_DOMAIN: dict[Domain, AckStatus] = {",
        ]
    )
    for pair in ack["domain_status_pairs"]:
        lines.append(
            f"    Domain.{pair['domain']}: AckStatus.{pair['status']},"
        )
    lines.extend(["}", "", "", "@dataclass(frozen=True)", "class ClearHeader:"])
    lines.extend(python_dataclass_fields(header["fields"]))
    lines.extend(["", "", "@dataclass(frozen=True)", "class Reading:"])
    lines.extend(python_dataclass_fields(reading["fields"]))
    lines.extend(["", "", "@dataclass(frozen=True)", "class Ack:"])
    lines.extend(python_dataclass_fields(ack["fields"]))

    lines.extend(
        [
            "",
            "",
            "def is_supported_control(control: int) -> bool:",
            "  return control == CONTROL",
            "",
            "",
            "def domain_is_reading(domain: int) -> bool:",
            "  return domain in {",
        ]
    )
    for domain in domains:
        if domain["body"] == "reading_body":
            lines.append(f"      Domain.{domain['name']},")
    lines.extend(["  }", "", "", "def domain_is_ack(domain: int) -> bool:", "  return domain in {"])
    for domain in domains:
        if domain["body"] == "ack_body":
            lines.append(f"      Domain.{domain['name']},")
    lines.extend(
        [
            "  }",
            "",
            "",
            "def ack_status_matches_domain(domain: int, status: int) -> bool:",
            "  try:",
            "    return ACK_STATUS_BY_DOMAIN[Domain(domain)] == AckStatus(status)",
            "  except (KeyError, ValueError):",
            "    return False",
            "",
            "",
            "def validate_reading(reading: Reading) -> None:",
        ]
    )
    lines.extend(python_validation_lines(reading))
    lines.extend(["", ""])

    lines.extend(
        python_encode_decode_functions(
            "clear_header",
            "ClearHeader",
            header["fields"],
            "CLEAR_HEADER_STRUCT",
            validator=None,
        )
    )
    lines.extend(
        [
            "def build_nonce(header: ClearHeader) -> bytes:",
            "  try:",
            "    return NONCE_STRUCT.pack(",
        ]
    )
    for field in schema["ccm"]["nonce"]["fields"]:
        lines.append(f"        header.{field['name']},")
    lines.extend(
        [
            "    )",
            "  except (struct.error, TypeError) as exc:",
            "    raise CodecError(f\"invalid nonce input: {exc}\") from exc",
            "",
            "",
        ]
    )
    lines.extend(
        python_encode_decode_functions(
            "reading",
            "Reading",
            reading["fields"],
            "READING_BODY_STRUCT",
            validator="validate_reading",
        )
    )
    lines.extend(
        [
            "def validate_ack(ack: Ack) -> None:",
            "  try:",
            "    AckStatus(ack.status)",
            "  except (TypeError, ValueError) as exc:",
            "    raise CodecError(f\"unknown ACK status: {ack.status}\") from exc",
            "",
            "",
        ]
    )
    lines.extend(
        python_encode_decode_functions(
            "ack",
            "Ack",
            ack["fields"],
            "ACK_BODY_STRUCT",
            validator="validate_ack",
        )
    )
    return "\n".join(lines)


def python_struct_format(fields: list[dict[str, Any]]) -> str:
    return "<" + "".join(type_info(field["type"])["struct"] for field in fields)


def python_dataclass_fields(fields: list[dict[str, Any]]) -> list[str]:
    return [
        f"  {field['name']}: {type_info(field['type'])['python']}"
        for field in fields
    ]


def python_validation_lines(reading_body: dict[str, Any]) -> list[str]:
    lines: list[str] = []
    for constraint in reading_body["structural_constraints"]:
        kind = constraint["kind"]
        if kind == "zero_unless_flag":
            lines.extend(
                [
                    f"  if not (reading.flags & ReadingFlag.{constraint['flag']}) and "
                    f"reading.{constraint['field']} != 0:",
                    f'    raise CodecError("{constraint["field"]} must be zero when '
                    f'{constraint["flag"]} is clear")',
                ]
            )
        elif kind == "all_zero_unless_flag":
            tests = " or ".join(
                f"reading.{field} != 0" for field in constraint["fields"]
            )
            lines.extend(
                [
                    f"  if not (reading.flags & ReadingFlag.{constraint['flag']}) "
                    f"and ({tests}):",
                    f'    raise CodecError("previous-cycle metrics must be zero when '
                    f'{constraint["flag"]} is clear")',
                ]
            )
        elif kind == "flag_iff_field_equals":
            lines.extend(
                [
                    f"  if bool(reading.flags & ReadingFlag.{constraint['flag']}) != "
                    f"(reading.{constraint['field']} == {constraint['value']}):",
                    f'    raise CodecError("{constraint["flag"]} does not match '
                    f'{constraint["field"]}")',
                ]
            )
        elif kind == "flag_requires_flag":
            lines.extend(
                [
                    f"  if (reading.flags & ReadingFlag.{constraint['flag']}) and not "
                    f"(reading.flags & ReadingFlag.{constraint['required_flag']}):",
                    f'    raise CodecError("{constraint["flag"]} requires '
                    f'{constraint["required_flag"]}")',
                ]
            )
        elif kind == "reserved_bits_zero":
            lines.extend(
                [
                    f"  if reading.{constraint['field']} & READING_RESERVED_FLAGS_MASK:",
                    '    raise CodecError("reserved reading flag bits must be zero")',
                ]
            )
        else:
            raise AssertionError(f"unsupported constraint after validation: {kind}")
    return lines


def python_encode_decode_functions(
    function_name: str,
    class_name: str,
    fields: list[dict[str, Any]],
    struct_name: str,
    validator: str | None,
) -> list[str]:
    lines = [f"def encode_{function_name}(value: {class_name}) -> bytes:"]
    if validator is not None:
        lines.append(f"  {validator}(value)")
    lines.extend(["  try:", f"    return {struct_name}.pack("])
    for field in fields:
        lines.append(f"        value.{field['name']},")
    lines.extend(
        [
            "    )",
            "  except (struct.error, TypeError) as exc:",
            f'    raise CodecError(f"invalid {function_name}: {{exc}}") from exc',
            "",
            "",
            f"def decode_{function_name}(data: bytes) -> {class_name}:",
            f"  if len(data) != {struct_name}.size:",
            f'    raise CodecError("{function_name} must be "',
            f'                     f"{{{struct_name}.size}} bytes, got {{len(data)}}")',
            f"  values = {struct_name}.unpack(data)",
            f"  result = {class_name}(",
        ]
    )
    for index, field in enumerate(fields):
        lines.append(f"      {field['name']}=values[{index}],")
    lines.append("  )")
    if validator is not None:
        lines.append(f"  {validator}(result)")
    lines.extend(["  return result", "", ""])
    return lines


def c_banner(schema_path: Path, schema_hash: str) -> str:
    return (
        f"/* Generated from {display_path(schema_path)} by "
        f"{display_path(GENERATOR_PATH)}.\n"
        f" * Schema SHA-256: {schema_hash}. Do not edit by hand. */"
    )


def python_banner(schema_path: Path, schema_hash: str) -> str:
    return (
        f"# Generated from {display_path(schema_path)} by "
        f"{display_path(GENERATOR_PATH)}.\n"
        f"# Schema SHA-256: {schema_hash}. Do not edit by hand."
    )


def c_hex(value: int, digits: int) -> str:
    return f"0x{value:0{digits}x}"


def require_object(value: dict[str, Any], key: str, context: str) -> dict[str, Any]:
    result = value.get(key)
    if not isinstance(result, dict):
        raise SchemaError(f"{context}.{key} must be an object")
    return result


def require_list(value: dict[str, Any], key: str, context: str) -> list[Any]:
    result = value.get(key)
    if not isinstance(result, list):
        raise SchemaError(f"{context}.{key} must be an array")
    return result


def require_string(value: dict[str, Any], key: str, context: str) -> str:
    result = value.get(key)
    if not isinstance(result, str):
        raise SchemaError(f"{context}.{key} must be a string")
    return result


def require_identifier(value: dict[str, Any], key: str, context: str) -> str:
    result = require_string(value, key, context)
    if IDENTIFIER_RE.fullmatch(result) is None:
        raise SchemaError(f"{context}.{key} is not an identifier: {result!r}")
    return result


def require_bool(value: dict[str, Any], key: str, context: str) -> bool:
    result = value.get(key)
    if not isinstance(result, bool):
        raise SchemaError(f"{context}.{key} must be a bool")
    return result


def require_int(
    value: dict[str, Any],
    key: str,
    context: str,
    minimum: int,
    maximum: int,
) -> int:
    result = value.get(key)
    if not isinstance(result, int) or isinstance(result, bool):
        raise SchemaError(f"{context}.{key} must be an integer")
    if result < minimum or result > maximum:
        raise SchemaError(
            f"{context}.{key} must be between {minimum} and {maximum}, got {result}"
        )
    return result


def require_equal(
    value: dict[str, Any], key: str, expected: object, context: str
) -> None:
    actual = value.get(key)
    if actual != expected or type(actual) is not type(expected):
        raise SchemaError(f"{context}.{key} must be {expected!r}, got {actual!r}")


def require_hex_matches(
    value: dict[str, Any], key: str, numeric_value: int, context: str
) -> None:
    text = require_string(value, key, context)
    try:
        parsed = int(text, 16)
    except ValueError as exc:
        raise SchemaError(f"{context}.{key} is not hexadecimal: {text!r}") from exc
    if parsed != numeric_value:
        raise SchemaError(
            f"{context}.{key} represents {parsed}, expected {numeric_value}"
        )


def display_path(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None


def write_text_atomic(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


if __name__ == "__main__":
    raise SystemExit(main())

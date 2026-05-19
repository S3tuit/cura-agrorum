#!/usr/bin/env python3
"""Generate protocol code shared by firmware and server.

This script is the source-of-truth generator for Cura Agrorum wire structs.
It reads JSON schemas from protocol/schemas/ and writes matching C headers for
the ESP32 firmware plus Python dataclass/struct decoders for the server.

Current generated protocol payloads:
  * reading_t: the sensor reading frame sent after sampling.
  * node_config_t: the node configuration/handshake frame sent before readings.
  * handshake_ack_t: the server response to the node configuration frame.

It also manages the local node identity used by firmware:
  * firmware/main/node_uuid.txt is an ignored, per-physical-node UUID file.
  * firmware/main/node_identity.h is generated from that UUID and included by C.

Normal use:
  python3 protocol/tools/generate.py

This regenerates tracked protocol outputs and, if node_uuid.txt does not exist,
creates one with a new UUID. Keep node_uuid.txt with the physical node and do
not commit it.

CI/check use:
  python3 protocol/tools/generate.py --check

This verifies generated tracked outputs are current. It does not create a new
node UUID when the UUID file is missing, so fresh checkouts can run it safely.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import re
import uuid
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PY_INIT = REPO_ROOT / "server" / "cura_server" / "generated" / "__init__.py"
DEFAULT_NODE_UUID = REPO_ROOT / "firmware" / "main" / "node_uuid.txt"
DEFAULT_NODE_IDENTITY = REPO_ROOT / "firmware" / "main" / "node_identity.h"


@dataclass(frozen=True)
class ProtocolOutput:
  schema: Path
  c_header: Path
  py_schema: Path


PROTOCOL_OUTPUTS = (
    ProtocolOutput(
        schema=REPO_ROOT / "protocol" / "schemas" / "reading_v1.json",
        c_header=REPO_ROOT / "firmware" / "main" / "reading.h",
        py_schema=REPO_ROOT / "server" / "cura_server" / "generated" / "reading_v1.py",
    ),
    ProtocolOutput(
        schema=REPO_ROOT / "protocol" / "schemas" / "node_config_v1.json",
        c_header=REPO_ROOT / "firmware" / "main" / "node_config.h",
        py_schema=REPO_ROOT
        / "server"
        / "cura_server"
        / "generated"
        / "node_config_v1.py",
    ),
    ProtocolOutput(
        schema=REPO_ROOT / "protocol" / "schemas" / "handshake_ack_v1.json",
        c_header=REPO_ROOT / "firmware" / "main" / "handshake_ack.h",
        py_schema=REPO_ROOT
        / "server"
        / "cura_server"
        / "generated"
        / "handshake_ack_v1.py",
    ),
)

TYPE_MAP = {
    "u8": {"c": "uint8_t", "struct": "B", "size": 1, "py": "int"},
    "u16": {"c": "uint16_t", "struct": "H", "size": 2, "py": "int"},
    "u32": {"c": "uint32_t", "struct": "I", "size": 4, "py": "int"},
    "i16": {"c": "int16_t", "struct": "h", "size": 2, "py": "int"},
}

IDENT_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
BYTES_RE = re.compile(r"^bytes\[(\d+)\]$")


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description="Generate Cura Agrorum protocol files")
  parser.add_argument("--check", action="store_true")
  parser.add_argument("--node-uuid-file", type=Path, default=DEFAULT_NODE_UUID)
  parser.add_argument(
      "--node-identity-header",
      type=Path,
      default=DEFAULT_NODE_IDENTITY,
  )
  args = parser.parse_args(argv)

  outputs: dict[Path, str] = {
      DEFAULT_PY_INIT: '"""Generated protocol schema modules."""\n',
  }

  for protocol_output in PROTOCOL_OUTPUTS:
    schema = load_schema(protocol_output.schema)
    outputs[protocol_output.c_header] = generate_c_header(
        schema, protocol_output.schema
    )
    outputs[protocol_output.py_schema] = generate_python_schema(
        schema, protocol_output.schema
    )

  node_uuid = load_node_uuid(args.node_uuid_file, create=not args.check)
  if node_uuid is not None:
    outputs[args.node_identity_header] = generate_node_identity_header(
        node_uuid, args.node_uuid_file
    )

  if args.check:
    mismatched = [
        path for path, content in outputs.items() if read_file(path) != content
    ]
    if mismatched:
      for path in mismatched:
        print(f"{path} is out of date")
      return 1
    return 0

  for path, content in outputs.items():
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
  return 0


def load_schema(path: Path) -> dict[str, Any]:
  schema = json.loads(path.read_text(encoding="utf-8"))
  validate_schema(schema)
  return schema


def validate_schema(schema: dict[str, Any]) -> None:
  required_top = ("schema_version", "record_type", "endianness", "record_name", "fields")
  for key in required_top:
    if key not in schema:
      raise ValueError(f"schema is missing {key}")

  if schema["endianness"] != "little":
    raise ValueError("only little-endian schemas are currently supported")
  if not IDENT_RE.match(schema["record_name"]):
    raise ValueError("record_name must be a valid C/Python identifier")

  seen_fields: set[str] = set()
  seen_flag_bits: set[int] = set()
  seen_flag_names: set[str] = set()
  for field in schema["fields"]:
    for key in ("name", "type", "description"):
      if key not in field:
        raise ValueError(f"field is missing {key}: {field}")
    if not IDENT_RE.match(field["name"]):
      raise ValueError(f"invalid field name: {field['name']}")
    if field["name"] in seen_fields:
      raise ValueError(f"duplicate field name: {field['name']}")
    seen_fields.add(field["name"])
    type_info(field["type"])

    if ("valid_flag" in field) != ("valid_flag_bit" in field):
      raise ValueError(f"field must set both valid_flag and valid_flag_bit: {field}")
    if "valid_flag" in field:
      if not IDENT_RE.match(field["valid_flag"]):
        raise ValueError(f"invalid flag name: {field['valid_flag']}")
      if field["valid_flag"] in seen_flag_names:
        raise ValueError(f"duplicate flag name: {field['valid_flag']}")
      seen_flag_names.add(field["valid_flag"])
      bit = field["valid_flag_bit"]
      if not isinstance(bit, int) or bit < 0 or bit > 31:
        raise ValueError(f"invalid flag bit: {field}")
      if bit in seen_flag_bits:
        raise ValueError(f"duplicate flag bit: {bit}")
      seen_flag_bits.add(bit)


def type_info(type_name: str) -> dict[str, Any]:
  if type_name in TYPE_MAP:
    return TYPE_MAP[type_name]
  match = BYTES_RE.match(type_name)
  if match is not None:
    size = int(match.group(1))
    if size <= 0:
      raise ValueError(f"invalid byte array size: {type_name}")
    return {"c": "uint8_t", "struct": f"{size}s", "size": size, "py": "bytes"}
  raise ValueError(f"unsupported field type: {type_name}")


def generate_c_header(schema: dict[str, Any], schema_path: Path) -> str:
  schema_macro, record_macro = c_protocol_macros(schema)
  lines = [
      generated_c_banner(schema_path),
      "#pragma once",
      "",
      "#include <stdint.h>",
      "",
      f"#define {schema_macro} {schema['schema_version']}",
      f"#define {record_macro} {schema['record_type']}",
  ]

  flag_fields = [field for field in schema["fields"] if "valid_flag" in field]
  if flag_fields:
    lines.extend(["", "// Bitmask of which sensor fields are valid in a reading_t instance."])
    for field in flag_fields:
      lines.append(f"#define {field['valid_flag']} (1u << {field['valid_flag_bit']})")

  lines.extend(["", f"typedef struct __attribute__((packed)) {{"])
  for field in schema["fields"]:
    info = type_info(field["type"])
    description = field["description"]
    if field["type"].startswith("bytes["):
      lines.append(f"  {info['c']} {field['name']}[{info['size']}]; // {description}")
    else:
      lines.append(f"  {info['c']} {field['name']}; // {description}")
  record_name = schema["record_name"]
  lines.extend(
      [
          f"}} {record_name};",
          "",
          f'_Static_assert(sizeof({record_name}) == {record_size(schema)}, "unexpected {record_name} size");',
          f'_Static_assert({schema_macro} <= UINT8_MAX, "TCP frame schema version is one byte");',
          f'_Static_assert({record_macro} <= UINT8_MAX, "TCP frame record type is one byte");',
          "#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)",
          f'_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "{record_name} wire schema requires little-endian target");',
          "#endif",
          "",
      ]
  )
  return "\n".join(lines)


def generate_python_schema(schema: dict[str, Any], schema_path: Path) -> str:
  schema_const, record_const, format_const, size_const = python_protocol_names(schema)
  class_name = python_class_name(schema["record_name"])
  from_tuple_name = python_from_tuple_name(schema["record_name"])
  struct_format = "<" + "".join(
      type_info(field["type"])["struct"] for field in schema["fields"]
  )
  lines = [
      generated_python_banner(schema_path),
      "from __future__ import annotations",
      "",
      "from dataclasses import dataclass",
      "import struct",
      "",
      f"{schema_const} = {schema['schema_version']}",
      f"{record_const} = {schema['record_type']}",
      f'{format_const} = "{struct_format}"',
      f"{size_const} = struct.calcsize({format_const})",
      "",
  ]

  for field in schema["fields"]:
    if "valid_flag" in field:
      lines.append(f"{field['valid_flag']} = 1 << {field['valid_flag_bit']}")

  lines.extend(["", "", "@dataclass(frozen=True)", f"class {class_name}:"])
  for field in schema["fields"]:
    info = type_info(field["type"])
    lines.append(f"  {field['name']}: {info['py']}")

  lines.extend(
      [
          "",
          "",
          f"def {from_tuple_name}(values: tuple[object, ...]) -> {class_name}:",
          f"  return {class_name}(",
      ]
  )
  for index, field in enumerate(schema["fields"]):
    lines.append(f"      {field['name']}=values[{index}],")
  lines.extend(["  )", ""])
  return "\n".join(lines)


def c_protocol_macros(schema: dict[str, Any]) -> tuple[str, str]:
  if schema["record_name"] == "reading_t":
    return "FILE_SCHEMA_VERSION", "CURA_RECORD_TYPE"
  prefix = macro_prefix(schema["record_name"])
  return f"{prefix}_SCHEMA_VERSION", f"{prefix}_RECORD_TYPE"


def python_protocol_names(schema: dict[str, Any]) -> tuple[str, str, str, str]:
  if schema["record_name"] == "reading_t":
    return "FILE_SCHEMA_VERSION", "CURA_RECORD_TYPE", "READING_FORMAT", "READING_SIZE"
  prefix = macro_prefix(schema["record_name"])
  return (
      f"{prefix}_SCHEMA_VERSION",
      f"{prefix}_RECORD_TYPE",
      f"{prefix}_FORMAT",
      f"{prefix}_SIZE",
  )


def macro_prefix(record_name: str) -> str:
  base = strip_record_suffix(record_name)
  return base.upper()


def python_class_name(record_name: str) -> str:
  return "".join(part.capitalize() for part in strip_record_suffix(record_name).split("_"))


def python_from_tuple_name(record_name: str) -> str:
  return f"{strip_record_suffix(record_name)}_from_tuple"


def strip_record_suffix(record_name: str) -> str:
  if record_name.endswith("_t"):
    return record_name[:-2]
  return record_name


def generated_c_banner(schema_path: Path) -> str:
  rel_schema = schema_path.resolve().relative_to(REPO_ROOT)
  return (
      f"/* Generated C header from {rel_schema} by "
      "protocol/tools/generate.py. Do not edit by hand. */"
  )


def generated_python_banner(schema_path: Path) -> str:
  rel_schema = schema_path.resolve().relative_to(REPO_ROOT)
  return (
      f"# Generated Python module from {rel_schema} by "
      "protocol/tools/generate.py. Do not edit by hand."
  )


def record_size(schema: dict[str, Any]) -> int:
  return sum(type_info(field["type"])["size"] for field in schema["fields"])


def load_node_uuid(path: Path, create: bool) -> uuid.UUID | None:
  if path.exists():
    return uuid.UUID(path.read_text(encoding="utf-8").strip())
  if not create:
    return None

  node_uuid = uuid.uuid4()
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(f"{node_uuid}\n", encoding="utf-8")
  return node_uuid


def generate_node_identity_header(node_uuid: uuid.UUID, uuid_path: Path) -> str:
  rel_uuid_path = uuid_path.resolve().relative_to(REPO_ROOT)
  byte_initializer = ", ".join(f"0x{byte:02x}" for byte in node_uuid.bytes)
  return "\n".join(
      [
          f"/* Generated firmware node identity from {rel_uuid_path} by "
          "protocol/tools/generate.py. Do not edit by hand. */",
          "#pragma once",
          "",
          f'#define CURA_NODE_UUID_STR "{node_uuid}"',
          f"#define CURA_NODE_UUID_BYTES {{{byte_initializer}}}",
          "",
      ]
  )


def read_file(path: Path) -> str | None:
  if not path.exists():
    return None
  return path.read_text(encoding="utf-8")


if __name__ == "__main__":
  raise SystemExit(main())

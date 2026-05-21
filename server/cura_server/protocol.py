from __future__ import annotations

from dataclasses import dataclass
import struct
from uuid import UUID

from .generated.node_config_v1 import (
    NODE_CONFIG_FORMAT,
    NODE_CONFIG_RECORD_TYPE,
    NODE_CONFIG_SCHEMA_VERSION,
    NODE_CONFIG_SIZE,
    NodeConfig,
    node_config_from_tuple,
)
from .generated.handshake_ack_v1 import (
    HANDSHAKE_ACK_FORMAT,
    HANDSHAKE_ACK_RECORD_TYPE,
    HANDSHAKE_ACK_SCHEMA_VERSION,
    HANDSHAKE_ACK_SIZE,
)
from .generated.reading_v1 import (
    CURA_RECORD_TYPE,
    FILE_SCHEMA_VERSION,
    READING_DS18B20_TEMP_OK,
    READING_ENV280_HUMIDITY_OK,
    READING_ENV280_PRESSURE_OK,
    READING_ENV280_TEMP_OK,
    READING_FORMAT,
    READING_SIZE,
    READING_SOIL_MV_OK,
    Reading,
    reading_from_tuple,
)

# This must be the same as the port advertised by Avahi (mDNS responder).
DEFAULT_PORT = 18032

# The header is in big endian (network byte order).
HEADER_FORMAT = "!HBB"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

HANDSHAKE_ACK_OK = 0
HANDSHAKE_ACK_ERROR = 1


class ProtocolError(ValueError):
  """Raised when a TCP frame is malformed for the selected schema."""


@dataclass(frozen=True)
class FrameHeader:
  payload_len: int
  record_type: int
  schema_version: int


def parse_header(data: bytes) -> FrameHeader:
  if len(data) != HEADER_SIZE:
    raise ProtocolError(f"header must be {HEADER_SIZE} bytes, got {len(data)}")

  payload_len, record_type, schema_version = struct.unpack(HEADER_FORMAT, data)
  return FrameHeader(
      payload_len=payload_len,
      record_type=record_type,
      schema_version=schema_version,
  )


def is_supported_reading_frame(header: FrameHeader) -> bool:
  return (
      header.record_type == CURA_RECORD_TYPE
      and header.schema_version == FILE_SCHEMA_VERSION
  )


def is_supported_node_config_frame(header: FrameHeader) -> bool:
  return (
      header.record_type == NODE_CONFIG_RECORD_TYPE
      and header.schema_version == NODE_CONFIG_SCHEMA_VERSION
  )


def decode_reading(payload: bytes) -> Reading:
  # The firmware sends one TCP frame per logical message. Keeping the server
  # decoder exact here avoids silently accepting old batch-style payloads.
  if len(payload) != READING_SIZE:
    raise ProtocolError(
        f"reading payload must be {READING_SIZE} bytes, got {len(payload)}"
    )

  fields = struct.unpack(READING_FORMAT, payload)
  return reading_from_tuple(fields)


def decode_node_config(payload: bytes) -> NodeConfig:
  if len(payload) != NODE_CONFIG_SIZE:
    raise ProtocolError(
        f"node_config payload must be {NODE_CONFIG_SIZE} bytes, got {len(payload)}"
    )

  fields = struct.unpack(NODE_CONFIG_FORMAT, payload)
  return node_config_from_tuple(fields)


def encode_handshake_ack(status: int) -> bytes:
  """Encode the only server response frame currently used by firmware.

  Reading frames intentionally have no ACK for now. The current transport-level
  confirmation is the graceful TCP close; later this should become a persisted
  reading ACK keyed by (node_uuid, sample_id).
  """
  if status < 0 or status > 0xFFFFFFFF:
    raise ProtocolError(f"handshake ack status out of range: {status}")

  payload = struct.pack(HANDSHAKE_ACK_FORMAT, status)
  header = struct.pack(
      HEADER_FORMAT,
      HANDSHAKE_ACK_SIZE,
      HANDSHAKE_ACK_RECORD_TYPE,
      HANDSHAKE_ACK_SCHEMA_VERSION,
  )
  return header + payload


def hex_preview(payload: bytes, max_bytes: int = 16) -> str:
  preview = payload[:max_bytes].hex(" ")
  if len(payload) > max_bytes:
    return f"{preview} ..."
  return preview


def format_reading(
    peer: str,
    header: FrameHeader,
    reading: Reading,
    index: int = 1,
    total: int = 1,
) -> str:
  prefix = (
      f"client={peer} record_type={header.record_type}"
      f" schema={header.schema_version}"
      f" reading={index}/{total}"
      f" node={format_node_uuid_bytes(reading.node_uuid)}"
  )
  return (
      f"{prefix} sample_id={reading.sample_id}"
      f" boot={reading.bootno}"
      f" wake=0x{reading.wake_causes:08x}"
      f" run={reading.run_ms}ms"
      f" soil_mv={_field(reading.flags, READING_SOIL_MV_OK, reading.soil_mv, 'mV')}"
      f" ds18b20={_field(reading.flags, READING_DS18B20_TEMP_OK, _centi(reading.ds18b20_centi_c, 'C'))}"
      f" env280_temp={_field(reading.flags, READING_ENV280_TEMP_OK, _centi(reading.env280_centi_c, 'C'))}"
      f" pressure={_field(reading.flags, READING_ENV280_PRESSURE_OK, reading.env280_pressure_pa, 'Pa')}"
      f" humidity={_field(reading.flags, READING_ENV280_HUMIDITY_OK, _centi(reading.env280_humidity_centi_pct, '%'))}"
      f" flags=0x{reading.flags:02x}"
  )


def format_node_config(peer: str, header: FrameHeader, config: NodeConfig) -> str:
  return (
      f"client={peer} record_type={header.record_type}"
      f" schema={header.schema_version}"
      f" node={format_node_uuid(config)}"
      f" soil_sensor={config.soil_sensor_id}"
      f" ds18b20_sensor={config.ds18b20_sensor_id}"
      f" env280_sensor={config.env280_sensor_id}"
      f" dry={config.soil_dry_mv}mV"
      f" wet={config.soil_wet_mv}mV"
  )


def format_node_uuid(config: NodeConfig) -> str:
  return format_node_uuid_bytes(config.node_uuid)


def node_uuid_from_bytes(node_uuid: bytes) -> UUID:
  try:
    return UUID(bytes=node_uuid)
  except ValueError as exc:
    raise ProtocolError(f"invalid node UUID bytes: {exc}") from exc


def format_node_uuid_bytes(node_uuid: bytes) -> str:
  return str(node_uuid_from_bytes(node_uuid))


def format_unsupported_frame(peer: str, header: FrameHeader, payload: bytes) -> str:
  return (
      f"client={peer} unsupported frame record_type={header.record_type}"
      f" schema={header.schema_version} payload_len={header.payload_len}"
      f" preview={hex_preview(payload)}"
  )


def _field(flags: int, flag: int, value: object, suffix: str = "") -> str:
  if (flags & flag) == 0:
    return "missing"
  return f"{value}{suffix}"


def _centi(value: int, unit: str) -> str:
  sign = "-" if value < 0 else ""
  absolute = abs(value)
  return f"{sign}{absolute // 100}.{absolute % 100:02d}{unit}"

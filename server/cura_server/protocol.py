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
from .generated.reading_v1 import (
    CURA_RECORD_TYPE,
    FILE_SCHEMA_VERSION,
    READING_DS18B20_TEMP_OK,
    READING_ENV280_CHIP_ID_OK,
    READING_ENV280_HUMIDITY_OK,
    READING_ENV280_PRESSURE_OK,
    READING_ENV280_TEMP_OK,
    READING_FORMAT,
    READING_SIZE,
    READING_SOIL_MV_OK,
    READING_SOIL_RAW_OK,
    Reading,
    reading_from_tuple,
)

# This must be the same as the port advertised by Avahi (mDNS responder).
DEFAULT_PORT = 18032

# The header is in big endian (network byte order).
HEADER_FORMAT = "!HBB"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


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


def decode_readings(payload: bytes) -> list[Reading]:
  if len(payload) == 0:
    raise ProtocolError("reading payload is empty")
  if len(payload) % READING_SIZE != 0:
    raise ProtocolError(
        f"reading payload must be a multiple of {READING_SIZE} bytes, "
        f"got {len(payload)}"
    )

  readings: list[Reading] = []
  for offset in range(0, len(payload), READING_SIZE):
    fields = struct.unpack_from(READING_FORMAT, payload, offset)
    readings.append(reading_from_tuple(fields))

  return readings


def decode_node_config(payload: bytes) -> NodeConfig:
  if len(payload) != NODE_CONFIG_SIZE:
    raise ProtocolError(
        f"node_config payload must be {NODE_CONFIG_SIZE} bytes, got {len(payload)}"
    )

  fields = struct.unpack(NODE_CONFIG_FORMAT, payload)
  return node_config_from_tuple(fields)


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
    node_uuid: str | None = None,
) -> str:
  prefix = (
      f"client={peer} record_type={header.record_type}"
      f" schema={header.schema_version}"
      f" reading={index}/{total}"
  )
  if node_uuid is not None:
    prefix += f" node={node_uuid}"
  return (
      f"{prefix} boot={reading.bootno}"
      f" wake=0x{reading.wake_causes:08x}"
      f" run={reading.run_ms}ms"
      f" soil_raw={_field(reading.flags, READING_SOIL_RAW_OK, reading.soil_raw)}"
      f" soil_mv={_field(reading.flags, READING_SOIL_MV_OK, reading.soil_mv, 'mV')}"
      f" ds18b20={_field(reading.flags, READING_DS18B20_TEMP_OK, _centi(reading.ds18b20_centi_c, 'C'))}"
      f" env280_temp={_field(reading.flags, READING_ENV280_TEMP_OK, _centi(reading.env280_centi_c, 'C'))}"
      f" pressure={_field(reading.flags, READING_ENV280_PRESSURE_OK, reading.env280_pressure_pa, 'Pa')}"
      f" humidity={_field(reading.flags, READING_ENV280_HUMIDITY_OK, _centi(reading.env280_humidity_centi_pct, '%'))}"
      f" env280_chip={_field(reading.flags, READING_ENV280_CHIP_ID_OK, f'0x{reading.env280_chip_id:02x}')}"
      f" flags=0x{reading.flags:02x}"
  )


def format_node_config(peer: str, header: FrameHeader, config: NodeConfig) -> str:
  return (
      f"client={peer} record_type={header.record_type}"
      f" schema={header.schema_version}"
      f" node={format_node_uuid(config)}"
      f" reading_schema={config.reading_schema_version}"
      f" soil_sensor={config.soil_sensor_id}"
      f" soil_adc_gpio={config.soil_adc_gpio}"
      f" soil_adc_atten={config.soil_adc_atten_db_x10 / 10:g}dB"
      f" dry={config.soil_dry_mv}mV"
      f" wet={config.soil_wet_mv}mV"
      f" ds18b20_sensor={config.ds18b20_sensor_id}"
      f" ds18b20_gpio={config.ds18b20_gpio}"
      f" ds18b20_pullup={config.ds18b20_enable_internal_pullup}"
      f" ds18b20_resolution={config.ds18b20_resolution_bits}bit"
      f" env280_sensor={config.env280_sensor_id}"
      f" env280_i2c_port={config.env280_i2c_port}"
      f" env280_i2c_sda={config.env280_i2c_sda_gpio}"
      f" env280_i2c_scl={config.env280_i2c_scl_gpio}"
      f" env280_i2c_freq={config.env280_i2c_freq_hz}Hz"
      f" env280_i2c_addr=0x{config.env280_i2c_addr:02x}"
      f" env280_pullups={config.env280_enable_internal_pullups}"
  )


def format_node_uuid(config: NodeConfig) -> str:
  return str(UUID(bytes=config.node_uuid))


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

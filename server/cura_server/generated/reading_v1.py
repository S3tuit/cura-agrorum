# Generated Python module from protocol/schemas/reading_v1.json by protocol/tools/generate.py. Do not edit by hand.
from __future__ import annotations

from dataclasses import dataclass
import struct

FILE_SCHEMA_VERSION = 1
CURA_RECORD_TYPE = 1
READING_FORMAT = "<16sIIHHhhIHB2s"
READING_SIZE = struct.calcsize(READING_FORMAT)

READING_SOIL_MV_OK = 1 << 0
READING_DS18B20_TEMP_OK = 1 << 1
READING_ENV280_TEMP_OK = 1 << 2
READING_ENV280_PRESSURE_OK = 1 << 3
READING_ENV280_HUMIDITY_OK = 1 << 4


@dataclass(frozen=True)
class Reading:
  node_uuid: bytes
  bootno: int
  wake_causes: int
  run_ms: int
  soil_mv: int
  ds18b20_centi_c: int
  env280_centi_c: int
  env280_pressure_pa: int
  env280_humidity_centi_pct: int
  flags: int
  reserved: bytes


def reading_from_tuple(values: tuple[object, ...]) -> Reading:
  return Reading(
      node_uuid=values[0],
      bootno=values[1],
      wake_causes=values[2],
      run_ms=values[3],
      soil_mv=values[4],
      ds18b20_centi_c=values[5],
      env280_centi_c=values[6],
      env280_pressure_pa=values[7],
      env280_humidity_centi_pct=values[8],
      flags=values[9],
      reserved=values[10],
  )

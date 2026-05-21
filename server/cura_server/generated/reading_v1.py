# Generated Python module from protocol/schemas/reading_v1.json by protocol/tools/generate.py. Do not edit by hand.
from __future__ import annotations

from dataclasses import dataclass
import struct

FILE_SCHEMA_VERSION = 1
CURA_RECORD_TYPE = 1
READING_FORMAT = "<16sIIIHHhhIHB1s"
READING_SIZE = struct.calcsize(READING_FORMAT)

READING_SOIL_MV_OK = 1 << 0
READING_DS18B20_TEMP_OK = 1 << 1
READING_ENV280_TEMP_OK = 1 << 2
READING_ENV280_PRESSURE_OK = 1 << 3
READING_ENV280_HUMIDITY_OK = 1 << 4


@dataclass(frozen=True)
class Reading:
  node_uuid: bytes
  sample_id: int
  bootno: int
  wake_causes: int
  run_ms: int
  soil_mv: int
  ds18b20_centi_c: int
  env280_centi_c: int
  env280_pressure_pa: int
  env280_humidity_centi_pct: int
  flags: int
  padding: bytes


def reading_from_tuple(values: tuple[object, ...]) -> Reading:
  return Reading(
      node_uuid=values[0],
      sample_id=values[1],
      bootno=values[2],
      wake_causes=values[3],
      run_ms=values[4],
      soil_mv=values[5],
      ds18b20_centi_c=values[6],
      env280_centi_c=values[7],
      env280_pressure_pa=values[8],
      env280_humidity_centi_pct=values[9],
      flags=values[10],
      padding=values[11],
  )

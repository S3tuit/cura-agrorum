# Generated Python module from protocol/schemas/node_config_v1.json by protocol/tools/generate.py. Do not edit by hand.
from __future__ import annotations

from dataclasses import dataclass
import struct

NODE_CONFIG_SCHEMA_VERSION = 1
NODE_CONFIG_RECORD_TYPE = 2
NODE_CONFIG_FORMAT = "<16sHHHHH"
NODE_CONFIG_SIZE = struct.calcsize(NODE_CONFIG_FORMAT)



@dataclass(frozen=True)
class NodeConfig:
  node_uuid: bytes
  soil_sensor_id: int
  ds18b20_sensor_id: int
  env280_sensor_id: int
  soil_dry_mv: int
  soil_wet_mv: int


def node_config_from_tuple(values: tuple[object, ...]) -> NodeConfig:
  return NodeConfig(
      node_uuid=values[0],
      soil_sensor_id=values[1],
      ds18b20_sensor_id=values[2],
      env280_sensor_id=values[3],
      soil_dry_mv=values[4],
      soil_wet_mv=values[5],
  )

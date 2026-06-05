# Generated Python module from protocol/schemas/fault_v1.json by protocol/tools/generate.py. Do not edit by hand.
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct

FAULT_SCHEMA_VERSION = 1
FAULT_RECORD_TYPE = 3
FAULT_FORMAT = "<16s8sIIiii"
FAULT_SIZE = struct.calcsize(FAULT_FORMAT)


class FaultOperation(IntEnum):
  CURA_FAULT_NVS_INIT = 1
  CURA_FAULT_LITTLEFS_MOUNT = 2
  CURA_FAULT_QUEUE_METADATA_OPEN = 3
  CURA_FAULT_QUEUE_SEGMENT_READ = 4
  CURA_FAULT_QUEUE_SEGMENT_WRITE = 5
  CURA_FAULT_QUEUE_SEGMENT_DELETE = 6


@dataclass(frozen=True)
class Fault:
  node_uuid: bytes
  fault_id: bytes
  sample_id: int
  bootno: int
  operation: int
  esp_err: int
  posix_errno: int


def fault_from_tuple(values: tuple[object, ...]) -> Fault:
  return Fault(
      node_uuid=values[0],
      fault_id=values[1],
      sample_id=values[2],
      bootno=values[3],
      operation=values[4],
      esp_err=values[5],
      posix_errno=values[6],
  )

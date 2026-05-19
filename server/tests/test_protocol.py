import struct
import unittest

from cura_server.generated.node_config_v1 import (
    NODE_CONFIG_FORMAT,
    NODE_CONFIG_RECORD_TYPE,
    NODE_CONFIG_SCHEMA_VERSION,
    NODE_CONFIG_SIZE,
)
from cura_server.protocol import (
    CURA_RECORD_TYPE,
    FILE_SCHEMA_VERSION,
    HEADER_SIZE,
    ProtocolError,
    READING_DS18B20_TEMP_OK,
    READING_ENV280_CHIP_ID_OK,
    READING_ENV280_HUMIDITY_OK,
    READING_ENV280_PRESSURE_OK,
    READING_ENV280_TEMP_OK,
    READING_FORMAT,
    READING_SIZE,
    READING_SOIL_MV_OK,
    READING_SOIL_RAW_OK,
    decode_node_config,
    decode_readings,
    format_node_config,
    format_reading,
    format_node_uuid,
    is_supported_node_config_frame,
    is_supported_reading_frame,
    parse_header,
)


NODE_UUID_BYTES = bytes.fromhex("00112233445566778899aabbccddeeff")


def make_payload(flags=None):
  if flags is None:
    flags = (
        READING_SOIL_RAW_OK
        | READING_SOIL_MV_OK
        | READING_DS18B20_TEMP_OK
        | READING_ENV280_TEMP_OK
        | READING_ENV280_PRESSURE_OK
        | READING_ENV280_HUMIDITY_OK
        | READING_ENV280_CHIP_ID_OK
    )

  return struct.pack(
      READING_FORMAT,
      7,
      0x00000004,
      123,
      2400,
      780,
      2134,
      2250,
      101234,
      4852,
      0x60,
      flags,
      b"\x00\x00",
  )


def make_node_config_payload():
  return struct.pack(
      NODE_CONFIG_FORMAT,
      NODE_UUID_BYTES,
      FILE_SCHEMA_VERSION,
      1,
      2,
      3,
      34,
      120,
      2180,
      660,
      4,
      0,
      12,
      0,
      21,
      22,
      100000,
      0x76,
      1,
      b"\x00\x00\x00",
  )


class ProtocolTest(unittest.TestCase):
  def test_parse_header(self):
    header = parse_header(bytes([0x00, READING_SIZE, CURA_RECORD_TYPE, FILE_SCHEMA_VERSION]))

    self.assertEqual(header.payload_len, READING_SIZE)
    self.assertEqual(header.record_type, CURA_RECORD_TYPE)
    self.assertEqual(header.schema_version, FILE_SCHEMA_VERSION)

  def test_parse_header_rejects_wrong_size(self):
    with self.assertRaises(ProtocolError):
      parse_header(b"\x00")

  def test_decode_one_reading(self):
    readings = decode_readings(make_payload())

    self.assertEqual(len(readings), 1)
    self.assertEqual(readings[0].bootno, 7)
    self.assertEqual(readings[0].soil_mv, 780)
    self.assertEqual(readings[0].ds18b20_centi_c, 2134)
    self.assertEqual(readings[0].env280_chip_id, 0x60)

  def test_decode_node_config(self):
    config = decode_node_config(make_node_config_payload())

    self.assertEqual(config.node_uuid, NODE_UUID_BYTES)
    self.assertEqual(config.reading_schema_version, FILE_SCHEMA_VERSION)
    self.assertEqual(config.soil_adc_gpio, 34)
    self.assertEqual(config.soil_adc_atten_db_x10, 120)
    self.assertEqual(config.ds18b20_resolution_bits, 12)
    self.assertEqual(config.env280_i2c_addr, 0x76)

  def test_decode_node_config_rejects_wrong_size(self):
    with self.assertRaises(ProtocolError):
      decode_node_config(make_node_config_payload()[:-1])

  def test_supported_frame_helpers(self):
    reading_header = parse_header(
        bytes([0x00, READING_SIZE, CURA_RECORD_TYPE, FILE_SCHEMA_VERSION])
    )
    config_header = parse_header(
        bytes([0x00, NODE_CONFIG_SIZE, NODE_CONFIG_RECORD_TYPE, NODE_CONFIG_SCHEMA_VERSION])
    )

    self.assertTrue(is_supported_reading_frame(reading_header))
    self.assertFalse(is_supported_node_config_frame(reading_header))
    self.assertTrue(is_supported_node_config_frame(config_header))
    self.assertFalse(is_supported_reading_frame(config_header))

  def test_format_node_config(self):
    header = parse_header(
        bytes([0x00, NODE_CONFIG_SIZE, NODE_CONFIG_RECORD_TYPE, NODE_CONFIG_SCHEMA_VERSION])
    )
    config = decode_node_config(make_node_config_payload())

    line = format_node_config("127.0.0.1:10000", header, config)

    self.assertIn("node=00112233-4455-6677-8899-aabbccddeeff", line)
    self.assertIn("soil_adc_gpio=34", line)
    self.assertEqual(format_node_uuid(config), "00112233-4455-6677-8899-aabbccddeeff")

  def test_decode_multiple_readings(self):
    readings = decode_readings(make_payload() + make_payload())

    self.assertEqual(len(readings), 2)
    self.assertEqual(readings[1].env280_pressure_pa, 101234)

  def test_decode_rejects_non_multiple_payload_size(self):
    with self.assertRaises(ProtocolError):
      decode_readings(make_payload()[:-1])

  def test_format_reading_uses_flags_for_missing_values(self):
    header = parse_header(bytes([0x00, READING_SIZE, CURA_RECORD_TYPE, FILE_SCHEMA_VERSION]))
    reading = decode_readings(make_payload(flags=READING_SOIL_RAW_OK))[0]

    line = format_reading("127.0.0.1:10000", header, reading)

    self.assertIn("soil_raw=2400", line)
    self.assertIn("soil_mv=missing", line)
    self.assertIn("ds18b20=missing", line)

  def test_format_reading_can_include_node_uuid(self):
    header = parse_header(bytes([0x00, READING_SIZE, CURA_RECORD_TYPE, FILE_SCHEMA_VERSION]))
    reading = decode_readings(make_payload())[0]

    line = format_reading(
        "127.0.0.1:10000",
        header,
        reading,
        node_uuid="00112233-4455-6677-8899-aabbccddeeff",
    )

    self.assertIn("node=00112233-4455-6677-8899-aabbccddeeff", line)


if __name__ == "__main__":
  unittest.main()

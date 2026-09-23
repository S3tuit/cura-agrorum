#include "node_persistence_test.h"

#include <limits.h>
#include <string.h>

void node_persistence_test_reset_all(void) {
  node_persistence_test_reset();
  fake_backend_reset();
}

void node_persistence_test_restart(void) {
  node_persistence_test_reset();
  fake_backend_simulate_restart();
}

cura_lora_v2_reading_t node_persistence_test_make_reading(uint16_t marker) {
  cura_lora_v2_reading_t reading = {0};
  reading.sample_id = marker;
  reading.run_ms = marker;
  reading.soil_0_mv = (uint16_t)(1000U + marker);
  reading.soil_temp_0_centi_c = (int16_t)(2000 + (int32_t)marker);
  reading.flags =
      CURA_LORA_V2_FLAG_SOIL_0_VALID | CURA_LORA_V2_FLAG_SOIL_TEMP_0_VALID;
  return reading;
}

cura_lora_v2_reading_t node_persistence_test_make_boundary_reading(void) {
  cura_lora_v2_reading_t reading = {
      .sample_id = UINT32_MAX,
      .run_ms = UINT16_MAX,
      .soil_0_mv = UINT16_MAX,
      .soil_1_mv = 0U,
      .soil_temp_0_centi_c = INT16_MIN,
      .soil_temp_1_centi_c = INT16_MAX,
      .enclosure_centi_c = INT16_MIN,
      .enclosure_pressure_pa = UINT32_MAX,
      .enclosure_humidity_centi_pct = UINT16_MAX,
      .reset_reason = CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP,
      .previous_current_tx_attempts = UINT8_MAX,
      .previous_awake_ms = UINT16_MAX,
      .previous_current_delivery_ms = UINT16_MAX,
      .previous_cycle_tx_attempts = UINT8_MAX,
      .previous_cycle_accepted_readings = UINT8_MAX,
      .flags = CURA_LORA_V2_FLAG_DEEP_SLEEP_BOOT |
               CURA_LORA_V2_FLAG_SOIL_0_VALID | CURA_LORA_V2_FLAG_SOIL_1_VALID |
               CURA_LORA_V2_FLAG_SOIL_TEMP_0_VALID |
               CURA_LORA_V2_FLAG_SOIL_TEMP_1_VALID |
               CURA_LORA_V2_FLAG_ENCLOSURE_TEMP_VALID |
               CURA_LORA_V2_FLAG_ENCLOSURE_PRESSURE_VALID |
               CURA_LORA_V2_FLAG_ENCLOSURE_HUMIDITY_VALID |
               CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID |
               CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED,
  };
  return reading;
}

bool node_persistence_test_readings_equal(const cura_lora_v2_reading_t *left,
                                          const cura_lora_v2_reading_t *right) {
  uint8_t left_body[CURA_LORA_V2_READING_BODY_SIZE];
  uint8_t right_body[CURA_LORA_V2_READING_BODY_SIZE];
  return cura_lora_v2_encode_reading(left_body, sizeof(left_body), left) ==
             CURA_LORA_V2_CODEC_OK &&
         cura_lora_v2_encode_reading(right_body, sizeof(right_body), right) ==
             CURA_LORA_V2_CODEC_OK &&
         memcmp(left_body, right_body, sizeof(left_body)) == 0;
}

bool node_persistence_test_assert_diag(
    const diagn_context_t *diag, curag_operation_t operation,
    node_persistence_resource_t resource, node_persistence_stage_t stage,
    node_persistence_backend_status_kind_t kind, int32_t status) {
  if (diag == NULL || diag->operation != operation ||
      diag->context_length != CURAG_PERSISTENCE_CONTEXT_V1_SIZE ||
      diag->context_schema != CURAG_PERSISTENCE_CONTEXT_V1 ||
      diag->context[0] != resource || diag->context[1] != stage ||
      diag->context[2] != kind ||
      node_persistence_load_le32(diag->context + 3U) != (uint32_t)status) {
    return false;
  }
  for (size_t index = CURAG_PERSISTENCE_CONTEXT_V1_SIZE;
       index < CURAG_DIAGNOSTIC_CONTEXT_MAX; ++index) {
    if (diag->context[index] != 0U) {
      return false;
    }
  }
  return true;
}

bool node_persistence_test_snapshot(
    const char *path, node_persistence_test_snapshot_t *out_snapshot) {
  if (out_snapshot == NULL) {
    return false;
  }
  memset(out_snapshot, 0, sizeof(*out_snapshot));
  return fake_backend_read_file(path, out_snapshot->bytes,
                                sizeof(out_snapshot->bytes),
                                &out_snapshot->length);
}

bool node_persistence_test_snapshots_equal(
    const node_persistence_test_snapshot_t *left,
    const node_persistence_test_snapshot_t *right) {
  return left != NULL && right != NULL && left->length == right->length &&
         memcmp(left->bytes, right->bytes, left->length) == 0;
}

bool node_persistence_test_encode_reading_record(
    uint8_t record_type, uint32_t sample_id,
    const cura_lora_v2_reading_t *reading,
    uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE], size_t *out_length) {
  uint8_t payload[NODE_PERSISTENCE_READING_PAYLOAD_SIZE];
  cura_lora_v2_reading_t stored_reading;
  if (reading == NULL) {
    return false;
  }
  stored_reading = *reading;
  stored_reading.sample_id = sample_id;
  return reading != NULL && output != NULL && out_length != NULL &&
         cura_lora_v2_encode_reading(payload, sizeof(payload),
                                     &stored_reading) ==
             CURA_LORA_V2_CODEC_OK &&
         node_persistence_record_encode(node_persistence_backend(), record_type,
                                        payload, sizeof(payload), output,
                                        out_length);
}

void node_persistence_test_recalculate_crc(uint8_t *record,
                                           size_t record_length) {
  const uint32_t crc = node_persistence_backend()->crc32_iso_hdlc(
      record, record_length - sizeof(uint32_t));
  node_persistence_store_le32(record + record_length - sizeof(uint32_t), crc);
}

bool node_persistence_test_pending_ids(uint32_t *output, size_t capacity,
                                       size_t *out_count) {
  if (out_count == NULL || (output == NULL && capacity != 0U)) {
    return false;
  }
  node_persistence_test_snapshot_t snapshot;
  if (!fake_backend_file_exists(TEST_PENDING_PATH)) {
    *out_count = 0U;
    return true;
  }
  if (!node_persistence_test_snapshot(TEST_PENDING_PATH, &snapshot)) {
    return false;
  }

  size_t offset = 0U;
  size_t count = 0U;
  while (offset < snapshot.length) {
    if (snapshot.length - offset < NODE_PERSISTENCE_RECORD_HEADER_SIZE) {
      return false;
    }
    const size_t payload_length =
        node_persistence_load_le16(snapshot.bytes + offset + 6U);
    const size_t record_length =
        payload_length + NODE_PERSISTENCE_RECORD_OVERHEAD;
    if (record_length > snapshot.length - offset || count >= capacity ||
        node_persistence_record_validate(
            node_persistence_backend(), NODE_PERSISTENCE_LOG_PENDING,
            snapshot.bytes + offset,
            record_length) != NODE_PERSISTENCE_RECORD_VALID) {
      return false;
    }
    if (snapshot.bytes[offset + 5U] ==
        NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING) {
      uint8_t body[CURA_LORA_V2_READING_BODY_SIZE];
      cura_lora_v2_reading_t reading;
      if (!node_persistence_record_decode_reading(snapshot.bytes + offset,
                                                  record_length, body) ||
          cura_lora_v2_decode_reading(&reading, body, sizeof(body)) !=
              CURA_LORA_V2_CODEC_OK) {
        return false;
      }
      output[count++] = reading.sample_id;
    }
    offset += record_length;
  }
  *out_count = count;
  return true;
}

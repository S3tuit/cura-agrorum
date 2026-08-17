#include "persistence_test_support.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_littlefs.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "protocol_v2_lora_schema_generated.h"
#include "unity.h"

#define HWTEST_NVS_NAMESPACE "cura_lora_v2"
#define HWTEST_NVS_SAMPLE_ID_KEY "next_sample_id"
#define HWTEST_NVS_MESSAGE_ID_KEY "next_message_id"

static void close_component(void) {
  node_persistence_test_reset();
  if (esp_littlefs_mounted(HWTEST_LITTLEFS_PARTITION)) {
    TEST_ESP_OK(esp_vfs_littlefs_unregister(HWTEST_LITTLEFS_PARTITION));
  }
  const esp_err_t result = nvs_flash_deinit_partition(HWTEST_NVS_PARTITION);
  TEST_ASSERT_TRUE(result == ESP_OK || result == ESP_ERR_NVS_NOT_INITIALIZED);
}

void hwtest_erase_state(void) {
  close_component();
  TEST_ESP_OK(nvs_flash_erase_partition(HWTEST_NVS_PARTITION));
  TEST_ESP_OK(esp_littlefs_format(HWTEST_LITTLEFS_PARTITION));
}

void hwtest_finish_case(void) { hwtest_erase_state(); }

void hwtest_simulate_component_restart(void) { close_component(); }

void hwtest_mount_inspector(void) {
  if (esp_littlefs_mounted(HWTEST_LITTLEFS_PARTITION)) {
    return;
  }
  const esp_vfs_littlefs_conf_t configuration = {
      .base_path = NODE_PERSISTENCE_MOUNT_PATH,
      .partition_label = HWTEST_LITTLEFS_PARTITION,
      .partition = NULL,
      .format_if_mount_failed = false,
      .read_only = false,
      .dont_mount = false,
      .grow_on_mount = false,
  };
  TEST_ESP_OK(esp_vfs_littlefs_register(&configuration));
}

cura_lora_v2_reading_t hwtest_make_reading(uint16_t marker) {
  const cura_lora_v2_reading_t reading = {
      .sample_id = marker,
      .run_ms = (uint16_t)(100U + marker),
      .soil_0_mv = (uint16_t)(1000U + marker),
      .soil_1_mv = (uint16_t)(1100U + marker),
      .soil_temp_0_centi_c = (int16_t)(2000 + marker),
      .soil_temp_1_centi_c = (int16_t)(1900 + marker),
      .enclosure_centi_c = (int16_t)(2200 + marker),
      .enclosure_pressure_pa = UINT32_C(100000) + marker,
      .enclosure_humidity_centi_pct = (uint16_t)(5000U + marker),
      .reset_reason = CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP,
      .previous_current_tx_attempts = 2U,
      .previous_awake_ms = (uint16_t)(300U + marker),
      .previous_current_delivery_ms = (uint16_t)(80U + marker),
      .previous_cycle_tx_attempts = 3U,
      .previous_cycle_accepted_readings = 1U,
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

void hwtest_assert_reading_equal(const cura_lora_v2_reading_t *expected,
                                 const cura_lora_v2_reading_t *actual) {
  uint8_t expected_body[CURA_LORA_V2_READING_BODY_SIZE];
  uint8_t actual_body[CURA_LORA_V2_READING_BODY_SIZE];
  TEST_ASSERT_EQUAL(CURA_LORA_V2_CODEC_OK,
                    cura_lora_v2_encode_reading(
                        expected_body, sizeof(expected_body), expected));
  TEST_ASSERT_EQUAL(
      CURA_LORA_V2_CODEC_OK,
      cura_lora_v2_encode_reading(actual_body, sizeof(actual_body), actual));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_body, actual_body,
                               CURA_LORA_V2_READING_BODY_SIZE);
}

void hwtest_seed_next_sample_id(uint32_t value) {
  TEST_ESP_OK(nvs_flash_init_partition(HWTEST_NVS_PARTITION));
  nvs_handle_t handle = 0;
  TEST_ESP_OK(nvs_open_from_partition(
      HWTEST_NVS_PARTITION, HWTEST_NVS_NAMESPACE, NVS_READWRITE, &handle));
  TEST_ESP_OK(nvs_set_u32(handle, HWTEST_NVS_SAMPLE_ID_KEY, value));
  TEST_ESP_OK(nvs_commit(handle));
  nvs_close(handle);
  TEST_ESP_OK(nvs_flash_deinit_partition(HWTEST_NVS_PARTITION));
}

uint32_t hwtest_read_next_sample_id(void) {
  TEST_ESP_OK(nvs_flash_init_partition(HWTEST_NVS_PARTITION));
  nvs_handle_t handle = 0;
  uint32_t value = 0U;
  TEST_ESP_OK(nvs_open_from_partition(
      HWTEST_NVS_PARTITION, HWTEST_NVS_NAMESPACE, NVS_READONLY, &handle));
  TEST_ESP_OK(nvs_get_u32(handle, HWTEST_NVS_SAMPLE_ID_KEY, &value));
  nvs_close(handle);
  TEST_ESP_OK(nvs_flash_deinit_partition(HWTEST_NVS_PARTITION));
  return value;
}

void hwtest_seed_next_message_id(uint32_t value) {
  TEST_ESP_OK(nvs_flash_init_partition(HWTEST_NVS_PARTITION));
  nvs_handle_t handle = 0;
  TEST_ESP_OK(nvs_open_from_partition(
      HWTEST_NVS_PARTITION, HWTEST_NVS_NAMESPACE, NVS_READWRITE, &handle));
  TEST_ESP_OK(nvs_set_u32(handle, HWTEST_NVS_MESSAGE_ID_KEY, value));
  TEST_ESP_OK(nvs_commit(handle));
  nvs_close(handle);
  TEST_ESP_OK(nvs_flash_deinit_partition(HWTEST_NVS_PARTITION));
}

uint32_t hwtest_read_next_message_id(void) {
  TEST_ESP_OK(nvs_flash_init_partition(HWTEST_NVS_PARTITION));
  nvs_handle_t handle = 0;
  uint32_t value = 0U;
  TEST_ESP_OK(nvs_open_from_partition(
      HWTEST_NVS_PARTITION, HWTEST_NVS_NAMESPACE, NVS_READONLY, &handle));
  TEST_ESP_OK(nvs_get_u32(handle, HWTEST_NVS_MESSAGE_ID_KEY, &value));
  nvs_close(handle);
  TEST_ESP_OK(nvs_flash_deinit_partition(HWTEST_NVS_PARTITION));
  return value;
}

void hwtest_snapshot(const char *path, hwtest_snapshot_t *out_snapshot) {
  TEST_ASSERT_NOT_NULL(path);
  TEST_ASSERT_NOT_NULL(out_snapshot);
  hwtest_mount_inspector();
  const int descriptor = open(path, O_RDONLY);
  TEST_ASSERT_GREATER_OR_EQUAL(0, descriptor);
  struct stat status;
  TEST_ASSERT_EQUAL_INT(0, fstat(descriptor, &status));
  TEST_ASSERT_GREATER_OR_EQUAL(0, status.st_size);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(sizeof(out_snapshot->bytes),
                                   (uint32_t)status.st_size);
  out_snapshot->length = (size_t)status.st_size;
  size_t completed = 0U;
  while (completed < out_snapshot->length) {
    const ssize_t count = read(descriptor, out_snapshot->bytes + completed,
                               out_snapshot->length - completed);
    TEST_ASSERT_GREATER_THAN(0, count);
    completed += (size_t)count;
  }
  TEST_ASSERT_EQUAL_INT(0, close(descriptor));
}

void hwtest_assert_snapshot_equal(const hwtest_snapshot_t *expected,
                                  const hwtest_snapshot_t *actual) {
  TEST_ASSERT_EQUAL_UINT32(expected->length, actual->length);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected->bytes, actual->bytes,
                               expected->length);
}

bool hwtest_path_exists(const char *path) {
  hwtest_mount_inspector();
  struct stat status;
  return stat(path, &status) == 0;
}

static void write_all(int descriptor, const uint8_t *bytes, size_t length) {
  size_t completed = 0U;
  while (completed < length) {
    const ssize_t count =
        write(descriptor, bytes + completed, length - completed);
    TEST_ASSERT_GREATER_THAN(0, count);
    completed += (size_t)count;
  }
}

void hwtest_append_raw(const char *path, const uint8_t *bytes, size_t length) {
  hwtest_mount_inspector();
  const int descriptor = open(path, O_WRONLY | O_CREAT | O_APPEND, 0600);
  TEST_ASSERT_GREATER_OR_EQUAL(0, descriptor);
  write_all(descriptor, bytes, length);
  TEST_ASSERT_EQUAL_INT(0, fsync(descriptor));
  TEST_ASSERT_EQUAL_INT(0, close(descriptor));
}

void hwtest_replace_raw(const char *path, const uint8_t *bytes, size_t length) {
  hwtest_mount_inspector();
  const int descriptor = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
  TEST_ASSERT_GREATER_OR_EQUAL(0, descriptor);
  write_all(descriptor, bytes, length);
  TEST_ASSERT_EQUAL_INT(0, fsync(descriptor));
  TEST_ASSERT_EQUAL_INT(0, close(descriptor));
}

size_t
hwtest_encode_reading_record(uint8_t record_type, uint32_t sample_id,
                             const cura_lora_v2_reading_t *reading,
                             uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE]) {
  uint8_t payload[NODE_PERSISTENCE_READING_PAYLOAD_SIZE];
  cura_lora_v2_reading_t stored_reading = *reading;
  stored_reading.sample_id = sample_id;
  TEST_ASSERT_EQUAL(
      CURA_LORA_V2_CODEC_OK,
      cura_lora_v2_encode_reading(payload, sizeof(payload), &stored_reading));
  size_t length = 0U;
  TEST_ASSERT_TRUE(node_persistence_record_encode(
      node_persistence_backend(), record_type, payload, sizeof(payload), output,
      &length));
  return length;
}

size_t hwtest_encode_diagnostic_record(
    const node_diagnostic_event_t *event,
    uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE]) {
  TEST_ASSERT_NOT_NULL(event);
  const diagn_context_t *context = event->context;
  const uint8_t context_length = context == NULL ? 0U : context->context_length;
  uint8_t payload[NODE_PERSISTENCE_DIAGNOSTIC_MAX_PAYLOAD_SIZE];
  node_persistence_store_le16(payload, curag_error_domain(event->error));
  node_persistence_store_le16(payload + 2U, curag_error_code(event->error));
  node_persistence_store_le16(payload + 4U, event->flags);
  node_persistence_store_le32(payload + 6U, event->application_offset_ms);
  node_persistence_store_le32(payload + 10U, event->cycle_sample_id);
  node_persistence_store_le32(payload + 14U, event->message_id);
  node_persistence_store_le16(
      payload + 18U, context == NULL ? CURAG_OP_NONE : context->operation);
  payload[20U] = context_length;
  payload[21U] =
      context == NULL ? CURAG_CONTEXT_SCHEMA_NONE : context->context_schema;
  if (context_length != 0U) {
    memcpy(payload + NODE_PERSISTENCE_DIAGNOSTIC_PREFIX_SIZE, context->context,
           context_length);
  }
  const size_t payload_length =
      NODE_PERSISTENCE_DIAGNOSTIC_PREFIX_SIZE + context_length;
  size_t record_length = 0U;
  TEST_ASSERT_TRUE(node_persistence_record_encode(
      node_persistence_backend(), NODE_PERSISTENCE_RECORD_TYPE_DIAGNOSTIC_EVENT,
      payload, payload_length, output, &record_length));
  return record_length;
}

size_t hwtest_encode_delivery_record(
    const node_delivery_event_t *event,
    uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE]) {
  TEST_ASSERT_NOT_NULL(event);
  uint8_t payload[NODE_PERSISTENCE_DELIVERY_STARTED_PAYLOAD_SIZE];
  node_persistence_store_le32(payload, event->cycle_sample_id);
  node_persistence_store_le32(payload + 4U, event->sample_id);
  node_persistence_store_le32(payload + 8U, event->message_id);
  payload[12U] = event->domain;
  uint8_t record_type = 0U;
  size_t payload_length = 0U;
  if (event->type == NODE_DELIVERY_EVENT_STARTED) {
    node_persistence_store_le32(payload + 13U,
                                event->detail.started.start_offset_ms);
    record_type = NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_STARTED;
    payload_length = NODE_PERSISTENCE_DELIVERY_STARTED_PAYLOAD_SIZE;
  } else {
    TEST_ASSERT_EQUAL(NODE_DELIVERY_EVENT_FINISHED, event->type);
    payload[13U] = event->detail.finished.attempt_count;
    payload[14U] = event->detail.finished.final_result;
    record_type = NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_FINISHED;
    payload_length = NODE_PERSISTENCE_DELIVERY_FINISHED_PAYLOAD_SIZE;
  }
  size_t record_length = 0U;
  TEST_ASSERT_TRUE(node_persistence_record_encode(
      node_persistence_backend(), record_type, payload, payload_length, output,
      &record_length));
  return record_length;
}

void hwtest_recalculate_record_crc(uint8_t *record, size_t record_length) {
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(NODE_PERSISTENCE_RECORD_OVERHEAD,
                                      record_length);
  const uint32_t crc = node_persistence_backend()->crc32_iso_hdlc(
      record, record_length - sizeof(uint32_t));
  node_persistence_store_le32(record + record_length - sizeof(uint32_t), crc);
}

node_diagnostic_event_t hwtest_make_diagnostic(diagn_context_t *context,
                                               uint8_t marker) {
  memset(context, 0, sizeof(*context));
  context->operation = CURAG_OP_WRITE;
  context->context_length = CURAG_PERSISTENCE_CONTEXT_V1_SIZE;
  context->context_schema = CURAG_PERSISTENCE_CONTEXT_V1;
  for (size_t index = 0U; index < context->context_length; ++index) {
    context->context[index] = (uint8_t)(marker + index);
  }
  const node_diagnostic_event_t event = {
      .error = CURAG_EIO,
      .flags = NODE_DIAGNOSTIC_APPLICATION_OFFSET_VALID |
               NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID |
               NODE_DIAGNOSTIC_MESSAGE_ID_VALID,
      .application_offset_ms = UINT32_C(1000) + marker,
      .cycle_sample_id = UINT32_C(2000) + marker,
      .message_id = UINT32_C(3000) + marker,
      .context = context,
  };
  return event;
}

node_delivery_event_t hwtest_make_delivery_started(uint32_t cycle_sample_id,
                                                   uint32_t sample_id) {
  const node_delivery_event_t event = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = cycle_sample_id,
      .sample_id = sample_id,
      .message_id = sample_id + UINT32_C(100),
      .domain = CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
      .detail.started = {.start_offset_ms = 1234U},
  };
  return event;
}

node_delivery_event_t hwtest_make_delivery_finished(uint32_t cycle_sample_id,
                                                    uint32_t sample_id) {
  const node_delivery_event_t event = {
      .type = NODE_DELIVERY_EVENT_FINISHED,
      .cycle_sample_id = cycle_sample_id,
      .sample_id = sample_id,
      .message_id = sample_id + UINT32_C(100),
      .domain = CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
      .detail.finished = {.attempt_count = 2U,
                          .final_result = NODE_DELIVERY_RESULT_ACCEPTED},
  };
  return event;
}

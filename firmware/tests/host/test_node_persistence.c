#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fake_node_persistence_backend.h"
#include "node_common.h"
#include "node_persistence.h"
#include "node_persistence_backend.h"
#include "node_persistence_record.h"
#include "protocol_v2_lora_schema_generated.h"

#define PENDING_PATH NODE_PERSISTENCE_MOUNT_PATH "/pending.log"
#define COMPACT_PATH NODE_PERSISTENCE_MOUNT_PATH "/pending.compact"
#define QUARANTINE_PATH NODE_PERSISTENCE_MOUNT_PATH "/quarantine.log"
#define DIAGNOSTIC_PATH NODE_PERSISTENCE_MOUNT_PATH "/diagnostic.log"
#define DELIVERY_PATH NODE_PERSISTENCE_MOUNT_PATH "/delivery.log"

#define ASSERT_TRUE(expression)                                                \
  do {                                                                         \
    if (!(expression)) {                                                       \
      fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__,     \
              #expression);                                                    \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define ASSERT_EQ_U32(expected, actual)                                        \
  do {                                                                         \
    const uint32_t expected_value_ = (uint32_t)(expected);                     \
    const uint32_t actual_value_ = (uint32_t)(actual);                         \
    if (expected_value_ != actual_value_) {                                    \
      fprintf(stderr, "%s:%d: expected 0x%08lx, got 0x%08lx: %s\n", __FILE__,  \
              __LINE__, (unsigned long)expected_value_,                        \
              (unsigned long)actual_value_, #actual);                          \
      return false;                                                            \
    }                                                                          \
  } while (0)

static void reset_test(void) {
  node_persistence_test_reset();
  fake_backend_reset();
}

static cura_lora_v2_reading_t make_reading(uint16_t run_ms) {
  cura_lora_v2_reading_t reading = {0};
  reading.run_ms = run_ms;
  reading.soil_0_mv = (uint16_t)(1000U + run_ms);
  reading.soil_temp_0_centi_c = (int16_t)(2000 + (int32_t)run_ms);
  reading.flags =
      CURA_LORA_V2_FLAG_SOIL_0_VALID | CURA_LORA_V2_FLAG_SOIL_TEMP_0_VALID;
  return reading;
}

static bool readings_equal(const cura_lora_v2_reading_t *left,
                           const cura_lora_v2_reading_t *right) {
  uint8_t left_body[CURA_LORA_V2_READING_BODY_SIZE];
  uint8_t right_body[CURA_LORA_V2_READING_BODY_SIZE];
  return cura_lora_v2_encode_reading(left_body, sizeof(left_body), left) ==
             CURA_LORA_V2_CODEC_OK &&
         cura_lora_v2_encode_reading(right_body, sizeof(right_body), right) ==
             CURA_LORA_V2_CODEC_OK &&
         memcmp(left_body, right_body, sizeof(left_body)) == 0;
}

static bool assert_persistence_diag(const diagn_context_t *diag,
                                    curag_operation_t operation,
                                    node_persistence_resource_t resource,
                                    node_persistence_stage_t stage,
                                    node_persistence_backend_status_kind_t kind,
                                    int32_t status) {
  ASSERT_EQ_U32(operation, diag->operation);
  ASSERT_EQ_U32(CURAG_PERSISTENCE_CONTEXT_V1_SIZE, diag->context_length);
  ASSERT_EQ_U32(CURAG_PERSISTENCE_CONTEXT_V1, diag->context_schema);
  ASSERT_EQ_U32(resource, diag->context[0]);
  ASSERT_EQ_U32(stage, diag->context[1]);
  ASSERT_EQ_U32(kind, diag->context[2]);
  ASSERT_EQ_U32((uint32_t)status,
                node_persistence_load_le32(diag->context + 3U));
  return true;
}

static bool test_sample_id_claiming_and_exhaustion(void) {
  reset_test();
  diagn_context_t diag;
  uint32_t sample_id = UINT32_MAX;
  ASSERT_EQ_U32(CURAG_OK, node_persistence_claim_sample_id(&sample_id, &diag));
  ASSERT_EQ_U32(0U, sample_id);
  ASSERT_EQ_U32(CURAG_OP_NONE, diag.operation);
  ASSERT_EQ_U32(CURAG_OK, node_persistence_claim_sample_id(&sample_id, &diag));
  ASSERT_EQ_U32(1U, sample_id);
  ASSERT_EQ_U32(1U, fake_backend_calls()->nvs_init);
  ASSERT_EQ_U32(2U, fake_backend_calls()->nvs_commit);

  node_persistence_test_reset();
  ASSERT_EQ_U32(CURAG_OK, node_persistence_claim_sample_id(&sample_id, &diag));
  ASSERT_EQ_U32(2U, sample_id);
  ASSERT_EQ_U32(2U, fake_backend_calls()->nvs_init);

  reset_test();
  fake_backend_seed_next_sample_id(UINT32_MAX);
  sample_id = UINT32_C(0xa5a5a5a5);
  ASSERT_EQ_U32(CURAG_ESAMPLE_ID_EXHAUSTED,
                node_persistence_claim_sample_id(&sample_id, &diag));
  ASSERT_EQ_U32(UINT32_C(0xa5a5a5a5), sample_id);
  ASSERT_TRUE(assert_persistence_diag(
      &diag, CURAG_OP_VALIDATE, NODE_PERSISTENCE_RESOURCE_NVS_SAMPLE_COUNTER,
      NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));

  reset_test();
  fake_backend_fail_next(FAKE_BACKEND_OP_NVS_COMMIT, -77);
  sample_id = UINT32_C(0x12345678);
  ASSERT_EQ_U32(CURAG_ENVS_ACCESS,
                node_persistence_claim_sample_id(&sample_id, &diag));
  ASSERT_EQ_U32(UINT32_C(0x12345678), sample_id);
  ASSERT_TRUE(assert_persistence_diag(
      &diag, CURAG_OP_SYNC, NODE_PERSISTENCE_RESOURCE_NVS_SAMPLE_COUNTER,
      NODE_PERSISTENCE_STAGE_COMMIT, NODE_PERSISTENCE_BACKEND_ESP_ERR, -77));
  return true;
}

static bool test_lazy_initialization_and_failure_caching(void) {
  reset_test();
  fake_backend_fail_next(FAKE_BACKEND_OP_NVS_INIT, -11);
  uint32_t sample_id = 0U;
  diagn_context_t diag;
  ASSERT_EQ_U32(CURAG_ENVS_INIT,
                node_persistence_claim_sample_id(&sample_id, &diag));
  ASSERT_EQ_U32(CURAG_ENVS_INIT,
                node_persistence_claim_sample_id(&sample_id, &diag));
  ASSERT_EQ_U32(1U, fake_backend_calls()->nvs_init);

  const cura_lora_v2_reading_t reading = make_reading(1U);
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_pending_reading(4U, &reading, &diag));
  ASSERT_EQ_U32(1U, fake_backend_calls()->littlefs_mount);

  reset_test();
  fake_backend_fail_next(FAKE_BACKEND_OP_LITTLEFS_MOUNT, -22);
  ASSERT_EQ_U32(CURAG_ELITTLEFS_INIT,
                node_persistence_append_pending_reading(4U, &reading, &diag));
  ASSERT_EQ_U32(CURAG_ELITTLEFS_INIT,
                node_persistence_append_pending_reading(4U, &reading, &diag));
  ASSERT_EQ_U32(1U, fake_backend_calls()->littlefs_mount);
  ASSERT_TRUE(assert_persistence_diag(&diag, CURAG_OP_INITIALIZE,
                                      NODE_PERSISTENCE_RESOURCE_LITTLEFS,
                                      NODE_PERSISTENCE_STAGE_INITIALIZE,
                                      NODE_PERSISTENCE_BACKEND_ESP_ERR, -22));

  ASSERT_EQ_U32(CURAG_OK, node_persistence_claim_sample_id(&sample_id, &diag));
  ASSERT_EQ_U32(1U, fake_backend_calls()->nvs_init);
  return true;
}

static bool test_pending_lifo_and_exact_removal(void) {
  reset_test();
  const cura_lora_v2_reading_t first = make_reading(10U);
  const cura_lora_v2_reading_t second = make_reading(20U);
  diagn_context_t diag;
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_pending_reading(100U, &first, &diag));
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_pending_reading(101U, &second, &diag));
  ASSERT_EQ_U32(2U * 46U, fake_backend_file_size(PENDING_PATH));

  node_persistence_test_reset();
  uint32_t sample_id = 0U;
  cura_lora_v2_reading_t actual;
  bool found = false;
  ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                              &sample_id, &actual, &found, &diag));
  ASSERT_TRUE(found);
  ASSERT_EQ_U32(101U, sample_id);
  ASSERT_TRUE(readings_equal(&second, &actual));

  ASSERT_EQ_U32(CURAG_ERECORD_MISMATCH,
                node_persistence_remove_newest_reading(100U, &diag));
  ASSERT_EQ_U32(2U * 46U, fake_backend_file_size(PENDING_PATH));
  ASSERT_EQ_U32(CURAG_OK, node_persistence_remove_newest_reading(101U, &diag));
  ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                              &sample_id, &actual, &found, &diag));
  ASSERT_TRUE(found);
  ASSERT_EQ_U32(100U, sample_id);
  ASSERT_TRUE(readings_equal(&first, &actual));
  ASSERT_EQ_U32(CURAG_OK, node_persistence_remove_newest_reading(100U, &diag));
  ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                              &sample_id, &actual, &found, &diag));
  ASSERT_TRUE(!found);
  return true;
}

static bool test_record_families_and_sync(void) {
  reset_test();
  const cura_lora_v2_reading_t reading = make_reading(3U);
  diagn_context_t diag;
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_quarantine_reading(9U, &reading, &diag));

  node_delivery_event_t delivery = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = 10U,
      .sample_id = 9U,
      .domain = CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK,
      .detail.started = {.start_offset_ms = 55U},
  };
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_delivery_event(&delivery, &diag));
  delivery.type = NODE_DELIVERY_EVENT_FINISHED;
  delivery.detail.finished.attempt_count = 2U;
  delivery.detail.finished.final_result = NODE_DELIVERY_RESULT_ACCEPTED;
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_delivery_event(&delivery, &diag));

  diagn_context_t cause;
  curag_diagnostic_context_clear(&cause);
  cause.operation = CURAG_OP_WRITE;
  cause.context_length = CURAG_PERSISTENCE_CONTEXT_V1_SIZE;
  cause.context_schema = CURAG_PERSISTENCE_CONTEXT_V1;
  cause.context[0] = NODE_PERSISTENCE_RESOURCE_PENDING_LOG;
  cause.context[1] = NODE_PERSISTENCE_STAGE_WRITE;
  cause.context[2] = NODE_PERSISTENCE_BACKEND_ERRNO;
  node_persistence_store_le32(cause.context + 3U, 5U);
  const node_diagnostic_event_t event = {
      .error = CURAG_EIO,
      .flags = NODE_DIAGNOSTIC_APPLICATION_OFFSET_VALID |
               NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID,
      .application_offset_ms = 44U,
      .cycle_sample_id = 10U,
      .context = &cause,
  };
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_diagnostic_event(&event, &diag));

  ASSERT_EQ_U32(46U, fake_backend_file_size(QUARANTINE_PATH));
  ASSERT_EQ_U32(52U, fake_backend_file_size(DELIVERY_PATH));
  ASSERT_EQ_U32(39U, fake_backend_file_size(DIAGNOSTIC_PATH));
  const uint32_t syncs_before = fake_backend_calls()->file_sync;
  ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  ASSERT_TRUE(fake_backend_calls()->file_sync >= syncs_before + 3U);

  uint8_t bytes[100];
  size_t length = 0U;
  ASSERT_TRUE(
      fake_backend_read_file(QUARANTINE_PATH, bytes, sizeof(bytes), &length));
  ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_VALID,
                node_persistence_record_validate(
                    node_persistence_backend(), NODE_PERSISTENCE_LOG_QUARANTINE,
                    bytes, length));
  ASSERT_TRUE(
      fake_backend_read_file(DIAGNOSTIC_PATH, bytes, sizeof(bytes), &length));
  ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_VALID,
                node_persistence_record_validate(
                    node_persistence_backend(), NODE_PERSISTENCE_LOG_DIAGNOSTIC,
                    bytes, length));
  return true;
}

static bool test_invalid_inputs_and_diagnostics(void) {
  reset_test();
  diagn_context_t diag;
  memset(&diag, 0xa5, sizeof(diag));
  ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                node_persistence_claim_sample_id(NULL, &diag));
  ASSERT_TRUE(assert_persistence_diag(
      &diag, CURAG_OP_VALIDATE, NODE_PERSISTENCE_RESOURCE_NVS_SAMPLE_COUNTER,
      NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));

  cura_lora_v2_reading_t invalid = {0};
  invalid.soil_0_mv = 1U;
  ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                node_persistence_append_pending_reading(0U, &invalid, &diag));
  ASSERT_TRUE(assert_persistence_diag(
      &diag, CURAG_OP_VALIDATE, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));

  uint32_t id = 0U;
  cura_lora_v2_reading_t reading;
  bool found = false;
  ASSERT_EQ_U32(
      CURAG_EINVALID_ARGUMENT,
      node_persistence_peek_most_recent_pending(&id, &reading, NULL, &diag));
  ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                node_persistence_append_diagnostic_event(NULL, &diag));

  const cura_lora_v2_reading_t valid = make_reading(2U);
  memset(&diag, 0xa5, sizeof(diag));
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_pending_reading(1U, &valid, &diag));
  ASSERT_EQ_U32(CURAG_OP_NONE, diag.operation);
  ASSERT_EQ_U32(0U, diag.context_length);
  ASSERT_EQ_U32(0U, diag.context_schema);
  ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(NULL));
  (void)found;
  return true;
}

static bool test_backend_failure_contexts_and_sync_behavior(void) {
  reset_test();
  diagn_context_t diag;
  ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  ASSERT_EQ_U32(0U, fake_backend_calls()->nvs_init);
  ASSERT_EQ_U32(0U, fake_backend_calls()->littlefs_mount);
  ASSERT_EQ_U32(0U, fake_backend_calls()->file_sync);

  const cura_lora_v2_reading_t reading = make_reading(2U);
  fake_backend_fail_next(FAKE_BACKEND_OP_FILE_WRITE, 28);
  ASSERT_EQ_U32(CURAG_EIO,
                node_persistence_append_pending_reading(1U, &reading, &diag));
  ASSERT_TRUE(assert_persistence_diag(
      &diag, CURAG_OP_APPEND, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_WRITE, NODE_PERSISTENCE_BACKEND_ERRNO, 28));
  ASSERT_EQ_U32(0U, fake_backend_file_size(PENDING_PATH));

  reset_test();
  const node_diagnostic_event_t event = {
      .error = CURAG_EIO,
      .flags = 0U,
      .application_offset_ms = 0U,
      .cycle_sample_id = 0U,
      .context = NULL,
  };
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_diagnostic_event(&event, &diag));
  fake_backend_fail_next(FAKE_BACKEND_OP_FILE_SYNC, 5);
  ASSERT_EQ_U32(CURAG_EIO, node_persistence_sync_all(&diag));
  ASSERT_TRUE(assert_persistence_diag(
      &diag, CURAG_OP_SYNC, NODE_PERSISTENCE_RESOURCE_DIAGNOSTIC_LOG,
      NODE_PERSISTENCE_STAGE_SYNC, NODE_PERSISTENCE_BACKEND_ERRNO, 5));
  const uint32_t mounts = fake_backend_calls()->littlefs_mount;
  ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  ASSERT_EQ_U32(mounts, fake_backend_calls()->littlefs_mount);
  return true;
}

static bool test_tail_recovery_and_untrusted_boundary(void) {
  reset_test();
  const cura_lora_v2_reading_t reading = make_reading(7U);
  diagn_context_t diag;
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_pending_reading(7U, &reading, &diag));
  ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));

  const uint8_t torn[] = {0x23U, 0xecU, 0x6fU};
  ASSERT_TRUE(fake_backend_append_bytes(PENDING_PATH, torn, sizeof(torn)));
  uint32_t id = 0U;
  cura_lora_v2_reading_t actual;
  bool found = false;
  ASSERT_EQ_U32(
      CURAG_ECORRUPT_RECORD,
      node_persistence_peek_most_recent_pending(&id, &actual, &found, &diag));
  ASSERT_EQ_U32(46U, fake_backend_file_size(PENDING_PATH));
  ASSERT_TRUE(assert_persistence_diag(
      &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_TRUNCATE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));
  ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                              &id, &actual, &found, &diag));
  ASSERT_TRUE(found && id == 7U);

  uint8_t unsupported[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t unsupported_length = 0U;
  const uint8_t payload[] = {1U, 2U, 3U};
  ASSERT_TRUE(node_persistence_record_encode(
      node_persistence_backend(), UINT8_C(0x77), payload, sizeof(payload),
      unsupported, &unsupported_length));
  ASSERT_TRUE(
      fake_backend_append_bytes(PENDING_PATH, unsupported, unsupported_length));
  ASSERT_EQ_U32(
      CURAG_EUNSUPPORTED_RECORD,
      node_persistence_peek_most_recent_pending(&id, &actual, &found, &diag));
  ASSERT_EQ_U32(46U, fake_backend_file_size(PENDING_PATH));

  ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  node_persistence_test_reset();
  uint8_t garbage[600];
  memset(garbage, 0xa5, sizeof(garbage));
  ASSERT_TRUE(fake_backend_write_file(PENDING_PATH, garbage, sizeof(garbage)));
  ASSERT_EQ_U32(
      CURAG_ECORRUPT_RECORD,
      node_persistence_peek_most_recent_pending(&id, &actual, &found, &diag));
  ASSERT_EQ_U32(sizeof(garbage), fake_backend_file_size(PENDING_PATH));
  ASSERT_TRUE(assert_persistence_diag(
      &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_TAIL_SCAN, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));
  return true;
}

static bool test_pending_compaction_retains_newest_half(void) {
  reset_test();
  diagn_context_t diag;
  for (uint32_t id = 0U; id < 6U; ++id) {
    const cura_lora_v2_reading_t reading = make_reading((uint16_t)id);
    ASSERT_EQ_U32(CURAG_OK,
                  node_persistence_append_pending_reading(id, &reading, &diag));
  }
  ASSERT_EQ_U32(3U * 46U, fake_backend_file_size(PENDING_PATH));
  ASSERT_TRUE(!fake_backend_file_exists(COMPACT_PATH));

  const uint32_t expected[] = {5U, 4U, 3U};
  for (size_t index = 0U; index < sizeof(expected) / sizeof(expected[0]);
       ++index) {
    uint32_t id = 0U;
    cura_lora_v2_reading_t reading;
    bool found = false;
    ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                &id, &reading, &found, &diag));
    ASSERT_TRUE(found);
    ASSERT_EQ_U32(expected[index], id);
    ASSERT_EQ_U32(CURAG_OK, node_persistence_remove_newest_reading(id, &diag));
  }
  uint32_t id = 0U;
  cura_lora_v2_reading_t reading;
  bool found = true;
  ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                              &id, &reading, &found, &diag));
  ASSERT_TRUE(!found);
  return true;
}

static bool test_fixed_log_quota_and_stale_compact(void) {
  reset_test();
  const cura_lora_v2_reading_t reading = make_reading(1U);
  diagn_context_t diag;
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_quarantine_reading(1U, &reading, &diag));
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_quarantine_reading(2U, &reading, &diag));
  ASSERT_EQ_U32(CURAG_ELOG_FULL,
                node_persistence_quarantine_reading(3U, &reading, &diag));
  ASSERT_EQ_U32(92U, fake_backend_file_size(QUARANTINE_PATH));
  ASSERT_TRUE(assert_persistence_diag(&diag, CURAG_OP_APPEND,
                                      NODE_PERSISTENCE_RESOURCE_QUARANTINE_LOG,
                                      NODE_PERSISTENCE_STAGE_QUOTA_CHECK,
                                      NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));

  reset_test();
  ASSERT_EQ_U32(CURAG_OK,
                node_persistence_append_pending_reading(1U, &reading, &diag));
  ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  const uint8_t stale[] = {1U, 2U, 3U};
  ASSERT_TRUE(fake_backend_write_file(COMPACT_PATH, stale, sizeof(stale)));
  node_persistence_test_reset();
  uint32_t id = 0U;
  cura_lora_v2_reading_t actual;
  bool found = false;
  ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                              &id, &actual, &found, &diag));
  ASSERT_TRUE(found && id == 1U);
  ASSERT_TRUE(!fake_backend_file_exists(COMPACT_PATH));

  reset_test();
  ASSERT_TRUE(fake_backend_write_file(COMPACT_PATH, stale, sizeof(stale)));
  ASSERT_EQ_U32(
      CURAG_ECORRUPT_RECORD,
      node_persistence_peek_most_recent_pending(&id, &actual, &found, &diag));
  ASSERT_TRUE(fake_backend_file_exists(COMPACT_PATH));
  return true;
}

static bool test_record_codec_boundaries(void) {
  reset_test();
  static const uint8_t check[] = "123456789";
  ASSERT_EQ_U32(UINT32_C(0xcbf43926),
                node_persistence_backend()->crc32_iso_hdlc(check, 9U));

  uint8_t payload[NODE_PERSISTENCE_RECORD_MAX_PAYLOAD_SIZE];
  memset(payload, 0x5a, sizeof(payload));
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t length = 0U;
  ASSERT_TRUE(node_persistence_record_encode(
      node_persistence_backend(), NODE_PERSISTENCE_RECORD_TYPE_DIAGNOSTIC_EVENT,
      payload, sizeof(payload), record, &length));
  ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_MAX_SIZE, length);
  ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_VALID,
                node_persistence_record_validate_structural(
                    node_persistence_backend(), record, length));
  record[length - 1U] ^= UINT8_C(0x01);
  ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_INVALID_FRAMING,
                node_persistence_record_validate_structural(
                    node_persistence_backend(), record, length));
  ASSERT_TRUE(!node_persistence_record_encode(node_persistence_backend(), 1U,
                                              payload, sizeof(payload) + 1U,
                                              record, &length));
  ASSERT_TRUE(!node_persistence_record_encode(
      NULL, 1U, payload, sizeof(payload), record, &length));
  return true;
}

typedef bool (*test_function_t)(void);

typedef struct {
  const char *name;
  test_function_t function;
} test_case_t;

int main(void) {
  static const test_case_t tests[] = {
      {"sample_id_claiming_and_exhaustion",
       test_sample_id_claiming_and_exhaustion},
      {"lazy_initialization_and_failure_caching",
       test_lazy_initialization_and_failure_caching},
      {"pending_lifo_and_exact_removal", test_pending_lifo_and_exact_removal},
      {"record_families_and_sync", test_record_families_and_sync},
      {"invalid_inputs_and_diagnostics", test_invalid_inputs_and_diagnostics},
      {"backend_failure_contexts_and_sync_behavior",
       test_backend_failure_contexts_and_sync_behavior},
      {"tail_recovery_and_untrusted_boundary",
       test_tail_recovery_and_untrusted_boundary},
      {"pending_compaction_retains_newest_half",
       test_pending_compaction_retains_newest_half},
      {"fixed_log_quota_and_stale_compact",
       test_fixed_log_quota_and_stale_compact},
      {"record_codec_boundaries", test_record_codec_boundaries},
  };

  size_t failures = 0U;
  for (size_t index = 0U; index < sizeof(tests) / sizeof(tests[0]); ++index) {
    if (tests[index].function()) {
      printf("PASS %s\n", tests[index].name);
    } else {
      printf("FAIL %s\n", tests[index].name);
      ++failures;
    }
  }
  node_persistence_test_reset();
  if (failures != 0U) {
    fprintf(stderr, "%zu node_persistence test(s) failed\n", failures);
    return 1;
  }
  return 0;
}

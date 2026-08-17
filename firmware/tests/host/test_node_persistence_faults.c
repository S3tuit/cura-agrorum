#include "node_persistence_test.h"

#include <errno.h>

typedef struct {
  fake_backend_operation_t backend_operation;
  err_curag_t error;
  curag_operation_t operation;
  node_persistence_stage_t stage;
  int32_t status;
} nvs_failure_case_t;

static bool every_nvs_failure_has_exact_error_and_context(void) {
  static const nvs_failure_case_t cases[] = {
      {FAKE_BACKEND_OP_NVS_INIT, CURAG_ENVS_INIT, CURAG_OP_INITIALIZE,
       NODE_PERSISTENCE_STAGE_INITIALIZE, -101},
      {FAKE_BACKEND_OP_NVS_OPEN, CURAG_ENVS_ACCESS, CURAG_OP_INITIALIZE,
       NODE_PERSISTENCE_STAGE_OPEN, -102},
      {FAKE_BACKEND_OP_NVS_GET, CURAG_ENVS_ACCESS, CURAG_OP_READ,
       NODE_PERSISTENCE_STAGE_GET, -103},
      {FAKE_BACKEND_OP_NVS_SET, CURAG_ENVS_ACCESS, CURAG_OP_WRITE,
       NODE_PERSISTENCE_STAGE_SET, -104},
      {FAKE_BACKEND_OP_NVS_COMMIT, CURAG_ENVS_ACCESS, CURAG_OP_SYNC,
       NODE_PERSISTENCE_STAGE_COMMIT, -105},
  };

  for (size_t index = 0U; index < sizeof(cases) / sizeof(cases[0]); ++index) {
    node_persistence_test_reset_all();
    fake_backend_fail_on(cases[index].backend_operation,
                         FAKE_BACKEND_RESOURCE_NVS, 1U, cases[index].status);
    diagn_context_t diag;
    uint32_t id = UINT32_C(0xa5a5a5a5);
    TEST_ASSERT_EQ_U32(cases[index].error,
                       node_persistence_claim_sample_id(&id, &diag));
    TEST_ASSERT_EQ_U32(UINT32_C(0xa5a5a5a5), id);
    TEST_ASSERT(node_persistence_test_assert_diag(
        &diag, cases[index].operation,
        NODE_PERSISTENCE_RESOURCE_NVS_SAMPLE_COUNTER, cases[index].stage,
        NODE_PERSISTENCE_BACKEND_ESP_ERR, cases[index].status));
  }
  return true;
}

static bool basic_file_failures_have_exact_error_and_context(void) {
  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(1U);
  diagn_context_t diag;

  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_PATH_STAT, FAKE_BACKEND_RESOURCE_COMPACT,
                       1U, EACCES);
  TEST_ASSERT_EQ_U32(CURAG_EIO,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
      NODE_PERSISTENCE_STAGE_STAT, NODE_PERSISTENCE_BACKEND_ERRNO, EACCES));

  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_OPEN, FAKE_BACKEND_RESOURCE_PENDING,
                       1U, EMFILE);
  TEST_ASSERT_EQ_U32(CURAG_EIO,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_APPEND, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_OPEN, NODE_PERSISTENCE_BACKEND_ERRNO, EMFILE));

  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_WRITE,
                       FAKE_BACKEND_RESOURCE_PENDING, 1U, ENOSPC);
  TEST_ASSERT_EQ_U32(CURAG_EIO,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_APPEND, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_WRITE, NODE_PERSISTENCE_BACKEND_ERRNO, ENOSPC));

  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_SYNC, FAKE_BACKEND_RESOURCE_PENDING,
                       1U, EIO);
  TEST_ASSERT_EQ_U32(CURAG_EIO,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_SYNC, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_SYNC, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));

  node_persistence_test_reset_all();
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&reading, &diag));
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_SIZE, FAKE_BACKEND_RESOURCE_PENDING,
                       1U, EOVERFLOW);
  node_pending_reading_t pending;
  bool found = false;
  TEST_ASSERT_EQ_U32(CURAG_EIO, node_persistence_peek_most_recent_pending(
                                    &pending, &found, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_STAT, NODE_PERSISTENCE_BACKEND_ERRNO, EOVERFLOW));

  node_persistence_test_restart();
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_READ, FAKE_BACKEND_RESOURCE_PENDING,
                       1U, EIO);
  TEST_ASSERT_EQ_U32(CURAG_EIO, node_persistence_peek_most_recent_pending(
                                    &pending, &found, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_READ, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));

  node_persistence_test_restart();
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_TRUNCATE,
                       FAKE_BACKEND_RESOURCE_PENDING, 1U, EIO);
  TEST_ASSERT_EQ_U32(CURAG_EIO,
                     node_persistence_remove_newest_reading(1U, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_REMOVE, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_TRUNCATE, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));

  node_persistence_test_restart();
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                   &pending, &found, &diag));
  TEST_ASSERT(found);
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_CLOSE,
                       FAKE_BACKEND_RESOURCE_PENDING, 1U, EIO);
  TEST_ASSERT_EQ_U32(CURAG_EIO, node_persistence_sync_all(&diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_SYNC, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_CLOSE, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));
  return true;
}

static bool stale_compact_remove_failure_has_exact_context(void) {
  node_persistence_test_reset_all();
  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(1U);
  diagn_context_t diag;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  const uint8_t stale[] = {1U, 2U, 3U};
  TEST_ASSERT(fake_backend_write_file(TEST_COMPACT_PATH, stale, sizeof(stale)));
  node_persistence_test_restart();
  fake_backend_fail_on(FAKE_BACKEND_OP_PATH_REMOVE,
                       FAKE_BACKEND_RESOURCE_COMPACT, 1U, EACCES);
  node_pending_reading_t pending;
  bool found = false;
  TEST_ASSERT_EQ_U32(CURAG_EIO, node_persistence_peek_most_recent_pending(
                                    &pending, &found, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
      NODE_PERSISTENCE_STAGE_REMOVE, NODE_PERSISTENCE_BACKEND_ERRNO, EACCES));
  TEST_ASSERT(fake_backend_file_exists(TEST_COMPACT_PATH));
  return true;
}

typedef struct {
  fake_backend_operation_t backend_operation;
  fake_backend_resource_t backend_resource;
  uint32_t occurrence;
  node_persistence_resource_t diagnostic_resource;
  node_persistence_stage_t stage;
} compaction_failure_case_t;

static bool prepare_pending_for_compaction(diagn_context_t *diag) {
  for (uint32_t id = 0U; id < 5U; ++id) {
    const cura_lora_v2_reading_t reading =
        node_persistence_test_make_reading((uint16_t)id);
    if (node_persistence_append_pending_reading(&reading, diag) != CURAG_OK) {
      return false;
    }
  }
  return true;
}

static bool every_compaction_backend_failure_has_exact_context(void) {
  static const compaction_failure_case_t cases[] = {
      {FAKE_BACKEND_OP_FILE_READ, FAKE_BACKEND_RESOURCE_PENDING, 3U,
       NODE_PERSISTENCE_RESOURCE_PENDING_LOG, NODE_PERSISTENCE_STAGE_READ},
      {FAKE_BACKEND_OP_FILE_OPEN, FAKE_BACKEND_RESOURCE_COMPACT, 1U,
       NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT, NODE_PERSISTENCE_STAGE_OPEN},
      {FAKE_BACKEND_OP_FILE_WRITE, FAKE_BACKEND_RESOURCE_COMPACT, 1U,
       NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT, NODE_PERSISTENCE_STAGE_WRITE},
      {FAKE_BACKEND_OP_FILE_SYNC, FAKE_BACKEND_RESOURCE_COMPACT, 1U,
       NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT, NODE_PERSISTENCE_STAGE_SYNC},
      {FAKE_BACKEND_OP_FILE_CLOSE, FAKE_BACKEND_RESOURCE_COMPACT, 1U,
       NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT, NODE_PERSISTENCE_STAGE_CLOSE},
      {FAKE_BACKEND_OP_PATH_RENAME, FAKE_BACKEND_RESOURCE_COMPACT, 1U,
       NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
       NODE_PERSISTENCE_STAGE_RENAME},
  };

  for (size_t index = 0U; index < sizeof(cases) / sizeof(cases[0]); ++index) {
    node_persistence_test_reset_all();
    diagn_context_t diag;
    TEST_ASSERT(prepare_pending_for_compaction(&diag));
    fake_backend_fail_on(cases[index].backend_operation,
                         cases[index].backend_resource, cases[index].occurrence,
                         EIO);
    const cura_lora_v2_reading_t reading =
        node_persistence_test_make_reading(5U);
    TEST_ASSERT_EQ_U32(
        CURAG_EIO, node_persistence_append_pending_reading(&reading, &diag));
    TEST_ASSERT(node_persistence_test_assert_diag(
        &diag, CURAG_OP_COMPACT, cases[index].diagnostic_resource,
        cases[index].stage, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));
  }
  return true;
}

static bool recovery_primitive_failures_are_not_hidden(void) {
  uint8_t first[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t first_length = 0U;
  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(1U);
  TEST_ASSERT(node_persistence_test_encode_reading_record(
      NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING, 1U, &reading, first,
      &first_length));
  const uint8_t suffix[] = {1U, 2U, 3U};

  node_persistence_test_reset_all();
  TEST_ASSERT(fake_backend_write_file(TEST_PENDING_PATH, first, first_length));
  TEST_ASSERT(
      fake_backend_append_bytes(TEST_PENDING_PATH, suffix, sizeof(suffix)));
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_TRUNCATE,
                       FAKE_BACKEND_RESOURCE_PENDING, 1U, EIO);
  diagn_context_t diag;
  node_pending_reading_t pending;
  bool found = false;
  TEST_ASSERT_EQ_U32(CURAG_EIO, node_persistence_peek_most_recent_pending(
                                    &pending, &found, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_TRUNCATE, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));

  node_persistence_test_reset_all();
  TEST_ASSERT(fake_backend_write_file(TEST_PENDING_PATH, first, first_length));
  TEST_ASSERT(
      fake_backend_append_bytes(TEST_PENDING_PATH, suffix, sizeof(suffix)));
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_SYNC, FAKE_BACKEND_RESOURCE_PENDING,
                       1U, EIO);
  TEST_ASSERT_EQ_U32(CURAG_EIO, node_persistence_peek_most_recent_pending(
                                    &pending, &found, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_SYNC, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));
  TEST_ASSERT_EQ_SIZE(first_length, fake_backend_file_size(TEST_PENDING_PATH));
  return true;
}

static bool record_family_failures_name_the_correct_resource(void) {
  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(1U);
  diagn_context_t diag;

  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_WRITE,
                       FAKE_BACKEND_RESOURCE_QUARANTINE, 1U, EIO);
  TEST_ASSERT_EQ_U32(CURAG_EIO,
                     node_persistence_quarantine_reading(&reading, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_APPEND, NODE_PERSISTENCE_RESOURCE_QUARANTINE_LOG,
      NODE_PERSISTENCE_STAGE_WRITE, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));

  node_persistence_test_reset_all();
  const node_diagnostic_event_t diagnostic = {.error = CURAG_EIO};
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_WRITE,
                       FAKE_BACKEND_RESOURCE_DIAGNOSTIC, 1U, EIO);
  TEST_ASSERT_EQ_U32(
      CURAG_EIO, node_persistence_append_diagnostic_event(&diagnostic, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_APPEND, NODE_PERSISTENCE_RESOURCE_DIAGNOSTIC_LOG,
      NODE_PERSISTENCE_STAGE_WRITE, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));

  node_persistence_test_reset_all();
  const node_delivery_event_t delivery = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .domain = CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
  };
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_SYNC,
                       FAKE_BACKEND_RESOURCE_DELIVERY, 1U, EIO);
  TEST_ASSERT_EQ_U32(CURAG_EIO,
                     node_persistence_append_delivery_event(&delivery, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_SYNC, NODE_PERSISTENCE_RESOURCE_DELIVERY_LOG,
      NODE_PERSISTENCE_STAGE_SYNC, NODE_PERSISTENCE_BACKEND_ERRNO, EIO));
  return true;
}

static bool sync_all_skips_unused_and_cached_failed_backends(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_NVS_INIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                                             FAKE_BACKEND_RESOURCE_LITTLEFS));

  fake_backend_fail_on(FAKE_BACKEND_OP_NVS_INIT, FAKE_BACKEND_RESOURCE_NVS, 1U,
                       -1);
  uint32_t id = 0U;
  TEST_ASSERT_EQ_U32(CURAG_ENVS_INIT,
                     node_persistence_claim_sample_id(&id, &diag));
  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(1U);
  fake_backend_fail_on(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                       FAKE_BACKEND_RESOURCE_LITTLEFS, 1U, -2);
  TEST_ASSERT_EQ_U32(CURAG_ELITTLEFS_INIT,
                     node_persistence_append_pending_reading(&reading, &diag));
  const size_t nvs_initializations =
      fake_backend_count(FAKE_BACKEND_OP_NVS_INIT, FAKE_BACKEND_RESOURCE_NVS);
  const size_t mounts = fake_backend_count(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                                           FAKE_BACKEND_RESOURCE_LITTLEFS);
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  TEST_ASSERT_EQ_SIZE(
      nvs_initializations,
      fake_backend_count(FAKE_BACKEND_OP_NVS_INIT, FAKE_BACKEND_RESOURCE_NVS));
  TEST_ASSERT_EQ_SIZE(mounts,
                      fake_backend_count(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                                         FAKE_BACKEND_RESOURCE_LITTLEFS));
  return true;
}

static bool null_diagnostic_output_does_not_change_failure(void) {
  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(1U);
  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_WRITE,
                       FAKE_BACKEND_RESOURCE_PENDING, 1U, EIO);
  diagn_context_t diag;
  const err_curag_t with_diag =
      node_persistence_append_pending_reading(&reading, &diag);

  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_WRITE,
                       FAKE_BACKEND_RESOURCE_PENDING, 1U, EIO);
  const err_curag_t without_diag =
      node_persistence_append_pending_reading(&reading, NULL);
  TEST_ASSERT_EQ_U32(CURAG_EIO, with_diag);
  TEST_ASSERT_EQ_U32(with_diag, without_diag);
  return true;
}

static bool empty_backlog_is_distinct_from_backend_failure(void) {
  node_persistence_test_reset_all();
  node_pending_reading_t pending;
  bool found = true;
  diagn_context_t diag;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                   &pending, &found, &diag));
  TEST_ASSERT(!found);

  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                       FAKE_BACKEND_RESOURCE_LITTLEFS, 1U, -9);
  TEST_ASSERT_EQ_U32(
      CURAG_ELITTLEFS_INIT,
      node_persistence_peek_most_recent_pending(&pending, &found, &diag));
  return true;
}

static const node_persistence_test_case_t CASES[] = {
    {"every_nvs_failure_has_exact_error_and_context",
     every_nvs_failure_has_exact_error_and_context},
    {"basic_file_failures_have_exact_error_and_context",
     basic_file_failures_have_exact_error_and_context},
    {"stale_compact_remove_failure_has_exact_context",
     stale_compact_remove_failure_has_exact_context},
    {"every_compaction_backend_failure_has_exact_context",
     every_compaction_backend_failure_has_exact_context},
    {"recovery_primitive_failures_are_not_hidden",
     recovery_primitive_failures_are_not_hidden},
    {"record_family_failures_name_the_correct_resource",
     record_family_failures_name_the_correct_resource},
    {"sync_all_skips_unused_and_cached_failed_backends",
     sync_all_skips_unused_and_cached_failed_backends},
    {"null_diagnostic_output_does_not_change_failure",
     null_diagnostic_output_does_not_change_failure},
    {"empty_backlog_is_distinct_from_backend_failure",
     empty_backlog_is_distinct_from_backend_failure},
};

const node_persistence_test_group_t NODE_PERSISTENCE_FAULT_TEST_GROUP = {
    .name = "faults",
    .cases = CASES,
    .count = sizeof(CASES) / sizeof(CASES[0]),
};

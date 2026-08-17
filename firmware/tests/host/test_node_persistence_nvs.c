#include "node_persistence_test.h"

#include <string.h>

static bool sample_ids_are_committed_across_restarts(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  for (uint32_t expected = 0U; expected < 3U; ++expected) {
    uint32_t actual = UINT32_MAX;
    TEST_ASSERT_EQ_U32(CURAG_OK,
                       node_persistence_claim_sample_id(&actual, &diag));
    TEST_ASSERT_EQ_U32(expected, actual);
    TEST_ASSERT(fake_backend_next_sample_id_found());
    TEST_ASSERT_EQ_U32(expected + 1U, fake_backend_next_sample_id());
    node_persistence_test_restart();
  }
  TEST_ASSERT_EQ_SIZE(3U, fake_backend_count(FAKE_BACKEND_OP_NVS_INIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                                             FAKE_BACKEND_RESOURCE_LITTLEFS));
  return true;
}

static bool repeated_claims_initialize_once_per_wake(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  uint32_t sample_id = UINT32_MAX;
  for (uint32_t expected = 0U; expected < 4U; ++expected) {
    TEST_ASSERT_EQ_U32(CURAG_OK,
                       node_persistence_claim_sample_id(&sample_id, &diag));
    TEST_ASSERT_EQ_U32(expected, sample_id);
  }
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_NVS_INIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_NVS_OPEN,
                                             FAKE_BACKEND_RESOURCE_NVS));
  TEST_ASSERT_EQ_SIZE(4U, fake_backend_count(FAKE_BACKEND_OP_NVS_COMMIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  return true;
}

static bool message_ids_are_committed_across_restarts(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  for (uint32_t expected = 0U; expected < 3U; ++expected) {
    uint32_t actual = UINT32_MAX;
    TEST_ASSERT_EQ_U32(CURAG_OK,
                       node_persistence_claim_message_id(&actual, &diag));
    TEST_ASSERT_EQ_U32(expected, actual);
    TEST_ASSERT(fake_backend_next_message_id_found());
    TEST_ASSERT_EQ_U32(expected + 1U, fake_backend_next_message_id());
    node_persistence_test_restart();
  }
  TEST_ASSERT_EQ_SIZE(3U, fake_backend_count(FAKE_BACKEND_OP_NVS_INIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  return true;
}

static bool sample_and_message_claims_are_independent(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  uint32_t sample_id = UINT32_MAX;
  uint32_t message_id = UINT32_MAX;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQ_U32(0U, sample_id);
  TEST_ASSERT_EQ_U32(0U, message_id);
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQ_U32(1U, message_id);
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQ_U32(1U, sample_id);
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_NVS_INIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  TEST_ASSERT_EQ_SIZE(4U, fake_backend_count(FAKE_BACKEND_OP_NVS_COMMIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  return true;
}

static bool exhaustion_is_stable_and_returns_no_id(void) {
  node_persistence_test_reset_all();
  fake_backend_seed_next_sample_id(UINT32_MAX);
  diagn_context_t diag;
  for (size_t attempt = 0U; attempt < 2U; ++attempt) {
    uint32_t sample_id = UINT32_C(0xa5a5a5a5);
    TEST_ASSERT_EQ_U32(CURAG_ESAMPLE_ID_EXHAUSTED,
                       node_persistence_claim_sample_id(&sample_id, &diag));
    TEST_ASSERT_EQ_U32(UINT32_C(0xa5a5a5a5), sample_id);
    TEST_ASSERT(node_persistence_test_assert_diag(
        &diag, CURAG_OP_VALIDATE, NODE_PERSISTENCE_RESOURCE_NVS_SAMPLE_COUNTER,
        NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));
    TEST_ASSERT_EQ_U32(UINT32_MAX, fake_backend_next_sample_id());
    node_persistence_test_restart();
  }
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_NVS_SET,
                                             FAKE_BACKEND_RESOURCE_NVS));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_NVS_COMMIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  return true;
}

static bool commit_failure_returns_no_id(void) {
  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_NVS_COMMIT, FAKE_BACKEND_RESOURCE_NVS,
                       1U, -77);
  diagn_context_t diag;
  uint32_t sample_id = UINT32_C(0x12345678);
  TEST_ASSERT_EQ_U32(CURAG_ENVS_ACCESS,
                     node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQ_U32(UINT32_C(0x12345678), sample_id);
  TEST_ASSERT(!fake_backend_next_sample_id_found());
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_SYNC, NODE_PERSISTENCE_RESOURCE_NVS_SAMPLE_COUNTER,
      NODE_PERSISTENCE_STAGE_COMMIT, NODE_PERSISTENCE_BACKEND_ESP_ERR, -77));

  node_persistence_test_restart();
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQ_U32(0U, sample_id);
  return true;
}

static bool message_exhaustion_is_stable_and_returns_no_id(void) {
  node_persistence_test_reset_all();
  fake_backend_seed_next_message_id(UINT32_MAX);
  diagn_context_t diag;
  for (size_t attempt = 0U; attempt < 2U; ++attempt) {
    uint32_t message_id = UINT32_C(0xa5a5a5a5);
    TEST_ASSERT_EQ_U32(CURAG_EMESSAGE_ID_EXHAUSTED,
                       node_persistence_claim_message_id(&message_id, &diag));
    TEST_ASSERT_EQ_U32(UINT32_C(0xa5a5a5a5), message_id);
    TEST_ASSERT(node_persistence_test_assert_diag(
        &diag, CURAG_OP_VALIDATE, NODE_PERSISTENCE_RESOURCE_NVS_MESSAGE_COUNTER,
        NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));
    TEST_ASSERT_EQ_U32(UINT32_MAX, fake_backend_next_message_id());
    node_persistence_test_restart();
  }
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_NVS_SET,
                                             FAKE_BACKEND_RESOURCE_NVS));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_NVS_COMMIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  return true;
}

static bool message_commit_failure_returns_no_id(void) {
  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_NVS_COMMIT, FAKE_BACKEND_RESOURCE_NVS,
                       1U, -78);
  diagn_context_t diag;
  uint32_t message_id = UINT32_C(0x12345678);
  TEST_ASSERT_EQ_U32(CURAG_ENVS_ACCESS,
                     node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQ_U32(UINT32_C(0x12345678), message_id);
  TEST_ASSERT(!fake_backend_next_message_id_found());
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_SYNC, NODE_PERSISTENCE_RESOURCE_NVS_MESSAGE_COUNTER,
      NODE_PERSISTENCE_STAGE_COMMIT, NODE_PERSISTENCE_BACKEND_ESP_ERR, -78));

  node_persistence_test_restart();
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQ_U32(0U, message_id);
  return true;
}

static bool ambiguous_message_commit_may_skip_but_never_reuses(void) {
  node_persistence_test_reset_all();
  fake_backend_ambiguous_nvs_commit_on(1U, -79);
  diagn_context_t diag;
  uint32_t message_id = UINT32_C(0x12345678);
  TEST_ASSERT_EQ_U32(CURAG_ENVS_ACCESS,
                     node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQ_U32(UINT32_C(0x12345678), message_id);
  TEST_ASSERT(fake_backend_next_message_id_found());
  TEST_ASSERT_EQ_U32(1U, fake_backend_next_message_id());

  node_persistence_test_restart();
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQ_U32(1U, message_id);
  TEST_ASSERT_EQ_U32(2U, fake_backend_next_message_id());
  return true;
}

static bool backend_initialization_is_independent_and_failure_is_cached(void) {
  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_NVS_INIT, FAKE_BACKEND_RESOURCE_NVS, 1U,
                       -11);
  diagn_context_t diag;
  uint32_t sample_id = 0U;
  TEST_ASSERT_EQ_U32(CURAG_ENVS_INIT,
                     node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQ_U32(CURAG_ENVS_INIT,
                     node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_NVS_INIT,
                                             FAKE_BACKEND_RESOURCE_NVS));

  const node_diagnostic_event_t event = {
      .error = CURAG_ENVS_INIT,
      .flags = 0U,
      .application_offset_ms = 0U,
      .cycle_sample_id = 0U,
      .context = NULL,
  };
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_diagnostic_event(&event, &diag));
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                                             FAKE_BACKEND_RESOURCE_LITTLEFS));

  node_persistence_test_reset_all();
  fake_backend_fail_on(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                       FAKE_BACKEND_RESOURCE_LITTLEFS, 1U, -22);
  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(1U);
  TEST_ASSERT_EQ_U32(CURAG_ELITTLEFS_INIT,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT_EQ_U32(CURAG_ELITTLEFS_INIT,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                                             FAKE_BACKEND_RESOURCE_LITTLEFS));
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQ_U32(0U, sample_id);
  return true;
}

static bool null_output_is_rejected_without_initializing_nvs(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  memset(&diag, 0xa5, sizeof(diag));
  TEST_ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                     node_persistence_claim_sample_id(NULL, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_VALIDATE, NODE_PERSISTENCE_RESOURCE_NVS_SAMPLE_COUNTER,
      NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_NVS_INIT,
                                             FAKE_BACKEND_RESOURCE_NVS));

  memset(&diag, 0xa5, sizeof(diag));
  TEST_ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                     node_persistence_claim_message_id(NULL, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_VALIDATE, NODE_PERSISTENCE_RESOURCE_NVS_MESSAGE_COUNTER,
      NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_NVS_INIT,
                                             FAKE_BACKEND_RESOURCE_NVS));
  return true;
}

static const node_persistence_test_case_t CASES[] = {
    {"sample_ids_are_committed_across_restarts",
     sample_ids_are_committed_across_restarts},
    {"repeated_claims_initialize_once_per_wake",
     repeated_claims_initialize_once_per_wake},
    {"message_ids_are_committed_across_restarts",
     message_ids_are_committed_across_restarts},
    {"sample_and_message_claims_are_independent",
     sample_and_message_claims_are_independent},
    {"exhaustion_is_stable_and_returns_no_id",
     exhaustion_is_stable_and_returns_no_id},
    {"commit_failure_returns_no_id", commit_failure_returns_no_id},
    {"message_exhaustion_is_stable_and_returns_no_id",
     message_exhaustion_is_stable_and_returns_no_id},
    {"message_commit_failure_returns_no_id",
     message_commit_failure_returns_no_id},
    {"ambiguous_message_commit_may_skip_but_never_reuses",
     ambiguous_message_commit_may_skip_but_never_reuses},
    {"backend_initialization_is_independent_and_failure_is_cached",
     backend_initialization_is_independent_and_failure_is_cached},
    {"null_output_is_rejected_without_initializing_nvs",
     null_output_is_rejected_without_initializing_nvs},
};

const node_persistence_test_group_t NODE_PERSISTENCE_NVS_TEST_GROUP = {
    .name = "nvs",
    .cases = CASES,
    .count = sizeof(CASES) / sizeof(CASES[0]),
};

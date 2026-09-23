#include "node_persistence_test.h"

#include <string.h>

static void
make_binding_frame(uint32_t message_id,
                   cura_lora_v2_authenticated_reading_frame_t *output) {
  memset(output->bytes, 0xa5, sizeof(output->bytes));
  output->bytes[CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET] =
      CURA_LORA_V2_CONTROL;
  output->bytes[CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET] =
      CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK;
  node_persistence_store_le32(
      output->bytes + CURA_LORA_V2_CLEAR_HEADER_MESSAGE_ID_OFFSET, message_id);
}

static bool pending_compaction_retains_newest_half_across_restart(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  for (uint32_t id = 0U; id < 6U; ++id) {
    const cura_lora_v2_reading_t reading =
        node_persistence_test_make_reading((uint16_t)id);
    TEST_ASSERT_EQ_U32(
        CURAG_OK, node_persistence_append_pending_reading(&reading, &diag));
  }
  TEST_ASSERT_EQ_SIZE(138U, fake_backend_file_size(TEST_PENDING_PATH));
  TEST_ASSERT(!fake_backend_file_exists(TEST_COMPACT_PATH));
  node_persistence_test_restart();

  uint32_t ids[3];
  size_t count = 0U;
  TEST_ASSERT(node_persistence_test_pending_ids(ids, 3U, &count));
  TEST_ASSERT_EQ_SIZE(3U, count);
  TEST_ASSERT_EQ_U32(3U, ids[0]);
  TEST_ASSERT_EQ_U32(4U, ids[1]);
  TEST_ASSERT_EQ_U32(5U, ids[2]);

  for (uint32_t expected = 6U; expected > 3U; --expected) {
    node_pending_reading_t pending;
    bool found = false;
    TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                     &pending, &found, &diag));
    TEST_ASSERT(found);
    TEST_ASSERT_EQ_U32(expected - 1U, pending.reading.sample_id);
    TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_remove_newest_reading(
                                     pending.reading.sample_id, &diag));
  }
  return true;
}

static bool interrupted_compact_is_never_promoted(void) {
  const uint8_t compact_bytes[] = {0x11U, 0x22U, 0x33U, 0x44U};
  cura_lora_v2_reading_t reading = node_persistence_test_make_reading(1U);
  diagn_context_t diag;
  node_pending_reading_t pending;
  bool found = false;

  node_persistence_test_reset_all();
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  TEST_ASSERT(fake_backend_write_file(TEST_COMPACT_PATH, compact_bytes,
                                      sizeof(compact_bytes)));
  node_persistence_test_restart();
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                   &pending, &found, &diag));
  TEST_ASSERT(found && pending.reading.sample_id == 1U);
  TEST_ASSERT(!fake_backend_file_exists(TEST_COMPACT_PATH));

  node_persistence_test_reset_all();
  TEST_ASSERT(fake_backend_write_file(TEST_COMPACT_PATH, compact_bytes,
                                      sizeof(compact_bytes)));
  TEST_ASSERT_EQ_U32(
      CURAG_ECORRUPT_RECORD,
      node_persistence_peek_most_recent_pending(&pending, &found, &diag));
  TEST_ASSERT(!fake_backend_file_exists(TEST_PENDING_PATH));
  TEST_ASSERT(fake_backend_file_exists(TEST_COMPACT_PATH));

  node_persistence_test_reset_all();
  uint8_t garbage[600];
  memset(garbage, 0xa5, sizeof(garbage));
  TEST_ASSERT(
      fake_backend_write_file(TEST_PENDING_PATH, garbage, sizeof(garbage)));
  TEST_ASSERT(fake_backend_write_file(TEST_COMPACT_PATH, compact_bytes,
                                      sizeof(compact_bytes)));
  TEST_ASSERT_EQ_U32(
      CURAG_ECORRUPT_RECORD,
      node_persistence_peek_most_recent_pending(&pending, &found, &diag));
  TEST_ASSERT_EQ_SIZE(sizeof(garbage),
                      fake_backend_file_size(TEST_PENDING_PATH));
  TEST_ASSERT(fake_backend_file_exists(TEST_COMPACT_PATH));
  return true;
}

static bool pending_compaction_never_splits_a_bound_item(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  for (uint32_t sample_id = 1U; sample_id <= 2U; ++sample_id) {
    const cura_lora_v2_reading_t reading =
        node_persistence_test_make_reading((uint16_t)sample_id);
    const uint32_t message_id = UINT32_C(100) + sample_id;
    cura_lora_v2_authenticated_reading_frame_t frame;
    make_binding_frame(message_id, &frame);
    TEST_ASSERT_EQ_U32(
        CURAG_OK, node_persistence_append_pending_reading(&reading, &diag));
    TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_bind_newest_backlog_frame(
                                     sample_id, message_id, &frame, &diag));
  }
  const cura_lora_v2_reading_t newest = node_persistence_test_make_reading(3U);
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&newest, &diag));
  node_persistence_test_restart();

  node_pending_reading_t pending;
  bool found = false;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                   &pending, &found, &diag));
  TEST_ASSERT(found);
  TEST_ASSERT(!pending.backlog_bound);
  TEST_ASSERT_EQ_U32(3U, pending.reading.sample_id);
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_remove_newest_reading(3U, &diag));

  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                   &pending, &found, &diag));
  TEST_ASSERT(found);
  TEST_ASSERT(pending.backlog_bound);
  TEST_ASSERT_EQ_U32(2U, pending.reading.sample_id);
  TEST_ASSERT_EQ_U32(102U, pending.message_id);
  cura_lora_v2_authenticated_reading_frame_t expected;
  make_binding_frame(102U, &expected);
  TEST_ASSERT(
      memcmp(expected.bytes, pending.frame.bytes, sizeof(expected.bytes)) == 0);
  return true;
}

static bool full_nonpending_logs_reject_without_modifying_existing_bytes(void) {
  diagn_context_t diag;
  cura_lora_v2_reading_t reading = node_persistence_test_make_reading(1U);

  node_persistence_test_reset_all();
  for (uint32_t id = 0U; id < 11U; ++id) {
    reading.sample_id = id;
    TEST_ASSERT_EQ_U32(CURAG_OK,
                       node_persistence_quarantine_reading(&reading, &diag));
  }
  node_persistence_test_snapshot_t before;
  node_persistence_test_snapshot_t after;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_QUARANTINE_PATH, &before));
  reading.sample_id = 11U;
  TEST_ASSERT_EQ_U32(CURAG_ELOG_FULL,
                     node_persistence_quarantine_reading(&reading, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_APPEND, NODE_PERSISTENCE_RESOURCE_QUARANTINE_LOG,
      NODE_PERSISTENCE_STAGE_QUOTA_CHECK, NODE_PERSISTENCE_BACKEND_NO_ERROR,
      0));
  TEST_ASSERT(node_persistence_test_snapshot(TEST_QUARANTINE_PATH, &after));
  TEST_ASSERT(node_persistence_test_snapshots_equal(&before, &after));

  node_persistence_test_reset_all();
  const node_diagnostic_event_t diagnostic = {.error = CURAG_EIO};
  for (size_t index = 0U; index < 14U; ++index) {
    TEST_ASSERT_EQ_U32(
        CURAG_OK, node_persistence_append_diagnostic_event(&diagnostic, &diag));
  }
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DIAGNOSTIC_PATH, &before));
  TEST_ASSERT_EQ_U32(CURAG_ELOG_FULL, node_persistence_append_diagnostic_event(
                                          &diagnostic, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_APPEND, NODE_PERSISTENCE_RESOURCE_DIAGNOSTIC_LOG,
      NODE_PERSISTENCE_STAGE_QUOTA_CHECK, NODE_PERSISTENCE_BACKEND_NO_ERROR,
      0));
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DIAGNOSTIC_PATH, &after));
  TEST_ASSERT(node_persistence_test_snapshots_equal(&before, &after));

  node_persistence_test_reset_all();
  const node_delivery_event_t delivery = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .domain = CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
  };
  for (size_t index = 0U; index < 16U; ++index) {
    TEST_ASSERT_EQ_U32(
        CURAG_OK, node_persistence_append_delivery_event(&delivery, &diag));
  }
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DELIVERY_PATH, &before));
  TEST_ASSERT_EQ_U32(CURAG_ELOG_FULL,
                     node_persistence_append_delivery_event(&delivery, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_APPEND, NODE_PERSISTENCE_RESOURCE_DELIVERY_LOG,
      NODE_PERSISTENCE_STAGE_QUOTA_CHECK, NODE_PERSISTENCE_BACKEND_NO_ERROR,
      0));
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DELIVERY_PATH, &after));
  TEST_ASSERT(node_persistence_test_snapshots_equal(&before, &after));
  return true;
}

static bool delivery_log_preserves_matched_and_unmatched_episodes(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  node_delivery_event_t event = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = 1U,
      .sample_id = 9U,
      .message_id = 19U,
      .domain = CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK,
      .detail.started = {.start_offset_ms = 10U},
  };
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&event, &diag));
  event.type = NODE_DELIVERY_EVENT_FINISHED;
  event.detail.finished.attempt_count = 2U;
  event.detail.finished.final_result = NODE_DELIVERY_RESULT_ACCEPTED;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&event, &diag));

  event.type = NODE_DELIVERY_EVENT_STARTED;
  event.cycle_sample_id = 2U;
  event.message_id = 20U;
  event.detail.started.start_offset_ms = 20U;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&event, &diag));

  event.type = NODE_DELIVERY_EVENT_FINISHED;
  event.cycle_sample_id = 3U;
  event.sample_id = 10U;
  event.message_id = 21U;
  event.detail.finished.attempt_count = 1U;
  event.detail.finished.final_result = NODE_DELIVERY_RESULT_MALFORMED;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&event, &diag));
  node_persistence_test_restart();

  node_persistence_test_snapshot_t snapshot;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DELIVERY_PATH, &snapshot));
  const size_t lengths[] = {31U, 29U, 31U, 29U};
  const uint8_t types[] = {
      NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_STARTED,
      NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_FINISHED,
      NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_STARTED,
      NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_FINISHED,
  };
  const uint32_t cycles[] = {1U, 1U, 2U, 3U};
  const uint32_t samples[] = {9U, 9U, 9U, 10U};
  const uint32_t messages[] = {19U, 19U, 20U, 21U};
  size_t offset = 0U;
  for (size_t index = 0U; index < 4U; ++index) {
    TEST_ASSERT_EQ_U32(types[index], snapshot.bytes[offset + 5U]);
    TEST_ASSERT_EQ_U32(cycles[index], node_persistence_load_le32(
                                          snapshot.bytes + offset + 8U));
    TEST_ASSERT_EQ_U32(samples[index], node_persistence_load_le32(
                                           snapshot.bytes + offset + 12U));
    TEST_ASSERT_EQ_U32(messages[index], node_persistence_load_le32(
                                            snapshot.bytes + offset + 16U));
    TEST_ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_VALID,
                       node_persistence_record_validate(
                           node_persistence_backend(),
                           NODE_PERSISTENCE_LOG_DELIVERY,
                           snapshot.bytes + offset, lengths[index]));
    offset += lengths[index];
  }
  TEST_ASSERT_EQ_SIZE(snapshot.length, offset);
  return true;
}

static bool deterministic_churn_matches_reference_model(void) {
  node_persistence_test_reset_all();
  uint32_t model[4];
  size_t model_count = 0U;
  uint32_t next_id = 0U;
  uint32_t random_state = UINT32_C(0x6d2b79f5);
  diagn_context_t diag;

  for (size_t step = 0U; step < 500U; ++step) {
    random_state = random_state * UINT32_C(1664525) + UINT32_C(1013904223);
    const bool append =
        model_count == 0U || (model_count < sizeof(model) / sizeof(model[0]) &&
                              (random_state & UINT32_C(3)) != 0U);
    if (append) {
      const cura_lora_v2_reading_t reading =
          node_persistence_test_make_reading((uint16_t)next_id);
      TEST_ASSERT_EQ_U32(
          CURAG_OK, node_persistence_append_pending_reading(&reading, &diag));
      model[model_count++] = next_id++;
    } else {
      node_pending_reading_t pending;
      bool found = false;
      TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                       &pending, &found, &diag));
      TEST_ASSERT(found);
      TEST_ASSERT_EQ_U32(model[model_count - 1U], pending.reading.sample_id);
      TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_remove_newest_reading(
                                       pending.reading.sample_id, &diag));
      --model_count;
    }

    if (step % 7U == 0U) {
      node_persistence_test_restart();
      uint32_t persisted[4];
      size_t persisted_count = 0U;
      TEST_ASSERT(node_persistence_test_pending_ids(
          persisted, sizeof(persisted) / sizeof(persisted[0]),
          &persisted_count));
      TEST_ASSERT_EQ_SIZE(model_count, persisted_count);
      TEST_ASSERT(memcmp(model, persisted, model_count * sizeof(model[0])) ==
                  0);
    }
  }
  return true;
}

static const node_persistence_test_case_t CASES[] = {
    {"pending_compaction_retains_newest_half_across_restart",
     pending_compaction_retains_newest_half_across_restart},
    {"interrupted_compact_is_never_promoted",
     interrupted_compact_is_never_promoted},
    {"pending_compaction_never_splits_a_bound_item",
     pending_compaction_never_splits_a_bound_item},
    {"full_nonpending_logs_reject_without_modifying_existing_bytes",
     full_nonpending_logs_reject_without_modifying_existing_bytes},
    {"delivery_log_preserves_matched_and_unmatched_episodes",
     delivery_log_preserves_matched_and_unmatched_episodes},
    {"deterministic_churn_matches_reference_model",
     deterministic_churn_matches_reference_model},
};

const node_persistence_test_group_t NODE_PERSISTENCE_RETENTION_TEST_GROUP = {
    .name = "retention",
    .cases = CASES,
    .count = sizeof(CASES) / sizeof(CASES[0]),
};

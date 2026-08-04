#include "node_persistence_test.h"

#include <string.h>

static bool validate_records(const node_persistence_test_snapshot_t *snapshot,
                             node_persistence_log_kind_t log_kind,
                             size_t expected_count) {
  size_t offset = 0U;
  size_t count = 0U;
  while (offset < snapshot->length) {
    if (snapshot->length - offset < NODE_PERSISTENCE_RECORD_OVERHEAD) {
      return false;
    }
    const size_t payload_length =
        node_persistence_load_le16(snapshot->bytes + offset + 6U);
    const size_t record_length =
        payload_length + NODE_PERSISTENCE_RECORD_OVERHEAD;
    if (record_length > snapshot->length - offset ||
        node_persistence_record_validate(
            node_persistence_backend(), log_kind, snapshot->bytes + offset,
            record_length) != NODE_PERSISTENCE_RECORD_VALID) {
      return false;
    }
    offset += record_length;
    ++count;
  }
  return count == expected_count;
}

static bool pending_round_trip_is_newest_first_and_survives_restart(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  cura_lora_v2_reading_t expected[3];
  for (uint32_t id = 0U; id < 3U; ++id) {
    expected[id] = node_persistence_test_make_reading((uint16_t)(10U + id));
    TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_append_pending_reading(
                                     id, &expected[id], &diag));
  }

  for (uint32_t id = 3U; id > 0U; --id) {
    node_persistence_test_restart();
    uint32_t actual_id = UINT32_MAX;
    cura_lora_v2_reading_t actual;
    bool found = false;
    TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                     &actual_id, &actual, &found, &diag));
    TEST_ASSERT(found);
    TEST_ASSERT_EQ_U32(id - 1U, actual_id);
    TEST_ASSERT(
        node_persistence_test_readings_equal(&expected[id - 1U], &actual));
    TEST_ASSERT_EQ_U32(
        CURAG_OK, node_persistence_remove_newest_reading(actual_id, &diag));
  }

  node_persistence_test_restart();
  uint32_t unused_id = 0U;
  cura_lora_v2_reading_t unused_reading;
  bool found = true;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                   &unused_id, &unused_reading, &found, &diag));
  TEST_ASSERT(!found);
  return true;
}

static bool removal_requires_exact_id_and_empty_is_not_an_error(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  TEST_ASSERT_EQ_U32(CURAG_ERECORD_MISMATCH,
                     node_persistence_remove_newest_reading(1U, &diag));

  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(4U);
  TEST_ASSERT_EQ_U32(
      CURAG_OK, node_persistence_append_pending_reading(4U, &reading, &diag));
  node_persistence_test_snapshot_t before;
  node_persistence_test_snapshot_t after;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_PENDING_PATH, &before));
  TEST_ASSERT_EQ_U32(CURAG_ERECORD_MISMATCH,
                     node_persistence_remove_newest_reading(3U, &diag));
  TEST_ASSERT(node_persistence_test_snapshot(TEST_PENDING_PATH, &after));
  TEST_ASSERT(node_persistence_test_snapshots_equal(&before, &after));
  return true;
}

static bool every_record_type_round_trips_boundary_values(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  const cura_lora_v2_reading_t reading =
      node_persistence_test_make_boundary_reading();
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_append_pending_reading(
                                   UINT32_MAX, &reading, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_quarantine_reading(
                                   UINT32_MAX, &reading, &diag));

  diagn_context_t context;
  curag_diagnostic_context_clear(&context);
  context.operation = CURAG_OP_SLEEP;
  context.context_length = CURAG_DIAGNOSTIC_CONTEXT_MAX;
  context.context_schema = UINT8_MAX;
  memset(context.context, 0xa5, sizeof(context.context));
  const node_diagnostic_event_t diagnostic = {
      .error = UINT32_MAX,
      .flags = NODE_DIAGNOSTIC_APPLICATION_OFFSET_VALID |
               NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID,
      .application_offset_ms = UINT32_MAX,
      .cycle_sample_id = UINT32_MAX,
      .context = &context,
  };
  TEST_ASSERT_EQ_U32(
      CURAG_OK, node_persistence_append_diagnostic_event(&diagnostic, &diag));

  node_delivery_event_t delivery = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = UINT32_MAX,
      .sample_id = UINT32_MAX,
      .domain = CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
      .detail.started = {.start_offset_ms = UINT32_MAX},
  };
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&delivery, &diag));
  delivery.type = NODE_DELIVERY_EVENT_FINISHED;
  delivery.domain = CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK;
  delivery.detail.finished.attempt_count = UINT8_MAX;
  delivery.detail.finished.final_result =
      NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&delivery, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));

  node_persistence_test_snapshot_t snapshot;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_PENDING_PATH, &snapshot));
  TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_PENDING, 1U));
  TEST_ASSERT_EQ_U32(UINT32_MAX,
                     node_persistence_load_le32(snapshot.bytes + 8U));
  uint32_t decoded_id = 0U;
  uint8_t body[CURA_LORA_V2_READING_BODY_SIZE];
  cura_lora_v2_reading_t decoded;
  TEST_ASSERT(node_persistence_record_decode_reading(
      snapshot.bytes, snapshot.length, &decoded_id, body));
  TEST_ASSERT_EQ_U32(UINT32_MAX, decoded_id);
  TEST_ASSERT_EQ_U32(CURA_LORA_V2_CODEC_OK,
                     cura_lora_v2_decode_reading(&decoded, body, sizeof(body)));
  TEST_ASSERT(node_persistence_test_readings_equal(&reading, &decoded));

  TEST_ASSERT(node_persistence_test_snapshot(TEST_QUARANTINE_PATH, &snapshot));
  TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_QUARANTINE, 1U));
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DIAGNOSTIC_PATH, &snapshot));
  TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_DIAGNOSTIC, 1U));
  TEST_ASSERT_EQ_SIZE(284U, snapshot.length);
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DELIVERY_PATH, &snapshot));
  TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_DELIVERY, 2U));

  uint8_t payload[NODE_PERSISTENCE_RECORD_MAX_PAYLOAD_SIZE];
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  memset(payload, 0x5a, sizeof(payload));
  size_t record_length = 0U;
  TEST_ASSERT(node_persistence_record_encode(node_persistence_backend(),
                                             UINT8_C(0x77), NULL, 0U, record,
                                             &record_length));
  TEST_ASSERT_EQ_SIZE(NODE_PERSISTENCE_RECORD_OVERHEAD, record_length);
  TEST_ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_VALID,
                     node_persistence_record_validate_structural(
                         node_persistence_backend(), record, record_length));
  TEST_ASSERT(node_persistence_record_encode(
      node_persistence_backend(), UINT8_C(0x77), payload, sizeof(payload),
      record, &record_length));
  TEST_ASSERT_EQ_SIZE(NODE_PERSISTENCE_RECORD_MAX_SIZE, record_length);
  TEST_ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_VALID,
                     node_persistence_record_validate_structural(
                         node_persistence_backend(), record, record_length));
  TEST_ASSERT(!node_persistence_record_encode(
      node_persistence_backend(), UINT8_C(0x77), payload, sizeof(payload) + 1U,
      record, &record_length));
  return true;
}

static bool delivery_is_synced_before_each_successful_return(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  node_delivery_event_t event = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = 8U,
      .sample_id = 7U,
      .domain = CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
      .detail.started = {.start_offset_ms = 6U},
  };
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&event, &diag));
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_FILE_SYNC,
                                             FAKE_BACKEND_RESOURCE_DELIVERY));
  node_persistence_test_restart();

  event.type = NODE_DELIVERY_EVENT_FINISHED;
  event.detail.finished.attempt_count = 3U;
  event.detail.finished.final_result = NODE_DELIVERY_RESULT_ACCEPTED;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&event, &diag));
  TEST_ASSERT_EQ_SIZE(2U, fake_backend_count(FAKE_BACKEND_OP_FILE_SYNC,
                                             FAKE_BACKEND_RESOURCE_DELIVERY));
  node_persistence_test_restart();

  node_persistence_test_snapshot_t snapshot;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DELIVERY_PATH, &snapshot));
  TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_DELIVERY, 2U));
  return true;
}

static bool
diagnostics_buffer_until_sync_and_sync_closes_without_unmount(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  const node_diagnostic_event_t event = {
      .error = CURAG_EIO,
      .flags = 0U,
      .application_offset_ms = 0U,
      .cycle_sample_id = 0U,
      .context = NULL,
  };
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_diagnostic_event(&event, &diag));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_FILE_SYNC,
                                             FAKE_BACKEND_RESOURCE_DIAGNOSTIC));
  TEST_ASSERT(fake_backend_littlefs_is_mounted());
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_FILE_SYNC,
                                             FAKE_BACKEND_RESOURCE_DIAGNOSTIC));
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_FILE_CLOSE,
                                             FAKE_BACKEND_RESOURCE_DIAGNOSTIC));
  TEST_ASSERT(fake_backend_littlefs_is_mounted());

  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_FILE_SYNC,
                                             FAKE_BACKEND_RESOURCE_DIAGNOSTIC));
  return true;
}

static bool invalid_inputs_and_optional_diagnostics_are_consistent(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  cura_lora_v2_reading_t invalid = {0};
  invalid.soil_0_mv = 1U;
  TEST_ASSERT_EQ_U32(
      CURAG_EINVALID_ARGUMENT,
      node_persistence_append_pending_reading(0U, &invalid, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_VALIDATE, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));
  TEST_ASSERT_EQ_U32(
      CURAG_EINVALID_ARGUMENT,
      node_persistence_append_pending_reading(0U, &invalid, NULL));

  uint32_t id = 0U;
  cura_lora_v2_reading_t reading;
  bool found = false;
  TEST_ASSERT_EQ_U32(
      CURAG_EINVALID_ARGUMENT,
      node_persistence_peek_most_recent_pending(NULL, &reading, &found, &diag));
  TEST_ASSERT_EQ_U32(
      CURAG_EINVALID_ARGUMENT,
      node_persistence_peek_most_recent_pending(&id, NULL, &found, &diag));
  TEST_ASSERT_EQ_U32(
      CURAG_EINVALID_ARGUMENT,
      node_persistence_peek_most_recent_pending(&id, &reading, NULL, &diag));

  node_diagnostic_event_t diagnostic = {
      .error = CURAG_EIO,
      .flags = NODE_DIAGNOSTIC_RESERVED_FLAGS_MASK,
  };
  TEST_ASSERT_EQ_U32(
      CURAG_EINVALID_ARGUMENT,
      node_persistence_append_diagnostic_event(&diagnostic, &diag));
  node_delivery_event_t delivery = {
      .type = NODE_DELIVERY_EVENT_FINISHED,
      .domain = CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      .detail.finished =
          {
              .attempt_count = 1U,
              .final_result = NODE_DELIVERY_RESULT_ACCEPTED,
          },
  };
  TEST_ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                     node_persistence_append_delivery_event(&delivery, &diag));

  const cura_lora_v2_reading_t valid = node_persistence_test_make_reading(2U);
  memset(&diag, 0xa5, sizeof(diag));
  TEST_ASSERT_EQ_U32(
      CURAG_OK, node_persistence_append_pending_reading(2U, &valid, &diag));
  const diagn_context_t cleared = {0};
  TEST_ASSERT(memcmp(&diag, &cleared, sizeof(diag)) == 0);
  return true;
}

static const node_persistence_test_case_t CASES[] = {
    {"pending_round_trip_is_newest_first_and_survives_restart",
     pending_round_trip_is_newest_first_and_survives_restart},
    {"removal_requires_exact_id_and_empty_is_not_an_error",
     removal_requires_exact_id_and_empty_is_not_an_error},
    {"every_record_type_round_trips_boundary_values",
     every_record_type_round_trips_boundary_values},
    {"delivery_is_synced_before_each_successful_return",
     delivery_is_synced_before_each_successful_return},
    {"diagnostics_buffer_until_sync_and_sync_closes_without_unmount",
     diagnostics_buffer_until_sync_and_sync_closes_without_unmount},
    {"invalid_inputs_and_optional_diagnostics_are_consistent",
     invalid_inputs_and_optional_diagnostics_are_consistent},
};

const node_persistence_test_group_t NODE_PERSISTENCE_RECORD_TEST_GROUP = {
    .name = "records",
    .cases = CASES,
    .count = sizeof(CASES) / sizeof(CASES[0]),
};

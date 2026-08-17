#include "node_persistence_test.h"

#include <errno.h>
#include <string.h>

static bool make_records(uint8_t first[NODE_PERSISTENCE_RECORD_MAX_SIZE],
                         size_t *first_length,
                         uint8_t second[NODE_PERSISTENCE_RECORD_MAX_SIZE],
                         size_t *second_length) {
  const cura_lora_v2_reading_t first_reading =
      node_persistence_test_make_reading(1U);
  const cura_lora_v2_reading_t second_reading =
      node_persistence_test_make_reading(2U);
  return node_persistence_test_encode_reading_record(
             NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING, 1U, &first_reading,
             first, first_length) &&
         node_persistence_test_encode_reading_record(
             NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING, 2U, &second_reading,
             second, second_length);
}

static bool assert_peek_id(uint32_t expected_id) {
  diagn_context_t diag;
  node_pending_reading_t pending;
  bool found = false;
  return node_persistence_peek_most_recent_pending(&pending, &found, &diag) ==
             CURAG_OK &&
         found && pending.reading.sample_id == expected_id;
}

static bool every_truncated_final_record_boundary_is_recovered(void) {
  uint8_t first[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  uint8_t second[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t first_length = 0U;
  size_t second_length = 0U;
  TEST_ASSERT(make_records(first, &first_length, second, &second_length));

  for (size_t prefix = 1U; prefix < second_length; ++prefix) {
    node_persistence_test_reset_all();
    TEST_ASSERT(
        fake_backend_write_file(TEST_PENDING_PATH, first, first_length));
    TEST_ASSERT(fake_backend_append_bytes(TEST_PENDING_PATH, second, prefix));
    fake_backend_clear_trace();

    diagn_context_t diag;
    node_pending_reading_t pending;
    bool found = false;
    TEST_ASSERT_EQ_U32(
        CURAG_ECORRUPT_RECORD,
        node_persistence_peek_most_recent_pending(&pending, &found, &diag));
    TEST_ASSERT_EQ_SIZE(first_length,
                        fake_backend_file_size(TEST_PENDING_PATH));
    TEST_ASSERT(node_persistence_test_assert_diag(
        &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
        NODE_PERSISTENCE_STAGE_TRUNCATE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));
    TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_FILE_TRUNCATE,
                                               FAKE_BACKEND_RESOURCE_PENDING));
    TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_FILE_SYNC,
                                               FAKE_BACKEND_RESOURCE_PENDING));
    TEST_ASSERT(assert_peek_id(1U));
  }
  return true;
}

static bool assert_complete_bad_tail_recovery(const uint8_t *bad_record,
                                              size_t bad_length,
                                              err_curag_t expected_error) {
  uint8_t first[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  uint8_t ignored[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t first_length = 0U;
  size_t ignored_length = 0U;
  if (!make_records(first, &first_length, ignored, &ignored_length)) {
    return false;
  }
  node_persistence_test_reset_all();
  if (!fake_backend_write_file(TEST_PENDING_PATH, first, first_length) ||
      !fake_backend_append_bytes(TEST_PENDING_PATH, bad_record, bad_length)) {
    return false;
  }
  fake_backend_clear_trace();

  diagn_context_t diag;
  node_pending_reading_t pending;
  bool found = false;
  if (node_persistence_peek_most_recent_pending(&pending, &found, &diag) !=
          expected_error ||
      fake_backend_file_size(TEST_PENDING_PATH) != first_length ||
      fake_backend_count(FAKE_BACKEND_OP_FILE_SYNC,
                         FAKE_BACKEND_RESOURCE_PENDING) != 1U ||
      !assert_peek_id(1U)) {
    return false;
  }
  return true;
}

static bool crc_semantic_type_and_version_tails_are_recovered(void) {
  uint8_t first[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t first_length = 0U;
  size_t record_length = 0U;
  TEST_ASSERT(make_records(first, &first_length, record, &record_length));
  (void)first;
  (void)first_length;

  record[record_length - 1U] ^= UINT8_C(0x01);
  TEST_ASSERT(assert_complete_bad_tail_recovery(record, record_length,
                                                CURAG_ECORRUPT_RECORD));

  TEST_ASSERT(make_records(first, &first_length, record, &record_length));
  node_persistence_store_le16(record + NODE_PERSISTENCE_RECORD_HEADER_SIZE +
                                  CURA_LORA_V2_READING_FLAGS_OFFSET,
                              0U);
  node_persistence_test_recalculate_crc(record, record_length);
  TEST_ASSERT(assert_complete_bad_tail_recovery(record, record_length,
                                                CURAG_ECORRUPT_RECORD));

  TEST_ASSERT(make_records(first, &first_length, record, &record_length));
  record[5U] = NODE_PERSISTENCE_RECORD_TYPE_QUARANTINED_READING;
  node_persistence_test_recalculate_crc(record, record_length);
  TEST_ASSERT(assert_complete_bad_tail_recovery(record, record_length,
                                                CURAG_EUNSUPPORTED_RECORD));

  TEST_ASSERT(make_records(first, &first_length, record, &record_length));
  record[4U] = UINT8_C(1);
  node_persistence_test_recalculate_crc(record, record_length);
  TEST_ASSERT(assert_complete_bad_tail_recovery(record, record_length,
                                                CURAG_EUNSUPPORTED_RECORD));
  return true;
}

static bool append_repairs_first_and_never_buries_corruption(void) {
  uint8_t first[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  uint8_t bad[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t first_length = 0U;
  size_t bad_length = 0U;
  TEST_ASSERT(make_records(first, &first_length, bad, &bad_length));
  bad[bad_length - 1U] ^= UINT8_C(0x80);

  node_persistence_test_reset_all();
  TEST_ASSERT(fake_backend_write_file(TEST_PENDING_PATH, first, first_length));
  TEST_ASSERT(fake_backend_append_bytes(TEST_PENDING_PATH, bad, bad_length));
  fake_backend_clear_trace();
  diagn_context_t diag;
  const cura_lora_v2_reading_t third = node_persistence_test_make_reading(3U);
  TEST_ASSERT_EQ_U32(CURAG_ECORRUPT_RECORD,
                     node_persistence_append_pending_reading(&third, &diag));
  TEST_ASSERT_EQ_SIZE(first_length, fake_backend_file_size(TEST_PENDING_PATH));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_FILE_WRITE,
                                             FAKE_BACKEND_RESOURCE_PENDING));

  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&third, &diag));
  uint32_t ids[2];
  size_t count = 0U;
  TEST_ASSERT(node_persistence_test_pending_ids(ids, 2U, &count));
  TEST_ASSERT_EQ_SIZE(2U, count);
  TEST_ASSERT_EQ_U32(1U, ids[0]);
  TEST_ASSERT_EQ_U32(3U, ids[1]);
  return true;
}

static bool complete_mismatched_binding_is_removed_before_append(void) {
  node_persistence_test_reset_all();
  const cura_lora_v2_reading_t first = node_persistence_test_make_reading(1U);
  uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE];
  memset(frame, 0xa5, sizeof(frame));
  frame[CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET] = CURA_LORA_V2_CONTROL;
  frame[CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET] =
      CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK;
  node_persistence_store_le32(
      frame + CURA_LORA_V2_CLEAR_HEADER_MESSAGE_ID_OFFSET, 9U);

  diagn_context_t diag;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&first, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_bind_newest_backlog_frame(
                                   first.sample_id, 9U, frame, &diag));

  node_persistence_test_snapshot_t snapshot;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_PENDING_PATH, &snapshot));
  const size_t reading_length =
      NODE_PERSISTENCE_READING_PAYLOAD_SIZE + NODE_PERSISTENCE_RECORD_OVERHEAD;
  const size_t binding_length = NODE_PERSISTENCE_BACKLOG_BINDING_PAYLOAD_SIZE +
                                NODE_PERSISTENCE_RECORD_OVERHEAD;
  TEST_ASSERT_EQ_SIZE(reading_length + binding_length, snapshot.length);
  uint8_t *binding = snapshot.bytes + reading_length;
  node_persistence_store_le32(binding + NODE_PERSISTENCE_RECORD_HEADER_SIZE,
                              first.sample_id + 1U);
  node_persistence_test_recalculate_crc(binding, binding_length);
  TEST_ASSERT(fake_backend_write_file(TEST_PENDING_PATH, snapshot.bytes,
                                      snapshot.length));

  node_persistence_test_restart();
  fake_backend_clear_trace();
  const cura_lora_v2_reading_t second = node_persistence_test_make_reading(2U);
  TEST_ASSERT_EQ_U32(CURAG_ECORRUPT_RECORD,
                     node_persistence_append_pending_reading(&second, &diag));
  TEST_ASSERT_EQ_SIZE(reading_length,
                      fake_backend_file_size(TEST_PENDING_PATH));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_FILE_WRITE,
                                             FAKE_BACKEND_RESOURCE_PENDING));
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&second, &diag));
  uint32_t ids[2];
  size_t count = 0U;
  TEST_ASSERT(node_persistence_test_pending_ids(ids, 2U, &count));
  TEST_ASSERT_EQ_SIZE(2U, count);
  TEST_ASSERT_EQ_U32(1U, ids[0]);
  TEST_ASSERT_EQ_U32(2U, ids[1]);
  return true;
}

static bool unprovable_boundary_is_preserved_repeatedly(void) {
  node_persistence_test_reset_all();
  uint8_t garbage[600];
  for (size_t index = 0U; index < sizeof(garbage); ++index) {
    garbage[index] = (uint8_t)(index * 31U + 7U);
  }
  TEST_ASSERT(
      fake_backend_write_file(TEST_PENDING_PATH, garbage, sizeof(garbage)));
  node_persistence_test_snapshot_t original;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_PENDING_PATH, &original));

  for (size_t attempt = 0U; attempt < 2U; ++attempt) {
    node_persistence_test_restart();
    diagn_context_t diag;
    node_pending_reading_t pending;
    bool found = false;
    TEST_ASSERT_EQ_U32(
        CURAG_ECORRUPT_RECORD,
        node_persistence_peek_most_recent_pending(&pending, &found, &diag));
    node_persistence_test_snapshot_t after;
    TEST_ASSERT(node_persistence_test_snapshot(TEST_PENDING_PATH, &after));
    TEST_ASSERT(node_persistence_test_snapshots_equal(&original, &after));
    TEST_ASSERT(node_persistence_test_assert_diag(
        &diag, CURAG_OP_RECOVER, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
        NODE_PERSISTENCE_STAGE_TAIL_SCAN, NODE_PERSISTENCE_BACKEND_NO_ERROR,
        0));
  }

  const cura_lora_v2_reading_t new_reading =
      node_persistence_test_make_reading(9U);
  diagn_context_t diag;
  fake_backend_clear_trace();
  TEST_ASSERT_EQ_U32(
      CURAG_ECORRUPT_RECORD,
      node_persistence_append_pending_reading(&new_reading, &diag));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_count(FAKE_BACKEND_OP_FILE_WRITE,
                                             FAKE_BACKEND_RESOURCE_PENDING));
  node_persistence_test_snapshot_t after_append;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_PENDING_PATH, &after_append));
  TEST_ASSERT(node_persistence_test_snapshots_equal(&original, &after_append));
  return true;
}

static bool removal_after_recovery_requires_a_second_call(void) {
  uint8_t first[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  uint8_t second[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t first_length = 0U;
  size_t second_length = 0U;
  TEST_ASSERT(make_records(first, &first_length, second, &second_length));
  (void)second_length;
  node_persistence_test_reset_all();
  TEST_ASSERT(fake_backend_write_file(TEST_PENDING_PATH, first, first_length));
  TEST_ASSERT(fake_backend_append_bytes(TEST_PENDING_PATH, second, 3U));

  diagn_context_t diag;
  TEST_ASSERT_EQ_U32(CURAG_ECORRUPT_RECORD,
                     node_persistence_remove_newest_reading(1U, &diag));
  TEST_ASSERT_EQ_SIZE(first_length, fake_backend_file_size(TEST_PENDING_PATH));
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_remove_newest_reading(1U, &diag));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_file_size(TEST_PENDING_PATH));
  return true;
}

static bool partial_application_append_is_recovered_after_restart(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  const cura_lora_v2_reading_t first = node_persistence_test_make_reading(1U);
  const cura_lora_v2_reading_t second = node_persistence_test_make_reading(2U);
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&first, &diag));
  fake_backend_partial_write_on(FAKE_BACKEND_RESOURCE_PENDING, 1U, 3U, EIO);
  TEST_ASSERT_EQ_U32(CURAG_EIO,
                     node_persistence_append_pending_reading(&second, &diag));
  TEST_ASSERT_EQ_SIZE(49U, fake_backend_file_size(TEST_PENDING_PATH));
  const size_t trace_count = fake_backend_trace_count();
  TEST_ASSERT(trace_count > 0U);
  const fake_backend_trace_entry_t *last_write = NULL;
  for (size_t index = trace_count; index > 0U; --index) {
    const fake_backend_trace_entry_t *entry = fake_backend_trace_at(index - 1U);
    if (entry->operation == FAKE_BACKEND_OP_FILE_WRITE &&
        entry->resource == FAKE_BACKEND_RESOURCE_PENDING) {
      last_write = entry;
      break;
    }
  }
  TEST_ASSERT(last_write != NULL);
  TEST_ASSERT_EQ_SIZE(3U, last_write->completed_length);

  node_persistence_test_restart();
  node_pending_reading_t pending;
  bool found = false;
  TEST_ASSERT_EQ_U32(
      CURAG_ECORRUPT_RECORD,
      node_persistence_peek_most_recent_pending(&pending, &found, &diag));
  TEST_ASSERT(assert_peek_id(1U));
  return true;
}

static bool
restart_between_quarantine_and_removal_allows_duplicates_not_loss(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(7U);
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_quarantine_reading(&reading, &diag));
  node_persistence_test_restart();
  TEST_ASSERT(assert_peek_id(7U));
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_quarantine_reading(&reading, &diag));

  node_persistence_test_snapshot_t quarantine;
  TEST_ASSERT(
      node_persistence_test_snapshot(TEST_QUARANTINE_PATH, &quarantine));
  TEST_ASSERT_EQ_SIZE(92U, quarantine.length);
  TEST_ASSERT_EQ_U32(
      NODE_PERSISTENCE_RECORD_VALID,
      node_persistence_record_validate(node_persistence_backend(),
                                       NODE_PERSISTENCE_LOG_QUARANTINE,
                                       quarantine.bytes, 46U));
  TEST_ASSERT_EQ_U32(
      NODE_PERSISTENCE_RECORD_VALID,
      node_persistence_record_validate(node_persistence_backend(),
                                       NODE_PERSISTENCE_LOG_QUARANTINE,
                                       quarantine.bytes + 46U, 46U));

  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_remove_newest_reading(7U, &diag));
  node_persistence_test_restart();
  node_pending_reading_t pending;
  bool found = true;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                   &pending, &found, &diag));
  TEST_ASSERT(!found);
  return true;
}

static const node_persistence_test_case_t CASES[] = {
    {"every_truncated_final_record_boundary_is_recovered",
     every_truncated_final_record_boundary_is_recovered},
    {"crc_semantic_type_and_version_tails_are_recovered",
     crc_semantic_type_and_version_tails_are_recovered},
    {"append_repairs_first_and_never_buries_corruption",
     append_repairs_first_and_never_buries_corruption},
    {"complete_mismatched_binding_is_removed_before_append",
     complete_mismatched_binding_is_removed_before_append},
    {"unprovable_boundary_is_preserved_repeatedly",
     unprovable_boundary_is_preserved_repeatedly},
    {"removal_after_recovery_requires_a_second_call",
     removal_after_recovery_requires_a_second_call},
    {"partial_application_append_is_recovered_after_restart",
     partial_application_append_is_recovered_after_restart},
    {"restart_between_quarantine_and_removal_allows_duplicates_not_loss",
     restart_between_quarantine_and_removal_allows_duplicates_not_loss},
};

const node_persistence_test_group_t NODE_PERSISTENCE_RECOVERY_TEST_GROUP = {
    .name = "recovery",
    .cases = CASES,
    .count = sizeof(CASES) / sizeof(CASES[0]),
};

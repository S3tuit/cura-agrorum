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
    expected[id].sample_id = id;
    TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_append_pending_reading(
                                     &expected[id], &diag));
  }

  for (uint32_t id = 3U; id > 0U; --id) {
    node_persistence_test_restart();
    node_pending_reading_t pending;
    bool found = false;
    TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                     &pending, &found, &diag));
    TEST_ASSERT(found);
    TEST_ASSERT(!pending.backlog_bound);
    TEST_ASSERT_EQ_U32(id - 1U, pending.reading.sample_id);
    TEST_ASSERT(node_persistence_test_readings_equal(&expected[id - 1U],
                                                     &pending.reading));
    TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_remove_newest_reading(
                                     pending.reading.sample_id, &diag));
  }

  node_persistence_test_restart();
  node_pending_reading_t unused;
  bool found = true;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                   &unused, &found, &diag));
  TEST_ASSERT(!found);
  return true;
}

static bool removal_requires_exact_id_and_empty_is_not_an_error(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  TEST_ASSERT_EQ_U32(CURAG_ERECORD_MISMATCH,
                     node_persistence_remove_newest_reading(1U, &diag));

  const cura_lora_v2_reading_t reading = node_persistence_test_make_reading(4U);
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&reading, &diag));
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
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_quarantine_reading(&reading, &diag));

  diagn_context_t context;
  curag_diagnostic_context_clear(&context);
  context.operation = CURAG_OP_SLEEP;
  context.context_length = CURAG_DIAGNOSTIC_CONTEXT_MAX;
  context.context_schema = UINT8_MAX;
  memset(context.context, 0xa5, sizeof(context.context));
  const node_diagnostic_event_t diagnostic = {
      .error = UINT32_MAX,
      .flags = NODE_DIAGNOSTIC_APPLICATION_OFFSET_VALID |
               NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID |
               NODE_DIAGNOSTIC_MESSAGE_ID_VALID,
      .application_offset_ms = UINT32_MAX,
      .cycle_sample_id = UINT32_MAX,
      .message_id = UINT32_MAX,
      .context = &context,
  };
  TEST_ASSERT_EQ_U32(
      CURAG_OK, node_persistence_append_diagnostic_event(&diagnostic, &diag));

  node_delivery_event_t delivery = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = UINT32_MAX,
      .sample_id = UINT32_MAX,
      .message_id = UINT32_MAX,
      .domain = CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
      .detail.started = {.start_offset_ms = UINT32_MAX},
  };
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&delivery, &diag));
  delivery.type = NODE_DELIVERY_EVENT_FINISHED;
  delivery.domain = CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK;
  delivery.detail.finished.attempt_count = 2U;
  delivery.detail.finished.final_result = NODE_DELIVERY_RESULT_ACCEPTED;
  node_persistence_test_fill_tx_calls(&delivery);
  delivery.detail.finished.application_start_us = UINT64_MAX;
  delivery.detail.finished.tx_calls[1].set_tx_at_us = UINT64_MAX;
  delivery.detail.finished.tx_calls[1].tx_done_at_us = UINT64_MAX;
  delivery.detail.finished.tx_calls[1].ack_rx_done_at_us = UINT64_MAX;
  delivery.detail.finished.tx_calls[1].ack_rssi_dbm_x2 = INT16_MIN;
  delivery.detail.finished.tx_calls[1].ack_snr_db_x4 = INT16_MAX;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&delivery, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));

  node_persistence_test_snapshot_t snapshot;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_PENDING_PATH, &snapshot));
  TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_PENDING, 1U));
  TEST_ASSERT_EQ_U32(UINT32_MAX,
                     node_persistence_load_le32(snapshot.bytes + 8U));
  uint8_t body[CURA_LORA_V2_READING_BODY_SIZE];
  cura_lora_v2_reading_t decoded;
  TEST_ASSERT(node_persistence_record_decode_reading(snapshot.bytes,
                                                     snapshot.length, body));
  TEST_ASSERT_EQ_U32(CURA_LORA_V2_CODEC_OK,
                     cura_lora_v2_decode_reading(&decoded, body, sizeof(body)));
  TEST_ASSERT(node_persistence_test_readings_equal(&reading, &decoded));

  TEST_ASSERT(node_persistence_test_snapshot(TEST_QUARANTINE_PATH, &snapshot));
  TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_QUARANTINE, 1U));
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DIAGNOSTIC_PATH, &snapshot));
  TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_DIAGNOSTIC, 1U));
  TEST_ASSERT_EQ_SIZE(288U, snapshot.length);
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

static bool backlog_binding_round_trips_exact_frame_across_restart(void) {
  node_persistence_test_reset_all();
  cura_lora_v2_reading_t reading = node_persistence_test_make_reading(17U);
  const uint32_t message_id = UINT32_C(0x11223344);
  cura_lora_v2_authenticated_reading_frame_t frame;
  memset(frame.bytes, 0xa5, sizeof(frame.bytes));
  frame.bytes[CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET] = CURA_LORA_V2_CONTROL;
  frame.bytes[CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET] =
      CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK;
  node_persistence_store_le32(
      frame.bytes + CURA_LORA_V2_CLEAR_HEADER_MESSAGE_ID_OFFSET, message_id);

  diagn_context_t diag;
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&reading, &diag));
  TEST_ASSERT_EQ_U32(CURAG_ERECORD_MISMATCH,
                     node_persistence_bind_newest_backlog_frame(
                         reading.sample_id + 1U, message_id, &frame, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_bind_newest_backlog_frame(
                         reading.sample_id, message_id, &frame, &diag));
  TEST_ASSERT_EQ_U32(CURAG_ERECORD_MISMATCH,
                     node_persistence_bind_newest_backlog_frame(
                         reading.sample_id, message_id, &frame, &diag));

  node_persistence_test_restart();
  node_pending_reading_t pending;
  bool found = false;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                   &pending, &found, &diag));
  TEST_ASSERT(found);
  TEST_ASSERT(pending.backlog_bound);
  TEST_ASSERT_EQ_U32(reading.sample_id, pending.reading.sample_id);
  TEST_ASSERT_EQ_U32(message_id, pending.message_id);
  TEST_ASSERT(memcmp(frame.bytes, pending.frame.bytes, sizeof(frame.bytes)) ==
              0);
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_remove_newest_reading(
                                   reading.sample_id, &diag));
  TEST_ASSERT_EQ_SIZE(0U, fake_backend_file_size(TEST_PENDING_PATH));
  return true;
}

static bool delivery_outcomes_survive_recovery_and_reject_unknown_values(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  node_delivery_event_t event = {
      .type = NODE_DELIVERY_EVENT_FINISHED,
      .cycle_sample_id = 8U,
      .sample_id = 7U,
      .message_id = 9U,
      .domain = CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
      .detail.finished = {.attempt_count = 2U},
  };
  for (uint8_t result = 1U; result <= 8U; ++result) {
    event.detail.finished.final_result = result;
    node_persistence_test_fill_tx_calls(&event);
    TEST_ASSERT_EQ_U32(CURAG_OK,
                       node_persistence_append_delivery_event(&event, &diag));
  }
  node_persistence_test_snapshot_t before;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DELIVERY_PATH, &before));
  TEST_ASSERT(validate_records(&before, NODE_PERSISTENCE_LOG_DELIVERY, 8U));
  const size_t record_length = 98U;
  TEST_ASSERT_EQ_SIZE(8U * record_length, before.length);
  TEST_ASSERT_EQ_U32(8U, before.bytes[7U * record_length + 22U]);
  node_persistence_test_restart();
  /* Appending after restart forces recovery to validate the new terminal tail.
   */
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&event, &diag));
  node_persistence_test_snapshot_t after;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DELIVERY_PATH, &after));
  TEST_ASSERT(validate_records(&after, NODE_PERSISTENCE_LOG_DELIVERY, 9U));
  TEST_ASSERT(memcmp(before.bytes, after.bytes, before.length) == 0);

  const uint8_t invalid[] = {0U, 9U, UINT8_MAX};
  for (size_t index = 0U; index < sizeof(invalid); ++index) {
    event.detail.finished.final_result = invalid[index];
    TEST_ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                       node_persistence_append_delivery_event(&event, &diag));
    uint8_t record[98];
    memcpy(record, before.bytes + 7U * record_length, sizeof(record));
    record[22U] = invalid[index];
    node_persistence_test_recalculate_crc(record, sizeof(record));
    TEST_ASSERT_EQ_U32(
        NODE_PERSISTENCE_RECORD_INVALID_PAYLOAD,
        node_persistence_record_validate(node_persistence_backend(),
                                         NODE_PERSISTENCE_LOG_DELIVERY, record,
                                         sizeof(record)));
  }
  node_persistence_test_snapshot_t unchanged;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DELIVERY_PATH, &unchanged));
  TEST_ASSERT(node_persistence_test_snapshots_equal(&after, &unchanged));
  return true;
}

static bool delivery_is_synced_before_each_successful_return(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  node_delivery_event_t event = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = 8U,
      .sample_id = 7U,
      .message_id = 9U,
      .domain = CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
      .detail.started = {.start_offset_ms = 6U},
  };
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&event, &diag));
  TEST_ASSERT_EQ_SIZE(1U, fake_backend_count(FAKE_BACKEND_OP_FILE_SYNC,
                                             FAKE_BACKEND_RESOURCE_DELIVERY));
  node_persistence_test_restart();

  event.type = NODE_DELIVERY_EVENT_FINISHED;
  event.detail.finished.attempt_count = 2U;
  event.detail.finished.final_result = NODE_DELIVERY_RESULT_ACCEPTED;
  node_persistence_test_fill_tx_calls(&event);
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

static node_delivery_event_t finished_with_calls(uint8_t attempts,
                                                 uint8_t final_result) {
  node_delivery_event_t event = {
      .type = NODE_DELIVERY_EVENT_FINISHED,
      .cycle_sample_id = 10U,
      .sample_id = 9U,
      .message_id = 1002U,
      .domain = CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK,
      .detail.finished = {.attempt_count = attempts,
                          .final_result = final_result},
  };
  node_persistence_test_fill_tx_calls(&event);
  return event;
}

static node_persistence_record_result_t
validate_mutated(const uint8_t *valid, size_t length, size_t payload_offset,
                 uint8_t value) {
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  memcpy(record, valid, length);
  record[NODE_PERSISTENCE_RECORD_HEADER_SIZE + payload_offset] = value;
  node_persistence_test_recalculate_crc(record, length);
  return node_persistence_record_validate(node_persistence_backend(),
                                          NODE_PERSISTENCE_LOG_DELIVERY,
                                          record, length);
}

#define SLOT(index, byte)                                                      \
  (NODE_PERSISTENCE_DELIVERY_TX_CALL_OFFSET +                                  \
   (index) * NODE_PERSISTENCE_DELIVERY_TX_CALL_SIZE + (byte))

/* Only canonical, internally consistent call histories are valid. */
static bool delivery_tx_calls_must_be_canonical(void) {
  node_persistence_test_reset_all();
  uint8_t two[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t two_length = 0U;
  const node_delivery_event_t accepted =
      finished_with_calls(2U, NODE_DELIVERY_RESULT_ACCEPTED);
  TEST_ASSERT(node_persistence_record_encode_delivery(
      node_persistence_backend(), &accepted, two, &two_length));
  TEST_ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_VALID,
                     validate_mutated(two, two_length, 0U, two[8U]));
  const struct {
    size_t offset;
    uint8_t value;
  } invalid_two[] = {
      {SLOT(0U, 0U), UINT8_C(0x07)}, /* reserved flag bit */
      {SLOT(0U, 0U), UINT8_C(0x02)}, /* TX_DONE without a start */
      {SLOT(0U, 0U), UINT8_C(0x01)}, /* stale completion time */
      {SLOT(0U, 18U), UINT8_C(1)},   /* ACK time on a timeout */
      {SLOT(0U, 28U), UINT8_C(1)},   /* ACK SNR on a timeout */
      {SLOT(0U, 1U), NODE_DELIVERY_TX_OUTCOME_LOCAL_ERROR}, /* no retry */
      {SLOT(1U, 1U), NODE_DELIVERY_TX_OUTCOME_INVALID},
      {SLOT(1U, 1U), UINT8_C(5)},
      {14U, NODE_DELIVERY_RESULT_NO_ACK_ATTEMPT_LIMIT}, /* ACK vs result */
      {13U, 1U},                                         /* charged count */
      {23U, 3U},                                         /* slot overflow */
  };
  for (size_t index = 0U; index < sizeof(invalid_two) / sizeof(invalid_two[0]);
       ++index) {
    TEST_ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_INVALID_PAYLOAD,
                       validate_mutated(two, two_length,
                                        invalid_two[index].offset,
                                        invalid_two[index].value));
  }

  uint8_t one[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t one_length = 0U;
  const node_delivery_event_t timeout_then_deadline =
      finished_with_calls(1U, NODE_DELIVERY_RESULT_RADIO_CYCLE_DEADLINE);
  TEST_ASSERT(node_persistence_record_encode_delivery(
      node_persistence_backend(), &timeout_then_deadline, one, &one_length));
  TEST_ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_VALID,
                     validate_mutated(one, one_length, 0U, one[8U]));
  /* An unused slot is all zero, and a lone call cannot claim an ACK. */
  TEST_ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_INVALID_PAYLOAD,
                     validate_mutated(one, one_length, SLOT(1U, 29U), 1U));
  TEST_ASSERT_EQ_U32(
      NODE_PERSISTENCE_RECORD_INVALID_PAYLOAD,
      validate_mutated(one, one_length, SLOT(0U, 1U),
                       NODE_DELIVERY_TX_OUTCOME_ACK_RECEIVED));
  /* No call at all is valid only for a non-ACK result. */
  node_delivery_event_t preflight =
      finished_with_calls(0U, NODE_DELIVERY_RESULT_AIRTIME_BUDGET_END);
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_delivery_event(&preflight, NULL));
  preflight.detail.finished.final_result = NODE_DELIVERY_RESULT_ACCEPTED;
  TEST_ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                     node_persistence_append_delivery_event(&preflight, NULL));
  return true;
}

/* The encoder drops values outside their validity condition, so a stale
 * driver time or unrelated signal value can never be persisted. */
static bool delivery_encoder_zeroes_inactive_fields(void) {
  node_persistence_test_reset_all();
  node_delivery_event_t event =
      finished_with_calls(0U, NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR);
  event.detail.finished.tx_call_count = 1U;
  event.detail.finished.tx_calls[0] = (node_delivery_tx_call_t){
      .tx_started = false,
      .tx_done = false,
      .outcome = NODE_DELIVERY_TX_OUTCOME_LOCAL_ERROR,
      .set_tx_at_us = 5U,
      .tx_done_at_us = 6U,
      .ack_rx_done_at_us = 7U,
      .ack_rssi_dbm_x2 = INT16_C(-8),
      .ack_snr_db_x4 = INT16_C(9),
  };
  event.detail.finished.tx_calls[1].set_tx_at_us = 10U; /* unused slot */
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t length = 0U;
  TEST_ASSERT(node_persistence_record_encode_delivery(
      node_persistence_backend(), &event, record, &length));
  TEST_ASSERT_EQ_U32(NODE_PERSISTENCE_RECORD_VALID,
                     node_persistence_record_validate(
                         node_persistence_backend(),
                         NODE_PERSISTENCE_LOG_DELIVERY, record, length));
  const uint8_t *slots = record + NODE_PERSISTENCE_RECORD_HEADER_SIZE +
                         NODE_PERSISTENCE_DELIVERY_TX_CALL_OFFSET;
  TEST_ASSERT_EQ_U32(0U, slots[0U]);
  TEST_ASSERT_EQ_U32(NODE_DELIVERY_TX_OUTCOME_LOCAL_ERROR, slots[1U]);
  for (size_t index = 2U; index < 2U * NODE_PERSISTENCE_DELIVERY_TX_CALL_SIZE;
       ++index) {
    TEST_ASSERT_EQ_U32(0U, slots[index]);
  }
  return true;
}

/* A torn finished record is removed by recovery, never read as a timeout. */
static bool torn_finished_record_is_removed_not_reinterpreted(void) {
  const node_delivery_event_t started = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = 10U,
      .sample_id = 9U,
      .message_id = 1002U,
      .domain = CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK,
      .detail.started = {.start_offset_ms = 400U},
  };
  const node_delivery_event_t finished =
      finished_with_calls(2U, NODE_DELIVERY_RESULT_ACCEPTED);
  uint8_t first[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  uint8_t second[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t first_length = 0U;
  size_t second_length = 0U;
  TEST_ASSERT(node_persistence_record_encode_delivery(
      node_persistence_backend(), &started, first, &first_length));
  TEST_ASSERT(node_persistence_record_encode_delivery(
      node_persistence_backend(), &finished, second, &second_length));
  for (size_t prefix = 1U; prefix < second_length; ++prefix) {
    node_persistence_test_reset_all();
    TEST_ASSERT(fake_backend_write_file(TEST_DELIVERY_PATH, first, first_length));
    TEST_ASSERT(fake_backend_append_bytes(TEST_DELIVERY_PATH, second, prefix));
    diagn_context_t diag;
    err_curag_t result = node_persistence_append_delivery_event(&started, &diag);
    if (result != CURAG_OK) {
      TEST_ASSERT_EQ_U32(CURAG_ECORRUPT_RECORD, result);
      TEST_ASSERT_EQ_SIZE(first_length,
                          fake_backend_file_size(TEST_DELIVERY_PATH));
      result = node_persistence_append_delivery_event(&started, &diag);
    }
    TEST_ASSERT_EQ_U32(CURAG_OK, result);
    node_persistence_test_snapshot_t snapshot;
    TEST_ASSERT(node_persistence_test_snapshot(TEST_DELIVERY_PATH, &snapshot));
    TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_DELIVERY, 2U));
    TEST_ASSERT_EQ_SIZE(2U * first_length, snapshot.length);
  }
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
  TEST_ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                     node_persistence_append_pending_reading(&invalid, &diag));
  TEST_ASSERT(node_persistence_test_assert_diag(
      &diag, CURAG_OP_VALIDATE, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
      NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR, 0));
  TEST_ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                     node_persistence_append_pending_reading(&invalid, NULL));

  node_pending_reading_t pending;
  bool found = false;
  TEST_ASSERT_EQ_U32(
      CURAG_EINVALID_ARGUMENT,
      node_persistence_peek_most_recent_pending(NULL, &found, &diag));
  TEST_ASSERT_EQ_U32(
      CURAG_EINVALID_ARGUMENT,
      node_persistence_peek_most_recent_pending(&pending, NULL, &diag));

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
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_pending_reading(&valid, &diag));
  const diagn_context_t cleared = {0};
  TEST_ASSERT(memcmp(&diag, &cleared, sizeof(diag)) == 0);
  return true;
}

static bool cleanup_diagnostic_is_valid_but_next_operation_is_not(void) {
  node_persistence_test_reset_all();
  diagn_context_t diag;
  diagn_context_t sensor_context;
  curag_diagnostic_context_clear(&sensor_context);
  sensor_context.operation = CURAG_OP_CLEANUP;
  sensor_context.context_schema = UINT8_C(1);
  sensor_context.context_length = UINT8_C(48);
  sensor_context.context[0] = UINT8_C(0xa5);
  node_diagnostic_event_t event = {
      .error = curag_error_make(CURAG_EDOM_SENSORS, UINT16_C(5)),
      .flags = NODE_DIAGNOSTIC_APPLICATION_OFFSET_VALID |
               NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID,
      .application_offset_ms = 12U,
      .cycle_sample_id = 34U,
      .context = &sensor_context,
  };
  TEST_ASSERT_EQ_U32(CURAG_OK,
                     node_persistence_append_diagnostic_event(&event, &diag));

  sensor_context.operation = (curag_operation_t)(CURAG_OP_CLEANUP + 1U);
  TEST_ASSERT_EQ_U32(CURAG_EINVALID_ARGUMENT,
                     node_persistence_append_diagnostic_event(&event, &diag));
  TEST_ASSERT_EQ_U32(CURAG_OK, node_persistence_sync_all(&diag));

  node_persistence_test_snapshot_t snapshot;
  TEST_ASSERT(node_persistence_test_snapshot(TEST_DIAGNOSTIC_PATH, &snapshot));
  TEST_ASSERT(validate_records(&snapshot, NODE_PERSISTENCE_LOG_DIAGNOSTIC, 1U));
  return true;
}

static const node_persistence_test_case_t CASES[] = {
    {"delivery_outcomes_survive_recovery_and_reject_unknown_values",
     delivery_outcomes_survive_recovery_and_reject_unknown_values},
    {"pending_round_trip_is_newest_first_and_survives_restart",
     pending_round_trip_is_newest_first_and_survives_restart},
    {"removal_requires_exact_id_and_empty_is_not_an_error",
     removal_requires_exact_id_and_empty_is_not_an_error},
    {"every_record_type_round_trips_boundary_values",
     every_record_type_round_trips_boundary_values},
    {"backlog_binding_round_trips_exact_frame_across_restart",
     backlog_binding_round_trips_exact_frame_across_restart},
    {"delivery_is_synced_before_each_successful_return",
     delivery_is_synced_before_each_successful_return},
    {"delivery_tx_calls_must_be_canonical",
     delivery_tx_calls_must_be_canonical},
    {"delivery_encoder_zeroes_inactive_fields",
     delivery_encoder_zeroes_inactive_fields},
    {"torn_finished_record_is_removed_not_reinterpreted",
     torn_finished_record_is_removed_not_reinterpreted},
    {"diagnostics_buffer_until_sync_and_sync_closes_without_unmount",
     diagnostics_buffer_until_sync_and_sync_closes_without_unmount},
    {"invalid_inputs_and_optional_diagnostics_are_consistent",
     invalid_inputs_and_optional_diagnostics_are_consistent},
    {"cleanup_diagnostic_is_valid_but_next_operation_is_not",
     cleanup_diagnostic_is_valid_but_next_operation_is_not},
};

const node_persistence_test_group_t NODE_PERSISTENCE_RECORD_TEST_GROUP = {
    .name = "records",
    .cases = CASES,
    .count = sizeof(CASES) / sizeof(CASES[0]),
};

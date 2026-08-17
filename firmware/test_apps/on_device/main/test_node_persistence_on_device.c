#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_littlefs.h"
#include "esp_system.h"
#include "node_persistence.h"
#include "persistence_test_support.h"
#include "unity.h"

void setUp(void) {}

void tearDown(void) {}

static void append_pending(uint32_t sample_id) {
  const cura_lora_v2_reading_t reading =
      hwtest_make_reading((uint16_t)sample_id);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_append_pending_reading(&reading, &diag));
}

static void assert_pending(uint32_t expected_id) {
  node_pending_reading_t pending;
  bool found = false;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                        &pending, &found, &diag));
  TEST_ASSERT_TRUE(found);
  TEST_ASSERT_EQUAL_UINT32(expected_id, pending.reading.sample_id);
  const cura_lora_v2_reading_t expected_reading =
      hwtest_make_reading((uint16_t)expected_id);
  hwtest_assert_reading_equal(&expected_reading, &pending.reading);
}

static void assert_pending_empty(void) {
  node_pending_reading_t pending;
  bool found = true;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                        &pending, &found, &diag));
  TEST_ASSERT_FALSE(found);
}

static void remove_pending(uint32_t sample_id) {
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_remove_newest_reading(sample_id, &diag));
}

static void restart_monotonic_stage_1(void) {
  hwtest_erase_state();
  uint32_t sample_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(0U, sample_id);
  esp_restart();
}

static void restart_monotonic_stage_2(void) {
  uint32_t sample_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(1U, sample_id);
  esp_restart();
}

static void restart_monotonic_stage_3(void) {
  uint32_t sample_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(2U, sample_id);
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "persistence sample IDs are monotonic across software restarts",
    "[node_persistence][reset=SW_CPU_RESET,SW_CPU_RESET]",
    restart_monotonic_stage_1, restart_monotonic_stage_2,
    restart_monotonic_stage_3);

static void restart_committed_claim_stage_1(void) {
  hwtest_erase_state();
  uint32_t sample_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(0U, sample_id);
  esp_restart();
}

static void restart_committed_claim_stage_2(void) {
  uint32_t sample_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(1U, sample_id);
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "a successful persistence claim is committed before return",
    "[node_persistence][reset=SW_CPU_RESET]", restart_committed_claim_stage_1,
    restart_committed_claim_stage_2);

static void restart_exhaustion_stage_1(void) {
  hwtest_erase_state();
  hwtest_seed_next_sample_id(UINT32_MAX);
  uint32_t sample_id = 7U;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_ESAMPLE_ID_EXHAUSTED,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(7U, sample_id);
  esp_restart();
}

static void restart_exhaustion_stage_2(void) {
  uint32_t sample_id = 9U;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_ESAMPLE_ID_EXHAUSTED,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(9U, sample_id);
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
  node_persistence_test_reset();
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, hwtest_read_next_sample_id());
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES("persistence sample ID exhaustion survives restart",
                          "[node_persistence][reset=SW_CPU_RESET]",
                          restart_exhaustion_stage_1,
                          restart_exhaustion_stage_2);

static void restart_message_monotonic_stage_1(void) {
  hwtest_erase_state();
  uint32_t message_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(0U, message_id);
  esp_restart();
}

static void restart_message_monotonic_stage_2(void) {
  uint32_t message_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(1U, message_id);
  esp_restart();
}

static void restart_message_monotonic_stage_3(void) {
  uint32_t message_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(2U, message_id);
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "persistence message IDs are monotonic across software restarts",
    "[node_persistence][reset=SW_CPU_RESET,SW_CPU_RESET]",
    restart_message_monotonic_stage_1, restart_message_monotonic_stage_2,
    restart_message_monotonic_stage_3);

static void restart_message_committed_stage_1(void) {
  hwtest_erase_state();
  uint32_t message_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(0U, message_id);
  esp_restart();
}

static void restart_message_committed_stage_2(void) {
  TEST_ASSERT_EQUAL_UINT32(1U, hwtest_read_next_message_id());
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "a successful message claim is committed before return",
    "[node_persistence][reset=SW_CPU_RESET]", restart_message_committed_stage_1,
    restart_message_committed_stage_2);

static void restart_message_exhaustion_stage_1(void) {
  hwtest_erase_state();
  hwtest_seed_next_message_id(UINT32_MAX);
  uint32_t message_id = 7U;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_EMESSAGE_ID_EXHAUSTED,
      node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(7U, message_id);
  esp_restart();
}

static void restart_message_exhaustion_stage_2(void) {
  uint32_t message_id = 9U;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_EMESSAGE_ID_EXHAUSTED,
      node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(9U, message_id);
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, hwtest_read_next_message_id());
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES("persistence message ID exhaustion survives restart",
                          "[node_persistence][reset=SW_CPU_RESET]",
                          restart_message_exhaustion_stage_1,
                          restart_message_exhaustion_stage_2);

TEST_CASE("sample and message counters advance independently",
          "[node_persistence]") {
  hwtest_erase_state();
  diagn_context_t diag;
  uint32_t sample_id = UINT32_MAX;
  uint32_t message_id = UINT32_MAX;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_claim_message_id(&message_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(0U, sample_id);
  TEST_ASSERT_EQUAL_UINT32(0U, message_id);
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
  node_persistence_test_reset();
  TEST_ASSERT_EQUAL_UINT32(1U, hwtest_read_next_sample_id());
  TEST_ASSERT_EQUAL_UINT32(1U, hwtest_read_next_message_id());
  hwtest_finish_case();
}

TEST_CASE("persistence initializes NVS and LittleFS independently",
          "[node_persistence]") {
  hwtest_erase_state();
  TEST_ASSERT_FALSE(esp_littlefs_mounted(HWTEST_LITTLEFS_PARTITION));
  uint32_t sample_id = UINT32_MAX;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(0U, sample_id);
  TEST_ASSERT_FALSE(esp_littlefs_mounted(HWTEST_LITTLEFS_PARTITION));
  append_pending(0U);
  TEST_ASSERT_TRUE(esp_littlefs_mounted(HWTEST_LITTLEFS_PARTITION));
  assert_pending(0U);
  hwtest_finish_case();
}

static void restart_pending_round_trip_stage_1(void) {
  hwtest_erase_state();
  append_pending(42U);
  esp_restart();
}

static void restart_pending_round_trip_stage_2(void) {
  assert_pending(42U);
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "pending reading round trip survives software restart",
    "[node_persistence][reset=SW_CPU_RESET]",
    restart_pending_round_trip_stage_1, restart_pending_round_trip_stage_2);

static void restart_bound_backlog_stage_1(void) {
  hwtest_erase_state();
  append_pending(42U);
  uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE];
  memset(frame, 0xa5, sizeof(frame));
  frame[CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET] = CURA_LORA_V2_CONTROL;
  frame[CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET] =
      CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK;
  node_persistence_store_le32(frame +
                                  CURA_LORA_V2_CLEAR_HEADER_MESSAGE_ID_OFFSET,
                              UINT32_C(0x11223344));
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_bind_newest_backlog_frame(
                              42U, UINT32_C(0x11223344), frame, &diag));
  esp_restart();
}

static void restart_bound_backlog_stage_2(void) {
  node_pending_reading_t pending;
  bool found = false;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                        &pending, &found, &diag));
  TEST_ASSERT_TRUE(found);
  TEST_ASSERT_TRUE(pending.backlog_bound);
  TEST_ASSERT_EQUAL_UINT32(42U, pending.reading.sample_id);
  TEST_ASSERT_EQUAL_HEX32(UINT32_C(0x11223344), pending.message_id);
  uint8_t expected[CURA_LORA_V2_READING_FRAME_SIZE];
  memset(expected, 0xa5, sizeof(expected));
  expected[CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET] = CURA_LORA_V2_CONTROL;
  expected[CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET] =
      CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK;
  node_persistence_store_le32(expected +
                                  CURA_LORA_V2_CLEAR_HEADER_MESSAGE_ID_OFFSET,
                              UINT32_C(0x11223344));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, pending.frame, sizeof(expected));
  remove_pending(42U);
  assert_pending_empty();
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "bound backlog reuses exact frame across software restart",
    "[node_persistence][reset=SW_CPU_RESET]", restart_bound_backlog_stage_1,
    restart_bound_backlog_stage_2);

TEST_CASE("pending selection and removal are newest first",
          "[node_persistence]") {
  hwtest_erase_state();
  for (uint32_t sample_id = 0U; sample_id < 3U; ++sample_id) {
    append_pending(sample_id);
  }
  for (uint32_t expected = 3U; expected > 0U; --expected) {
    assert_pending(expected - 1U);
    remove_pending(expected - 1U);
  }
  assert_pending_empty();
  hwtest_finish_case();
}

TEST_CASE("pending removal requires the expected sample ID",
          "[node_persistence]") {
  hwtest_erase_state();
  append_pending(4U);
  static hwtest_snapshot_t before;
  hwtest_snapshot(HWTEST_PENDING_PATH, &before);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_ERECORD_MISMATCH,
                          node_persistence_remove_newest_reading(3U, &diag));
  static hwtest_snapshot_t after;
  hwtest_snapshot(HWTEST_PENDING_PATH, &after);
  hwtest_assert_snapshot_equal(&before, &after);
  assert_pending(4U);
  hwtest_finish_case();
}

static void restart_removed_stage_1(void) {
  hwtest_erase_state();
  append_pending(1U);
  append_pending(2U);
  remove_pending(2U);
  esp_restart();
}

static void restart_removed_stage_2(void) {
  assert_pending(1U);
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES("removed pending data stays removed across restart",
                          "[node_persistence][reset=SW_CPU_RESET]",
                          restart_removed_stage_1, restart_removed_stage_2);

static void restart_quarantine_stage_1(void) {
  hwtest_erase_state();
  const cura_lora_v2_reading_t reading = hwtest_make_reading(7U);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_quarantine_reading(&reading, &diag));
  esp_restart();
}

static void restart_quarantine_stage_2(void) {
  static hwtest_snapshot_t actual;
  hwtest_snapshot(HWTEST_QUARANTINE_PATH, &actual);
  static hwtest_snapshot_t expected;
  const cura_lora_v2_reading_t reading = hwtest_make_reading(7U);
  expected.length = hwtest_encode_reading_record(
      NODE_PERSISTENCE_RECORD_TYPE_QUARANTINED_READING, 7U, &reading,
      expected.bytes);
  hwtest_assert_snapshot_equal(&expected, &actual);
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "quarantined reading survives restart with exact encoding",
    "[node_persistence][reset=SW_CPU_RESET]", restart_quarantine_stage_1,
    restart_quarantine_stage_2);

static void restart_delivery_stage_1(void) {
  hwtest_erase_state();
  const node_delivery_event_t started = hwtest_make_delivery_started(11U, 7U);
  const node_delivery_event_t finished = hwtest_make_delivery_finished(11U, 7U);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_append_delivery_event(&started, &diag));
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_append_delivery_event(&finished, &diag));
  esp_restart();
}

static void restart_delivery_stage_2(void) {
  static hwtest_snapshot_t actual;
  hwtest_snapshot(HWTEST_DELIVERY_PATH, &actual);
  static hwtest_snapshot_t expected;
  const node_delivery_event_t started = hwtest_make_delivery_started(11U, 7U);
  const node_delivery_event_t finished = hwtest_make_delivery_finished(11U, 7U);
  expected.length = hwtest_encode_delivery_record(&started, expected.bytes);
  expected.length += hwtest_encode_delivery_record(
      &finished, expected.bytes + expected.length);
  hwtest_assert_snapshot_equal(&expected, &actual);
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "delivery events are durable before successful return",
    "[node_persistence][reset=SW_CPU_RESET]", restart_delivery_stage_1,
    restart_delivery_stage_2);

static void restart_diagnostic_stage_1(void) {
  hwtest_erase_state();
  diagn_context_t context;
  const node_diagnostic_event_t event = hwtest_make_diagnostic(&context, 3U);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_append_diagnostic_event(&event, &diag));
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
  esp_restart();
}

static void restart_diagnostic_stage_2(void) {
  static hwtest_snapshot_t actual;
  hwtest_snapshot(HWTEST_DIAGNOSTIC_PATH, &actual);
  static hwtest_snapshot_t expected;
  diagn_context_t context;
  const node_diagnostic_event_t event = hwtest_make_diagnostic(&context, 3U);
  expected.length = hwtest_encode_diagnostic_record(&event, expected.bytes);
  hwtest_assert_snapshot_equal(&expected, &actual);
  hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES("diagnostics become durable after sync all",
                          "[node_persistence][reset=SW_CPU_RESET]",
                          restart_diagnostic_stage_1,
                          restart_diagnostic_stage_2);

TEST_CASE("sync all closes handles without unregistering LittleFS",
          "[node_persistence]") {
  hwtest_erase_state();
  diagn_context_t context;
  const node_diagnostic_event_t event = hwtest_make_diagnostic(&context, 1U);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_append_diagnostic_event(&event, &diag));
  TEST_ASSERT_TRUE(esp_littlefs_mounted(HWTEST_LITTLEFS_PARTITION));
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
  TEST_ASSERT_TRUE(esp_littlefs_mounted(HWTEST_LITTLEFS_PARTITION));
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_append_diagnostic_event(&event, &diag));
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
  hwtest_finish_case();
}

static void
prepare_tail_recovery(uint8_t record[static NODE_PERSISTENCE_RECORD_MAX_SIZE],
                      size_t *out_length) {
  append_pending(1U);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
  const cura_lora_v2_reading_t reading = hwtest_make_reading(2U);
  *out_length = hwtest_encode_reading_record(
      NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING, 2U, &reading, record);
}

static void assert_recovered_tail(err_curag_t expected_first_error) {
  node_pending_reading_t pending;
  bool found = false;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      expected_first_error,
      node_persistence_peek_most_recent_pending(&pending, &found, &diag));
  assert_pending(1U);
  static hwtest_snapshot_t snapshot;
  hwtest_snapshot(HWTEST_PENDING_PATH, &snapshot);
  TEST_ASSERT_EQUAL_UINT32(NODE_PERSISTENCE_READING_PAYLOAD_SIZE +
                               NODE_PERSISTENCE_RECORD_OVERHEAD,
                           snapshot.length);
}

TEST_CASE("LittleFS removes a representative torn pending tail",
          "[node_persistence]") {
  hwtest_erase_state();
  static uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t record_length = 0U;
  prepare_tail_recovery(record, &record_length);
  TEST_ASSERT_GREATER_THAN(17U, record_length);
  hwtest_append_raw(HWTEST_PENDING_PATH, record, 17U);
  assert_recovered_tail(CURAG_ECORRUPT_RECORD);
  hwtest_finish_case();
}

TEST_CASE("LittleFS removes a complete CRC-invalid pending tail",
          "[node_persistence]") {
  hwtest_erase_state();
  static uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t record_length = 0U;
  prepare_tail_recovery(record, &record_length);
  record[record_length - 1U] ^= UINT8_C(0x80);
  hwtest_append_raw(HWTEST_PENDING_PATH, record, record_length);
  assert_recovered_tail(CURAG_ECORRUPT_RECORD);
  hwtest_finish_case();
}

TEST_CASE("LittleFS removes unsupported pending type and version tails",
          "[node_persistence]") {
  for (size_t variant = 0U; variant < 2U; ++variant) {
    hwtest_erase_state();
    static uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
    size_t record_length = 0U;
    prepare_tail_recovery(record, &record_length);
    if (variant == 0U) {
      record[4U] = UINT8_C(0x7f);
    } else {
      record[5U] = UINT8_C(0x7f);
    }
    hwtest_recalculate_record_crc(record, record_length);
    hwtest_append_raw(HWTEST_PENDING_PATH, record, record_length);
    assert_recovered_tail(CURAG_EUNSUPPORTED_RECORD);
  }
  hwtest_finish_case();
}

TEST_CASE("LittleFS removes a semantically invalid pending tail",
          "[node_persistence]") {
  hwtest_erase_state();
  static uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t record_length = 0U;
  prepare_tail_recovery(record, &record_length);
  const size_t flags_offset =
      NODE_PERSISTENCE_RECORD_HEADER_SIZE + CURA_LORA_V2_READING_FLAGS_OFFSET;
  uint16_t flags = node_persistence_load_le16(record + flags_offset);
  flags &= (uint16_t)~CURA_LORA_V2_FLAG_SOIL_0_VALID;
  node_persistence_store_le16(record + flags_offset, flags);
  hwtest_recalculate_record_crc(record, record_length);
  hwtest_append_raw(HWTEST_PENDING_PATH, record, record_length);
  assert_recovered_tail(CURAG_ECORRUPT_RECORD);
  hwtest_finish_case();
}

TEST_CASE("LittleFS preserves an unprovable pending boundary",
          "[node_persistence]") {
  hwtest_erase_state();
  assert_pending_empty();
  static uint8_t bytes[NODE_PERSISTENCE_RECORD_MAX_SIZE + 88U];
  for (size_t index = 0U; index < sizeof(bytes); ++index) {
    bytes[index] = (uint8_t)(index * 31U + 7U);
  }
  hwtest_replace_raw(HWTEST_PENDING_PATH, bytes, sizeof(bytes));
  static hwtest_snapshot_t expected;
  hwtest_snapshot(HWTEST_PENDING_PATH, &expected);
  for (size_t attempt = 0U; attempt < 2U; ++attempt) {
    node_pending_reading_t pending;
    bool found = false;
    diagn_context_t diag;
    TEST_ASSERT_EQUAL_HEX32(
        CURAG_ECORRUPT_RECORD,
        node_persistence_peek_most_recent_pending(&pending, &found, &diag));
    static hwtest_snapshot_t actual;
    hwtest_snapshot(HWTEST_PENDING_PATH, &actual);
    hwtest_assert_snapshot_equal(&expected, &actual);
  }
  hwtest_finish_case();
}

TEST_CASE("persistence append repairs corruption before appending",
          "[node_persistence]") {
  hwtest_erase_state();
  static uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t record_length = 0U;
  prepare_tail_recovery(record, &record_length);
  record[record_length - 1U] ^= UINT8_C(1);
  hwtest_append_raw(HWTEST_PENDING_PATH, record, record_length);
  const cura_lora_v2_reading_t reading = hwtest_make_reading(3U);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_ECORRUPT_RECORD,
      node_persistence_append_pending_reading(&reading, &diag));
  assert_pending(1U);
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_append_pending_reading(&reading, &diag));
  assert_pending(3U);
  hwtest_finish_case();
}

TEST_CASE("stale pending compact file is never promoted",
          "[node_persistence]") {
  hwtest_erase_state();
  append_pending(1U);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
  const cura_lora_v2_reading_t reading = hwtest_make_reading(99U);
  static uint8_t compact_record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  const size_t compact_length =
      hwtest_encode_reading_record(NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING,
                                   99U, &reading, compact_record);
  hwtest_replace_raw(HWTEST_COMPACT_PATH, compact_record, compact_length);
  hwtest_simulate_component_restart();
  assert_pending(1U);
  TEST_ASSERT_FALSE(hwtest_path_exists(HWTEST_COMPACT_PATH));
  hwtest_finish_case();
}

TEST_CASE("pending compaction retains the newest half",
          "[node_persistence][slow]") {
  hwtest_erase_state();
  for (uint32_t sample_id = 0U; sample_id < 12U; ++sample_id) {
    append_pending(sample_id);
  }
  for (uint32_t expected = 12U; expected > 6U; --expected) {
    assert_pending(expected - 1U);
    remove_pending(expected - 1U);
  }
  assert_pending_empty();
  TEST_ASSERT_FALSE(hwtest_path_exists(HWTEST_COMPACT_PATH));
  hwtest_finish_case();
}

TEST_CASE("full non-pending logs reject without changing old data",
          "[node_persistence][slow]") {
  hwtest_erase_state();
  diagn_context_t diag;
  static hwtest_snapshot_t before;
  static hwtest_snapshot_t after;

  for (uint32_t id = 0U; id < 2U; ++id) {
    const cura_lora_v2_reading_t reading = hwtest_make_reading((uint16_t)id);
    TEST_ASSERT_EQUAL_HEX32(
        CURAG_OK, node_persistence_quarantine_reading(&reading, &diag));
  }
  hwtest_snapshot(HWTEST_QUARANTINE_PATH, &before);
  const cura_lora_v2_reading_t rejected = hwtest_make_reading(3U);
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_ELOG_FULL, node_persistence_quarantine_reading(&rejected, &diag));
  hwtest_snapshot(HWTEST_QUARANTINE_PATH, &after);
  hwtest_assert_snapshot_equal(&before, &after);

  diagn_context_t context;
  const node_diagnostic_event_t first_diagnostic =
      hwtest_make_diagnostic(&context, 0U);
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_append_diagnostic_event(
                                        &first_diagnostic, &diag));
  bool diagnostic_full = false;
  for (uint8_t marker = 1U; marker < 16U; ++marker) {
    TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
    hwtest_snapshot(HWTEST_DIAGNOSTIC_PATH, &before);
    const node_diagnostic_event_t event =
        hwtest_make_diagnostic(&context, marker);
    const err_curag_t result =
        node_persistence_append_diagnostic_event(&event, &diag);
    if (result == CURAG_OK) {
      continue;
    }
    TEST_ASSERT_EQUAL_HEX32(CURAG_ELOG_FULL, result);
    hwtest_snapshot(HWTEST_DIAGNOSTIC_PATH, &after);
    hwtest_assert_snapshot_equal(&before, &after);
    diagnostic_full = true;
    break;
  }
  TEST_ASSERT_TRUE(diagnostic_full);

  for (uint32_t id = 0U; id < 4U; ++id) {
    const node_delivery_event_t event = hwtest_make_delivery_started(20U, id);
    TEST_ASSERT_EQUAL_HEX32(
        CURAG_OK, node_persistence_append_delivery_event(&event, &diag));
  }
  hwtest_snapshot(HWTEST_DELIVERY_PATH, &before);
  const node_delivery_event_t extra_delivery =
      hwtest_make_delivery_started(20U, 5U);
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_ELOG_FULL,
      node_persistence_append_delivery_event(&extra_delivery, &diag));
  hwtest_snapshot(HWTEST_DELIVERY_PATH, &after);
  hwtest_assert_snapshot_equal(&before, &after);
  hwtest_finish_case();
}

#define CHURN_OPERATION_COUNT 500U
#define CHURN_STAGE_LENGTH 100U
#define CHURN_MODEL_CAPACITY 16U
#define CHURN_PENDING_RECORD_SIZE                                              \
  (NODE_PERSISTENCE_READING_PAYLOAD_SIZE + NODE_PERSISTENCE_RECORD_OVERHEAD)
#define CHURN_MAX_RECORDS (512U / CHURN_PENDING_RECORD_SIZE)
#define CHURN_RETAIN_RECORDS (256U / CHURN_PENDING_RECORD_SIZE)

typedef struct {
  uint32_t ids[CHURN_MODEL_CAPACITY];
  size_t count;
} churn_model_t;

static void churn_model_apply(churn_model_t *model, uint32_t operation) {
  const uint32_t kind = operation % 4U;
  if (kind < 2U) {
    if (model->count + 1U > CHURN_MAX_RECORDS) {
      const size_t keep = model->count < CHURN_RETAIN_RECORDS
                              ? model->count
                              : CHURN_RETAIN_RECORDS;
      memmove(model->ids, model->ids + model->count - keep,
              keep * sizeof(model->ids[0]));
      model->count = keep;
    }
    TEST_ASSERT_LESS_THAN_UINT32(CHURN_MODEL_CAPACITY, model->count);
    model->ids[model->count++] = operation;
  } else if (kind == 3U && model->count != 0U) {
    --model->count;
  }
}

static churn_model_t churn_model_at(uint32_t operation_count) {
  churn_model_t model = {0};
  for (uint32_t operation = 0U; operation < operation_count; ++operation) {
    churn_model_apply(&model, operation);
  }
  return model;
}

static void assert_pending_matches_model(const churn_model_t *model) {
  if (model->count == 0U) {
    assert_pending_empty();
  } else {
    assert_pending(model->ids[model->count - 1U]);
  }
}

static void run_churn_stage(uint32_t begin, bool first, bool last) {
  if (first) {
    hwtest_erase_state();
  }
  churn_model_t model = churn_model_at(begin);
  assert_pending_matches_model(&model);
  const uint32_t end = begin + CHURN_STAGE_LENGTH;
  for (uint32_t operation = begin; operation < end; ++operation) {
    const uint32_t kind = operation % 4U;
    if (kind < 2U) {
      append_pending(operation);
    } else if (kind == 3U && model.count != 0U) {
      remove_pending(model.ids[model.count - 1U]);
    } else {
      assert_pending_matches_model(&model);
    }
    churn_model_apply(&model, operation);
    assert_pending_matches_model(&model);
  }
  if (last) {
    hwtest_finish_case();
  } else {
    esp_restart();
  }
}

static void churn_stage_1(void) { run_churn_stage(0U, true, false); }
static void churn_stage_2(void) { run_churn_stage(100U, false, false); }
static void churn_stage_3(void) { run_churn_stage(200U, false, false); }
static void churn_stage_4(void) { run_churn_stage(300U, false, false); }
static void churn_stage_5(void) { run_churn_stage(400U, false, true); }

TEST_CASE_MULTIPLE_STAGES(
    "persistence churn matches a reference model across restarts",
    "[node_persistence][slow][reset=SW_CPU_RESET,SW_CPU_RESET,SW_CPU_RESET,SW_"
    "CPU_RESET]",
    churn_stage_1, churn_stage_2, churn_stage_3, churn_stage_4, churn_stage_5);

#include "node_core_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "node_persistence_backend.h"
#include "node_persistence_record.h"

/* Bitwise CRC-32/ISO-HDLC, enough to run the production record codec. */
static uint32_t crc32_iso_hdlc(const uint8_t *input, size_t length) {
  uint32_t crc = UINT32_MAX;
  for (size_t index = 0U; index < length; ++index) {
    crc ^= input[index];
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? UINT32_C(0xedb88320) : 0U);
    }
  }
  return ~crc;
}

static const node_persistence_backend_t CODEC_BACKEND = {
    .crc32_iso_hdlc = crc32_iso_hdlc,
};

static const node_delivery_event_t *finished_event(size_t index) {
  const node_delivery_event_t *event = &fake_node_core.delivery_events[index];
  return event->type == NODE_DELIVERY_EVENT_FINISHED ? event : NULL;
}

/* node_core output must be exactly what persistence accepts and stores. */
static bool encodes_as_valid_record(const node_delivery_event_t *event,
                                    uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE]) {
  size_t length = 0U;
  return node_persistence_record_encode_delivery(&CODEC_BACKEND, event, record,
                                                 &length) &&
         length == NODE_PERSISTENCE_DELIVERY_FINISHED_PAYLOAD_SIZE +
                       NODE_PERSISTENCE_RECORD_OVERHEAD &&
         node_persistence_record_validate(&CODEC_BACKEND,
                                          NODE_PERSISTENCE_LOG_DELIVERY, record,
                                          length) ==
             NODE_PERSISTENCE_RECORD_VALID;
}

static const uint8_t *encoded_slot(const uint8_t *record, size_t index) {
  return record + NODE_PERSISTENCE_RECORD_HEADER_SIZE +
         NODE_PERSISTENCE_DELIVERY_TX_CALL_OFFSET +
         index * NODE_PERSISTENCE_DELIVERY_TX_CALL_SIZE;
}

static bool call_has_no_ack(const node_delivery_tx_call_t *call) {
  return call->ack_rx_done_at_us == 0U && call->ack_rssi_dbm_x2 == 0 &&
         call->ack_snr_db_x4 == 0;
}

/* No delivery-log write may separate the first transmit from the last RX. */
static bool no_delivery_write_inside_radio_work(void) {
  size_t first_radio = SIZE_MAX;
  size_t last_radio = 0U;
  for (size_t index = 0U; index < fake_node_core.trace_count; ++index) {
    const fake_node_core_trace_t value = fake_node_core.trace[index];
    if (value == FAKE_CORE_TRACE_TRANSMIT || value == FAKE_CORE_TRACE_RECEIVE) {
      first_radio = first_radio == SIZE_MAX ? index : first_radio;
      last_radio = index;
    }
  }
  for (size_t index = first_radio; index < last_radio; ++index) {
    if (fake_node_core.trace[index] == FAKE_CORE_TRACE_DELIVERY_EVENT) {
      return false;
    }
  }
  return first_radio != SIZE_MAX;
}

static bool scripted_ack(uint32_t message_id, cura_lora_v2_domain_t domain,
                         cura_lora_v2_ack_status_t status, uint64_t set_tx_at_us,
                         int16_t rssi_dbm_x2, int16_t snr_db_x4) {
  const uint64_t tx_done = set_tx_at_us + core_test_reading_airtime_us();
  if (!core_test_script_ack(message_id, domain, status, set_tx_at_us, tx_done,
                            tx_done + UINT64_C(177300))) {
    return false;
  }
  fake_node_core_rx_script_t *rx =
      &fake_node_core.rx_scripts[fake_node_core.rx_script_count - 1U];
  rx->result.rssi_dbm_x2 = rssi_dbm_x2;
  rx->result.snr_db_x4 = snr_db_x4;
  return true;
}

/* A first-call timeout followed by an ACK keeps both calls, in order, with
 * the driver's own event times and the ACK's own signal statistics. */
static bool tx_calls_record_timeout_then_ack(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const uint64_t application_start = fake_node_core.now_us;
  const uint64_t tx1_set = application_start + UINT64_C(1000);
  const uint64_t tx1_done = tx1_set + core_test_reading_airtime_us();
  const uint64_t retry_at =
      tx1_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_tx_done(tx1_set, tx1_done);
  fake_node_core_script_rx_deadline(retry_at);
  const uint64_t tx2_set = retry_at + UINT64_C(1000);
  CORE_TEST_ASSERT(scripted_ack(CORE_TEST_FIRST_MESSAGE_ID,
                                CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
                                CURA_LORA_V2_ACK_STATUS_ACCEPTED, tx2_set,
                                INT16_C(-241), INT16_C(-37)));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.delivery_event_count);
  const node_delivery_event_t *event = finished_event(1U);
  CORE_TEST_ASSERT(event != NULL);
  CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_RESULT_ACCEPTED,
                          event->detail.finished.final_result);
  CORE_TEST_ASSERT_EQ_U32(2U, event->detail.finished.attempt_count);
  CORE_TEST_ASSERT_EQ_U32(2U, event->detail.finished.tx_call_count);
  CORE_TEST_ASSERT_EQ_U64(application_start,
                          event->detail.finished.application_start_us);
  const node_delivery_tx_call_t *first = &event->detail.finished.tx_calls[0];
  CORE_TEST_ASSERT(first->tx_started && first->tx_done);
  CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_TX_OUTCOME_ACK_TIMEOUT, first->outcome);
  CORE_TEST_ASSERT_EQ_U64(tx1_set, first->set_tx_at_us);
  CORE_TEST_ASSERT_EQ_U64(tx1_done, first->tx_done_at_us);
  CORE_TEST_ASSERT(call_has_no_ack(first));
  const node_delivery_tx_call_t *second = &event->detail.finished.tx_calls[1];
  CORE_TEST_ASSERT(second->tx_started && second->tx_done);
  CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_TX_OUTCOME_ACK_RECEIVED,
                          second->outcome);
  CORE_TEST_ASSERT_EQ_U64(tx2_set, second->set_tx_at_us);
  CORE_TEST_ASSERT_EQ_U64(tx2_set + core_test_reading_airtime_us(),
                          second->tx_done_at_us);
  CORE_TEST_ASSERT_EQ_U64(second->tx_done_at_us + UINT64_C(177300),
                          second->ack_rx_done_at_us);
  CORE_TEST_ASSERT_EQ_U32((uint16_t)INT16_C(-241),
                          (uint16_t)second->ack_rssi_dbm_x2);
  CORE_TEST_ASSERT_EQ_U32((uint16_t)INT16_C(-37),
                          (uint16_t)second->ack_snr_db_x4);
  CORE_TEST_ASSERT(no_delivery_write_inside_radio_work());
  /* The retry still sent identical bytes under the same message identity. */
  CORE_TEST_ASSERT(memcmp(fake_node_core.transmissions[0].payload,
                          fake_node_core.transmissions[1].payload,
                          CURA_LORA_V2_READING_FRAME_SIZE) == 0);

  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  CORE_TEST_ASSERT(encodes_as_valid_record(event, record));
  CORE_TEST_ASSERT_EQ_U64(application_start,
                          node_persistence_load_le64(
                              record + NODE_PERSISTENCE_RECORD_HEADER_SIZE + 15U));
  CORE_TEST_ASSERT_EQ_U64(second->ack_rx_done_at_us,
                          node_persistence_load_le64(encoded_slot(record, 1U) + 18U));
  CORE_TEST_ASSERT_EQ_U32(
      (uint16_t)INT16_C(-37),
      node_persistence_load_le16(encoded_slot(record, 1U) + 28U));
  return true;
}

/* Every valid ACK status is an ACK_RECEIVED call; zero SNR is a value. */
static bool tx_calls_record_every_ack_status(void) {
  const cura_lora_v2_domain_t domains[] = {
      CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_DOMAIN_ACK_RETRY_LATER_DOWNLINK,
      CURA_LORA_V2_DOMAIN_ACK_REJECTED_UNSUPPORTED_DOWNLINK,
      CURA_LORA_V2_DOMAIN_ACK_REJECTED_MALFORMED_DOWNLINK,
  };
  const cura_lora_v2_ack_status_t statuses[] = {
      CURA_LORA_V2_ACK_STATUS_ACCEPTED,
      CURA_LORA_V2_ACK_STATUS_RETRY_LATER,
      CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED,
      CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED,
  };
  const node_delivery_final_result_t results[] = {
      NODE_DELIVERY_RESULT_ACCEPTED,
      NODE_DELIVERY_RESULT_RETRY_LATER,
      NODE_DELIVERY_RESULT_UNSUPPORTED,
      NODE_DELIVERY_RESULT_MALFORMED,
  };
  for (size_t variant = 0U; variant < 4U; ++variant) {
    node_rtc_record_t rtc;
    node_platform_ports_t platform;
    core_test_setup(&rtc, &platform);
    CORE_TEST_ASSERT(scripted_ack(CORE_TEST_FIRST_MESSAGE_ID, domains[variant],
                                  statuses[variant],
                                  fake_node_core.now_us + UINT64_C(1000),
                                  INT16_C(-96), INT16_C(0)));
    core_test_run(&rtc, &platform);
    const node_delivery_event_t *event = finished_event(1U);
    CORE_TEST_ASSERT(event != NULL);
    CORE_TEST_ASSERT_EQ_U32(results[variant],
                            event->detail.finished.final_result);
    CORE_TEST_ASSERT_EQ_U32(1U, event->detail.finished.tx_call_count);
    const node_delivery_tx_call_t *call = &event->detail.finished.tx_calls[0];
    CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_TX_OUTCOME_ACK_RECEIVED,
                            call->outcome);
    CORE_TEST_ASSERT_EQ_U32(0U, (uint16_t)call->ack_snr_db_x4);
    CORE_TEST_ASSERT(call->ack_rx_done_at_us != 0U);
    uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
    CORE_TEST_ASSERT(encodes_as_valid_record(event, record));
  }
  return true;
}

/* Complete silence leaves two ordinary timeouts and no ACK evidence. */
static bool tx_calls_record_complete_silence(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const uint64_t tx1_set = fake_node_core.now_us + UINT64_C(1000);
  const uint64_t tx1_done = tx1_set + core_test_reading_airtime_us();
  const uint64_t retry_at =
      tx1_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_tx_done(tx1_set, tx1_done);
  fake_node_core_script_rx_deadline(retry_at);
  const uint64_t tx2_set = retry_at + UINT64_C(1000);
  const uint64_t tx2_done = tx2_set + core_test_reading_airtime_us();
  fake_node_core_script_tx_done(tx2_set, tx2_done);
  fake_node_core_script_rx_deadline(tx2_done + NODE_CORE_ACK_WAIT_US);

  core_test_run(&rtc, &platform);

  const node_delivery_event_t *event = finished_event(1U);
  CORE_TEST_ASSERT(event != NULL);
  CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_RESULT_NO_ACK_ATTEMPT_LIMIT,
                          event->detail.finished.final_result);
  CORE_TEST_ASSERT_EQ_U32(2U, event->detail.finished.tx_call_count);
  for (size_t index = 0U; index < 2U; ++index) {
    const node_delivery_tx_call_t *call =
        &event->detail.finished.tx_calls[index];
    CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_TX_OUTCOME_ACK_TIMEOUT,
                            call->outcome);
    CORE_TEST_ASSERT(call_has_no_ack(call));
  }
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.transmission_count);
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  CORE_TEST_ASSERT(encodes_as_valid_record(event, record));
  return true;
}

/* Unrelated packets never supply ACK time or signal statistics. */
static bool tx_calls_ignore_invalid_acks(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const uint64_t tx_set = fake_node_core.now_us + UINT64_C(1000);
  const uint64_t tx_done = tx_set + core_test_reading_airtime_us();
  fake_node_core_script_tx_done(tx_set, tx_done);
  uint8_t wrong[CURA_LORA_V2_ACK_FRAME_SIZE];
  CORE_TEST_ASSERT(fake_node_core_make_ack(
      wrong, CORE_TEST_IDENTITY.node_key, CORE_TEST_IDENTITY.node_id,
      CORE_TEST_FIRST_MESSAGE_ID + 1U, CURA_LORA_V2_CONTROL,
      CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_ACK_STATUS_ACCEPTED));
  fake_node_core_script_rx_packet(wrong, sizeof(wrong),
                                  tx_done + UINT64_C(100000),
                                  tx_done + UINT64_C(100000));
  fake_node_core.rx_scripts[0].result.rssi_dbm_x2 = INT16_C(-80);
  fake_node_core.rx_scripts[0].result.snr_db_x4 = INT16_C(20);
  const uint64_t retry_at =
      tx_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_rx_deadline(retry_at);
  const uint64_t tx2_set = retry_at + UINT64_C(1000);
  const uint64_t tx2_done = tx2_set + core_test_reading_airtime_us();
  fake_node_core_script_tx_done(tx2_set, tx2_done);
  fake_node_core_script_rx_deadline(tx2_done + NODE_CORE_ACK_WAIT_US);

  core_test_run(&rtc, &platform);

  const node_delivery_event_t *event = finished_event(1U);
  CORE_TEST_ASSERT(event != NULL);
  CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_RESULT_NO_ACK_ATTEMPT_LIMIT,
                          event->detail.finished.final_result);
  const node_delivery_tx_call_t *call = &event->detail.finished.tx_calls[0];
  CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_TX_OUTCOME_ACK_TIMEOUT, call->outcome);
  CORE_TEST_ASSERT(call_has_no_ack(call));
  CORE_TEST_ASSERT(core_test_has_diagnostic(CURAG_EDOM_CORE,
                                            CURAG_ECORE_EACK_MESSAGE_ID));
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  CORE_TEST_ASSERT(encodes_as_valid_record(event, record));
  return true;
}

typedef enum {
  FAILURE_TX_NOT_STARTED = 0,
  FAILURE_TX_UNCERTAIN,
  FAILURE_PRE_START_DEADLINE,
  FAILURE_RX_ERROR,
  FAILURE_TRUNCATED_WAIT,
  FAILURE_PREFLIGHT,
  FAILURE_VARIANT_COUNT,
} failure_variant_t;

/* Local failures, a cut wait and a pre-start deadline are distinct; no
 * completion or ACK time is invented, and a preflight rejection has no call. */
static bool tx_calls_classify_failures(void) {
  const err_curag_t io = curag_error_make(CURAG_EDOM_RADIO, CURAG_ERADIO_EIO);
  const err_curag_t deadline =
      curag_error_make(CURAG_EDOM_RADIO, CURAG_ERADIO_EDEADLINE);
  for (int variant = 0; variant < FAILURE_VARIANT_COUNT; ++variant) {
    node_rtc_record_t rtc;
    node_platform_ports_t platform;
    core_test_setup(&rtc, &platform);
    const uint64_t radio_deadline =
        fake_node_core.now_us + NODE_CORE_RADIO_CYCLE_LIMIT_US;
    uint64_t start = fake_node_core.now_us;
    if (variant == FAILURE_TRUNCATED_WAIT) {
      start = radio_deadline - core_test_reading_min_tx_window_us();
    } else if (variant == FAILURE_PREFLIGHT) {
      start = radio_deadline - core_test_reading_min_tx_window_us() + 1U;
    }
    fake_node_core.sensor_advance_us = start - fake_node_core.now_us;
    const uint64_t set_tx = start + UINT64_C(1000);
    const uint64_t done = set_tx + core_test_reading_airtime_us();
    node_delivery_final_result_t expected_result =
        NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR;
    node_delivery_tx_outcome_t expected_outcome =
        NODE_DELIVERY_TX_OUTCOME_LOCAL_ERROR;
    bool started = true;
    bool completed = true;
    switch ((failure_variant_t)variant) {
    case FAILURE_TX_NOT_STARTED:
      /* A stale driver time must not survive without tx_started. */
      fake_node_core_script_tx_error(io, false, false, UINT64_C(5), 0U, set_tx);
      started = completed = false;
      break;
    case FAILURE_TX_UNCERTAIN:
      fake_node_core_script_tx_error(io, true, false, set_tx, UINT64_C(7), done);
      completed = false;
      break;
    case FAILURE_PRE_START_DEADLINE:
      fake_node_core_script_tx_error(deadline, false, false, 0U, 0U, set_tx);
      started = completed = false;
      expected_result = NODE_DELIVERY_RESULT_RADIO_CYCLE_DEADLINE;
      expected_outcome = NODE_DELIVERY_TX_OUTCOME_DEADLINE_EXPIRED;
      break;
    case FAILURE_RX_ERROR:
      fake_node_core_script_tx_done(set_tx, done);
      fake_node_core_script_rx_error(io, done + UINT64_C(1000));
      break;
    case FAILURE_TRUNCATED_WAIT:
      fake_node_core_script_tx_done(set_tx, done);
      fake_node_core_script_rx_deadline(radio_deadline);
      expected_result = NODE_DELIVERY_RESULT_RADIO_CYCLE_DEADLINE;
      expected_outcome = NODE_DELIVERY_TX_OUTCOME_DEADLINE_EXPIRED;
      break;
    case FAILURE_PREFLIGHT:
    case FAILURE_VARIANT_COUNT:
      expected_result = NODE_DELIVERY_RESULT_RADIO_CYCLE_DEADLINE;
      break;
    }

    core_test_run(&rtc, &platform);

    const node_delivery_event_t *event = finished_event(1U);
    CORE_TEST_ASSERT(event != NULL);
    CORE_TEST_ASSERT_EQ_U32(expected_result,
                            event->detail.finished.final_result);
    uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
    CORE_TEST_ASSERT(encodes_as_valid_record(event, record));
    if (variant == FAILURE_PREFLIGHT) {
      CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.transmission_count);
      CORE_TEST_ASSERT_EQ_U32(0U, event->detail.finished.tx_call_count);
      continue;
    }
    CORE_TEST_ASSERT_EQ_U32(1U, event->detail.finished.tx_call_count);
    CORE_TEST_ASSERT_EQ_U32(started ? 1U : 0U,
                            event->detail.finished.attempt_count);
    const node_delivery_tx_call_t *call = &event->detail.finished.tx_calls[0];
    CORE_TEST_ASSERT_EQ_U32(expected_outcome, call->outcome);
    CORE_TEST_ASSERT(call->tx_started == started);
    CORE_TEST_ASSERT(call->tx_done == completed);
    CORE_TEST_ASSERT(call_has_no_ack(call));
    const uint8_t *slot = encoded_slot(record, 0U);
    CORE_TEST_ASSERT_EQ_U64(started ? set_tx : 0U,
                            node_persistence_load_le64(slot + 2U));
    CORE_TEST_ASSERT_EQ_U64(completed ? done : 0U,
                            node_persistence_load_le64(slot + 10U));
  }
  return true;
}

/* Backlog evidence names the transmitting wake and the older reading. */
static bool tx_calls_identify_backlog_wake(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const cura_lora_v2_reading_t older = core_test_reading(9U);
  fake_node_core_add_pending(9U, &older);
  fake_node_core.claimed_sample_id = 10U;
  fake_node_core.claimed_message_ids[0] = CORE_TEST_FIRST_MESSAGE_ID;
  fake_node_core.claimed_message_ids[1] = CORE_TEST_FIRST_MESSAGE_ID + 1U;
  fake_node_core.claimed_message_id_count = 2U;
  const uint64_t application_start = fake_node_core.now_us;
  CORE_TEST_ASSERT(scripted_ack(CORE_TEST_FIRST_MESSAGE_ID,
                                CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
                                CURA_LORA_V2_ACK_STATUS_ACCEPTED,
                                application_start + UINT64_C(1000), INT16_C(-90),
                                INT16_C(12)));
  CORE_TEST_ASSERT(scripted_ack(CORE_TEST_FIRST_MESSAGE_ID + 1U,
                                CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
                                CURA_LORA_V2_ACK_STATUS_ACCEPTED,
                                application_start + UINT64_C(400000),
                                INT16_C(-92), INT16_C(8)));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(4U, fake_node_core.delivery_event_count);
  const node_delivery_event_t *backlog = finished_event(3U);
  CORE_TEST_ASSERT(backlog != NULL);
  CORE_TEST_ASSERT_EQ_U32(CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK,
                          backlog->domain);
  CORE_TEST_ASSERT_EQ_U32(10U, backlog->cycle_sample_id);
  CORE_TEST_ASSERT_EQ_U32(9U, backlog->sample_id);
  CORE_TEST_ASSERT_EQ_U64(application_start,
                          backlog->detail.finished.application_start_us);
  CORE_TEST_ASSERT_EQ_U32(1U, backlog->detail.finished.tx_call_count);
  CORE_TEST_ASSERT_EQ_U32((uint16_t)INT16_C(-92),
                          (uint16_t)backlog->detail.finished.tx_calls[0]
                              .ack_rssi_dbm_x2);
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  CORE_TEST_ASSERT(encodes_as_valid_record(finished_event(1U), record));
  CORE_TEST_ASSERT(encodes_as_valid_record(backlog, record));
  return true;
}

typedef struct {
  const char *name;
  bool (*function)(void);
} evidence_case_t;

bool node_core_test_evidence(const char *name) {
  static const evidence_case_t CASES[] = {
      {"tx_calls_record_timeout_then_ack", tx_calls_record_timeout_then_ack},
      {"tx_calls_record_every_ack_status", tx_calls_record_every_ack_status},
      {"tx_calls_record_complete_silence", tx_calls_record_complete_silence},
      {"tx_calls_ignore_invalid_acks", tx_calls_ignore_invalid_acks},
      {"tx_calls_classify_failures", tx_calls_classify_failures},
      {"tx_calls_identify_backlog_wake", tx_calls_identify_backlog_wake},
  };
  for (size_t index = 0U; index < sizeof(CASES) / sizeof(CASES[0]); ++index) {
    if (strcmp(name, CASES[index].name) == 0) {
      return CASES[index].function();
    }
  }
  return false;
}

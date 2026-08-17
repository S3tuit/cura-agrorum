#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "node_common.h"
#include "node_platform_ports.h"
#include "protocol_v2_lora_schema_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NODE_RTC_COMMITTED_V1 UINT32_C(0x43524731)

#define NODE_CORE_RADIO_CYCLE_LIMIT_US UINT64_C(30000000)
#define NODE_CORE_TX_AIRTIME_BUDGET_US UINT64_C(8000000)
#define NODE_CORE_ACK_WAIT_US UINT64_C(500000)
#define NODE_CORE_RETRY_JITTER_MIN_US UINT32_C(100000)
#define NODE_CORE_RETRY_JITTER_MAX_US UINT32_C(500000)
#define NODE_CORE_DEEP_SLEEP_DURATION_US UINT64_C(900000000)

/* CURAG_EDOM_CORE error-code assignments. */
#define CURAG_ECORE_EINVALID_ARGUMENT UINT16_C(1)
#define CURAG_ECORE_ETIME_RANGE UINT16_C(2)
#define CURAG_ECORE_ECODEC UINT16_C(3)
#define CURAG_ECORE_ECRYPTO UINT16_C(4)
#define CURAG_ECORE_EMETRICS_OVERFLOW UINT16_C(5)
#define CURAG_ECORE_EACK_LENGTH UINT16_C(6)
#define CURAG_ECORE_EACK_AUTHENTICATION UINT16_C(7)
#define CURAG_ECORE_EACK_CONTROL UINT16_C(8)
#define CURAG_ECORE_EACK_NODE_ID UINT16_C(9)
#define CURAG_ECORE_EACK_MESSAGE_ID UINT16_C(10)
#define CURAG_ECORE_EACK_DOMAIN UINT16_C(11)
#define CURAG_ECORE_EACK_BODY UINT16_C(12)
#define CURAG_ECORE_EACK_STATUS UINT16_C(13)
#define CURAG_ECORE_EACK_TIMESTAMP UINT16_C(14)

typedef struct {
  uint8_t node_id[8];
  uint8_t node_key[CURA_LORA_V2_KEY_SIZE];
} node_identity_t;

typedef struct {
  uint8_t current_tx_attempts;
  uint16_t awake_ms;
  uint16_t current_delivery_ms;
  uint8_t cycle_tx_attempts;
  uint8_t accepted_readings;
  bool current_accepted;
} node_cycle_metrics_t;

typedef struct {
  uint32_t commit_marker;
  uint32_t completed_sample_id;
  node_cycle_metrics_t metrics;
} node_rtc_record_t;

/* Copies a retained record and invalidates its resident marker immediately. */
void node_rtc_record_take(node_rtc_record_t *retained,
                          node_rtc_record_t *out_copy);

/*
 * Returns valid metrics only for an immediately preceding completed
 * deep-sleep wake. On false, out_metrics is completely zeroed.
 */
bool node_rtc_record_validate_previous(const node_rtc_record_t *record,
                                       uint8_t reset_reason,
                                       uint32_t current_sample_id,
                                       node_cycle_metrics_t *out_metrics);

/*
 * Commits one semantically valid, representable completed cycle. The marker
 * is invalidated first and written last. Invalid metrics leave it invalid.
 */
bool node_rtc_record_commit(node_rtc_record_t *record,
                            uint32_t completed_sample_id,
                            const node_cycle_metrics_t *metrics);

/* Pure inclusive boundary check for one complete reading transmission. */
bool node_core_attempt_fits(uint64_t now_us, uint64_t deadline_us,
                            uint64_t charged_airtime_us);

/* Runs one complete wake. A production sleep callback does not return. */
void node_cycle_run(const node_platform_ports_t *platform,
                    const node_identity_t *identity,
                    node_rtc_record_t *rtc_record);

#ifdef __cplusplus
}
#endif

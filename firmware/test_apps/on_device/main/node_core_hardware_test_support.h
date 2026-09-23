#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "node_core.h"
#include "node_sensors.h"
#include "protocol_v2_lora_schema_generated.h"

#define CORE_HWTEST_MAX_TRANSMISSIONS 64U
#define CORE_HWTEST_MAX_RESPONSES 8U

typedef enum {
  CORE_HWTEST_RESPONSE_SILENCE = 0,
  CORE_HWTEST_RESPONSE_ACCEPTED,
  CORE_HWTEST_RESPONSE_RETRY_LATER,
  CORE_HWTEST_RESPONSE_REJECTED_UNSUPPORTED,
  CORE_HWTEST_RESPONSE_REJECTED_MALFORMED,
} core_hwtest_response_t;

typedef struct {
  uint32_t transmission_count;
  cura_lora_v2_authenticated_reading_frame_t
      transmissions[CORE_HWTEST_MAX_TRANSMISSIONS];
  uint32_t checkpoint_transmission_count;
  uint32_t sensor_call_count;
  uint32_t force_power_off_call_count;
  uint32_t radio_sleep_call_count;
  bool backlog_binding_verified_before_tx;
  bool adapter_contract_failed;
} core_hwtest_retained_state_t;

typedef struct {
  uint32_t hook_call_count;
  bool hook_saw_committed_copy;
  bool hook_saw_invalid_retained;
  bool restart_hook_armed;
  bool restart_hook_fired;
  uint32_t restart_hook_expected_sample_id;
  bool hook_contract_failed;
} core_hwtest_restart_evidence_t;

extern node_rtc_record_t core_hwtest_rtc_record;
extern core_hwtest_retained_state_t core_hwtest_retained;
extern core_hwtest_restart_evidence_t core_hwtest_restart_evidence;

extern const node_identity_t CORE_HWTEST_IDENTITY;

void core_hwtest_reset_retained(void);
void core_hwtest_begin_case(void);
void core_hwtest_finish_case(void);
node_sensor_sample_t core_hwtest_sensor_sample(uint16_t marker);
void core_hwtest_configure_boot(const node_sensor_sample_t *sensor_sample,
                                const core_hwtest_response_t *responses,
                                size_t response_count);
node_platform_ports_t core_hwtest_platform(void);
void core_hwtest_run_cycle(const node_sensor_sample_t *sensor_sample,
                           const core_hwtest_response_t *responses,
                           size_t response_count);
bool core_hwtest_decode_transmission(size_t index,
                                     cura_lora_v2_clear_header_t *out_header,
                                     cura_lora_v2_reading_t *out_reading);
void core_hwtest_arm_restart_after_rtc_take(uint32_t expected_sample_id);
void core_hwtest_assert_adapter_counts(uint32_t expected_sensor_calls,
                                       uint32_t expected_cleanup_calls);

#include "fault_cntl.h"

#include <stdint.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_random.h"
#include "fault.h"
#include "node_identity.h"

_Static_assert(sizeof(esp_err_t) == sizeof(int32_t),
               "fault schema requires 32-bit esp_err_t");
_Static_assert(sizeof(int) == sizeof(int32_t),
               "fault schema requires 32-bit POSIX errno");

RTC_DATA_ATTR static fault_t s_fault_state;

/* 1 if a fault has already been set and is waiting to be sent to the server,
 * else 0.*/
RTC_DATA_ATTR static uint32_t pending = 0;

static uint32_t s_current_bootno;
static uint32_t s_current_sample_id = UINT32_MAX;

void fault_init(uint32_t bootno) {
  if (!pending) {
    memset(&s_fault_state, 0, sizeof(s_fault_state));
  }
  s_current_bootno = bootno;
  s_current_sample_id = UINT32_MAX;
}

void fault_set_sample_id(uint32_t sample_id) {
  s_current_sample_id = sample_id;
}

void fault_record(fault_operation_t operation, esp_err_t esp_err,
                  int posix_errno) {
  if (pending) {
    return;
  }

  fault_t fault = {
      .node_uuid = CURA_NODE_UUID_BYTES,
      .sample_id = s_current_sample_id,
      .bootno = s_current_bootno,
      .operation = operation,
      .esp_err = (int32_t)esp_err,
      .posix_errno = (int32_t)posix_errno,
  };

  esp_fill_random(fault.fault_id, sizeof(fault.fault_id));
  s_fault_state = fault;
  pending = 1;
}

esp_err_t fault_append_to_builder(wire_builder_t *builder, bool *appended) {
  if (builder == NULL || appended == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  *appended = false;
  if (!pending) {
    return ESP_OK;
  }

  uint8_t *payload = NULL;
  esp_err_t ret =
      wire_builder_reserve_event(builder, sizeof(fault_t), FAULT_SCHEMA_VERSION,
                                 FAULT_RECORD_TYPE, &payload);
  if (ret != ESP_OK) {
    return ret;
  }

  memcpy(payload, &s_fault_state, sizeof(s_fault_state));
  *appended = true;
  return ESP_OK;
}

void fault_clear(void) {
  memset(&s_fault_state, 0, sizeof(s_fault_state));
  pending = 0;
}

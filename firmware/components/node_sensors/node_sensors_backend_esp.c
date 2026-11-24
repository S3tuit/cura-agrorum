#include "node_sensors_backend.h"
#include <stdbool.h>
#include "node_sensors.h"
#include "node_sensors_power_gate.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_CURA_SEN0600_0
#include "node_sensors_sen0600.h"
#endif

static bool enabled;
static bool off_time_known;
static int64_t off_at_us;

static node_sensors_backend_result_t esp_result(curag_operation_t operation, esp_err_t status) {
  return (node_sensors_backend_result_t){
      .kind = status == ESP_OK ? NODE_SENSOR_BACKEND_STATUS_NONE : NODE_SENSOR_BACKEND_STATUS_ESP_ERR,
      .status = status, .operation = status == ESP_OK ? CURAG_OP_NONE : operation,
  };
}

uint8_t node_sensors_backend_selection(void) {
  return (uint8_t)((CONFIG_CURA_SEN0193_0 ? 1U : 0U) |
                   (CONFIG_CURA_SEN0193_1 ? 2U : 0U) |
                   (CONFIG_CURA_SEN0600_0 ? 4U : 0U) |
                   (CONFIG_CURA_SEN0600_1 ? 8U : 0U));
}

uint32_t node_sensors_backend_stabilization_ms(void) {
  return CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS;
}

void node_sensors_backend_delay_ms(uint32_t duration_ms) {
  /* A tick delay can wake just after the next tick, short of the requested
   * elapsed interval. Startup, discharge and Modbus silence are lower bounds. */
  const int64_t deadline = esp_timer_get_time() + (int64_t)duration_ms * 1000;
  int64_t remaining;
  while ((remaining = deadline - esp_timer_get_time()) > 0) {
    const TickType_t ticks = pdMS_TO_TICKS((uint32_t)((remaining + 999) / 1000));
    vTaskDelay(ticks == 0 ? 1 : ticks);
  }
}

node_sensors_backend_result_t node_sensors_backend_power_on(void) {
#if CONFIG_CURA_SEN0600_0
  /* Unknown history is assumed sufficiently old by the operating contract.
   * Within one boot, repeated acquisition must respect the measured discharge. */
  if (off_time_known) {
    const int64_t remaining = 10000000 - (esp_timer_get_time() - off_at_us);
    if (remaining > 0) node_sensors_backend_delay_ms((uint32_t)((remaining + 999) / 1000));
  }
#endif
  enabled = true; /* Also clean up after an ambiguous/partial gate operation. */
  esp_err_t status = node_sensors_power_gate_on();
#if CONFIG_CURA_SEN0600_0
  if (status == ESP_OK) status = node_sensors_sen0600_start();
#endif
  return esp_result(CURAG_OP_POWER_ON, status);
}

node_sensors_backend_result_t node_sensors_backend_power_off(void) {
  esp_err_t cleanup = ESP_OK;
#if CONFIG_CURA_SEN0600_0
  cleanup = node_sensors_sen0600_release();
#endif
  const esp_err_t gate = node_sensors_power_gate_off();
  if (gate == ESP_OK && enabled) {
    off_at_us = esp_timer_get_time();
    off_time_known = true;
    enabled = false;
  }
  return esp_result(CURAG_OP_POWER_OFF, gate != ESP_OK ? gate : cleanup);
}

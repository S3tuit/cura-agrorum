#include "config_session.h"

#include <inttypes.h>
#include <string.h>

#include "config_ack.h"
#include "device_config.h"
#include "esp_log.h"
#include "node_config.h"
#include "node_identity.h"
#include "wire.h"

static const char *TAG = "node_config";

static node_config_t make_node_config(void) {
  return (node_config_t){
      .node_uuid = CURA_NODE_UUID_BYTES,
      .soil_sensor_id = SOIL_SENSOR_ID,
      .ds18b20_sensor_id = DS18B20_SENSOR_ID,
      .env280_sensor_id = ENV280_SENSOR_ID,
      .soil_dry_mv = SOIL_DRY_MV,
      .soil_wet_mv = SOIL_WET_MV,
  };
}

esp_err_t add_node_config_event(wire_builder_t *builder) {
  if (builder == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Business rule: the server persists this config so it can interpret soil mV
   * readings with the calibration that was active when the node reported them.
   */
  uint8_t *node_config_payload = NULL;
  esp_err_t ret = wire_builder_reserve_event(
      builder, sizeof(node_config_t), NODE_CONFIG_SCHEMA_VERSION,
      NODE_CONFIG_RECORD_TYPE, &node_config_payload);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "node config frame build failed: %s", esp_err_to_name(ret));
    return ret;
  }

  node_config_t *node_config = (node_config_t *)node_config_payload;
  *node_config = make_node_config();
  return ESP_OK;
}

esp_err_t read_config_ack(int fd) {
  uint8_t ack_payload[sizeof(config_ack_t)] = {0};
  wire_event_t event = {
      .payload = ack_payload,
      .payload_capacity = sizeof(ack_payload),
  };

  esp_err_t ret = wire_read_single_event(fd, &event);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "config ACK read failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Business rule: only the exact ACK schema can authorize caching this server
   * endpoint. Any other frame means the connection is in an unknown protocol
   * state, even if TCP delivered all bytes. */
  if (event.record_type != CONFIG_ACK_RECORD_TYPE ||
      event.schema_version != CONFIG_ACK_SCHEMA_VERSION ||
      event.payload_len != sizeof(config_ack_t)) {
    ESP_LOGW(TAG, "unexpected config ACK frame record_type=%u schema=%u len=%u",
             (unsigned)event.record_type, (unsigned)event.schema_version,
             (unsigned)event.payload_len);
    return ESP_ERR_INVALID_RESPONSE;
  }

  config_ack_t ack;
  memcpy(&ack, ack_payload, sizeof(ack));

  if (ack.status != 0) {
    ESP_LOGW(TAG, "config batch rejected status=%" PRIu32, ack.status);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "config batch accepted node=%s", CURA_NODE_UUID_STR);
  return ESP_OK;
}

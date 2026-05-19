#include "handshake.h"

#include <inttypes.h>
#include <string.h>

#include "device_config.h"
#include "esp_log.h"
#include "handshake_ack.h"
#include "node_config.h"
#include "node_identity.h"
#include "tcp.h"

static const char *TAG = "handshake";

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

esp_err_t do_handshake(int fd) {
  const node_config_t node_config = make_node_config();

  /* Business rule: the server persists this config so it can interpret soil mV
   * readings with the calibration that was active when the node reported them. */
  esp_err_t ret =
      tcp_send_message(fd, &node_config, sizeof(node_config),
                       NODE_CONFIG_SCHEMA_VERSION, NODE_CONFIG_RECORD_TYPE);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "node config send failed: %s", esp_err_to_name(ret));
    return ret;
  }

  uint8_t ack_payload[sizeof(handshake_ack_t)] = {0};
  tcp_message_t message = {
      .payload = ack_payload,
      .payload_capacity = sizeof(ack_payload),
  };

  ret = tcp_read_message(fd, &message);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "handshake ACK read failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Business rule: only the exact ACK schema can authorize readings. Any other
   * frame means the connection is not in the expected protocol state. */
  if (message.record_type != HANDSHAKE_ACK_RECORD_TYPE ||
      message.schema_version != HANDSHAKE_ACK_SCHEMA_VERSION ||
      message.payload_len != sizeof(handshake_ack_t)) {
    ESP_LOGW(TAG,
             "unexpected handshake ACK frame record_type=%u schema=%u len=%u",
             (unsigned)message.record_type, (unsigned)message.schema_version,
             (unsigned)message.payload_len);
    return ESP_ERR_INVALID_RESPONSE;
  }

  handshake_ack_t ack;
  memcpy(&ack, ack_payload, sizeof(ack));

  if (ack.status != 0) {
    ESP_LOGW(TAG, "handshake rejected status=%" PRIu32, ack.status);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "handshake accepted node=%s", CURA_NODE_UUID_STR);
  return ESP_OK;
}

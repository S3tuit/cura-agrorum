#include "wifi.h"

#include <inttypes.h>
#include <stdint.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "profile.h"
#include "storage.h"

#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY
#define WIFI_CONNECT_TIMEOUT_MS CONFIG_CURA_WIFI_CONNECT_TIMEOUT_MS
#define GATEWAY_IPV4_ADDR CONFIG_CURA_GATEWAY_IPV4_ADDR
#define GATEWAY_PORT CONFIG_CURA_GATEWAY_PORT

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "wifi";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < WIFI_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      DEBUG_LOGI(TAG, "retrying WiFi connection");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGW(TAG, "WiFi disconnected");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    DEBUG_LOGI(TAG, "ESP32 IP: " IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

/* Creates a new netif for WiFi in Station mode.
 * Prefer this over 'esp_netif_create_default_wifi_sta()' since this doesn't
 * abort the program if it fails. */
static esp_err_t create_wifi_sta_netif(esp_netif_t **out_netif) {
  if (out_netif == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();
  esp_netif_t *netif = esp_netif_new(&netif_config);
  if (netif == NULL) {
    ESP_LOGE(TAG, "WiFi station netif creation failed");
    return ESP_ERR_NO_MEM;
  }

  esp_err_t ret = esp_netif_attach_wifi_station(netif);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi station netif attach failed: %s", esp_err_to_name(ret));
    esp_netif_destroy(netif);
    return ret;
  }

  ret = esp_wifi_set_default_wifi_sta_handlers();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi default STA handler registration failed: %s",
             esp_err_to_name(ret));
    esp_wifi_clear_default_wifi_driver_and_handlers(netif);
    esp_netif_destroy(netif);
    return ret;
  }

  *out_netif = netif;
  return ESP_OK;
}

esp_err_t cura_wifi_connect(void) {
  s_wifi_event_group = xEventGroupCreate();
  if (s_wifi_event_group == NULL) {
    ESP_LOGE(TAG, "failed to create WiFi event group");
    return ESP_ERR_NO_MEM;
  }

  const esp_err_t storage_ret = cura_storage_init();
  if (storage_ret != ESP_OK) {
    ESP_LOGW(TAG, "NVS unavailable; continuing with WiFi persistence disabled");
  }

  esp_err_t ret = esp_netif_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "network interface init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_event_loop_create_default();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "event loop init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  esp_netif_t *netif = NULL;
  ret = create_wifi_sta_netif(&netif);
  if (ret != ESP_OK) {
    return ret;
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  cfg.nvs_enable = storage_ret == ESP_OK;

  ret = esp_wifi_init(&cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // We currently never unregister these 2 instances so we pass NULL for the
  // 'instance' argument.
  ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi event handler registration failed: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                            &wifi_event_handler, NULL, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "IP event handler registration failed: %s",
             esp_err_to_name(ret));
    return ret;
  }

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASS,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          },
  };

  ret = esp_wifi_set_mode(WIFI_MODE_STA);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi mode config failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi station config failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_wifi_start();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(ret));
    return ret;
  }

  DEBUG_LOGI(TAG, "connecting to WiFi SSID:%s", WIFI_SSID);
  EventBits_t bits = xEventGroupWaitBits(
      s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
      pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

  if (bits & WIFI_CONNECTED_BIT) {
    DEBUG_LOGI(TAG, "connected to WiFi SSID:%s", WIFI_SSID);
    return ESP_OK;
  }
  if (bits & WIFI_FAIL_BIT) {
    ESP_LOGE(TAG, "failed to connect to WiFi SSID:%s", WIFI_SSID);
    return ESP_FAIL;
  }
  if (bits == 0) {
    ESP_LOGE(TAG, "WiFi connection timed out after %dms",
             WIFI_CONNECT_TIMEOUT_MS);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGE(TAG, "unexpected WiFi event bits: 0x%" PRIx32, (uint32_t)bits);
  return ESP_FAIL;
}

esp_err_t wifi_get_gateway_endpoint(esp_ip4_addr_t *host_ip, uint16_t *port) {
  if (host_ip == NULL || port == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = esp_netif_str_to_ip4(GATEWAY_IPV4_ADDR, host_ip);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "gateway IPv4 address %s is invalid: %s", GATEWAY_IPV4_ADDR,
             esp_err_to_name(ret));
    return ret;
  }

  *port = (uint16_t)GATEWAY_PORT;
  DEBUG_LOGI(TAG, "using gateway " IPSTR ":%u", IP2STR(host_ip),
             (unsigned)*port);
  return ESP_OK;
}

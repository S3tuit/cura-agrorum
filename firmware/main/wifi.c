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
#include "mdns.h"
#include "nvs_flash.h"

#include "profile.h"

#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
#define WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define WIFI_H2E_IDENTIFIER ""
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define WIFI_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
#define WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define WIFI_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

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

static esp_err_t init_nvs(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  return ret;
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

  esp_err_t ret = init_nvs();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_netif_init();
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
              .threshold.authmode = WIFI_SCAN_AUTH_MODE_THRESHOLD,
              .sae_pwe_h2e = WIFI_SAE_MODE,
              .sae_h2e_identifier = WIFI_H2E_IDENTIFIER,
#ifdef CONFIG_ESP_WIFI_WPA3_COMPATIBLE_SUPPORT
              .disable_wpa3_compatible_mode = 0,
#endif
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
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    DEBUG_LOGI(TAG, "connected to WiFi SSID:%s", WIFI_SSID);
    return ESP_OK;
  }
  if (bits & WIFI_FAIL_BIT) {
    ESP_LOGE(TAG, "failed to connect to WiFi SSID:%s", WIFI_SSID);
    return ESP_FAIL;
  }

  ESP_LOGE(TAG, "unexpected WiFi event bits: 0x%" PRIx32, (uint32_t)bits);
  return ESP_FAIL;
}

static esp_err_t copy_result_ipv4(const mdns_result_t *result,
                                  esp_ip4_addr_t *host_ip, uint16_t *port) {
  if (result->port == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  for (const mdns_ip_addr_t *addr = result->addr; addr != NULL;
       addr = addr->next) {
    if (addr->addr.type == ESP_IPADDR_TYPE_V4) {
      *host_ip = addr->addr.u_addr.ip4;
      *port = result->port;
      DEBUG_LOGI(TAG,
                 "resolved %s.%s.local instance=\"%s\" host=%s port=%u "
                 "ip=" IPSTR,
                 CONFIG_CURA_MDNS_SERVICE_TYPE, CONFIG_CURA_MDNS_PROTO,
                 result->instance_name != NULL ? result->instance_name : "",
                 result->hostname != NULL ? result->hostname : "",
                 (unsigned)result->port, IP2STR(&addr->addr.u_addr.ip4));
      return ESP_OK;
    }
  }

  return ESP_ERR_NOT_FOUND;
}

static esp_err_t resolve_service_host_ipv4(const mdns_result_t *result,
                                           esp_ip4_addr_t *host_ip,
                                           uint16_t *port) {
  if (result->hostname == NULL) {
    return ESP_ERR_NOT_FOUND;
  }
  if (result->port == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_ip4_addr_t addr = {0};
  esp_err_t ret =
      mdns_query_a(result->hostname, CONFIG_CURA_MDNS_QUERY_TIMEOUT_MS, &addr);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "could not resolve %s: %s", result->hostname,
             esp_err_to_name(ret));
    return ret;
  }

  *host_ip = addr;
  *port = result->port;

  DEBUG_LOGI(TAG,
             "resolved %s.%s.local instance=\"%s\" host=%s port=%u ip=" IPSTR,
             CONFIG_CURA_MDNS_SERVICE_TYPE, CONFIG_CURA_MDNS_PROTO,
             result->instance_name != NULL ? result->instance_name : "",
             result->hostname, (unsigned)result->port, IP2STR(&addr));
  return ESP_OK;
}

esp_err_t wifi_resolve_gateway(esp_ip4_addr_t *host_ip, uint16_t *port) {
  if (host_ip == NULL || port == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = mdns_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "mDNS init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  DEBUG_LOGI(TAG, "querying mDNS service %s.%s.local",
             CONFIG_CURA_MDNS_SERVICE_TYPE, CONFIG_CURA_MDNS_PROTO);

  mdns_result_t *results = NULL;
  ret = mdns_query_ptr(CONFIG_CURA_MDNS_SERVICE_TYPE, CONFIG_CURA_MDNS_PROTO,
                       CONFIG_CURA_MDNS_QUERY_TIMEOUT_MS,
                       CONFIG_CURA_MDNS_MAX_RESULTS, &results);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "mDNS PTR query failed: %s", esp_err_to_name(ret));
    return ret;
  }
  if (results == NULL) {
    ESP_LOGE(TAG, "mDNS service %s.%s.local not found",
             CONFIG_CURA_MDNS_SERVICE_TYPE, CONFIG_CURA_MDNS_PROTO);
    return ESP_ERR_NOT_FOUND;
  }

  ret = ESP_ERR_NOT_FOUND;
  for (const mdns_result_t *result = results; result != NULL;
       result = result->next) {
    ret = copy_result_ipv4(result, host_ip, port);
    if (ret == ESP_OK) {
      break;
    }

    ret = resolve_service_host_ipv4(result, host_ip, port);
    if (ret == ESP_OK) {
      break;
    }
  }

  mdns_query_results_free(results);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "mDNS service found, but no IPv4 address was resolved");
  }
  return ret;
}

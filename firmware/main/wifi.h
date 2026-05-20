#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "esp_netif_ip_addr.h"

/* Connects the ESP32 to the configured WiFi network.
 *
 * Side effects: initializes NVS, initializes esp-netif, creates the default
 * event loop, and starts WiFi STA mode.
 *
 * Returns ESP_OK once the station has an IP address. Returns an ESP-IDF error
 * code if setup fails or the configured retry budget is exhausted.
 */
esp_err_t cura_wifi_connect(void);

/* Resolves the configured Cura Agrorum gateway mDNS service.
 *
 * The ESP32 must already be connected to WiFi. Returns ESP_OK on success and
 * writes the gateway IPv4 address to 'host_ip' and TCP port to 'port'. Returns
 * an ESP-IDF error code on failure; output values are unspecified when the
 * function fails.
 */
esp_err_t wifi_resolve_gateway(esp_ip4_addr_t *host_ip, uint16_t *port);

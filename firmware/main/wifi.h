#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "esp_netif_ip_addr.h"

/* Connects the ESP32 to the configured WiFi network and resolves the gateway
 * advertised as CONFIG_CURA_MDNS_SERVICE_TYPE.CONFIG_CURA_MDNS_PROTO.local.
 *
 * Side effects: initializes NVS, initializes esp-netif, creates the default
 * event loop, starts WiFi STA mode, and runs an mDNS query.
 *
 * Returns ESP_OK on success and writes the gateway IPv4 address to 'host_ip'
 * and TCP port to 'port'. Returns an ESP-IDF error code on failure; output
 * values are unspecified when the function fails.
 */
esp_err_t wifi_connect_and_resolve_gateway(esp_ip4_addr_t *host_ip,
                                           uint16_t *port);

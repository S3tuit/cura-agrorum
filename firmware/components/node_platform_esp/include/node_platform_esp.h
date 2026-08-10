#pragma once

#include "node_platform_ports.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Returns the immutable ESP-IDF implementation of the node platform ports.
 * The returned table has process lifetime and every callback is stateless.
 */
const node_platform_ports_t *node_platform_esp_ports(void);

#ifdef __cplusplus
}
#endif

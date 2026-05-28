#pragma once

#include "esp_err.h"
#include "wire.h"

/* Appends this node's generated identity and sensor configuration to builder.
 *
 * Config is sent only when the RTC-cached server session is not trusted. The
 * server persists it before accepting later readings in the same frame, so the
 * event must be added before reading events in that frame.
 */
esp_err_t add_node_config_event(wire_builder_t *builder);

/* Reads the server response for a frame that carried node_config_t.
 *
 * Reading-only frames intentionally have no ACK right now. Config frames wait
 * for one config_ack_t so the node caches the gateway session only after the
 * server durably accepted the config batch.
 */
esp_err_t read_config_ack(int fd);

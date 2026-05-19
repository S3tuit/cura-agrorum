#pragma once

#include "esp_err.h"

/* Performs the node configuration handshake on an already-connected socket.
 *
 * The handshake is intentionally connection-local: the node sends its generated
 * UUID plus current sensor configuration, then waits for one ACK frame from the
 * server. Returns ESP_OK only when the server ACK is well-formed and has
 * status == 0. The function does not close fd.
 */
esp_err_t do_handshake(int fd);

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_netif_ip_addr.h"

typedef struct {
  uint8_t record_type;
  uint8_t schema_version;
  uint16_t payload_len;
  uint8_t *payload;
  size_t payload_capacity;
} tcp_message_t;

/* Opens a TCP connection to host_ip:port.
 *
 * Returns a socket file descriptor >= 0 when the connection is open. Returns
 * -1 when the input is invalid, the socket cannot be created, or the connection
 * attempt fails or times out. The caller owns the returned descriptor and must
 * close it with tcp_disconnect().
 *
 * The public source-level name is tcp_connect(), but the linked symbol is
 * cura_tcp_connect to avoid colliding with lwIP's own tcp_connect symbol.
 */
int cura_tcp_connect(const esp_ip4_addr_t *host_ip, uint16_t port);
#define tcp_connect cura_tcp_connect

/* Sends exactly one framed TCP protocol message on an open socket.
 *
 * Frame format:
 *   2 bytes payload length, big-endian
 *   1 byte record type
 *   1 byte schema version
 *   payload_len raw payload bytes
 *
 * The function does not close fd.
 *
 * WARN: returning OK does not mean the remote application received the bytes.
 * It only means the local TCP stack accepted the bytes into its send buffer.
 * After that, the kernel/lwIP is responsible for transmitting, retransmitting,
 * and eventually either delivering or giving up.
 */
esp_err_t tcp_send_message(int fd, const void *payload, size_t payload_len,
                           uint8_t schema_version, uint8_t record_type);

/* Reads exactly one framed TCP protocol message from an open socket.
 *
 * The caller supplies payload storage through message->payload and
 * message->payload_capacity. The function fills record_type, schema_version,
 * payload_len, and payload bytes. It returns ESP_ERR_INVALID_SIZE if the peer's
 * payload_len is larger than the caller-provided capacity; it never allocates
 * memory based on data received from the peer.
 */
esp_err_t tcp_read_message(int fd, tcp_message_t *message);

/* Gracefully finishes and closes an open descriptor returned by tcp_connect().
 *
 * The ESP32 usually enters deep sleep immediately after sending a reading. A
 * plain close() can report success while bytes are still queued in lwIP, then
 * deep sleep tears WiFi down before the server receives the full frame. This
 * function half-closes the write side, waits briefly for the server to close
 * its side, and only then closes the descriptor.
 *
 * Returns ESP_OK when the graceful close completed, ESP_ERR_TIMEOUT when the
 * peer did not close before the receive timeout, ESP_ERR_INVALID_ARG for an
 * invalid fd, and ESP_FAIL for hard socket errors. A non-OK return should make
 * callers distrust any cached TCP/session state.
 */
esp_err_t tcp_disconnect(int fd);

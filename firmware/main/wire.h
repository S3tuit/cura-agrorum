#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_netif_ip_addr.h"

/* Wire protocol v1 constants.
 *
 * CURA_WIRE_MAX_BODY_SIZE is the largest envelope body the firmware will send.
 * The builder owns one fixed-size encoded frame buffer large enough for the
 * 4-byte transport length prefix plus that body. This trades RAM for
 * deterministic allocation and simple future event replay from LittleFS.
 *
 * The builder should NOT be stack allocated since it may require too much RAM
 * for the stack.
 */
#define CURA_WIRE_ENVELOPE_VERSION 1u
#define CURA_WIRE_FRAME_HEADER_SIZE 4u
#define CURA_WIRE_ENVELOPE_HEADER_SIZE 4u
#define CURA_WIRE_EVENT_HEADER_SIZE 4u

#ifndef CURA_WIRE_MAX_BODY_SIZE
#define CURA_WIRE_MAX_BODY_SIZE 2048u
#endif

#define CURA_WIRE_MAX_FRAME_SIZE                                               \
  (CURA_WIRE_FRAME_HEADER_SIZE + CURA_WIRE_MAX_BODY_SIZE)

#ifndef CURA_WIRE_MAX_EVENTS
#define CURA_WIRE_MAX_EVENTS 64u
#endif

#if CURA_WIRE_MAX_BODY_SIZE < CURA_WIRE_ENVELOPE_HEADER_SIZE
#error "CURA_WIRE_MAX_BODY_SIZE is too small for an envelope header"
#endif

typedef struct {
  uint8_t record_type;
  uint8_t schema_version;
  uint16_t payload_len;
  uint8_t *payload;
  size_t payload_capacity;
} wire_event_t;

typedef struct {
  uint8_t buffer[CURA_WIRE_MAX_FRAME_SIZE];
  size_t len;
  uint16_t event_count;
} wire_builder_t;

/* Opens a TCP connection to host_ip:port.
 *
 * Returns a socket file descriptor >= 0 when the connection is open. Returns
 * -1 when the input is invalid, the socket cannot be created, or the connection
 * attempt fails or times out. The caller owns the returned descriptor and
 * should close it with wire_disconnect().
 */
int wire_connect(const esp_ip4_addr_t *host_ip, uint16_t port);

/* Resets a builder so it can encode a new frame.
 *
 * The backing buffer is intentionally not cleared. Callers must fill every
 * payload byte returned by wire_builder_reserve_event() before sending.
 */
void wire_builder_init(wire_builder_t *builder);

/* Adds one event to the frame and returns writable payload storage.
 *
 * This reserves payload_len bytes inside builder->buffer and stores the event
 * header immediately before them. The returned payload pointer is owned by the
 * builder and remains valid until wire_builder_init() is called again or the
 * builder object goes out of scope.
 *
 * The pointer is byte storage inside the encoded frame. Cast it only to packed
 * wire payload structs generated from protocol/schemas/, or use memcpy.
 *
 * The payload is not initialized. Callers must write the full payload before
 * calling wire_builder_send().
 */
esp_err_t wire_builder_reserve_event(wire_builder_t *builder,
                                     size_t payload_len, uint8_t schema_version,
                                     uint8_t record_type,
                                     uint8_t **payload_out);

/* Returns the encoded size of one event inside an event stream.
 *
 * 'events' must point to a stream of encoded wire events without frame or
 * envelope headers. 'offset' selects the event to inspect. On success,
 * one_event_bytes is set to 4-byte event header + payload_len.
 */
esp_err_t wire_encoded_event_size(const uint8_t *events, size_t event_bytes,
                                  size_t offset, size_t *one_event_bytes);

/* Exposes the encoded event stream currently held by a builder.
 *
 * The returned pointer starts at the first event header and excludes the frame
 * length and envelope header. The pointer is owned by the builder and remains
 * valid until the builder is reset or destroyed.
 */
esp_err_t wire_builder_get_encoded_events(const wire_builder_t *builder,
                                          const uint8_t **events,
                                          size_t *event_bytes,
                                          uint16_t *event_count);

/* Finds the complete encoded event that owns a reserved payload pointer.
 *
 * event_payload must be a pointer previously returned by
 * wire_builder_reserve_event() for this builder. The returned encoded_event
 * pointer starts at the event header, not the payload, and encoded_event_bytes
 * includes the event header plus payload bytes. The pointer is owned by the
 * builder and remains valid until the builder is reset or destroyed.
 */
esp_err_t wire_builder_get_encoded_event(const wire_builder_t *builder,
                                         const uint8_t *event_payload,
                                         const uint8_t **encoded_event,
                                         size_t *encoded_event_bytes);

/* Reserves raw encoded-event storage at the end of a builder.
 *
 * This two-step API is for storage replay paths that can read encoded events
 * directly into the builder buffer:
 *
 *   uint8_t *dst = NULL;
 *   wire_builder_reserve_encoded_events(builder, bytes, &dst);
 *   read(fd, dst, bytes);
 *   wire_builder_commit_events(builder);
 *
 * reserve advances builder->len but intentionally does not update event_count,
 * because the bytes are untrusted until they are read and committed. If the
 * read or commit fails, reset the builder with wire_builder_init() before
 * reusing it. Keep only one outstanding reservation per builder.
 */
esp_err_t wire_builder_reserve_encoded_events(wire_builder_t *builder,
                                              size_t event_bytes,
                                              uint8_t **events_out);

/* Re-counts and commits all encoded events currently held by a builder.
 *
 * This validates the complete event stream after the envelope header and sets
 * builder->event_count from the encoded bytes. Use it after writing into
 * storage reserved by wire_builder_reserve_encoded_events(). On failure, reset
 * the builder before reusing it because builder->len may include untrusted
 * reserved bytes.
 */
esp_err_t wire_builder_commit_events(wire_builder_t *builder);

/* Finalizes and sends the encoded frame on an open socket.
 *
 * The function writes the frame length and envelope header in network byte
 * order, then sends builder->buffer[0:builder->len]. It does not reset the
 * builder and it does not close fd.
 *
 * WARN: returning OK does not mean the remote application received the bytes.
 * It only means the local TCP stack accepted the bytes into its send buffer.
 * After that, the kernel/lwIP is responsible for transmitting, retransmitting,
 * and eventually either delivering or giving up.
 */
esp_err_t wire_builder_send(int fd, wire_builder_t *builder);

/* Reads exactly one single-event frame from an open socket.
 *
 * The caller supplies payload storage through event->payload and
 * event->payload_capacity. The function fills record_type, schema_version,
 * payload_len, and payload bytes. It rejects frames with zero or multiple
 * events because current firmware responses are expected to be one event, such
 * as the config ACK.
 *
 * It returns ESP_ERR_INVALID_SIZE if the peer's payload_len is larger than the
 * caller-provided capacity; it never allocates memory based on peer data.
 */
esp_err_t wire_read_single_event(int fd, wire_event_t *event);

/* Gracefully finishes and closes an open descriptor returned by wire_connect().
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
esp_err_t wire_disconnect(int fd);

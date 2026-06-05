#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "wire.h"

/* LittleFS queue limits.
 *
 * EVENT_QUEUE_MAX_BYTES is the reserved budget for buffered event bytes inside
 * the storage partition. Segment files are capped to half a max wire body so a
 * replay frame has enough room for envelope overhead and future small events.
 */
#define EVENT_QUEUE_MAX_BYTES (1u << 20)
#define EVENT_QUEUE_SEGMENT_EVENT_BYTES (CURA_WIRE_MAX_BODY_SIZE / 2u)
#define EVENT_QUEUE_PATH_MAX 32u

typedef struct {
  bool valid;
  uint32_t segment_seq;
} event_queue_bookmark_t;

/* Prepares a builder for sending when durable backlog may exist.
 *
 * If no backlog exists, this leaves builder untouched and returns an invalid
 * bookmark. If backlog exists, the oldest queued segment is appended to builder
 * after the current events. The bookmark is tied to that queued segment and
 * remains valid only until event_queue_commit_sent() is called.
 *
 * TODO: when this fails we should guarantee 'builder' is not changed.
 */
esp_err_t event_queue_prepare_send(wire_builder_t *builder,
                                   event_queue_bookmark_t *bookmark);

/* Commits a queued segment after the server acknowledged durable persistence.
 *
 * A bookmark without a queued segment is a no-op. A bookmark with a queued
 * segment deletes the segment loaded by event_queue_prepare_send() and advances
 * the queue head.
 */
esp_err_t event_queue_commit_sent(const event_queue_bookmark_t *bookmark);

/* Buffers one encoded event from a builder after a send failure.
 *
 * event_payload must be a payload pointer returned by
 * wire_builder_reserve_event() for builder. Only that event is persisted.
 */
esp_err_t event_queue_buffer_unsent_event(const wire_builder_t *builder,
                                          const uint8_t *event_payload);

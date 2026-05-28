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
  const uint8_t *unsaved_events;
  size_t unsaved_event_bytes;
} event_queue_bookmark_t;

/* Prepares a builder for sending when durable backlog may exist.
 *
 * If no backlog exists, this leaves builder untouched and returns a bookmark
 * containing only the current unsaved event byte range. If backlog exists, the
 * oldest queued segment is appended to builder after the current events. The
 * bookmark is tied to the same builder instance and remains valid only until
 * that builder is reset or reused.
 *
 * TODO: when this fails we should guarantee 'builder' is not changed.
 */
esp_err_t event_queue_prepare_send(wire_builder_t *builder,
                                   event_queue_bookmark_t *bookmark);

/* Commits a queued segment after the frame was sent and gracefully closed.
 *
 * A bookmark without a queued segment is a no-op. A bookmark with a queued
 * segment deletes the segment loaded by event_queue_prepare_send() and advances
 * the queue head.
 */
esp_err_t event_queue_commit_sent(const event_queue_bookmark_t *bookmark);

/* Buffers builder events after a send failure.
 *
 * The bookmark is required. If bookmark->valid is false, no queued segment was
 * appended and all builder events are written. If bookmark->valid is true, only
 * the bookmark's unsaved byte range is written because any queued suffix is
 * already durable.
 *
 * TODO: this and the event_queue_bookmark_t can be deleted to simply the
 * interface.
 */
esp_err_t event_queue_buffer_unsent(const wire_builder_t *builder,
                                    const event_queue_bookmark_t *bookmark);

/* Buffers one encoded event from a builder after a send failure.
 *
 * event_payload must be a payload pointer returned by
 * wire_builder_reserve_event() for builder. Only that event is persisted.
 */
esp_err_t event_queue_buffer_unsent_event(const wire_builder_t *builder,
                                          const uint8_t *event_payload);

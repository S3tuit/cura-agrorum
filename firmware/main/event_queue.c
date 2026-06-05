#include "event_queue.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_littlefs.h"
#include "esp_log.h"
#include "nvs.h"
#include "storage.h"

static const char *TAG = "event_queue";

#define EVENT_QUEUE_MOUNT_PATH "/queue"
#define EVENT_QUEUE_PARTITION_LABEL "storage"
#define EVENT_QUEUE_NVS_NAMESPACE "cura"
#define EVENT_QUEUE_NVS_HEAD_KEY "q_head"
#define EVENT_QUEUE_NVS_TAIL_KEY "q_tail"
#define EVENT_QUEUE_FILE_SUFFIX ".log"
#define EVENT_QUEUE_MAX_SEGMENTS                                               \
  (EVENT_QUEUE_MAX_BYTES / EVENT_QUEUE_SEGMENT_EVENT_BYTES)

#if EVENT_QUEUE_SEGMENT_EVENT_BYTES < CURA_WIRE_EVENT_HEADER_SIZE
#error "EVENT_QUEUE_SEGMENT_EVENT_BYTES is too small for one event header"
#endif

#if EVENT_QUEUE_MAX_SEGMENTS < 2
#error "EVENT_QUEUE_MAX_BYTES must hold at least two queue segments"
#endif

typedef struct {
  uint32_t head_seq;
  uint32_t tail_seq;
} event_queue_metadata_t;

static bool s_littlefs_mounted;

/* Returns whether the NVS head/tail window has no pending segments. */
static bool queue_empty(const event_queue_metadata_t *metadata) {
  return metadata->head_seq == metadata->tail_seq;
}

/* Returns the number of segment slots currently claimed by metadata. */
static uint32_t queued_segment_count(const event_queue_metadata_t *metadata) {
  return metadata->tail_seq - metadata->head_seq;
}

/* Maps a monotonic segment sequence into the circular filename slot. */
static uint32_t segment_slot(uint32_t segment_seq) {
  return segment_seq % EVENT_QUEUE_MAX_SEGMENTS;
}

/* Builds the LittleFS path for the circular segment slot. */
static esp_err_t build_segment_path(uint32_t segment_seq, char *path,
                                    size_t path_capacity) {
  if (path == NULL || path_capacity == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  const int written = snprintf(
      path, path_capacity, "%s/%08" PRIx32 "%s", EVENT_QUEUE_MOUNT_PATH,
      segment_slot(segment_seq), EVENT_QUEUE_FILE_SUFFIX);
  if (written < 0 || (size_t)written >= path_capacity) {
    return ESP_ERR_INVALID_SIZE;
  }
  return ESP_OK;
}

/* Mounts the LittleFS storage partition only when queue I/O is needed. */
static esp_err_t mount_littlefs(void) {
  if (s_littlefs_mounted) {
    return ESP_OK;
  }

  const esp_vfs_littlefs_conf_t conf = {
      .base_path = EVENT_QUEUE_MOUNT_PATH,
      .partition_label = EVENT_QUEUE_PARTITION_LABEL,
      .format_if_mount_failed = true,
      .dont_mount = false,
  };
  esp_err_t ret = esp_vfs_littlefs_register(&conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LittleFS mount failed: %s", esp_err_to_name(ret));
    return ret;
  }

  s_littlefs_mounted = true;
  return ESP_OK;
}

/* Opens the shared NVS namespace that stores queue metadata. */
static esp_err_t open_metadata(nvs_handle_t *handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = cura_storage_init();
  if (ret != ESP_OK) {
    return ret;
  }

  ret = nvs_open(EVENT_QUEUE_NVS_NAMESPACE, NVS_READWRITE, handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS queue metadata open failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

/* Reads queue head/tail from NVS and normalizes missing or invalid state. */
static esp_err_t read_metadata(event_queue_metadata_t *metadata) {
  if (metadata == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle = 0;
  esp_err_t ret = open_metadata(&handle);
  if (ret != ESP_OK) {
    return ret;
  }

  metadata->head_seq = 0;
  metadata->tail_seq = 0;

  ret = nvs_get_u32(handle, EVENT_QUEUE_NVS_HEAD_KEY, &metadata->head_seq);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    metadata->head_seq = 0;
    ret = ESP_OK;
  }
  if (ret == ESP_OK) {
    ret = nvs_get_u32(handle, EVENT_QUEUE_NVS_TAIL_KEY, &metadata->tail_seq);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
      metadata->tail_seq = metadata->head_seq;
      ret = ESP_OK;
    }
  }
  nvs_close(handle);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS queue metadata read failed: %s", esp_err_to_name(ret));
    return ret;
  }
  if (queued_segment_count(metadata) > EVENT_QUEUE_MAX_SEGMENTS) {
    ESP_LOGW(TAG, "queue metadata exceeded capacity; resetting queue window");
    metadata->head_seq = metadata->tail_seq;
  }
  return ESP_OK;
}

/* Persists queue head/tail after a segment is created, sent, or dropped. */
static esp_err_t write_metadata(const event_queue_metadata_t *metadata) {
  if (metadata == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle = 0;
  esp_err_t ret = open_metadata(&handle);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = nvs_set_u32(handle, EVENT_QUEUE_NVS_HEAD_KEY, metadata->head_seq);
  if (ret == ESP_OK) {
    ret = nvs_set_u32(handle, EVENT_QUEUE_NVS_TAIL_KEY, metadata->tail_seq);
  }
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }
  nvs_close(handle);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS queue metadata write failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

/* Deletes one queue segment file; missing files are already gone. */
static esp_err_t delete_segment_file(uint32_t segment_seq) {
  char path[EVENT_QUEUE_PATH_MAX] = {0};
  esp_err_t ret = build_segment_path(segment_seq, path, sizeof(path));
  if (ret != ESP_OK) {
    return ret;
  }

  if (unlink(path) == 0 || errno == ENOENT) {
    return ESP_OK;
  }

  ESP_LOGE(TAG, "delete queue segment %s failed: %s", path, strerror(errno));
  return ESP_FAIL;
}

/* Drops the oldest whole segment when capacity or corruption forces eviction.
 */
static esp_err_t drop_oldest_segment(event_queue_metadata_t *metadata) {
  if (metadata == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (queue_empty(metadata)) {
    return ESP_OK;
  }

  const uint32_t dropped_seq = metadata->head_seq;
  esp_err_t ret = delete_segment_file(dropped_seq);
  if (ret != ESP_OK) {
    return ret;
  }

  metadata->head_seq++;
  ret = write_metadata(metadata);
  if (ret == ESP_OK) {
    ESP_LOGW(TAG, "dropped oldest buffered segment seq=%" PRIu32, dropped_seq);
  }
  return ret;
}

/* Returns a file's current size. */
static esp_err_t file_size(const char *path, size_t *size) {
  if (path == NULL || size == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  struct stat st;
  if (stat(path, &st) != 0) {
    return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
  }
  if (st.st_size < 0) {
    return ESP_ERR_INVALID_SIZE;
  }

  *size = (size_t)st.st_size;
  return ESP_OK;
}

/* Writes a full byte range to a POSIX fd, handling short writes. */
static esp_err_t write_all(int fd, const uint8_t *data, size_t len) {
  if (fd < 0 || (len > 0 && data == NULL)) {
    return ESP_ERR_INVALID_ARG;
  }

  size_t written_total = 0;
  while (written_total < len) {
    ssize_t written = write(fd, &data[written_total], len - written_total);
    if (written > 0) {
      written_total += (size_t)written;
      continue;
    }
    if (written == 0) {
      ESP_LOGE(TAG, "queue file write returned 0");
      return ESP_FAIL;
    }
    if (errno == EINTR) {
      continue;
    }

    ESP_LOGE(TAG, "queue file write failed: %s", strerror(errno));
    return ESP_FAIL;
  }

  return ESP_OK;
}

/* Reads exactly len bytes from a POSIX fd into caller-provided storage. */
static esp_err_t read_all(int fd, uint8_t *data, size_t len) {
  if (fd < 0 || (len > 0 && data == NULL)) {
    return ESP_ERR_INVALID_ARG;
  }

  size_t read_total = 0;
  while (read_total < len) {
    ssize_t received = read(fd, &data[read_total], len - read_total);
    if (received > 0) {
      read_total += (size_t)received;
      continue;
    }
    if (received == 0) {
      ESP_LOGE(TAG, "queue file ended early");
      return ESP_FAIL;
    }
    if (errno == EINTR) {
      continue;
    }

    ESP_LOGE(TAG, "queue file read failed: %s", strerror(errno));
    return ESP_FAIL;
  }

  return ESP_OK;
}

/* Closes a POSIX fd and preserves any earlier error. */
static esp_err_t close_fd(int fd, esp_err_t previous_ret) {
  if (fd < 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (close(fd) != 0) {
    ESP_LOGE(TAG, "queue file close failed: %s", strerror(errno));
    return ESP_FAIL;
  }
  return previous_ret;
}

/* Appends a whole-event byte range to a chosen segment file. */
static esp_err_t append_events_to_segment(uint32_t segment_seq,
                                          const uint8_t *events,
                                          size_t event_bytes) {
  char path[EVENT_QUEUE_PATH_MAX] = {0};
  esp_err_t ret = build_segment_path(segment_seq, path, sizeof(path));
  if (ret != ESP_OK) {
    return ret;
  }

  const int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (fd < 0) {
    ESP_LOGE(TAG, "open queue segment %s failed: %s", path, strerror(errno));
    return ESP_FAIL;
  }

  ret = write_all(fd, events, event_bytes);
  return close_fd(fd, ret);
}

/* Chooses the tail segment for append. This may create room by dropping old
 * segments.
 */
static esp_err_t choose_append_segment(event_queue_metadata_t *metadata,
                                       size_t next_event_bytes,
                                       uint32_t *segment_seq,
                                       size_t *segment_size,
                                       bool *created_segment) {
  if (metadata == NULL || segment_seq == NULL || segment_size == NULL ||
      created_segment == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  *created_segment = false;
  *segment_size = 0;

  if (next_event_bytes > EVENT_QUEUE_SEGMENT_EVENT_BYTES) {
    ESP_LOGE(TAG, "single event is too large for a queue segment");
    return ESP_ERR_INVALID_SIZE;
  }

  if (!queue_empty(metadata)) {
    const uint32_t current_tail_seq = metadata->tail_seq - 1;
    char tail_path[EVENT_QUEUE_PATH_MAX] = {0};
    esp_err_t ret =
        build_segment_path(current_tail_seq, tail_path, sizeof(tail_path));
    if (ret != ESP_OK) {
      return ret;
    }

    size_t tail_size = 0;
    ret = file_size(tail_path, &tail_size);
    if (ret == ESP_OK &&
        tail_size + next_event_bytes <= EVENT_QUEUE_SEGMENT_EVENT_BYTES) {
      *segment_seq = current_tail_seq;
      *segment_size = tail_size;
      return ESP_OK;
    }
    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
      ESP_LOGW(TAG, "tail queue segment stat failed; opening a new segment");
    }
  }

  while (queued_segment_count(metadata) >= EVENT_QUEUE_MAX_SEGMENTS) {
    esp_err_t ret = drop_oldest_segment(metadata);
    if (ret != ESP_OK) {
      return ret;
    }
  }

  *segment_seq = metadata->tail_seq;
  *created_segment = true;
  return ESP_OK;
}

/* Appends encoded events in whole-event ranges that fit segment boundaries. */
static esp_err_t append_encoded_events(const uint8_t *events,
                                       size_t event_bytes) {
  if (event_bytes == 0) {
    return ESP_OK;
  }
  if (events == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  event_queue_metadata_t metadata = {0};
  esp_err_t ret = read_metadata(&metadata);
  if (ret != ESP_OK) {
    return ret;
  }

  size_t offset = 0;
  while (offset < event_bytes) {
    size_t next_event_bytes = 0;
    ret =
        wire_encoded_event_size(events, event_bytes, offset, &next_event_bytes);
    if (ret != ESP_OK) {
      return ret;
    }

    uint32_t segment_seq = 0;
    size_t segment_size = 0;
    bool created_segment = false;
    ret = choose_append_segment(&metadata, next_event_bytes, &segment_seq,
                                &segment_size, &created_segment);
    if (ret != ESP_OK) {
      return ret;
    }

    size_t range_bytes = 0;
    const size_t segment_remaining =
        EVENT_QUEUE_SEGMENT_EVENT_BYTES - segment_size;

    while (offset + range_bytes < event_bytes) {
      size_t one_event_bytes = 0;
      ret = wire_encoded_event_size(events, event_bytes, offset + range_bytes,
                                    &one_event_bytes);
      if (ret != ESP_OK) {
        return ret;
      }
      if (range_bytes + one_event_bytes > segment_remaining) {
        break;
      }

      range_bytes += one_event_bytes;
    }

    ret = append_events_to_segment(segment_seq, &events[offset], range_bytes);
    if (ret != ESP_OK) {
      return ret;
    }

    if (created_segment) {
      metadata.tail_seq = segment_seq + 1;
      ret = write_metadata(&metadata);
      if (ret != ESP_OK) {
        return ret;
      }
    }

    offset += range_bytes;
  }

  return ESP_OK;
}

/* Appends one queue segment to the builder and records its bookmark. */
static esp_err_t load_segment_into_builder(uint32_t segment_seq,
                                           wire_builder_t *builder,
                                           event_queue_bookmark_t *bookmark) {
  char path[EVENT_QUEUE_PATH_MAX] = {0};
  esp_err_t ret = build_segment_path(segment_seq, path, sizeof(path));
  if (ret != ESP_OK) {
    return ret;
  }

  size_t segment_size = 0;
  ret = file_size(path, &segment_size);
  if (ret != ESP_OK) {
    return ret;
  }
  if (segment_size == 0 || segment_size > EVENT_QUEUE_SEGMENT_EVENT_BYTES) {
    ESP_LOGW(TAG, "queue segment %s has invalid size %u", path,
             (unsigned)segment_size);
    return ESP_ERR_INVALID_SIZE;
  }

  const size_t original_len = builder->len;
  const uint16_t original_event_count = builder->event_count;
  uint8_t *encoded_events = NULL;
  ret = wire_builder_reserve_encoded_events(builder, segment_size,
                                            &encoded_events);
  if (ret != ESP_OK) {
    return ret;
  }

  const int fd = open(path, O_RDONLY);
  if (fd < 0) {
    ESP_LOGE(TAG, "open queue segment %s failed: %s", path, strerror(errno));
    builder->len = original_len;
    builder->event_count = original_event_count;
    return ESP_FAIL;
  }
  ret = read_all(fd, encoded_events, segment_size);
  if (ret != ESP_OK) {
    ret = close_fd(fd, ret);
    builder->len = original_len;
    builder->event_count = original_event_count;
    return ret;
  }
  ret = close_fd(fd, ESP_OK);
  if (ret != ESP_OK) {
    builder->len = original_len;
    builder->event_count = original_event_count;
    return ret;
  }

  ret = wire_builder_commit_events(builder);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "queue segment %s is corrupt", path);
    builder->len = original_len;
    builder->event_count = original_event_count;
    return ret;
  }

  bookmark->valid = true;
  bookmark->segment_seq = segment_seq;
  return ESP_OK;
}

/* Loads the oldest readable segment, discarding corrupt segments as needed. */
static esp_err_t load_oldest_segment(wire_builder_t *builder,
                                     event_queue_bookmark_t *bookmark) {
  event_queue_metadata_t metadata = {0};
  esp_err_t ret = read_metadata(&metadata);
  if (ret != ESP_OK) {
    return ret;
  }

  while (!queue_empty(&metadata)) {
    const uint32_t segment_seq = metadata.head_seq;
    ret = load_segment_into_builder(segment_seq, builder, bookmark);
    if (ret == ESP_OK) {
      return ESP_OK;
    }

    ESP_LOGW(TAG, "discarding unreadable queue segment seq=%" PRIu32,
             segment_seq);
    esp_err_t drop_ret = drop_oldest_segment(&metadata);
    if (drop_ret != ESP_OK) {
      return drop_ret;
    }
  }

  return ESP_OK;
}

esp_err_t event_queue_prepare_send(wire_builder_t *builder,
                                   event_queue_bookmark_t *bookmark) {
  if (builder == NULL || bookmark == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(bookmark, 0, sizeof(*bookmark));
  bookmark->valid = false;

  event_queue_metadata_t metadata = {0};
  esp_err_t ret = read_metadata(&metadata);
  if (ret != ESP_OK) {
    return ret;
  }
  if (queue_empty(&metadata)) {
    return ESP_OK;
  }

  ret = mount_littlefs();
  if (ret != ESP_OK) {
    return ret;
  }

  return load_oldest_segment(builder, bookmark);
}

esp_err_t event_queue_commit_sent(const event_queue_bookmark_t *bookmark) {
  if (bookmark == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!bookmark->valid) {
    return ESP_OK;
  }

  esp_err_t ret = mount_littlefs();
  if (ret != ESP_OK) {
    return ret;
  }

  event_queue_metadata_t metadata = {0};
  ret = read_metadata(&metadata);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = delete_segment_file(bookmark->segment_seq);
  if (ret != ESP_OK) {
    return ret;
  }

  if (metadata.head_seq == bookmark->segment_seq) {
    metadata.head_seq++;
    if (metadata.head_seq > metadata.tail_seq) {
      metadata.tail_seq = metadata.head_seq;
    }
    ret = write_metadata(&metadata);
  }
  return ret;
}

esp_err_t event_queue_buffer_unsent_event(const wire_builder_t *builder,
                                          const uint8_t *event_payload) {
  if (builder == NULL || event_payload == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  const uint8_t *event = NULL;
  size_t event_bytes = 0;
  esp_err_t ret = wire_builder_get_encoded_event(builder, event_payload, &event,
                                                 &event_bytes);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mount_littlefs();
  if (ret != ESP_OK) {
    return ret;
  }
  return append_encoded_events(event, event_bytes);
}

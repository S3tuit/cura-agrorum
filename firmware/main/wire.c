#include "wire.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esp_log.h"
#include "profile.h"

static const char *TAG = "wire";

/*-- Read/Write converting host long - network long --*/
static void write_u16(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)((value >> 8) & 0xff);
  dst[1] = (uint8_t)(value & 0xff);
}

static void write_u32(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)((value >> 24) & 0xff);
  dst[1] = (uint8_t)((value >> 16) & 0xff);
  dst[2] = (uint8_t)((value >> 8) & 0xff);
  dst[3] = (uint8_t)(value & 0xff);
}

static uint16_t read_u16(const uint8_t *src) {
  return ((uint16_t)src[0] << 8) | (uint16_t)src[1];
}

static uint32_t read_u32(const uint8_t *src) {
  return ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) |
         ((uint32_t)src[2] << 8) | (uint32_t)src[3];
}

static struct timeval timeout_ms_to_timeval(int timeout_ms) {
  struct timeval timeout = {
      .tv_sec = timeout_ms / 1000,
      .tv_usec = (timeout_ms % 1000) * 1000,
  };
  return timeout;
}

static esp_err_t restore_socket_flags(int fd, int flags) {
  if (fcntl(fd, F_SETFL, flags) < 0) {
    ESP_LOGE(TAG, "restore socket flags failed: %s", strerror(errno));
    return ESP_FAIL;
  }
  return ESP_OK;
}

/* The ESP32 should not sit awake forever waiting on a network operation.
 * Configure blocking send/recv calls to fail after their Kconfig timeout. */
static void configure_io_timeouts(int fd) {
  struct timeval send_timeout =
      timeout_ms_to_timeval(CONFIG_CURA_TCP_SEND_TIMEOUT_MS);
  if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout,
                 sizeof(send_timeout)) < 0) {
    ESP_LOGW(TAG, "setting send timeout failed: %s", strerror(errno));
  }

  struct timeval read_timeout =
      timeout_ms_to_timeval(CONFIG_CURA_TCP_READ_TIMEOUT_MS);
  if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout,
                 sizeof(read_timeout)) < 0) {
    ESP_LOGW(TAG, "setting receive timeout failed: %s", strerror(errno));
  }
}

/* Temporarily makes the socket nonblocking so the connect phase has its own
 * timeout, then restores the caller-visible blocking socket mode. */
static esp_err_t connect_with_timeout(int fd, const struct sockaddr *addr,
                                      socklen_t addr_len) {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    ESP_LOGE(TAG, "get socket flags failed: %s", strerror(errno));
    return ESP_FAIL;
  }

  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    ESP_LOGE(TAG, "set nonblocking socket failed: %s", strerror(errno));
    return ESP_FAIL;
  }

  int ret = connect(fd, addr, addr_len);
  if (ret == 0) {
    return restore_socket_flags(fd, flags);
  }
  if (errno != EINPROGRESS) {
    ESP_LOGE(TAG, "connect failed: %s", strerror(errno));
    restore_socket_flags(fd, flags);
    return ESP_FAIL;
  }

  fd_set write_fds;
  FD_ZERO(&write_fds);
  FD_SET(fd, &write_fds);
  struct timeval timeout =
      timeout_ms_to_timeval(CONFIG_CURA_TCP_CONNECT_TIMEOUT_MS);

  // For a nonblocking connect, "writable" means the connection attempt
  // finished. SO_ERROR below tells us whether it actually succeeded.
  ret = select(fd + 1, NULL, &write_fds, NULL, &timeout);
  if (ret == 0) {
    ESP_LOGE(TAG, "connect timed out");
    restore_socket_flags(fd, flags);
    return ESP_ERR_TIMEOUT;
  }
  if (ret < 0) {
    ESP_LOGE(TAG, "connect select failed: %s", strerror(errno));
    restore_socket_flags(fd, flags);
    return ESP_FAIL;
  }

  int socket_error = 0;
  socklen_t socket_error_len = sizeof(socket_error);
  if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_len) <
      0) {
    ESP_LOGE(TAG, "connect status check failed: %s", strerror(errno));
    restore_socket_flags(fd, flags);
    return ESP_FAIL;
  }
  if (socket_error != 0) {
    ESP_LOGE(TAG, "connect failed: %s", strerror(socket_error));
    restore_socket_flags(fd, flags);
    return ESP_FAIL;
  }

  return restore_socket_flags(fd, flags);
}

static esp_err_t send_all(int fd, const void *buffer, size_t len) {
  const uint8_t *cursor = (const uint8_t *)buffer;
  size_t remaining = len;

  // TCP send may accept only part of the buffer. Keep advancing until this
  // frame is fully written or the socket reports an error.
  while (remaining > 0) {
    ssize_t sent = send(fd, cursor, remaining, 0);
    if (sent > 0) {
      cursor += sent;
      remaining -= (size_t)sent;
      continue;
    }
    if (sent == 0) {
      ESP_LOGE(TAG, "socket closed while sending");
      return ESP_FAIL;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      ESP_LOGE(TAG, "send timed out");
      return ESP_ERR_TIMEOUT;
    }

    ESP_LOGE(TAG, "send failed: %s", strerror(errno));
    return ESP_FAIL;
  }

  return ESP_OK;
}

/* Reads a complete byte range using the socket receive timeout configured by
 * wire_connect(). */
static esp_err_t read_all(int fd, void *buffer, size_t len) {
  if (fd < 0 || (len > 0 && buffer == NULL)) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t *cursor = (uint8_t *)buffer;
  size_t remaining = len;
  while (remaining > 0) {
    ssize_t received = recv(fd, cursor, remaining, 0);
    if (received > 0) {
      cursor += received;
      remaining -= (size_t)received;
      continue;
    }
    if (received == 0) {
      ESP_LOGE(TAG, "socket closed while reading");
      return ESP_FAIL;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      ESP_LOGE(TAG, "read timed out");
      return ESP_ERR_TIMEOUT;
    }

    ESP_LOGE(TAG, "read failed: %s", strerror(errno));
    return ESP_FAIL;
  }

  return ESP_OK;
}

/* After the final frame, give the peer a chance to observe EOF and close its
 * side before the ESP32 enters deep sleep. This turns "send copied bytes into
 * the local TCP buffer" into a stronger signal that the server process actually
 * consumed the frame. */
static esp_err_t wait_for_peer_close(int fd) {
  uint8_t discard[32];

  while (true) {
    ssize_t received = recv(fd, discard, sizeof(discard), 0);
    if (received > 0) {
      continue;
    }
    if (received == 0) {
      DEBUG_LOGI(TAG, "peer closed TCP connection");
      return ESP_OK;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      ESP_LOGW(TAG, "timed out waiting for peer TCP close");
      return ESP_ERR_TIMEOUT;
    }

    ESP_LOGW(TAG, "waiting for peer TCP close failed: %s", strerror(errno));
    return ESP_FAIL;
  }
}

int wire_connect(const esp_ip4_addr_t *host_ip, uint16_t port) {
  if (host_ip == NULL || port == 0) {
    return -1;
  }

  int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (fd < 0) {
    ESP_LOGE(TAG, "socket create failed: %s", strerror(errno));
    return -1;
  }

  configure_io_timeouts(fd);

  struct sockaddr_in destination = {
      .sin_family = AF_INET,
      .sin_port = htons(port),
      .sin_addr.s_addr = host_ip->addr,
  };

  esp_err_t ret = connect_with_timeout(
      fd, (const struct sockaddr *)&destination, sizeof(destination));
  if (ret != ESP_OK) {
    close(fd);
    return -1;
  }

  DEBUG_LOGI(TAG, "connected to " IPSTR ":%u", IP2STR(host_ip), (unsigned)port);
  return fd;
}

void wire_builder_init(wire_builder_t *builder) {
  if (builder == NULL) {
    return;
  }

  builder->len = CURA_WIRE_FRAME_HEADER_SIZE + CURA_WIRE_ENVELOPE_HEADER_SIZE;
  builder->event_count = 0;
}

esp_err_t wire_builder_reserve_event(wire_builder_t *builder,
                                     size_t payload_len, uint8_t schema_version,
                                     uint8_t record_type,
                                     uint8_t **payload_out) {
  if (builder == NULL || (payload_len > 0 && payload_out == NULL)) {
    return ESP_ERR_INVALID_ARG;
  }
  if (payload_len > UINT16_MAX) {
    ESP_LOGE(TAG, "payload length %u exceeds event header capacity",
             (unsigned)payload_len);
    return ESP_ERR_INVALID_SIZE;
  }
  if (builder->event_count >= CURA_WIRE_MAX_EVENTS) {
    ESP_LOGE(TAG, "event count %u exceeds frame limit",
             (unsigned)CURA_WIRE_MAX_EVENTS);
    return ESP_ERR_INVALID_SIZE;
  }
  if (builder->len <
          CURA_WIRE_FRAME_HEADER_SIZE + CURA_WIRE_ENVELOPE_HEADER_SIZE ||
      builder->len > CURA_WIRE_MAX_FRAME_SIZE) {
    ESP_LOGE(TAG, "builder has invalid length %u", (unsigned)builder->len);
    return ESP_ERR_INVALID_STATE;
  }

  const size_t event_size = CURA_WIRE_EVENT_HEADER_SIZE + payload_len;
  if (event_size > CURA_WIRE_MAX_FRAME_SIZE - builder->len) {
    ESP_LOGE(TAG, "event does not fit in frame: len=%u payload_len=%u max=%u",
             (unsigned)builder->len, (unsigned)payload_len,
             (unsigned)CURA_WIRE_MAX_FRAME_SIZE);
    return ESP_ERR_INVALID_SIZE;
  }

  const uint16_t payload_len_u16 = (uint16_t)payload_len;
  uint8_t *event_header = &builder->buffer[builder->len];
  event_header[0] = record_type;
  event_header[1] = schema_version;
  write_u16(&event_header[2], payload_len_u16);

  uint8_t *payload = event_header + CURA_WIRE_EVENT_HEADER_SIZE;
  if (payload_out != NULL) {
    *payload_out = payload;
  }

  builder->len += event_size;
  builder->event_count++;

  return ESP_OK;
}

esp_err_t wire_encoded_event_size(const uint8_t *events, size_t event_bytes,
                                  size_t offset, size_t *one_event_bytes) {
  if (events == NULL || one_event_bytes == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (offset > event_bytes ||
      event_bytes - offset < CURA_WIRE_EVENT_HEADER_SIZE) {
    return ESP_ERR_INVALID_SIZE;
  }

  const uint16_t payload_len = read_u16(&events[offset + 2]);
  const size_t size = CURA_WIRE_EVENT_HEADER_SIZE + payload_len;
  if (size > event_bytes - offset) {
    ESP_LOGE(TAG, "encoded event payload exceeds stream size");
    return ESP_ERR_INVALID_SIZE;
  }

  *one_event_bytes = size;
  return ESP_OK;
}

static esp_err_t count_encoded_events(const uint8_t *events, size_t event_bytes,
                                      uint16_t *event_count) {
  if ((event_bytes > 0 && events == NULL) || event_count == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  size_t offset = 0;
  uint16_t count = 0;
  while (offset < event_bytes) {
    size_t event_size = 0;
    esp_err_t ret =
        wire_encoded_event_size(events, event_bytes, offset, &event_size);
    if (ret != ESP_OK) {
      return ret;
    }

    if (count == UINT16_MAX) {
      ESP_LOGE(TAG, "encoded event stream contains too many events");
      return ESP_ERR_INVALID_SIZE;
    }
    count++;
    offset += event_size;
  }

  *event_count = count;
  return ESP_OK;
}

esp_err_t wire_builder_reserve_encoded_events(wire_builder_t *builder,
                                              size_t event_bytes,
                                              uint8_t **events_out) {
  if (builder == NULL || event_bytes == 0 || events_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (builder->len <
          CURA_WIRE_FRAME_HEADER_SIZE + CURA_WIRE_ENVELOPE_HEADER_SIZE ||
      builder->len > CURA_WIRE_MAX_FRAME_SIZE) {
    ESP_LOGE(TAG, "builder has invalid length %u", (unsigned)builder->len);
    return ESP_ERR_INVALID_STATE;
  }
  if (event_bytes > CURA_WIRE_MAX_FRAME_SIZE - builder->len) {
    ESP_LOGE(TAG, "encoded events do not fit in frame");
    return ESP_ERR_INVALID_SIZE;
  }

  *events_out = &builder->buffer[builder->len];
  builder->len += event_bytes;
  return ESP_OK;
}

esp_err_t wire_builder_commit_events(wire_builder_t *builder) {
  if (builder == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (builder->len <
          CURA_WIRE_FRAME_HEADER_SIZE + CURA_WIRE_ENVELOPE_HEADER_SIZE ||
      builder->len > CURA_WIRE_MAX_FRAME_SIZE) {
    ESP_LOGE(TAG, "builder has invalid length %u", (unsigned)builder->len);
    return ESP_ERR_INVALID_STATE;
  }

  const size_t events_offset =
      CURA_WIRE_FRAME_HEADER_SIZE + CURA_WIRE_ENVELOPE_HEADER_SIZE;
  const size_t event_bytes = builder->len - events_offset;
  const uint8_t *events = &builder->buffer[events_offset];
  uint16_t counted_events = 0;
  esp_err_t ret = count_encoded_events(events, event_bytes, &counted_events);
  if (ret != ESP_OK) {
    return ret;
  }
  if (counted_events > CURA_WIRE_MAX_EVENTS) {
    ESP_LOGE(TAG, "encoded events exceed frame event limit");
    return ESP_ERR_INVALID_SIZE;
  }

  builder->event_count = counted_events;
  return ESP_OK;
}

esp_err_t wire_builder_get_encoded_event(const wire_builder_t *builder,
                                         const uint8_t *event_payload,
                                         const uint8_t **encoded_event,
                                         size_t *encoded_event_bytes) {
  if (builder == NULL || event_payload == NULL || encoded_event == NULL ||
      encoded_event_bytes == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (builder->len <
          CURA_WIRE_FRAME_HEADER_SIZE + CURA_WIRE_ENVELOPE_HEADER_SIZE ||
      builder->len > CURA_WIRE_MAX_FRAME_SIZE) {
    ESP_LOGE(TAG, "builder has invalid length %u", (unsigned)builder->len);
    return ESP_ERR_INVALID_STATE;
  }

  const size_t events_offset =
      CURA_WIRE_FRAME_HEADER_SIZE + CURA_WIRE_ENVELOPE_HEADER_SIZE;
  const uint8_t *events = &builder->buffer[events_offset];
  const size_t event_bytes = builder->len - events_offset;

  size_t offset = 0;
  while (offset < event_bytes) {
    size_t one_event_bytes = 0;
    esp_err_t ret =
        wire_encoded_event_size(events, event_bytes, offset, &one_event_bytes);
    if (ret != ESP_OK) {
      return ret;
    }

    const uint8_t *event_header = &events[offset];
    const uint8_t *payload = event_header + CURA_WIRE_EVENT_HEADER_SIZE;
    if (payload == event_payload) {
      *encoded_event = event_header;
      *encoded_event_bytes = one_event_bytes;
      return ESP_OK;
    }

    offset += one_event_bytes;
  }

  return ESP_ERR_NOT_FOUND;
}

esp_err_t wire_builder_send(int fd, wire_builder_t *builder) {
  if (fd < 0 || builder == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (builder->event_count == 0) {
    ESP_LOGE(TAG, "cannot send empty TCP frame");
    return ESP_ERR_INVALID_STATE;
  }
  if (builder->len <
          CURA_WIRE_FRAME_HEADER_SIZE + CURA_WIRE_ENVELOPE_HEADER_SIZE ||
      builder->len > CURA_WIRE_MAX_FRAME_SIZE) {
    ESP_LOGE(TAG, "builder has invalid length %u", (unsigned)builder->len);
    return ESP_ERR_INVALID_STATE;
  }

  const size_t body_len = builder->len - CURA_WIRE_FRAME_HEADER_SIZE;

  write_u32(&builder->buffer[0], (uint32_t)body_len);
  write_u16(&builder->buffer[CURA_WIRE_FRAME_HEADER_SIZE],
            (uint16_t)CURA_WIRE_ENVELOPE_VERSION);
  write_u16(&builder->buffer[CURA_WIRE_FRAME_HEADER_SIZE + 2],
            builder->event_count);

  esp_err_t ret = send_all(fd, builder->buffer, builder->len);
  if (ret == ESP_OK) {
    DEBUG_LOGI(TAG, "sent frame body_len=%u events=%u", (unsigned)body_len,
               (unsigned)builder->event_count);
  }
  return ret;
}

esp_err_t wire_read_single_event(int fd,
                                 const wire_expected_event_t *expected) {
  if (fd < 0 || expected == NULL ||
      (expected->payload_size > 0 && expected->payload == NULL)) {
    return ESP_ERR_INVALID_ARG;
  }
  if (expected->payload_size > UINT16_MAX) {
    ESP_LOGE(TAG, "expected payload size %u exceeds event header capacity",
             (unsigned)expected->payload_size);
    return ESP_ERR_INVALID_SIZE;
  }

  const size_t expected_body_len = CURA_WIRE_ENVELOPE_HEADER_SIZE +
                                   CURA_WIRE_EVENT_HEADER_SIZE +
                                   expected->payload_size;
  if (expected_body_len > CURA_WIRE_MAX_BODY_SIZE) {
    ESP_LOGE(TAG, "expected single-event body size %u exceeds wire limit",
             (unsigned)expected_body_len);
    return ESP_ERR_INVALID_SIZE;
  }

  uint8_t frame_header[CURA_WIRE_FRAME_HEADER_SIZE] = {0};
  esp_err_t ret = read_all(fd, frame_header, sizeof(frame_header));
  if (ret != ESP_OK) {
    return ret;
  }

  const uint32_t body_len = read_u32(frame_header);
  if (body_len != expected_body_len) {
    ESP_LOGE(TAG, "response body length %u does not match expected %u",
             (unsigned)body_len, (unsigned)expected_body_len);
    return ESP_ERR_INVALID_RESPONSE;
  }

  uint8_t envelope_header[CURA_WIRE_ENVELOPE_HEADER_SIZE] = {0};
  ret = read_all(fd, envelope_header, sizeof(envelope_header));
  if (ret != ESP_OK) {
    return ret;
  }

  const uint16_t envelope_version = read_u16(&envelope_header[0]);
  const uint16_t event_count = read_u16(&envelope_header[2]);
  if (envelope_version != CURA_WIRE_ENVELOPE_VERSION) {
    ESP_LOGE(TAG, "response envelope version %u does not match expected %u",
             (unsigned)envelope_version, (unsigned)CURA_WIRE_ENVELOPE_VERSION);
    return ESP_ERR_INVALID_RESPONSE;
  }
  if (event_count != 1) {
    ESP_LOGE(TAG, "response event count %u does not match expected 1",
             (unsigned)event_count);
    return ESP_ERR_INVALID_RESPONSE;
  }

  uint8_t event_header[CURA_WIRE_EVENT_HEADER_SIZE] = {0};
  ret = read_all(fd, event_header, sizeof(event_header));
  if (ret != ESP_OK) {
    return ret;
  }

  const uint8_t record_type = event_header[0];
  const uint8_t schema_version = event_header[1];
  const uint16_t payload_len = read_u16(&event_header[2]);
  if (record_type != expected->record_type) {
    ESP_LOGE(TAG, "response record type %u does not match expected %u",
             (unsigned)record_type, (unsigned)expected->record_type);
    return ESP_ERR_INVALID_RESPONSE;
  }
  if (schema_version != expected->schema_version) {
    ESP_LOGE(TAG, "response schema version %u does not match expected %u",
             (unsigned)schema_version, (unsigned)expected->schema_version);
    return ESP_ERR_INVALID_RESPONSE;
  }
  if (payload_len != expected->payload_size) {
    ESP_LOGE(TAG, "response payload length %u does not match expected %u",
             (unsigned)payload_len, (unsigned)expected->payload_size);
    return ESP_ERR_INVALID_RESPONSE;
  }

  ret = read_all(fd, expected->payload, expected->payload_size);
  if (ret == ESP_OK) {
    DEBUG_LOGI(TAG, "read event record_type=%u schema=%u payload_len=%u",
               (unsigned)record_type, (unsigned)schema_version,
               (unsigned)payload_len);
  }
  return ret;
}

esp_err_t wire_disconnect(int fd) {
  if (fd < 0) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t ret = ESP_OK;

  if (shutdown(fd, SHUT_WR) < 0) {
    ESP_LOGW(TAG, "socket write shutdown failed: %s", strerror(errno));
    ret = ESP_FAIL;
  } else {
    ret = wait_for_peer_close(fd);
  }
  if (close(fd) < 0) {
    ESP_LOGE(TAG, "socket close failed: %s", strerror(errno));
    ret = ESP_FAIL;
  }
  return ret;
}

#include "tcp.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esp_log.h"
#include "profile.h"

static const char *TAG = "tcp";

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

/* Reads at most 'len' bytes from 'fd' and stores them into 'buffer'.
 * This keeps reading until the frame segment is complete, the timeout fires, or
 * the peer closes the connection.*/
static esp_err_t read_all(int fd, void *buffer, size_t len) {
  uint8_t *cursor = (uint8_t *)buffer;
  size_t remaining = len;

  // TCP recv may return partial data.
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

int cura_tcp_connect(const esp_ip4_addr_t *host_ip, uint16_t port) {
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

esp_err_t tcp_send_message(int fd, const void *payload, size_t payload_len,
                           uint8_t schema_version, uint8_t record_type) {
  if (fd < 0 || payload == NULL || payload_len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (payload_len > UINT16_MAX) {
    ESP_LOGE(TAG, "payload length %u exceeds TCP frame limit",
             (unsigned)payload_len);
    return ESP_ERR_INVALID_SIZE;
  }

  const uint16_t payload_len_u16 = (uint16_t)payload_len;
  uint8_t header[4] = {
      (uint8_t)((payload_len_u16 >> 8) & 0xff),
      (uint8_t)(payload_len_u16 & 0xff),
      record_type,
      schema_version,
  };

  esp_err_t ret = send_all(fd, header, sizeof(header));
  if (ret == ESP_OK) {
    ret = send_all(fd, payload, payload_len);
  }
  if (ret == ESP_OK) {
    DEBUG_LOGI(TAG, "sent record_type=%u schema=%u payload_len=%u",
               (unsigned)record_type, (unsigned)schema_version,
               (unsigned)payload_len);
  }
  return ret;
}

esp_err_t tcp_read_message(int fd, tcp_message_t *message) {
  if (fd < 0 || message == NULL || message->payload == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t header[4] = {0};
  esp_err_t ret = read_all(fd, header, sizeof(header));
  if (ret != ESP_OK) {
    return ret;
  }

  const uint16_t payload_len = ((uint16_t)header[0] << 8) | (uint16_t)header[1];
  message->payload_len = payload_len;
  message->record_type = header[2];
  message->schema_version = header[3];

  if (payload_len > message->payload_capacity) {
    ESP_LOGE(TAG, "incoming payload length %u exceeds buffer capacity %u",
             (unsigned)payload_len, (unsigned)message->payload_capacity);
    return ESP_ERR_INVALID_SIZE;
  }

  ret = read_all(fd, message->payload, payload_len);
  if (ret == ESP_OK) {
    DEBUG_LOGI(TAG, "read record_type=%u schema=%u payload_len=%u",
               (unsigned)message->record_type,
               (unsigned)message->schema_version,
               (unsigned)message->payload_len);
  }
  return ret;
}

void tcp_disconnect(int fd) {
  if (fd < 0) {
    return;
  }
  if (close(fd) < 0) {
    ESP_LOGE(TAG, "socket close failed: %s", strerror(errno));
  }
}

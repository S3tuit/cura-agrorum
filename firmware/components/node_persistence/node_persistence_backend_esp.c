#include "node_persistence_backend.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_crc.h"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "nvs.h"
#include "nvs_flash.h"

#define NODE_PERSISTENCE_NVS_NAMESPACE "cura_lora_v2"
#define NODE_PERSISTENCE_NVS_SAMPLE_ID_KEY "next_sample_id"
#define NODE_PERSISTENCE_NVS_MESSAGE_ID_KEY "next_message_id"

static const char *counter_key(node_persistence_nvs_counter_t counter) {
  switch (counter) {
  case NODE_PERSISTENCE_NVS_COUNTER_SAMPLE:
    return NODE_PERSISTENCE_NVS_SAMPLE_ID_KEY;
  case NODE_PERSISTENCE_NVS_COUNTER_MESSAGE:
    return NODE_PERSISTENCE_NVS_MESSAGE_ID_KEY;
  default:
    return NULL;
  }
}

static int32_t backend_nvs_init(void) {
  return (int32_t)nvs_flash_init_partition(
      NODE_PERSISTENCE_NVS_PARTITION_LABEL);
}

static int32_t backend_nvs_open(node_persistence_nvs_handle_t *out_handle) {
  if (out_handle == NULL) {
    return (int32_t)ESP_ERR_INVALID_ARG;
  }
  nvs_handle_t handle = 0;
  const esp_err_t result = nvs_open_from_partition(
      NODE_PERSISTENCE_NVS_PARTITION_LABEL, NODE_PERSISTENCE_NVS_NAMESPACE,
      NVS_READWRITE, &handle);
  if (result == ESP_OK) {
    *out_handle = (node_persistence_nvs_handle_t)handle;
  }
  return (int32_t)result;
}

static int32_t backend_nvs_get_counter(node_persistence_nvs_handle_t handle,
                                       node_persistence_nvs_counter_t counter,
                                       uint32_t *out_value, bool *out_found) {
  const char *key = counter_key(counter);
  if (key == NULL || out_value == NULL || out_found == NULL) {
    return (int32_t)ESP_ERR_INVALID_ARG;
  }
  const esp_err_t result = nvs_get_u32((nvs_handle_t)handle, key, out_value);
  if (result == ESP_ERR_NVS_NOT_FOUND) {
    *out_found = false;
    *out_value = 0U;
    return (int32_t)ESP_OK;
  }
  if (result == ESP_OK) {
    *out_found = true;
  }
  return (int32_t)result;
}

static int32_t backend_nvs_set_counter(node_persistence_nvs_handle_t handle,
                                       node_persistence_nvs_counter_t counter,
                                       uint32_t value) {
  const char *key = counter_key(counter);
  return key == NULL ? (int32_t)ESP_ERR_INVALID_ARG
                     : (int32_t)nvs_set_u32((nvs_handle_t)handle, key, value);
}

static int32_t backend_nvs_commit(node_persistence_nvs_handle_t handle) {
  return (int32_t)nvs_commit((nvs_handle_t)handle);
}

static void backend_nvs_close(node_persistence_nvs_handle_t handle) {
  nvs_close((nvs_handle_t)handle);
}

static int32_t backend_littlefs_mount(void) {
  const esp_vfs_littlefs_conf_t configuration = {
      .base_path = NODE_PERSISTENCE_MOUNT_PATH,
      .partition_label = NODE_PERSISTENCE_PARTITION_LABEL,
      .partition = NULL,
      .format_if_mount_failed = false,
      .read_only = false,
      .dont_mount = false,
      .grow_on_mount = false,
  };
  return (int32_t)esp_vfs_littlefs_register(&configuration);
}

static int32_t backend_path_stat(const char *path, bool *out_exists,
                                 uint64_t *out_size) {
  if (path == NULL || out_exists == NULL || out_size == NULL) {
    return EINVAL;
  }
  struct stat status;
  if (stat(path, &status) == 0) {
    if (status.st_size < 0) {
      return EIO;
    }
    *out_exists = true;
    *out_size = (uint64_t)status.st_size;
    return 0;
  }
  const int saved_errno = errno;
  if (saved_errno == ENOENT) {
    *out_exists = false;
    *out_size = 0U;
    return 0;
  }
  return saved_errno;
}

static int32_t backend_file_open(const char *path, bool create, bool truncate,
                                 node_persistence_file_handle_t *out_handle) {
  if (path == NULL || out_handle == NULL || (truncate && !create)) {
    return EINVAL;
  }
  int flags = O_RDWR;
  if (create) {
    flags |= O_CREAT;
  }
  if (truncate) {
    flags |= O_TRUNC;
  }
  const int descriptor = open(path, flags, 0600);
  if (descriptor < 0) {
    return errno;
  }
  *out_handle = (node_persistence_file_handle_t)descriptor;
  return 0;
}

static int32_t backend_file_size(node_persistence_file_handle_t handle,
                                 uint64_t *out_size) {
  if (out_size == NULL) {
    return EINVAL;
  }
  struct stat status;
  if (fstat((int)handle, &status) != 0) {
    return errno;
  }
  if (status.st_size < 0) {
    return EIO;
  }
  *out_size = (uint64_t)status.st_size;
  return 0;
}

static int32_t seek_to(node_persistence_file_handle_t handle, uint64_t offset) {
  const off_t requested = (off_t)offset;
  if (requested < 0 || (uint64_t)requested != offset) {
    return EOVERFLOW;
  }
  if (lseek((int)handle, requested, SEEK_SET) < 0) {
    return errno;
  }
  return 0;
}

static int32_t backend_file_read_exact(node_persistence_file_handle_t handle,
                                       uint64_t offset, uint8_t *output,
                                       size_t length) {
  if (output == NULL && length != 0U) {
    return EINVAL;
  }
  int32_t result = seek_to(handle, offset);
  if (result != 0) {
    return result;
  }
  size_t completed = 0U;
  while (completed < length) {
    const ssize_t count =
        read((int)handle, output + completed, length - completed);
    if (count > 0) {
      completed += (size_t)count;
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    return count == 0 ? EIO : errno;
  }
  return 0;
}

static int32_t backend_file_write_exact(node_persistence_file_handle_t handle,
                                        uint64_t offset, const uint8_t *input,
                                        size_t length) {
  if (input == NULL && length != 0U) {
    return EINVAL;
  }
  int32_t result = seek_to(handle, offset);
  if (result != 0) {
    return result;
  }
  size_t completed = 0U;
  while (completed < length) {
    const ssize_t count =
        write((int)handle, input + completed, length - completed);
    if (count > 0) {
      completed += (size_t)count;
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    return count == 0 ? EIO : errno;
  }
  return 0;
}

static int32_t backend_file_sync(node_persistence_file_handle_t handle) {
  return fsync((int)handle) == 0 ? 0 : errno;
}

static int32_t backend_file_truncate(node_persistence_file_handle_t handle,
                                     uint64_t size) {
  const off_t requested = (off_t)size;
  if (requested < 0 || (uint64_t)requested != size) {
    return EOVERFLOW;
  }
  return ftruncate((int)handle, requested) == 0 ? 0 : errno;
}

static int32_t backend_file_close(node_persistence_file_handle_t handle) {
  return close((int)handle) == 0 ? 0 : errno;
}

static int32_t backend_path_remove(const char *path) {
  if (path == NULL) {
    return EINVAL;
  }
  return unlink(path) == 0 ? 0 : errno;
}

static int32_t backend_path_rename(const char *source,
                                   const char *destination) {
  if (source == NULL || destination == NULL) {
    return EINVAL;
  }
  return rename(source, destination) == 0 ? 0 : errno;
}

static uint32_t backend_crc32_iso_hdlc(const uint8_t *input, size_t length) {
  return esp_crc32_le(0U, input, (uint32_t)length);
}

static const node_persistence_backend_t BACKEND = {
    .nvs_init = backend_nvs_init,
    .nvs_open = backend_nvs_open,
    .nvs_get_counter = backend_nvs_get_counter,
    .nvs_set_counter = backend_nvs_set_counter,
    .nvs_commit = backend_nvs_commit,
    .nvs_close = backend_nvs_close,
    .littlefs_mount = backend_littlefs_mount,
    .path_stat = backend_path_stat,
    .file_open = backend_file_open,
    .file_size = backend_file_size,
    .file_read_exact = backend_file_read_exact,
    .file_write_exact = backend_file_write_exact,
    .file_sync = backend_file_sync,
    .file_truncate = backend_file_truncate,
    .file_close = backend_file_close,
    .path_remove = backend_path_remove,
    .path_rename = backend_path_rename,
    .crc32_iso_hdlc = backend_crc32_iso_hdlc,
};

const node_persistence_backend_t *node_persistence_backend(void) {
  return &BACKEND;
}

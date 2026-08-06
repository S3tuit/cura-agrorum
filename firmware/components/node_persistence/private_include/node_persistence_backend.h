#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef intptr_t node_persistence_file_handle_t;
typedef uintptr_t node_persistence_nvs_handle_t;

#define NODE_PERSISTENCE_INVALID_FILE_HANDLE                                   \
  ((node_persistence_file_handle_t) - 1)
#ifndef NODE_PERSISTENCE_MOUNT_PATH
#define NODE_PERSISTENCE_MOUNT_PATH "/cura"
#endif

#ifndef NODE_PERSISTENCE_PARTITION_LABEL
#define NODE_PERSISTENCE_PARTITION_LABEL "storage"
#endif

#ifndef NODE_PERSISTENCE_NVS_PARTITION_LABEL
#define NODE_PERSISTENCE_NVS_PARTITION_LABEL "nvs"
#endif

/*
 * Private backend used by the production ESP-IDF adapter and host tests.
 * ESP operations return an exact esp_err_t encoded as int32_t. File operations
 * return zero or a positive errno value and perform exact transfers.
 */
typedef struct {
  int32_t (*nvs_init)(void);
  int32_t (*nvs_open)(node_persistence_nvs_handle_t *out_handle);
  int32_t (*nvs_get_next_sample_id)(node_persistence_nvs_handle_t handle,
                                    uint32_t *out_value, bool *out_found);
  int32_t (*nvs_set_next_sample_id)(node_persistence_nvs_handle_t handle,
                                    uint32_t value);
  int32_t (*nvs_commit)(node_persistence_nvs_handle_t handle);
  void (*nvs_close)(node_persistence_nvs_handle_t handle);

  int32_t (*littlefs_mount)(void);
  int32_t (*path_stat)(const char *path, bool *out_exists, uint64_t *out_size);
  int32_t (*file_open)(const char *path, bool create, bool truncate,
                       node_persistence_file_handle_t *out_handle);
  int32_t (*file_size)(node_persistence_file_handle_t handle,
                       uint64_t *out_size);
  int32_t (*file_read_exact)(node_persistence_file_handle_t handle,
                             uint64_t offset, uint8_t *output, size_t length);
  int32_t (*file_write_exact)(node_persistence_file_handle_t handle,
                              uint64_t offset, const uint8_t *input,
                              size_t length);
  int32_t (*file_sync)(node_persistence_file_handle_t handle);
  int32_t (*file_truncate)(node_persistence_file_handle_t handle,
                           uint64_t size);
  int32_t (*file_close)(node_persistence_file_handle_t handle);
  int32_t (*path_remove)(const char *path);
  int32_t (*path_rename)(const char *source, const char *destination);

  uint32_t (*crc32_iso_hdlc)(const uint8_t *input, size_t length);
} node_persistence_backend_t;

const node_persistence_backend_t *node_persistence_backend(void);

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  FAKE_BACKEND_OP_NONE = 0,
  FAKE_BACKEND_OP_NVS_INIT,
  FAKE_BACKEND_OP_NVS_OPEN,
  FAKE_BACKEND_OP_NVS_GET,
  FAKE_BACKEND_OP_NVS_SET,
  FAKE_BACKEND_OP_NVS_COMMIT,
  FAKE_BACKEND_OP_LITTLEFS_MOUNT,
  FAKE_BACKEND_OP_PATH_STAT,
  FAKE_BACKEND_OP_FILE_OPEN,
  FAKE_BACKEND_OP_FILE_SIZE,
  FAKE_BACKEND_OP_FILE_READ,
  FAKE_BACKEND_OP_FILE_WRITE,
  FAKE_BACKEND_OP_FILE_SYNC,
  FAKE_BACKEND_OP_FILE_TRUNCATE,
  FAKE_BACKEND_OP_FILE_CLOSE,
  FAKE_BACKEND_OP_PATH_REMOVE,
  FAKE_BACKEND_OP_PATH_RENAME,
} fake_backend_operation_t;

typedef struct {
  uint32_t nvs_init;
  uint32_t nvs_open;
  uint32_t nvs_get;
  uint32_t nvs_set;
  uint32_t nvs_commit;
  uint32_t nvs_close;
  uint32_t littlefs_mount;
  uint32_t path_stat;
  uint32_t file_open;
  uint32_t file_size;
  uint32_t file_read;
  uint32_t file_write;
  uint32_t file_sync;
  uint32_t file_truncate;
  uint32_t file_close;
  uint32_t path_remove;
  uint32_t path_rename;
} fake_backend_calls_t;

void fake_backend_reset(void);
void fake_backend_fail_next(fake_backend_operation_t operation, int32_t status);
const fake_backend_calls_t *fake_backend_calls(void);

void fake_backend_seed_next_sample_id(uint32_t value);
bool fake_backend_file_exists(const char *component_path);
uint64_t fake_backend_file_size(const char *component_path);
bool fake_backend_append_bytes(const char *component_path, const uint8_t *bytes,
                               size_t length);
bool fake_backend_write_file(const char *component_path, const uint8_t *bytes,
                             size_t length);
bool fake_backend_read_file(const char *component_path, uint8_t *output,
                            size_t output_size, size_t *out_length);

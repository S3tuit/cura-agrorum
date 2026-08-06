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
  FAKE_BACKEND_OP_NVS_CLOSE,
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

typedef enum {
  FAKE_BACKEND_RESOURCE_ANY = 0,
  FAKE_BACKEND_RESOURCE_NVS,
  FAKE_BACKEND_RESOURCE_LITTLEFS,
  FAKE_BACKEND_RESOURCE_PENDING,
  FAKE_BACKEND_RESOURCE_COMPACT,
  FAKE_BACKEND_RESOURCE_QUARANTINE,
  FAKE_BACKEND_RESOURCE_DIAGNOSTIC,
  FAKE_BACKEND_RESOURCE_DELIVERY,
} fake_backend_resource_t;

typedef struct {
  fake_backend_operation_t operation;
  fake_backend_resource_t resource;
  uint64_t offset;
  size_t requested_length;
  size_t completed_length;
  int32_t status;
} fake_backend_trace_entry_t;

void fake_backend_reset(void);
void fake_backend_simulate_restart(void);

/* Fails the selected future matching invocation; occurrence is one-based. */
void fake_backend_fail_on(fake_backend_operation_t operation,
                          fake_backend_resource_t resource, uint32_t occurrence,
                          int32_t status);

/* Writes a prefix, then returns status from the selected FILE_WRITE call. */
void fake_backend_partial_write_on(fake_backend_resource_t resource,
                                   uint32_t occurrence, size_t completed_length,
                                   int32_t status);

void fake_backend_clear_trace(void);
size_t fake_backend_trace_count(void);
const fake_backend_trace_entry_t *fake_backend_trace_at(size_t index);
size_t fake_backend_count(fake_backend_operation_t operation,
                          fake_backend_resource_t resource);
bool fake_backend_littlefs_is_mounted(void);

void fake_backend_seed_next_sample_id(uint32_t value);
bool fake_backend_next_sample_id_found(void);
uint32_t fake_backend_next_sample_id(void);

bool fake_backend_file_exists(const char *component_path);
uint64_t fake_backend_file_size(const char *component_path);
bool fake_backend_append_bytes(const char *component_path, const uint8_t *bytes,
                               size_t length);
bool fake_backend_write_file(const char *component_path, const uint8_t *bytes,
                             size_t length);
bool fake_backend_read_file(const char *component_path, uint8_t *output,
                            size_t output_size, size_t *out_length);

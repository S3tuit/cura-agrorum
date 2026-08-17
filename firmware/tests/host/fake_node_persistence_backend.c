#include "fake_node_persistence_backend.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "node_persistence_backend.h"

#define TEST_ROOT_SIZE 160U
#define TEST_PATH_SIZE 256U
#define TRACE_CAPACITY 16384U
#define HANDLE_CAPACITY 16U

typedef struct {
  bool active;
  int descriptor;
  fake_backend_resource_t resource;
} fake_handle_t;

typedef struct {
  bool active;
  fake_backend_operation_t operation;
  fake_backend_resource_t resource;
  uint32_t remaining_matches;
  int32_t status;
  bool partial_write;
  size_t completed_length;
} fake_fault_t;

typedef struct {
  char root[TEST_ROOT_SIZE];
  bool initialized;
  bool littlefs_mounted;
  bool nvs_found[NODE_PERSISTENCE_NVS_COUNTER_COUNT];
  uint32_t nvs_committed[NODE_PERSISTENCE_NVS_COUNTER_COUNT];
  uint32_t nvs_staged[NODE_PERSISTENCE_NVS_COUNTER_COUNT];
  bool nvs_staged_dirty[NODE_PERSISTENCE_NVS_COUNTER_COUNT];
  fake_handle_t handles[HANDLE_CAPACITY];
  fake_fault_t fault;
  fake_backend_trace_entry_t trace[TRACE_CAPACITY];
  size_t trace_count;
} fake_backend_state_t;

static fake_backend_state_t s_fake;

static void initialize_root(void) {
  if (s_fake.initialized) {
    return;
  }
  const int count = snprintf(s_fake.root, sizeof(s_fake.root),
                             "/tmp/cura-node-persistence-%ld", (long)getpid());
  if (count < 0 || (size_t)count >= sizeof(s_fake.root)) {
    abort();
  }
  if (mkdir(s_fake.root, 0700) != 0 && errno != EEXIST) {
    abort();
  }
  s_fake.initialized = true;
}

static bool translate_path(const char *component_path,
                           char output[TEST_PATH_SIZE]) {
  initialize_root();
  if (component_path == NULL) {
    return false;
  }
  const size_t mount_length = strlen(NODE_PERSISTENCE_MOUNT_PATH);
  if (strncmp(component_path, NODE_PERSISTENCE_MOUNT_PATH, mount_length) != 0 ||
      (component_path[mount_length] != '\0' &&
       component_path[mount_length] != '/')) {
    return false;
  }
  const int count = snprintf(output, TEST_PATH_SIZE, "%s%s", s_fake.root,
                             component_path + mount_length);
  return count >= 0 && (size_t)count < TEST_PATH_SIZE;
}

static fake_backend_resource_t resource_from_path(const char *path) {
  if (path == NULL) {
    return FAKE_BACKEND_RESOURCE_ANY;
  }
  if (strcmp(path, NODE_PERSISTENCE_MOUNT_PATH "/pending.log") == 0) {
    return FAKE_BACKEND_RESOURCE_PENDING;
  }
  if (strcmp(path, NODE_PERSISTENCE_MOUNT_PATH "/pending.compact") == 0) {
    return FAKE_BACKEND_RESOURCE_COMPACT;
  }
  if (strcmp(path, NODE_PERSISTENCE_MOUNT_PATH "/quarantine.log") == 0) {
    return FAKE_BACKEND_RESOURCE_QUARANTINE;
  }
  if (strcmp(path, NODE_PERSISTENCE_MOUNT_PATH "/diagnostic.log") == 0) {
    return FAKE_BACKEND_RESOURCE_DIAGNOSTIC;
  }
  if (strcmp(path, NODE_PERSISTENCE_MOUNT_PATH "/delivery.log") == 0) {
    return FAKE_BACKEND_RESOURCE_DELIVERY;
  }
  return FAKE_BACKEND_RESOURCE_LITTLEFS;
}

static void trace_call(fake_backend_operation_t operation,
                       fake_backend_resource_t resource, uint64_t offset,
                       size_t requested_length, size_t completed_length,
                       int32_t status) {
  if (s_fake.trace_count >= TRACE_CAPACITY) {
    abort();
  }
  s_fake.trace[s_fake.trace_count++] = (fake_backend_trace_entry_t){
      .operation = operation,
      .resource = resource,
      .offset = offset,
      .requested_length = requested_length,
      .completed_length = completed_length,
      .status = status,
  };
}

static bool fault_matches(fake_backend_operation_t operation,
                          fake_backend_resource_t resource) {
  if (!s_fake.fault.active || s_fake.fault.operation != operation ||
      (s_fake.fault.resource != FAKE_BACKEND_RESOURCE_ANY &&
       s_fake.fault.resource != resource)) {
    return false;
  }
  if (--s_fake.fault.remaining_matches != 0U) {
    return false;
  }
  return true;
}

static int32_t consume_failure(fake_backend_operation_t operation,
                               fake_backend_resource_t resource,
                               bool *out_partial, size_t *out_completed) {
  if (!fault_matches(operation, resource)) {
    return 0;
  }
  const int32_t status = s_fake.fault.status;
  *out_partial = s_fake.fault.partial_write;
  *out_completed = s_fake.fault.completed_length;
  s_fake.fault.active = false;
  return status;
}

static void register_handle(int descriptor, fake_backend_resource_t resource) {
  for (size_t index = 0U; index < HANDLE_CAPACITY; ++index) {
    if (!s_fake.handles[index].active) {
      s_fake.handles[index] = (fake_handle_t){
          .active = true,
          .descriptor = descriptor,
          .resource = resource,
      };
      return;
    }
  }
  abort();
}

static fake_backend_resource_t handle_resource(int descriptor) {
  for (size_t index = 0U; index < HANDLE_CAPACITY; ++index) {
    if (s_fake.handles[index].active &&
        s_fake.handles[index].descriptor == descriptor) {
      return s_fake.handles[index].resource;
    }
  }
  return FAKE_BACKEND_RESOURCE_LITTLEFS;
}

static void unregister_handle(int descriptor) {
  for (size_t index = 0U; index < HANDLE_CAPACITY; ++index) {
    if (s_fake.handles[index].active &&
        s_fake.handles[index].descriptor == descriptor) {
      s_fake.handles[index].active = false;
      return;
    }
  }
}

static int32_t fake_nvs_init(void) {
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure =
      consume_failure(FAKE_BACKEND_OP_NVS_INIT, FAKE_BACKEND_RESOURCE_NVS,
                      &partial, &completed);
  (void)partial;
  trace_call(FAKE_BACKEND_OP_NVS_INIT, FAKE_BACKEND_RESOURCE_NVS, 0U, 0U,
             completed, failure);
  return failure;
}

static int32_t fake_nvs_open(node_persistence_nvs_handle_t *out_handle) {
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure =
      consume_failure(FAKE_BACKEND_OP_NVS_OPEN, FAKE_BACKEND_RESOURCE_NVS,
                      &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_NVS_OPEN, FAKE_BACKEND_RESOURCE_NVS, 0U, 0U,
               completed, failure);
    return failure;
  }
  if (out_handle == NULL) {
    trace_call(FAKE_BACKEND_OP_NVS_OPEN, FAKE_BACKEND_RESOURCE_NVS, 0U, 0U, 0U,
               EINVAL);
    return EINVAL;
  }
  *out_handle = (node_persistence_nvs_handle_t)1U;
  trace_call(FAKE_BACKEND_OP_NVS_OPEN, FAKE_BACKEND_RESOURCE_NVS, 0U, 0U, 0U,
             0);
  return 0;
}

static int32_t fake_nvs_get(node_persistence_nvs_handle_t handle,
                            node_persistence_nvs_counter_t counter,
                            uint32_t *out_value, bool *out_found) {
  (void)handle;
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(
      FAKE_BACKEND_OP_NVS_GET, FAKE_BACKEND_RESOURCE_NVS, &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_NVS_GET, FAKE_BACKEND_RESOURCE_NVS, 0U, 0U,
               completed, failure);
    return failure;
  }
  if (counter >= NODE_PERSISTENCE_NVS_COUNTER_COUNT || out_value == NULL ||
      out_found == NULL) {
    trace_call(FAKE_BACKEND_OP_NVS_GET, FAKE_BACKEND_RESOURCE_NVS, 0U, 0U, 0U,
               EINVAL);
    return EINVAL;
  }
  *out_found = s_fake.nvs_found[counter];
  *out_value = s_fake.nvs_committed[counter];
  trace_call(FAKE_BACKEND_OP_NVS_GET, FAKE_BACKEND_RESOURCE_NVS, 0U, 0U, 0U, 0);
  return 0;
}

static int32_t fake_nvs_set(node_persistence_nvs_handle_t handle,
                            node_persistence_nvs_counter_t counter,
                            uint32_t value) {
  (void)handle;
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(
      FAKE_BACKEND_OP_NVS_SET, FAKE_BACKEND_RESOURCE_NVS, &partial, &completed);
  (void)partial;
  if (counter >= NODE_PERSISTENCE_NVS_COUNTER_COUNT) {
    return EINVAL;
  }
  if (failure == 0) {
    s_fake.nvs_staged[counter] = value;
    s_fake.nvs_staged_dirty[counter] = true;
  }
  trace_call(FAKE_BACKEND_OP_NVS_SET, FAKE_BACKEND_RESOURCE_NVS, 0U,
             sizeof(value), failure == 0 ? sizeof(value) : completed, failure);
  return failure;
}

static int32_t fake_nvs_commit(node_persistence_nvs_handle_t handle) {
  (void)handle;
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure =
      consume_failure(FAKE_BACKEND_OP_NVS_COMMIT, FAKE_BACKEND_RESOURCE_NVS,
                      &partial, &completed);
  if (failure == 0 || partial) {
    for (size_t index = 0U; index < NODE_PERSISTENCE_NVS_COUNTER_COUNT;
         ++index) {
      if (s_fake.nvs_staged_dirty[index]) {
        s_fake.nvs_committed[index] = s_fake.nvs_staged[index];
        s_fake.nvs_found[index] = true;
        s_fake.nvs_staged_dirty[index] = false;
      }
    }
  }
  trace_call(FAKE_BACKEND_OP_NVS_COMMIT, FAKE_BACKEND_RESOURCE_NVS, 0U, 0U,
             completed, failure);
  return failure;
}

static void fake_nvs_close(node_persistence_nvs_handle_t handle) {
  (void)handle;
  trace_call(FAKE_BACKEND_OP_NVS_CLOSE, FAKE_BACKEND_RESOURCE_NVS, 0U, 0U, 0U,
             0);
}

static int32_t fake_littlefs_mount(void) {
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure =
      consume_failure(FAKE_BACKEND_OP_LITTLEFS_MOUNT,
                      FAKE_BACKEND_RESOURCE_LITTLEFS, &partial, &completed);
  (void)partial;
  if (failure == 0) {
    s_fake.littlefs_mounted = true;
  }
  trace_call(FAKE_BACKEND_OP_LITTLEFS_MOUNT, FAKE_BACKEND_RESOURCE_LITTLEFS, 0U,
             0U, completed, failure);
  return failure;
}

static int32_t fake_path_stat(const char *path, bool *out_exists,
                              uint64_t *out_size) {
  const fake_backend_resource_t resource = resource_from_path(path);
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_PATH_STAT, resource,
                                          &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_PATH_STAT, resource, 0U, 0U, completed, failure);
    return failure;
  }
  if (out_exists == NULL || out_size == NULL) {
    trace_call(FAKE_BACKEND_OP_PATH_STAT, resource, 0U, 0U, 0U, EINVAL);
    return EINVAL;
  }
  char translated[TEST_PATH_SIZE];
  if (!translate_path(path, translated)) {
    trace_call(FAKE_BACKEND_OP_PATH_STAT, resource, 0U, 0U, 0U, EINVAL);
    return EINVAL;
  }
  struct stat status;
  if (stat(translated, &status) == 0) {
    if (status.st_size < 0) {
      trace_call(FAKE_BACKEND_OP_PATH_STAT, resource, 0U, 0U, 0U, EIO);
      return EIO;
    }
    *out_exists = true;
    *out_size = (uint64_t)status.st_size;
    trace_call(FAKE_BACKEND_OP_PATH_STAT, resource, 0U, 0U, 0U, 0);
    return 0;
  }
  if (errno == ENOENT) {
    *out_exists = false;
    *out_size = 0U;
    trace_call(FAKE_BACKEND_OP_PATH_STAT, resource, 0U, 0U, 0U, 0);
    return 0;
  }
  const int32_t result = errno;
  trace_call(FAKE_BACKEND_OP_PATH_STAT, resource, 0U, 0U, 0U, result);
  return result;
}

static int32_t fake_file_open(const char *path, bool create, bool truncate,
                              node_persistence_file_handle_t *out_handle) {
  const fake_backend_resource_t resource = resource_from_path(path);
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_OPEN, resource,
                                          &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_FILE_OPEN, resource, 0U, 0U, completed, failure);
    return failure;
  }
  if (out_handle == NULL || (truncate && !create)) {
    trace_call(FAKE_BACKEND_OP_FILE_OPEN, resource, 0U, 0U, 0U, EINVAL);
    return EINVAL;
  }
  char translated[TEST_PATH_SIZE];
  if (!translate_path(path, translated)) {
    trace_call(FAKE_BACKEND_OP_FILE_OPEN, resource, 0U, 0U, 0U, EINVAL);
    return EINVAL;
  }
  int flags = O_RDWR;
  if (create) {
    flags |= O_CREAT;
  }
  if (truncate) {
    flags |= O_TRUNC;
  }
  const int descriptor = open(translated, flags, 0600);
  if (descriptor < 0) {
    const int32_t result = errno;
    trace_call(FAKE_BACKEND_OP_FILE_OPEN, resource, 0U, 0U, 0U, result);
    return result;
  }
  register_handle(descriptor, resource);
  *out_handle = (node_persistence_file_handle_t)descriptor;
  trace_call(FAKE_BACKEND_OP_FILE_OPEN, resource, 0U, 0U, 0U, 0);
  return 0;
}

static int32_t fake_file_size(node_persistence_file_handle_t handle,
                              uint64_t *out_size) {
  const fake_backend_resource_t resource = handle_resource((int)handle);
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_SIZE, resource,
                                          &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_FILE_SIZE, resource, 0U, 0U, completed, failure);
    return failure;
  }
  if (out_size == NULL) {
    trace_call(FAKE_BACKEND_OP_FILE_SIZE, resource, 0U, 0U, 0U, EINVAL);
    return EINVAL;
  }
  struct stat status;
  if (fstat((int)handle, &status) != 0 || status.st_size < 0) {
    const int32_t result = errno == 0 ? EIO : errno;
    trace_call(FAKE_BACKEND_OP_FILE_SIZE, resource, 0U, 0U, 0U, result);
    return result;
  }
  *out_size = (uint64_t)status.st_size;
  trace_call(FAKE_BACKEND_OP_FILE_SIZE, resource, 0U, 0U, 0U, 0);
  return 0;
}

static int32_t seek_to(node_persistence_file_handle_t handle, uint64_t offset) {
  const off_t target = (off_t)offset;
  if (target < 0 || (uint64_t)target != offset) {
    return EOVERFLOW;
  }
  return lseek((int)handle, target, SEEK_SET) >= 0 ? 0 : errno;
}

static int32_t fake_file_read_exact(node_persistence_file_handle_t handle,
                                    uint64_t offset, uint8_t *output,
                                    size_t length) {
  const fake_backend_resource_t resource = handle_resource((int)handle);
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_READ, resource,
                                          &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_FILE_READ, resource, offset, length, completed,
               failure);
    return failure;
  }
  if (output == NULL && length != 0U) {
    trace_call(FAKE_BACKEND_OP_FILE_READ, resource, offset, length, 0U, EINVAL);
    return EINVAL;
  }
  int32_t result = seek_to(handle, offset);
  if (result != 0) {
    trace_call(FAKE_BACKEND_OP_FILE_READ, resource, offset, length, 0U, result);
    return result;
  }
  completed = 0U;
  while (completed < length) {
    const ssize_t count =
        read((int)handle, output + completed, length - completed);
    if (count > 0) {
      completed += (size_t)count;
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      result = count == 0 ? EIO : errno;
      trace_call(FAKE_BACKEND_OP_FILE_READ, resource, offset, length, completed,
                 result);
      return result;
    }
  }
  trace_call(FAKE_BACKEND_OP_FILE_READ, resource, offset, length, completed, 0);
  return 0;
}

static int32_t write_prefix(node_persistence_file_handle_t handle,
                            const uint8_t *input, size_t length,
                            size_t *out_completed) {
  size_t completed = 0U;
  while (completed < length) {
    const ssize_t count =
        write((int)handle, input + completed, length - completed);
    if (count > 0) {
      completed += (size_t)count;
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      *out_completed = completed;
      return count == 0 ? EIO : errno;
    }
  }
  *out_completed = completed;
  return 0;
}

static int32_t fake_file_write_exact(node_persistence_file_handle_t handle,
                                     uint64_t offset, const uint8_t *input,
                                     size_t length) {
  const fake_backend_resource_t resource = handle_resource((int)handle);
  bool partial = false;
  size_t fault_completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_WRITE, resource,
                                          &partial, &fault_completed);
  if (input == NULL && length != 0U) {
    trace_call(FAKE_BACKEND_OP_FILE_WRITE, resource, offset, length, 0U,
               EINVAL);
    return EINVAL;
  }
  int32_t result = seek_to(handle, offset);
  if (result != 0) {
    trace_call(FAKE_BACKEND_OP_FILE_WRITE, resource, offset, length, 0U,
               result);
    return result;
  }
  if (failure != 0 && !partial) {
    trace_call(FAKE_BACKEND_OP_FILE_WRITE, resource, offset, length, 0U,
               failure);
    return failure;
  }

  size_t requested = length;
  if (failure != 0 && fault_completed < requested) {
    requested = fault_completed;
  }
  size_t completed = 0U;
  result = write_prefix(handle, input, requested, &completed);
  if (result != 0) {
    trace_call(FAKE_BACKEND_OP_FILE_WRITE, resource, offset, length, completed,
               result);
    return result;
  }
  const int32_t returned_status = failure != 0 ? failure : 0;
  trace_call(FAKE_BACKEND_OP_FILE_WRITE, resource, offset, length, completed,
             returned_status);
  return returned_status;
}

static int32_t fake_file_sync(node_persistence_file_handle_t handle) {
  const fake_backend_resource_t resource = handle_resource((int)handle);
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_SYNC, resource,
                                          &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_FILE_SYNC, resource, 0U, 0U, completed, failure);
    return failure;
  }
  const int32_t result = fsync((int)handle) == 0 ? 0 : errno;
  trace_call(FAKE_BACKEND_OP_FILE_SYNC, resource, 0U, 0U, 0U, result);
  return result;
}

static int32_t fake_file_truncate(node_persistence_file_handle_t handle,
                                  uint64_t size) {
  const fake_backend_resource_t resource = handle_resource((int)handle);
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_TRUNCATE,
                                          resource, &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_FILE_TRUNCATE, resource, size, 0U, completed,
               failure);
    return failure;
  }
  const off_t target = (off_t)size;
  const int32_t result =
      target < 0 || (uint64_t)target != size
          ? EOVERFLOW
          : (ftruncate((int)handle, target) == 0 ? 0 : errno);
  trace_call(FAKE_BACKEND_OP_FILE_TRUNCATE, resource, size, 0U, 0U, result);
  return result;
}

static int32_t fake_file_close(node_persistence_file_handle_t handle) {
  const int descriptor = (int)handle;
  const fake_backend_resource_t resource = handle_resource(descriptor);
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_CLOSE, resource,
                                          &partial, &completed);
  (void)partial;
  const int close_result = close(descriptor);
  unregister_handle(descriptor);
  const int32_t result =
      failure != 0 ? failure : (close_result == 0 ? 0 : errno);
  trace_call(FAKE_BACKEND_OP_FILE_CLOSE, resource, 0U, 0U, completed, result);
  return result;
}

static int32_t fake_path_remove(const char *path) {
  const fake_backend_resource_t resource = resource_from_path(path);
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_PATH_REMOVE, resource,
                                          &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_PATH_REMOVE, resource, 0U, 0U, completed,
               failure);
    return failure;
  }
  char translated[TEST_PATH_SIZE];
  const int32_t result = !translate_path(path, translated)
                             ? EINVAL
                             : (unlink(translated) == 0 ? 0 : errno);
  trace_call(FAKE_BACKEND_OP_PATH_REMOVE, resource, 0U, 0U, 0U, result);
  return result;
}

static int32_t fake_path_rename(const char *source, const char *destination) {
  const fake_backend_resource_t resource = resource_from_path(source);
  bool partial = false;
  size_t completed = 0U;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_PATH_RENAME, resource,
                                          &partial, &completed);
  (void)partial;
  if (failure != 0) {
    trace_call(FAKE_BACKEND_OP_PATH_RENAME, resource, 0U, 0U, completed,
               failure);
    return failure;
  }
  char translated_source[TEST_PATH_SIZE];
  char translated_destination[TEST_PATH_SIZE];
  const int32_t result =
      !translate_path(source, translated_source) ||
              !translate_path(destination, translated_destination)
          ? EINVAL
          : (rename(translated_source, translated_destination) == 0 ? 0
                                                                    : errno);
  trace_call(FAKE_BACKEND_OP_PATH_RENAME, resource, 0U, 0U, 0U, result);
  return result;
}

static uint32_t fake_crc32_iso_hdlc(const uint8_t *input, size_t length) {
  uint32_t crc = UINT32_MAX;
  for (size_t index = 0U; index < length; ++index) {
    crc ^= input[index];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc =
          (crc >> 1U) ^ (UINT32_C(0xedb88320) & (uint32_t)-(int32_t)(crc & 1U));
    }
  }
  return ~crc;
}

static const node_persistence_backend_t BACKEND = {
    .nvs_init = fake_nvs_init,
    .nvs_open = fake_nvs_open,
    .nvs_get_counter = fake_nvs_get,
    .nvs_set_counter = fake_nvs_set,
    .nvs_commit = fake_nvs_commit,
    .nvs_close = fake_nvs_close,
    .littlefs_mount = fake_littlefs_mount,
    .path_stat = fake_path_stat,
    .file_open = fake_file_open,
    .file_size = fake_file_size,
    .file_read_exact = fake_file_read_exact,
    .file_write_exact = fake_file_write_exact,
    .file_sync = fake_file_sync,
    .file_truncate = fake_file_truncate,
    .file_close = fake_file_close,
    .path_remove = fake_path_remove,
    .path_rename = fake_path_rename,
    .crc32_iso_hdlc = fake_crc32_iso_hdlc,
};

const node_persistence_backend_t *node_persistence_backend(void) {
  return &BACKEND;
}

static void remove_test_file(const char *component_path) {
  char translated[TEST_PATH_SIZE];
  if (translate_path(component_path, translated)) {
    (void)unlink(translated);
  }
}

static void close_test_handles(void) {
  for (size_t index = 0U; index < HANDLE_CAPACITY; ++index) {
    if (s_fake.handles[index].active) {
      (void)close(s_fake.handles[index].descriptor);
      s_fake.handles[index].active = false;
    }
  }
}

void fake_backend_reset(void) {
  initialize_root();
  close_test_handles();
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/pending.log");
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/pending.compact");
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/quarantine.log");
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/diagnostic.log");
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/delivery.log");
  s_fake.littlefs_mounted = false;
  memset(s_fake.nvs_found, 0, sizeof(s_fake.nvs_found));
  memset(s_fake.nvs_committed, 0, sizeof(s_fake.nvs_committed));
  memset(s_fake.nvs_staged, 0, sizeof(s_fake.nvs_staged));
  memset(s_fake.nvs_staged_dirty, 0, sizeof(s_fake.nvs_staged_dirty));
  memset(&s_fake.fault, 0, sizeof(s_fake.fault));
  memset(s_fake.trace, 0, sizeof(s_fake.trace));
  s_fake.trace_count = 0U;
}

void fake_backend_simulate_restart(void) {
  close_test_handles();
  s_fake.littlefs_mounted = false;
  memcpy(s_fake.nvs_staged, s_fake.nvs_committed, sizeof(s_fake.nvs_staged));
  memset(s_fake.nvs_staged_dirty, 0, sizeof(s_fake.nvs_staged_dirty));
  memset(&s_fake.fault, 0, sizeof(s_fake.fault));
}

void fake_backend_fail_on(fake_backend_operation_t operation,
                          fake_backend_resource_t resource, uint32_t occurrence,
                          int32_t status) {
  if (operation == FAKE_BACKEND_OP_NONE || occurrence == 0U || status == 0) {
    abort();
  }
  s_fake.fault = (fake_fault_t){
      .active = true,
      .operation = operation,
      .resource = resource,
      .remaining_matches = occurrence,
      .status = status,
  };
}

void fake_backend_ambiguous_nvs_commit_on(uint32_t occurrence, int32_t status) {
  fake_backend_fail_on(FAKE_BACKEND_OP_NVS_COMMIT, FAKE_BACKEND_RESOURCE_NVS,
                       occurrence, status);
  s_fake.fault.partial_write = true;
}

void fake_backend_partial_write_on(fake_backend_resource_t resource,
                                   uint32_t occurrence, size_t completed_length,
                                   int32_t status) {
  fake_backend_fail_on(FAKE_BACKEND_OP_FILE_WRITE, resource, occurrence,
                       status);
  s_fake.fault.partial_write = true;
  s_fake.fault.completed_length = completed_length;
}

void fake_backend_clear_trace(void) {
  memset(s_fake.trace, 0, sizeof(s_fake.trace));
  s_fake.trace_count = 0U;
}

size_t fake_backend_trace_count(void) { return s_fake.trace_count; }

const fake_backend_trace_entry_t *fake_backend_trace_at(size_t index) {
  return index < s_fake.trace_count ? &s_fake.trace[index] : NULL;
}

size_t fake_backend_count(fake_backend_operation_t operation,
                          fake_backend_resource_t resource) {
  size_t count = 0U;
  for (size_t index = 0U; index < s_fake.trace_count; ++index) {
    if (s_fake.trace[index].operation == operation &&
        (resource == FAKE_BACKEND_RESOURCE_ANY ||
         s_fake.trace[index].resource == resource)) {
      ++count;
    }
  }
  return count;
}

bool fake_backend_littlefs_is_mounted(void) { return s_fake.littlefs_mounted; }

void fake_backend_seed_next_sample_id(uint32_t value) {
  s_fake.nvs_found[NODE_PERSISTENCE_NVS_COUNTER_SAMPLE] = true;
  s_fake.nvs_committed[NODE_PERSISTENCE_NVS_COUNTER_SAMPLE] = value;
  s_fake.nvs_staged[NODE_PERSISTENCE_NVS_COUNTER_SAMPLE] = value;
  s_fake.nvs_staged_dirty[NODE_PERSISTENCE_NVS_COUNTER_SAMPLE] = false;
}

bool fake_backend_next_sample_id_found(void) {
  return s_fake.nvs_found[NODE_PERSISTENCE_NVS_COUNTER_SAMPLE];
}

uint32_t fake_backend_next_sample_id(void) {
  return s_fake.nvs_committed[NODE_PERSISTENCE_NVS_COUNTER_SAMPLE];
}

void fake_backend_seed_next_message_id(uint32_t value) {
  s_fake.nvs_found[NODE_PERSISTENCE_NVS_COUNTER_MESSAGE] = true;
  s_fake.nvs_committed[NODE_PERSISTENCE_NVS_COUNTER_MESSAGE] = value;
  s_fake.nvs_staged[NODE_PERSISTENCE_NVS_COUNTER_MESSAGE] = value;
  s_fake.nvs_staged_dirty[NODE_PERSISTENCE_NVS_COUNTER_MESSAGE] = false;
}

bool fake_backend_next_message_id_found(void) {
  return s_fake.nvs_found[NODE_PERSISTENCE_NVS_COUNTER_MESSAGE];
}

uint32_t fake_backend_next_message_id(void) {
  return s_fake.nvs_committed[NODE_PERSISTENCE_NVS_COUNTER_MESSAGE];
}

bool fake_backend_file_exists(const char *component_path) {
  char translated[TEST_PATH_SIZE];
  return translate_path(component_path, translated) &&
         access(translated, F_OK) == 0;
}

uint64_t fake_backend_file_size(const char *component_path) {
  char translated[TEST_PATH_SIZE];
  struct stat status;
  if (!translate_path(component_path, translated) ||
      stat(translated, &status) != 0 || status.st_size < 0) {
    return UINT64_MAX;
  }
  return (uint64_t)status.st_size;
}

static bool raw_write(int descriptor, const uint8_t *bytes, size_t length) {
  size_t completed = 0U;
  while (completed < length) {
    const ssize_t count =
        write(descriptor, bytes + completed, length - completed);
    if (count > 0) {
      completed += (size_t)count;
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      return false;
    }
  }
  return true;
}

bool fake_backend_append_bytes(const char *component_path, const uint8_t *bytes,
                               size_t length) {
  char translated[TEST_PATH_SIZE];
  if (!translate_path(component_path, translated) ||
      (bytes == NULL && length != 0U)) {
    return false;
  }
  const int descriptor = open(translated, O_WRONLY | O_CREAT | O_APPEND, 0600);
  if (descriptor < 0) {
    return false;
  }
  const bool success = raw_write(descriptor, bytes, length);
  return close(descriptor) == 0 && success;
}

bool fake_backend_write_file(const char *component_path, const uint8_t *bytes,
                             size_t length) {
  char translated[TEST_PATH_SIZE];
  if (!translate_path(component_path, translated) ||
      (bytes == NULL && length != 0U)) {
    return false;
  }
  const int descriptor = open(translated, O_WRONLY | O_CREAT | O_TRUNC, 0600);
  if (descriptor < 0) {
    return false;
  }
  const bool success = raw_write(descriptor, bytes, length);
  return close(descriptor) == 0 && success;
}

bool fake_backend_read_file(const char *component_path, uint8_t *output,
                            size_t output_size, size_t *out_length) {
  if (out_length == NULL || (output == NULL && output_size != 0U)) {
    return false;
  }
  char translated[TEST_PATH_SIZE];
  if (!translate_path(component_path, translated)) {
    return false;
  }
  const int descriptor = open(translated, O_RDONLY);
  if (descriptor < 0) {
    return false;
  }
  struct stat status;
  if (fstat(descriptor, &status) != 0 || status.st_size < 0 ||
      (uint64_t)status.st_size > output_size) {
    (void)close(descriptor);
    return false;
  }
  const size_t length = (size_t)status.st_size;
  size_t completed = 0U;
  while (completed < length) {
    const ssize_t count =
        read(descriptor, output + completed, length - completed);
    if (count > 0) {
      completed += (size_t)count;
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      (void)close(descriptor);
      return false;
    }
  }
  *out_length = length;
  return close(descriptor) == 0;
}

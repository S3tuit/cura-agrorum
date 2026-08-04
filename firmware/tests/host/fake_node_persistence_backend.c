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

typedef struct {
  char root[TEST_ROOT_SIZE];
  bool initialized;
  bool nvs_found;
  uint32_t nvs_committed;
  uint32_t nvs_staged;
  fake_backend_operation_t failure_operation;
  int32_t failure_status;
  fake_backend_calls_t calls;
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
  if (strncmp(component_path, NODE_PERSISTENCE_MOUNT_PATH, mount_length) != 0) {
    return false;
  }
  const int count = snprintf(output, TEST_PATH_SIZE, "%s%s", s_fake.root,
                             component_path + mount_length);
  return count >= 0 && (size_t)count < TEST_PATH_SIZE;
}

static int32_t consume_failure(fake_backend_operation_t operation) {
  if (s_fake.failure_operation != operation) {
    return 0;
  }
  s_fake.failure_operation = FAKE_BACKEND_OP_NONE;
  return s_fake.failure_status;
}

static int32_t fake_nvs_init(void) {
  ++s_fake.calls.nvs_init;
  return consume_failure(FAKE_BACKEND_OP_NVS_INIT);
}

static int32_t fake_nvs_open(node_persistence_nvs_handle_t *out_handle) {
  ++s_fake.calls.nvs_open;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_NVS_OPEN);
  if (failure != 0) {
    return failure;
  }
  if (out_handle == NULL) {
    return EINVAL;
  }
  *out_handle = (node_persistence_nvs_handle_t)1U;
  return 0;
}

static int32_t fake_nvs_get(node_persistence_nvs_handle_t handle,
                            uint32_t *out_value, bool *out_found) {
  (void)handle;
  ++s_fake.calls.nvs_get;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_NVS_GET);
  if (failure != 0) {
    return failure;
  }
  if (out_value == NULL || out_found == NULL) {
    return EINVAL;
  }
  *out_found = s_fake.nvs_found;
  *out_value = s_fake.nvs_committed;
  return 0;
}

static int32_t fake_nvs_set(node_persistence_nvs_handle_t handle,
                            uint32_t value) {
  (void)handle;
  ++s_fake.calls.nvs_set;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_NVS_SET);
  if (failure != 0) {
    return failure;
  }
  s_fake.nvs_staged = value;
  return 0;
}

static int32_t fake_nvs_commit(node_persistence_nvs_handle_t handle) {
  (void)handle;
  ++s_fake.calls.nvs_commit;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_NVS_COMMIT);
  if (failure != 0) {
    return failure;
  }
  s_fake.nvs_committed = s_fake.nvs_staged;
  s_fake.nvs_found = true;
  return 0;
}

static void fake_nvs_close(node_persistence_nvs_handle_t handle) {
  (void)handle;
  ++s_fake.calls.nvs_close;
}

static int32_t fake_littlefs_mount(void) {
  ++s_fake.calls.littlefs_mount;
  return consume_failure(FAKE_BACKEND_OP_LITTLEFS_MOUNT);
}

static int32_t fake_path_stat(const char *path, bool *out_exists,
                              uint64_t *out_size) {
  ++s_fake.calls.path_stat;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_PATH_STAT);
  if (failure != 0) {
    return failure;
  }
  if (out_exists == NULL || out_size == NULL) {
    return EINVAL;
  }
  char translated[TEST_PATH_SIZE];
  if (!translate_path(path, translated)) {
    return EINVAL;
  }
  struct stat status;
  if (stat(translated, &status) == 0) {
    if (status.st_size < 0) {
      return EIO;
    }
    *out_exists = true;
    *out_size = (uint64_t)status.st_size;
    return 0;
  }
  if (errno == ENOENT) {
    *out_exists = false;
    *out_size = 0U;
    return 0;
  }
  return errno;
}

static int32_t fake_file_open(const char *path, bool create, bool truncate,
                              node_persistence_file_handle_t *out_handle) {
  ++s_fake.calls.file_open;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_OPEN);
  if (failure != 0) {
    return failure;
  }
  if (out_handle == NULL || (truncate && !create)) {
    return EINVAL;
  }
  char translated[TEST_PATH_SIZE];
  if (!translate_path(path, translated)) {
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
    return errno;
  }
  *out_handle = (node_persistence_file_handle_t)descriptor;
  return 0;
}

static int32_t fake_file_size(node_persistence_file_handle_t handle,
                              uint64_t *out_size) {
  ++s_fake.calls.file_size;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_SIZE);
  if (failure != 0) {
    return failure;
  }
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
  const off_t target = (off_t)offset;
  if (target < 0 || (uint64_t)target != offset) {
    return EOVERFLOW;
  }
  return lseek((int)handle, target, SEEK_SET) >= 0 ? 0 : errno;
}

static int32_t fake_file_read_exact(node_persistence_file_handle_t handle,
                                    uint64_t offset, uint8_t *output,
                                    size_t length) {
  ++s_fake.calls.file_read;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_READ);
  if (failure != 0) {
    return failure;
  }
  if (output == NULL && length != 0U) {
    return EINVAL;
  }
  int32_t status = seek_to(handle, offset);
  if (status != 0) {
    return status;
  }
  size_t done = 0U;
  while (done < length) {
    const ssize_t count = read((int)handle, output + done, length - done);
    if (count > 0) {
      done += (size_t)count;
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      return count == 0 ? EIO : errno;
    }
  }
  return 0;
}

static int32_t fake_file_write_exact(node_persistence_file_handle_t handle,
                                     uint64_t offset, const uint8_t *input,
                                     size_t length) {
  ++s_fake.calls.file_write;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_WRITE);
  if (failure != 0) {
    return failure;
  }
  if (input == NULL && length != 0U) {
    return EINVAL;
  }
  int32_t status = seek_to(handle, offset);
  if (status != 0) {
    return status;
  }
  size_t done = 0U;
  while (done < length) {
    const ssize_t count = write((int)handle, input + done, length - done);
    if (count > 0) {
      done += (size_t)count;
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      return count == 0 ? EIO : errno;
    }
  }
  return 0;
}

static int32_t fake_file_sync(node_persistence_file_handle_t handle) {
  ++s_fake.calls.file_sync;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_SYNC);
  if (failure != 0) {
    return failure;
  }
  return fsync((int)handle) == 0 ? 0 : errno;
}

static int32_t fake_file_truncate(node_persistence_file_handle_t handle,
                                  uint64_t size) {
  ++s_fake.calls.file_truncate;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_TRUNCATE);
  if (failure != 0) {
    return failure;
  }
  const off_t target = (off_t)size;
  if (target < 0 || (uint64_t)target != size) {
    return EOVERFLOW;
  }
  return ftruncate((int)handle, target) == 0 ? 0 : errno;
}

static int32_t fake_file_close(node_persistence_file_handle_t handle) {
  ++s_fake.calls.file_close;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_FILE_CLOSE);
  const int close_result = close((int)handle);
  if (failure != 0) {
    return failure;
  }
  return close_result == 0 ? 0 : errno;
}

static int32_t fake_path_remove(const char *path) {
  ++s_fake.calls.path_remove;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_PATH_REMOVE);
  if (failure != 0) {
    return failure;
  }
  char translated[TEST_PATH_SIZE];
  if (!translate_path(path, translated)) {
    return EINVAL;
  }
  return unlink(translated) == 0 ? 0 : errno;
}

static int32_t fake_path_rename(const char *source, const char *destination) {
  ++s_fake.calls.path_rename;
  const int32_t failure = consume_failure(FAKE_BACKEND_OP_PATH_RENAME);
  if (failure != 0) {
    return failure;
  }
  char translated_source[TEST_PATH_SIZE];
  char translated_destination[TEST_PATH_SIZE];
  if (!translate_path(source, translated_source) ||
      !translate_path(destination, translated_destination)) {
    return EINVAL;
  }
  return rename(translated_source, translated_destination) == 0 ? 0 : errno;
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
    .nvs_get_next_sample_id = fake_nvs_get,
    .nvs_set_next_sample_id = fake_nvs_set,
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

void fake_backend_reset(void) {
  initialize_root();
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/pending.log");
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/pending.compact");
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/quarantine.log");
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/diagnostic.log");
  remove_test_file(NODE_PERSISTENCE_MOUNT_PATH "/delivery.log");
  s_fake.nvs_found = false;
  s_fake.nvs_committed = 0U;
  s_fake.nvs_staged = 0U;
  s_fake.failure_operation = FAKE_BACKEND_OP_NONE;
  s_fake.failure_status = 0;
  memset(&s_fake.calls, 0, sizeof(s_fake.calls));
}

void fake_backend_fail_next(fake_backend_operation_t operation,
                            int32_t status) {
  s_fake.failure_operation = operation;
  s_fake.failure_status = status;
}

const fake_backend_calls_t *fake_backend_calls(void) { return &s_fake.calls; }

void fake_backend_seed_next_sample_id(uint32_t value) {
  s_fake.nvs_found = true;
  s_fake.nvs_committed = value;
  s_fake.nvs_staged = value;
}

bool fake_backend_file_exists(const char *component_path) {
  char translated[TEST_PATH_SIZE];
  if (!translate_path(component_path, translated)) {
    return false;
  }
  return access(translated, F_OK) == 0;
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
  size_t done = 0U;
  while (done < length) {
    const ssize_t count = write(descriptor, bytes + done, length - done);
    if (count > 0) {
      done += (size_t)count;
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      (void)close(descriptor);
      return false;
    }
  }
  return close(descriptor) == 0;
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
  size_t done = 0U;
  while (done < length) {
    const ssize_t count = write(descriptor, bytes + done, length - done);
    if (count > 0) {
      done += (size_t)count;
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      (void)close(descriptor);
      return false;
    }
  }
  return close(descriptor) == 0;
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
  size_t done = 0U;
  while (done < length) {
    const ssize_t count = read(descriptor, output + done, length - done);
    if (count > 0) {
      done += (size_t)count;
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

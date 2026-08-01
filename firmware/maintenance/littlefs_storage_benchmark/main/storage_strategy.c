#include "storage_strategy.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "benchmark_config.h"

#define SAMPLE_FILENAME_LENGTH 12u

static uint32_t load_le32(const uint8_t bytes[4]) {
  return ((uint32_t)bytes[0]) | ((uint32_t)bytes[1] << 8u) |
         ((uint32_t)bytes[2] << 16u) | ((uint32_t)bytes[3] << 24u);
}

static void store_le32(uint8_t bytes[4], uint32_t value) {
  bytes[0] = (uint8_t)value;
  bytes[1] = (uint8_t)(value >> 8u);
  bytes[2] = (uint8_t)(value >> 16u);
  bytes[3] = (uint8_t)(value >> 24u);
}

static esp_err_t build_sample_path(uint32_t sample_id, char *path,
                                   size_t path_capacity) {
  const int written = snprintf(path, path_capacity, "%s/%08" PRIx32 ".rec",
                               BENCHMARK_DIRECTORY_PATH, sample_id);
  if (written < 0 || (size_t)written >= path_capacity) {
    errno = ENAMETOOLONG;
    return ESP_ERR_INVALID_SIZE;
  }
  return ESP_OK;
}

static esp_err_t write_all(int fd, const uint8_t *data, size_t size) {
  size_t written = 0;
  while (written < size) {
    const ssize_t result = write(fd, data + written, size - written);
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result <= 0) {
      return ESP_FAIL;
    }
    written += (size_t)result;
  }
  return ESP_OK;
}

static esp_err_t read_exact_at(int fd, off_t offset, uint8_t *data,
                               size_t size) {
  if (lseek(fd, offset, SEEK_SET) < 0) {
    return ESP_FAIL;
  }

  size_t read_size = 0;
  while (read_size < size) {
    const ssize_t result = read(fd, data + read_size, size - read_size);
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result <= 0) {
      errno = EIO;
      return ESP_FAIL;
    }
    read_size += (size_t)result;
  }
  return ESP_OK;
}

static esp_err_t sync_and_close(int fd, esp_err_t operation_result) {
  int saved_errno = errno;
  if (operation_result == ESP_OK && fsync(fd) != 0) {
    operation_result = ESP_FAIL;
    saved_errno = errno;
  }
  if (close(fd) != 0 && operation_result == ESP_OK) {
    operation_result = ESP_FAIL;
    saved_errno = errno;
  }
  errno = saved_errno;
  return operation_result;
}

static int hex_value(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  return -1;
}

static bool parse_sample_filename(const char *name, uint32_t *sample_id_out) {
  if (strlen(name) != SAMPLE_FILENAME_LENGTH || strcmp(name + 8, ".rec") != 0) {
    return false;
  }

  uint32_t sample_id = 0;
  for (size_t index = 0; index < 8; ++index) {
    const int nibble = hex_value(name[index]);
    if (nibble < 0) {
      return false;
    }
    sample_id = (sample_id << 4u) | (uint32_t)nibble;
  }
  if (sample_id == 0) {
    return false;
  }

  *sample_id_out = sample_id;
  return true;
}

static esp_err_t read_and_validate_file(const char *path, uint32_t sample_id,
                                        size_t record_size,
                                        uint8_t *record_out) {
  const int fd = open(path, O_RDONLY);
  if (fd < 0) {
    return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
  }

  struct stat info;
  esp_err_t result = ESP_OK;
  if (fstat(fd, &info) != 0) {
    result = ESP_FAIL;
  } else if (info.st_size != (off_t)record_size) {
    errno = EINVAL;
    result = ESP_FAIL;
  }
  if (result == ESP_OK) {
    result = read_exact_at(fd, 0, record_out, record_size);
  }
  int saved_errno = errno;
  if (close(fd) != 0 && result == ESP_OK) {
    saved_errno = errno;
    result = ESP_FAIL;
  }
  if (result == ESP_OK &&
      !storage_strategy_record_is_valid(sample_id, record_out, record_size)) {
    errno = EILSEQ;
    result = ESP_ERR_INVALID_CRC;
  } else if (result != ESP_OK) {
    errno = saved_errno;
  }
  return result;
}

static esp_err_t read_log_tail(const storage_strategy_t *strategy,
                               uint32_t *sample_id_out, uint8_t *record_out) {
  const int fd = open(BENCHMARK_LOG_PATH, O_RDONLY);
  if (fd < 0) {
    return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
  }

  struct stat info;
  esp_err_t result = ESP_OK;
  if (fstat(fd, &info) != 0) {
    result = ESP_FAIL;
  } else if (info.st_size == 0) {
    result = ESP_ERR_NOT_FOUND;
  } else if (info.st_size < 0 ||
             ((size_t)info.st_size % strategy->record_size) != 0) {
    errno = EINVAL;
    result = ESP_FAIL;
  }

  if (result == ESP_OK) {
    result = read_exact_at(fd, info.st_size - (off_t)strategy->record_size,
                           record_out, strategy->record_size);
  }
  int saved_errno = errno;
  if (close(fd) != 0 && result == ESP_OK) {
    saved_errno = errno;
    result = ESP_FAIL;
  }
  errno = saved_errno;

  if (result != ESP_OK) {
    return result;
  }

  const uint32_t sample_id = load_le32(record_out);
  if (!storage_strategy_record_is_valid(sample_id, record_out,
                                        strategy->record_size)) {
    errno = EILSEQ;
    return ESP_ERR_INVALID_CRC;
  }
  *sample_id_out = sample_id;
  return ESP_OK;
}

const char *storage_strategy_name(storage_strategy_kind_t kind) {
  switch (kind) {
  case STORAGE_STRATEGY_SINGLE_LOG:
    return "single_log";
  case STORAGE_STRATEGY_ONE_FILE_PER_READING:
    return "one_file_per_reading";
  default:
    return "unknown";
  }
}

void storage_strategy_make_record(uint32_t sample_id, uint8_t *record,
                                  size_t record_size) {
  store_le32(record, sample_id);
  for (size_t index = sizeof(sample_id); index < record_size; ++index) {
    const uint32_t mixed =
        sample_id * 0x9e3779b1u + (uint32_t)index * 0x45d9f3bu;
    record[index] = (uint8_t)(mixed ^ (mixed >> 8u) ^ (mixed >> 16u));
  }
}

bool storage_strategy_record_is_valid(uint32_t sample_id, const uint8_t *record,
                                      size_t record_size) {
  if (record == NULL || record_size < sizeof(sample_id) ||
      load_le32(record) != sample_id) {
    return false;
  }

  uint8_t expected[BENCHMARK_MAX_RECORD_SIZE];
  if (record_size > sizeof(expected)) {
    return false;
  }
  storage_strategy_make_record(sample_id, expected, record_size);
  return memcmp(record, expected, record_size) == 0;
}

esp_err_t storage_strategy_insert(const storage_strategy_t *strategy,
                                  uint32_t sample_id, const uint8_t *record) {
  if (strategy == NULL || record == NULL || sample_id == 0 ||
      strategy->record_size < sizeof(sample_id) ||
      strategy->record_size > BENCHMARK_MAX_RECORD_SIZE) {
    return ESP_ERR_INVALID_ARG;
  }

  char path[sizeof(BENCHMARK_DIRECTORY_PATH) + SAMPLE_FILENAME_LENGTH + 2u];
  int flags = O_WRONLY | O_CREAT;
  if (strategy->kind == STORAGE_STRATEGY_SINGLE_LOG) {
    strcpy(path, BENCHMARK_LOG_PATH);
    flags |= O_APPEND;
  } else if (strategy->kind == STORAGE_STRATEGY_ONE_FILE_PER_READING) {
    esp_err_t result = build_sample_path(sample_id, path, sizeof(path));
    if (result != ESP_OK) {
      return result;
    }
    flags |= O_EXCL;
  } else {
    return ESP_ERR_INVALID_ARG;
  }

  const int fd = open(path, flags, 0600);
  if (fd < 0) {
    return ESP_FAIL;
  }
  return sync_and_close(fd, write_all(fd, record, strategy->record_size));
}

esp_err_t storage_strategy_delete(const storage_strategy_t *strategy,
                                  uint32_t sample_id) {
  if (strategy == NULL || sample_id == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  if (strategy->kind == STORAGE_STRATEGY_ONE_FILE_PER_READING) {
    char path[sizeof(BENCHMARK_DIRECTORY_PATH) + SAMPLE_FILENAME_LENGTH + 2u];
    esp_err_t result = build_sample_path(sample_id, path, sizeof(path));
    if (result != ESP_OK) {
      return result;
    }
    return unlink(path) == 0 ? ESP_OK : ESP_FAIL;
  }
  if (strategy->kind != STORAGE_STRATEGY_SINGLE_LOG) {
    return ESP_ERR_INVALID_ARG;
  }

  const int fd = open(BENCHMARK_LOG_PATH, O_RDWR);
  if (fd < 0) {
    return ESP_FAIL;
  }

  struct stat info;
  esp_err_t result = ESP_OK;
  if (fstat(fd, &info) != 0) {
    result = ESP_FAIL;
  } else if (info.st_size < (off_t)strategy->record_size ||
             ((size_t)info.st_size % strategy->record_size) != 0) {
    errno = EINVAL;
    result = ESP_FAIL;
  } else if (ftruncate(fd, info.st_size - (off_t)strategy->record_size) != 0) {
    result = ESP_FAIL;
  }
  return sync_and_close(fd, result);
}

esp_err_t
storage_strategy_peek_by_known_head(const storage_strategy_t *strategy,
                                    bool head_valid, uint32_t head_sample_id,
                                    uint8_t *record_out) {
  if (strategy == NULL || record_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!head_valid) {
    return ESP_ERR_NOT_FOUND;
  }

  if (strategy->kind == STORAGE_STRATEGY_SINGLE_LOG) {
    uint32_t found_sample_id = 0;
    esp_err_t result = read_log_tail(strategy, &found_sample_id, record_out);
    if (result == ESP_OK && found_sample_id != head_sample_id) {
      errno = EILSEQ;
      return ESP_ERR_INVALID_STATE;
    }
    return result;
  }
  if (strategy->kind != STORAGE_STRATEGY_ONE_FILE_PER_READING) {
    return ESP_ERR_INVALID_ARG;
  }

  char path[sizeof(BENCHMARK_DIRECTORY_PATH) + SAMPLE_FILENAME_LENGTH + 2u];
  esp_err_t result = build_sample_path(head_sample_id, path, sizeof(path));
  if (result != ESP_OK) {
    return result;
  }
  return read_and_validate_file(path, head_sample_id, strategy->record_size,
                                record_out);
}

esp_err_t storage_strategy_peek_by_scan(const storage_strategy_t *strategy,
                                        uint32_t *sample_id_out,
                                        uint8_t *record_out) {
  if (strategy == NULL || sample_id_out == NULL || record_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (strategy->kind == STORAGE_STRATEGY_SINGLE_LOG) {
    return read_log_tail(strategy, sample_id_out, record_out);
  }
  if (strategy->kind != STORAGE_STRATEGY_ONE_FILE_PER_READING) {
    return ESP_ERR_INVALID_ARG;
  }

  DIR *directory = opendir(BENCHMARK_DIRECTORY_PATH);
  if (directory == NULL) {
    return ESP_FAIL;
  }

  bool found = false;
  uint32_t newest_sample_id = 0;
  errno = 0;
  for (struct dirent *entry = readdir(directory); entry != NULL;
       entry = readdir(directory)) {
    uint32_t sample_id = 0;
    if (parse_sample_filename(entry->d_name, &sample_id) &&
        (!found || sample_id > newest_sample_id)) {
      found = true;
      newest_sample_id = sample_id;
    }
  }
  const int read_errno = errno;
  if (closedir(directory) != 0) {
    return ESP_FAIL;
  }
  if (read_errno != 0) {
    errno = read_errno;
    return ESP_FAIL;
  }
  if (!found) {
    return ESP_ERR_NOT_FOUND;
  }

  char path[sizeof(BENCHMARK_DIRECTORY_PATH) + SAMPLE_FILENAME_LENGTH + 2u];
  esp_err_t result = build_sample_path(newest_sample_id, path, sizeof(path));
  if (result != ESP_OK) {
    return result;
  }
  result = read_and_validate_file(path, newest_sample_id, strategy->record_size,
                                  record_out);
  if (result == ESP_OK) {
    *sample_id_out = newest_sample_id;
  }
  return result;
}

esp_err_t storage_strategy_count(const storage_strategy_t *strategy,
                                 size_t *count_out) {
  if (strategy == NULL || count_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (strategy->kind == STORAGE_STRATEGY_SINGLE_LOG) {
    struct stat info;
    if (stat(BENCHMARK_LOG_PATH, &info) != 0) {
      if (errno == ENOENT) {
        *count_out = 0;
        return ESP_OK;
      }
      return ESP_FAIL;
    }
    if (info.st_size < 0 ||
        ((size_t)info.st_size % strategy->record_size) != 0) {
      errno = EINVAL;
      return ESP_FAIL;
    }
    *count_out = (size_t)info.st_size / strategy->record_size;
    return ESP_OK;
  }
  if (strategy->kind != STORAGE_STRATEGY_ONE_FILE_PER_READING) {
    return ESP_ERR_INVALID_ARG;
  }

  DIR *directory = opendir(BENCHMARK_DIRECTORY_PATH);
  if (directory == NULL) {
    return ESP_FAIL;
  }

  size_t count = 0;
  errno = 0;
  for (struct dirent *entry = readdir(directory); entry != NULL;
       entry = readdir(directory)) {
    uint32_t sample_id = 0;
    if (parse_sample_filename(entry->d_name, &sample_id)) {
      count++;
    }
  }
  const int read_errno = errno;
  if (closedir(directory) != 0) {
    return ESP_FAIL;
  }
  if (read_errno != 0) {
    errno = read_errno;
    return ESP_FAIL;
  }

  *count_out = count;
  return ESP_OK;
}

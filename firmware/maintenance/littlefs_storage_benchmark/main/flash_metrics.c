#include "flash_metrics.h"

#include <string.h>

#include "esp_err.h"

static const esp_partition_t *s_partition;
static bool s_run_enabled;
static flash_phase_t s_phase = FLASH_PHASE_NONE;
static flash_metrics_t s_run_total;
static flash_metrics_t s_campaign_total;
static flash_metrics_t s_phase_metrics[FLASH_PHASE_COUNT];

extern esp_err_t __real_esp_partition_read(const esp_partition_t *partition,
                                           size_t src_offset, void *dst,
                                           size_t size);
extern esp_err_t __real_esp_partition_write(const esp_partition_t *partition,
                                            size_t dst_offset, const void *src,
                                            size_t size);
extern esp_err_t
__real_esp_partition_erase_range(const esp_partition_t *partition,
                                 size_t offset, size_t size);

static bool is_benchmark_partition(const esp_partition_t *partition) {
  return s_partition != NULL && partition != NULL &&
         partition->address == s_partition->address &&
         partition->size == s_partition->size &&
         strcmp(partition->label, s_partition->label) == 0;
}

static void record_read(flash_metrics_t *metrics, size_t size) {
  metrics->read_calls++;
  metrics->read_bytes += size;
}

static void record_write(flash_metrics_t *metrics, size_t size) {
  metrics->write_calls++;
  metrics->write_bytes += size;
}

static void record_erase(flash_metrics_t *metrics, size_t offset, size_t size) {
  metrics->erase_calls++;
  metrics->erase_bytes += size;

  if (size == 0) {
    return;
  }

  const size_t first_sector = offset / BENCHMARK_SECTOR_SIZE;
  const size_t last_sector = (offset + size - 1u) / BENCHMARK_SECTOR_SIZE;
  for (size_t sector = first_sector;
       sector <= last_sector && sector < STORAGE_SECTOR_COUNT; ++sector) {
    metrics->erase_count_by_sector[sector]++;
  }
}

static flash_metrics_t *current_phase_metrics(void) {
  if (s_phase < 0 || s_phase >= FLASH_PHASE_COUNT) {
    return NULL;
  }
  return &s_phase_metrics[s_phase];
}

void flash_metrics_initialize(const esp_partition_t *partition) {
  s_partition = partition;
  s_run_enabled = false;
  s_phase = FLASH_PHASE_NONE;
  memset(&s_run_total, 0, sizeof(s_run_total));
  memset(&s_campaign_total, 0, sizeof(s_campaign_total));
  memset(s_phase_metrics, 0, sizeof(s_phase_metrics));
}

void flash_metrics_begin_run(void) {
  memset(&s_run_total, 0, sizeof(s_run_total));
  memset(s_phase_metrics, 0, sizeof(s_phase_metrics));
  s_phase = FLASH_PHASE_NONE;
  s_run_enabled = true;
}

void flash_metrics_end_run(void) {
  s_phase = FLASH_PHASE_NONE;
  s_run_enabled = false;
}

void flash_metrics_set_phase(flash_phase_t phase) { s_phase = phase; }

const flash_metrics_t *flash_metrics_run_total(void) { return &s_run_total; }

const flash_metrics_t *flash_metrics_campaign_total(void) {
  return &s_campaign_total;
}

const flash_metrics_t *flash_metrics_for_phase(flash_phase_t phase) {
  if (phase < 0 || phase >= FLASH_PHASE_COUNT) {
    return NULL;
  }
  return &s_phase_metrics[phase];
}

const char *flash_metrics_phase_name(flash_phase_t phase) {
  static const char *const names[FLASH_PHASE_COUNT] = {
      [FLASH_PHASE_FORMAT] = "format",
      [FLASH_PHASE_PREPARE] = "prepare",
      [FLASH_PHASE_MOUNT] = "mount",
      [FLASH_PHASE_INSERT] = "insert",
      [FLASH_PHASE_DELETE] = "delete",
      [FLASH_PHASE_PEEK_SCAN] = "peek_scan",
      [FLASH_PHASE_PEEK_KNOWN_HEAD] = "peek_known_head",
      [FLASH_PHASE_VERIFY] = "verify",
      [FLASH_PHASE_UNMOUNT] = "unmount",
  };

  if (phase < 0 || phase >= FLASH_PHASE_COUNT) {
    return "none";
  }
  return names[phase];
}

uint32_t flash_metrics_hottest_sector(const flash_metrics_t *metrics,
                                      uint32_t *sector_out) {
  uint32_t hottest_count = 0;
  uint32_t hottest_sector = 0;

  if (metrics != NULL) {
    for (uint32_t sector = 0; sector < STORAGE_SECTOR_COUNT; ++sector) {
      if (metrics->erase_count_by_sector[sector] > hottest_count) {
        hottest_count = metrics->erase_count_by_sector[sector];
        hottest_sector = sector;
      }
    }
  }

  if (sector_out != NULL) {
    *sector_out = hottest_sector;
  }
  return hottest_count;
}

size_t flash_metrics_erased_sector_count(const flash_metrics_t *metrics) {
  size_t count = 0;
  if (metrics != NULL) {
    for (size_t sector = 0; sector < STORAGE_SECTOR_COUNT; ++sector) {
      count += metrics->erase_count_by_sector[sector] != 0;
    }
  }
  return count;
}

esp_err_t __wrap_esp_partition_read(const esp_partition_t *partition,
                                    size_t src_offset, void *dst, size_t size) {
  esp_err_t result =
      __real_esp_partition_read(partition, src_offset, dst, size);
  if (result == ESP_OK && s_run_enabled && is_benchmark_partition(partition)) {
    record_read(&s_run_total, size);
    record_read(&s_campaign_total, size);
    flash_metrics_t *phase_metrics = current_phase_metrics();
    if (phase_metrics != NULL) {
      record_read(phase_metrics, size);
    }
  }
  return result;
}

esp_err_t __wrap_esp_partition_write(const esp_partition_t *partition,
                                     size_t dst_offset, const void *src,
                                     size_t size) {
  esp_err_t result =
      __real_esp_partition_write(partition, dst_offset, src, size);
  if (result == ESP_OK && s_run_enabled && is_benchmark_partition(partition)) {
    record_write(&s_run_total, size);
    record_write(&s_campaign_total, size);
    flash_metrics_t *phase_metrics = current_phase_metrics();
    if (phase_metrics != NULL) {
      record_write(phase_metrics, size);
    }
  }
  return result;
}

esp_err_t __wrap_esp_partition_erase_range(const esp_partition_t *partition,
                                           size_t offset, size_t size) {
  esp_err_t result = __real_esp_partition_erase_range(partition, offset, size);
  if (result == ESP_OK && s_run_enabled && is_benchmark_partition(partition)) {
    record_erase(&s_run_total, offset, size);
    record_erase(&s_campaign_total, offset, size);
    flash_metrics_t *phase_metrics = current_phase_metrics();
    if (phase_metrics != NULL) {
      record_erase(phase_metrics, offset, size);
    }
  }
  return result;
}

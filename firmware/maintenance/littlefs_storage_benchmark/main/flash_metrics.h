#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "benchmark_config.h"
#include "esp_partition.h"

typedef struct {
  uint64_t read_calls;
  uint64_t read_bytes;
  uint64_t write_calls;
  uint64_t write_bytes;
  uint64_t erase_calls;
  uint64_t erase_bytes;
  uint32_t erase_count_by_sector[STORAGE_SECTOR_COUNT];
} flash_metrics_t;

typedef enum {
  FLASH_PHASE_FORMAT = 0,
  FLASH_PHASE_PREPARE,
  FLASH_PHASE_MOUNT,
  FLASH_PHASE_INSERT,
  FLASH_PHASE_DELETE,
  FLASH_PHASE_PEEK_SCAN,
  FLASH_PHASE_PEEK_KNOWN_HEAD,
  FLASH_PHASE_VERIFY,
  FLASH_PHASE_UNMOUNT,
  FLASH_PHASE_COUNT,
  FLASH_PHASE_NONE = -1,
} flash_phase_t;

/* Starts campaign-level accounting for the given raw-flash partition. */
void flash_metrics_initialize(const esp_partition_t *partition);

/* Clears and enables per-run counters without clearing campaign counters. */
void flash_metrics_begin_run(void);

/* Stops per-run accounting. No storage operation should occur while stopped. */
void flash_metrics_end_run(void);

/* Attributes subsequent successful partition operations to this phase. */
void flash_metrics_set_phase(flash_phase_t phase);

const flash_metrics_t *flash_metrics_run_total(void);
const flash_metrics_t *flash_metrics_campaign_total(void);
const flash_metrics_t *flash_metrics_for_phase(flash_phase_t phase);
const char *flash_metrics_phase_name(flash_phase_t phase);

/* Finds the maximum sector erase count and its lowest-numbered sector. */
uint32_t flash_metrics_hottest_sector(const flash_metrics_t *metrics,
                                      uint32_t *sector_out);

/* Counts sectors with at least one recorded erase. */
size_t flash_metrics_erased_sector_count(const flash_metrics_t *metrics);

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "benchmark_config.h"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "flash_metrics.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "storage_strategy.h"

static const char *TAG = "littlefs_bench";

#ifdef CONFIG_LITTLEFS_USE_MTIME
#error "The benchmark requires CONFIG_LITTLEFS_USE_MTIME to be disabled"
#endif

typedef enum {
  WORKLOAD_ONLINE_50_50 = 0,
  WORKLOAD_GROWING_60_40,
  WORKLOAD_COUNT,
} workload_kind_t;

typedef struct {
  uint32_t values[BENCHMARK_WAKE_CYCLES];
  size_t count;
  uint64_t sum;
} timing_series_t;

typedef struct {
  bool valid;
  uint32_t sample_id;
  size_t pending_count;
} pending_model_t;

static const size_t s_record_sizes[] = {50u, 100u, 150u};
static uint8_t s_growing_delete_schedule[BENCHMARK_WAKE_CYCLES];
static uint8_t s_record[BENCHMARK_MAX_RECORD_SIZE];
static uint8_t s_peek_record[BENCHMARK_MAX_RECORD_SIZE];
static timing_series_t s_insert_times;
static timing_series_t s_delete_times;
static timing_series_t s_wake_times;
static timing_series_t s_peek_operation_times;
static timing_series_t s_peek_wake_times;
static bool s_mounted;

static const char *workload_name(workload_kind_t workload) {
  switch (workload) {
  case WORKLOAD_ONLINE_50_50:
    return "online_50_50";
  case WORKLOAD_GROWING_60_40:
    return "growing_60_40";
  default:
    return "unknown";
  }
}

static uint32_t xorshift32(uint32_t *state) {
  uint32_t value = *state;
  value ^= value << 13u;
  value ^= value >> 17u;
  value ^= value << 5u;
  *state = value;
  return value;
}

static void prepare_growing_schedule(void) {
  for (size_t index = 0; index < BENCHMARK_WAKE_CYCLES; ++index) {
    s_growing_delete_schedule[index] = index < BENCHMARK_GROWING_DELETE_CYCLES;
  }

  uint32_t random_state = 0x43555241u; /* "CURA" */
  for (size_t count = BENCHMARK_WAKE_CYCLES; count > 1; --count) {
    const size_t other = xorshift32(&random_state) % count;
    const uint8_t temporary = s_growing_delete_schedule[count - 1u];
    s_growing_delete_schedule[count - 1u] = s_growing_delete_schedule[other];
    s_growing_delete_schedule[other] = temporary;
  }
}

static bool should_delete_current(workload_kind_t workload, size_t cycle) {
  if (workload == WORKLOAD_ONLINE_50_50) {
    return true;
  }
  return s_growing_delete_schedule[cycle] != 0;
}

static void timing_reset(timing_series_t *series) {
  series->count = 0;
  series->sum = 0;
}

static esp_err_t timing_add(timing_series_t *series, int64_t start_us,
                            int64_t end_us) {
  if (series->count >= BENCHMARK_WAKE_CYCLES || end_us < start_us) {
    return ESP_ERR_INVALID_STATE;
  }
  const uint64_t elapsed = (uint64_t)(end_us - start_us);
  if (elapsed > UINT32_MAX) {
    return ESP_ERR_INVALID_SIZE;
  }
  series->values[series->count++] = (uint32_t)elapsed;
  series->sum += elapsed;
  return ESP_OK;
}

static int compare_u32(const void *left, const void *right) {
  const uint32_t a = *(const uint32_t *)left;
  const uint32_t b = *(const uint32_t *)right;
  return (a > b) - (a < b);
}

static uint32_t percentile(const timing_series_t *series, unsigned percent) {
  if (series->count == 0) {
    return 0;
  }
  size_t rank = (percent * series->count + 99u) / 100u;
  if (rank == 0) {
    rank = 1;
  }
  return series->values[rank - 1u];
}

static void print_timing(const char *run_id, const char *metric,
                         timing_series_t *series, int32_t backlog) {
  if (series->count == 0) {
    return;
  }

  qsort(series->values, series->count, sizeof(series->values[0]), compare_u32);
  printf("BENCH_TIMING,run=%s,metric=%s,backlog=%" PRId32 ",samples=%u,"
         "mean_us=%.2f,p10_us=%" PRIu32 ",p50_us=%" PRIu32 ",p90_us=%" PRIu32
         ",p99_us=%" PRIu32 ",max_us=%" PRIu32 "\n",
         run_id, metric, backlog, (unsigned)series->count,
         (double)series->sum / (double)series->count, percentile(series, 10),
         percentile(series, 50), percentile(series, 90), percentile(series, 99),
         series->values[series->count - 1u]);
}

static void print_flash_metrics(const char *scope, const char *run_id,
                                const char *phase,
                                const flash_metrics_t *metrics) {
  uint32_t hottest_sector = 0;
  const uint32_t hottest_count =
      flash_metrics_hottest_sector(metrics, &hottest_sector);
  printf("BENCH_FLASH,scope=%s,run=%s,phase=%s,read_calls=%" PRIu64
         ",read_bytes=%" PRIu64 ",write_calls=%" PRIu64 ",write_bytes=%" PRIu64
         ",erase_calls=%" PRIu64 ",erase_bytes=%" PRIu64
         ",erased_sectors=%u,hottest_sector=%" PRIu32 ",hottest_erases=%" PRIu32
         ",life_used_pct_at_100k_cycles=%.3f\n",
         scope, run_id, phase, metrics->read_calls, metrics->read_bytes,
         metrics->write_calls, metrics->write_bytes, metrics->erase_calls,
         metrics->erase_bytes,
         (unsigned)flash_metrics_erased_sector_count(metrics), hottest_sector,
         hottest_count, (double)hottest_count / 1000.0);
}

static void print_erase_counts(const char *scope, const char *run_id,
                               const flash_metrics_t *metrics) {
  const size_t counts_per_line = 32u;
  for (size_t first = 0; first < STORAGE_SECTOR_COUNT;
       first += counts_per_line) {
    const size_t end = first + counts_per_line < STORAGE_SECTOR_COUNT
                           ? first + counts_per_line
                           : STORAGE_SECTOR_COUNT;
    printf("BENCH_ERASE_COUNTS,scope=%s,run=%s,first_sector=%u,counts=", scope,
           run_id, (unsigned)first);
    for (size_t sector = first; sector < end; ++sector) {
      printf("%s%" PRIu32, sector == first ? "" : ";",
             metrics->erase_count_by_sector[sector]);
    }
    putchar('\n');
  }
}

static esp_err_t mount_storage(void) {
  if (s_mounted) {
    return ESP_ERR_INVALID_STATE;
  }

  const esp_vfs_littlefs_conf_t configuration = {
      .base_path = BENCHMARK_MOUNT_PATH,
      .partition_label = BENCHMARK_STORAGE_LABEL,
      .format_if_mount_failed = false,
      .dont_mount = false,
  };
  flash_metrics_set_phase(FLASH_PHASE_MOUNT);
  const esp_err_t result = esp_vfs_littlefs_register(&configuration);
  flash_metrics_set_phase(FLASH_PHASE_NONE);
  if (result == ESP_OK) {
    s_mounted = true;
  }
  return result;
}

static esp_err_t unmount_storage(void) {
  if (!s_mounted) {
    return ESP_ERR_INVALID_STATE;
  }

  flash_metrics_set_phase(FLASH_PHASE_UNMOUNT);
  const esp_err_t result = esp_vfs_littlefs_unregister(BENCHMARK_STORAGE_LABEL);
  flash_metrics_set_phase(FLASH_PHASE_NONE);
  if (result == ESP_OK) {
    s_mounted = false;
  }
  return result;
}

static esp_err_t prepare_fresh_filesystem(void) {
  flash_metrics_set_phase(FLASH_PHASE_FORMAT);
  esp_err_t result = esp_littlefs_format(BENCHMARK_STORAGE_LABEL);
  flash_metrics_set_phase(FLASH_PHASE_NONE);
  if (result != ESP_OK) {
    return result;
  }

  result = mount_storage();
  if (result != ESP_OK) {
    return result;
  }

  flash_metrics_set_phase(FLASH_PHASE_PREPARE);
  if (mkdir(BENCHMARK_DIRECTORY_PATH, 0700) != 0) {
    result = ESP_FAIL;
  }
  flash_metrics_set_phase(FLASH_PHASE_NONE);

  const esp_err_t unmount_result = unmount_storage();
  return result != ESP_OK ? result : unmount_result;
}

static esp_err_t verify_pending_count(const storage_strategy_t *strategy,
                                      size_t expected_count) {
  esp_err_t result = mount_storage();
  if (result != ESP_OK) {
    return result;
  }

  size_t actual_count = 0;
  flash_metrics_set_phase(FLASH_PHASE_VERIFY);
  result = storage_strategy_count(strategy, &actual_count);
  flash_metrics_set_phase(FLASH_PHASE_NONE);
  if (result == ESP_OK && actual_count != expected_count) {
    ESP_LOGE(TAG, "pending count mismatch: expected=%u actual=%u",
             (unsigned)expected_count, (unsigned)actual_count);
    result = ESP_ERR_INVALID_STATE;
  }

  const esp_err_t unmount_result = unmount_storage();
  return result != ESP_OK ? result : unmount_result;
}

static esp_err_t run_one_peek(const storage_strategy_t *strategy,
                              pending_model_t model, bool scan,
                              timing_series_t *operation_times,
                              timing_series_t *wake_times) {
  const int64_t wake_start = esp_timer_get_time();
  esp_err_t result = mount_storage();
  if (result != ESP_OK) {
    return result;
  }

  uint32_t found_sample_id = 0;
  const int64_t operation_start = esp_timer_get_time();
  flash_metrics_set_phase(scan ? FLASH_PHASE_PEEK_SCAN
                               : FLASH_PHASE_PEEK_KNOWN_HEAD);
  if (scan) {
    result = storage_strategy_peek_by_scan(strategy, &found_sample_id,
                                           s_peek_record);
  } else {
    result = storage_strategy_peek_by_known_head(
        strategy, model.valid, model.sample_id, s_peek_record);
    found_sample_id = model.sample_id;
  }
  flash_metrics_set_phase(FLASH_PHASE_NONE);
  const int saved_errno = errno;
  const int64_t operation_end = esp_timer_get_time();

  if (model.valid) {
    if (result != ESP_OK || found_sample_id != model.sample_id) {
      ESP_LOGE(TAG,
               "peek mismatch: scan=%d expected=%" PRIu32 " found=%" PRIu32
               " result=%s errno=%d",
               scan, model.sample_id, found_sample_id, esp_err_to_name(result),
               saved_errno);
      if (result == ESP_OK) {
        result = ESP_ERR_INVALID_STATE;
      }
    }
  } else if (result == ESP_ERR_NOT_FOUND) {
    result = ESP_OK;
  } else {
    ESP_LOGE(TAG, "empty peek returned %s errno=%d", esp_err_to_name(result),
             saved_errno);
    if (result == ESP_OK) {
      result = ESP_ERR_INVALID_STATE;
    }
  }

  const esp_err_t unmount_result = unmount_storage();
  const int64_t wake_end = esp_timer_get_time();
  if (result == ESP_OK) {
    result = timing_add(operation_times, operation_start, operation_end);
  }
  if (result == ESP_OK) {
    result = timing_add(wake_times, wake_start, wake_end);
  }
  if (result == ESP_OK) {
    result = unmount_result;
  }
  return result;
}

static esp_err_t run_peek_suite(const char *run_id,
                                const storage_strategy_t *strategy,
                                pending_model_t model) {
  static const bool scan_options[] = {true, false};
  for (size_t option = 0;
       option < sizeof(scan_options) / sizeof(scan_options[0]); ++option) {
    timing_reset(&s_peek_operation_times);
    timing_reset(&s_peek_wake_times);
    for (size_t repetition = 0; repetition < BENCHMARK_PEEK_REPETITIONS;
         ++repetition) {
      esp_err_t result =
          run_one_peek(strategy, model, scan_options[option],
                       &s_peek_operation_times, &s_peek_wake_times);
      if (result != ESP_OK) {
        return result;
      }
      vTaskDelay(1);
    }

    const char *operation_metric =
        scan_options[option] ? "peek_by_scan" : "peek_by_known_head";
    const char *wake_metric = scan_options[option]
                                  ? "peek_by_scan_cold_wake"
                                  : "peek_by_known_head_cold_wake";
    print_timing(run_id, operation_metric, &s_peek_operation_times,
                 (int32_t)model.pending_count);
    print_timing(run_id, wake_metric, &s_peek_wake_times,
                 (int32_t)model.pending_count);
  }
  return ESP_OK;
}

static esp_err_t run_workload(const char *run_id,
                              const storage_strategy_t *strategy,
                              workload_kind_t workload,
                              pending_model_t *model_out) {
  timing_reset(&s_insert_times);
  timing_reset(&s_delete_times);
  timing_reset(&s_wake_times);

  pending_model_t model = {0};
  bool measured_backlog_100 = false;

  for (size_t cycle = 0; cycle < BENCHMARK_WAKE_CYCLES; ++cycle) {
    const uint32_t sample_id = (uint32_t)cycle + 1u;
    const pending_model_t previous_model = model;
    storage_strategy_make_record(sample_id, s_record, strategy->record_size);

    const int64_t wake_start = esp_timer_get_time();
    esp_err_t result = mount_storage();
    if (result != ESP_OK) {
      return result;
    }

    const int64_t insert_start = esp_timer_get_time();
    flash_metrics_set_phase(FLASH_PHASE_INSERT);
    result = storage_strategy_insert(strategy, sample_id, s_record);
    flash_metrics_set_phase(FLASH_PHASE_NONE);
    const int insert_errno = errno;
    const int64_t insert_end = esp_timer_get_time();
    if (result == ESP_OK) {
      result = timing_add(&s_insert_times, insert_start, insert_end);
    }
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "insert failed at cycle=%u: %s errno=%d", (unsigned)cycle,
               esp_err_to_name(result), insert_errno);
    } else {
      model.valid = true;
      model.sample_id = sample_id;
      model.pending_count++;
    }

    if (result == ESP_OK && should_delete_current(workload, cycle)) {
      const int64_t delete_start = esp_timer_get_time();
      flash_metrics_set_phase(FLASH_PHASE_DELETE);
      result = storage_strategy_delete(strategy, sample_id);
      flash_metrics_set_phase(FLASH_PHASE_NONE);
      const int delete_errno = errno;
      const int64_t delete_end = esp_timer_get_time();
      if (result == ESP_OK) {
        result = timing_add(&s_delete_times, delete_start, delete_end);
      }
      if (result != ESP_OK) {
        ESP_LOGE(TAG, "delete failed at cycle=%u: %s errno=%d", (unsigned)cycle,
                 esp_err_to_name(result), delete_errno);
      } else {
        model = previous_model;
      }
    }

    const esp_err_t unmount_result = unmount_storage();
    const int64_t wake_end = esp_timer_get_time();
    if (result == ESP_OK) {
      result = unmount_result;
    }
    if (result == ESP_OK) {
      result = timing_add(&s_wake_times, wake_start, wake_end);
    }
    if (result != ESP_OK) {
      return result;
    }

    if (workload == WORKLOAD_GROWING_60_40 && !measured_backlog_100 &&
        model.pending_count == 100u) {
      result = run_peek_suite(run_id, strategy, model);
      if (result != ESP_OK) {
        return result;
      }
      measured_backlog_100 = true;
    }

    vTaskDelay(1);
  }

  const size_t expected_count =
      workload == WORKLOAD_ONLINE_50_50
          ? 0u
          : BENCHMARK_WAKE_CYCLES - BENCHMARK_GROWING_DELETE_CYCLES;
  if (model.pending_count != expected_count ||
      model.valid != (expected_count != 0)) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = verify_pending_count(strategy, expected_count);
  if (result != ESP_OK) {
    return result;
  }
  result = run_peek_suite(run_id, strategy, model);
  if (result != ESP_OK) {
    return result;
  }

  *model_out = model;
  return ESP_OK;
}

static void print_run_results(const char *run_id) {
  print_timing(run_id, "insert", &s_insert_times, -1);
  print_timing(run_id, "delete", &s_delete_times, -1);
  print_timing(run_id, "storage_wake", &s_wake_times, -1);

  for (flash_phase_t phase = FLASH_PHASE_FORMAT; phase < FLASH_PHASE_COUNT;
       phase = (flash_phase_t)(phase + 1)) {
    print_flash_metrics("phase", run_id, flash_metrics_phase_name(phase),
                        flash_metrics_for_phase(phase));
  }
  print_flash_metrics("run", run_id, "all", flash_metrics_run_total());
  print_erase_counts("run", run_id, flash_metrics_run_total());
}

static esp_err_t run_benchmark(const storage_strategy_t *strategy,
                               workload_kind_t workload) {
  char run_id[96];
  const int written = snprintf(
      run_id, sizeof(run_id), "%s_%u_%s", storage_strategy_name(strategy->kind),
      (unsigned)strategy->record_size, workload_name(workload));
  if (written < 0 || (size_t)written >= sizeof(run_id)) {
    return ESP_ERR_INVALID_SIZE;
  }

  const unsigned deletes = workload == WORKLOAD_ONLINE_50_50
                               ? BENCHMARK_WAKE_CYCLES
                               : BENCHMARK_GROWING_DELETE_CYCLES;
  printf("BENCH_RUN_BEGIN,run=%s,strategy=%s,workload=%s,wakes=%u,"
         "inserts=%u,deletes=%u,record_size=%u\n",
         run_id, storage_strategy_name(strategy->kind), workload_name(workload),
         BENCHMARK_WAKE_CYCLES, BENCHMARK_WAKE_CYCLES, deletes,
         (unsigned)strategy->record_size);
  fflush(stdout);

  flash_metrics_begin_run();
  esp_err_t result = prepare_fresh_filesystem();
  if (result == ESP_OK) {
    pending_model_t final_model;
    result = run_workload(run_id, strategy, workload, &final_model);
  }
  if (s_mounted) {
    const esp_err_t unmount_result = unmount_storage();
    if (result == ESP_OK) {
      result = unmount_result;
    }
  }
  flash_metrics_end_run();

  if (result == ESP_OK) {
    print_run_results(run_id);
    printf("BENCH_RUN_END,run=%s,status=ok\n", run_id);
  } else {
    printf("BENCH_RUN_END,run=%s,status=failed,error=%s,errno=%d\n", run_id,
           esp_err_to_name(result), errno);
  }
  fflush(stdout);
  return result;
}

static esp_err_t validate_storage_partition(const esp_partition_t **out) {
  const esp_partition_t *partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_LITTLEFS,
      BENCHMARK_STORAGE_LABEL);
  if (partition == NULL) {
    ESP_LOGE(TAG, "partition '%s' not found", BENCHMARK_STORAGE_LABEL);
    return ESP_ERR_NOT_FOUND;
  }
  if (partition->size != BENCHMARK_STORAGE_BYTES ||
      partition->erase_size != BENCHMARK_SECTOR_SIZE || partition->encrypted ||
      partition->readonly) {
    ESP_LOGE(TAG,
             "unexpected partition: size=%" PRIu32 " erase=%" PRIu32
             " encrypted=%d readonly=%d",
             partition->size, partition->erase_size, partition->encrypted,
             partition->readonly);
    return ESP_ERR_INVALID_STATE;
  }
  *out = partition;
  return ESP_OK;
}

void app_main(void) {
  ESP_LOGE(TAG, "DESTRUCTIVE LITTLEFS STORAGE BENCHMARK");
  ESP_LOGE(TAG, "partition '%s' will be repeatedly formatted in 5 seconds",
           BENCHMARK_STORAGE_LABEL);
  ESP_LOGI(TAG, "target=esp32c6 storage=%u bytes sectors=%u mtime=disabled",
           BENCHMARK_STORAGE_BYTES, STORAGE_SECTOR_COUNT);
  vTaskDelay(pdMS_TO_TICKS(BENCHMARK_WARNING_DELAY_MS));

  const esp_partition_t *partition = NULL;
  esp_err_t result = validate_storage_partition(&partition);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "benchmark aborted: %s", esp_err_to_name(result));
    return;
  }

  prepare_growing_schedule();
  flash_metrics_initialize(partition);

  for (size_t size_index = 0;
       size_index < sizeof(s_record_sizes) / sizeof(s_record_sizes[0]);
       ++size_index) {
    for (workload_kind_t workload = WORKLOAD_ONLINE_50_50;
         workload < WORKLOAD_COUNT;
         workload = (workload_kind_t)(workload + 1)) {
      for (storage_strategy_kind_t kind = STORAGE_STRATEGY_SINGLE_LOG;
           kind < STORAGE_STRATEGY_COUNT;
           kind = (storage_strategy_kind_t)(kind + 1)) {
        const storage_strategy_t strategy = {
            .kind = kind,
            .record_size = s_record_sizes[size_index],
        };
        result = run_benchmark(&strategy, workload);
        if (result != ESP_OK) {
          ESP_LOGE(TAG, "campaign stopped after failure: %s",
                   esp_err_to_name(result));
          goto finished;
        }
      }
    }
  }

finished:
  print_flash_metrics("campaign", "all", "all", flash_metrics_campaign_total());
  print_erase_counts("campaign", "all", flash_metrics_campaign_total());
  printf("BENCH_CAMPAIGN_END,status=%s\n", result == ESP_OK ? "ok" : "failed");
  fflush(stdout);
  ESP_LOGI(TAG, "benchmark stopped; reset repeats the destructive campaign");
  while (true) {
    vTaskDelay(portMAX_DELAY);
  }
}

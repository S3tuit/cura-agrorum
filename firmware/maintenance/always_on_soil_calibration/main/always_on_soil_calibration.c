#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soil_sensor.h"

static const char *TAG = "always_on_soil_calibration";

#define SAMPLE_CAPACITY 1000
#define SAMPLE_PERIOD_TICKS pdMS_TO_TICKS(1000)

static uint16_t samples[SAMPLE_CAPACITY];
static size_t sample_count;
static uint64_t sum_mv;
static uint16_t sorted_samples[SAMPLE_CAPACITY];

static int compare_u16(const void *left, const void *right) {
  const uint16_t a = *(const uint16_t *)left;
  const uint16_t b = *(const uint16_t *)right;
  return (a > b) - (a < b);
}

static uint64_t median_centimv(const uint16_t *sorted, size_t count) {
  if (count == 0) {
    return 0;
  }

  const size_t middle = count / 2;
  if ((count % 2) == 1) {
    return (uint64_t)sorted[middle] * 100;
  }
  return ((uint64_t)sorted[middle - 1] + sorted[middle]) * 50;
}

static void log_stats(size_t count, uint16_t current_mv, uint64_t sum_mv) {
  memcpy(sorted_samples, samples, count * sizeof(samples[0]));
  qsort(sorted_samples, count, sizeof(sorted_samples[0]), compare_u16);

  const uint64_t mean_centimv = (sum_mv * 100) / count;
  const uint64_t median = median_centimv(sorted_samples, count);

  ESP_LOGI(TAG,
           "sample=%u/%u current_mv=%" PRIu16 " mean_mv=%" PRIu64 ".%02" PRIu64
           " median_mv=%" PRIu64 ".%02" PRIu64,
           (unsigned)count, (unsigned)SAMPLE_CAPACITY, current_mv,
           mean_centimv / 100, mean_centimv % 100, median / 100, median % 100);
}

static void delay_forever(void) {
  ESP_LOGI(TAG, "sample buffer full; delaying forever");
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}

static void delay_until_next_sample(void) {
  ESP_LOGI(TAG, "delaying for 1000 ms before next sample");
  vTaskDelay(SAMPLE_PERIOD_TICKS);
}

void app_main(void) {
  ESP_LOGI(TAG, "always-on soil calibration; collecting %u successful samples",
           (unsigned)SAMPLE_CAPACITY);

  while (1) {
    if (sample_count >= SAMPLE_CAPACITY) {
      delay_forever();
    }

    uint16_t soil_mv = 0;
    esp_err_t ret = soil_sensor_read_mv(CONFIG_CURA_SOIL_ADC_GPIO, &soil_mv);
    if (ret == ESP_OK) {
      samples[sample_count] = soil_mv;
      sample_count++;
      sum_mv += soil_mv;
      log_stats(sample_count, soil_mv, sum_mv);
    } else {
      ESP_LOGE(TAG, "soil sample failed: %s", esp_err_to_name(ret));
    }

    if (sample_count >= SAMPLE_CAPACITY) {
      delay_forever();
    }
    delay_until_next_sample();
  }
}

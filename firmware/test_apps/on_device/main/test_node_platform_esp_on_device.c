#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "node_platform_esp.h"
#include "unity.h"

#define IMMEDIATE_CLOCK_READS 4096U
#define DELAYED_CLOCK_READS 8U
#define CLOCK_DELAY_MS 20U
#define RANDOM_RANGE_SAMPLES 4096U

TEST_CASE("platform clock is nondecreasing", "[node_platform]") {
  const node_platform_ports_t *const ports = node_platform_esp_ports();
  TEST_ASSERT_NOT_NULL(ports);
  TEST_ASSERT_NOT_NULL(ports->clock.monotonic_us);

  uint64_t previous = ports->clock.monotonic_us(ports->clock.context);
  for (size_t index = 0U; index < IMMEDIATE_CLOCK_READS; ++index) {
    const uint64_t current = ports->clock.monotonic_us(ports->clock.context);
    TEST_ASSERT_TRUE(current >= previous);
    previous = current;
  }

  for (size_t index = 0U; index < DELAYED_CLOCK_READS; ++index) {
    vTaskDelay(pdMS_TO_TICKS(CLOCK_DELAY_MS));
    const uint64_t current = ports->clock.monotonic_us(ports->clock.context);
    TEST_ASSERT_TRUE(current >= previous);
    previous = current;
  }
}

static void assert_random_range(const node_randomness_port_t *randomness,
                                uint32_t minimum, uint32_t maximum) {
  for (size_t index = 0U; index < RANDOM_RANGE_SAMPLES; ++index) {
    const uint32_t value = randomness->uniform_u32_inclusive(
        randomness->context, minimum, maximum);
    TEST_ASSERT_TRUE(value >= minimum);
    TEST_ASSERT_TRUE(value <= maximum);
  }
}

TEST_CASE("platform randomness respects inclusive ranges", "[node_platform]") {
  const node_platform_ports_t *const ports = node_platform_esp_ports();
  TEST_ASSERT_NOT_NULL(ports);
  TEST_ASSERT_NOT_NULL(ports->randomness.uniform_u32_inclusive);

  assert_random_range(&ports->randomness, UINT32_C(100000), UINT32_C(500000));
  assert_random_range(&ports->randomness, 0U, 1U);
  assert_random_range(&ports->randomness, 0U, UINT32_MAX);
}

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "fake_node_core_dependencies.h"
#include "node_core.h"

#define CORE_TEST_FIRST_MESSAGE_ID UINT32_C(1001)

#define CORE_TEST_ASSERT(expression)                                           \
  do {                                                                         \
    if (!(expression)) {                                                       \
      fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__,     \
              #expression);                                                    \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define CORE_TEST_ASSERT_EQ_U64(expected, actual)                              \
  do {                                                                         \
    const uint64_t expected_ = (uint64_t)(expected);                           \
    const uint64_t actual_ = (uint64_t)(actual);                               \
    if (expected_ != actual_) {                                                \
      fprintf(stderr, "%s:%d: expected %llu, got %llu: %s\n", __FILE__,        \
              __LINE__, (unsigned long long)expected_,                         \
              (unsigned long long)actual_, #actual);                           \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define CORE_TEST_ASSERT_EQ_SIZE(expected, actual)                             \
  CORE_TEST_ASSERT_EQ_U64((size_t)(expected), (size_t)(actual))

#define CORE_TEST_ASSERT_EQ_U32(expected, actual)                              \
  CORE_TEST_ASSERT_EQ_U64((uint32_t)(expected), (uint32_t)(actual))

extern const node_identity_t CORE_TEST_IDENTITY;

void core_test_setup(node_rtc_record_t *rtc,
                     node_platform_ports_t *out_platform);
void core_test_run(node_rtc_record_t *rtc,
                   const node_platform_ports_t *platform);
cura_lora_v2_reading_t core_test_reading(uint16_t marker);
uint64_t core_test_reading_airtime_us(void);
uint64_t core_test_reading_airtime_charge_us(void);
uint64_t core_test_reading_min_tx_window_us(void);
bool core_test_script_ack(uint32_t message_id, cura_lora_v2_domain_t domain,
                          cura_lora_v2_ack_status_t status,
                          uint64_t set_tx_at_us, uint64_t tx_done_at_us,
                          uint64_t ack_at_us);
bool core_test_decode_transmission(size_t index,
                                   cura_lora_v2_clear_header_t *out_header,
                                   cura_lora_v2_reading_t *out_reading);
bool core_test_has_diagnostic(curag_error_domain_t domain,
                              curag_error_code_t code);
bool core_test_cleanup_is_complete(void);

bool node_core_test_initialization(const char *name);
bool node_core_test_delivery(const char *name);
bool node_core_test_transitions(const char *name);
bool node_core_test_finalization(const char *name);

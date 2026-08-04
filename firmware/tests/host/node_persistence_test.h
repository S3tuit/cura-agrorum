#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "fake_node_persistence_backend.h"
#include "node_common.h"
#include "node_persistence.h"
#include "node_persistence_backend.h"
#include "node_persistence_record.h"
#include "protocol_v2_lora_schema_generated.h"

#define TEST_PENDING_PATH NODE_PERSISTENCE_MOUNT_PATH "/pending.log"
#define TEST_COMPACT_PATH NODE_PERSISTENCE_MOUNT_PATH "/pending.compact"
#define TEST_QUARANTINE_PATH NODE_PERSISTENCE_MOUNT_PATH "/quarantine.log"
#define TEST_DIAGNOSTIC_PATH NODE_PERSISTENCE_MOUNT_PATH "/diagnostic.log"
#define TEST_DELIVERY_PATH NODE_PERSISTENCE_MOUNT_PATH "/delivery.log"

#define TEST_SNAPSHOT_CAPACITY 4096U

#define TEST_ASSERT(expression)                                                \
  do {                                                                         \
    if (!(expression)) {                                                       \
      fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__,     \
              #expression);                                                    \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define TEST_ASSERT_EQ_U32(expected, actual)                                   \
  do {                                                                         \
    const uint32_t test_expected_ = (uint32_t)(expected);                      \
    const uint32_t test_actual_ = (uint32_t)(actual);                          \
    if (test_expected_ != test_actual_) {                                      \
      fprintf(stderr, "%s:%d: expected 0x%08lx, got 0x%08lx: %s\n", __FILE__,  \
              __LINE__, (unsigned long)test_expected_,                         \
              (unsigned long)test_actual_, #actual);                           \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define TEST_ASSERT_EQ_SIZE(expected, actual)                                  \
  do {                                                                         \
    const size_t test_expected_ = (size_t)(expected);                          \
    const size_t test_actual_ = (size_t)(actual);                              \
    if (test_expected_ != test_actual_) {                                      \
      fprintf(stderr, "%s:%d: expected %zu, got %zu: %s\n", __FILE__,          \
              __LINE__, test_expected_, test_actual_, #actual);                \
      return false;                                                            \
    }                                                                          \
  } while (0)

typedef bool (*node_persistence_test_function_t)(void);

typedef struct {
  const char *name;
  node_persistence_test_function_t function;
} node_persistence_test_case_t;

typedef struct {
  const char *name;
  const node_persistence_test_case_t *cases;
  size_t count;
} node_persistence_test_group_t;

typedef struct {
  uint8_t bytes[TEST_SNAPSHOT_CAPACITY];
  size_t length;
} node_persistence_test_snapshot_t;

extern const node_persistence_test_group_t NODE_PERSISTENCE_NVS_TEST_GROUP;
extern const node_persistence_test_group_t NODE_PERSISTENCE_RECORD_TEST_GROUP;
extern const node_persistence_test_group_t NODE_PERSISTENCE_RECOVERY_TEST_GROUP;
extern const node_persistence_test_group_t NODE_PERSISTENCE_FAULT_TEST_GROUP;
extern const node_persistence_test_group_t
    NODE_PERSISTENCE_RETENTION_TEST_GROUP;

void node_persistence_test_reset_all(void);
void node_persistence_test_restart(void);
cura_lora_v2_reading_t node_persistence_test_make_reading(uint16_t marker);
cura_lora_v2_reading_t node_persistence_test_make_boundary_reading(void);
bool node_persistence_test_readings_equal(const cura_lora_v2_reading_t *left,
                                          const cura_lora_v2_reading_t *right);
bool node_persistence_test_assert_diag(
    const diagn_context_t *diag, curag_operation_t operation,
    node_persistence_resource_t resource, node_persistence_stage_t stage,
    node_persistence_backend_status_kind_t kind, int32_t status);
bool node_persistence_test_snapshot(
    const char *path, node_persistence_test_snapshot_t *out_snapshot);
bool node_persistence_test_snapshots_equal(
    const node_persistence_test_snapshot_t *left,
    const node_persistence_test_snapshot_t *right);
bool node_persistence_test_encode_reading_record(
    uint8_t record_type, uint32_t sample_id,
    const cura_lora_v2_reading_t *reading,
    uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE], size_t *out_length);
void node_persistence_test_recalculate_crc(uint8_t *record,
                                           size_t record_length);
bool node_persistence_test_pending_ids(uint32_t *output, size_t capacity,
                                       size_t *out_count);

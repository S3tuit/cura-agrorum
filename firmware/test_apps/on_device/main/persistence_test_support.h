#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "node_persistence.h"
#include "node_persistence_record.h"

#define HWTEST_NVS_PARTITION "nvs_test"
#define HWTEST_LITTLEFS_PARTITION "storage_test"

#define HWTEST_PENDING_PATH NODE_PERSISTENCE_MOUNT_PATH "/pending.log"
#define HWTEST_COMPACT_PATH NODE_PERSISTENCE_MOUNT_PATH "/pending.compact"
#define HWTEST_QUARANTINE_PATH NODE_PERSISTENCE_MOUNT_PATH "/quarantine.log"
#define HWTEST_DIAGNOSTIC_PATH NODE_PERSISTENCE_MOUNT_PATH "/diagnostic.log"
#define HWTEST_DELIVERY_PATH NODE_PERSISTENCE_MOUNT_PATH "/delivery.log"

typedef struct {
  size_t length;
  uint8_t bytes[NODE_PERSISTENCE_RECORD_MAX_SIZE + 128U];
} hwtest_snapshot_t;

void hwtest_erase_state(void);
void hwtest_finish_case(void);
void hwtest_simulate_component_restart(void);
void hwtest_mount_inspector(void);

cura_lora_v2_reading_t hwtest_make_reading(uint16_t marker);
void hwtest_assert_reading_equal(const cura_lora_v2_reading_t *expected,
                                 const cura_lora_v2_reading_t *actual);

void hwtest_seed_next_sample_id(uint32_t value);
uint32_t hwtest_read_next_sample_id(void);
void hwtest_seed_next_message_id(uint32_t value);
uint32_t hwtest_read_next_message_id(void);

void hwtest_snapshot(const char *path, hwtest_snapshot_t *out_snapshot);
void hwtest_assert_snapshot_equal(const hwtest_snapshot_t *expected,
                                  const hwtest_snapshot_t *actual);
bool hwtest_path_exists(const char *path);
void hwtest_append_raw(const char *path, const uint8_t *bytes, size_t length);
void hwtest_replace_raw(const char *path, const uint8_t *bytes, size_t length);

size_t
hwtest_encode_reading_record(uint8_t record_type, uint32_t sample_id,
                             const cura_lora_v2_reading_t *reading,
                             uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE]);
size_t hwtest_encode_diagnostic_record(
    const node_diagnostic_event_t *event,
    uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE]);
size_t
hwtest_encode_delivery_record(const node_delivery_event_t *event,
                              uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE]);
void hwtest_recalculate_record_crc(uint8_t *record, size_t record_length);

node_diagnostic_event_t hwtest_make_diagnostic(diagn_context_t *context,
                                               uint8_t marker);
node_delivery_event_t hwtest_make_delivery_started(uint32_t cycle_sample_id,
                                                   uint32_t sample_id);
node_delivery_event_t hwtest_make_delivery_finished(uint32_t cycle_sample_id,
                                                    uint32_t sample_id);

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
  STORAGE_STRATEGY_SINGLE_LOG = 0,
  STORAGE_STRATEGY_ONE_FILE_PER_READING,
  STORAGE_STRATEGY_COUNT,
} storage_strategy_kind_t;

typedef struct {
  storage_strategy_kind_t kind;
  size_t record_size;
} storage_strategy_t;

const char *storage_strategy_name(storage_strategy_kind_t kind);

/* Creates deterministic, non-erased-looking bytes for a sample. */
void storage_strategy_make_record(uint32_t sample_id, uint8_t *record,
                                  size_t record_size);

/* Appends the record to pending.log or creates one sample-named file. */
esp_err_t storage_strategy_insert(const storage_strategy_t *strategy,
                                  uint32_t sample_id, const uint8_t *record);

/* Removes the current head by truncation or unlinking its sample-named file. */
esp_err_t storage_strategy_delete(const storage_strategy_t *strategy,
                                  uint32_t sample_id);

/* Reads the newest record using the supplied head, modelling valid RTC state.
 */
esp_err_t
storage_strategy_peek_by_known_head(const storage_strategy_t *strategy,
                                    bool head_valid, uint32_t head_sample_id,
                                    uint8_t *record_out);

/* Finds and reads the newest record without a head, modelling recovery. */
esp_err_t storage_strategy_peek_by_scan(const storage_strategy_t *strategy,
                                        uint32_t *sample_id_out,
                                        uint8_t *record_out);

/* Counts structurally valid pending records for end-of-run verification. */
esp_err_t storage_strategy_count(const storage_strategy_t *strategy,
                                 size_t *count_out);

/* Validates both the encoded sample ID and deterministic filler. */
bool storage_strategy_record_is_valid(uint32_t sample_id, const uint8_t *record,
                                      size_t record_size);

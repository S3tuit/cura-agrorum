/*
 * Build from the repository root:
 *   component=firmware/components/protocol_v2_lora
 *   clang -std=c11 -g -O1 -fsanitize=fuzzer,address,undefined \
 *     -fno-omit-frame-pointer -I"$component/include" \
 *     protocol/protocol-v2-lora/tests/c/fuzz_codec.c \
 *     "$component/protocol_v2_lora_schema_generated.c" -o /tmp/fuzz_codec
 *
 * Run with the ignored, persistent local corpus:
 *   corpus=protocol/protocol-v2-lora/.fuzz-corpus/codec
 *   mkdir -p "$corpus"
 *   /tmp/fuzz_codec "$corpus" \
 *     -max_len=64 -use_value_profile=1
 * libFuzzer selects a random seed by default and prints it. Pass
 * -seed=<reported-seed> to reproduce an earlier run.
 *
 * The byte-oriented protocol_fuzz_one_input() is engine-neutral. The
 * LLVMFuzzerTestOneInput() wrapper is the only libFuzzer-specific API.
 */

#include "protocol_v2_lora_schema_generated.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define FUZZ_CANARY UINT8_C(0xa5)
#define FUZZ_EXTRA_CAPACITY 8u
#define FUZZ_OPERATION_COUNT 7u

typedef struct {
  const uint8_t *data;
  size_t size;
  size_t offset;
} fuzz_reader_t;

typedef enum {
  FUZZ_DECODE_HEADER = 0,
  FUZZ_DECODE_READING = 1,
  FUZZ_DECODE_ACK = 2,
  FUZZ_ENCODE_HEADER = 3,
  FUZZ_ENCODE_READING = 4,
  FUZZ_ENCODE_ACK = 5,
  FUZZ_BUILD_NONCE = 6,
} fuzz_operation_t;

static void require_condition(bool condition) {
  if (!condition) {
    abort();
  }
}

static bool bytes_are(const uint8_t *bytes, size_t size, uint8_t expected) {
  for (size_t index = 0; index < size; index++) {
    if (bytes[index] != expected) {
      return false;
    }
  }
  return true;
}

static bool headers_equal(const cura_lora_v2_clear_header_t *left,
                          const cura_lora_v2_clear_header_t *right) {
  return left->control == right->control && left->domain == right->domain &&
         memcmp(left->node_id, right->node_id, sizeof(left->node_id)) == 0 &&
         left->sample_id == right->sample_id;
}

static bool readings_equal(const cura_lora_v2_reading_t *left,
                           const cura_lora_v2_reading_t *right) {
  return left->run_ms == right->run_ms && left->soil_0_mv == right->soil_0_mv &&
         left->soil_1_mv == right->soil_1_mv &&
         left->soil_temp_0_centi_c == right->soil_temp_0_centi_c &&
         left->soil_temp_1_centi_c == right->soil_temp_1_centi_c &&
         left->enclosure_centi_c == right->enclosure_centi_c &&
         left->enclosure_pressure_pa == right->enclosure_pressure_pa &&
         left->enclosure_humidity_centi_pct ==
             right->enclosure_humidity_centi_pct &&
         left->reset_reason == right->reset_reason &&
         left->previous_current_tx_attempts ==
             right->previous_current_tx_attempts &&
         left->previous_awake_ms == right->previous_awake_ms &&
         left->previous_current_delivery_ms ==
             right->previous_current_delivery_ms &&
         left->previous_cycle_tx_attempts ==
             right->previous_cycle_tx_attempts &&
         left->previous_cycle_accepted_readings ==
             right->previous_cycle_accepted_readings &&
         left->flags == right->flags;
}

static uint8_t reader_take_u8(fuzz_reader_t *reader) {
  uint8_t value = 0;
  if (reader->offset < reader->size) {
    value = reader->data[reader->offset];
  }
  reader->offset++;
  return value;
}

static uint16_t reader_take_u16(fuzz_reader_t *reader) {
  const uint16_t low = reader_take_u8(reader);
  const uint16_t high = reader_take_u8(reader);
  return low | (uint16_t)(high << 8);
}

static uint32_t reader_take_u32(fuzz_reader_t *reader) {
  const uint32_t byte_0 = reader_take_u8(reader);
  const uint32_t byte_1 = reader_take_u8(reader);
  const uint32_t byte_2 = reader_take_u8(reader);
  const uint32_t byte_3 = reader_take_u8(reader);
  return byte_0 | (byte_1 << 8) | (byte_2 << 16) | (byte_3 << 24);
}

static int16_t reader_take_i16(fuzz_reader_t *reader) {
  const uint16_t raw = reader_take_u16(reader);
  if (raw <= (uint16_t)INT16_MAX) {
    return (int16_t)raw;
  }
  return (int16_t)(-1 - (int32_t)(UINT16_MAX - raw));
}

static cura_lora_v2_clear_header_t reader_take_header(fuzz_reader_t *reader) {
  cura_lora_v2_clear_header_t header = {
      .control = reader_take_u8(reader),
      .domain = reader_take_u8(reader),
  };
  for (size_t index = 0; index < sizeof(header.node_id); index++) {
    header.node_id[index] = reader_take_u8(reader);
  }
  header.sample_id = reader_take_u32(reader);
  return header;
}

static cura_lora_v2_reading_t reader_take_valid_reading(fuzz_reader_t *reader) {
  cura_lora_v2_reading_t reading = {
      .run_ms = reader_take_u16(reader),
      .soil_0_mv = reader_take_u16(reader),
      .soil_1_mv = reader_take_u16(reader),
      .soil_temp_0_centi_c = reader_take_i16(reader),
      .soil_temp_1_centi_c = reader_take_i16(reader),
      .enclosure_centi_c = reader_take_i16(reader),
      .enclosure_pressure_pa = reader_take_u32(reader),
      .enclosure_humidity_centi_pct = reader_take_u16(reader),
      .reset_reason = reader_take_u8(reader),
      .previous_current_tx_attempts = reader_take_u8(reader),
      .previous_awake_ms = reader_take_u16(reader),
      .previous_current_delivery_ms = reader_take_u16(reader),
      .previous_cycle_tx_attempts = reader_take_u8(reader),
      .previous_cycle_accepted_readings = reader_take_u8(reader),
      .flags = reader_take_u16(reader),
  };

  reading.flags &= (uint16_t)~CURA_LORA_V2_READING_RESERVED_FLAGS_MASK;

  if ((reading.flags & CURA_LORA_V2_FLAG_SOIL_0_VALID) == 0u) {
    reading.soil_0_mv = 0;
  }
  if ((reading.flags & CURA_LORA_V2_FLAG_SOIL_1_VALID) == 0u) {
    reading.soil_1_mv = 0;
  }
  if ((reading.flags & CURA_LORA_V2_FLAG_SOIL_TEMP_0_VALID) == 0u) {
    reading.soil_temp_0_centi_c = 0;
  }
  if ((reading.flags & CURA_LORA_V2_FLAG_SOIL_TEMP_1_VALID) == 0u) {
    reading.soil_temp_1_centi_c = 0;
  }
  if ((reading.flags & CURA_LORA_V2_FLAG_ENCLOSURE_TEMP_VALID) == 0u) {
    reading.enclosure_centi_c = 0;
  }
  if ((reading.flags & CURA_LORA_V2_FLAG_ENCLOSURE_PRESSURE_VALID) == 0u) {
    reading.enclosure_pressure_pa = 0;
  }
  if ((reading.flags & CURA_LORA_V2_FLAG_ENCLOSURE_HUMIDITY_VALID) == 0u) {
    reading.enclosure_humidity_centi_pct = 0;
  }

  if ((reading.flags & CURA_LORA_V2_FLAG_DEEP_SLEEP_BOOT) != 0u) {
    reading.reset_reason = CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP;
  } else if (reading.reset_reason ==
             CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP) {
    reading.reset_reason = CURA_LORA_V2_RESET_REASON_ESP_RST_UNKNOWN;
  }

  if ((reading.flags & CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID) == 0u) {
    reading.flags &= (uint16_t)~CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED;
    reading.previous_current_tx_attempts = 0;
    reading.previous_awake_ms = 0;
    reading.previous_current_delivery_ms = 0;
    reading.previous_cycle_tx_attempts = 0;
    reading.previous_cycle_accepted_readings = 0;
  } else if ((reading.flags & CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED) ==
             0u) {
    reading.previous_current_delivery_ms = 0;
  }

  require_condition(cura_lora_v2_validate_reading(&reading) ==
                    CURA_LORA_V2_CODEC_OK);
  return reading;
}

static size_t reader_take_capacity(fuzz_reader_t *reader, size_t required) {
  return (size_t)reader_take_u8(reader) % (required + FUZZ_EXTRA_CAPACITY + 1u);
}

static uint8_t *allocate_output(size_t capacity) {
  const size_t allocation_size = capacity == 0 ? 1u : capacity;
  uint8_t *output = malloc(allocation_size);
  require_condition(output != NULL);
  memset(output, FUZZ_CANARY, allocation_size);
  return output;
}

static void check_output_tail(const uint8_t *output, size_t written,
                              size_t capacity) {
  require_condition(capacity >= written);
  require_condition(
      bytes_are(&output[written], capacity - written, FUZZ_CANARY));
}

static void fuzz_decode_header(const uint8_t *data, size_t size) {
  cura_lora_v2_clear_header_t decoded;
  const cura_lora_v2_codec_result_t result =
      cura_lora_v2_decode_clear_header(&decoded, data, size);

  if (size != CURA_LORA_V2_CLEAR_HEADER_SIZE) {
    require_condition(result == CURA_LORA_V2_CODEC_INVALID_LENGTH);
    return;
  }

  require_condition(result == CURA_LORA_V2_CODEC_OK);
  uint8_t encoded[CURA_LORA_V2_CLEAR_HEADER_SIZE];
  require_condition(
      cura_lora_v2_encode_clear_header(encoded, sizeof(encoded), &decoded) ==
      CURA_LORA_V2_CODEC_OK);
  require_condition(memcmp(encoded, data, sizeof(encoded)) == 0);
}

static void fuzz_decode_reading(const uint8_t *data, size_t size) {
  cura_lora_v2_reading_t decoded;
  const cura_lora_v2_codec_result_t result =
      cura_lora_v2_decode_reading(&decoded, data, size);

  if (size != CURA_LORA_V2_READING_BODY_SIZE) {
    require_condition(result == CURA_LORA_V2_CODEC_INVALID_LENGTH);
    return;
  }
  if (result == CURA_LORA_V2_CODEC_MALFORMED) {
    return;
  }

  require_condition(result == CURA_LORA_V2_CODEC_OK);
  uint8_t encoded[CURA_LORA_V2_READING_BODY_SIZE];
  require_condition(
      cura_lora_v2_encode_reading(encoded, sizeof(encoded), &decoded) ==
      CURA_LORA_V2_CODEC_OK);
  require_condition(memcmp(encoded, data, sizeof(encoded)) == 0);
}

static void fuzz_decode_ack(const uint8_t *data, size_t size) {
  cura_lora_v2_ack_t decoded;
  const cura_lora_v2_codec_result_t result =
      cura_lora_v2_decode_ack(&decoded, data, size);

  if (size != CURA_LORA_V2_ACK_BODY_SIZE) {
    require_condition(result == CURA_LORA_V2_CODEC_INVALID_LENGTH);
    return;
  }
  if (data[0] > CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED) {
    require_condition(result == CURA_LORA_V2_CODEC_MALFORMED);
    return;
  }

  require_condition(result == CURA_LORA_V2_CODEC_OK);
  uint8_t encoded[CURA_LORA_V2_ACK_BODY_SIZE];
  require_condition(cura_lora_v2_encode_ack(encoded, sizeof(encoded),
                                            &decoded) == CURA_LORA_V2_CODEC_OK);
  require_condition(memcmp(encoded, data, sizeof(encoded)) == 0);
}

static void fuzz_encode_header(fuzz_reader_t *reader) {
  const size_t capacity =
      reader_take_capacity(reader, CURA_LORA_V2_CLEAR_HEADER_SIZE);
  const cura_lora_v2_clear_header_t header = reader_take_header(reader);
  uint8_t *output = allocate_output(capacity);

  const cura_lora_v2_codec_result_t result =
      cura_lora_v2_encode_clear_header(output, capacity, &header);
  if (capacity < CURA_LORA_V2_CLEAR_HEADER_SIZE) {
    require_condition(result == CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL);
    require_condition(
        bytes_are(output, capacity == 0 ? 1u : capacity, FUZZ_CANARY));
  } else {
    require_condition(result == CURA_LORA_V2_CODEC_OK);
    check_output_tail(output, CURA_LORA_V2_CLEAR_HEADER_SIZE, capacity);

    cura_lora_v2_clear_header_t decoded;
    require_condition(cura_lora_v2_decode_clear_header(
                          &decoded, output, CURA_LORA_V2_CLEAR_HEADER_SIZE) ==
                      CURA_LORA_V2_CODEC_OK);
    require_condition(headers_equal(&decoded, &header));
  }
  free(output);
}

static void fuzz_encode_reading(fuzz_reader_t *reader) {
  const size_t capacity =
      reader_take_capacity(reader, CURA_LORA_V2_READING_BODY_SIZE);
  const cura_lora_v2_reading_t reading = reader_take_valid_reading(reader);
  uint8_t *output = allocate_output(capacity);

  const cura_lora_v2_codec_result_t result =
      cura_lora_v2_encode_reading(output, capacity, &reading);
  if (capacity < CURA_LORA_V2_READING_BODY_SIZE) {
    require_condition(result == CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL);
    require_condition(
        bytes_are(output, capacity == 0 ? 1u : capacity, FUZZ_CANARY));
  } else {
    require_condition(result == CURA_LORA_V2_CODEC_OK);
    check_output_tail(output, CURA_LORA_V2_READING_BODY_SIZE, capacity);

    cura_lora_v2_reading_t decoded;
    require_condition(cura_lora_v2_decode_reading(
                          &decoded, output, CURA_LORA_V2_READING_BODY_SIZE) ==
                      CURA_LORA_V2_CODEC_OK);
    require_condition(readings_equal(&decoded, &reading));
  }
  free(output);
}

static void fuzz_encode_ack(fuzz_reader_t *reader) {
  const size_t capacity =
      reader_take_capacity(reader, CURA_LORA_V2_ACK_BODY_SIZE);
  const cura_lora_v2_ack_t ack = {
      .status = reader_take_u8(reader),
  };
  uint8_t *output = allocate_output(capacity);

  const cura_lora_v2_codec_result_t result =
      cura_lora_v2_encode_ack(output, capacity, &ack);
  if (capacity < CURA_LORA_V2_ACK_BODY_SIZE) {
    require_condition(result == CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL);
    require_condition(
        bytes_are(output, capacity == 0 ? 1u : capacity, FUZZ_CANARY));
  } else if (ack.status > CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED) {
    require_condition(result == CURA_LORA_V2_CODEC_MALFORMED);
  } else {
    require_condition(result == CURA_LORA_V2_CODEC_OK);
    require_condition(output[0] == ack.status);
    check_output_tail(output, CURA_LORA_V2_ACK_BODY_SIZE, capacity);
  }
  free(output);
}

static void fuzz_build_nonce(fuzz_reader_t *reader) {
  const size_t capacity = reader_take_capacity(reader, CURA_LORA_V2_NONCE_SIZE);
  const cura_lora_v2_clear_header_t header = reader_take_header(reader);
  uint8_t *output = allocate_output(capacity);

  const cura_lora_v2_codec_result_t result =
      cura_lora_v2_build_nonce(output, capacity, &header);
  if (capacity < CURA_LORA_V2_NONCE_SIZE) {
    require_condition(result == CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL);
    require_condition(
        bytes_are(output, capacity == 0 ? 1u : capacity, FUZZ_CANARY));
  } else {
    require_condition(result == CURA_LORA_V2_CODEC_OK);
    require_condition(memcmp(output, header.node_id, sizeof(header.node_id)) ==
                      0);
    require_condition(output[CURA_LORA_V2_NONCE_SAMPLE_ID_OFFSET] ==
                      (uint8_t)header.sample_id);
    require_condition(output[CURA_LORA_V2_NONCE_SAMPLE_ID_OFFSET + 1u] ==
                      (uint8_t)(header.sample_id >> 8));
    require_condition(output[CURA_LORA_V2_NONCE_SAMPLE_ID_OFFSET + 2u] ==
                      (uint8_t)(header.sample_id >> 16));
    require_condition(output[CURA_LORA_V2_NONCE_SAMPLE_ID_OFFSET + 3u] ==
                      (uint8_t)(header.sample_id >> 24));
    require_condition(output[CURA_LORA_V2_NONCE_DOMAIN_OFFSET] ==
                      header.domain);
    check_output_tail(output, CURA_LORA_V2_NONCE_SIZE, capacity);
  }
  free(output);
}

void protocol_v2_lora_fuzz_one_input(const uint8_t *data, size_t size) {
  if (size == 0) {
    return;
  }

  const fuzz_operation_t operation =
      (fuzz_operation_t)(data[0] % FUZZ_OPERATION_COUNT);
  fuzz_reader_t reader = {
      .data = &data[1],
      .size = size - 1u,
      .offset = 0,
  };

  switch (operation) {
  case FUZZ_DECODE_HEADER:
    fuzz_decode_header(reader.data, reader.size);
    break;
  case FUZZ_DECODE_READING:
    fuzz_decode_reading(reader.data, reader.size);
    break;
  case FUZZ_DECODE_ACK:
    fuzz_decode_ack(reader.data, reader.size);
    break;
  case FUZZ_ENCODE_HEADER:
    fuzz_encode_header(&reader);
    break;
  case FUZZ_ENCODE_READING:
    fuzz_encode_reading(&reader);
    break;
  case FUZZ_ENCODE_ACK:
    fuzz_encode_ack(&reader);
    break;
  case FUZZ_BUILD_NONCE:
    fuzz_build_nonce(&reader);
    break;
  }
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  protocol_v2_lora_fuzz_one_input(data, size);
  return 0;
}

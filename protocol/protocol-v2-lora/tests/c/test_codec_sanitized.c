#include "protocol_v2_lora_golden_vectors_generated.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_CANARY UINT8_C(0xa5)
#define EXTRA_OUTPUT_BYTES 8u

#define CHECK(expression)                                                      \
  do {                                                                         \
    if (!(expression)) {                                                       \
      fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__,         \
              #expression);                                                    \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define RUN_TEST(function)                                                     \
  do {                                                                         \
    if (!(function)()) {                                                       \
      fprintf(stderr, "failed: %s\n", #function);                              \
      return EXIT_FAILURE;                                                     \
    }                                                                          \
    tests_run++;                                                               \
  } while (0)

static bool bytes_equal(const uint8_t *left, const uint8_t *right,
                        size_t size) {
  return memcmp(left, right, size) == 0;
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
         bytes_equal(left->node_id, right->node_id, sizeof(left->node_id)) &&
         left->message_id == right->message_id;
}

static bool readings_equal(const cura_lora_v2_reading_t *left,
                           const cura_lora_v2_reading_t *right) {
  return left->sample_id == right->sample_id &&
         left->run_ms == right->run_ms && left->soil_0_mv == right->soil_0_mv &&
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

static bool round_trip_reading(const cura_lora_v2_reading_t *input) {
  uint8_t encoded[CURA_LORA_V2_READING_BODY_SIZE];
  cura_lora_v2_reading_t decoded;

  CHECK(cura_lora_v2_encode_reading(encoded, sizeof(encoded), input) ==
        CURA_LORA_V2_CODEC_OK);
  CHECK(cura_lora_v2_decode_reading(&decoded, encoded, sizeof(encoded)) ==
        CURA_LORA_V2_CODEC_OK);
  CHECK(readings_equal(input, &decoded));
  return true;
}

static bool test_boolean_helpers_smoke(void) {
  CHECK(cura_lora_v2_is_supported_control(CURA_LORA_V2_CONTROL));
  CHECK(!cura_lora_v2_is_supported_control(UINT8_C(0x21)));

  CHECK(cura_lora_v2_domain_is_reading(
      CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK));
  CHECK(!cura_lora_v2_domain_is_reading(
      CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK));

  CHECK(cura_lora_v2_domain_is_ack(CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK));
  CHECK(
      !cura_lora_v2_domain_is_ack(CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK));

  CHECK(cura_lora_v2_ack_status_matches_domain(
      CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_ACK_STATUS_ACCEPTED));
  CHECK(!cura_lora_v2_ack_status_matches_domain(
      CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_ACK_STATUS_RETRY_LATER));
  return true;
}

static bool test_validate_reading_paths(void) {
  cura_lora_v2_reading_t malformed = TEST_GOLDEN_READING;

  CHECK(cura_lora_v2_validate_reading(NULL) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_validate_reading(&TEST_GOLDEN_READING) ==
        CURA_LORA_V2_CODEC_OK);

  malformed.flags |= UINT16_C(1) << 15;
  CHECK(cura_lora_v2_validate_reading(&malformed) ==
        CURA_LORA_V2_CODEC_MALFORMED);
  return true;
}

static bool test_encode_header_null_arguments(void) {
  uint8_t output[CURA_LORA_V2_CLEAR_HEADER_SIZE];

  CHECK(cura_lora_v2_encode_clear_header(NULL, sizeof(output),
                                         &TEST_GOLDEN_HEADER) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_encode_clear_header(output, sizeof(output), NULL) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  return true;
}

static bool test_encode_header_undersized_output(void) {
  for (size_t size = 0; size < CURA_LORA_V2_CLEAR_HEADER_SIZE; size++) {
    uint8_t storage[CURA_LORA_V2_CLEAR_HEADER_SIZE + 2u];
    memset(storage, TEST_CANARY, sizeof(storage));

    CHECK(cura_lora_v2_encode_clear_header(&storage[1], size,
                                           &TEST_GOLDEN_HEADER) ==
          CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL);
    CHECK(bytes_are(storage, sizeof(storage), TEST_CANARY));
  }
  return true;
}

static bool test_encode_header_exact_and_oversized_output(void) {
  uint8_t exact[CURA_LORA_V2_CLEAR_HEADER_SIZE + 2u];
  memset(exact, TEST_CANARY, sizeof(exact));

  CHECK(cura_lora_v2_encode_clear_header(
            &exact[1], CURA_LORA_V2_CLEAR_HEADER_SIZE, &TEST_GOLDEN_HEADER) ==
        CURA_LORA_V2_CODEC_OK);
  CHECK(exact[0] == TEST_CANARY);
  CHECK(bytes_equal(&exact[1], TEST_GOLDEN_HEADER_BYTES,
                    CURA_LORA_V2_CLEAR_HEADER_SIZE));
  CHECK(exact[sizeof(exact) - 1u] == TEST_CANARY);

  uint8_t oversized[CURA_LORA_V2_CLEAR_HEADER_SIZE + EXTRA_OUTPUT_BYTES + 2u];
  memset(oversized, TEST_CANARY, sizeof(oversized));

  CHECK(cura_lora_v2_encode_clear_header(
            &oversized[1], CURA_LORA_V2_CLEAR_HEADER_SIZE + EXTRA_OUTPUT_BYTES,
            &TEST_GOLDEN_HEADER) == CURA_LORA_V2_CODEC_OK);
  CHECK(oversized[0] == TEST_CANARY);
  CHECK(bytes_equal(&oversized[1], TEST_GOLDEN_HEADER_BYTES,
                    CURA_LORA_V2_CLEAR_HEADER_SIZE));
  CHECK(bytes_are(&oversized[1 + CURA_LORA_V2_CLEAR_HEADER_SIZE],
                  EXTRA_OUTPUT_BYTES + 1u, TEST_CANARY));
  return true;
}

static bool test_decode_header_null_arguments(void) {
  cura_lora_v2_clear_header_t decoded;

  CHECK(cura_lora_v2_decode_clear_header(NULL, TEST_GOLDEN_HEADER_BYTES,
                                         CURA_LORA_V2_CLEAR_HEADER_SIZE) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_decode_clear_header(&decoded, NULL,
                                         CURA_LORA_V2_CLEAR_HEADER_SIZE) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  return true;
}

static bool test_decode_header_wrong_sizes(void) {
  cura_lora_v2_clear_header_t decoded;
  uint8_t nonnull = 0;

  CHECK(cura_lora_v2_decode_clear_header(&decoded, &nonnull, 0) ==
        CURA_LORA_V2_CODEC_INVALID_LENGTH);

  for (size_t size = 1; size < CURA_LORA_V2_CLEAR_HEADER_SIZE; size++) {
    uint8_t *input = malloc(size);
    CHECK(input != NULL);
    memcpy(input, TEST_GOLDEN_HEADER_BYTES, size);
    const cura_lora_v2_codec_result_t result =
        cura_lora_v2_decode_clear_header(&decoded, input, size);
    free(input);
    CHECK(result == CURA_LORA_V2_CODEC_INVALID_LENGTH);
  }

  const size_t oversized_sizes[] = {
      CURA_LORA_V2_CLEAR_HEADER_SIZE + 1u,
      CURA_LORA_V2_CLEAR_HEADER_SIZE + EXTRA_OUTPUT_BYTES,
  };
  for (size_t index = 0;
       index < sizeof(oversized_sizes) / sizeof(oversized_sizes[0]); index++) {
    const size_t size = oversized_sizes[index];
    uint8_t *input = calloc(size, 1u);
    CHECK(input != NULL);
    memcpy(input, TEST_GOLDEN_HEADER_BYTES, CURA_LORA_V2_CLEAR_HEADER_SIZE);
    const cura_lora_v2_codec_result_t result =
        cura_lora_v2_decode_clear_header(&decoded, input, size);
    free(input);
    CHECK(result == CURA_LORA_V2_CODEC_INVALID_LENGTH);
  }
  return true;
}

static bool test_decode_header_golden_and_boundaries(void) {
  cura_lora_v2_clear_header_t decoded;

  CHECK(cura_lora_v2_decode_clear_header(&decoded, TEST_GOLDEN_HEADER_BYTES,
                                         CURA_LORA_V2_CLEAR_HEADER_SIZE) ==
        CURA_LORA_V2_CODEC_OK);
  CHECK(headers_equal(&decoded, &TEST_GOLDEN_HEADER));

  const uint32_t message_ids[] = {
      UINT32_C(0),          UINT32_C(1), UINT32_C(0x7fffffff),
      UINT32_C(0x80000000), UINT32_MAX,
  };
  for (size_t index = 0; index < sizeof(message_ids) / sizeof(message_ids[0]);
       index++) {
    cura_lora_v2_clear_header_t input = TEST_GOLDEN_HEADER;
    uint8_t encoded[CURA_LORA_V2_CLEAR_HEADER_SIZE];
    input.message_id = message_ids[index];

    CHECK(cura_lora_v2_encode_clear_header(encoded, sizeof(encoded), &input) ==
          CURA_LORA_V2_CODEC_OK);
    CHECK(cura_lora_v2_decode_clear_header(
              &decoded, encoded, sizeof(encoded)) == CURA_LORA_V2_CODEC_OK);
    CHECK(headers_equal(&decoded, &input));
  }
  return true;
}

static bool test_build_nonce_arguments_and_sizes(void) {
  uint8_t output[CURA_LORA_V2_NONCE_SIZE];

  CHECK(cura_lora_v2_build_nonce(NULL, sizeof(output), &TEST_GOLDEN_HEADER) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_build_nonce(output, sizeof(output), NULL) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);

  for (size_t size = 0; size < CURA_LORA_V2_NONCE_SIZE; size++) {
    uint8_t storage[CURA_LORA_V2_NONCE_SIZE + 2u];
    memset(storage, TEST_CANARY, sizeof(storage));

    CHECK(cura_lora_v2_build_nonce(&storage[1], size, &TEST_GOLDEN_HEADER) ==
          CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL);
    CHECK(bytes_are(storage, sizeof(storage), TEST_CANARY));
  }
  return true;
}

static bool test_build_nonce_exact_and_oversized_output(void) {
  uint8_t exact[CURA_LORA_V2_NONCE_SIZE + 2u];
  memset(exact, TEST_CANARY, sizeof(exact));

  CHECK(cura_lora_v2_build_nonce(&exact[1], CURA_LORA_V2_NONCE_SIZE,
                                 &TEST_GOLDEN_HEADER) == CURA_LORA_V2_CODEC_OK);
  CHECK(exact[0] == TEST_CANARY);
  CHECK(bytes_equal(&exact[1], TEST_GOLDEN_NONCE, CURA_LORA_V2_NONCE_SIZE));
  CHECK(exact[sizeof(exact) - 1u] == TEST_CANARY);

  uint8_t oversized[CURA_LORA_V2_NONCE_SIZE + EXTRA_OUTPUT_BYTES + 2u];
  memset(oversized, TEST_CANARY, sizeof(oversized));

  CHECK(cura_lora_v2_build_nonce(&oversized[1],
                                 CURA_LORA_V2_NONCE_SIZE + EXTRA_OUTPUT_BYTES,
                                 &TEST_GOLDEN_HEADER) == CURA_LORA_V2_CODEC_OK);
  CHECK(bytes_equal(&oversized[1], TEST_GOLDEN_NONCE, CURA_LORA_V2_NONCE_SIZE));
  CHECK(bytes_are(&oversized[1 + CURA_LORA_V2_NONCE_SIZE],
                  EXTRA_OUTPUT_BYTES + 1u, TEST_CANARY));
  return true;
}

static bool test_encode_reading_null_and_undersized_output(void) {
  uint8_t output[CURA_LORA_V2_READING_BODY_SIZE];

  CHECK(
      cura_lora_v2_encode_reading(NULL, sizeof(output), &TEST_GOLDEN_READING) ==
      CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_encode_reading(output, sizeof(output), NULL) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);

  for (size_t size = 0; size < CURA_LORA_V2_READING_BODY_SIZE; size++) {
    uint8_t storage[CURA_LORA_V2_READING_BODY_SIZE + 2u];
    memset(storage, TEST_CANARY, sizeof(storage));

    CHECK(
        cura_lora_v2_encode_reading(&storage[1], size, &TEST_GOLDEN_READING) ==
        CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL);
    CHECK(bytes_are(storage, sizeof(storage), TEST_CANARY));
  }
  return true;
}

static bool test_encode_reading_exact_and_oversized_output(void) {
  uint8_t exact[CURA_LORA_V2_READING_BODY_SIZE + 2u];
  memset(exact, TEST_CANARY, sizeof(exact));

  CHECK(cura_lora_v2_encode_reading(&exact[1], CURA_LORA_V2_READING_BODY_SIZE,
                                    &TEST_GOLDEN_READING) ==
        CURA_LORA_V2_CODEC_OK);
  CHECK(exact[0] == TEST_CANARY);
  CHECK(bytes_equal(&exact[1], TEST_GOLDEN_READING_BYTES,
                    CURA_LORA_V2_READING_BODY_SIZE));
  CHECK(exact[sizeof(exact) - 1u] == TEST_CANARY);

  uint8_t oversized[CURA_LORA_V2_READING_BODY_SIZE + EXTRA_OUTPUT_BYTES + 2u];
  memset(oversized, TEST_CANARY, sizeof(oversized));

  CHECK(cura_lora_v2_encode_reading(
            &oversized[1], CURA_LORA_V2_READING_BODY_SIZE + EXTRA_OUTPUT_BYTES,
            &TEST_GOLDEN_READING) == CURA_LORA_V2_CODEC_OK);
  CHECK(bytes_equal(&oversized[1], TEST_GOLDEN_READING_BYTES,
                    CURA_LORA_V2_READING_BODY_SIZE));
  CHECK(bytes_are(&oversized[1 + CURA_LORA_V2_READING_BODY_SIZE],
                  EXTRA_OUTPUT_BYTES + 1u, TEST_CANARY));
  return true;
}

static bool test_reading_representative_malformed_paths(void) {
  cura_lora_v2_reading_t malformed = TEST_GOLDEN_READING;
  uint8_t output[CURA_LORA_V2_READING_BODY_SIZE];
  uint8_t malformed_body[CURA_LORA_V2_READING_BODY_SIZE];
  cura_lora_v2_reading_t decoded;

  malformed.flags |= UINT16_C(1) << 15;
  CHECK(cura_lora_v2_encode_reading(output, sizeof(output), &malformed) ==
        CURA_LORA_V2_CODEC_MALFORMED);

  memcpy(malformed_body, TEST_GOLDEN_READING_BYTES, sizeof(malformed_body));
  malformed_body[CURA_LORA_V2_READING_FLAGS_OFFSET + 1u] |= UINT8_C(0x80);
  CHECK(cura_lora_v2_decode_reading(&decoded, malformed_body,
                                    sizeof(malformed_body)) ==
        CURA_LORA_V2_CODEC_MALFORMED);
  return true;
}

static bool test_decode_reading_null_and_wrong_sizes(void) {
  cura_lora_v2_reading_t decoded;
  uint8_t nonnull = 0;

  CHECK(cura_lora_v2_decode_reading(NULL, TEST_GOLDEN_READING_BYTES,
                                    CURA_LORA_V2_READING_BODY_SIZE) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_decode_reading(&decoded, NULL,
                                    CURA_LORA_V2_READING_BODY_SIZE) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_decode_reading(&decoded, &nonnull, 0) ==
        CURA_LORA_V2_CODEC_INVALID_LENGTH);

  for (size_t size = 1; size < CURA_LORA_V2_READING_BODY_SIZE; size++) {
    uint8_t *input = malloc(size);
    CHECK(input != NULL);
    memcpy(input, TEST_GOLDEN_READING_BYTES, size);
    const cura_lora_v2_codec_result_t result =
        cura_lora_v2_decode_reading(&decoded, input, size);
    free(input);
    CHECK(result == CURA_LORA_V2_CODEC_INVALID_LENGTH);
  }

  const size_t oversized_sizes[] = {
      CURA_LORA_V2_READING_BODY_SIZE + 1u,
      CURA_LORA_V2_READING_BODY_SIZE + EXTRA_OUTPUT_BYTES,
  };
  for (size_t index = 0;
       index < sizeof(oversized_sizes) / sizeof(oversized_sizes[0]); index++) {
    const size_t size = oversized_sizes[index];
    uint8_t *input = calloc(size, 1u);
    CHECK(input != NULL);
    memcpy(input, TEST_GOLDEN_READING_BYTES, CURA_LORA_V2_READING_BODY_SIZE);
    const cura_lora_v2_codec_result_t result =
        cura_lora_v2_decode_reading(&decoded, input, size);
    free(input);
    CHECK(result == CURA_LORA_V2_CODEC_INVALID_LENGTH);
  }
  return true;
}

static bool test_decode_reading_golden(void) {
  cura_lora_v2_reading_t decoded;

  CHECK(cura_lora_v2_decode_reading(&decoded, TEST_GOLDEN_READING_BYTES,
                                    CURA_LORA_V2_READING_BODY_SIZE) ==
        CURA_LORA_V2_CODEC_OK);
  CHECK(readings_equal(&decoded, &TEST_GOLDEN_READING));
  return true;
}

static bool test_reading_integer_boundaries(void) {
  const int16_t signed_values[] = {
      INT16_MIN, INT16_C(-1), INT16_C(0), INT16_C(1), INT16_MAX,
  };
  for (size_t index = 0;
       index < sizeof(signed_values) / sizeof(signed_values[0]); index++) {
    cura_lora_v2_reading_t reading = TEST_GOLDEN_READING;
    reading.soil_temp_0_centi_c = signed_values[index];
    reading.soil_temp_1_centi_c = signed_values[index];
    reading.enclosure_centi_c = signed_values[index];
    CHECK(round_trip_reading(&reading));
  }

  const cura_lora_v2_reading_t zero = {0};
  CHECK(round_trip_reading(&zero));

  cura_lora_v2_reading_t maximum = TEST_GOLDEN_READING;
  maximum.run_ms = UINT16_MAX;
  maximum.soil_0_mv = UINT16_MAX;
  maximum.soil_1_mv = UINT16_MAX;
  maximum.enclosure_pressure_pa = UINT32_MAX;
  maximum.enclosure_humidity_centi_pct = UINT16_MAX;
  maximum.reset_reason = UINT8_MAX;
  maximum.previous_current_tx_attempts = UINT8_MAX;
  maximum.previous_awake_ms = UINT16_MAX;
  maximum.previous_current_delivery_ms = UINT16_MAX;
  maximum.previous_cycle_tx_attempts = UINT8_MAX;
  maximum.previous_cycle_accepted_readings = UINT8_MAX;
  maximum.flags &= (uint16_t)~CURA_LORA_V2_FLAG_DEEP_SLEEP_BOOT;
  CHECK(round_trip_reading(&maximum));
  return true;
}

static bool test_reading_misaligned_wire_buffer(void) {
  uint8_t storage[CURA_LORA_V2_READING_BODY_SIZE + 4u];
  cura_lora_v2_reading_t decoded;
  size_t offset = 0;
  const size_t alignment = _Alignof(uint32_t);

  if (alignment > 1u) {
    while (offset < alignment &&
           ((uintptr_t)&storage[offset] % alignment) == 0u) {
      offset++;
    }
    CHECK(offset < alignment);
  }

  CHECK(cura_lora_v2_encode_reading(
            &storage[offset], CURA_LORA_V2_READING_BODY_SIZE,
            &TEST_GOLDEN_READING) == CURA_LORA_V2_CODEC_OK);
  CHECK(cura_lora_v2_decode_reading(&decoded, &storage[offset],
                                    CURA_LORA_V2_READING_BODY_SIZE) ==
        CURA_LORA_V2_CODEC_OK);
  CHECK(readings_equal(&decoded, &TEST_GOLDEN_READING));
  return true;
}

static bool test_encode_ack_arguments_sizes_and_statuses(void) {
  uint8_t output[CURA_LORA_V2_ACK_BODY_SIZE + EXTRA_OUTPUT_BYTES];
  cura_lora_v2_ack_t ack = {
      .status = CURA_LORA_V2_ACK_STATUS_ACCEPTED,
  };

  CHECK(cura_lora_v2_encode_ack(NULL, CURA_LORA_V2_ACK_BODY_SIZE, &ack) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_encode_ack(output, CURA_LORA_V2_ACK_BODY_SIZE, NULL) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);

  memset(output, TEST_CANARY, sizeof(output));
  CHECK(cura_lora_v2_encode_ack(output, 0, &ack) ==
        CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL);
  CHECK(bytes_are(output, sizeof(output), TEST_CANARY));

  for (uint8_t status = CURA_LORA_V2_ACK_STATUS_ACCEPTED;
       status <= CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED; status++) {
    ack.status = status;
    memset(output, TEST_CANARY, sizeof(output));
    CHECK(cura_lora_v2_encode_ack(output, sizeof(output), &ack) ==
          CURA_LORA_V2_CODEC_OK);
    CHECK(output[0] == status);
    CHECK(bytes_are(&output[1], sizeof(output) - 1u, TEST_CANARY));
  }

  ack.status = UINT8_C(4);
  CHECK(cura_lora_v2_encode_ack(output, sizeof(output), &ack) ==
        CURA_LORA_V2_CODEC_MALFORMED);
  ack.status = UINT8_MAX;
  CHECK(cura_lora_v2_encode_ack(output, sizeof(output), &ack) ==
        CURA_LORA_V2_CODEC_MALFORMED);
  return true;
}

static bool test_decode_ack_arguments_sizes_and_statuses(void) {
  cura_lora_v2_ack_t decoded;
  uint8_t input[2] = {0};

  CHECK(cura_lora_v2_decode_ack(NULL, input, CURA_LORA_V2_ACK_BODY_SIZE) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_decode_ack(&decoded, NULL, CURA_LORA_V2_ACK_BODY_SIZE) ==
        CURA_LORA_V2_CODEC_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_decode_ack(&decoded, input, 0) ==
        CURA_LORA_V2_CODEC_INVALID_LENGTH);
  CHECK(cura_lora_v2_decode_ack(&decoded, input, sizeof(input)) ==
        CURA_LORA_V2_CODEC_INVALID_LENGTH);

  for (uint8_t status = CURA_LORA_V2_ACK_STATUS_ACCEPTED;
       status <= CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED; status++) {
    input[0] = status;
    CHECK(
        cura_lora_v2_decode_ack(&decoded, input, CURA_LORA_V2_ACK_BODY_SIZE) ==
        CURA_LORA_V2_CODEC_OK);
    CHECK(decoded.status == status);
  }

  input[0] = UINT8_C(4);
  CHECK(cura_lora_v2_decode_ack(&decoded, input, CURA_LORA_V2_ACK_BODY_SIZE) ==
        CURA_LORA_V2_CODEC_MALFORMED);
  input[0] = UINT8_MAX;
  CHECK(cura_lora_v2_decode_ack(&decoded, input, CURA_LORA_V2_ACK_BODY_SIZE) ==
        CURA_LORA_V2_CODEC_MALFORMED);
  return true;
}

int main(void) {
  unsigned int tests_run = 0;

  RUN_TEST(test_boolean_helpers_smoke);
  RUN_TEST(test_validate_reading_paths);
  RUN_TEST(test_encode_header_null_arguments);
  RUN_TEST(test_encode_header_undersized_output);
  RUN_TEST(test_encode_header_exact_and_oversized_output);
  RUN_TEST(test_decode_header_null_arguments);
  RUN_TEST(test_decode_header_wrong_sizes);
  RUN_TEST(test_decode_header_golden_and_boundaries);
  RUN_TEST(test_build_nonce_arguments_and_sizes);
  RUN_TEST(test_build_nonce_exact_and_oversized_output);
  RUN_TEST(test_encode_reading_null_and_undersized_output);
  RUN_TEST(test_encode_reading_exact_and_oversized_output);
  RUN_TEST(test_reading_representative_malformed_paths);
  RUN_TEST(test_decode_reading_null_and_wrong_sizes);
  RUN_TEST(test_decode_reading_golden);
  RUN_TEST(test_reading_integer_boundaries);
  RUN_TEST(test_reading_misaligned_wire_buffer);
  RUN_TEST(test_encode_ack_arguments_sizes_and_statuses);
  RUN_TEST(test_decode_ack_arguments_sizes_and_statuses);

  printf("%u sanitized C codec tests passed\n", tests_run);
  return EXIT_SUCCESS;
}

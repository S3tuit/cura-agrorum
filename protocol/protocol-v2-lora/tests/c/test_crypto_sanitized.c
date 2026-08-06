#include "protocol_v2_lora_crypto.h"
#include "protocol_v2_lora_crypto_vectors_generated.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_CANARY UINT8_C(0xa5)
#define TEST_SENTINEL UINT8_C(0x5a)
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

static void set_header_sentinel(cura_lora_v2_clear_header_t *header) {
  memset(header, TEST_SENTINEL, sizeof(*header));
}

static bool header_is_sentinel(const cura_lora_v2_clear_header_t *header) {
  return bytes_are((const uint8_t *)header, sizeof(*header), TEST_SENTINEL);
}

static bool test_golden_seal_and_open(void) {
  uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE];
  size_t frame_size = 0u;

  CHECK(cura_lora_v2_seal_frame(
            frame, sizeof(frame), &frame_size, TEST_CRYPTO_NODE_KEY,
            &TEST_CRYPTO_HEADER, TEST_CRYPTO_READING_BODY,
            sizeof(TEST_CRYPTO_READING_BODY)) == CURA_LORA_V2_CRYPTO_OK);
  CHECK(frame_size == sizeof(TEST_CRYPTO_READING_FRAME));
  CHECK(memcmp(frame, TEST_CRYPTO_READING_FRAME, sizeof(frame)) == 0);

  cura_lora_v2_clear_header_t header;
  uint8_t plaintext[CURA_LORA_V2_READING_BODY_SIZE];
  size_t plaintext_size = 0u;
  CHECK(cura_lora_v2_open_frame(&header, plaintext, sizeof(plaintext),
                                &plaintext_size, TEST_CRYPTO_NODE_KEY, frame,
                                frame_size) == CURA_LORA_V2_CRYPTO_OK);
  CHECK(headers_equal(&header, &TEST_CRYPTO_HEADER));
  CHECK(plaintext_size == sizeof(TEST_CRYPTO_READING_BODY));
  CHECK(memcmp(plaintext, TEST_CRYPTO_READING_BODY, sizeof(plaintext)) == 0);
  return true;
}

static bool test_seal_null_arguments(void) {
  uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE];
  size_t frame_size = SIZE_MAX;

  CHECK(cura_lora_v2_seal_frame(NULL, sizeof(frame), &frame_size,
                                TEST_CRYPTO_NODE_KEY, &TEST_CRYPTO_HEADER,
                                TEST_CRYPTO_READING_BODY,
                                sizeof(TEST_CRYPTO_READING_BODY)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  CHECK(frame_size == 0u);
  CHECK(cura_lora_v2_seal_frame(frame, sizeof(frame), NULL,
                                TEST_CRYPTO_NODE_KEY, &TEST_CRYPTO_HEADER,
                                TEST_CRYPTO_READING_BODY,
                                sizeof(TEST_CRYPTO_READING_BODY)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_seal_frame(frame, sizeof(frame), &frame_size, NULL,
                                &TEST_CRYPTO_HEADER, TEST_CRYPTO_READING_BODY,
                                sizeof(TEST_CRYPTO_READING_BODY)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_seal_frame(
            frame, sizeof(frame), &frame_size, TEST_CRYPTO_NODE_KEY, NULL,
            TEST_CRYPTO_READING_BODY, sizeof(TEST_CRYPTO_READING_BODY)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_seal_frame(frame, sizeof(frame), &frame_size,
                                TEST_CRYPTO_NODE_KEY, &TEST_CRYPTO_HEADER, NULL,
                                sizeof(TEST_CRYPTO_READING_BODY)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  return true;
}

static bool test_seal_invalid_body_sizes(void) {
  uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE];
  uint8_t body[CURA_LORA_V2_MAX_BODY_SIZE + 1u] = {0};
  size_t frame_size = SIZE_MAX;

  CHECK(cura_lora_v2_seal_frame(frame, sizeof(frame), &frame_size,
                                TEST_CRYPTO_NODE_KEY, &TEST_CRYPTO_HEADER, body,
                                0u) == CURA_LORA_V2_CRYPTO_INVALID_LENGTH);
  CHECK(frame_size == 0u);
  CHECK(cura_lora_v2_seal_frame(frame, sizeof(frame), &frame_size,
                                TEST_CRYPTO_NODE_KEY, &TEST_CRYPTO_HEADER, body,
                                sizeof(body)) ==
        CURA_LORA_V2_CRYPTO_INVALID_LENGTH);
  CHECK(frame_size == 0u);
  return true;
}

static bool test_seal_undersized_outputs_are_unchanged(void) {
  for (size_t capacity = 0u; capacity < CURA_LORA_V2_READING_FRAME_SIZE;
       capacity++) {
    uint8_t storage[CURA_LORA_V2_READING_FRAME_SIZE + 2u];
    memset(storage, TEST_CANARY, sizeof(storage));
    size_t frame_size = SIZE_MAX;

    CHECK(cura_lora_v2_seal_frame(&storage[1], capacity, &frame_size,
                                  TEST_CRYPTO_NODE_KEY, &TEST_CRYPTO_HEADER,
                                  TEST_CRYPTO_READING_BODY,
                                  sizeof(TEST_CRYPTO_READING_BODY)) ==
          CURA_LORA_V2_CRYPTO_BUFFER_TOO_SMALL);
    CHECK(frame_size == 0u);
    CHECK(bytes_are(storage, sizeof(storage), TEST_CANARY));
  }
  return true;
}

static bool test_seal_oversized_output_tail_is_unchanged(void) {
  uint8_t storage[CURA_LORA_V2_READING_FRAME_SIZE + EXTRA_OUTPUT_BYTES + 2u];
  memset(storage, TEST_CANARY, sizeof(storage));
  size_t frame_size = 0u;

  CHECK(cura_lora_v2_seal_frame(
            &storage[1], sizeof(storage) - 2u, &frame_size,
            TEST_CRYPTO_NODE_KEY, &TEST_CRYPTO_HEADER, TEST_CRYPTO_READING_BODY,
            sizeof(TEST_CRYPTO_READING_BODY)) == CURA_LORA_V2_CRYPTO_OK);
  CHECK(frame_size == CURA_LORA_V2_READING_FRAME_SIZE);
  CHECK(storage[0] == TEST_CANARY);
  CHECK(memcmp(&storage[1], TEST_CRYPTO_READING_FRAME, frame_size) == 0);
  CHECK(bytes_are(&storage[1 + frame_size], EXTRA_OUTPUT_BYTES + 1u,
                  TEST_CANARY));
  return true;
}

static bool test_open_null_arguments(void) {
  cura_lora_v2_clear_header_t header;
  uint8_t plaintext[CURA_LORA_V2_READING_BODY_SIZE];
  size_t plaintext_size = SIZE_MAX;

  CHECK(cura_lora_v2_open_frame(NULL, plaintext, sizeof(plaintext),
                                &plaintext_size, TEST_CRYPTO_NODE_KEY,
                                TEST_CRYPTO_READING_FRAME,
                                sizeof(TEST_CRYPTO_READING_FRAME)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  CHECK(plaintext_size == 0u);
  CHECK(cura_lora_v2_open_frame(&header, NULL, sizeof(plaintext),
                                &plaintext_size, TEST_CRYPTO_NODE_KEY,
                                TEST_CRYPTO_READING_FRAME,
                                sizeof(TEST_CRYPTO_READING_FRAME)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_open_frame(&header, plaintext, sizeof(plaintext), NULL,
                                TEST_CRYPTO_NODE_KEY, TEST_CRYPTO_READING_FRAME,
                                sizeof(TEST_CRYPTO_READING_FRAME)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_open_frame(
            &header, plaintext, sizeof(plaintext), &plaintext_size, NULL,
            TEST_CRYPTO_READING_FRAME, sizeof(TEST_CRYPTO_READING_FRAME)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  CHECK(cura_lora_v2_open_frame(&header, plaintext, sizeof(plaintext),
                                &plaintext_size, TEST_CRYPTO_NODE_KEY, NULL,
                                sizeof(TEST_CRYPTO_READING_FRAME)) ==
        CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT);
  return true;
}

static bool test_open_invalid_frame_sizes(void) {
  cura_lora_v2_clear_header_t header;
  uint8_t plaintext[CURA_LORA_V2_READING_BODY_SIZE];
  size_t plaintext_size = SIZE_MAX;

  const size_t invalid_sizes[] = {
      0u,
      CURA_LORA_V2_MIN_FRAME_SIZE - 1u,
      CURA_LORA_V2_MAX_FRAME_SIZE + 1u,
  };
  for (size_t index = 0u;
       index < sizeof(invalid_sizes) / sizeof(invalid_sizes[0]); index++) {
    CHECK(cura_lora_v2_open_frame(
              &header, plaintext, sizeof(plaintext), &plaintext_size,
              TEST_CRYPTO_NODE_KEY, TEST_CRYPTO_READING_FRAME,
              invalid_sizes[index]) == CURA_LORA_V2_CRYPTO_INVALID_LENGTH);
    CHECK(plaintext_size == 0u);
  }
  return true;
}

static bool test_open_undersized_outputs_are_unchanged(void) {
  for (size_t capacity = 0u; capacity < CURA_LORA_V2_READING_BODY_SIZE;
       capacity++) {
    cura_lora_v2_clear_header_t header;
    set_header_sentinel(&header);
    uint8_t plaintext[CURA_LORA_V2_READING_BODY_SIZE];
    memset(plaintext, TEST_CANARY, sizeof(plaintext));
    size_t plaintext_size = SIZE_MAX;

    CHECK(cura_lora_v2_open_frame(&header, plaintext, capacity, &plaintext_size,
                                  TEST_CRYPTO_NODE_KEY,
                                  TEST_CRYPTO_READING_FRAME,
                                  sizeof(TEST_CRYPTO_READING_FRAME)) ==
          CURA_LORA_V2_CRYPTO_BUFFER_TOO_SMALL);
    CHECK(plaintext_size == 0u);
    CHECK(header_is_sentinel(&header));
    CHECK(bytes_are(plaintext, sizeof(plaintext), TEST_CANARY));
  }
  return true;
}

static bool test_authentication_failure_publishes_nothing(void) {
  for (size_t index = 0u; index < sizeof(TEST_CRYPTO_READING_FRAME); index++) {
    uint8_t tampered[sizeof(TEST_CRYPTO_READING_FRAME)];
    memcpy(tampered, TEST_CRYPTO_READING_FRAME, sizeof(tampered));
    tampered[index] ^= UINT8_C(0x01);

    cura_lora_v2_clear_header_t header;
    set_header_sentinel(&header);
    uint8_t plaintext[CURA_LORA_V2_READING_BODY_SIZE];
    memset(plaintext, TEST_CANARY, sizeof(plaintext));
    size_t plaintext_size = SIZE_MAX;

    CHECK(cura_lora_v2_open_frame(&header, plaintext, sizeof(plaintext),
                                  &plaintext_size, TEST_CRYPTO_NODE_KEY,
                                  tampered, sizeof(tampered)) ==
          CURA_LORA_V2_CRYPTO_AUTHENTICATION_FAILED);
    CHECK(plaintext_size == 0u);
    CHECK(header_is_sentinel(&header));
    CHECK(bytes_are(plaintext, sizeof(plaintext), TEST_CANARY));
  }

  uint8_t wrong_key[sizeof(TEST_CRYPTO_NODE_KEY)];
  memcpy(wrong_key, TEST_CRYPTO_NODE_KEY, sizeof(wrong_key));
  wrong_key[0] ^= UINT8_C(0x80);
  cura_lora_v2_clear_header_t header;
  set_header_sentinel(&header);
  uint8_t plaintext[CURA_LORA_V2_READING_BODY_SIZE];
  memset(plaintext, TEST_CANARY, sizeof(plaintext));
  size_t plaintext_size = SIZE_MAX;

  CHECK(cura_lora_v2_open_frame(
            &header, plaintext, sizeof(plaintext), &plaintext_size, wrong_key,
            TEST_CRYPTO_READING_FRAME, sizeof(TEST_CRYPTO_READING_FRAME)) ==
        CURA_LORA_V2_CRYPTO_AUTHENTICATION_FAILED);
  CHECK(plaintext_size == 0u);
  CHECK(header_is_sentinel(&header));
  CHECK(bytes_are(plaintext, sizeof(plaintext), TEST_CANARY));
  return true;
}

int main(void) {
  unsigned tests_run = 0u;

  RUN_TEST(test_golden_seal_and_open);
  RUN_TEST(test_seal_null_arguments);
  RUN_TEST(test_seal_invalid_body_sizes);
  RUN_TEST(test_seal_undersized_outputs_are_unchanged);
  RUN_TEST(test_seal_oversized_output_tail_is_unchanged);
  RUN_TEST(test_open_null_arguments);
  RUN_TEST(test_open_invalid_frame_sizes);
  RUN_TEST(test_open_undersized_outputs_are_unchanged);
  RUN_TEST(test_authentication_failure_publishes_nothing);

  printf("protocol v2 LoRa crypto sanitizer harness: %u tests passed\n",
         tests_run);
  return EXIT_SUCCESS;
}

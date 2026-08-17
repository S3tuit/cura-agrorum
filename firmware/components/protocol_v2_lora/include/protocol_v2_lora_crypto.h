/*
 * Authenticated frame layer for Cura Agrorum LoRa protocol v2.
 *
 * Wire contract: protocol/protocol-v2-lora/README.md, especially "Routine
 * packet", "CCM nonce", and "Receiver validation".
 */
#pragma once

#include "protocol_v2_lora_schema_generated.h"

#include <stddef.h>
#include <stdint.h>

#define CURA_LORA_V2_MIN_BODY_SIZE CURA_LORA_V2_ACK_BODY_SIZE
#define CURA_LORA_V2_MAX_BODY_SIZE CURA_LORA_V2_READING_BODY_SIZE
#define CURA_LORA_V2_MIN_FRAME_SIZE CURA_LORA_V2_ACK_FRAME_SIZE
#define CURA_LORA_V2_MAX_FRAME_SIZE CURA_LORA_V2_READING_FRAME_SIZE

typedef enum {
  CURA_LORA_V2_CRYPTO_OK = 0,
  CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT,
  CURA_LORA_V2_CRYPTO_BUFFER_TOO_SMALL,
  CURA_LORA_V2_CRYPTO_INVALID_LENGTH,
  CURA_LORA_V2_CRYPTO_AUTHENTICATION_FAILED,
  CURA_LORA_V2_CRYPTO_BACKEND_ERROR,
} cura_lora_v2_crypto_result_t;

/*
 * Encrypts and authenticates one already-serialized application body.
 *
 * Successful output:
 *   encoded clear header || ciphertext body || eight-byte CCM tag
 *
 * The encoded 14-byte header is both transmitted in clear and supplied to CCM
 * as AAD. The nonce is built from header->node_id, header->message_id and
 * header->domain. Ciphertext length equals plaintext_body_size.
 *
 * Parameters:
 * - output: writable frame buffer. On success, receives exactly
 *   CURA_LORA_V2_CLEAR_HEADER_SIZE + plaintext_body_size +
 *   CURA_LORA_V2_TAG_SIZE bytes. It is unchanged on failure.
 * - output_size: capacity of output in bytes. It may be larger than the
 *   required frame; bytes after output_length are not modified.
 * - output_length: required writable size_t. Set to the complete frame length
 *   on success and zero on every failure.
 * - node_key: exactly CURA_LORA_V2_KEY_SIZE readable bytes containing this
 *   node's derived AES key, not the receiver group master key.
 * - header: logical clear header to serialize, authenticate and use for nonce
 *   construction. This layer does not require a supported control or domain.
 * - plaintext_body: readable, already-serialized application body. This layer
 *   encrypts it without interpreting its fields.
 * - plaintext_body_size: number of readable bytes at plaintext_body. It must
 *   be between CURA_LORA_V2_MIN_BODY_SIZE and
 *   CURA_LORA_V2_MAX_BODY_SIZE, inclusive.
 *
 * Returns:
 * - CURA_LORA_V2_CRYPTO_OK on success.
 * - CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT for a required NULL pointer or an
 *   unencodable header.
 * - CURA_LORA_V2_CRYPTO_INVALID_LENGTH for an unsupported body size.
 * - CURA_LORA_V2_CRYPTO_BUFFER_TOO_SMALL when output_size is insufficient.
 * - CURA_LORA_V2_CRYPTO_BACKEND_ERROR when the crypto provider fails.
 *
 * Semantic control, domain and body validation remains the caller's
 * responsibility. Caller-provided input and output storage must not overlap.
 */
cura_lora_v2_crypto_result_t cura_lora_v2_seal_frame(
    uint8_t *output, size_t output_size, size_t *output_length,
    const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
    const cura_lora_v2_clear_header_t *header, const uint8_t *plaintext_body,
    size_t plaintext_body_size);

/*
 * Authenticates and decrypts one complete received application frame.
 *
 * The first CURA_LORA_V2_CLEAR_HEADER_SIZE bytes of frame are used directly as
 * CCM AAD. They are also decoded to construct the nonce. Neither the decoded
 * header nor plaintext is published until CCM authentication succeeds.
 *
 * Parameters:
 * - header: required writable structure. On success, receives the authenticated
 *   decoded clear header. It is unchanged on failure.
 * - plaintext_body: writable body buffer. On success, receives the
 *   authenticated plaintext body. It is unchanged on failure.
 * - plaintext_body_capacity: capacity of plaintext_body in bytes. It must be at
 *   least frame_size - CURA_LORA_V2_CLEAR_HEADER_SIZE -
 *   CURA_LORA_V2_TAG_SIZE.
 * - plaintext_body_length: required writable size_t. Set to the plaintext body
 *   length on success and zero on every failure.
 * - node_key: exactly CURA_LORA_V2_KEY_SIZE readable bytes containing the key
 *   selected from the frame's untrusted claimed node_id.
 * - frame: readable complete frame in clear-header || ciphertext || tag order.
 *   The caller may inspect its header for key lookup, but must not trust those
 *   fields before this function succeeds.
 * - frame_size: number of readable bytes at frame. It must be between
 *   CURA_LORA_V2_MIN_FRAME_SIZE and CURA_LORA_V2_MAX_FRAME_SIZE, inclusive.
 *
 * Returns:
 * - CURA_LORA_V2_CRYPTO_OK on successful authentication and decryption.
 * - CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT for a required NULL pointer or an
 *   undecodable fixed header.
 * - CURA_LORA_V2_CRYPTO_INVALID_LENGTH for an unsupported complete frame size.
 * - CURA_LORA_V2_CRYPTO_BUFFER_TOO_SMALL when plaintext_body_capacity is
 *   insufficient.
 * - CURA_LORA_V2_CRYPTO_AUTHENTICATION_FAILED when CCM verification fails,
 *   including frames received under a wrong key or with corrupted header,
 *   ciphertext or tag bytes.
 * - CURA_LORA_V2_CRYPTO_BACKEND_ERROR for other crypto-provider failures.
 *
 * This function establishes authenticity only. The caller must subsequently
 * validate control, domain, expected body length and decoded body semantics.
 * Caller-provided input and output storage must not overlap.
 */
cura_lora_v2_crypto_result_t
cura_lora_v2_open_frame(cura_lora_v2_clear_header_t *header,
                        uint8_t *plaintext_body, size_t plaintext_body_capacity,
                        size_t *plaintext_body_length,
                        const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
                        const uint8_t *frame, size_t frame_size);

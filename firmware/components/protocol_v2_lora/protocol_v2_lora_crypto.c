/*
 * Authenticated frame layer for Cura Agrorum LoRa protocol v2.
 *
 * Wire contract: protocol/protocol-v2-lora/README.md, especially "Routine
 * packet", "CCM nonce", and "Receiver validation".
 */

#include "protocol_v2_lora_crypto.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if defined(CURA_LORA_V2_CRYPTO_USE_OPENSSL)
#include <openssl/evp.h>
#else
#include <psa/crypto.h>
#endif

#define CURA_LORA_V2_ENCRYPTED_BODY_MAX_SIZE                                   \
  (CURA_LORA_V2_MAX_BODY_SIZE + CURA_LORA_V2_TAG_SIZE)

static bool body_size_is_valid(size_t size) {
  return size >= CURA_LORA_V2_MIN_BODY_SIZE &&
         size <= CURA_LORA_V2_MAX_BODY_SIZE;
}

static void secure_zero(void *memory, size_t size) {
  volatile uint8_t *bytes = memory;
  while (size > 0u) {
    *bytes = 0u;
    bytes++;
    size--;
  }
}

#if defined(CURA_LORA_V2_CRYPTO_USE_OPENSSL)

static cura_lora_v2_crypto_result_t
backend_encrypt(const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
                const uint8_t nonce[CURA_LORA_V2_NONCE_SIZE],
                const uint8_t associated_data[CURA_LORA_V2_CLEAR_HEADER_SIZE],
                const uint8_t *plaintext, size_t plaintext_size,
                uint8_t *encrypted, size_t encrypted_capacity,
                size_t *encrypted_size) {
  if (encrypted_capacity < plaintext_size + CURA_LORA_V2_TAG_SIZE) {
    return CURA_LORA_V2_CRYPTO_BUFFER_TOO_SMALL;
  }

  EVP_CIPHER_CTX *context = EVP_CIPHER_CTX_new();
  if (context == NULL) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }

  int length = 0;
  int written = 0;
  bool ok =
      EVP_EncryptInit_ex(context, EVP_aes_128_ccm(), NULL, NULL, NULL) == 1 &&
      EVP_CIPHER_CTX_ctrl(context, EVP_CTRL_AEAD_SET_IVLEN,
                          CURA_LORA_V2_NONCE_SIZE, NULL) == 1 &&
      EVP_CIPHER_CTX_ctrl(context, EVP_CTRL_AEAD_SET_TAG, CURA_LORA_V2_TAG_SIZE,
                          NULL) == 1 &&
      EVP_EncryptInit_ex(context, NULL, NULL, node_key, nonce) == 1 &&
      EVP_EncryptUpdate(context, NULL, &length, NULL, (int)plaintext_size) ==
          1 &&
      EVP_EncryptUpdate(context, NULL, &length, associated_data,
                        CURA_LORA_V2_CLEAR_HEADER_SIZE) == 1 &&
      EVP_EncryptUpdate(context, encrypted, &written, plaintext,
                        (int)plaintext_size) == 1 &&
      written == (int)plaintext_size &&
      EVP_CIPHER_CTX_ctrl(context, EVP_CTRL_AEAD_GET_TAG, CURA_LORA_V2_TAG_SIZE,
                          &encrypted[plaintext_size]) == 1;

  EVP_CIPHER_CTX_free(context);
  if (!ok) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }

  *encrypted_size = plaintext_size + CURA_LORA_V2_TAG_SIZE;
  return CURA_LORA_V2_CRYPTO_OK;
}

static cura_lora_v2_crypto_result_t
backend_decrypt(const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
                const uint8_t nonce[CURA_LORA_V2_NONCE_SIZE],
                const uint8_t associated_data[CURA_LORA_V2_CLEAR_HEADER_SIZE],
                const uint8_t *encrypted, size_t encrypted_size,
                uint8_t *plaintext, size_t plaintext_capacity,
                size_t *plaintext_size) {
  const size_t ciphertext_size = encrypted_size - CURA_LORA_V2_TAG_SIZE;
  if (plaintext_capacity < ciphertext_size) {
    return CURA_LORA_V2_CRYPTO_BUFFER_TOO_SMALL;
  }

  EVP_CIPHER_CTX *context = EVP_CIPHER_CTX_new();
  if (context == NULL) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }

  int length = 0;
  int written = 0;
  const bool setup_ok =
      EVP_DecryptInit_ex(context, EVP_aes_128_ccm(), NULL, NULL, NULL) == 1 &&
      EVP_CIPHER_CTX_ctrl(context, EVP_CTRL_AEAD_SET_IVLEN,
                          CURA_LORA_V2_NONCE_SIZE, NULL) == 1 &&
      EVP_CIPHER_CTX_ctrl(context, EVP_CTRL_AEAD_SET_TAG, CURA_LORA_V2_TAG_SIZE,
                          (void *)&encrypted[ciphertext_size]) == 1 &&
      EVP_DecryptInit_ex(context, NULL, NULL, node_key, nonce) == 1 &&
      EVP_DecryptUpdate(context, NULL, &length, NULL, (int)ciphertext_size) ==
          1 &&
      EVP_DecryptUpdate(context, NULL, &length, associated_data,
                        CURA_LORA_V2_CLEAR_HEADER_SIZE) == 1;

  if (!setup_ok) {
    EVP_CIPHER_CTX_free(context);
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }

  const int decrypt_result = EVP_DecryptUpdate(context, plaintext, &written,
                                               encrypted, (int)ciphertext_size);
  EVP_CIPHER_CTX_free(context);
  if (decrypt_result != 1) {
    return CURA_LORA_V2_CRYPTO_AUTHENTICATION_FAILED;
  }
  if (written != (int)ciphertext_size) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }

  *plaintext_size = ciphertext_size;
  return CURA_LORA_V2_CRYPTO_OK;
}

#else

#define CURA_LORA_V2_PSA_ALGORITHM                                             \
  PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, CURA_LORA_V2_TAG_SIZE)

static psa_status_t
import_node_key(const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
                psa_key_usage_t usage, psa_key_id_t *key_id) {
  psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
  psa_set_key_usage_flags(&attributes, usage);
  psa_set_key_algorithm(&attributes, CURA_LORA_V2_PSA_ALGORITHM);
  psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
  psa_set_key_bits(&attributes, CURA_LORA_V2_KEY_SIZE * 8u);

  const psa_status_t status =
      psa_import_key(&attributes, node_key, CURA_LORA_V2_KEY_SIZE, key_id);
  psa_reset_key_attributes(&attributes);
  return status;
}

static cura_lora_v2_crypto_result_t
backend_encrypt(const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
                const uint8_t nonce[CURA_LORA_V2_NONCE_SIZE],
                const uint8_t associated_data[CURA_LORA_V2_CLEAR_HEADER_SIZE],
                const uint8_t *plaintext, size_t plaintext_size,
                uint8_t *encrypted, size_t encrypted_capacity,
                size_t *encrypted_size) {
  if (psa_crypto_init() != PSA_SUCCESS) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }

  psa_key_id_t key_id = 0;
  if (import_node_key(node_key, PSA_KEY_USAGE_ENCRYPT, &key_id) !=
      PSA_SUCCESS) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }

  const psa_status_t crypto_status = psa_aead_encrypt(
      key_id, CURA_LORA_V2_PSA_ALGORITHM, nonce, CURA_LORA_V2_NONCE_SIZE,
      associated_data, CURA_LORA_V2_CLEAR_HEADER_SIZE, plaintext,
      plaintext_size, encrypted, encrypted_capacity, encrypted_size);
  const psa_status_t destroy_status = psa_destroy_key(key_id);

  if (crypto_status != PSA_SUCCESS || destroy_status != PSA_SUCCESS) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }
  return CURA_LORA_V2_CRYPTO_OK;
}

static cura_lora_v2_crypto_result_t
backend_decrypt(const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
                const uint8_t nonce[CURA_LORA_V2_NONCE_SIZE],
                const uint8_t associated_data[CURA_LORA_V2_CLEAR_HEADER_SIZE],
                const uint8_t *encrypted, size_t encrypted_size,
                uint8_t *plaintext, size_t plaintext_capacity,
                size_t *plaintext_size) {
  if (psa_crypto_init() != PSA_SUCCESS) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }

  psa_key_id_t key_id = 0;
  if (import_node_key(node_key, PSA_KEY_USAGE_DECRYPT, &key_id) !=
      PSA_SUCCESS) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }

  const psa_status_t crypto_status = psa_aead_decrypt(
      key_id, CURA_LORA_V2_PSA_ALGORITHM, nonce, CURA_LORA_V2_NONCE_SIZE,
      associated_data, CURA_LORA_V2_CLEAR_HEADER_SIZE, encrypted,
      encrypted_size, plaintext, plaintext_capacity, plaintext_size);
  const psa_status_t destroy_status = psa_destroy_key(key_id);

  if (destroy_status != PSA_SUCCESS) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }
  if (crypto_status == PSA_ERROR_INVALID_SIGNATURE) {
    return CURA_LORA_V2_CRYPTO_AUTHENTICATION_FAILED;
  }
  if (crypto_status != PSA_SUCCESS) {
    return CURA_LORA_V2_CRYPTO_BACKEND_ERROR;
  }
  return CURA_LORA_V2_CRYPTO_OK;
}

#endif

cura_lora_v2_crypto_result_t cura_lora_v2_seal_frame(
    uint8_t *output, size_t output_size, size_t *output_length,
    const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
    const cura_lora_v2_clear_header_t *header, const uint8_t *plaintext_body,
    size_t plaintext_body_size) {
  if (output_length != NULL) {
    *output_length = 0u;
  }
  if (output == NULL || output_length == NULL || node_key == NULL ||
      header == NULL || plaintext_body == NULL) {
    return CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT;
  }
  if (!body_size_is_valid(plaintext_body_size)) {
    return CURA_LORA_V2_CRYPTO_INVALID_LENGTH;
  }

  const size_t required_size = CURA_LORA_V2_CLEAR_HEADER_SIZE +
                               plaintext_body_size + CURA_LORA_V2_TAG_SIZE;
  if (output_size < required_size) {
    return CURA_LORA_V2_CRYPTO_BUFFER_TOO_SMALL;
  }

  uint8_t associated_data[CURA_LORA_V2_CLEAR_HEADER_SIZE];
  uint8_t nonce[CURA_LORA_V2_NONCE_SIZE];
  uint8_t encrypted[CURA_LORA_V2_ENCRYPTED_BODY_MAX_SIZE];
  size_t encrypted_size = 0u;

  if (cura_lora_v2_encode_clear_header(associated_data, sizeof(associated_data),
                                       header) != CURA_LORA_V2_CODEC_OK ||
      cura_lora_v2_build_nonce(nonce, sizeof(nonce), header) !=
          CURA_LORA_V2_CODEC_OK) {
    return CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT;
  }

  const cura_lora_v2_crypto_result_t result = backend_encrypt(
      node_key, nonce, associated_data, plaintext_body, plaintext_body_size,
      encrypted, sizeof(encrypted), &encrypted_size);
  if (result != CURA_LORA_V2_CRYPTO_OK ||
      encrypted_size != plaintext_body_size + CURA_LORA_V2_TAG_SIZE) {
    secure_zero(encrypted, sizeof(encrypted));
    return result == CURA_LORA_V2_CRYPTO_OK ? CURA_LORA_V2_CRYPTO_BACKEND_ERROR
                                            : result;
  }

  memcpy(output, associated_data, sizeof(associated_data));
  memcpy(&output[sizeof(associated_data)], encrypted, encrypted_size);
  *output_length = required_size;
  secure_zero(encrypted, sizeof(encrypted));
  return CURA_LORA_V2_CRYPTO_OK;
}

cura_lora_v2_crypto_result_t
cura_lora_v2_open_frame(cura_lora_v2_clear_header_t *header,
                        uint8_t *plaintext_body, size_t plaintext_body_capacity,
                        size_t *plaintext_body_length,
                        const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
                        const uint8_t *frame, size_t frame_size) {
  if (plaintext_body_length != NULL) {
    *plaintext_body_length = 0u;
  }
  if (header == NULL || plaintext_body == NULL ||
      plaintext_body_length == NULL || node_key == NULL || frame == NULL) {
    return CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT;
  }
  if (frame_size < CURA_LORA_V2_MIN_FRAME_SIZE ||
      frame_size > CURA_LORA_V2_MAX_FRAME_SIZE) {
    return CURA_LORA_V2_CRYPTO_INVALID_LENGTH;
  }

  const size_t plaintext_size =
      frame_size - CURA_LORA_V2_CLEAR_HEADER_SIZE - CURA_LORA_V2_TAG_SIZE;
  if (!body_size_is_valid(plaintext_size)) {
    return CURA_LORA_V2_CRYPTO_INVALID_LENGTH;
  }
  if (plaintext_body_capacity < plaintext_size) {
    return CURA_LORA_V2_CRYPTO_BUFFER_TOO_SMALL;
  }

  cura_lora_v2_clear_header_t decoded_header;
  uint8_t nonce[CURA_LORA_V2_NONCE_SIZE];
  uint8_t decrypted[CURA_LORA_V2_MAX_BODY_SIZE];
  size_t decrypted_size = 0u;

  if (cura_lora_v2_decode_clear_header(&decoded_header, frame,
                                       CURA_LORA_V2_CLEAR_HEADER_SIZE) !=
          CURA_LORA_V2_CODEC_OK ||
      cura_lora_v2_build_nonce(nonce, sizeof(nonce), &decoded_header) !=
          CURA_LORA_V2_CODEC_OK) {
    return CURA_LORA_V2_CRYPTO_INVALID_ARGUMENT;
  }

  const uint8_t *encrypted = &frame[CURA_LORA_V2_CLEAR_HEADER_SIZE];
  const size_t encrypted_size = frame_size - CURA_LORA_V2_CLEAR_HEADER_SIZE;
  const cura_lora_v2_crypto_result_t result =
      backend_decrypt(node_key, nonce, frame, encrypted, encrypted_size,
                      decrypted, sizeof(decrypted), &decrypted_size);
  if (result != CURA_LORA_V2_CRYPTO_OK || decrypted_size != plaintext_size) {
    secure_zero(decrypted, sizeof(decrypted));
    return result == CURA_LORA_V2_CRYPTO_OK ? CURA_LORA_V2_CRYPTO_BACKEND_ERROR
                                            : result;
  }

  *header = decoded_header;
  memcpy(plaintext_body, decrypted, decrypted_size);
  *plaintext_body_length = decrypted_size;
  secure_zero(decrypted, sizeof(decrypted));
  return CURA_LORA_V2_CRYPTO_OK;
}

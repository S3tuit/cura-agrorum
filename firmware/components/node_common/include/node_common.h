#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Shared result type for fallible node components.
 *
 * The upper 16 bits contain a curag_error_domain_t and the lower 16 bits
 * contain a domain-specific curag_error_code_t. Zero is the only success
 * value. Domain and error-code assignments are append-only once deployed.
 */
typedef uint32_t err_curag_t;
typedef uint16_t curag_error_domain_t;
typedef uint16_t curag_error_code_t;

#define CURAG_OK UINT32_C(0)

#define CURAG_EDOM_NONE UINT16_C(0)
#define CURAG_EDOM_PERSISTENCE UINT16_C(1)
#define CURAG_EDOM_RADIO UINT16_C(2)
#define CURAG_EDOM_SENSORS UINT16_C(3)

#define CURAG_ECODE_NONE UINT16_C(0)

/*
 * Packs a component error domain and its domain-specific error code.
 *
 * domain: The component that owns the error-code namespace.
 * code:   The error code within that domain.
 *
 * Returns the packed err_curag_t. Components must use nonzero domain and code
 * values for failures and CURAG_OK for success.
 */
static inline err_curag_t curag_error_make(curag_error_domain_t domain,
                                           curag_error_code_t code) {
  return ((err_curag_t)domain << 16U) | (err_curag_t)code;
}

/* Returns the error-domain half of error. */
static inline curag_error_domain_t curag_error_domain(err_curag_t error) {
  return (curag_error_domain_t)(error >> 16U);
}

/* Returns the domain-specific error-code half of error. */
static inline curag_error_code_t curag_error_code(err_curag_t error) {
  return (curag_error_code_t)(error & UINT32_C(0xffff));
}

/* Returns true only for the unique success value, CURAG_OK. */
static inline bool curag_error_is_success(err_curag_t error) {
  return error == CURAG_OK;
}

/*
 * Stable cross-component diagnostic operations. These describe the action
 * being attempted rather than a public function or private helper name.
 * Assignments are append-only once deployed.
 */
typedef uint16_t curag_operation_t;

#define CURAG_OP_NONE UINT16_C(0)
#define CURAG_OP_INITIALIZE UINT16_C(1)
#define CURAG_OP_VALIDATE UINT16_C(2)
#define CURAG_OP_READ UINT16_C(3)
#define CURAG_OP_WRITE UINT16_C(4)
#define CURAG_OP_APPEND UINT16_C(5)
#define CURAG_OP_REMOVE UINT16_C(6)
#define CURAG_OP_SYNC UINT16_C(7)
#define CURAG_OP_RECOVER UINT16_C(8)
#define CURAG_OP_COMPACT UINT16_C(9)
#define CURAG_OP_POWER_ON UINT16_C(10)
#define CURAG_OP_POWER_OFF UINT16_C(11)
#define CURAG_OP_ENCODE UINT16_C(12)
#define CURAG_OP_DECODE UINT16_C(13)
#define CURAG_OP_ENCRYPT UINT16_C(14)
#define CURAG_OP_DECRYPT UINT16_C(15)
#define CURAG_OP_TRANSMIT UINT16_C(16)
#define CURAG_OP_RECEIVE UINT16_C(17)
#define CURAG_OP_SLEEP UINT16_C(18)

typedef uint8_t curag_context_schema_t;

#define CURAG_CONTEXT_SCHEMA_NONE UINT8_C(0)
#define CURAG_DIAGNOSTIC_CONTEXT_MAX 252U

/*
 * Opaque component diagnostic detail returned alongside err_curag_t.
 *
 * operation:      Stable CURAG_OP_* value for the failed action.
 * context_length: Number of meaningful bytes in context.
 * context_schema: Domain-local schema used to interpret context. Zero if and
 *                 only if context_length is zero.
 * context:        Canonically encoded domain-local data. It is never
 *                 serialized by copying this native structure.
 */
typedef struct {
  curag_operation_t operation;
  uint8_t context_length;
  curag_context_schema_t context_schema;
  uint8_t context[CURAG_DIAGNOSTIC_CONTEXT_MAX];
} diagn_context_t;

/*
 * Clears every byte of an optional caller-owned diagnostic output.
 *
 * context: Diagnostic context to clear, or NULL to discard diagnostic detail.
 *
 * A cleared context has operation, schema and length set to their NONE values.
 */
void curag_diagnostic_context_clear(diagn_context_t *context);

#if defined(__cplusplus)
static_assert(sizeof(err_curag_t) == 4U, "err_curag_t must be 32 bits");
static_assert(offsetof(diagn_context_t, operation) == 0U,
              "unexpected diagnostic operation offset");
static_assert(offsetof(diagn_context_t, context_length) == 2U,
              "unexpected diagnostic length offset");
static_assert(offsetof(diagn_context_t, context_schema) == 3U,
              "unexpected diagnostic schema offset");
static_assert(offsetof(diagn_context_t, context) == 4U,
              "unexpected diagnostic context offset");
static_assert(sizeof(diagn_context_t) == 256U,
              "unexpected diagn_context_t layout");
#else
_Static_assert(sizeof(err_curag_t) == 4U, "err_curag_t must be 32 bits");
_Static_assert(offsetof(diagn_context_t, operation) == 0U,
               "unexpected diagnostic operation offset");
_Static_assert(offsetof(diagn_context_t, context_length) == 2U,
               "unexpected diagnostic length offset");
_Static_assert(offsetof(diagn_context_t, context_schema) == 3U,
               "unexpected diagnostic schema offset");
_Static_assert(offsetof(diagn_context_t, context) == 4U,
               "unexpected diagnostic context offset");
_Static_assert(sizeof(diagn_context_t) == 256U,
               "unexpected diagn_context_t layout");
#endif

#ifdef __cplusplus
}
#endif

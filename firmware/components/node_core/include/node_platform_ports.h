#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Infallible monotonic clock used throughout one wake cycle.
 *
 * monotonic_us returns a nondecreasing time in microseconds from an
 * unspecified epoch. The context is borrowed and may be NULL.
 */
typedef struct {
  void *context;
  uint64_t (*monotonic_us)(void *context);
} node_clock_port_t;

/*
 * Non-cryptographic retry-randomness source.
 *
 * uniform_u32_inclusive requires minimum <= maximum and returns a uniformly
 * selected value from that inclusive interval. The context is borrowed and
 * may be NULL.
 */
typedef struct {
  void *context;
  uint32_t (*uniform_u32_inclusive)(void *context, uint32_t minimum,
                                    uint32_t maximum);
} node_randomness_port_t;

/*
 * Platform lifecycle operations.
 *
 * get_reset_reason returns the protocol-facing reset-reason byte.
 * enter_deep_sleep_for receives a nonzero relative duration in microseconds
 * and is terminal in production. The context is borrowed and may be NULL.
 */
typedef struct {
  void *context;
  uint8_t (*get_reset_reason)(void *context);
  void (*enter_deep_sleep_for)(void *context, uint64_t duration_us);
} node_system_port_t;

/* Complete set of platform dependencies borrowed by node_core for one wake. */
typedef struct {
  node_clock_port_t clock;
  node_randomness_port_t randomness;
  node_system_port_t system;
} node_platform_ports_t;

#ifdef __cplusplus
}
#endif

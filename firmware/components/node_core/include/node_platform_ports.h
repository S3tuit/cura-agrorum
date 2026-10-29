#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Infallible monotonic clock used throughout one wake cycle.
 *
 * monotonic_us returns a nondecreasing time in microseconds from an
 * unspecified epoch, including time spent blocked. The radio backend must use
 * the same clock; epochs need not survive sleep. Context is borrowed and may
 * be NULL. A backwards clock is a composition error, not repaired by the core.
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
 * may be NULL. Equal bounds and the full u32 interval are supported without
 * bias or overflow. This source must not supply identity/key randomness.
 */
typedef struct {
  void *context;
  uint32_t (*uniform_u32_inclusive)(void *context, uint32_t minimum,
                                    uint32_t maximum);
} node_randomness_port_t;

/*
 * Platform lifecycle operations.
 *
 * get_reset_reason returns the protocol-facing reset-reason byte, not a sleep
 * wakeup cause. Unassigned byte values remain valid; out-of-byte values map to 0.
 * enter_deep_sleep_for receives a nonzero relative duration in microseconds
 * and is terminal in production. Timer-setup failure is contained by the adapter
 * without reopening persistence after final sync. Context is borrowed and may
 * be NULL; a returning host fake terminates the cycle just as a real sleep does.
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

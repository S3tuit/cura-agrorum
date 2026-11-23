#pragma once
#include <stddef.h>
#include <stdint.h>

/* Return received length, zero for silence, negative for transport failure.
 * The transport owns framing gaps and a bounded response deadline. */
typedef int (*sen0600_exchange_t)(void *context, const uint8_t request[8],
                                uint8_t *response, size_t capacity);
typedef enum {
  SEN0600_ASSIGNED = 0,
  SEN0600_ALREADY_TWO,
  SEN0600_ABSENT,
  SEN0600_CONFLICT,
  SEN0600_BAD_REPLY,
  SEN0600_WRITE_UNCERTAIN,
  SEN0600_VERIFY_FAILED,
} sen0600_commission_result_t;

/* Only one physically isolated probe is permitted. Identical replies from
 * two probes sharing address 1 cannot be distinguished in software. */
sen0600_commission_result_t sen0600_assign_two(sen0600_exchange_t exchange,
                                             void *context);

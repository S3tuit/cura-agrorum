#pragma once
#include <stddef.h>
#include <stdint.h>

/* Test-only interrupted TX; the production singleton must already be initialized. */
uint64_t radio_header_abort(const uint8_t *payload, size_t length);
void radio_header_wait_since(uint64_t previous_start);
void radio_header_dump(void);

#pragma once

#include <stddef.h>
#include <stdint.h>

/* Minimum remaining window required at the instant immediately before SetTx. */
uint64_t sx1262_radio_min_set_tx_window_us(size_t payload_length);

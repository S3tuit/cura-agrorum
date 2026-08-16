#include "sx1262_radio.h"

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include "sx1262_radio_timing.h"

#define SX1262_RADIO_TX_WATCHDOG_MARGIN_US UINT64_C(5000)
#define SX1262_RADIO_TX_PREPARE_ALLOWANCE_US UINT64_C(5000)
#define SX1262_RADIO_RTC_STEPS_PER_SECOND UINT64_C(64000)
#define SX1262_RADIO_US_PER_SECOND UINT64_C(1000000)

static uint64_t ceil_div_u64(uint64_t numerator, uint64_t denominator) {
  return numerator / denominator + (numerator % denominator != 0U ? 1U : 0U);
}

static uint64_t saturating_add_u64(uint64_t left, uint64_t right) {
  return UINT64_MAX - left < right ? UINT64_MAX : left + right;
}

uint64_t sx1262_radio_airtime_us(size_t payload_length) {
  if (payload_length == 0U || payload_length > SX1262_RADIO_MAX_PAYLOAD_SIZE) {
    return UINT64_MAX;
  }

  const uint64_t spreading_factor = SX1262_RADIO_SPREADING_FACTOR;
  const uint64_t effective_spreading_factor =
      spreading_factor - (SX1262_RADIO_LOW_DATA_RATE_OPTIMIZE ? 2U : 0U);
  const int64_t payload_numerator =
      (int64_t)(UINT64_C(8) * (uint64_t)payload_length) -
      (int64_t)(UINT64_C(4) * spreading_factor) + INT64_C(28) +
      (SX1262_RADIO_PAYLOAD_CRC ? INT64_C(16) : INT64_C(0)) -
      (SX1262_RADIO_EXPLICIT_HEADER ? INT64_C(0) : INT64_C(20));

  uint64_t payload_symbols = UINT64_C(8);
  if (payload_numerator > 0) {
    const uint64_t payload_denominator =
        UINT64_C(4) * effective_spreading_factor;
    payload_symbols +=
        ceil_div_u64((uint64_t)payload_numerator, payload_denominator) *
        SX1262_RADIO_CODING_RATE_DENOMINATOR;
  }

  const uint64_t quarter_symbols = UINT64_C(4) * SX1262_RADIO_PREAMBLE_SYMBOLS +
                                   UINT64_C(17) + UINT64_C(4) * payload_symbols;
  const uint64_t symbol_duration_numerator_us =
      (UINT64_C(1) << SX1262_RADIO_SPREADING_FACTOR) *
      SX1262_RADIO_US_PER_SECOND;
  return ceil_div_u64(quarter_symbols * symbol_duration_numerator_us,
                      UINT64_C(4) * SX1262_RADIO_BANDWIDTH_HZ);
}

uint64_t sx1262_radio_min_set_tx_window_us(size_t payload_length) {
  const uint64_t airtime_us = sx1262_radio_airtime_us(payload_length);
  if (airtime_us == UINT64_MAX) {
    return UINT64_MAX;
  }

  const uint64_t tx_duration_us =
      saturating_add_u64(airtime_us, SX1262_RADIO_TX_RAMP_US);
  const uint64_t required_watchdog_steps =
      ceil_div_u64(tx_duration_us * SX1262_RADIO_RTC_STEPS_PER_SECOND,
                   SX1262_RADIO_US_PER_SECOND);
  const uint64_t watchdog_input_us =
      ceil_div_u64(required_watchdog_steps * SX1262_RADIO_US_PER_SECOND,
                   SX1262_RADIO_RTC_STEPS_PER_SECOND);
  return saturating_add_u64(SX1262_RADIO_TX_WATCHDOG_MARGIN_US,
                            watchdog_input_us);
}

uint64_t sx1262_radio_min_tx_window_us(size_t payload_length) {
  return saturating_add_u64(sx1262_radio_min_set_tx_window_us(payload_length),
                            SX1262_RADIO_TX_PREPARE_ALLOWANCE_US);
}

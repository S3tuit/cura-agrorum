#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sx1262_radio.h"

/* SX1262 IRQ bit assignments used by the component policy and backend. */
#define SX1262_RADIO_IRQ_TX_DONE UINT16_C(1U << 0U)
#define SX1262_RADIO_IRQ_RX_DONE UINT16_C(1U << 1U)
#define SX1262_RADIO_IRQ_HEADER_ERROR UINT16_C(1U << 5U)
#define SX1262_RADIO_IRQ_CRC_ERROR UINT16_C(1U << 6U)
#define SX1262_RADIO_IRQ_TIMEOUT UINT16_C(1U << 9U)
#define SX1262_RADIO_IRQ_ALL UINT16_C(0x43ff)

typedef struct {
  uint32_t frequency_hz;
  uint32_t bandwidth_hz;
  uint16_t preamble_symbols;
  uint16_t ramp_us;
  uint16_t tcxo_startup_ms;
  int8_t radiated_tx_power_dbm;
  int8_t tx_params_power_dbm;
  uint8_t spreading_factor;
  uint8_t coding_rate_denominator;
  uint8_t sync_word;
  uint8_t pa_duty_cycle;
  uint8_t pa_hp_max;
  uint8_t pa_device_sel;
  uint8_t pa_lut;
  uint8_t tcxo_voltage;
  bool low_data_rate_optimize;
  bool explicit_header;
  bool payload_crc;
  bool boosted_rx;
  bool dcdc_regulator;
  bool dio2_rf_switch;
  bool dio3_tcxo;
} sx1262_radio_profile_t;

/*
 * Canonical failure detail returned by the private backend. error_code is a
 * CURAG_ERADIO_* value. flags use CURAG_RADIO_CONTEXT_* assignments.
 */
typedef struct {
  uint16_t error_code;
  uint8_t command_opcode;
  uint8_t stage;
  uint8_t flags;
  uint8_t backend_status_kind;
  int32_t backend_status;
  uint8_t chip_status;
  uint16_t irq_status;
  uint16_t device_errors;
} sx1262_radio_backend_error_t;

typedef struct {
  uint16_t payload_length;
  uint8_t start_offset;
} sx1262_radio_backend_rx_buffer_status_t;

typedef struct {
  int16_t rssi_dbm_x2;
  int16_t snr_db_x4;
} sx1262_radio_backend_packet_status_t;

void sx1262_radio_backend_error_clear(sx1262_radio_backend_error_t *error);
uint64_t sx1262_radio_backend_monotonic_us(void);

bool sx1262_radio_backend_initialize(const sx1262_radio_profile_t *profile,
                                     sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_set_standby(sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_set_packet_params(
    uint8_t payload_length, bool inverted_iq,
    sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_configure_irq(uint16_t irq_mask,
                                        sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_clear_irq(uint16_t irq_mask,
                                    sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_write_payload(const uint8_t *payload,
                                        uint8_t payload_length,
                                        sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_start_tx(uint32_t watchdog_rtc_steps,
                                   sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_start_continuous_rx(
    sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_wait_dio1(uint64_t deadline_monotonic_us,
                                    bool *out_observed, uint64_t *out_irq_at_us,
                                    sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_get_irq(uint16_t *out_irq_status,
                                  sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_handle_rx_done(sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_get_rx_buffer_status(
    sx1262_radio_backend_rx_buffer_status_t *out_status,
    sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_read_buffer(uint8_t offset, uint8_t *output,
                                      uint8_t output_length,
                                      sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_get_packet_status(
    sx1262_radio_backend_packet_status_t *out_status,
    sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_get_device_errors(
    uint16_t *out_device_errors, sx1262_radio_backend_error_t *error);
bool sx1262_radio_backend_set_sleep_cold(sx1262_radio_backend_error_t *error);

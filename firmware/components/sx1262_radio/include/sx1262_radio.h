#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "node_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SX1262_RADIO_MAX_PAYLOAD_SIZE 255U

/* Fixed LoRa v2 pilot PHY profile. */
#define SX1262_RADIO_FREQUENCY_HZ UINT32_C(868100000)
#define SX1262_RADIO_TX_POWER_DBM INT8_C(14)
#define SX1262_RADIO_SPREADING_FACTOR UINT8_C(7)
#define SX1262_RADIO_BANDWIDTH_HZ UINT32_C(125000)
#define SX1262_RADIO_CODING_RATE_DENOMINATOR UINT8_C(5)
#define SX1262_RADIO_PREAMBLE_SYMBOLS UINT16_C(8)
#define SX1262_RADIO_SYNC_WORD UINT8_C(0x12)
#define SX1262_RADIO_TX_RAMP_US UINT16_C(40)

/* CURAG_EDOM_RADIO error-code assignments. */
#define CURAG_ERADIO_EINVALID_ARGUMENT UINT16_C(1)
#define CURAG_ERADIO_EINVALID_STATE UINT16_C(2)
#define CURAG_ERADIO_EIO UINT16_C(3)
#define CURAG_ERADIO_EBUSY_TIMEOUT UINT16_C(4)
#define CURAG_ERADIO_ECOMMAND_STATUS UINT16_C(5)
#define CURAG_ERADIO_EDEADLINE UINT16_C(6)
#define CURAG_ERADIO_EUNEXPECTED_IRQ UINT16_C(7)
#define CURAG_ERADIO_EDEVICE_ERROR UINT16_C(8)

#define CURAG_RADIO_CONTEXT_V1 UINT8_C(1)
#define CURAG_RADIO_CONTEXT_V1_LENGTH UINT8_C(14)

typedef enum {
  CURAG_RADIO_STATE_UNTOUCHED = 0,
  CURAG_RADIO_STATE_READY = 1,
  CURAG_RADIO_STATE_FAILED = 2,
  CURAG_RADIO_STATE_SLEEPING = 3,
} curag_radio_state_t;

#define CURAG_RADIO_CONTEXT_CHIP_STATUS_VALID UINT8_C(1U << 0U)
#define CURAG_RADIO_CONTEXT_IRQ_STATUS_VALID UINT8_C(1U << 1U)
#define CURAG_RADIO_CONTEXT_DEVICE_ERRORS_VALID UINT8_C(1U << 2U)
#define CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED UINT8_C(1U << 3U)

typedef enum {
  CURAG_RADIO_BACKEND_STATUS_NONE = 0,
  CURAG_RADIO_BACKEND_STATUS_ESP_ERR = 1,
  CURAG_RADIO_BACKEND_STATUS_SX1262_DRIVER = 2,
} curag_radio_backend_status_kind_t;

typedef enum {
  CURAG_RADIO_STAGE_NONE = 0,
  CURAG_RADIO_STAGE_STATE_CHECK = 1,
  CURAG_RADIO_STAGE_VALIDATE_INPUT = 2,
  CURAG_RADIO_STAGE_CONFIGURE_GPIO = 3,
  CURAG_RADIO_STAGE_CONFIGURE_SPI = 4,
  CURAG_RADIO_STAGE_RESET = 5,
  CURAG_RADIO_STAGE_WAKE = 6,
  CURAG_RADIO_STAGE_WAIT_BUSY = 7,
  CURAG_RADIO_STAGE_WRITE_COMMAND = 8,
  CURAG_RADIO_STAGE_READ_COMMAND = 9,
  CURAG_RADIO_STAGE_CONFIGURE_IRQ = 10,
  CURAG_RADIO_STAGE_WAIT_IRQ = 11,
  CURAG_RADIO_STAGE_READ_IRQ = 12,
  CURAG_RADIO_STAGE_CLEAR_IRQ = 13,
  CURAG_RADIO_STAGE_WRITE_BUFFER = 14,
  CURAG_RADIO_STAGE_READ_BUFFER = 15,
  CURAG_RADIO_STAGE_READ_PACKET_STATUS = 16,
  CURAG_RADIO_STAGE_DETACH_IRQ = 17,
  CURAG_RADIO_STAGE_CAPTURE_TIME = 18,
} curag_radio_stage_t;

typedef struct {
  bool tx_started;
  bool tx_done;
  uint64_t set_tx_at_us;
  uint64_t tx_done_at_us;
} sx1262_radio_tx_result_t;

typedef enum {
  SX1262_RADIO_RX_INVALID = 0,
  SX1262_RADIO_RX_PACKET = 1,
  SX1262_RADIO_RX_DEADLINE = 2,
} sx1262_radio_rx_outcome_t;

typedef struct {
  sx1262_radio_rx_outcome_t outcome;
  uint64_t rx_done_at_us;
  int16_t rssi_dbm_x2;
  int16_t snr_db_x4;
  uint8_t payload_length;
  uint8_t payload[SX1262_RADIO_MAX_PAYLOAD_SIZE];
} sx1262_radio_rx_result_t;

/*
 * Transmits one normal-IQ uplink before deadline_monotonic_us.
 *
 * payload:               Borrowed bytes, non-null for the duration of the
 *                        call.
 * payload_length:        Number of bytes to transmit, from 1 through 255.
 * deadline_monotonic_us: Absolute deadline in the node monotonic clock domain.
 * out_result:            Required caller-owned progress/timestamp output.
 * out_diag:              Optional caller-owned diagnostic detail.
 *
 * Returns CURAG_OK only after TX_DONE. tx_started is also true when SetTx
 * crossed SPI but a later failure left its effect uncertain, so the caller can
 * conservatively account for the possible attempt.
 */
err_curag_t sx1262_radio_transmit_uplink(const uint8_t *payload,
                                         size_t payload_length,
                                         uint64_t deadline_monotonic_us,
                                         sx1262_radio_tx_result_t *out_result,
                                         diagn_context_t *out_diag);

/*
 * Receives one inverted-IQ downlink under an absolute deadline.
 *
 * deadline_monotonic_us: Absolute deadline in the node monotonic clock domain.
 * out_result:            Required caller-owned packet/deadline output.
 * out_diag:              Optional caller-owned diagnostic detail.
 *
 * Returns CURAG_OK for both RX_PACKET and normal RX_DEADLINE. The radio must
 * already be READY following a successful transmit.
 */
err_curag_t
sx1262_radio_receive_downlink_until(uint64_t deadline_monotonic_us,
                                    sx1262_radio_rx_result_t *out_result,
                                    diagn_context_t *out_diag);

/*
 * Stops active operation through STDBY_RC and requests cold-start sleep.
 *
 * out_diag: Optional caller-owned diagnostic detail.
 *
 * The operation is an idempotent no-op before hardware is touched and after a
 * successful sleep. A returned error is diagnostic only; callers must still
 * put the MCU to sleep.
 */
err_curag_t sx1262_radio_sleep(diagn_context_t *out_diag);

#ifdef __cplusplus
}
#endif

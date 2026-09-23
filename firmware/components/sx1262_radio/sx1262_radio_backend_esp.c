#include "sx1262_radio_backend.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sx126x.h"
#include "sx126x_hal.h"

#define RADIO_SPI_HOST SPI2_HOST
#define RADIO_SPI_CLOCK_HZ 8000000
#define RADIO_SPI_MAX_TRANSFER_SIZE 260U
#define RADIO_BUSY_TIMEOUT_US UINT64_C(10000)
#define RADIO_RESET_BUSY_TIMEOUT_US UINT64_C(20000)
#define RADIO_BUSY_POLL_US UINT32_C(50)

#define SX1262_OPCODE_SET_SLEEP UINT8_C(0x84)
#define SX1262_OPCODE_SET_STANDBY UINT8_C(0x80)
#define SX1262_OPCODE_SET_TX UINT8_C(0x83)
#define SX1262_OPCODE_SET_RX UINT8_C(0x82)
#define SX1262_OPCODE_SET_REGULATOR_MODE UINT8_C(0x96)
#define SX1262_OPCODE_CALIBRATE UINT8_C(0x89)
#define SX1262_OPCODE_CALIBRATE_IMAGE UINT8_C(0x98)
#define SX1262_OPCODE_SET_PA_CONFIG UINT8_C(0x95)
#define SX1262_OPCODE_SET_FALLBACK UINT8_C(0x93)
#define SX1262_OPCODE_WRITE_REGISTER UINT8_C(0x0d)
#define SX1262_OPCODE_WRITE_BUFFER UINT8_C(0x0e)
#define SX1262_OPCODE_READ_BUFFER UINT8_C(0x1e)
#define SX1262_OPCODE_SET_DIO_IRQ_PARAMS UINT8_C(0x08)
#define SX1262_OPCODE_GET_IRQ_STATUS UINT8_C(0x12)
#define SX1262_OPCODE_CLEAR_IRQ_STATUS UINT8_C(0x02)
#define SX1262_OPCODE_SET_DIO2_RF_SWITCH UINT8_C(0x9d)
#define SX1262_OPCODE_SET_DIO3_TCXO UINT8_C(0x97)
#define SX1262_OPCODE_SET_RF_FREQUENCY UINT8_C(0x86)
#define SX1262_OPCODE_SET_PACKET_TYPE UINT8_C(0x8a)
#define SX1262_OPCODE_SET_TX_PARAMS UINT8_C(0x8e)
#define SX1262_OPCODE_SET_MODULATION_PARAMS UINT8_C(0x8b)
#define SX1262_OPCODE_SET_PACKET_PARAMS UINT8_C(0x8c)
#define SX1262_OPCODE_SET_BUFFER_BASE UINT8_C(0x8f)
#define SX1262_OPCODE_GET_RX_BUFFER_STATUS UINT8_C(0x13)
#define SX1262_OPCODE_GET_PACKET_STATUS UINT8_C(0x14)
#define SX1262_OPCODE_GET_STATUS UINT8_C(0xc0)
#define SX1262_OPCODE_GET_DEVICE_ERRORS UINT8_C(0x17)
#define SX1262_OPCODE_CLEAR_DEVICE_ERRORS UINT8_C(0x07)

typedef struct {
  spi_device_handle_t spi;
  SemaphoreHandle_t dio1_semaphore;
  StaticSemaphore_t dio1_semaphore_storage;
  volatile uint64_t dio1_at_us;
  sx1262_radio_profile_t profile;
  sx1262_radio_backend_error_t last_failure;
  uint8_t active_stage;
  uint8_t active_opcode;
  bool active_command_transferred;
  bool hardware_touched;
  bool gpio_ready;
  bool spi_ready;
} sx1262_radio_esp_context_t;

static sx1262_radio_esp_context_t s_backend;

static void set_semantic_failure(uint16_t error_code, curag_radio_stage_t stage,
                                 uint8_t command_opcode) {
  sx1262_radio_backend_error_clear(&s_backend.last_failure);
  s_backend.last_failure.error_code = error_code;
  s_backend.last_failure.stage = (uint8_t)stage;
  s_backend.last_failure.command_opcode = command_opcode;
  if (s_backend.hardware_touched) {
    s_backend.last_failure.flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED;
  }
}

static void set_esp_failure(esp_err_t status, curag_radio_stage_t stage,
                            uint8_t command_opcode) {
  set_semantic_failure(CURAG_ERADIO_EIO, stage, command_opcode);
  s_backend.last_failure.backend_status_kind =
      CURAG_RADIO_BACKEND_STATUS_ESP_ERR;
  s_backend.last_failure.backend_status = status;
}

static void prepare_driver_call(curag_radio_stage_t stage,
                                uint8_t command_opcode) {
  sx1262_radio_backend_error_clear(&s_backend.last_failure);
  s_backend.active_stage = (uint8_t)stage;
  s_backend.active_opcode = command_opcode;
  s_backend.active_command_transferred = false;
}

static void copy_last_failure(sx1262_radio_backend_error_t *error) {
  if (error != NULL) {
    *error = s_backend.last_failure;
  }
}

static bool finish_driver_call(sx126x_status_t status,
                               curag_radio_stage_t fallback_stage,
                               uint8_t fallback_opcode,
                               sx1262_radio_backend_error_t *error) {
  if (status == SX126X_STATUS_OK) {
    return true;
  }
  if (s_backend.last_failure.error_code == CURAG_ECODE_NONE) {
    set_semantic_failure(CURAG_ERADIO_ECOMMAND_STATUS, fallback_stage,
                         fallback_opcode);
    s_backend.last_failure.backend_status_kind =
        CURAG_RADIO_BACKEND_STATUS_SX1262_DRIVER;
    s_backend.last_failure.backend_status = (int32_t)status;
  }
  copy_last_failure(error);
  return false;
}

static bool wait_while_busy(uint64_t timeout_us) {
  const uint64_t started_at = sx1262_radio_backend_monotonic_us();
  while (gpio_get_level((gpio_num_t)CONFIG_CURA_SX1262_BUSY_GPIO) != 0) {
    const uint64_t now = sx1262_radio_backend_monotonic_us();
    if (now - started_at >= timeout_us) {
      set_semantic_failure(CURAG_ERADIO_EBUSY_TIMEOUT,
                           CURAG_RADIO_STAGE_WAIT_BUSY,
                           s_backend.active_opcode);
      return false;
    }
    esp_rom_delay_us(RADIO_BUSY_POLL_US);
  }
  return true;
}

static bool spi_transfer(const uint8_t *transmit, uint8_t *receive,
                         size_t length, curag_radio_stage_t stage,
                         uint8_t command_opcode) {
  spi_transaction_t transaction = {0};
  transaction.length = length * 8U;
  transaction.rxlength = receive == NULL ? 0U : length * 8U;
  transaction.tx_buffer = transmit;
  transaction.rx_buffer = receive;
  const esp_err_t status = spi_device_transmit(s_backend.spi, &transaction);
  if (status != ESP_OK) {
    set_esp_failure(status, stage, command_opcode);
    return false;
  }
  return true;
}

static void IRAM_ATTR dio1_isr(void *argument) {
  sx1262_radio_esp_context_t *const context = argument;
  context->dio1_at_us = (uint64_t)esp_timer_get_time();
  BaseType_t higher_priority_woken = pdFALSE;
  xSemaphoreGiveFromISR(context->dio1_semaphore, &higher_priority_woken);
  if (higher_priority_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static void discard_stale_dio1_event(void) {
  /* Only a new arm discards stale state; IRQ acknowledgement must not. */
  while (xSemaphoreTake(s_backend.dio1_semaphore, 0U) == pdTRUE) {
  }
  s_backend.dio1_at_us = 0U;
}

static bool configure_gpio(sx1262_radio_backend_error_t *error) {
  gpio_config_t reset_config = {
      .pin_bit_mask = UINT64_C(1) << CONFIG_CURA_SX1262_RESET_GPIO,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t status = gpio_config(&reset_config);
  if (status != ESP_OK) {
    set_esp_failure(status, CURAG_RADIO_STAGE_CONFIGURE_GPIO, 0U);
    copy_last_failure(error);
    return false;
  }
  s_backend.hardware_touched = true;

  status = gpio_set_level((gpio_num_t)CONFIG_CURA_SX1262_RESET_GPIO, 1);
  if (status != ESP_OK) {
    set_esp_failure(status, CURAG_RADIO_STAGE_CONFIGURE_GPIO, 0U);
    copy_last_failure(error);
    return false;
  }

  gpio_config_t input_config = {
      .pin_bit_mask = (UINT64_C(1) << CONFIG_CURA_SX1262_BUSY_GPIO) |
                      (UINT64_C(1) << CONFIG_CURA_SX1262_DIO1_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  status = gpio_config(&input_config);
  if (status != ESP_OK) {
    set_esp_failure(status, CURAG_RADIO_STAGE_CONFIGURE_GPIO, 0U);
    copy_last_failure(error);
    return false;
  }

  s_backend.dio1_semaphore =
      xSemaphoreCreateBinaryStatic(&s_backend.dio1_semaphore_storage);
  if (s_backend.dio1_semaphore == NULL) {
    set_semantic_failure(CURAG_ERADIO_EIO, CURAG_RADIO_STAGE_CONFIGURE_IRQ, 0U);
    copy_last_failure(error);
    return false;
  }

  status = gpio_install_isr_service(0);
  if (status != ESP_OK && status != ESP_ERR_INVALID_STATE) {
    set_esp_failure(status, CURAG_RADIO_STAGE_CONFIGURE_IRQ, 0U);
    copy_last_failure(error);
    return false;
  }
  status = gpio_isr_handler_add((gpio_num_t)CONFIG_CURA_SX1262_DIO1_GPIO,
                                dio1_isr, &s_backend);
  if (status != ESP_OK) {
    set_esp_failure(status, CURAG_RADIO_STAGE_CONFIGURE_IRQ, 0U);
    copy_last_failure(error);
    return false;
  }
  status = gpio_set_intr_type((gpio_num_t)CONFIG_CURA_SX1262_DIO1_GPIO,
                              GPIO_INTR_POSEDGE);
  if (status != ESP_OK) {
    set_esp_failure(status, CURAG_RADIO_STAGE_CONFIGURE_IRQ, 0U);
    copy_last_failure(error);
    return false;
  }
  s_backend.gpio_ready = true;
  return true;
}

static bool configure_spi(sx1262_radio_backend_error_t *error) {
  const spi_bus_config_t bus_config = {
      .mosi_io_num = CONFIG_CURA_SX1262_MOSI_GPIO,
      .miso_io_num = CONFIG_CURA_SX1262_MISO_GPIO,
      .sclk_io_num = CONFIG_CURA_SX1262_SCLK_GPIO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = (int)RADIO_SPI_MAX_TRANSFER_SIZE,
  };
  esp_err_t status =
      spi_bus_initialize(RADIO_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
  if (status != ESP_OK) {
    set_esp_failure(status, CURAG_RADIO_STAGE_CONFIGURE_SPI, 0U);
    copy_last_failure(error);
    return false;
  }

  const spi_device_interface_config_t device_config = {
      .clock_speed_hz = RADIO_SPI_CLOCK_HZ,
      .mode = 0,
      .spics_io_num = CONFIG_CURA_SX1262_CS_GPIO,
      .queue_size = 1,
  };
  status = spi_bus_add_device(RADIO_SPI_HOST, &device_config, &s_backend.spi);
  if (status != ESP_OK) {
    set_esp_failure(status, CURAG_RADIO_STAGE_CONFIGURE_SPI, 0U);
    copy_last_failure(error);
    return false;
  }
  s_backend.spi_ready = true;
  return true;
}

static bool profile_is_supported(const sx1262_radio_profile_t *profile) {
  return profile != NULL &&
         profile->frequency_hz == SX1262_RADIO_FREQUENCY_HZ &&
         profile->bandwidth_hz == SX1262_RADIO_BANDWIDTH_HZ &&
         profile->preamble_symbols == SX1262_RADIO_PREAMBLE_SYMBOLS &&
         profile->ramp_us == SX1262_RADIO_TX_RAMP_US &&
         profile->radiated_tx_power_dbm == SX1262_RADIO_TX_POWER_DBM &&
         profile->spreading_factor == SX1262_RADIO_SPREADING_FACTOR &&
         profile->coding_rate_denominator ==
             SX1262_RADIO_CODING_RATE_DENOMINATOR &&
         profile->sync_word == SX1262_RADIO_SYNC_WORD &&
         profile->tcxo_voltage == (uint8_t)SX126X_TCXO_CTRL_1_7V &&
         profile->dcdc_regulator && profile->dio2_rf_switch &&
         profile->dio3_tcxo;
}

uint64_t sx1262_radio_backend_monotonic_us(void) {
  const int64_t now = esp_timer_get_time();
  return now < 0 ? 0U : (uint64_t)now;
}

sx126x_hal_status_t sx126x_hal_write(const void *context,
                                     const uint8_t *command,
                                     const uint16_t command_length,
                                     const uint8_t *data,
                                     const uint16_t data_length) {
  (void)context;
  if (!s_backend.spi_ready || command == NULL || command_length == 0U ||
      ((data == NULL) && data_length != 0U) ||
      (size_t)command_length + (size_t)data_length >
          RADIO_SPI_MAX_TRANSFER_SIZE) {
    set_semantic_failure(CURAG_ERADIO_EIO, CURAG_RADIO_STAGE_WRITE_COMMAND,
                         command == NULL ? 0U : command[0]);
    return SX126X_HAL_STATUS_ERROR;
  }

  s_backend.active_opcode = command[0];
  if (!wait_while_busy(RADIO_BUSY_TIMEOUT_US)) {
    return SX126X_HAL_STATUS_ERROR;
  }

  uint8_t transmit[RADIO_SPI_MAX_TRANSFER_SIZE] = {0};
  memcpy(transmit, command, command_length);
  if (data_length > 0U) {
    memcpy(&transmit[command_length], data, data_length);
  }
  const size_t total_length = (size_t)command_length + (size_t)data_length;
  if (!spi_transfer(transmit, NULL, total_length,
                    (curag_radio_stage_t)s_backend.active_stage, command[0])) {
    return SX126X_HAL_STATUS_ERROR;
  }
  s_backend.active_command_transferred = true;

  if (command[0] != SX1262_OPCODE_SET_SLEEP &&
      !wait_while_busy(RADIO_BUSY_TIMEOUT_US)) {
    return SX126X_HAL_STATUS_ERROR;
  }
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length) {
  (void)context;
  if (!s_backend.spi_ready || command == NULL || command_length == 0U ||
      ((data == NULL) && data_length != 0U) ||
      (size_t)command_length + (size_t)data_length >
          RADIO_SPI_MAX_TRANSFER_SIZE) {
    set_semantic_failure(CURAG_ERADIO_EIO, CURAG_RADIO_STAGE_READ_COMMAND,
                         command == NULL ? 0U : command[0]);
    return SX126X_HAL_STATUS_ERROR;
  }

  s_backend.active_opcode = command[0];
  if (!wait_while_busy(RADIO_BUSY_TIMEOUT_US)) {
    return SX126X_HAL_STATUS_ERROR;
  }

  uint8_t transmit[RADIO_SPI_MAX_TRANSFER_SIZE] = {0};
  uint8_t receive[RADIO_SPI_MAX_TRANSFER_SIZE] = {0};
  memcpy(transmit, command, command_length);
  const size_t total_length = (size_t)command_length + (size_t)data_length;
  if (!spi_transfer(transmit, receive, total_length,
                    (curag_radio_stage_t)s_backend.active_stage, command[0])) {
    return SX126X_HAL_STATUS_ERROR;
  }
  s_backend.active_command_transferred = true;
  if (data_length > 0U) {
    memcpy(data, &receive[command_length], data_length);
  }
  if (!wait_while_busy(RADIO_BUSY_TIMEOUT_US)) {
    return SX126X_HAL_STATUS_ERROR;
  }
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context) {
  (void)context;
  if (!s_backend.gpio_ready) {
    set_semantic_failure(CURAG_ERADIO_EIO, CURAG_RADIO_STAGE_RESET, 0U);
    return SX126X_HAL_STATUS_ERROR;
  }
  esp_err_t status =
      gpio_set_level((gpio_num_t)CONFIG_CURA_SX1262_RESET_GPIO, 0);
  if (status != ESP_OK) {
    set_esp_failure(status, CURAG_RADIO_STAGE_RESET, 0U);
    return SX126X_HAL_STATUS_ERROR;
  }
  esp_rom_delay_us(200U);
  status = gpio_set_level((gpio_num_t)CONFIG_CURA_SX1262_RESET_GPIO, 1);
  if (status != ESP_OK) {
    set_esp_failure(status, CURAG_RADIO_STAGE_RESET, 0U);
    return SX126X_HAL_STATUS_ERROR;
  }
  if (!wait_while_busy(RADIO_RESET_BUSY_TIMEOUT_US)) {
    return SX126X_HAL_STATUS_ERROR;
  }
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context) {
  (void)context;
  if (!s_backend.spi_ready) {
    set_semantic_failure(CURAG_ERADIO_EIO, CURAG_RADIO_STAGE_WAKE, 0U);
    return SX126X_HAL_STATUS_ERROR;
  }
  const uint8_t transmit[2] = {UINT8_C(0xc0), UINT8_C(0x00)};
  if (!spi_transfer(transmit, NULL, sizeof(transmit), CURAG_RADIO_STAGE_WAKE,
                    UINT8_C(0xc0))) {
    return SX126X_HAL_STATUS_ERROR;
  }
  s_backend.active_command_transferred = true;
  if (!wait_while_busy(RADIO_BUSY_TIMEOUT_US)) {
    return SX126X_HAL_STATUS_ERROR;
  }
  return SX126X_HAL_STATUS_OK;
}

static sx1262_radio_backend_call_result_t
finalize_driver_call(sx126x_status_t status, curag_radio_stage_t stage,
                     uint8_t opcode, sx1262_radio_backend_error_t *error) {
  const bool command_transferred = s_backend.active_command_transferred;
  if (!finish_driver_call(status, stage, opcode, error)) {
    return command_transferred ? SX1262_COMMAND_UNCERTAIN
                               : SX1262_COMMAND_FAILED;
  }
  if (opcode == 0U || opcode == SX1262_OPCODE_SET_SLEEP) {
    /* Reset is HAL-confirmed; a sleeping radio cannot answer GetStatus. */
    return SX1262_COMMAND_CONFIRMED;
  }

  sx126x_chip_status_t chip_status = {0};
  prepare_driver_call(CURAG_RADIO_STAGE_READ_COMMAND, SX1262_OPCODE_GET_STATUS);
  if (!finish_driver_call(sx126x_get_status(&s_backend, &chip_status),
                          CURAG_RADIO_STAGE_READ_COMMAND,
                          SX1262_OPCODE_GET_STATUS, error)) {
    return command_transferred ? SX1262_COMMAND_UNCERTAIN
                               : SX1262_COMMAND_FAILED;
  }
  if (chip_status.cmd_status == SX126X_CMD_STATUS_CMD_TIMEOUT ||
      chip_status.cmd_status == SX126X_CMD_STATUS_CMD_PROCESS_ERROR ||
      chip_status.cmd_status == SX126X_CMD_STATUS_CMD_EXEC_FAILURE) {
    set_semantic_failure(CURAG_ERADIO_ECOMMAND_STATUS, stage, opcode);
    s_backend.last_failure.flags =
        (uint8_t)(s_backend.last_failure.flags |
                  CURAG_RADIO_CONTEXT_CHIP_STATUS_VALID);
    s_backend.last_failure.chip_status =
        (uint8_t)(((uint8_t)chip_status.chip_mode << 4U) |
                  ((uint8_t)chip_status.cmd_status << 1U));
    copy_last_failure(error);
    return SX1262_COMMAND_FAILED;
  }
  return SX1262_COMMAND_CONFIRMED;
}

bool sx1262_radio_backend_initialize(const sx1262_radio_profile_t *profile,
                                     sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  memset(&s_backend, 0, sizeof(s_backend));
  if (!profile_is_supported(profile)) {
    set_semantic_failure(CURAG_ERADIO_EINVALID_ARGUMENT,
                         CURAG_RADIO_STAGE_VALIDATE_INPUT, 0U);
    copy_last_failure(error);
    return false;
  }
  s_backend.profile = *profile;
  if (!configure_gpio(error) || !configure_spi(error)) {
    return false;
  }

  prepare_driver_call(CURAG_RADIO_STAGE_RESET, 0U);
  if (finalize_driver_call(sx126x_reset(&s_backend), CURAG_RADIO_STAGE_RESET,
                           0U, error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_STANDBY);
  if (finalize_driver_call(
          sx126x_set_standby(&s_backend, SX126X_STANDBY_CFG_RC),
          CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_STANDBY,
          error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_WRITE_REGISTER);
  if (finalize_driver_call(
          sx126x_cfg_tx_clamp(&s_backend), CURAG_RADIO_STAGE_WRITE_COMMAND,
          SX1262_OPCODE_WRITE_REGISTER, error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_DIO3_TCXO);
  if (finalize_driver_call(
          sx126x_set_dio3_as_tcxo_ctrl(
              &s_backend, (sx126x_tcxo_ctrl_voltages_t)profile->tcxo_voltage,
              (uint32_t)profile->tcxo_startup_ms * UINT32_C(64)),
          CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_DIO3_TCXO,
          error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_CLEAR_DEVICE_ERRORS);
  if (finalize_driver_call(sx126x_clear_device_errors(&s_backend),
                           CURAG_RADIO_STAGE_WRITE_COMMAND,
                           SX1262_OPCODE_CLEAR_DEVICE_ERRORS,
                           error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_CALIBRATE);
  if (finalize_driver_call(sx126x_cal(&s_backend, SX126X_CAL_ALL),
                           CURAG_RADIO_STAGE_WRITE_COMMAND,
                           SX1262_OPCODE_CALIBRATE,
                           error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_CALIBRATE_IMAGE);
  if (finalize_driver_call(sx126x_cal_img_in_mhz(&s_backend, 863U, 870U),
                           CURAG_RADIO_STAGE_WRITE_COMMAND,
                           SX1262_OPCODE_CALIBRATE_IMAGE,
                           error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_DIO2_RF_SWITCH);
  if (finalize_driver_call(sx126x_set_dio2_as_rf_sw_ctrl(&s_backend, true),
                           CURAG_RADIO_STAGE_WRITE_COMMAND,
                           SX1262_OPCODE_SET_DIO2_RF_SWITCH,
                           error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_REGULATOR_MODE);
  if (finalize_driver_call(
          sx126x_set_reg_mode(&s_backend, SX126X_REG_MODE_DCDC),
          CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_REGULATOR_MODE,
          error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_BUFFER_BASE);
  if (finalize_driver_call(sx126x_set_buffer_base_address(&s_backend, 0U, 0U),
                           CURAG_RADIO_STAGE_WRITE_COMMAND,
                           SX1262_OPCODE_SET_BUFFER_BASE,
                           error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_PACKET_TYPE);
  if (finalize_driver_call(
          sx126x_set_pkt_type(&s_backend, SX126X_PKT_TYPE_LORA),
          CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_PACKET_TYPE,
          error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_RF_FREQUENCY);
  if (finalize_driver_call(
          sx126x_set_rf_freq(&s_backend, profile->frequency_hz),
          CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_RF_FREQUENCY,
          error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }

  const sx126x_mod_params_lora_t modulation = {
      .sf = SX126X_LORA_SF7,
      .bw = SX126X_LORA_BW_125,
      .cr = SX126X_LORA_CR_4_5,
      .ldro = 0U,
  };
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_MODULATION_PARAMS);
  if (finalize_driver_call(sx126x_set_lora_mod_params(&s_backend, &modulation),
                           CURAG_RADIO_STAGE_WRITE_COMMAND,
                           SX1262_OPCODE_SET_MODULATION_PARAMS,
                           error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }

  const sx126x_pa_cfg_params_t pa = {
      .pa_duty_cycle = profile->pa_duty_cycle,
      .hp_max = profile->pa_hp_max,
      .device_sel = profile->pa_device_sel,
      .pa_lut = profile->pa_lut,
  };
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_PA_CONFIG);
  if (finalize_driver_call(
          sx126x_set_pa_cfg(&s_backend, &pa), CURAG_RADIO_STAGE_WRITE_COMMAND,
          SX1262_OPCODE_SET_PA_CONFIG, error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_TX_PARAMS);
  if (finalize_driver_call(
          sx126x_set_tx_params(&s_backend, profile->tx_params_power_dbm,
                               SX126X_RAMP_40_US),
          CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_TX_PARAMS,
          error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_WRITE_REGISTER);
  if (finalize_driver_call(
          sx126x_set_lora_sync_word(&s_backend, profile->sync_word),
          CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_WRITE_REGISTER,
          error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_WRITE_REGISTER);
  if (finalize_driver_call(sx126x_cfg_rx_boosted(&s_backend, true),
                           CURAG_RADIO_STAGE_WRITE_COMMAND,
                           SX1262_OPCODE_WRITE_REGISTER,
                           error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_FALLBACK);
  if (finalize_driver_call(
          sx126x_set_rx_tx_fallback_mode(&s_backend, SX126X_FALLBACK_STDBY_RC),
          CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_FALLBACK,
          error) != SX1262_COMMAND_CONFIRMED) {
    return false;
  }

  uint16_t device_errors = 0U;
  if (sx1262_radio_backend_get_device_errors(&device_errors, error) !=
      SX1262_COMMAND_CONFIRMED) {
    return false;
  }
  if (device_errors != 0U) {
    set_semantic_failure(CURAG_ERADIO_EDEVICE_ERROR,
                         CURAG_RADIO_STAGE_READ_COMMAND,
                         SX1262_OPCODE_GET_DEVICE_ERRORS);
    s_backend.last_failure.flags =
        (uint8_t)(s_backend.last_failure.flags |
                  CURAG_RADIO_CONTEXT_DEVICE_ERRORS_VALID);
    s_backend.last_failure.device_errors = device_errors;
    copy_last_failure(error);
    return false;
  }
  return true;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_set_standby(sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_STANDBY);
  return finalize_driver_call(
      sx126x_set_standby(&s_backend, SX126X_STANDBY_CFG_RC),
      CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_STANDBY, error);
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_set_packet_params(uint8_t payload_length, bool inverted_iq,
                                       sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  const sx126x_pkt_params_lora_t packet = {
      .preamble_len_in_symb = s_backend.profile.preamble_symbols,
      .header_type = s_backend.profile.explicit_header
                         ? SX126X_LORA_PKT_EXPLICIT
                         : SX126X_LORA_PKT_IMPLICIT,
      .pld_len_in_bytes = payload_length,
      .crc_is_on = s_backend.profile.payload_crc,
      .invert_iq_is_on = inverted_iq,
  };
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_SET_PACKET_PARAMS);
  return finalize_driver_call(sx126x_set_lora_pkt_params(&s_backend, &packet),
                              CURAG_RADIO_STAGE_WRITE_COMMAND,
                              SX1262_OPCODE_SET_PACKET_PARAMS, error);
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_configure_irq(uint16_t irq_mask,
                                   sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  prepare_driver_call(CURAG_RADIO_STAGE_CONFIGURE_IRQ,
                      SX1262_OPCODE_SET_DIO_IRQ_PARAMS);
  return finalize_driver_call(
      sx126x_set_dio_irq_params(&s_backend, irq_mask, irq_mask, 0U, 0U),
      CURAG_RADIO_STAGE_CONFIGURE_IRQ, SX1262_OPCODE_SET_DIO_IRQ_PARAMS, error);
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_clear_irq(uint16_t irq_mask,
                               sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  prepare_driver_call(CURAG_RADIO_STAGE_CLEAR_IRQ,
                      SX1262_OPCODE_CLEAR_IRQ_STATUS);
  return finalize_driver_call(sx126x_clear_irq_status(&s_backend, irq_mask),
                              CURAG_RADIO_STAGE_CLEAR_IRQ,
                              SX1262_OPCODE_CLEAR_IRQ_STATUS, error);
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_write_payload(const uint8_t *payload,
                                   uint8_t payload_length,
                                   sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_BUFFER,
                      SX1262_OPCODE_WRITE_BUFFER);
  return finalize_driver_call(
      sx126x_write_buffer(&s_backend, 0U, payload, payload_length),
      CURAG_RADIO_STAGE_WRITE_BUFFER, SX1262_OPCODE_WRITE_BUFFER, error);
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_start_tx(uint32_t watchdog_rtc_steps,
                              sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  discard_stale_dio1_event();
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_TX);
  return finalize_driver_call(
      sx126x_set_tx_with_timeout_in_rtc_step(&s_backend, watchdog_rtc_steps),
      CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_TX, error);
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_start_single_rx(sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  discard_stale_dio1_event();
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_RX);
  return finalize_driver_call(
      sx126x_set_rx_with_timeout_in_rtc_step(&s_backend, SX126X_RX_SINGLE_MODE),
      CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_RX, error);
}

bool sx1262_radio_backend_wait_dio1(uint64_t deadline_monotonic_us,
                                    bool *out_observed, uint64_t *out_irq_at_us,
                                    sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  if (out_observed == NULL || out_irq_at_us == NULL ||
      s_backend.dio1_semaphore == NULL) {
    set_semantic_failure(CURAG_ERADIO_EINVALID_ARGUMENT,
                         CURAG_RADIO_STAGE_WAIT_IRQ, 0U);
    copy_last_failure(error);
    return false;
  }
  *out_observed = false;
  *out_irq_at_us = 0U;

  while (true) {
    /* Pending ISR time wins even when the task resumes at the deadline. */
    if (xSemaphoreTake(s_backend.dio1_semaphore, 0U) == pdTRUE) {
      *out_observed = true;
      *out_irq_at_us = s_backend.dio1_at_us;
      return true;
    }
    const uint64_t now = sx1262_radio_backend_monotonic_us();
    if (now >= deadline_monotonic_us) {
      return true;
    }
    const uint64_t remaining_us = deadline_monotonic_us - now;
    uint64_t remaining_ms = remaining_us / UINT64_C(1000);
    if ((remaining_us % UINT64_C(1000)) != 0U) {
      ++remaining_ms;
    }
    if (remaining_ms > UINT32_MAX) {
      remaining_ms = UINT32_MAX;
    }
    TickType_t wait_ticks = pdMS_TO_TICKS((uint32_t)remaining_ms);
    if (wait_ticks == 0U) {
      wait_ticks = 1U;
    }
    if (xSemaphoreTake(s_backend.dio1_semaphore, wait_ticks) == pdTRUE) {
      *out_observed = true;
      *out_irq_at_us = s_backend.dio1_at_us;
      return true;
    }
  }
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_get_irq(uint16_t *out_irq_status,
                             sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  if (out_irq_status == NULL) {
    set_semantic_failure(CURAG_ERADIO_EINVALID_ARGUMENT,
                         CURAG_RADIO_STAGE_READ_IRQ,
                         SX1262_OPCODE_GET_IRQ_STATUS);
    copy_last_failure(error);
    return SX1262_COMMAND_FAILED;
  }
  sx126x_irq_mask_t irq_status = 0U;
  prepare_driver_call(CURAG_RADIO_STAGE_READ_IRQ, SX1262_OPCODE_GET_IRQ_STATUS);
  const sx1262_radio_backend_call_result_t result = finalize_driver_call(
      sx126x_get_irq_status(&s_backend, &irq_status),
      CURAG_RADIO_STAGE_READ_IRQ, SX1262_OPCODE_GET_IRQ_STATUS, error);
  if (result != SX1262_COMMAND_CONFIRMED) {
    return result;
  }
  *out_irq_status = irq_status;
  return SX1262_COMMAND_CONFIRMED;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_handle_rx_done(sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND,
                      SX1262_OPCODE_WRITE_REGISTER);
  return finalize_driver_call(sx126x_handle_rx_done(&s_backend),
                              CURAG_RADIO_STAGE_WRITE_COMMAND,
                              SX1262_OPCODE_WRITE_REGISTER, error);
}

sx1262_radio_backend_call_result_t sx1262_radio_backend_get_rx_buffer_status(
    sx1262_radio_backend_rx_buffer_status_t *out_status,
    sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  if (out_status == NULL) {
    set_semantic_failure(CURAG_ERADIO_EINVALID_ARGUMENT,
                         CURAG_RADIO_STAGE_READ_BUFFER,
                         SX1262_OPCODE_GET_RX_BUFFER_STATUS);
    copy_last_failure(error);
    return SX1262_COMMAND_FAILED;
  }
  sx126x_rx_buffer_status_t status = {0};
  prepare_driver_call(CURAG_RADIO_STAGE_READ_BUFFER,
                      SX1262_OPCODE_GET_RX_BUFFER_STATUS);
  const sx1262_radio_backend_call_result_t result = finalize_driver_call(
      sx126x_get_rx_buffer_status(&s_backend, &status),
      CURAG_RADIO_STAGE_READ_BUFFER, SX1262_OPCODE_GET_RX_BUFFER_STATUS, error);
  if (result != SX1262_COMMAND_CONFIRMED) {
    return result;
  }
  out_status->payload_length = status.pld_len_in_bytes;
  out_status->start_offset = status.buffer_start_pointer;
  return SX1262_COMMAND_CONFIRMED;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_read_buffer(uint8_t offset, uint8_t *output,
                                 uint8_t output_length,
                                 sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  prepare_driver_call(CURAG_RADIO_STAGE_READ_BUFFER, SX1262_OPCODE_READ_BUFFER);
  return finalize_driver_call(
      sx126x_read_buffer(&s_backend, offset, output, output_length),
      CURAG_RADIO_STAGE_READ_BUFFER, SX1262_OPCODE_READ_BUFFER, error);
}

sx1262_radio_backend_call_result_t sx1262_radio_backend_get_packet_status(
    sx1262_radio_backend_packet_status_t *out_status,
    sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  if (out_status == NULL) {
    set_semantic_failure(CURAG_ERADIO_EINVALID_ARGUMENT,
                         CURAG_RADIO_STAGE_READ_PACKET_STATUS,
                         SX1262_OPCODE_GET_PACKET_STATUS);
    copy_last_failure(error);
    return SX1262_COMMAND_FAILED;
  }

  const uint8_t command[2] = {SX1262_OPCODE_GET_PACKET_STATUS, 0U};
  uint8_t raw_status[3] = {0};
  prepare_driver_call(CURAG_RADIO_STAGE_READ_PACKET_STATUS,
                      SX1262_OPCODE_GET_PACKET_STATUS);
  const sx126x_status_t status = (sx126x_status_t)sx126x_hal_read(
      &s_backend, command, sizeof(command), raw_status, sizeof(raw_status));
  const sx1262_radio_backend_call_result_t result =
      finalize_driver_call(status, CURAG_RADIO_STAGE_READ_PACKET_STATUS,
                           SX1262_OPCODE_GET_PACKET_STATUS, error);
  if (result != SX1262_COMMAND_CONFIRMED) {
    return result;
  }
  out_status->rssi_dbm_x2 = -(int16_t)raw_status[0];
  out_status->snr_db_x4 = (int16_t)(int8_t)raw_status[1];
  return SX1262_COMMAND_CONFIRMED;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_get_device_errors(uint16_t *out_device_errors,
                                       sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  if (out_device_errors == NULL) {
    set_semantic_failure(CURAG_ERADIO_EINVALID_ARGUMENT,
                         CURAG_RADIO_STAGE_READ_COMMAND,
                         SX1262_OPCODE_GET_DEVICE_ERRORS);
    copy_last_failure(error);
    return SX1262_COMMAND_FAILED;
  }
  sx126x_errors_mask_t device_errors = 0U;
  prepare_driver_call(CURAG_RADIO_STAGE_READ_COMMAND,
                      SX1262_OPCODE_GET_DEVICE_ERRORS);
  const sx1262_radio_backend_call_result_t result = finalize_driver_call(
      sx126x_get_device_errors(&s_backend, &device_errors),
      CURAG_RADIO_STAGE_READ_COMMAND, SX1262_OPCODE_GET_DEVICE_ERRORS, error);
  if (result != SX1262_COMMAND_CONFIRMED) {
    return result;
  }
  *out_device_errors = device_errors;
  return SX1262_COMMAND_CONFIRMED;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_set_sleep_cold(sx1262_radio_backend_error_t *error) {
  sx1262_radio_backend_error_clear(error);
  prepare_driver_call(CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_SLEEP);
  return finalize_driver_call(
      sx126x_set_sleep(&s_backend, SX126X_SLEEP_CFG_COLD_START),
      CURAG_RADIO_STAGE_WRITE_COMMAND, SX1262_OPCODE_SET_SLEEP, error);
}

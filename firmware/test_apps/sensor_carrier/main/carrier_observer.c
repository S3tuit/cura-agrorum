/* App-only observers. No extra hardware operations or synchronous logging. */
#include "carrier_observer.h"
#include <stddef.h>
#include <string.h>
#include "bme280.h"
#include "driver/gpio.h"
#include "ds18b20.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "node_sensors_backend.h"
#include "node_sensors_power_gate.h"
#include "onewire_bus.h"
#include "onewire_device.h"
#include "sdkconfig.h"

static carrier_observation_t observed;
/* The production device survives per-sample observer epochs within this wake. */
static i2c_master_dev_handle_t bme_device;
static adc_oneshot_unit_handle_t soil_adc;
static adc_channel_t soil_channel;
static unsigned soil_index, soil_configured;
static int64_t soil_raw_sum;
static bool stabilization_done, inside_adc_read;

void carrier_observer_begin(uint64_t rom0, uint64_t rom1) {
  memset(&observed, 0, sizeof(observed));
  observed.roms[0] = rom0;
  observed.roms[1] = rom1;
  observed.minimum_wait_us = INT64_MAX;
  soil_adc = NULL;
  soil_index = 2;
  soil_configured = 0;
  soil_raw_sum = 0;
  stabilization_done = false;
  inside_adc_read = false;
}
carrier_observation_t carrier_observer_snapshot(void) { return observed; }

bool carrier_observer_sample_valid(unsigned expected_ds_mask, bool bme_present) {
  const unsigned expected_devices = expected_ds_mask == 3 ? 2 : 1;
  if (expected_ds_mask < 1 || expected_ds_mask > 3) return false;
  return !observed.invalid_sequence && observed.conversions == 1 &&
         observed.reads == expected_ds_mask && observed.pending_read == 0 &&
         observed.minimum_wait_us >= 750000 &&
         observed.gate_on == 1 && observed.gate_off == 1 &&
         observed.gate_low == 1 && observed.adc_new == 2 &&
         observed.adc_del == 2 && observed.bus_new == 1 &&
         observed.bus_del == 1 && observed.iter_new == 1 &&
         observed.iter_del == 1 && observed.ds_new == expected_devices &&
         observed.ds_del == expected_devices && observed.bme_forced == (bme_present ? 1U : 0U) && observed.bme_samples == 1 &&
         observed.i2c_new <= 1 && observed.bme_new <= 1 &&
         observed.stabilization_calls == 1 && observed.stabilization_ms == 200 &&
         soil_configured == 3 && observed.power_released &&
         observed.soil_reads[0] == 16 && observed.soil_reads[1] == 16 &&
         observed.soil_delays[0] == 15 && observed.soil_delays[1] == 15 &&
         observed.soil_calibrations[0] == 1 && observed.soil_calibrations[1] == 1;
}
bool carrier_observer_cleanup_valid(unsigned expected_gate_off) {
  return observed.gate_off == expected_gate_off && observed.gate_on == 0 &&
         observed.gate_low == 0 && observed.adc_new == 0 &&
         observed.bus_new == 0 && observed.iter_new == 0 &&
         observed.ds_new == 0 && observed.i2c_new == 0 &&
         observed.bme_new == 0 && observed.bme_init == 0 && observed.bme_forced == 0 &&
         observed.conversions == 0 && observed.reads == 0 && observed.bme_samples == 0;
}

/* Counting attempts also detects a forbidden initialization that failed. */
#define FORWARD(name, counter, params, args) \
  esp_err_t __real_##name params; \
  esp_err_t __wrap_##name params { \
    ++observed.counter; \
    return __real_##name args; \
  }
FORWARD(adc_oneshot_del_unit, adc_del, (adc_oneshot_unit_handle_t unit), (unit))
FORWARD(onewire_new_bus_rmt, bus_new,
        (const onewire_bus_config_t *config, const onewire_bus_rmt_config_t *rmt, onewire_bus_handle_t *out),
        (config, rmt, out))
FORWARD(onewire_bus_del, bus_del, (onewire_bus_handle_t bus), (bus))
FORWARD(onewire_new_device_iter, iter_new,
        (onewire_bus_handle_t bus, onewire_device_iter_handle_t *out), (bus, out))
FORWARD(onewire_del_device_iter, iter_del, (onewire_device_iter_handle_t iter), (iter))
FORWARD(ds18b20_new_device_from_enumeration, ds_new,
        (const onewire_device_t *device, const ds18b20_config_t *config, ds18b20_device_handle_t *out),
        (device, config, out))
FORWARD(ds18b20_del_device, ds_del, (ds18b20_device_handle_t device), (device))

esp_err_t __real_adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *config,
                                    adc_oneshot_unit_handle_t *out);
esp_err_t __wrap_adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *config,
                                    adc_oneshot_unit_handle_t *out) {
  ++observed.adc_new;
  if (!config || config->unit_id != ADC_UNIT_1 || !stabilization_done || observed.gate_off)
    observed.invalid_sequence = true;
  return __real_adc_oneshot_new_unit(config, out);
}
int8_t __real_bme280_init(struct bme280_dev *sensor);
int8_t __wrap_bme280_init(struct bme280_dev *sensor) {
  ++observed.bme_init;
  return __real_bme280_init(sensor);
}
node_sensors_backend_result_t __real_node_sensors_backend_sample_bme280(
    node_sensors_backend_enclosure_t *out);
node_sensors_backend_result_t __wrap_node_sensors_backend_sample_bme280(
    node_sensors_backend_enclosure_t *out) {
  ++observed.bme_samples;
  if (!observed.power_released || observed.gate_off != 1) observed.invalid_sequence = true;
  return __real_node_sensors_backend_sample_bme280(out);
}

esp_err_t __real_node_sensors_power_gate_on(void);
esp_err_t __wrap_node_sensors_power_gate_on(void) {
  ++observed.gate_on;
  observed.power_released = false;
  const esp_err_t result = __real_node_sensors_power_gate_on();
  if (result != ESP_OK) observed.invalid_sequence = true;
  return result;
}
esp_err_t __real_node_sensors_power_gate_off(void);
esp_err_t __wrap_node_sensors_power_gate_off(void) {
  ++observed.gate_off;
  const esp_err_t result = __real_node_sensors_power_gate_off();
  observed.power_released = result == ESP_OK;
  return result;
}
void __real_node_sensors_backend_delay_ms(uint32_t ms);
void __wrap_node_sensors_backend_delay_ms(uint32_t ms) {
  ++observed.stabilization_calls;
  observed.stabilization_ms = ms;
  if (observed.gate_on != 1 || observed.gate_low != 1 || observed.gate_off != 0 || ms != 200)
    observed.invalid_sequence = true;
  __real_node_sensors_backend_delay_ms(ms);
  stabilization_done = true;
}

esp_err_t __real_adc_oneshot_config_channel(adc_oneshot_unit_handle_t adc,
    adc_channel_t channel, const adc_oneshot_chan_cfg_t *config);
esp_err_t __wrap_adc_oneshot_config_channel(adc_oneshot_unit_handle_t adc,
    adc_channel_t channel, const adc_oneshot_chan_cfg_t *config) {
  const esp_err_t result = __real_adc_oneshot_config_channel(adc, channel, config);
  /* Approved C6 GPIO0/1 map to ADC1 channels 0/1; load_build checks the GPIOs. */
  soil_index = channel == ADC_CHANNEL_0 ? 0 : channel == ADC_CHANNEL_1 ? 1 : 2;
  if (result != ESP_OK || soil_index == 2 || !stabilization_done ||
      observed.gate_off != 0 || !config || config->atten != ADC_ATTEN_DB_12 ||
      config->bitwidth != ADC_BITWIDTH_DEFAULT || (soil_configured & (1U << soil_index))) {
    observed.invalid_sequence = true;
  }
  if (soil_index < 2) soil_configured |= 1U << soil_index;
  soil_adc = adc;
  soil_channel = channel;
  soil_raw_sum = 0;
  return result;
}

esp_err_t __real_adc_oneshot_read(adc_oneshot_unit_handle_t adc, adc_channel_t channel, int *raw);
esp_err_t __wrap_adc_oneshot_read(adc_oneshot_unit_handle_t adc, adc_channel_t channel, int *raw) {
  inside_adc_read = true;
  const esp_err_t result = __real_adc_oneshot_read(adc, channel, raw);
  inside_adc_read = false;
  if (soil_index >= 2 || adc != soil_adc || channel != soil_channel || result != ESP_OK || !raw) {
    observed.invalid_sequence = true;
  } else {
    if (observed.soil_reads[soil_index] != observed.soil_delays[soil_index] || observed.gate_off)
      observed.invalid_sequence = true;
    ++observed.soil_reads[soil_index];
    soil_raw_sum += *raw;
  }
  return result;
}

void __real_esp_rom_delay_us(uint32_t us);
void __wrap_esp_rom_delay_us(uint32_t us) {
  /* Ignore delays inside the ADC driver itself; observe the sampler's gaps. */
  if (!inside_adc_read && soil_index < 2 && observed.soil_reads[soil_index] > 0 &&
      observed.soil_reads[soil_index] < 16 && !observed.soil_calibrations[soil_index]) {
    if (us != 2000 || observed.soil_reads[soil_index] != observed.soil_delays[soil_index] + 1)
      observed.invalid_sequence = true;
    ++observed.soil_delays[soil_index];
  }
  __real_esp_rom_delay_us(us);
}

esp_err_t __real_adc_cali_raw_to_voltage(adc_cali_handle_t cali, int raw, int *mv);
esp_err_t __wrap_adc_cali_raw_to_voltage(adc_cali_handle_t cali, int raw, int *mv) {
  const esp_err_t result = __real_adc_cali_raw_to_voltage(cali, raw, mv);
  if (soil_index >= 2 || result != ESP_OK || !cali || !mv) {
    observed.invalid_sequence = true;
  } else {
    if (observed.soil_reads[soil_index] != 16 || raw != (soil_raw_sum + 8) / 16)
      observed.invalid_sequence = true;
    ++observed.soil_calibrations[soil_index];
    observed.soil_mv[soil_index] = *mv;
  }
  return result;
}

FORWARD(i2c_new_master_bus, i2c_new,
        (const i2c_master_bus_config_t *config, i2c_master_bus_handle_t *out), (config, out))
esp_err_t __real_i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
    const i2c_device_config_t *config, i2c_master_dev_handle_t *out);
esp_err_t __wrap_i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
    const i2c_device_config_t *config, i2c_master_dev_handle_t *out) {
  ++observed.bme_new;
  const esp_err_t result = __real_i2c_master_bus_add_device(bus, config, out);
  if (result == ESP_OK && out) bme_device = *out;
  return result;
}
esp_err_t __real_i2c_master_transmit(i2c_master_dev_handle_t device,
    const uint8_t *bytes, size_t size, int timeout_ms);
esp_err_t __wrap_i2c_master_transmit(i2c_master_dev_handle_t device,
    const uint8_t *bytes, size_t size, int timeout_ms) {
  if (size == 2 && bytes[0] == 0xf4 && (bytes[1] & 3) == 1) {
    ++observed.bme_forced;
    if (!observed.power_released || observed.gate_off != 1) observed.invalid_sequence = true;
  }
  return __real_i2c_master_transmit(device, bytes, size, timeout_ms);
}
esp_err_t __real_i2c_master_transmit_receive(i2c_master_dev_handle_t device,
    const uint8_t *command, size_t command_size, uint8_t *data, size_t length, int timeout_ms);
esp_err_t __wrap_i2c_master_transmit_receive(i2c_master_dev_handle_t device,
    const uint8_t *command, size_t command_size, uint8_t *data, size_t length, int timeout_ms) {
  const esp_err_t result = __real_i2c_master_transmit_receive(device, command, command_size,
                                                           data, length, timeout_ms);
  if (command_size == 1 && command && observed.bme_samples) {
    if (result != ESP_OK && observed.bme_read_error == ESP_OK) {
      observed.bme_read_error = result;
      observed.bme_error_register = command[0];
    }
    if (command[0] == 0x88 && length == 26 && result == ESP_OK) {
      memcpy(observed.bme_cal_t, data, sizeof(observed.bme_cal_t));
      observed.bme_cal_read = true;
    }
    if (command[0] == 0xf7 && length == sizeof(observed.bme_raw)) {
      ++observed.bme_data_reads;
      if (result == ESP_OK) memcpy(observed.bme_raw, data, sizeof(observed.bme_raw));
    }
  }
  return result;
}
int8_t __real_bme280_get_sensor_data(uint8_t select, struct bme280_data *data, struct bme280_dev *sensor);
int8_t __wrap_bme280_get_sensor_data(uint8_t select, struct bme280_data *data, struct bme280_dev *sensor) {
  const int8_t result = __real_bme280_get_sensor_data(select, data, sensor);
  observed.bme_data_result = result;
  if (result == BME280_OK && data) {
    observed.bme_temperature = data->temperature;
    observed.bme_pressure = data->pressure;
    observed.bme_humidity = data->humidity;
  }
  return result;
}
esp_err_t carrier_observer_bme_registers(uint8_t registers[2]) {
  if (!registers || !bme_device) return ESP_ERR_INVALID_STATE;
  const uint8_t address = 0xf3;
  return i2c_master_transmit_receive(bme_device, &address, 1, registers, 2, 20);
}
esp_err_t __real_gpio_set_level(gpio_num_t gpio, uint32_t level);
esp_err_t __wrap_gpio_set_level(gpio_num_t gpio, uint32_t level) {
  if (gpio == CONFIG_CURA_SENSOR_POWER_GATE_GPIO && level == 0) {
    ++observed.gate_low;
  }
  return __real_gpio_set_level(gpio, level);
}

esp_err_t __real_onewire_bus_write_bytes(onewire_bus_handle_t bus, const uint8_t *data, uint8_t size);
esp_err_t __wrap_onewire_bus_write_bytes(onewire_bus_handle_t bus, const uint8_t *data, uint8_t size) {
  const int64_t start = esp_timer_get_time();
  const esp_err_t result = __real_onewire_bus_write_bytes(bus, data, size);
  if (result != ESP_OK) {
    observed.invalid_sequence = true;
    return result;
  }
  if (size == 2 && data[0] == 0xcc && data[1] == 0x44) {
    ++observed.conversions;
    observed.conversion_us = esp_timer_get_time();
  } else if (size == 10 && data[0] == 0x55 && data[9] == 0xbe) {
    uint64_t rom = 0;
    for (unsigned i = 0; i < 8; ++i) {
      rom |= (uint64_t)data[1 + i] << (8 * i);
    }
    unsigned bit = rom == observed.roms[0] ? 1 : rom == observed.roms[1] ? 2 : 0;
    int64_t wait = start - observed.conversion_us;
    if (!bit || observed.conversions != 1 || wait < 750000 ||
        (observed.reads & bit) || observed.pending_read) {
      observed.invalid_sequence = true;
    }
    observed.pending_read = bit;
    if (wait < observed.minimum_wait_us) observed.minimum_wait_us = wait;
  }
  return result;
}

esp_err_t __real_onewire_bus_read_bytes(onewire_bus_handle_t bus, uint8_t *data, uint8_t size);
esp_err_t __wrap_onewire_bus_read_bytes(onewire_bus_handle_t bus, uint8_t *data, uint8_t size) {
  const esp_err_t result = __real_onewire_bus_read_bytes(bus, data, size);
  if (observed.pending_read) {
    /* The real DS driver subsequently checks the scratchpad CRC. A successful
     * sampler result is also mandatory; this observer supplies no temperature. */
    if (result != ESP_OK || size != 9 || (data[4] & 0x60) != 0x60) {
      observed.invalid_sequence = true;
    } else {
      observed.reads |= observed.pending_read;
    }
    observed.pending_read = 0;
  }
  return result;
}

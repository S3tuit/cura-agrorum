/* App-only observers. No extra hardware operations or synchronous logging. */
#include "carrier_observer.h"
#include <stddef.h>
#include <string.h>
#include "bme280.h"
#include "driver/gpio.h"
#include "ds18b20.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "i2c_bus.h"
#include "node_sensors_power_gate.h"
#include "onewire_bus.h"
#include "onewire_device.h"
#include "sdkconfig.h"

static carrier_observation_t observed;

void carrier_observer_begin(uint64_t rom0, uint64_t rom1) {
  memset(&observed, 0, sizeof(observed));
  observed.roms[0] = rom0;
  observed.roms[1] = rom1;
  observed.minimum_wait_us = INT64_MAX;
}
carrier_observation_t carrier_observer_snapshot(void) { return observed; }

bool carrier_observer_sample_valid(void) {
  return !observed.invalid_sequence && observed.conversions == 1 &&
         observed.reads == 3 && observed.minimum_wait_us >= 750000 &&
         observed.gate_on == 1 && observed.gate_off == 1 &&
         observed.gate_low == 1 && observed.adc_new == 2 &&
         observed.adc_del == 2 && observed.bus_new == 1 &&
         observed.bus_del == 1 && observed.iter_new == 1 &&
         observed.iter_del == 1 && observed.ds_new == 2 &&
         observed.ds_del == 2 && observed.bme_forced == 1 &&
         observed.i2c_new <= 1 && observed.bme_new <= 1;
}
bool carrier_observer_cleanup_valid(unsigned expected_gate_off) {
  return observed.gate_off == expected_gate_off && observed.gate_on == 0 &&
         observed.gate_low == 0 && observed.adc_new == 0 &&
         observed.bus_new == 0 && observed.iter_new == 0 &&
         observed.ds_new == 0 && observed.i2c_new == 0 &&
         observed.bme_new == 0 && observed.bme_init == 0 && observed.bme_forced == 0 &&
         observed.conversions == 0 && observed.reads == 0;
}

/* Counting attempts also detects a forbidden initialization that failed. */
#define FORWARD(name, counter, params, args) \
  esp_err_t __real_##name params; \
  esp_err_t __wrap_##name params { \
    ++observed.counter; \
    return __real_##name args; \
  }
FORWARD(node_sensors_power_gate_on, gate_on, (void), ())
FORWARD(node_sensors_power_gate_off, gate_off, (void), ())
FORWARD(adc_oneshot_new_unit, adc_new,
        (const adc_oneshot_unit_init_cfg_t *config, adc_oneshot_unit_handle_t *out), (config, out))
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
FORWARD(bme280_default_init, bme_init, (bme280_handle_t sensor), (sensor))
FORWARD(bme280_take_forced_measurement, bme_forced, (bme280_handle_t sensor), (sensor))

i2c_bus_handle_t __real_i2c_bus_create(i2c_port_t port, const i2c_config_t *config);
i2c_bus_handle_t __wrap_i2c_bus_create(i2c_port_t port, const i2c_config_t *config) {
  ++observed.i2c_new;
  return __real_i2c_bus_create(port, config);
}
bme280_handle_t __real_bme280_create(i2c_bus_handle_t bus, uint8_t address);
bme280_handle_t __wrap_bme280_create(i2c_bus_handle_t bus, uint8_t address) {
  ++observed.bme_new;
  return __real_bme280_create(bus, address);
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

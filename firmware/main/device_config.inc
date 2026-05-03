#pragma once

// Per-device/per-sensor constants. This file must be aligned with the physical
// ESP32 board and sensors currently flashed with this firmware.

#define DEVICE_ID "esp32-devkitc-v4-01"
#define SOIL_SENSOR_ID "soil-01"
#define DS18B20_SENSOR_ID "temp-01"
#define ENV280_SENSOR_ID "env-01"

#define SOIL_ADC_GPIO 34
#define SOIL_ADC_ATTEN ADC_ATTEN_DB_12

#define DS18B20_GPIO 4
#define DS18B20_ENABLE_INTERNAL_PULLUP 0
#define DS18B20_RESOLUTION DS18B20_RESOLUTION_12B

#define ENV280_I2C_PORT I2C_NUM_0
#define ENV280_I2C_SDA_GPIO 21
#define ENV280_I2C_SCL_GPIO 22
#define ENV280_I2C_FREQ_HZ 100000
#define ENV280_I2C_ADDR BMP280_I2C_ADDRESS_0
#define ENV280_ENABLE_INTERNAL_PULLUPS 1

// Current calibration for Soil Moisture Capacitive Module V1.2 unit 1.
// The sensor output decreases as moisture increases.
#define SOIL_DRY_MV 2180
#define SOIL_WET_MV 660

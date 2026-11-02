set(sensor_sources node_sensors.c node_sensors_backend_esp.c
                   node_sensors_bme280_esp.c node_sensors_power_gate_esp.c)
# ESP-IDF resolves dependencies before sdkconfig; only sources are conditional.
set(sensor_requires bosch_bme280 esp_driver_gpio esp_driver_i2c esp_timer
    esp_driver_uart ds18b20 esp_adc esp_driver_rmt onewire_bus)
if(CONFIG_CURA_SEN0600_0)
    list(APPEND sensor_sources node_sensors_sen0600_esp.c sen0600_modbus.c)
else()
    list(APPEND sensor_sources node_sensors_analog_esp.c node_sensors_soil_analog.c
         node_sensors_ds18b20_identity.c node_sensors_ds18b20_gpio_esp.c soil_sensor.c)
endif()

idf_component_register(SRCS ${sensor_sources} INCLUDE_DIRS include
    PRIV_INCLUDE_DIRS private_include REQUIRES node_common
    PRIV_REQUIRES ${sensor_requires})

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    include("${CMAKE_CURRENT_LIST_DIR}/sensor_config.cmake")
endif()

idf_component_register(
    SRCS "test_app_main.c" "test_sensor_carrier.c" "carrier_observer.c" "carrier_hold.c" "carrier_console.c"
    PRIV_REQUIRES unity node_sensors node_platform_esp esp_app_format esp_hw_support
                  onewire_bus ds18b20 bme280 i2c_bus esp_adc esp_driver_gpio esp_timer freertos
    WHOLE_ARCHIVE)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../../components/node_sensors/private_include")

# App-local interposition; every wrapper forwards to the actual linked driver.
foreach(symbol node_sensors_power_gate_on node_sensors_power_gate_off
        node_sensors_force_power_off
        gpio_set_level adc_oneshot_new_unit adc_oneshot_del_unit
        onewire_new_bus_rmt onewire_bus_del onewire_new_device_iter
        onewire_del_device_iter ds18b20_new_device_from_enumeration
        ds18b20_del_device bme280_default_init bme280_take_forced_measurement i2c_bus_create
        bme280_create onewire_bus_write_bytes onewire_bus_read_bytes
        unity_gets esp_rom_output_rx_one_char)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=${symbol}")
endforeach()

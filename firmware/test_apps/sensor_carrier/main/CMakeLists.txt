idf_component_register(
    SRCS "test_app_main.c" "test_sensor_carrier.c" "carrier_checks.c" "carrier_bme_preflight.c" "carrier_observer.c" "carrier_hold.c" "carrier_console.c" "carrier_core.c"
         "../../on_device/main/persistence_test_support.c"
    PRIV_REQUIRES unity node_sensors node_platform_esp esp_app_format esp_hw_support
                  onewire_bus ds18b20 bosch_bme280 esp_driver_i2c esp_adc esp_driver_gpio esp_timer freertos
                  node_core node_persistence protocol_v2_lora sx1262_radio
                  nvs_flash joltwallet__littlefs
    WHOLE_ARCHIVE)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../../components/node_sensors/private_include"
    "${CMAKE_CURRENT_LIST_DIR}/../../../components/node_persistence/private_include"
    "${CMAKE_CURRENT_LIST_DIR}/../../on_device/main")
target_compile_definitions(${COMPONENT_LIB} PRIVATE NODE_PERSISTENCE_TESTING=1
    NODE_PERSISTENCE_NVS_PARTITION_LABEL="nvs_test"
    NODE_PERSISTENCE_PARTITION_LABEL="storage_test")

# App-local interposition; every wrapper forwards to the actual linked driver.
foreach(symbol node_sensors_sample_all node_sensors_power_gate_on node_sensors_power_gate_off
        node_sensors_force_power_off
        gpio_set_level adc_oneshot_new_unit adc_oneshot_del_unit
        node_sensors_backend_delay_ms adc_oneshot_config_channel adc_oneshot_read
        adc_cali_raw_to_voltage esp_rom_delay_us
        onewire_new_bus_rmt onewire_bus_del onewire_new_device_iter
        onewire_del_device_iter ds18b20_new_device_from_enumeration
        ds18b20_del_device bme280_init node_sensors_backend_sample_bme280
        i2c_new_master_bus i2c_master_bus_add_device i2c_master_transmit
        i2c_master_transmit_receive bme280_get_sensor_data onewire_bus_write_bytes onewire_bus_read_bytes
        unity_gets esp_rom_output_rx_one_char)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=${symbol}")
endforeach()

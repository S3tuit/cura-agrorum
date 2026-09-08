idf_component_register(
    SRCS "test_app_main.c" "test_sensor_carrier.c"
    PRIV_REQUIRES unity node_sensors esp_app_format esp_hw_support
                  onewire_bus i2c_bus esp_driver_gpio esp_timer freertos
    WHOLE_ARCHIVE)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../../components/node_sensors/private_include")

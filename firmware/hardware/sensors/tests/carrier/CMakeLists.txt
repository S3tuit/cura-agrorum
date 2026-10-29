cmake_minimum_required(VERSION 3.22)

set(IDF_TARGET "esp32s3")
get_filename_component(CURA_FIRMWARE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${CURA_FIRMWARE_DIR}/components")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cura_sensor_carrier)

include("${CURA_FIRMWARE_DIR}/cmake/xiao_board.cmake")

# This app owns only test-labelled storage. Production component defaults and
# the bare-board application's build/quotas/sleep settings are unchanged.
idf_component_get_property(carrier_persistence node_persistence COMPONENT_LIB)
target_compile_definitions(${carrier_persistence} PRIVATE
    NODE_PERSISTENCE_TESTING=1
    NODE_PERSISTENCE_NVS_PARTITION_LABEL="nvs_test"
    NODE_PERSISTENCE_PARTITION_LABEL="storage_test")

foreach(setting CURA_SOIL_0_GPIO CURA_SOIL_1_GPIO
                CURA_SENSOR_POWER_GATE_GPIO CURA_DS18B20_GPIO
                CURA_I2C_SDA_GPIO CURA_I2C_SCL_GPIO
                CURA_SENSOR_POWER_STABILIZATION_MS
                CURA_DS18B20_0_ROM CURA_DS18B20_1_ROM
                ESP_CONSOLE_USB_SERIAL_JTAG FREERTOS_HZ)
    message(STATUS "Sensor carrier resolved CONFIG_${setting}=${CONFIG_${setting}}")
endforeach()

cmake_minimum_required(VERSION 3.22)

set(IDF_TARGET "esp32c6")
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
    "${CMAKE_CURRENT_LIST_DIR}/../../managed_components/joltwallet__littlefs")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cura_sensor_carrier)

foreach(setting CURA_SOIL_0_GPIO CURA_SOIL_1_GPIO
                CURA_SENSOR_POWER_GATE_GPIO CURA_DS18B20_GPIO
                CURA_I2C_SDA_GPIO CURA_I2C_SCL_GPIO
                CURA_SENSOR_POWER_STABILIZATION_MS
                CURA_DS18B20_0_ROM CURA_DS18B20_1_ROM
                ESP_CONSOLE_UART_NUM I2C_BUS_BACKWARD_CONFIG I2C_MS_TO_WAIT)
    message(STATUS "Sensor carrier resolved CONFIG_${setting}=${CONFIG_${setting}}")
endforeach()

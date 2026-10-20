cmake_minimum_required(VERSION 3.22)
set(IDF_TARGET "esp32s3")
get_filename_component(CURA_FIRMWARE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${CURA_FIRMWARE_DIR}/components")
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cura_radio_component)

include("${CURA_FIRMWARE_DIR}/cmake/xiao_board.cmake")

idf_component_get_property(rf_platform node_platform_esp COMPONENT_LIB)
target_compile_definitions(${rf_platform} PRIVATE CURA_RF_COMPONENT_TEST=1)
target_include_directories(${rf_platform} PRIVATE "${CURA_FIRMWARE_DIR}/tests/support")

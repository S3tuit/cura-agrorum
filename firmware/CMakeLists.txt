cmake_minimum_required(VERSION 3.22)

# The node is built for the XIAO ESP32S3. Keep the target in tracked
# project configuration so a fresh checkout cannot silently default to esp32.
set(IDF_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)

project(cura_agrorum_firmware)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/xiao_board.cmake")

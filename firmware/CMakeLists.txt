cmake_minimum_required(VERSION 3.22)

# The LoRa v2 pilot node is built for the ESP32-C6. Keep the target in tracked
# project configuration so a fresh checkout cannot silently default to esp32.
set(IDF_TARGET "esp32c6")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)

project(cura_agrorum_firmware)

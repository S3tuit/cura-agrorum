cmake_minimum_required(VERSION 3.22)

# This maintenance application is intentionally specific to the pilot C6 board.
set(IDF_TARGET "esp32c6")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)

project(cura_agrorum_littlefs_storage_benchmark)

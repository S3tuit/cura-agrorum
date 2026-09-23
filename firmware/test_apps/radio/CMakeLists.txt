cmake_minimum_required(VERSION 3.22)
set(IDF_TARGET "esp32c6")
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components"
    "${CMAKE_CURRENT_LIST_DIR}/../../managed_components/joltwallet__littlefs")
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cura_radio_component)

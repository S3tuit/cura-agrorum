cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/node_common"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/node_sensors"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)

project(cura_agrorum_always_on_soil_cal)

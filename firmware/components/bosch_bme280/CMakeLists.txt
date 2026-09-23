# Requirements expansion must not fetch dependencies in CMake script mode.
if(CMAKE_BUILD_EARLY_EXPANSION)
    idf_component_register()
    return()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/resolve_source.cmake")
cura_bme280_resolve(BME280_SOURCE)
idf_component_register(SRCS "${BME280_SOURCE}/bme280.c"
                       INCLUDE_DIRS "${BME280_SOURCE}")
target_compile_definitions(${COMPONENT_LIB} PUBLIC BME280_DOUBLE_ENABLE)

include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/source_pin.cmake")

# Shared by two real consumers: ESP-IDF and native CMake/CTest. No sibling-path
# fallback: an explicit override must be clean and match the immutable pin.
function(cura_bme280_resolve out_source)
    set(CURA_BME280_SOURCE_DIR "$ENV{CURA_BME280_SOURCE_DIR}" CACHE PATH
        "Explicit clean Bosch checkout matching the firmware commit and hashes")
    if(CURA_BME280_SOURCE_DIR)
        set(selection "explicit_clean_checkout")
        get_filename_component(source "${CURA_BME280_SOURCE_DIR}" REALPATH)
    else()
        set(selection "fetched_pin")
        include(FetchContent)
        FetchContent_Declare(cura_bosch_bme280
            GIT_REPOSITORY "${CURA_BME280_REPOSITORY}"
            GIT_TAG "${CURA_BME280_COMMIT}"
            GIT_SHALLOW FALSE)
        FetchContent_MakeAvailable(cura_bosch_bme280)
        set(source "${cura_bosch_bme280_SOURCE_DIR}")
    endif()
    execute_process(COMMAND git -C "${source}" rev-parse HEAD
        RESULT_VARIABLE status OUTPUT_VARIABLE head OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT status EQUAL 0 OR NOT head STREQUAL CURA_BME280_COMMIT)
        message(FATAL_ERROR "Bosch checkout must match pin ${CURA_BME280_COMMIT}: ${source}")
    endif()
    execute_process(COMMAND git -C "${source}" status --porcelain --untracked-files=no
        RESULT_VARIABLE status OUTPUT_VARIABLE changes OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT status EQUAL 0 OR NOT changes STREQUAL "")
        message(FATAL_ERROR "Bosch checkout has tracked changes: ${source}")
    endif()
    foreach(name bme280.c bme280.h bme280_defs.h LICENSE)
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${source}/${name}")
        string(REPLACE "." "_" key "${name}")
        file(SHA256 "${source}/${name}" actual)
        if(NOT actual STREQUAL CURA_BME280_SHA256_${key})
            message(FATAL_ERROR "Bosch ${name} differs from firmware content pin")
        endif()
    endforeach()
    message(STATUS "Bosch BME280 source=${source} commit=${head} compensation=double")
    # Evidence records the source actually consumed by this build.
    file(WRITE "${CMAKE_BINARY_DIR}/bosch-bme280-source.txt" "${source}\n${head}\n${selection}\n")
    set(${out_source} "${source}" PARENT_SCOPE)
endfunction()

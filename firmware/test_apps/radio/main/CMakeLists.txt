idf_component_register(SRCS "radio_app.c" "radio_observe.c" "radio_command.c" "radio_rejection.c" "radio_header_error.c"
    PRIV_REQUIRES unity sx1262_radio node_platform_esp esp_app_format esp_hw_support
                  esp_system esp_timer freertos protocol_v2_lora
    WHOLE_ARCHIVE)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/../../../components/sx1262_radio/private_include"
    "${CMAKE_CURRENT_LIST_DIR}/../../../components/sx1262_radio/vendor/semtech_sx126x_driver")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=sx126x_hal_write")
foreach(symbol initialize set_standby set_packet_params configure_irq clear_irq
        start_tx start_single_rx set_sleep_cold)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-Wl,--wrap=sx1262_radio_backend_${symbol}")
endforeach()
if(DEFINED REJECTION_INPUT_DIR AND NOT "${REJECTION_INPUT_DIR}" STREQUAL "")
    if(NOT IS_ABSOLUTE "${REJECTION_INPUT_DIR}" OR NOT EXISTS "${REJECTION_INPUT_DIR}/rejection_inputs.h")
        message(FATAL_ERROR "REJECTION_INPUT_DIR must name an absolute private prepared input directory")
    endif()
    # Reusing a credential bundle with changed frame construction could reuse
    # a CCM nonce with changed plaintext. Require a freshly prepared bundle.
    file(READ "${REJECTION_INPUT_DIR}/rejection-manifest.json" rejection_manifest)
    string(JSON expected_header GET "${rejection_manifest}" input_header_sha256)
    file(SHA256 "${REJECTION_INPUT_DIR}/rejection_inputs.h" actual_header)
    if(NOT actual_header STREQUAL expected_header)
        message(FATAL_ERROR "rejection matrix private input header changed; prepare a fresh bundle")
    endif()
    string(JSON source_count LENGTH "${rejection_manifest}" sources)
    math(EXPR last_source "${source_count} - 1")
    foreach(index RANGE ${last_source})
        string(JSON source MEMBER "${rejection_manifest}" sources ${index})
        string(JSON expected GET "${rejection_manifest}" sources "${source}")
        file(SHA256 "${CMAKE_CURRENT_LIST_DIR}/../../../../${source}" actual)
        if(NOT actual STREQUAL expected)
            message(FATAL_ERROR "rejection matrix source changed: ${source}; prepare a fresh bundle")
        endif()
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/../../../../${source}")
    endforeach()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${REJECTION_INPUT_DIR}/rejection-manifest.json" "${REJECTION_INPUT_DIR}/rejection_inputs.h")
    target_include_directories(${COMPONENT_LIB} PRIVATE "${REJECTION_INPUT_DIR}")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE REJECTION_ENABLED=1)
endif()

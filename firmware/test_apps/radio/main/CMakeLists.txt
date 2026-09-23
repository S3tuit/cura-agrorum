idf_component_register(SRCS "radio_app.c" "radio_observe.c" "radio_command.c" "radio_rejection.c"
    PRIV_REQUIRES unity sx1262_radio node_platform_esp esp_app_format esp_hw_support
                  esp_system esp_timer freertos protocol_v2_lora
    WHOLE_ARCHIVE)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/../../../components/sx1262_radio/private_include")
foreach(symbol initialize set_standby set_packet_params configure_irq clear_irq
        start_tx start_single_rx set_sleep_cold)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-Wl,--wrap=sx1262_radio_backend_${symbol}")
endforeach()
if(DEFINED RF023_INPUT_DIR AND NOT "${RF023_INPUT_DIR}" STREQUAL "")
    if(NOT IS_ABSOLUTE "${RF023_INPUT_DIR}" OR NOT EXISTS "${RF023_INPUT_DIR}/rf023_inputs.h")
        message(FATAL_ERROR "RF023_INPUT_DIR must name an absolute private prepared input directory")
    endif()
    # Reusing a credential bundle with changed frame construction could reuse
    # a CCM nonce with changed plaintext. Require a freshly prepared bundle.
    file(READ "${RF023_INPUT_DIR}/rf023-manifest.json" rf023_manifest)
    string(JSON expected_header GET "${rf023_manifest}" input_header_sha256)
    file(SHA256 "${RF023_INPUT_DIR}/rf023_inputs.h" actual_header)
    if(NOT actual_header STREQUAL expected_header)
        message(FATAL_ERROR "RF-023 private input header changed; prepare a fresh bundle")
    endif()
    string(JSON source_count LENGTH "${rf023_manifest}" sources)
    math(EXPR last_source "${source_count} - 1")
    foreach(index RANGE ${last_source})
        string(JSON source MEMBER "${rf023_manifest}" sources ${index})
        string(JSON expected GET "${rf023_manifest}" sources "${source}")
        file(SHA256 "${CMAKE_CURRENT_LIST_DIR}/../../../../${source}" actual)
        if(NOT actual STREQUAL expected)
            message(FATAL_ERROR "RF-023 source changed: ${source}; prepare a fresh bundle")
        endif()
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/../../../../${source}")
    endforeach()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${RF023_INPUT_DIR}/rf023-manifest.json" "${RF023_INPUT_DIR}/rf023_inputs.h")
    target_include_directories(${COMPONENT_LIB} PRIVATE "${RF023_INPUT_DIR}")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE RF023_ENABLED=1)
endif()

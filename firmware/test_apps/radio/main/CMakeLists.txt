idf_component_register(SRCS "radio_app.c" "radio_observe.c" "radio_command.c"
    PRIV_REQUIRES unity sx1262_radio node_platform_esp esp_app_format esp_hw_support
                  esp_system esp_timer freertos
    WHOLE_ARCHIVE)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../../components/sx1262_radio/private_include")
foreach(symbol initialize set_standby set_packet_params configure_irq clear_irq
        start_tx start_single_rx set_sleep_cold)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-Wl,--wrap=sx1262_radio_backend_${symbol}")
endforeach()

idf_component_register(
    SRCS "app_main.c"
    REQUIRES node_core node_platform_esp
    PRIV_REQUIRES sx1262_radio esp_timer
    # Retain passive RF wrappers referenced by components linked after main.
    WHOLE_ARCHIVE
)

if(CONFIG_NODE_RF_PHY_OBSERVATION)
    target_sources(${COMPONENT_LIB} PRIVATE "../test_apps/radio/main/node_rf_observe.c")
    target_include_directories(${COMPONENT_LIB} PRIVATE
        "../components/sx1262_radio/private_include"
        "../components/sx1262_radio/vendor/semtech_sx126x_driver")
    foreach(symbol sx126x_hal_read sx126x_hal_write sx1262_radio_backend_wait_dio1)
        target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=${symbol}")
    endforeach()
endif()

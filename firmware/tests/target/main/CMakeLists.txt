idf_component_register(
    SRCS
      "test_app_main.c"
      "${CURA_FIRMWARE_DIR}/tests/support/persistence_test_support.c"
      "node_core_hardware_test_support.c"
      "test_node_core_on_device.c"
      "test_node_persistence_on_device.c"
      "test_node_platform_esp_on_device.c"
      "test_node_rtc_on_device.c"
    PRIV_REQUIRES
      unity
      node_core
      node_persistence
      node_platform_esp
      node_sensors
      protocol_v2_lora
      sx1262_radio
      nvs_flash
      esp_partition
      esp_hw_support
      esp_timer
      freertos
      joltwallet__littlefs
    WHOLE_ARCHIVE)

target_compile_definitions(
    ${COMPONENT_LIB}
    PRIVATE
      NODE_PERSISTENCE_TESTING=1
      NODE_PERSISTENCE_NVS_PARTITION_LABEL="nvs_test"
      NODE_PERSISTENCE_PARTITION_LABEL="storage_test")

target_include_directories(
    ${COMPONENT_LIB}
    PRIVATE "${CURA_FIRMWARE_DIR}/components/node_persistence/private_include"
            "${CURA_FIRMWARE_DIR}/tests/support")

# Shared console handshake and stock Unity input yielding for native USB.
target_sources(${COMPONENT_LIB} PRIVATE
    "${CURA_FIRMWARE_DIR}/tests/support/usb_console_test_support.c"
    "${CURA_FIRMWARE_DIR}/tests/support/unity_console.c")
target_include_directories(${COMPONENT_LIB} PRIVATE "${CURA_FIRMWARE_DIR}/tests/support")
foreach(symbol unity_gets unity_putc esp_rom_output_rx_one_char)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=${symbol}")
endforeach()

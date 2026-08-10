idf_component_register(
    SRCS
      "test_app_main.c"
      "persistence_test_support.c"
      "test_node_persistence_on_device.c"
      "test_node_platform_esp_on_device.c"
    PRIV_REQUIRES
      unity
      node_persistence
      node_platform_esp
      protocol_v2_lora
      nvs_flash
      esp_partition
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
    PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../../components/node_persistence/private_include")

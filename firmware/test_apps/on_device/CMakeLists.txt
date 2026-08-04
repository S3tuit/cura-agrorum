cmake_minimum_required(VERSION 3.22)

set(IDF_TARGET "esp32c6")
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
    "${CMAKE_CURRENT_LIST_DIR}/../../managed_components/joltwallet__littlefs")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cura_node_persistence_hardware_tests)

idf_component_get_property(node_persistence_lib node_persistence COMPONENT_LIB)
target_compile_definitions(
    ${node_persistence_lib}
    PRIVATE
      NODE_PERSISTENCE_TESTING=1
      NODE_PERSISTENCE_NVS_PARTITION_LABEL="nvs_test"
      NODE_PERSISTENCE_PARTITION_LABEL="storage_test"
      NODE_PERSISTENCE_PENDING_LOG_LIMIT=512
      NODE_PERSISTENCE_PENDING_COMPACT_RETAIN_LIMIT=256
      NODE_PERSISTENCE_QUARANTINE_LOG_LIMIT=128
      NODE_PERSISTENCE_DIAGNOSTIC_LOG_LIMIT=128
      NODE_PERSISTENCE_DELIVERY_LOG_LIMIT=128)

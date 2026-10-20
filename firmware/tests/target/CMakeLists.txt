cmake_minimum_required(VERSION 3.22)

set(IDF_TARGET "esp32s3")
get_filename_component(CURA_FIRMWARE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${CURA_FIRMWARE_DIR}/components")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cura_node_persistence_hardware_tests)

include("${CURA_FIRMWARE_DIR}/cmake/xiao_board.cmake")

# Fits the core cases' complete delivery histories while keeping quota tests
# short. Implementation and quota assertions must use the same limit.
set(hwtest_delivery_log_limit 512)

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
      NODE_PERSISTENCE_DELIVERY_LOG_LIMIT=${hwtest_delivery_log_limit})

idf_component_get_property(main_lib main COMPONENT_LIB)
target_compile_definitions(
    ${main_lib} PRIVATE NODE_PERSISTENCE_DELIVERY_LOG_LIMIT=${hwtest_delivery_log_limit})

idf_component_get_property(node_core_lib node_core COMPONENT_LIB)
target_compile_definitions(
    ${node_core_lib}
    PRIVATE
      NODE_CORE_TESTING=1
      NODE_CORE_ON_DEVICE_TESTING=1
      NODE_CORE_TEST_DEEP_SLEEP_DURATION_US=250000ULL)

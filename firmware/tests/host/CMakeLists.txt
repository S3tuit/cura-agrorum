cmake_minimum_required(VERSION 3.20)

project(cura_agrorum_firmware_host_tests C)

include(CTest)

set(FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

add_executable(
  test_node_persistence
  fake_node_persistence_backend.c
  test_node_persistence.c
  ${FIRMWARE_ROOT}/components/node_common/node_common.c
  ${FIRMWARE_ROOT}/components/node_persistence/node_persistence.c
  ${FIRMWARE_ROOT}/components/node_persistence/node_persistence_record.c
  ${FIRMWARE_ROOT}/components/protocol_v2_lora/protocol_v2_lora_schema_generated.c
)

target_include_directories(
  test_node_persistence
  PRIVATE
    ${FIRMWARE_ROOT}/components/node_common/include
    ${FIRMWARE_ROOT}/components/node_persistence/include
    ${FIRMWARE_ROOT}/components/node_persistence/private_include
    ${FIRMWARE_ROOT}/components/protocol_v2_lora/include
)

target_compile_features(test_node_persistence PRIVATE c_std_11)
target_compile_definitions(
  test_node_persistence
  PRIVATE
    _POSIX_C_SOURCE=200809L
    NODE_PERSISTENCE_TESTING=1
    NODE_PERSISTENCE_PENDING_LOG_LIMIT=256
    NODE_PERSISTENCE_PENDING_COMPACT_RETAIN_LIMIT=128
    NODE_PERSISTENCE_QUARANTINE_LOG_LIMIT=100
    NODE_PERSISTENCE_DIAGNOSTIC_LOG_LIMIT=100
    NODE_PERSISTENCE_DELIVERY_LOG_LIMIT=100
)
target_compile_options(
  test_node_persistence
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
    -Wformat=2
    -Wshadow
    -Wconversion
    -Wsign-conversion
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(
    test_node_persistence PRIVATE -fsanitize=address,undefined
                                  -fno-omit-frame-pointer
  )
  target_link_options(
    test_node_persistence PRIVATE -fsanitize=address,undefined
                                  -fno-omit-frame-pointer
  )
endif()

add_test(NAME node_persistence COMMAND test_node_persistence)
set_tests_properties(
  node_persistence
  PROPERTIES ENVIRONMENT
             "ASAN_OPTIONS=detect_leaks=1:abort_on_error=1:halt_on_error=1:fast_unwind_on_malloc=0"
)

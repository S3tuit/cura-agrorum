cmake_minimum_required(VERSION 3.20)

project(cura_agrorum_firmware_host_tests C)

include(CTest)

set(FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

find_package(OpenSSL REQUIRED)

add_executable(
  test_node_core
  fake_node_core_dependencies.c
  test_node_core.c
  test_node_core_support.c
  test_node_core_initialization.c
  test_node_core_delivery.c
  test_node_core_transitions.c
  test_node_core_finalization.c
  ${FIRMWARE_ROOT}/components/node_common/node_common.c
  ${FIRMWARE_ROOT}/components/node_core/node_core.c
  ${FIRMWARE_ROOT}/components/protocol_v2_lora/protocol_v2_lora_crypto.c
  ${FIRMWARE_ROOT}/components/protocol_v2_lora/protocol_v2_lora_schema_generated.c
  ${FIRMWARE_ROOT}/components/sx1262_radio/sx1262_radio_timing.c
)

target_include_directories(
  test_node_core
  PRIVATE
    ${FIRMWARE_ROOT}/components/node_common/include
    ${FIRMWARE_ROOT}/components/node_core/include
    ${FIRMWARE_ROOT}/components/node_persistence/include
    ${FIRMWARE_ROOT}/components/node_sensors/include
    ${FIRMWARE_ROOT}/components/protocol_v2_lora/include
    ${FIRMWARE_ROOT}/components/sx1262_radio/include
    ${FIRMWARE_ROOT}/components/sx1262_radio/private_include
)
target_compile_features(test_node_core PRIVATE c_std_11)
target_compile_definitions(test_node_core PRIVATE CURA_LORA_V2_CRYPTO_USE_OPENSSL)
target_compile_options(
  test_node_core
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
target_link_libraries(test_node_core PRIVATE OpenSSL::Crypto)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(
    test_node_core PRIVATE -fsanitize=address,undefined
                           -fno-omit-frame-pointer
  )
  target_link_options(
    test_node_core PRIVATE -fsanitize=address,undefined
                           -fno-omit-frame-pointer
  )
endif()

set(
  NODE_CORE_TEST_SCENARIOS
  first_attempt_success
  claim_failure_invalidates_and_cleans_up
  valid_rtc_reaches_next_reading
  rejected_rtc_is_zeroed
  sensor_validity_maps_independently
  run_time_and_sampling_deadline_boundaries
  pending_failure_allows_ram_only_delivery
  invalid_composition_returns_without_side_effects
  retries_reuse_identical_frame
  invalid_acks_share_one_receive_interval
  unknown_ack_status_reports_status_diagnostic
  radio_progress_and_rx_errors_are_accounted
  started_tx_deadline_is_charged_local_error
  ack_retry_timestamp_boundary_is_enforced
  budget_boundaries_are_inclusive_and_independent
  wall_clock_exhaustion_reports_cycle_deadline
  zero_epoch_first_attempt_timestamp_is_retained
  current_ack_outcomes_apply_policy
  current_removal_failure_prevents_backlog
  backlog_is_drained_in_persistence_order_with_unchanged_body
  backlog_retry_later_and_lookup_failure_stop
  backlog_quarantine_removal_combinations
  accepted_backlog_removal_failure_stops
  airtime_budget_is_shared_across_backlog
  backlog_silence_retries_the_same_frame
  delivery_events_bracket_retries_and_failures_are_best_effort
  diagnostic_failure_is_not_logged_recursively
  current_and_backlog_metrics_accumulate_in_correct_domains
  unaccepted_current_has_zero_delivery_metric
  awake_time_includes_cleanup_and_sync_and_sleep_is_terminal
  rtc_metric_boundaries_and_semantics_are_enforced
  sync_induced_metric_overflow_has_no_late_diagnostic
  component_diagnostics_are_copied_opaquely
  cleanup_failures_do_not_prevent_sync_or_sleep
)

foreach(SCENARIO IN LISTS NODE_CORE_TEST_SCENARIOS)
  add_test(NAME "node_core.${SCENARIO}" COMMAND test_node_core "${SCENARIO}")
  set_tests_properties(
    "node_core.${SCENARIO}"
    PROPERTIES ENVIRONMENT
               "ASAN_OPTIONS=detect_leaks=1:abort_on_error=1:halt_on_error=1:fast_unwind_on_malloc=0"
  )
endforeach()

add_executable(
  test_node_persistence
  fake_node_persistence_backend.c
  test_node_persistence.c
  test_node_persistence_support.c
  test_node_persistence_nvs.c
  test_node_persistence_records.c
  test_node_persistence_recovery.c
  test_node_persistence_faults.c
  test_node_persistence_retention.c
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
    NODE_PERSISTENCE_QUARANTINE_LOG_LIMIT=512
    NODE_PERSISTENCE_DIAGNOSTIC_LOG_LIMIT=512
    NODE_PERSISTENCE_DELIVERY_LOG_LIMIT=512
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

add_executable(
  test_node_sensors
  fake_esp_gpio.c
  fake_node_sensors_backend.c
  test_node_sensors.c
  ${FIRMWARE_ROOT}/components/node_common/node_common.c
  ${FIRMWARE_ROOT}/components/node_sensors/node_sensors.c
  ${FIRMWARE_ROOT}/components/node_sensors/node_sensors_power_gate_esp.c
)

target_include_directories(
  test_node_sensors
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/fakes
    ${FIRMWARE_ROOT}/components/node_common/include
    ${FIRMWARE_ROOT}/components/node_sensors/include
    ${FIRMWARE_ROOT}/components/node_sensors/private_include
)

target_compile_features(test_node_sensors PRIVATE c_std_11)
target_compile_definitions(
  test_node_sensors
  PRIVATE
    NODE_SENSORS_TESTING=1
    CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS=200
)
target_compile_options(
  test_node_sensors
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
    test_node_sensors PRIVATE -fsanitize=address,undefined
                              -fno-omit-frame-pointer
  )
  target_link_options(
    test_node_sensors PRIVATE -fsanitize=address,undefined
                              -fno-omit-frame-pointer
  )
endif()

add_test(NAME node_sensors COMMAND test_node_sensors)
set_tests_properties(
  node_sensors
  PROPERTIES ENVIRONMENT
             "ASAN_OPTIONS=detect_leaks=1:abort_on_error=1:halt_on_error=1:fast_unwind_on_malloc=0"
)

add_executable(
  test_node_sensors_identity
  test_node_sensors_identity.c
  ${FIRMWARE_ROOT}/components/node_sensors/node_sensors_ds18b20_identity.c
)

target_include_directories(
  test_node_sensors_identity
  PRIVATE
    ${FIRMWARE_ROOT}/components/node_common/include
    ${FIRMWARE_ROOT}/components/node_sensors/include
    ${FIRMWARE_ROOT}/components/node_sensors/private_include
)
target_compile_features(test_node_sensors_identity PRIVATE c_std_11)
target_compile_options(
  test_node_sensors_identity
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
    test_node_sensors_identity PRIVATE -fsanitize=address,undefined
                                       -fno-omit-frame-pointer
  )
  target_link_options(
    test_node_sensors_identity PRIVATE -fsanitize=address,undefined
                                       -fno-omit-frame-pointer
  )
endif()

add_test(NAME node_sensors_identity COMMAND test_node_sensors_identity)
set_tests_properties(
  node_sensors_identity
  PROPERTIES ENVIRONMENT
             "ASAN_OPTIONS=detect_leaks=1:abort_on_error=1:halt_on_error=1:fast_unwind_on_malloc=0"
)

add_executable(
  test_node_platform_esp
  test_node_platform_esp.c
  ${FIRMWARE_ROOT}/components/node_platform_esp/node_platform_esp.c
)

target_include_directories(
  test_node_platform_esp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/fakes
    ${FIRMWARE_ROOT}/components/node_core/include
    ${FIRMWARE_ROOT}/components/node_platform_esp/include
)
target_compile_features(test_node_platform_esp PRIVATE c_std_11)
target_compile_options(
  test_node_platform_esp
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
    test_node_platform_esp PRIVATE -fsanitize=address,undefined
                                   -fno-omit-frame-pointer
  )
  target_link_options(
    test_node_platform_esp PRIVATE -fsanitize=address,undefined
                                   -fno-omit-frame-pointer
  )
endif()

add_test(NAME node_platform_esp COMMAND test_node_platform_esp)
set_tests_properties(
  node_platform_esp
  PROPERTIES ENVIRONMENT
             "ASAN_OPTIONS=detect_leaks=1:abort_on_error=1:halt_on_error=1:fast_unwind_on_malloc=0"
)

add_executable(
  test_sx1262_radio
  fake_sx1262_radio_backend.c
  test_sx1262_radio.c
  ${FIRMWARE_ROOT}/components/node_common/node_common.c
  ${FIRMWARE_ROOT}/components/sx1262_radio/sx1262_radio.c
  ${FIRMWARE_ROOT}/components/sx1262_radio/sx1262_radio_backend_common.c
  ${FIRMWARE_ROOT}/components/sx1262_radio/sx1262_radio_timing.c
)

target_include_directories(
  test_sx1262_radio
  PRIVATE
    ${FIRMWARE_ROOT}/components/node_common/include
    ${FIRMWARE_ROOT}/components/sx1262_radio/include
    ${FIRMWARE_ROOT}/components/sx1262_radio/private_include
)
target_compile_features(test_sx1262_radio PRIVATE c_std_11)
target_compile_options(
  test_sx1262_radio
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
    test_sx1262_radio PRIVATE -fsanitize=address,undefined
                              -fno-omit-frame-pointer
  )
  target_link_options(
    test_sx1262_radio PRIVATE -fsanitize=address,undefined
                              -fno-omit-frame-pointer
  )
endif()

set(
  SX1262_RADIO_TEST_SCENARIOS
  profile_and_lazy_initialization
  invalid_arguments_and_initial_deadline
  initialization_failure_is_cached
  payload_boundaries_and_watchdog
  insufficient_watchdog_window_is_rejected_before_set_tx
  deadline_reached_before_set_tx
  failure_before_set_tx_reports_no_attempt
  uncertain_set_tx_preserves_attempt
  uncertain_command_before_set_tx_reports_no_attempt
  failure_after_set_tx_preserves_attempt
  clear_failure_preserves_tx_done
  tx_timeout_and_unexpected_irq
  combined_tx_done_and_timeout_preserves_progress
  unexpected_irq_reports_device_error
  receive_exact_packet_and_iq_transition
  receive_pending_irq_at_deadline_wins
  receive_snapshots_before_clear_and_rearms
  receive_deadline_keeps_rx_armed
  receive_discards_phy_errors
  oversized_receive_is_bounded
  late_packet_is_discarded
  same_absolute_deadline_is_not_extended
  receive_local_error_is_distinct
  unexpected_receive_irq_fails
  sleep_lifecycle
  sleep_failure_retries_and_preserves_first_error
)

foreach(SCENARIO IN LISTS SX1262_RADIO_TEST_SCENARIOS)
  add_test(NAME "sx1262_radio.${SCENARIO}"
           COMMAND test_sx1262_radio "${SCENARIO}")
  set_tests_properties(
    "sx1262_radio.${SCENARIO}"
    PROPERTIES ENVIRONMENT
               "ASAN_OPTIONS=detect_leaks=1:abort_on_error=1:halt_on_error=1:fast_unwind_on_malloc=0"
  )
endforeach()

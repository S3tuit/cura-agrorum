idf_component_register(
    SRCS
        "flash_metrics.c"
        "littlefs_storage_benchmark.c"
        "storage_strategy.c"
    INCLUDE_DIRS "."
    REQUIRES esp_partition esp_timer littlefs
)

# Count the actual partition operations issued by the unmodified LittleFS
# adapter while forwarding them to ESP-IDF's real implementation.
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=esp_partition_read"
    "-Wl,--wrap=esp_partition_write"
    "-Wl,--wrap=esp_partition_erase_range"
)

idf_component_register(
    SRCS "node_platform_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES node_core
    PRIV_REQUIRES esp_hw_support esp_system esp_timer freertos log protocol_v2_lora
)

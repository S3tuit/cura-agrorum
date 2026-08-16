idf_component_register(
    SRCS "node_core.c"
    INCLUDE_DIRS "include"
    REQUIRES
        node_common
        node_persistence
        node_sensors
        protocol_v2_lora
        sx1262_radio
)

idf_component_register(
    SRCS
        "protocol_v2_lora_schema_generated.c"
        "protocol_v2_lora_crypto.c"
    INCLUDE_DIRS "include"
    REQUIRES mbedtls
)

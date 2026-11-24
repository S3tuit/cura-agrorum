idf_component_register(SRCS "assign.c" "commission.c" INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer)

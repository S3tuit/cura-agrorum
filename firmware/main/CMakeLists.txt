idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bmp280 ds18b20 esp_adc esp_driver_gpio esp_timer
                             esp_hw_support i2cdev onewire_bus)

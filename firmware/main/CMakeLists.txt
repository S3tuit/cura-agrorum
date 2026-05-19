idf_component_register(SRCS "main.c" "tcp.c" "wifi.c" "handshake.c"
                    INCLUDE_DIRS "."
                    REQUIRES bmp280 ds18b20 esp_adc esp_driver_gpio esp_timer
                             esp_event esp_hw_support esp_netif esp_wifi i2cdev
                             lwip mdns nvs_flash onewire_bus)

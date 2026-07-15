idf_component_register(SRCS "main.c" "wire.c" "wifi.c" "storage.c"
                             "event_queue.c" "fault_cntl.c"
                    INCLUDE_DIRS "."
                    REQUIRES bmp280 ds18b20 esp_driver_gpio esp_timer
                             esp_event esp_hw_support esp_netif esp_wifi i2cdev
                             littlefs lwip nvs_flash onewire_bus soil_sensor)

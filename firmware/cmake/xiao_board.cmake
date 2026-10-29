# D7/GPIO44 owns the sensor gate, including in builds that do not use sensors.
# Reject a stale UART configuration before it can drive that pad at boot.
if(NOT IDF_TARGET STREQUAL "esp32s3" OR
   NOT CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG OR
   NOT CONFIG_ESP_CONSOLE_SECONDARY_NONE OR
   NOT CONFIG_FREERTOS_UNICORE OR
   NOT CONFIG_ESPTOOLPY_FLASHSIZE STREQUAL "8MB")
    message(FATAL_ERROR "XIAO assembly requires esp32s3, primary USB Serial/JTAG console, no secondary console, single core and 8MB flash; regenerate sdkconfig from this app's defaults")
endif()

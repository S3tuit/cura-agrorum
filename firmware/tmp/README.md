# Cura no-op firmware

This is a standalone ESP-IDF firmware whose `app_main()` intentionally does
nothing. After ESP-IDF startup, the app main task returns and the system is left
idle.

Build and flash from this directory:

```sh
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p PORT flash
```

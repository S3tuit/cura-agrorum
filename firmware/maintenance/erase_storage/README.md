# erase_storage

Destructive maintenance firmware for preparing a node before deployment.

When flashed and run, it:

- Logs an `ESP_LOGE` warning.
- Waits 5 seconds.
- Erases and reinitializes the default `nvs` partition.
- Formats and mounts the `storage` LittleFS partition.
- Logs success or failure.
- Enters deep sleep with no wake source configured.

Build and flash from this directory:

```bash
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p PORT flash monitor
```

This app reuses `../../partitions.csv`, so it targets the same storage layout as
the production firmware.

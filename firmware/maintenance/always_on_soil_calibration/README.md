# always_on_soil_calibration

Maintenance firmware for collecting soil sensor calibration readings while the
ESP32 stays awake.

It reuses the production `soil_sensor_read_mv()` path, then once per second:

- Reads the current soil voltage in mV.
- Stores successful readings in a 1000-sample RAM buffer.
- Logs the current value, running mean, and running median.
- Uses `vTaskDelay()` for 1 second before the next sample.
- Delays forever after 1000 successful samples.

Failed reads are logged, are not counted toward the 1000-sample limit, and are
retried after the next 1-second delay.

Build and flash from this directory:

```bash
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p PORT flash monitor
```

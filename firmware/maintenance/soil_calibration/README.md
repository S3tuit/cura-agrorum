# soil_calibration

Maintenance firmware for collecting soil sensor calibration readings.

It reuses the production `soil_sensor_read_mv()` path, then on each wake:

- Reads the current soil voltage in mV.
- Stores successful readings in a 1000-sample RTC buffer.
- Logs the current value, running mean, and running median.
- Enters deep sleep for 2 seconds before the next sample.
- Sleeps forever after 1000 successful samples.

The RTC buffer is reset on a fresh non-timer boot. Failed reads are logged, are
not counted toward the 1000-sample limit, and are retried after the next
2-second deep-sleep interval.

Build and flash from this directory:

```bash
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p PORT flash monitor
```

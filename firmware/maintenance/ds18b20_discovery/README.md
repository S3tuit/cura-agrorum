# Discover DS18B20 ROM identities

Use the SEN0193/DS18B20 wiring in [HARDWARE.md](../../HARDWARE.md): GPIO4 for
the externally powered DS bus and GPIO44 for its active-low open-drain gate.
This image discovers up to two DS18B20 probes and prints their ROMs once, then
releases the bus and powers the rail off. It never reads measurements, opens
node storage or transmits. Do not flash it on the SEN0600 fixture: its shared
pins and gate polarity have different functions.

From the repository root, with the correct fixture connected:

```sh
source ~/esp/esp-idf/export.sh
idf.py -C firmware/maintenance/ds18b20_discovery -p /dev/ttyACM0 build flash monitor
```

Copy each `DS18B20_ROM` value exactly into the desired slot in
`firmware/deploy/sdkconfig.sensors`. Enumeration order does not assign slots.
To identify physical probes, discover them individually, removing all power
before changing connectors. Empty/error/extra-device results are not a valid
two-probe inventory. The app reports cleanup status even after discovery fails.
Flashing replaces the application; it does not erase NVS or LittleFS. Build the
production/acquisition image with the selected ROMs afterward.

# Assign SEN0600 address 2

This one-shot app commissions **one physically isolated SEN0600** from Modbus
address 1 to 2, at 9600 8N1. Use the GPIO6 active-high AO3401A/2N3904 fixture;
TX/RX are GPIO1/2, DE is GPIO4 and /RE is GPIO43. BME280 SCL is GPIO44 and is
not initialized by this app. This is the PCB bench mapping, not the GPIO44
gate mapping still present in the production checkout's defaults.

The [DFRobot register specification](https://wiki.dfrobot.com/sen0600/docs/19789)
defines function 0x06 and address register 0x07D0. The app reads that register
at addresses 2 and 1 first, writes 2 once only if 1 responds correctly and 2
is silent, then checks that 2 returns its address and 1 is silent. CRC,
function, length, register value and write echo are checked. An already
commissioned probe is read without writing again. Silence after the write
is resolved by readback; malformed acknowledgments stop with an uncertain
result. There are no automatic write retries or broadcasts.

Two probes at address 1 can produce indistinguishable replies. Software cannot
establish that only one is present: disconnect the first probe completely
before commissioning the second. The app neither accesses node storage nor
initializes radio/BME280. It releases UART/control pins and powers the switched
rail off after the attempt. It does not restart or repeat until reset.

## Build and use

```sh
# Select a working ESP-IDF venv with IDF_PYTHON_ENV_PATH if the default
# Python changed since ESP-IDF installation.
source ~/esp/esp-idf/export.sh
idf.py -C firmware/maintenance/sen0600_address build
python -m pytest firmware/maintenance/sen0600_address/test_commission.py -q
```

Before flashing, retain the installed application and a private flash backup;
node applications contain credentials. Verify the identified XIAO and its
partition table. This app uses the production partition layout, but flash only
`build/cura_sen0600_address.bin` at `0x10000`, without automatic reset. Do not
use `erase-flash` or the whole-project flash command. Keep the source battery
disconnected and isolate the carrier 3V3 path during USB service as specified
in the [PCB service-power procedure](../../../../cura-agrorum-power-pcbs/README.md#control-pins-and-service-power).

With power removed, connect only the second probe and verify GPIO6 gate wiring.
For the commissioning trial, power the fixture from XIAO USB alone, with the
SparkFun output and battery disconnected from the fixture and PPK bypassed.
Observe local 3V3 and sensor 6 V on the scope; USB-powered startup capability
still needs checking. This temporary arrangement captures the USB console
without paralleling two 3V3 supplies. Restore the regulated-source/PPK
arrangement, with XIAO USB absent, for load characterization.
Starting/resetting the app automatically attempts commissioning; no console
command is required. Save the complete console output, including request/reply
hex and `RESULT`/`CLEANUP` lines, in the bench logbook.

`ASSIGNED_2` proves immediate readback only. Remove power and verify that the
sensor rail has discharged, then run again on the same isolated probe. Expect
`ALREADY_2` with no write and `CLEANUP ESP_OK`. This checks persistence across
an observed power cycle; elapsed time alone does not prove rail discharge.
Other result/cleanup values require investigation before connecting both probes.

Restore the retained node application by an application-only flash. Use address
2 in its sole active slot to check the second probe alone, then addresses 1 and
2 for the combined fixture. Verify both readings at the receiver before load
characterization. Preserve NVS, counters and LittleFS through all app changes.

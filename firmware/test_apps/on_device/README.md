# ESP32-C6 hardware tests

This ESP-IDF application runs the on-device `node_persistence` Unity tests. It
uses dedicated `nvs_test` and `storage_test` partitions and erases their state
between cases. Multi-stage cases preserve state only across their deliberate
software resets.

Install the host runner and activate ESP-IDF once per shell:

```sh
.venv/bin/pip install -r firmware/tests/requirements-hardware.txt
source ~/esp/esp-idf/export.sh
```

Then run one of:

```sh
make test-hardware PORT=/dev/ttyUSB0       # fast cases only
make test-hardware-slow PORT=/dev/ttyUSB0  # slow cases only
make test-hardware-all PORT=/dev/ttyUSB0   # every case
```

Each target builds and flashes this test image, replacing the firmware already
on the board. Build output, generated configuration and managed-component links
are intentionally ignored by Git.

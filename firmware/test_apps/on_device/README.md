# ESP32-C6 hardware tests

This ESP-IDF application runs the on-device `node_persistence` Unity matrix and
the production `node_platform_esp` clock, randomness and timer-deep-sleep smoke
tests. Persistence cases use dedicated `nvs_test` and `storage_test` partitions
and erase their state between cases. Multi-stage cases preserve device state
only across their deliberate software-reset or deep-sleep stages.

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

The slow set includes the one-minute platform timer-deep-sleep round trip. Its
host runner checks elapsed time between explicit pre-sleep and post-boot serial
markers and the device verifies the protocol-facing deep-sleep reset reason.

Each target builds and flashes this test image, replacing the firmware already
on the board. Build output, generated configuration and managed-component links
are intentionally ignored by Git.

# ESP32-C6 hardware tests

This receiver-free ESP-IDF application runs the on-device `node_persistence`
Unity matrix, the complete bare-board RTC suite, six `node_core` integration
scenarios, and the production `node_platform_esp` clock, randomness, software
reset and timer-deep-sleep smoke tests. It requires only an
ESP32-C6-DEVKITM-1-N4; no receiver, radio module, sensor or power circuit is
accessed.

The core cases link production controller, persistence, protocol codec/crypto
and ESP-IDF reset/deep-sleep code. A fixed test identity and deterministic
test-only sensor, receiver-free radio, clock and randomness adapters replace
external I/O. Persistence uses dedicated `nvs_test` and `storage_test`
partitions. Every independent case erases RTC, NVS and LittleFS state before and
afterward; a multi-stage case preserves state only across its deliberate reset
or deep-sleep transitions.

## Connection

Connect the board through the USB-C connector labelled `UART`, which normally
appears as `/dev/ttyUSB0`. Do not use the connector labelled `USB`: the native
USB device disconnects while the ESP32 resets or enters deep sleep, but the test
runner requires a continuously enumerated serial ingress.

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

The fast set includes the ordinary persistence matrix, all six receiver-free
core integrations, all six fast RTC scenarios (including both RTC/NVS
continuity outcomes), and platform clock/randomness/software-reset checks. The
slow set includes persistence compaction, quota and churn, the
one-minute platform timer-deep-sleep round trip, and exactly 20 retained RTC
deep-sleep round trips. The host runner measures the one-minute interval between
explicit serial markers; the device verifies the reset reason and timer wake
cause.

Each target builds and flashes this test image, replacing the firmware already
on the board. Build output, generated configuration and managed-component links
are intentionally ignored by Git.

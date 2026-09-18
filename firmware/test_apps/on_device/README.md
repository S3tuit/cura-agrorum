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
external I/O. Persistence uses test-labelled `nvs_test` and `storage_test`
partitions that physically overlap production storage. Every independent case
erases RTC, NVS and LittleFS state before and afterward; a multi-stage case
preserves state only across its deliberate reset or deep-sleep transitions.

## Destructive test storage

The [test partition table](partitions.csv) and
[production partition table](../../partitions.csv) use the same physical ranges:

| Test label | Production label | Offset | Size |
|---|---|---|---|
| `nvs_test` | `nvs` | `0x9000` | 24 KiB |
| `storage_test` | `storage` | `0x110000` | 2944 KiB |

Different labels do not isolate flash bytes. `hwtest_erase_state()` erases NVS
and formats LittleFS; flashing also replaces the application and partition table.
Before running any hardware Make target, record the selected board/UART and the
operator's explicit permission to destroy its existing application and storage.
If existing state must be preserved, stop before flashing or selecting a case.
The unchanged Make targets do not enforce this procedural authorization.

After these tests, never restore an old credential header and resume its erased
counters. Before authenticated transmission, provision a new node ID and key
and perform the contracted full NVS/LittleFS/RTC identity-state reset. Do not
migrate old backlog into the new identity. The fixed test identity uses fake
radio operations and must never become a transmitting production identity.
See [keys and provisioning](../../../protocol/protocol-v2-lora/README.md#keys-and-provisioning).
An ordinary rebuild outside this destructive workflow must preserve valid
counters; this procedure does not authorize erasing other boards or live storage.

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

After recording the [destructive authorization](#destructive-test-storage),
run one of:

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

The repeated-RTC runner also has a host regression for delayed Unity submenus
and discarded early stage selectors. It requires the runner dependencies above
but does not access a board:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest \
    firmware/test_apps/on_device/runner_tests -q
```

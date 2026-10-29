# Firmware tests

The repository must pass its implemented tests. Before committing a new test,
make it pass; after software or hardware changes, rerun all necessary tests.
Contributors choose those tests during implementation using their judgment.
Keep useful failure lessons beside the affected code or in its documentation.

## Choose the required environment

| Location | Runs with | Entry point |
|---|---|---|
| [host](host/) | Native compiler; deterministic hardware/clock fakes | `make test-host` |
| [target](target/README.md) | Bare S3, real NVS/LittleFS/reset/sleep, fake sensor/radio I/O | `make test-hardware*` as documented locally |
| [sensor carrier](../hardware/sensors/tests/carrier/README.md) | S3, sensors, meter and operator | Local pytest runner and fixture guide |
| [radio component](../hardware/radio/tests/component/README.md) | S3 radio and separately controlled Pi peer | [Joint RF procedures](../../tests/rf/README.md) |

Each target/hardware directory contains a complete independently buildable
ESP-IDF project, its configuration, device code and host runner where applicable.
Each project resolves managed dependencies, including LittleFS, into its own
ignored `managed_components/` directory using its committed `dependencies.lock`.
The first build needs dependency downloads; no production build is required first.
[support](support/) holds shared persistence helpers and the native USB console
transport used by test images and runners. Test names and assertions live in their source;
this guide does not maintain a parallel case inventory.

## Host commands

Install the Python runner dependencies, then run from the repository root:

```sh
.venv/bin/pip install -r firmware/tests/requirements-hardware.txt
make test-host
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest firmware/tests/target/runner_tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=firmware/hardware/sensors/tests/carrier \
  .venv/bin/python -m pytest firmware/hardware/sensors/tests/carrier/runner_tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest firmware/tests/support/runner_tests -q
make test-rf-host
```

The target and sensor runner host tests create temporary test inputs; neither
requires ESP-IDF configuration or an existing target build. RF tooling has its
own [host prerequisites](../../tests/rf/README.md#host-checks).

`make test-host` builds CMake/CTest targets with strict warnings, ASan and UBSan.
The real core/codec uses deterministic component fakes; component policy uses
private backend seams, and BME driver/adapter tests use the same pinned source
as firmware. Fresh processes isolate wake-local singleton state. Protocol
generation and shared Python/C vectors remain in the [protocol suite](../../protocol/protocol-v2-lora/README.md).

Activate ESP-IDF for target builds:

```sh
source ~/esp/esp-idf/export.sh
make test-hardware-build
make test-rf-build
idf.py -C firmware/hardware/sensors/tests/carrier build
```

These commands build without flashing. Device execution is documented by each
suite: target tests and sensor `acquire`/`adc-reference` erase partitions that
overlap production storage. Follow their DUT confirmation and identity-replacement procedures.

## Native USB across reset and sleep

Use the XIAO USB-C native Serial/JTAG port (Espressif VID:PID `303a:1001`),
normally `/dev/ttyACM0`, with its unique USB serial number. Keep the same USB
socket and cable through a run. The shared transport binds that identity and
USB location, reopens the same board after reset/deep sleep even if its tty
number changes, and shares the reopened handle with the reader, writer and
reset controller. Reconnect never sends a reset or repeats a test command.
Ambiguous identity, substitution, uncertain writes or an expired reconnect
deadline fail the run. Test runners allow up to 650 seconds of absence for
manual deep-sleep observations; production RF capture allows 930 seconds.
Explicit unlimited sensor exploration permits unlimited absence while asleep
or held in reset, retaining the same device-substitution and write guards.

Test images repeat `CURAG_USB_READY <nonce>` for at most 30 seconds after boot.
The attached runner acknowledges that boot before the image emits identity/menu
records or enters a test. Use the runner rather than typing test commands into
a monitor. Production firmware and the formatter run autonomously; neither
uses this boot barrier. RF-observation builds have a separate
[terminal receipt/stop boundary](../../tests/rf/README.md#accelerated-rf-execution).
A successful host transport test does not qualify actual
USB wake/re-enumeration on a board.

## Results and isolated sessions

Logs, measurements, JUnit and intermediate A/B files are temporary working data.
Keep them while investigating failures or completing a sequence; no permanent
acceptance archive is required. Use ignored session directories and optional
`session.notes.md` explanations so another contributor can find what happened.
Preserve lasting lessons in code, regression tests or the owning procedure.

A separate worktree is useful when a long hardware session needs its own build,
configuration and results. It is optional for routine tests. A detached worktree
created at `HEAD` contains committed sources only: explicitly include intended
uncommitted modifications and new files when testing ongoing work. Keep test
output ignored there, bring back source changes through the normal review flow,
and remove the workspace only after diagnosis and follow-up are complete. A
sibling directory avoids relying on temporary-directory retention.

## Scope of observations

Host results and successful builds do not establish physical measurements.
Bare-S3 restart tests run after operations return; injected corrupt records
exercise recovery on LittleFS without reproducing power loss during a flash
write. Physical interruption of in-progress writes needs a separate rig.
Sensor fixtures own electrical limits; RF tests own interoperability/timing.
Accelerated RF sleep settings do not establish production cadence or endurance.

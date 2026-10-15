# Firmware tests

The repository must pass its implemented tests. Before committing a new test,
make it pass; after software or hardware changes, rerun all necessary tests.
Contributors choose those tests during implementation using their judgment.
Keep useful failure lessons beside the affected code or in its documentation.

## Choose the required environment

| Location | Runs with | Entry point |
|---|---|---|
| [host](host/) | Native compiler; deterministic hardware/clock fakes | `make test-host` |
| [target](target/README.md) | Bare C6, real NVS/LittleFS/reset/sleep, fake sensor/radio I/O | `make test-hardware*` as documented locally |
| [sensor carrier](../hardware/sensors/tests/carrier/README.md) | C6, sensors, meter and operator | Local pytest runner and fixture guide |
| [radio component](../hardware/radio/tests/component/README.md) | C6 radio and separately controlled Pi peer | [Joint RF procedures](../../tests/rf/README.md) |

Each target/hardware directory contains a complete independently buildable
ESP-IDF project, its configuration, device code and host runner where applicable.
Each project resolves managed dependencies, including LittleFS, into its own
ignored `managed_components/` directory using its committed `dependencies.lock`.
The first build needs dependency downloads; no production build is required first.
[support](support/) holds persistence inspection/erase helpers shared by the
bare-C6 and sensor projects. Test names and assertions live in their source;
this guide does not maintain a parallel case inventory.

## Host commands

Install the Python runner dependencies, then run from the repository root:

```sh
.venv/bin/pip install -r firmware/tests/requirements-hardware.txt
make test-host
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest firmware/tests/target/runner_tests -q
.venv/bin/python -m pytest firmware/hardware/sensors/tests/carrier/runner_tests -q
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
suite: target tests and sensor `reading` erase partitions that overlap production
storage. Follow their DUT confirmation and identity-replacement procedures.

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
Bare-C6 restart tests run after operations return; injected corrupt records
exercise recovery on LittleFS without reproducing power loss during a flash
write. Physical interruption of in-progress writes needs a separate rig.
Sensor fixtures own electrical limits; RF tests own interoperability/timing.
Accelerated RF sleep settings do not establish production cadence or endurance.

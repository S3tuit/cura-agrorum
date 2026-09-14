# Bosch BME280 dependency and regression ownership

Driver corrections are maintained in [S3tuit/BME280_SensorAPI](https://github.com/S3tuit/BME280_SensorAPI).
`source_pin.cmake` identifies the complete fork commit and SHA256 of the compiled
source, headers and license. The private ESP-IDF adapter is owned by
`node_sensors`; this component only builds the pinned Bosch driver in double mode.

The fork and its regressions are in separate repositories by agreement:
`firmware/tests/host/test_bme280_driver.c` compiles the actual Bosch source with
scripted transport callbacks, and `test_node_sensors_bme280.c` compiles the actual
adapter and driver against ESP-IDF/time boundary fakes. They run through
`CCACHE_DISABLE=1 make test-host`, with the existing CTest/sanitizer infrastructure.
The fork README points back here. Updating a driver pin requires rerunning these
regressions and the affected firmware/fixture checks in `firmware/TESTING.md`.
The fork contains no second test runner. A standalone reproducer is deferred
until the first upstream bug report is prepared; this does not defer regressions.

Both host CMake and ESP-IDF use `resolve_source.cmake`. The normal path fetches
that exact Git commit from the hosted fork and verifies file hashes. The current
published pin is `5f4119ed6ee638abc573e75516ad9aa6e1cd612a`. Missing or
mismatched sources fail configuration. There is no fallback to a sibling checkout
or an old managed component. To verify a clean local copy of the **same pin**
before publication, set `CURA_BME280_SOURCE_DIR` explicitly when configuring:

```sh
CURA_BME280_SOURCE_DIR=/home/s3tuit/devspace/BME280_SensorAPI CCACHE_DISABLE=1 make test-host
source ~/esp/esp-idf/export.sh
idf.py -C firmware/test_apps/sensor_carrier \
  -DCURA_BME280_SOURCE_DIR=/home/s3tuit/devspace/BME280_SensorAPI build
```

The override is a CMake cache setting; subsequent builds keep it until explicitly
changed/cleared. It must still match the commit and content pins and have no
tracked modifications. `bosch-bme280-source.txt` in each build directory records
the consumed checkout and commit. This is verification of pinned source bytes,
not proof that the hosted fork already serves that commit. Publishing a local
fork commit is a separate action; an unpublished pin cannot be fetched remotely.

## Local correction evidence

The upstream base is `c90d419492e26dd95586598a794e65eb2760753a`.
The originally tested local fork branch `cura-failure-low-power` contained separate changes:

- `ff69236`: preserve the communication error when a later NVM-status read fails
  after an earlier busy observation. The existing `nvm_later_error` driver test
  aborts on its assertion against the unmodified upstream source; the corrected
  pin passes. A first-read failure and the nominal combined-read case pass on
  both sources, distinguishing the specific regression.
- `e8fd7bebe3c297aec94b4c76baf58750e5457c3c`: project double-compensation adaptation
  and this repository relationship. The independent temperature/pressure outlier
  and undefined-pressure tests fail against the original clipping/fallback
  behavior and pass against the pin. This adaptation is distinct from the
  upstream polling defect.

On 2026-09-12, the full native firmware suite passed 127 cases (56 focused Bosch
and adapter cases), with strict project warnings and ASan/UBSan. Carrier native
and orchestration checks passed 559 cases. Carrier and bare-C6 builds succeeded;
production configuration and the selected sources compile, but the complete
production build is blocked by the absent private
`firmware/main/protocol_v2_lora_identity.h`. No identity was manufactured.
No protocol behavior changed. No hardware run or electrical acceptance follows
from these checks.

Local review artifacts are in ignored `firmware/build-host/bme280-review/`:
separate `git format-patch` files, `upstream-comparison.json` (process exit status,
not a Bosch diagnostic), and `build-inspection.json`. These are local preparation artifacts; no Bosch upstream issue/PR or standalone
reproducer was created. Publication of the dependency fork is recorded below.
The user subsequently published the changes as squash commit
`5f4119ed6ee638abc573e75516ad9aa6e1cd612a` on master. All four source/header/license
hashes match the previously tested pin. Cura now pins that published commit;
host and both test-app builds have also passed using fresh remote resolution
(`fetched_pin`) with the local override cleared.

On 2026-09-14, the approved channel-validation correction changed only Cura's
adapter and contracts: verify actual x1 settings after configuration, before
triggering and at completed conversion, accepting representable raw midpoint
codes when those checks pass. This requires no Bosch fork patch or pin change.
The firmware suite passes 147 cases (76 focused driver/adapter cases), including
independent midpoint vectors and disabled/non-x1 settings at each validation
phase; all 560 carrier native/runner cases pass. Both C6 test apps build with
the same fetched pin. Recorded hardware runs are retained in
[sensor-carrier evidences/](../../test_apps/sensor_carrier/evidences/).
The initial failed nominal run remains in local session logs; its raw code was
not captured.

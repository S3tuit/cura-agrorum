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

## Corrections protected by regressions

The fork preserves a communication error when an NVM-status read fails after
an earlier busy observation. Upstream polling could mask this error;
`nvm_later_error` in the driver tests protects the failure path.

Double compensation must preserve representable outliers and distinguish an
undefined pressure result, rather than clipping or substituting a plausible
value. Independent temperature/pressure vectors protect these adaptations.

The Cura adapter verifies actual x1 channel settings after configuration, before
triggering and at conversion completion. A raw midpoint is representable when
those checks pass; rejecting that code alone incorrectly rejects valid readings.
Disabled and non-x1 settings are tested at each validation phase.

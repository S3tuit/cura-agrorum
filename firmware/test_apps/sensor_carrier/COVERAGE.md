# Sensor test coverage

This index maps the sensor obligations in [TESTING.md](../../TESTING.md) to
implemented cases. [ARCHITECTURE.md](../../ARCHITECTURE.md)
owns runtime behavior, [INTERFACE.md](../../INTERFACE.md#node_sensors) owns sensor
results, and the [protocol reading contract](../../../protocol/protocol-v2-lora/README.md#reading-payload)
owns the encoded fields. Wiring belongs to [SENSOR_CARRIER.md](../on_device/SENSOR_CARRIER.md);
invocations belong to [README.md](README.md#acceptance-commands).

## Obligation-to-case mapping

Names in the executable column are exact Unity case names in
[test_sensor_carrier.c](main/test_sensor_carrier.c). The local
[carrier_assert_sample / carrier_assert_inventory](main/carrier_checks.c) checks
and [forwarding observer](main/carrier_observer.c) inspect real returned values
and production driver calls. They do not supply a sensor oracle.

Commands below use the [session helper](README.md#discovery-and-physical-probe-labeling).
Each hardware invocation requires its own confirmed DUT/fixture readiness;
this table is an index, not a batch of commands for one wiring state.

| Required sensor observation | Implemented executable / procedure |
|---|---|
| Complete configured inventory, absence detection and fresh acquisition boot | `carrier nominal preflight`, `carrier missing_ds0 preflight`, `carrier missing_ds1 preflight`, `carrier missing_bme280 preflight`; `fixture_preflight`; [runner](carrier_runner.py) checks identity/image and resets after released preflight resources |
| All five component groups, exact partial results, all seven encoded fields and flags | `carrier nominal acquisition and sample-return hold`; `carrier nominal core reading`; `fixture_reading` and [carrier_core_run](main/carrier_core.c) compare the sample actually consumed by core with opened frame plaintext and reopened pending storage |
| Atomic enclosure validity on success and failure | Nominal cases and `carrier missing_bme280 acquisition and sample-return hold` / `carrier missing_bme280 core reading` |
| At least 100 acquisitions, unchanged ROM identities, conversion freshness, duration and resource conditions | `carrier repeated nominal acquisition`; `--sensor-operation repeat --sensor-fixture nominal --sensor-repeat-count 100`; observer requires broadcast conversion, >=750 ms before addressed reads, 200 ms stabilization, 16 calibrated ADC reads per channel |
| Actual BME low-power mode without modifying the returned sample | `carrier BME280 sleep observation`; `observe_bme_sleep` also runs after each repeated acquisition; reads real F3/F4 without a reset, mode write or new conversion |
| Independent logical DS0 absence and exact diagnostic slot | `carrier missing_ds0 acquisition and sample-return hold`; `carrier missing_ds0 core reading`; [missing-DS procedure](README.md#missing-ds-probes-and-nominal-restoration) |
| Independent logical DS1 absence and exact diagnostic slot | Symmetric `carrier missing_ds1 acquisition and sample-return hold` / `carrier missing_ds1 core reading` |
| Whole BME absence with independent gated groups retained | `carrier missing_bme280 preflight` requires `probe=261`; acquisition requires enclosure-only `(ESP_ERR,264)`; [whole-connector removal](README.md#missing-bme280-and-nominal-restoration) |
| Soil ADC conversion and physical channel mapping | `carrier reference acquisition and sample-return hold` and `carrier adc_reference core reading`; [guided ADC A/B](README.md#adc-reference-positions-a-and-b), fresh measurements, <=75 mV error and >150 mV separation |
| DS logical identity follows configured ROM across connector exchange | `carrier reference acquisition and sample-return hold`, selected by `--sensor-operation ds-identity`; [guided thermal/connector A/B](README.md#ds-identity-through-a-connector-exchange), same predeclared warmed ROM and >=2 C separation |
| Sampling-owned shutdown after success and every returned failure; stable post-sampling back-power observation | `fixture_acquisition` and the untouched `sample-return` hold; guided nominal and all three missing fixtures |
| Idempotent final cleanup without bus initialization or rail enable | `carrier final cleanup hold`, two public force-off calls and `carrier_observer_cleanup_valid`; [final-cleanup command](README.md#acceptance-commands) |
| Stable active-low gate ON/OFF states | `carrier production gate-on hold`, `carrier production gate-off hold`; [electrical procedure](../../TESTING.md#node_sensors-manual-electrical-cases) |
| Intentional restart-owned cleanup | `carrier production restart cleanup`, `reset_stage_1` / `reset_stage_2`, production `node_platform_esp_restart` and forwarding cleanup observation |
| Held-reset and deep-sleep default-off; steady-state sleep back-power | `carrier enabled rail held reset`, `carrier enabled rail deep sleep`; [guided reset/sleep procedure](README.md#reset-held-reset-deep-sleep-and-back-power) |

The [reading sequence](README.md#production-core-reading-mapping) requires
nominal, missing DS0, missing DS1, missing BME, ADC A, ADC B and restored nominal.

| Reading fixture | Component validity | Protocol sensor flags (`flags & 0x00fe`) | Required outcome |
|---|---:|---:|---|
| nominal | `0x1f` | `0x00fe` | Seven exact same-acquisition values |
| missing_ds0 | `0x1b` | `0x00f6` | DS0 zero/invalid; unaffected values preserved |
| missing_ds1 | `0x17` | `0x00ee` | DS1 zero/invalid; unaffected values preserved |
| missing_bme280 | `0x0f` | `0x001e` | All three enclosure values zero/invalid together |
| adc_reference, A and B | `0x1f` | `0x00fe` | Fresh measured inputs map through the same core acquisition |

The local radio adapter emits no RF. Its returning terminal observer does not
enter real deep sleep.

## Host-only and deferred obligations

| Obligation | Executable coverage / status |
|---|---|
| Every sensor validity group maps independently; valid zero; complete sensor failure | [test_node_core_initialization.c](../../tests/host/test_node_core_initialization.c), `sensor_validity_maps_independently`; [test_node_sensors.c](../../tests/host/test_node_sensors.c) checks independent/complete failures and zero values |
| `run_ms` truncation, saturation at UINT16_MAX with ETIME_RANGE, sampling consumes radio deadline | `run_time_and_sampling_deadline_boundaries` in the same core host file; already implemented, not an undefined policy |
| Simultaneous fixed diagnostic slots, power failures, cleanup precedence, mandatory release and idempotence | `test_multiple_failures_remain_in_fixed_slots`, `test_power_off_failure_takes_precedence_and_preserves_sample`, `test_force_power_off_is_unconditional_then_idempotent` in the sensor host matrix; actual GPIO boundary tests in that file |
| ROM parsing, independent invalid identities and duplicate rejection | [test_node_sensors_identity.c](../../tests/host/test_node_sensors_identity.c), including `test_duplicate_identities_forbid_bus_access` |
| Actual Bosch compensation and private adapter allocation/transport/polling/budget/recovery failures | [test_bme280_driver.c](../../tests/host/test_bme280_driver.c) and [test_node_sensors_bme280.c](../../tests/host/test_node_sensors_bme280.c), compiled against the same immutable driver pin as the target |
| Late returned acquisition still reaches its untouched hold, then fails the unchanged duration assertion | [test_acquisition_hold.py](runner_tests/test_acquisition_hold.py), `test_every_return_reaches_hold_before_duration_assertion`; native execution of actual C functions, not target electrical evidence |
| Wrong fixture/diagnostic, missing/failed/ignored Unity cases, incomplete measurements, stale sources and invalid A/B binding | [runner_tests](runner_tests/), especially [test_fixture_checks.py](runner_tests/test_fixture_checks.py), [test_missing_ds.py](runner_tests/test_missing_ds.py), [test_reading.py](runner_tests/test_reading.py), [test_build_seal.py](runner_tests/test_build_seal.py) |
| Arbitrarily changing 1-Wire inventory or stalled SDK resources | No universal whole-call bound claimed; [TESTING.md](../../TESTING.md#node_sensors-automated-cases) defines finite-fixture completion and BME admission-budget scope |
| Rail-rise/shutdown waveforms, brief boot/reset/sleep glitches, inrush, bus edges, final microamp sleep budget | Not implemented; requires suitable instruments/custom board. Stable multimeter readings cannot establish these claims |
| Physical interruption of in-progress flash writes | Not implemented; software restart and injected corrupt records are separate evidence |
| SX1262 SPI/RF/peer exchanges, shared radio-clock timestamps and RF compliance | Outside this sensor suite; [radio test requirements](../../TESTING.md#sx1262_radio-hardware-strategy) remain separate |


## Fixture lessons

Remove the whole BME connector for the missing-device fixture. Removing VCC alone
can leave the device responding; preflight must establish absence before sampling.
Do not infer a back-power mechanism from that observation without measurement.

A returned acquisition must reach the untouched sample-return hold before checking
its duration. Otherwise a late return skips the physical shutdown observation.
The native hold-order regression protects this ordering.

A successful repeat run does not explain an earlier intermittent sensor error.
Keep error-specific diagnosis separate from the repeat-count assertion.

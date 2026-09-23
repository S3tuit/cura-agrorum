# Sensor coverage and evidence

This index maps the sensor obligations in [TESTING.md](../../TESTING.md) to
implemented cases and the original retained records. [ARCHITECTURE.md](../../ARCHITECTURE.md)
owns runtime behavior, [INTERFACE.md](../../INTERFACE.md#node_sensors) owns sensor
results, and the [protocol reading contract](../../../protocol/protocol-v2-lora/README.md#reading-payload)
owns the encoded fields. Wiring belongs to [SENSOR_CARRIER.md](../on_device/SENSOR_CARRIER.md);
invocations and evidence retention belong to [README.md](README.md#acceptance-commands).

The September 14 sensor/component and seven-step reading sequence is complete
for its recorded images, including nominal restoration. This is historical
acceptance with the applicability limits below. The September 17 acquisition
assertion-order correction and storage-erase warning have host/build verification
only: **hardware execution of the changed image is NOT RUN**. No evidence JSON was edited or
relabelled, and no new meter, RF, power-loss or current measurement is claimed.

## Obligation-to-case mapping

Names in the executable column are exact Unity case names in
[test_sensor_carrier.c](main/test_sensor_carrier.c). The local
[carrier_assert_sample / carrier_assert_inventory](main/carrier_checks.c) checks
and [forwarding observer](main/carrier_observer.c) inspect real returned values
and production driver calls. They do not supply a sensor oracle.

Commands below use the [session helper](README.md#discovery-and-physical-probe-labeling).
Each hardware invocation requires its own confirmed DUT/fixture readiness;
this table is an index, not a batch of commands for one wiring state.

| Required sensor observation | Implemented executable / procedure | Historical evidence in the catalogue |
|---|---|---|
| Complete configured inventory, absence detection and fresh acquisition boot | `carrier nominal preflight`, `carrier missing_ds0 preflight`, `carrier missing_ds1 preflight`, `carrier missing_bme280 preflight`; `fixture_preflight`; [runner](carrier_runner.py) checks identity/image and resets after released preflight resources | Preflight serial in each acquisition/reading record; both configured ROMs remain mandatory. Discovery is setup, not acceptance. |
| All five component groups, exact partial results, all seven encoded fields and flags | `carrier nominal acquisition and sample-return hold`; `carrier nominal core reading`; `fixture_reading` and [carrier_core_run](main/carrier_core.c) compare the sample actually consumed by core with opened frame plaintext and reopened pending storage | Nominal acquisition and all three nominal reading records |
| Atomic enclosure validity on success and failure | Nominal cases and `carrier missing_bme280 acquisition and sample-return hold` / `carrier missing_bme280 core reading` | Nominal and missing-BME component/reading records |
| At least 100 acquisitions, unchanged ROM identities, conversion freshness, duration and resource conditions | `carrier repeated nominal acquisition`; `--sensor-operation repeat --sensor-fixture nominal --sensor-repeat-count 100`; observer requires broadcast conversion, >=750 ms before addressed reads, 200 ms stabilization, 16 calibrated ADC reads per channel | Two repeat filenames contain the **same** 100-iteration run; count once |
| Actual BME low-power mode without modifying the returned sample | `carrier BME280 sleep observation`; `observe_bme_sleep` also runs after each repeated acquisition; reads real F3/F4 without a reset, mode write or new conversion | Both BME-sleep records and the one repeat run; register state does not establish microamp current |
| Independent logical DS0 absence and exact diagnostic slot | `carrier missing_ds0 acquisition and sample-return hold`; `carrier missing_ds0 core reading`; [missing-DS procedure](README.md#missing-ds-probes-and-nominal-restoration) | Missing-DS0 acquisition and reading |
| Independent logical DS1 absence and exact diagnostic slot | Symmetric `carrier missing_ds1 acquisition and sample-return hold` / `carrier missing_ds1 core reading` | Missing-DS1 acquisition and reading |
| Whole BME absence with independent gated groups retained | `carrier missing_bme280 preflight` requires `probe=261`; acquisition requires enclosure-only `(ESP_ERR,264)`; [whole-connector removal](README.md#missing-bme280-and-nominal-restoration) | Missing-BME acquisition and corrected reading; the responding-device setup failure is retained separately below |
| Soil ADC conversion and physical channel mapping | `carrier reference acquisition and sample-return hold` and `carrier adc_reference core reading`; [guided ADC A/B](README.md#adc-reference-positions-a-and-b), fresh measurements, <=75 mV error and >150 mV separation | Component ADC A/B and separate reading ADC A/B; A alone is incomplete |
| DS logical identity follows configured ROM across connector exchange | `carrier reference acquisition and sample-return hold`, selected by `--sensor-operation ds-identity`; [guided thermal/connector A/B](README.md#ds-identity-through-a-connector-exchange), same predeclared warmed ROM and >=2 C separation | September 11 DS identity A/B; bounded reuse justification below |
| Sampling-owned shutdown after success and every returned failure; stable post-sampling back-power observation | `fixture_acquisition` and the untouched `sample-return` hold; guided nominal and all three missing fixtures | Four guided acquisition records, each with independent TP_3V3/TP_GATE/TP_SW/TP_DQ readings; current late-return branch has native regression only |
| Idempotent final cleanup without bus initialization or rail enable | `carrier final cleanup hold`, two public force-off calls and `carrier_observer_cleanup_valid`; [final-cleanup command](README.md#acceptance-commands) | Final-cleanup record; separate from sampling-owned shutdown |
| Stable active-low gate ON/OFF states | `carrier production gate-on hold`, `carrier production gate-off hold`; [electrical procedure](../../TESTING.md#node_sensors-manual-electrical-cases) | Guided gate-on and gate-off records |
| Intentional restart-owned cleanup | `carrier production restart cleanup`, `reset_stage_1` / `reset_stage_2`, production `node_platform_esp_restart` and forwarding cleanup observation | Guided reset record; does not prove hardware default-off for arbitrary CPU resets |
| Held-reset and deep-sleep default-off; steady-state sleep back-power | `carrier enabled rail held reset`, `carrier enabled rail deep sleep`; [guided reset/sleep procedure](README.md#reset-held-reset-deep-sleep-and-back-power) | Held-reset and deep-sleep records, meter observations and sleep attestation; no timer-dwell or current claim |

The [reading sequence](README.md#production-core-reading-mapping) requires
nominal, missing DS0, missing DS1, missing BME, ADC A, ADC B and restored nominal.
The seven September 14 session records cover that sequence. The earlier nominal
record is retained separately and does not count as an additional required case.

| Reading fixture | Component validity | Protocol sensor flags (`flags & 0x00fe`) | Required outcome |
|---|---:|---:|---|
| nominal | `0x1f` | `0x00fe` | Seven exact same-acquisition values |
| missing_ds0 | `0x1b` | `0x00f6` | DS0 zero/invalid; unaffected values preserved |
| missing_ds1 | `0x17` | `0x00ee` | DS1 zero/invalid; unaffected values preserved |
| missing_bme280 | `0x0f` | `0x001e` | All three enclosure values zero/invalid together |
| adc_reference, A and B | `0x1f` | `0x00fe` | Fresh measured inputs map through the same core acquisition |

All eight retained reading outputs were independently revalidated with
[validate_reading](carrier_reading.py); the decoded result also matches each
record's retained `core_reading` event. Frame plaintext and actual pending
record body agree byte-for-byte. The local radio adapter emits no RF, and its
returning terminal observer does not enter real deep sleep.

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
| Rail-rise/shutdown waveforms, brief boot/reset/sleep glitches, inrush, bus edges, final microamp sleep budget | NOT RUN; requires suitable instruments/custom board. Stable multimeter readings cannot establish these claims |
| Physical interruption of in-progress flash writes | NOT RUN; software restart and injected corrupt records are separate evidence |
| SX1262 SPI/RF/peer exchanges, shared radio-clock timestamps and RF compliance | NOT RUN in this sensor workload; [radio test requirements](../../TESTING.md#sx1262_radio-hardware-strategy) remain separate |

## Evidence applicability

Every listed record is bound to DUT `cc8da2fc0224`, its original configuration,
ELF and source manifest. The configured logical ROMs are
`DF00000050F93828` (DS0 / physical DS2) and `7E000000540FA728` (DS1).
A source match establishes software applicability, not present wiring readiness.
Reuse of the recorded electrical result still requires the same relevant circuit,
including permanent R12 and the recorded pull-up arrangement.

| Original evidence group | Original ELF SHA256 | Applicable claim and limits |
|---|---|---|
| September 11 DS identity A/B | `77d5b88bc7e3e9816a89c5da80bd52deec0a1b502fa18b52f8e492f132eec234` | Exact A/B binding and >=2 C identity discrimination verified. The recorded backend hash matches `d06d96f^`; its DS acquisition function is byte-identical to the present function. ROM resolver, pad-release and pinned DS18B20/onewire source hashes also match. BME migration and app/build changes prevent whole-image equivalence; reuse only the DS identity claim |
| September 14 guided component session | `d39293932130d17049b103a3558f8b63797a7249b5eb168c8bdfc9bf5144459b` | Nominal/missing outcomes, gate/cleanup/reset/sleep DC observations and ADC A/B for that image. All recorded production component and observer/hold/assertion sources match the present files. Nine recorded app/build/runner paths differ after core integration and the hold-order correction; no blanket source-seal equivalence |
| Corrected-channel BME sleep and 100-repeat | `f759f33adf7ccfa6e113448a4620ef84183bd02956ea6feb2dee29177be20914` | Successful sleep observations and one complete 100-sample run. Same corrected Bosch/adapter content as the guided component session, with a separately recorded ELF; mode observations do not prove current consumption |
| September 14 core reading records | `d288e8e6bf5a9099eddbf38ebae7a19780e6a0b64e5c5e13df83b52e31df961f` | Original 168-source manifest matches the reviewed HEAD. Since reconciliation, its existing paths differ at `main/test_sensor_carrier.c` (hold assertion order) and `main/carrier_core.c` (pre-erase warning); the added native regression is also a new current seal input. Production core/sensors/persistence/crypto and the reading assertions remain unchanged; these are historical mapping results, not runs of the rebuilt image |

The September 14 groups have configuration SHA256
`9a551aa40f4360b5c52d404aaa64d7ad909809d1a1ed4c2380e2ba55eded110d`.
The DS identity pair has configuration SHA256
`58ed6b24f16746f9a1b03c8568929565874aaa83aaee99e44a86cfb987f14d7c`.
The differing configurations/images are retained explicitly. The current seal
must be checked with `verify_build` before any new invocation.

Both corrected-channel and guided component records identify Bosch commit
`5f4119ed6ee638abc573e75516ad9aa6e1cd612a`. Their source-selection modes and
ELFs differ; matching driver content does not make their binaries identical.
The historical DS pair predates that driver migration and supplies no Bosch
failure/sleep evidence.

The assertion-order correction changes only how the test reacts after a returned
acquisition: it captures duration, enters the required sample-return hold and
then rejects a duration outside 0..30000000 us. Existing successful observations
remain attributable to their original path. They do not exercise the corrected
late-return branch. Host deadlines, meter criteria and production timing are
unchanged; a host timeout remains a failure, not evidence of target recovery.

Storage erasure is deliberately destructive: `nvs_test` and `storage_test`
use the same physical ranges as production `nvs` and `storage`, and `reading`
erases/formats them before and after a case. The selected resolution is
[explicit documentation](README.md#destructive-test-storage) and a warning
immediately before the first erase. Test identities remain disposable and
never become production identities. After erasing production counters, follow
the existing identity replacement procedure to provision a new production node
ID and key before transmitting; do not restore the old identity or counter
backup. The records prove sensor mapping, not preservation of prior production
state. No additional admission mechanism or partition redesign is required.

## Evidence catalogue

The following files all match their locally retained raw `carrier-evidence.json`
bytes by SHA256. There are 26 files and 25 distinct run IDs. For each raw run,
`report.xml` and the nested pytest-embedded `dut.log` retain the original results.
Raw `build/` paths are local, ignored artifacts; the linked `evidences/` JSON
files are the portable archive.

Status abbreviations below preserve the exact original meanings:

- **G**: `accepted`, guided requirements complete for that operation.
- **S**: `software_passed_operator_acceptance_pending`; automated completion,
  with no implied electrical acceptance. Reading and BME/repeat cases assign no
  electrical hold of their own.
- **A**: `position_A_complete_sequence_incomplete`; never accepted alone.
  Its expected host failure remains in JUnit; both Unity cases passed.

All three B records contain `paired_acceptance.prior` matching their A record's
run ID, complete position event and SHA256. Fresh reference comparisons pass:
component ADC A errors 2/5 mV and B 5/4 mV; reading ADC A 3/4 mV and B 5/3 mV.
Reading reference separations are 461/460 mV. The DS pair preserves warmed
channel 1 with 17.38/21.44 C separation. Pair acceptance does not rewrite A.

<!-- Original evidence catalogue follows; keep hashes and records unchanged. -->

| Original file | Status | Run ID | File SHA256 | Raw run directory (local) |
|---|---|---|---|---|
| [acquire-missing_bme280-sensor-guided-evidence.json](evidences/acquire-missing_bme280-sensor-guided-evidence.json) | G | `92dd435fbbe64e388f325f3c315c0bca` | `97f72a2044662947a15622248331333e70adbd57977d19821210c073ee88bbee` | `build/session-20260914T140805+0200-XAR7GY/run-142948-aW9rm3` |
| [acquire-missing_ds0-sensor-guided-evidence.json](evidences/acquire-missing_ds0-sensor-guided-evidence.json) | G | `fd679c54cf2a447b8560a8cab6b9883f` | `b283115611ec5b2afa148d51d403450d1b0dfc80e7316ba438e1a673f6943e7d` | `build/session-20260914T140805+0200-XAR7GY/run-142506-YXElZ8` |
| [acquire-missing_ds1-sensor-guided-evidence.json](evidences/acquire-missing_ds1-sensor-guided-evidence.json) | G | `b992b19008014bec84e766ecd49565b6` | `5512fda3990eac887e684b038e8da9e1204e5b9a1dab21151ad2bca1cbf6c995` | `build/session-20260914T140805+0200-XAR7GY/run-142647-ISH38z` |
| [acquire-nominal-sensor-guided-evidence.json](evidences/acquire-nominal-sensor-guided-evidence.json) | G | `d55e126161674f6688bc48c4fc630a30` | `8de32f08ea894ad6d963aec247e7c2dcc032823f8e7b8e69757f981555ba1a1e` | `build/session-20260914T140805+0200-XAR7GY/run-144342-Wsv3QV` |
| [adc-reference-adc_reference-sensor-guided-A-evidence.json](evidences/adc-reference-adc_reference-sensor-guided-A-evidence.json) | A | `eb168342a9c84b7285ef30de464219bf` | `ae94de47818a08ee6c204e7659cf7963498326091416af82004e93f0971caed6` | `build/session-20260914T140805+0200-XAR7GY/run-143542-P70kd8` |
| [adc-reference-adc_reference-sensor-guided-B-evidence.json](evidences/adc-reference-adc_reference-sensor-guided-B-evidence.json) | G | `f328f2b5752d4a53a6281fbb7fe1ca68` | `f2452a22b663eff34040e55536c8ba12fc137b878f4de95f3ecb3f4926c64bfc` | `build/session-20260914T140805+0200-XAR7GY/run-144005-ep016S` |
| [bme-sleep-nominal-automatic-evidence.json](evidences/bme-sleep-nominal-automatic-evidence.json) | S | `b5a8516c9a574bc089eab76b0117f3dc` | `296ebb93673dae9737eecf04c93c3f8a190e18210266975fb14ffb5b308a003e` | `build/session-20260914T140805+0200-XAR7GY/run-144516-PNC3vC` |
| [bme-sleep-nominal-channel-validation-evidence.json](evidences/bme-sleep-nominal-channel-validation-evidence.json) | S | `a03d8f61c2554bea9e56e1c6a2df8f76` | `7686fbf58caad7e07fb6c7778ae293a02262d05c3a580ee86aa10d7ebd6faf72` | `build/session-bme-channel-20260914T122052-qmp1u2si/bme-sleep` |
| [deep-sleep-nominal-sensor-guided-evidence.json](evidences/deep-sleep-nominal-sensor-guided-evidence.json) | G | `79d60958d7d54392806ee507ef5f3bf7` | `b7bca41b21894967bc294b72131d075714882858486740de90d85bcd131fed66` | `build/session-20260914T140805+0200-XAR7GY/run-142111-sXzFGp` |
| [ds-identity-nominal-sensor-guided-A-evidence.json](evidences/ds-identity-nominal-sensor-guided-A-evidence.json) | A | `8731aa580db14a0896298946913c9f8a` | `bf9271b4bef95abe8d81e5e284c77fbeb7d80660818ff722ed0fded6089176bf` | `build/session-20260911T113529+0200-ETaWUK/run-120728-ZwlzoZ` |
| [ds-identity-nominal-sensor-guided-B-evidence.json](evidences/ds-identity-nominal-sensor-guided-B-evidence.json) | G | `f7e10d9e501d4b889af0119f6de6a15d` | `be0154129cd0434ef93f9bc2f5392e689bfc1e55e11ddd0550c85ae3b72394cc` | `build/session-20260911T113529+0200-ETaWUK/run-121054-BrP192` |
| [final-cleanup-nominal-sensor-guided-evidence.json](evidences/final-cleanup-nominal-sensor-guided-evidence.json) | G | `92e6712cc266413fa1fc9f4bf0e737f3` | `20a183defb60b7e529d096acde3285877cf8d32e496106a5163a7cfd186a685d` | `build/session-20260914T140805+0200-XAR7GY/run-141021-WfnY9t` |
| [gate-off-nominal-sensor-guided-evidence.json](evidences/gate-off-nominal-sensor-guided-evidence.json) | G | `a509600396474bc8aa4b61b2f230399a` | `5dac51175fcc374ff36053ce35c7214e910f9bf28192a02a7bfe3484eee7b96b` | `build/session-20260914T140805+0200-XAR7GY/run-141409-oTWTxH` |
| [gate-on-nominal-sensor-guided-evidence.json](evidences/gate-on-nominal-sensor-guided-evidence.json) | G | `701dc4ab99454a6c9b6171ba1c207107` | `056a3093a26e343e87d5bb586052917d3a224f8cadd33588fcdf61da68ee107c` | `build/session-20260914T140805+0200-XAR7GY/run-141204-8XbSFC` |
| [held-reset-nominal-sensor-guided-evidence.json](evidences/held-reset-nominal-sensor-guided-evidence.json) | G | `79cc221275664d99aeb98f6cc88fcdc1` | `d4338c45bbe241ac1a82574cd64a44887258a626e3aa946ac8320bfd95d1fe6a` | `build/session-20260914T140805+0200-XAR7GY/run-141750-NizCis` |
| [reading-adc_reference-sensor-guided-A-evidence.json](evidences/reading-adc_reference-sensor-guided-A-evidence.json) | A | `83fc8fdf220d4dd1bd0bfcefe174458c` | `75f8836b6e9d5bfda1f6a5d0ae3697996ba7a29038203c64c740d58d34762c36` | `build/session-20260914T164314+0200-dIWrUk/run-165928-QYpXps` |
| [reading-adc_reference-sensor-guided-B-evidence.json](evidences/reading-adc_reference-sensor-guided-B-evidence.json) | G | `e3392135ac3441f5ae5e40af4d05b222` | `0e04cfe24a11f9e627e96e15a2ae73e174d0713e891533d58304b94fc48b4c07` | `build/session-20260914T164314+0200-dIWrUk/run-170221-JZeSaS` |
| [reading-missing_bme280-automatic-evidence.json](evidences/reading-missing_bme280-automatic-evidence.json) | S | `61ce5df99512443a941fad879449021e` | `370c57947120cc31db49699556234b68364b55ef3a16adfe17a414d50846de17` | `build/session-20260914T164314+0200-dIWrUk/run-165538-GlXSRT` |
| [reading-missing_ds0-automatic-evidence.json](evidences/reading-missing_ds0-automatic-evidence.json) | S | `43a7c6f93b60429f8729b411a6c80788` | `d1fc5ee7aee29a40a56db04a7cd0d32b319a93834ed1aeaaabfda3e6294035db` | `build/session-20260914T164314+0200-dIWrUk/run-164939-xt7soe` |
| [reading-missing_ds1-automatic-evidence.json](evidences/reading-missing_ds1-automatic-evidence.json) | S | `8c2e89feaddc4545af38f92bf6a7b59a` | `73b934387891dafa2e20f22873b7fa42c320e16e59a154e9b83229918cb1ac2c` | `build/session-20260914T164314+0200-dIWrUk/run-165038-27buX0` |
| [reading-nominal-automatic-evidence.json](evidences/reading-nominal-automatic-evidence.json) | S | `1ccb8eda27054813a60cd69bf866b6da` | `9bcf0b5fee2c225948c3dcc88aba28cdc238e01116d040a4dbabe9c7a8f2b541` | `build/reading-nominal-20260914T160421-hixf8lk2` |
| [reading-nominal-automatic-restoration-evidence.json](evidences/reading-nominal-automatic-restoration-evidence.json) | S | `996983d25fee4eb69168adca46763527` | `f4a08d5709f961bf6493432b4714c4d837dd32c39a03a0664e5417d4c71b9fda` | `build/session-20260914T164314+0200-dIWrUk/run-170435-4aEexU` |
| [reading-nominal-automatic-session-start-evidence.json](evidences/reading-nominal-automatic-session-start-evidence.json) | S | `3794a61a56754f088125a96c0ef337ba` | `95160336513cae595a8b56c8c10638dca30e785b8990ac8e1f85a069e3b614e2` | `build/session-20260914T164314+0200-dIWrUk/run-164609-n7lGn4` |
| [repeat-nominal-automatic-evidence.json](evidences/repeat-nominal-automatic-evidence.json) | S | `da09e5ce7bcc4a05ae5c861a72338d81` | `935a13a40ca836047086eede0b8df17f0b067e81848c2cc29db87a4d28590053` | `build/session-bme-channel-20260914T122052-qmp1u2si/repeat` |
| [repeat-nominal-channel-validation-evidence.json](evidences/repeat-nominal-channel-validation-evidence.json) | S | `da09e5ce7bcc4a05ae5c861a72338d81` | `935a13a40ca836047086eede0b8df17f0b067e81848c2cc29db87a4d28590053` | `build/session-bme-channel-20260914T122052-qmp1u2si/repeat` |
| [reset-nominal-sensor-guided-evidence.json](evidences/reset-nominal-sensor-guided-evidence.json) | G | `d485b9575b424337904a0899ddee7c3a` | `5c90b57d87ffb1db76acb34bbbf16a727e5c4fb84dace73bf42096c12f174b26` | `build/session-20260914T140805+0200-XAR7GY/run-141539-kUrG5m` |

## Retained failures and completion limits

These attempts remain separate from accepted fixture results. Paths below are
relative to this app and identify original local artifacts, not portable passes.

| Attempt / original artifact | Observed result | Effect on acceptance |
|---|---|---|
| `build/session-20260914T164314+0200-dIWrUk/run-164933-OgrbOd/report.xml` | UART port unavailable; UsageError before any Unity case | Failed setup; no DUT result |
| `build/session-20260914T164314+0200-dIWrUk/run-165924-usScHG/report.xml` | Same unavailable-UART setup failure | Failed setup; no DUT result |
| `build/session-20260914T164314+0200-dIWrUk/run-165200-mvbiFi/` | Missing-BME preflight observed `probe=0`; selected preflight failed, acquisition never ran | Excluded. Operator reported removing only VCC. Corrected whole-connector run `run-165538-GlXSRT` observed `probe=261`; that is the retained reading result. No measured back-power mechanism is inferred |
| `build/session-bme-20260912T212730-p5robqc7/repeat/` | Original sample-five READ/ESP_ERR_INVALID_RESPONSE failure after four completed iterations | Retained unresolved causal attribution. Later corrected-channel 100-repeat success does not retrospectively explain this failure |

The initial unavailable-port attempt in the September 14 component session also
remains in its original session directory. The original A-position host failures
in all three pairs are prescribed incompletion, not independent successful
runs; B's exact binding closes each pair. The repeat archive's two filenames
are byte-identical copies of one run, not two repetitions.

Current-source target execution, the late-return electrical path, instrument
claims and later radio work remain NOT RUN as stated above. Any later production
or circuit change requires reassessing the relevant original evidence and
repeating affected observations; a new source seal is not a new hardware run.

## Validation provenance

The reconciliation baseline was HEAD
`4779361ff04180d34f846ea8942c9387092cc8cc`, independently checked against the
review's `4779361` reference before edits; index and tracked working tree
matched that commit. The review provided no separate dirty-tree snapshot or
workplan hashes. The current result consists of that baseline plus the app-local
hold-order correction, its native regression, the pre-erase warning and the
documentation of destructive test storage.

On 2026-09-17:

- `CCACHE_DISABLE=1 make test-host`: **147/147 PASS** after the hold correction.
  The later app-local warning does not change those production host-test inputs;
  this suite was not rerun solely for the warning.
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest firmware/test_apps/sensor_carrier/runner_tests -q`:
  **680 PASS**, three existing rich-click deprecation warnings. The new native
  hold regression has 40 cases: before the fix 16 failed specifically because
  late/negative-duration returns skipped the hold; afterward all 40 pass.
- After `source ~/esp/esp-idf/export.sh`,
  `CCACHE_DISABLE=1 idf.py -C firmware/test_apps/sensor_carrier build`: **PASS**.
  Actual compile commands and ELF verify real core/sensors/backend/persistence/
  crypto and the app observer, with no RF backend or bare-C6 fake sensor support.
  After the warning was added, the rebuilt binary is 436048 bytes in the
  unchanged 1048576-byte app partition. ELF inspection verifies the warning
  string and `puts` -> `fflush` -> first `hwtest_erase_state` ordering.
- `carrier_evidence.py --record-build` and `verify_build`: **PASS** for the
  rebuilt configuration and 169-source seal. This is build provenance, not DUT
  acceptance.
- All 26 archive/original hashes, all eight captured reading outputs, all three
  exact A/B bindings and recorded electrical/reference/identity values were
  rechecked. Raw JUnit records preserve expected A incompletion and rejected
  setup attempts. Historical source applicability was checked as described
  above; evidence bytes remain unchanged.

Rebuilt ELF SHA256:
`298bc2ca27dd5d13881fc6faf135c45a7ea27bd43b0c5d8864561e8fb2445124`.
Configuration SHA256:
`9a551aa40f4360b5c52d404aaa64d7ad909809d1a1ed4c2380e2ba55eded110d`.
The source-map SHA256, calculated over Python
`json.dumps(manifest['sources'], sort_keys=True).encode()`, is
`59e41ee2ae940bded88ca123d44dd21d126cffc97668a5d49d2d5f0832601f8f`.
The Bosch selection is `explicit_clean_checkout` of the content-verified
`5f4119ed6ee638abc573e75516ad9aa6e1cd612a` pin. Local reconciliation logs and
initial review/workplan hashes are retained in `/tmp/cura-all4-reconciliation/`.
The `f002-*` files there record the warning follow-up, including its repeated
680-case runner pass, build, seal and compiled call-order inspection.

The original stage-10 validation retains **147 host / 640 runner / 258 protocol
passes** in `build/stage10-validation-fpwpjlck/`, with its original
`carrier-build.json` and `d288e8e6...` ELF. The protocol result is historical;
protocol and bare-C6 DUT suites were **NOT RUN** during this app-local correction
because no production component, protocol behavior or shared bare-C6 harness
changed. No DUT was opened, reset or flashed during reconciliation.

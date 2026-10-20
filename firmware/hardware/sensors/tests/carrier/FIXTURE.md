# Sensor test fixture and measurements

Start from [firmware/HARDWARE.md](../../../../HARDWARE.md), the owner of nominal
wiring, component values and pin assignments. This guide owns test-only
connections and physical observations; [README.md](README.md) owns commands.
Use the XIAO's native USB-C connection and confirm the DUT and actual fixture before
each operation. Remove power before touching connectors, shunts or reference leads.

## Test-only reference inputs

Add inline removable links to both soil outputs and injection points before the
existing ADC series resistors. Keep R3/C4 and R4/C5 close to the XIAO pads.

```text
J_SOIL0.2 ---- JP_SOIL0 ----+---- R3 ---- GPIO1 / TP_ADC0
J_INJECT0 -----------------+
J_SOIL1.2 ---- JP_SOIL1 ----+---- R4 ---- GPIO2 / TP_ADC1
J_INJECT1 -----------------+

+3V3_DUT ---- JP_REF_ENABLE ----+---- R5 10 k ----+---- J_VREF_A (~1.185 V)
                                |                +---- R6 5.6 k ---- GND
                                +---- R7 2.2 k --+---- J_VREF_B (~1.650 V)
                                                 +---- R8 2.2 k ---- GND
```

Use 1% divider resistors and the common DUT ground. No extra divider capacitors
are needed: C4/C5 filter after the series resistors. Nominal divider values are
setup targets; the test requires fresh meter readings. Leave JP_REF_ENABLE open
and remove injection leads outside `adc_reference`. Never connect a reference
against an attached soil output. R12 and C3 remain fitted in every state.

## Fixture states

| State | Connections changed from nominal |
|---|---|
| `nominal` | Both soil probes in air, JP_SOIL0/1 fitted; both configured DS ROMs and BME connected; no reference leads |
| `missing_ds0` | Remove the complete power/data/ground connector belonging to configured logical ROM0; keep ROM1 connected |
| `missing_ds1` | Remove the complete connector belonging to logical ROM1; keep ROM0 connected |
| `missing_bme280` | Remove the complete four-wire BME connector; keep soil and both DS probes connected and preserve the configured carrier I2C pull-ups |
| `adc_reference` | Remove both soil connectors and JP_SOIL0/1; fit JP_REF_ENABLE; keep both DS probes and BME connected |

For reference position A, connect J_VREF_A to J_INJECT0 and J_VREF_B to
J_INJECT1. For B, exchange those two leads with power removed. Confirm the
complete state before each acquisition, and restore nominal unpowered afterward.
Select missing DS connectors by configured ROM, never by a deployment's labels
or physical order. Discovery and acquisition inventory checks are in the runner.

Removing only BME VCC can leave the device responding. The missing-BME fixture
requires whole-connector removal and a NACK-specific preflight; a timeout does
not establish absence. That observation alone does not identify a back-power
mechanism. Disconnecting an analog probe can still produce a valid ADC reading;
it is not a reliable acquisition-failure fixture.

## DC measurement procedure

Use a multimeter in DC-voltage mode, referenced to adjacent carrier ground.
Wait for the named hold-ready marker, then at least five seconds for an on-state
reading or ten seconds for an off-state reading. Require three consecutive stable
display updates at each point. A meter hold function does not capture transients.
Record independent readings in volts at the runner prompts.

| Observation | Required points and limits |
|---|---|
| Gate on / transition on | TP_3V3, TP_GATE, TP_SW, TP_DQ; gate <=0.2 V, switched rail within 0.1 V of TP_3V3, DQ within 0.1 V of TP_SW |
| Gate on with soil probes | Also TP_ADC0/1, each 2.0..2.7 V for the air-exposed fixture |
| Sample return, gate off, final cleanup, reset or sleep | TP_3V3, TP_GATE, TP_SW, TP_DQ; gate >=3.0 V and within 0.1 V of TP_3V3; switched rail and DQ each <=0.1 V |

The sample-return hold performs no further sensor, gate or cleanup operation.
It must begin even after a failed or late acquisition, before the duration
assertion reports failure. This isolates sampling-owned shutdown from final
cleanup. The separate final-cleanup operation exercises repeated force-off.
Do not fix a failing sample-return observation by adding a cleanup call.

Automatic holds last 60 seconds. Guided awake observations allow 180 seconds
and finish after valid readings and DUT acknowledgement; the host allows 195
seconds including transport/result margin. DS/ADC post-sample holds retain the
automatic 60 seconds. Setup confirmations are separate from observation windows.

For intentional restart, observe the production restart helper's force-off and
the untouched off-state after software reset. For held reset, keep EN asserted
while measuring, then release only at the prompt. For deep sleep, the test
enables the rail before terminal sleep without a timer wake: enter off readings
within 600 seconds and attest YES while the MCU is still asleep. Only then press
EN when prompted. Expected native USB disappearance is handled by the
identity-bound transport. A substituted device, stopped reader, early boot,
missing reading or missing attestation cannot pass. Reset/boot has a 30-second active deadline within the
615-second overall observation/boot ceiling. This checks default-off and steady
voltage during sleep; the bare-S3 suite separately checks timer wakeup.

If TP_SW remains above 0.1 V after settling, keep the failure and investigate
separately. Keep permanent R12 fitted without adding a parallel load or removing
it to change acceptance. A low voltage across R12 can conceal weak back-power;
measuring the gate alone does not establish the switched-rail voltage.

## ADC and DS identity comparisons

Immediately before each reference acquisition, measure TP_3V3 and both ADC
points. Each reference must be positive, below 2.0 V and below the measured
supply. Each reported ADC value must agree within 75 mV, inclusive. The two
references must differ by more than 150 mV so their +/-75 mV intervals cannot
overlap; ADC0 is lower in A and higher in B. The allowance accommodates meter,
wiring and conversion variation without accepting a channel swap. Fresh readings
are required for both positions, together with the matching first-position JSON.

For DS identity, predeclare the physical ROM to warm and keep both configured
probes connected. Its logical result must be at least 2 C warmer in both A and
B. Exchange complete physical connector positions with power removed. Logical
identity must follow ROM in both arrangements. This establishes identity mapping,
not sensor accuracy or a specified thermal response.

## Interpretation and limits

The repeated-acquisition check requires the complete requested count. A later
successful repeat does not explain an earlier intermittent error: retain the
failed run while diagnosing it. Temporary logs and measurements can be deleted
after diagnosis and follow-up; useful lessons belong in code or this procedure.

The software observes the selected finite fixture and checks actual acquisition
return within 30 seconds. A host timeout does not establish target recovery or
cleanup. BME admission budgets cannot preempt an in-flight SDK operation, and
1-Wire enumeration has no universal bound for arbitrary changing inventories.

Stable multimeter readings cannot establish the 200 ms rise waveform, exact
shutdown time, brief boot/reset/sleep glitches, switching edges, inrush, bus
waveforms or microamp sleep budget. Those need appropriate instruments. A
successful sensor test also does not establish radio or production cadence behavior.

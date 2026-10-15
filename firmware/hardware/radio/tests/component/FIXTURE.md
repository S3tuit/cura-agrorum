# Radio test fixture and preflight

[firmware/HARDWARE.md](../../../../HARDWARE.md#radio-circuit) owns nominal wiring
and connector numbering. This guide owns C6 fixture changes and static checks;
[README.md](README.md) describes the component image and
[tests/rf](../../../../../tests/rf/README.md) owns joint C6/Pi execution.
The Pi has its own [hardware guide](../../../../../receiver/HARDWARE.md).

## Removable test connections

Add JP_DIO1_N between the MCU-side DIO1/RN5 junction and J_RAD_N.9. Keep RN5
on the MCU side so opening the link leaves GPIO20 defined. Add removable
absence-only ties from MCU-side BUSY (GPIO19) to +3V3_DUT and MCU-side MISO
(GPIO14) to GND. All three stay on the carrier when the module is removed.

```text
GPIO19 / TP_N_BUSY ----+---- J_RAD_N.8 / module BUSY
                      +---- JP_ABS_BUSY_N ---- +3V3_DUT
GPIO14 / TP_N_MISO ----+---- J_RAD_N.5 / module MISO
                      +---- JP_ABS_MISO_N ---- GND
GPIO20 / TP_N_DIO1 ----+---- JP_DIO1_N ---- J_RAD_N.9 / module DIO1
                      +---- RN5 ---- GND
```

The absence jumpers are **direct ties**, not pull resistors. Fit them only after
removing the whole module, including power, ground and all seven signals.
Remove both ties before reinstalling the module. Never ground a connected
module's DIO1 output or force its connected BUSY/MISO outputs to a rail.

| State | Module / J_RAD_N | JP_DIO1_N | JP_ABS_BUSY_N | JP_ABS_MISO_N |
|---|---|---|---|---|
| `nominal` | All nine contacts connected | Fitted | Open | Open |
| `radio_absent` | Whole module removed | Fitted | BUSY to 3.3 V | MISO to GND |
| `dio1_disconnected` | All other contacts connected | Open | Open | Open |

J_HOST_N and RN5 remain connected in executable states. Keep any attached
sensor circuit nominal and record its state separately; component cases do not
sample sensors. No temporary CS preflight lead remains during execution.

For every transition, stop the test, disconnect UART USB and any other
power/back-power path, and verify the carrier rail is below 0.1 V. EN/reset,
stopping software and the module battery switch do not remove power. Make
antenna, jumper and connector changes only unpowered.

## Static preflight

1. Identify the C6, module, antenna and selected state. Check the HF module and
   `ANT_SW A (3V3)` population. Leave battery/Pico/VBUS/VSYS connections absent
   and the battery switch off. Attach the antenna unpowered.
2. Trace each C6 GPIO to its module contact using the nominal wiring table.
   Verify common ground, capacitor polarity, RN5 value/placement and no supply
   or signal shorts. Check DIO1 continuity with its link fitted and isolation
   with it open, accounting for semiconductor paths. In `radio_absent`, verify
   all nine module connections are absent before fitting the two rail ties.
   Perform resistance/continuity measurements only unpowered.
3. Isolate all seven C6 signal drivers by disconnecting J_HOST_N, keeping
   carrier supply/ground intact. With a module fitted, add a temporary lead
   from TP_N_CS to TP_N_3V3 to hold CS inactive. Hold C6 EN before applying UART
   USB power and keep it held until power is removed again. Do not flash or
   access the C6 during this power-only check. With the module absent, omit
   the CS lead and fit its two absence jumpers.
4. Use a DC voltmeter with black lead at TP_N_GND. Check the C6 supply,
   TP_N_3V3 and fitted module U1.36. Require 3.2..3.4 V at each supply point;
   this is the fixture admission band, not a module operating-range claim.
   Wait five seconds and require three stable display updates. Never connect
   the current-range leads across the supply.
5. Check the levels below. Low is <=0.2 times measured local 3V3; high is
   >=0.8 times that supply without exceeding it beyond meter resolution.
   An intermediate/unstable level or missing measurement leaves preflight
   unresolved. Remove power before changing state and repeat its supply check.
6. Restore nominal unpowered: remove both absence ties before reinstalling
   the module, fit DIO1, and remove the temporary CS-high lead before
   reconnecting J_HOST_N. Verify no C6 output remains tied to a rail. Leave
   the node unpowered until image/configuration and DUT readiness are confirmed.
   Keep the observations with the current test session while investigating.

| Power-only state, J_HOST_N isolated | CS | RESET | BUSY | MISO | MCU-side DIO1 |
|---|---|---|---|---|---|
| Nominal, temporary CS-high lead | High | High, internal pull-up | Low after settling | High impedance; no voltage criterion | Low |
| Radio absent, rail ties fitted | No criterion | No criterion | High | Low | Low |
| DIO1 disconnected, temporary CS-high lead | High | High, internal pull-up | Low after settling | High impedance; no voltage criterion | Low; also check open-link continuity |

A low MCU-side DIO1 alone does not prove disconnection. BUSY low does not prove
SPI or oscillator initialization. MISO is high impedance with CS inactive, as
described in [Semtech section 8.2](https://files.waveshare.com/wiki/SX1262-XXXM-LoRaWAN-GNSS-HAT/DS_SX1261-2_V1.2.pdf).
Watch for supply/reset errors under TX load during functional tests; static
checks do not establish dynamic margin.

If assembly changes sensor wiring or the shared supply, rerun the necessary
[sensor checks](../../../sensors/tests/carrier/README.md). To keep the radio
quiet during supply checks, disconnect J_HOST_N and fit the temporary CS-high
lead unpowered; remove it unpowered before restoring J_HOST_N.

## RF and fault observations

Use repeatable antenna positions at least 3 m apart, with fixed heights, common
vertical orientation, a clear path and no immediate metal obstruction. Keep
positions unchanged through a fault/restoration comparison. RSSI/SNR are not
calibrated power measurements. Follow the [operating envelope](../../../../../tests/rf/OPERATING_ENVELOPE.md)
and use [spec.py](../../../../../tests/rf/spec.py) for case names and airtime
charges, including uncertain transmissions. Confirm separate Pi radio ownership
and readiness; SSH timing and endpoint monotonic clocks are not interchangeable.

For `dio1_disconnected`, the peer must receive the real payload while the node
returns within its deadline with `tx_started=true`, `tx_done=false`, and
TRANSMIT/WAIT_IRQ/HARDWARE_TOUCHED diagnostic context. This consumes airtime.
For `radio_absent`, the BUSY tie causes bounded lazy-initialization failure
without starting TX; an I/O error needs its captured cause. Cleanup must permit
MCU deep sleep. Restore nominal unpowered and repeat the nominal exchange.

The actual [radio app](main/radio_app.c) and joint verifier own executable
packet/timing assertions. The host suite covers exact deadline boundaries and
backend errors. Component exchanges do not measure RF output power, emissions,
carrier accuracy, DIO1 electrical-edge timestamp error or microamp sleep current.
Forced-BUSY on a connected module, broken SPI and additional injected faults
require their own controlled fixtures; these connections do not define them.

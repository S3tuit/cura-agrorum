# Wio B2B radio fixture and preflight

[firmware/HARDWARE.md](../../../../HARDWARE.md#radio-circuit) owns the XIAO/Wio
B2B wiring and electrical profile. This guide owns fault access and physical
checks; [joint RF procedures](../../../../../tests/rf/README.md) own execution.
The Pi uses its separate [hardware guide](../../../../../receiver/HARDWARE.md).

## Fault access

The normal assembly mates the Wio directly to the XIAO B2B connector. Its radio
signals cannot be disconnected individually with XIAO header jumpers. The Wio
outer headers expose radio nets; they are not XIAO pad pass-throughs.

DIO1 isolation requires a correctly mapped B2B interposer. Pass every connector
contact through unchanged except for a removable link in the GPIO39/DIO1 path.
Fit a 10 kohm pull-down from **MCU-side** DIO1 to ground so the open link cannot
create floating IRQs. Expose MCU-side BUSY/GPIO40 and MISO/GPIO8 for the absence
fixture, with labelled removable direct ties to 3V3 and ground respectively.
Check the connector orientation and every contact against the manufacturer
schematic before mating either board. Do not cut the Wio or XIAO traces as a
substitute for a reviewed interposer.

```text
XIAO GPIO39 ----+---- JP_DIO1 ---- Wio DIO1
               +---- 10 kohm ---- GND
XIAO GPIO40 ----+---------------- Wio BUSY
               +---- JP_ABS_BUSY ---- +3V3_DUT
XIAO GPIO8  ----+---------------- Wio MISO
               +---- JP_ABS_MISO ---- GND
```

| `node_fixture` | `node_fault_access` | Required state |
|---|---|---|
| `nominal` | `b2b_direct` | Wio directly mated; no fault ties |
| `dio1_disconnected` | `b2b_interposer_dio1_open` | Wio fitted through interposer; only DIO1 link open; both absence ties open |
| `radio_absent` | `module_removed` | Wio completely removed; MCU breakout/interposer remains; BUSY tied to 3V3, MISO to GND, MCU DIO1 pull-down retained |

The absence ties are **direct rail ties**, fitted only after all Wio contacts
are disconnected. Remove both before reinstalling the module. Never force a
connected module's BUSY, MISO or DIO1 output to a rail. Removing only module
power does not establish absence and can back-power it through signals.

For every fixture change, stop the run, disconnect native USB and every other
power/back-power path, then verify 3V3 below 0.1 V. EN/reset does not remove
power. Change antennas and connectors only unpowered. Record the actual
interposer construction with the run; without one, DIO1 fault testing is
unavailable. Do not select that case on the direct stack.

## Preflight and electrical observations

1. Identify the XIAO, B2B Wio module and 868 MHz antenna. Confirm the module's
   IPEX socket is used, the B2B connector is aligned, and no through-header
   shorts XIAO pads to the Wio outer headers. Leave battery and 5V pads unused.
2. Unpowered, check common ground, supply polarity and shorts. With an
   interposer, verify every pass-through and the selected link/tie state;
   DIO1 isolation must be checked by continuity, not inferred from a low level.
   In absence state, prove complete module disconnection before fitting ties.
3. Keep MCU EN/reset held before applying USB for a supply-only check. Do not
   flash or release reset during this check. Measure XIAO 3V3 and module VCC
   with a voltmeter referenced to local ground. Require 3.2..3.4 V, including
   at the module: the 3.0 V TCXO needs its documented supply headroom. Wait
   five seconds and require three stable display updates. Remove power before
   changing state. Do not connect current-range leads across a supply.
4. Restore the selected executable fixture unpowered. Remove temporary test
   leads and confirm no connected output is tied to a rail. Keep sensors
   nominal and describe them separately; component episodes do not sample them.
   Admit firmware/configuration, DUT identity and airtime before releasing it.
5. During approved functional runs, observe VCC under TX load, RF enable and
   MCU-side DIO1 using suitable probes. RF enable must be high for TX and RX,
   then low through cold sleep and MCU deep sleep. Verify a subsequent wake
   can release the GPIO hold and initialize again. Check gate/DQ default-off
   behavior with the [sensor procedure](../../../sensors/tests/carrier/FIXTURE.md).

A static supply reading does not establish transient TCXO headroom, DC-DC
stability or sleep current. Inspect the supply waveform during start, TX, RX
and sleep; measure current with a method that preserves the supply voltage.
Qualify the directly mated assembly separately from any interposer fixture.
GPIO38 low alone does not prove the SX1262 entered cold sleep.

## Functional faults and qualification limits

Use fixed antenna positions at least 3 m apart with common vertical orientation,
fixed heights and a clear path. Keep them unchanged through fault/restoration
comparisons. Follow the [operating envelope](../../../../../tests/rf/OPERATING_ENVELOPE.md)
and whole-case charges in [spec.py](../../../../../tests/rf/spec.py). Independent
endpoint clocks and SSH arrival times do not establish radio timing.

First pass nominal ACK exchange. For disconnected DIO1, the peer must receive
the payload while the node returns within its deadline with `tx_started=true`,
`tx_done=false` and TRANSMIT/WAIT_IRQ/HARDWARE_TOUCHED context. This consumes
airtime. For absent radio, the MCU-side BUSY tie causes bounded initialization
failure before TX. RF-disable cleanup must still be attempted after failure;
observe GPIO38 inactive. Restore the direct nominal assembly unpowered and
repeat ACK exchange before another fault campaign.

The [component app](main/radio_app.c) and verifier own packet/timing assertions.
Before installation, qualify actual USB reconnection and retained RTC across
reset/timer wake, sensors and ADC references, nominal TX/RX and sleep/wake,
fault cleanup, DC-DC supply behavior, conducted/radiated output and sleep current.
Use the full production sleep interval for cadence/retention checks; accelerated
RF cases do not establish it. Retain setup, instrument limits and uncertainty
with physical results in the sibling logbook. Host tests, register settings,
RSSI/SNR and earlier-board captures do not establish these physical quantities.

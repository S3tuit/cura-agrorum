# RF operating envelope

Controlled tests use the documented [XIAO/Wio B2B assembly](../../firmware/HARDWARE.md)
and [Pi carrier](../../receiver/HARDWARE.md), including their declared
component fault fixtures. The configured profile is 868.1 MHz, intended +14 dBm,
SF7/BW125/CR4/5, explicit header, CRC, eight-symbol preamble, private sync word
and direction-specific IQ. S3 fault connections and static checks belong to the
[radio fixture](../../firmware/hardware/radio/tests/component/FIXTURE.md).

The configuration basis uses a nominal 2 dBi antenna and credits no unmeasured
RF-path loss. The estimate `ERP_dBm = P_port + G_dBi - L_dB - 2.15` gives
13.85 dBm (24.27 mW) at configured +14 dBm, only about 0.13 dB below the
25 mW basis. Actual output tolerance, antenna gain and path loss remain unmeasured.
The +22 dBm SetTxParams register and the Seeed Wio PA row selected by the
[firmware profile](../../firmware/components/sx1262_radio/sx1262_radio.c) express
the expected +14 dBm setting; it is not a measurement of output power.
The retained basis is [EU Decision 2025/105, annex row 48](https://www.boe.es/buscar/doc.php?id=DOUE-L-2025-80100);
the [ITU SRD report](https://www.itu.int/dms_pub/itu-r/opb/rep/R-REP-SM.2153-8-2021-PDF-E.pdf)
describes the ERP/EIRP reference conversion. This estimate does not bound RF
uncertainty, and reduced duty cycle cannot compensate for excessive power.

This operating envelope retains that configuration basis without measured
output-power, carrier or emissions qualification; combined RF uncertainty is
not bounded by a measurement. Component exchanges verify functional behavior,
not output power, frequency accuracy or spectral characteristics.

Declared component cases and production autonomous wakes, retries and resets
are allowed within the existing firmware wake/sleep policy. The operator owns
per-transmitter airtime accounting, admission and pacing across tests. Receiver
production airtime enforcement remains applicable.

Use [spec.py](spec.py) for whole-case charges. Include retries and uncertain
transmissions and preserve activity history across resets, reflashing and session
restarts. Unknown prior activity cannot be treated as a fresh allowance. The
configuration basis uses at most 36 seconds per continuous hour per transmitter;
an hour boundary does not reset that rolling interval. The node's per-wake budget
does not independently enforce it across unexpected resets.

The XIAO/Wio module and RF path require fresh output, supply and sleep
qualification against this basis; earlier-board captures do not establish them.
Reassess the envelope after module, antenna, RF-path, power or PHY changes, when
suitable instruments become available, or observations contradict the basis. Physical measurement requires declared quantities, limits,
calibrated instruments and uncertainty. Fixture checks and bounded cleanup are
required for every controlled run.

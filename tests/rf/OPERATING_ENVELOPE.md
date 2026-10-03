# RF operating envelope

Controlled tests use the documented [C6 carrier](../../firmware/test_apps/on_device/SENSOR_CARRIER.md)
and [Pi carrier](../../receiver/hardware/TEST_CARRIER.md), including their declared
component fault fixtures. The configured profile is 868.1 MHz, intended +14 dBm,
SF7/BW125/CR4/5, explicit header, CRC, eight-symbol preamble, private sync word
and direction-specific IQ. The [firmware assessment](../../firmware/TESTING.md#sx1262_radio-first-rf-operating-envelope)
describes the antenna/path estimate and configuration basis.

This operating envelope accepts that configuration basis without measured
output-power, carrier or emissions qualification; combined RF uncertainty is
not bounded by a measurement. Component exchanges verify functional behavior,
not output power, frequency accuracy or spectral characteristics.

Declared component cases and production autonomous wakes, retries and resets
are allowed within the existing firmware wake/sleep policy. The operator owns
per-transmitter airtime accounting, admission and pacing across tests. Receiver
production airtime enforcement remains applicable.

Reassess the envelope when the module, antenna, RF path, configured power or PHY
changes, suitable instruments become available, or observations contradict the
configuration basis. Physical measurement requires declared quantities, limits,
calibrated instruments and uncertainty. Fixture checks and bounded cleanup are
required for every controlled run.

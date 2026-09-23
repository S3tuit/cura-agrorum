# RF operating-envelope decision, 2026-09-18

**RF-018 disposition: accepted. DEP-023 operating-envelope approval: accepted.**
RF-018 physical power/carrier/emissions measurement remains **NOT RUN**.

The operator instructed:

> For RF-018 disposition, record the accepted evidence and defer physical measurement as NOT RUN.
> For DEP-023 approval, the allowed hardware/configuration are the schematics documented for firmware and receiver. Autonomous wakes, retries and resets are allowed. We can close it.

This extends the earlier authorization to operate at configured +14 dBm while
trusting the SX1262 configuration without output-power measurement equipment.
It explicitly accepts the existing source/configuration basis for this pilot;
it does not claim that a new manufacturer uncertainty bound was found.

Accepted basis and scope:

- Firmware hardware: [SENSOR_CARRIER.md](../../firmware/test_apps/on_device/SENSOR_CARRIER.md),
  including its declared nominal and component fault fixtures.
- Receiver hardware: [TEST_CARRIER.md](../../receiver/hardware/TEST_CARRIER.md),
  with the documented Waveshare EU868 module/antenna chain and pin configuration.
- Configured power and PHY: the existing [firmware assessment](../../firmware/TESTING.md#sx1262_radio-first-rf-operating-envelope)
  and firmware/receiver interface configurations: intended +14 dBm, 868.1 MHz,
  SF7/BW125/CR4/5, explicit header, CRC, preamble 8, private sync word and
  direction-specific IQ. The existing nominal antenna/path estimate is accepted
  as source-based evidence; actual combined uncertainty remains unbounded.
- Allowed operation includes the declared component episodes and production
  autonomous wakes, retries and resets under the existing protocol and
  DEC-002-approved firmware wake-budget/sleep policy. Receiver production
  enforcement is unchanged. The operator retains test airtime accounting,
  admission and pacing; no automated harness ledger/scheduler is required.

This decision supersedes DEC-003's previous requirement to establish numerical
output uncertainty before approving this pilot envelope. It records operator
acceptance of the documented basis and remaining uncertainty, not a measured
RF result, an independent regulatory determination, or full-service/deployment
acceptance. Nominal component exchanges only establish their recorded functional
assertions and are not power, frequency or spectral measurements.

Retain RF-018 measurement as a follow-up obligation. Revisit when suitable RF
instruments become available, the module/antenna/path/power/PHY changes, or
conflicting RF evidence appears. Predeclare calibrated instruments, quantities,
limits and uncertainty before physical execution. Existing fixture readiness,
test expectations, cleanup/restoration and aggregate acceptance gates remain.

This is a maintained operating decision, not a retained test execution. The
accepted hardware and configuration are specified by the linked contracts;
changes to that basis require revisiting the approval as described above.

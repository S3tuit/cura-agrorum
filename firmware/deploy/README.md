# Nominal pilot firmware inputs

[sdkconfig.pilot](sdkconfig.pilot) selects the declared production
[carrier wiring](../test_apps/on_device/SENSOR_CARRIER.md). The disconnected
reference-voltage branch JP_REF_ENABLE/R5/R6/R7/R8 is absent; all other nominal
connections remain. This input changes board routing, not production behavior,
PHY, airtime policy or the ordinary 900-second sleep cadence.

Build the real `main/app_main.c` in an isolated current-source staging tree.
Do not copy existing secret headers or receiver groups into that tree. Provision
a dedicated test group and node using the protocol-owned tools, placing the
new header only in staged `firmware/main`. Preserve file mode 0600 and trusted
parent directories; no key bytes belong in evidence or source control.

Provide a separate local `sdkconfig.probes` with the two actually discovered,
distinct DS ROMs assigned to logical channels. Probe selection is not a pilot
design gate, but unprovisioned zero ROM defaults cannot qualify nominal sensing.
From staged `firmware`, with no existing sdkconfig, use:

```sh
idf.py -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;deploy/sdkconfig.pilot;sdkconfig.probes' build
```

An existing sdkconfig overrides defaults; never assume adding a defaults file
changes already resolved settings. Verify the resulting sdkconfig, compiled
inputs, dependency locks, partition table and flash files, and retain their
hashes alongside ELF/binary hashes. Keep images containing test credentials
private; their hashes and public identifiers suffice in published evidence.

The production layout remains 4 MiB flash, NVS at 0x9000 (24 KiB), factory app
at 0x10000 (1 MiB), and LittleFS at 0x110000 (2944 KiB), with PHY data at
0xf000 (4 KiB). Bare-C6 and sensor-carrier destructive tests overlap these
storage ranges. After those tests, use a new ID/key and the complete contracted
identity-state reset before authenticated TX; never restore an old identity or
migrate its backlog. Ordinary rebuilds of an active identity preserve counters.

Identity replacement follows the
[maintenance procedure](../maintenance/erase_storage/README.md): stop production,
generate the new identity, erase complete flash, cold-power-cycle to clear RTC,
then build/flash the new image. The storage-only maintenance app is insufficient
because it preserves NVS. This physical transition must be operator-controlled;
do not run old firmware after its counters are erased. Use the strict receiver
group loader to verify the exact group/allowlist, trusted nonsymlink parents,
service UID ownership and 0600 mode at installation. Restart the receiver after
group/allowlist changes. Retire old IDs through the protocol tool when replacing
an existing active identity; retain historical receiver data separately.

Disposable bench credentials are separate from final production credentials.
Final production handover remains a later agreed transition, bound to the final
artifact and affected checks under the evidence-reuse policy. Lost, exhausted or
erased counters never permit resuming the previous identity. No blanket re-soak
or automatic evidence transfer to changed credentials is assumed here.

Building this image does not authorize autonomous transmission or complete
production handover. Keep the non-transmitting test image installed until
the selected RF case, identity/storage state and operator airtime readiness
are established. See [testing boundaries](../TESTING.md#pilot-production-fixture-and-configuration-sequencing).

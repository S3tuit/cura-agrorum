# Destructive storage tests — 8 September 2026

**3 PASS in 6.14 s.** These short tests qualify because they fill storage, deny
access and remount a dedicated 4 MiB fixture read-only. All fixture mounts were
removed afterward. The [result](results.json) identifies the cases and tested
source: baseline `6f0f5ef006fd2d93284323b2f1af250cb3d56810` plus recorded local hashes.

Target: `cura-receiver` Pi, Linux `6.18.39+rpt-rpi-v8/aarch64`, Python 3.13.5,
SQLite 3.46.1, underlying filesystem `/dev/mmcblk0p2`.

The fast component suite, process-kill checks and 30-second stress run have no
permanent capture. Component fault recovery does not prove full-service,
deployment-soak or physical power-loss behavior. Rerun the
[persistence procedure](../../../../TESTING.md#persistence) when storage,
SQLite/recovery code or the deployed filesystem changes. The closed-handle
failure lesson belongs to that procedure and its host regression.

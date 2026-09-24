# Firmware tests worth keeping

Historical runs from **2026-09-18**, on C6 `cc8da2fc0224` through its
UART-labelled USB connector. Both qualify for retention because they destroy
node storage, regardless of their short duration.

| Test | Observed result | Why retained |
|---|---|---|
| Bare-C6 `all` suite | 43 Unity cases PASS; 181.75 seconds | Replaces the app and erases production-overlapping NVS/LittleFS, with explicit operator permission. |
| Sensor-carrier nominal `reading` | Software PASS; all five sensor groups valid and persisted reading matched | Erases NVS and formats LittleFS before and after the case. |

[results.json](results.json) contains the device/build identities, individual
bare-C6 case names, essential reading observations and end states. It is a
curated record; hashes identify the discarded original inputs. Bare-C6 XML/UART
checksums and result counts were checked before removal.

The reading run's original status remains
`software_passed_operator_acceptance_pending`: no new meter or electrical
acceptance is claimed. Radio calls used test adapters. The board was left on
a nontransmitting carrier test image; production firmware was not flashed.
A fresh node ID/key and full identity-state reset are required before later
authenticated transmission. These historical results do not qualify changed
source, a production image or RF behavior.

Fast host checks, ordinary builds, discovery, nominal acquisition and cleanup
runs are inexpensive to repeat and have no permanent archive here. Their
removal does not change historical outcomes. Full build/dependency manifests,
duplicate XML and verbose UART logs were discarded rather than moved elsewhere.

Keep only destructive tests, tests requiring physical intervention, or runs of
**more than five minutes**: a short result, identifying inputs, essential observations
and restoration/limits. Routine rerunnable checks need no evidence archive.
Do not add a source snapshot or full build seal merely to prove a test ran.

Follow the repository [evidence policy](../../../EVIDENCE.md); useful failure
lessons belong in the owning code or documentation, not archived failed runs.

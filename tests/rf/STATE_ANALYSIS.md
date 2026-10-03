# State analysis

`state_analysis.py` compares receiver and node snapshots taken before and after
an observation interval. It reports retained readings, delivery outcomes and
gaps in the available evidence. Run it on the Pi or laptop where the captures
and required source files are available. It reads captures without accessing
devices or modifying the inputs; the operator defines the observation interval,
records interventions and decides acceptance.

## Before and after captures

Use the existing service-UID `service_probe.py snapshot` operation for a consistent SQLite backup. Do not copy a live SQLite main file without its transaction context. The analyzer accepts standalone backups, rejects adjacent WAL/journal files, validates integrity/foreign keys and checks the production schema fingerprint/group identity.

Use `production_node.capture_storage` for the stopped node's complete LittleFS image, public image binding and production build seal. Preserve NVS and identity-lifetime counters. At the initial boundary, capture before starting the observation interval; at the final boundary, stop production first, capture, and leave it stopped. Raw flash reading must not interrupt a measured wake sequence. Do not reformat between these two captures.

Retain the exact installed service configuration manifest (`schema`, `unit`, `package`, `test_root`, `user`, `files`), the production build seal and operator intervention/supply notes. This service manifest contains paths and hashes, not the private receiver-group contents. Keep credentials outside the capture set. Supply the public group ID on the command line.

Seal each already collected capture locally. Replace the paths/public ID below with the actual files; outputs must be new:

```sh
.venv/bin/python tests/rf/state_analysis.py seal \
  --database /captures/before/receiver.sqlite3 \
  --image /captures/before/node.bin \
  --binding /captures/before/node-binding.json \
  --build /captures/production-rf-build.json \
  --service /captures/service-config.json \
  --group PUBLIC_GROUP_ID_HEX \
  --output /captures/before/capture.json
```

Repeat for the final capture. Keep build/configuration/identity constant through the observation interval. The tool rejects a mismatch instead of silently combining two setups. The manifest binds absolute local paths and hashes. If moving the artifacts, reseal them at the new location after verifying the original hashes; retain the original manifest as provenance.

```sh
.venv/bin/python tests/rf/state_analysis.py report \
  --before /captures/before/capture.json \
  --after /captures/after/capture.json \
  --output /captures/state-analysis.json
```

The reader compiles the current production LittleFS/record-validation code with the existing read-only adapter. The report retains decoder source hashes. Invalid/torn images, changed input hashes, replaced append-only histories, conflicting transport identities and source/configuration mismatches fail without modifying captures. A failure is incomplete evidence, not a zero-loss result.

## Interpretation

- `samples` separates durable Pi storage, final node retention and quarantine. `missing_from_both` lists evidenced samples in neither final store; `conflicting_bodies` lists content disagreements. Retained-only readings are preserved but not delivered.
- `messages` maps transport IDs to sample IDs and reports received profiles, earlier observations, recorded attempts and per-cycle node outcomes. Current/backlog message IDs do not become two generated samples; retries do not become new transport identities.
- `counts` distinguishes observed samples, durable samples/added transport rows, retained samples, recorded node attempts, authenticated receiver profiles/retries, receiver ACK TX_DONE profiles and node-accepted deliveries. A Pi ACK transmission alone does not prove node acceptance.
- `unfinished_deliveries`, `sample_counter_gap_ranges`, `accepted_without_profile` and `observation_gaps` preserve missing-outcome/coverage uncertainty. A Pi power cut may lose volatile profiles while the durable reading survives. Recorded attempt totals exclude outcomes the node did not retain.
- Counter-gap ranges include the interval from the latest current delivery in the initial capture to new current deliveries. Earlier historical gaps stay outside the measured interval. This continuity anchor does not add historical or wholly unobserved samples to the denominator.
- `receiver_instances` reports boot and instance identity plus whether each has a clean-stop marker. Power cuts may leave no marker; record actual interventions separately from observed clean stops. Monotonic timestamps are not combined across boots or fabricated into UTC.
- `diagnostics` exposes retained node records for review. Every initial diagnostic and delivery record must remain as an identical ordered prefix of the final history; missing, shortened, changed or reordered history fails reconciliation. The report does not hide quarantine or diagnostics merely because every known sample is somewhere durable.

The observation denominator includes starting retained readings and new evidence in the receiver/node captures. Historical completed readings are excluded unless they participate again. Samples unseen by every input cannot be counted. Neither counter gaps nor nominal cadence establish an exact generated total. The report has no automatic acceptance decision: review duration, actual interventions, continuity of the actual supplies, receiver health/recovery, storage and airtime evidence with the operator.

Deployment plans and results belong in the sibling logbook. Plotting, link
targets, UTC reconstruction and maintenance/recovery policy are outside this
tool's scope.

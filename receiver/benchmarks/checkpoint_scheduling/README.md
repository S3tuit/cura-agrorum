# Real-worker checkpoint comparison

This fixture compares exact baseline and changed receiver sources on isolated
SQLite databases on the Pi storage medium. It never starts a service, accesses
radio/RTC hardware, changes SQLite durability mode, or opens pilot data.

Run the same harness against two source roots using the same Pi Python/SQLite
environment. Both use worker defaults: FULL commits, five-second ordinary
flushing, 64-entity batches, and each source's checkpoint policy. Select the two
source roots to compare and record their hashes and Python/SQLite versions with
the temporary results. Sources must not change during a run.

Before execution, confirm the Pi SSH endpoint, a writable parent directory on
the intended SD filesystem, and the matching whole-device block-stat path.
Each `--fixture-root` must be a new absolute directory. Put result JSON and
shell logs on RAM storage, such as `/dev/shm`, and retrieve them before reboot.
Keep the receiver service's condition the same for both runs; record background
activity rather than stopping a production service without agreement.

Default phases for each source:

1. Populate actual profile rows until WAL allocation exceeds 5 MiB, then fully
   checkpoint using the caller-driven component. The resulting allocation and
   history are retained. Preparation is excluded from measurement.
2. Fsync only private fixture files/directory, allow five seconds for delayed
   writes, and measure 30 seconds of whole-device background write counters.
3. Start the real persistence worker and allow six seconds for initial
   maintenance. Observe 30 seconds with no writes. The changed worker must make
   no idle checkpoint calls after initial complete coverage; the retained
   large WAL must not produce periodic maintenance.
4. Offer one valid profile unit per second for 120 seconds, with one synchronous
   state read per unit. Hold a real SQLite read transaction for the first
   30 seconds to exercise partial progress. Record actual checkpoint results,
   commit/control/queue durations and maximum sampled queue length.
5. Release the reader, close and drain the queue, commit a clean-stop marker and
   request bounded worker shutdown. Fsync the private files/directory, include
   a five-second writeback observation window, and verify exact durable counts
   and SQLite integrity.

Example for each source (replace the explicit paths after fixture selection):

```bash
python /path/to/changed/receiver/benchmarks/checkpoint_scheduling/run.py \
  --source-root /path/to/baseline-or-changed \
  --source-commit COMMIT_OF_SELECTED_SOURCE \
  --label baseline-or-changed \
  --fixture-root /sd-test-parent/checkpoint-baseline-or-changed \
  --block-stat /sys/block/mmcblk0/stat \
  --output /dev/shm/checkpoint-baseline-or-changed.json
```

Run target persistence checks for the changed source separately, selecting
`receiver/tests/hardware/test_persistence_worker.py` with `--receiver-hardware`
and initially excluding its `slow` cases. The timer/stall test uses a
one-second interval as a fixture parameter; the comparison above uses the
production five-second default.

Acceptance requires exact durable traffic and marker outcomes, integrity,
no worker failure, an idle checkpoint-call count of zero for the changed
source after complete initial coverage, and real reader-limited partial
progress followed by complete coverage. Control calls must meet their existing
five-second deadline and 0.5-second caller scheduling tolerance. Compare queue
and control percentiles as measurements rather than inferring a kernel-I/O
duration bound. Keep the five-second default pending an agreed tuning decision.

The JSON includes all raw checkpoint/control/queue observations, cumulative
worker counters, source hashes, fixture parameters and phase write-byte totals.
Checkpoint frame counts include previously checkpointed frames. Derive
complete/partial outcomes from each result rather than historical success
counters.

Whole-device write bytes use field 7 of the full `/sys/block/<device>/stat`
layout, in 512-byte sectors. Report raw totals and, if helpful, the estimate
after subtracting the background rate over each phase's actual duration.
Counter resets invalidate comparison. Background traffic varies, and the
private fsync/writeback window does not isolate NAND writes or prove all
unrelated filesystem writeback has finished. Do not infer SD wear, NAND write
amplification, remaining lifetime or physical-write savings from call counts.
[Linux block-stat documentation](https://docs.kernel.org/admin-guide/iostats.html).

`--host-smoke` permits short laptop runs only to check fixture plumbing and
baseline compatibility. Such results are explicitly labelled and provide no
Pi, RF or storage-medium acceptance. Keep the fixture databases until results
are reviewed; the harness does not delete them.

Interpret fewer checkpoint calls separately from storage latency or physical
write savings. An idle WAL can remain allocated after complete coverage; its
size alone must not trigger repeated maintenance. A Pi comparison removed idle
calls without reducing the largest sampled checkpoint stall (about 167 ms).
Whole-device background writes varied enough to make background subtraction
negative, so those counters could not establish flash-write or wear savings.
The five-second interval is not claimed to be optimal.

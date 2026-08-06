# Firmware component interfaces

Status: provisional pilot contract. This document defines the semantics that
host fakes and production adapters must share. Names, numeric values and C
layouts in this file may be revised during implementation.

## Common diagnostic result model

`err_curag_t` is the packed error value defined in
[`ARCHITECTURE.md`](ARCHITECTURE.md):

```text
31                    16 15                     0
+-----------------------+------------------------+
| error_domain          | error_code             |
+-----------------------+------------------------+
```

Zero means success. The persistence domain is
`CURAG_EDOM_PERSISTENCE = 1`. Its assigned error codes are:

| Value | Error code | Meaning |
|---:|---|---|
| `0` | `NONE` | Success only |
| `1` | `CURAG_ENVS_INIT` | NVS initialization failed |
| `2` | `CURAG_ENVS_ACCESS` | NVS namespace open, read, write or commit failed |
| `3` | `CURAG_ESAMPLE_ID_EXHAUSTED` | No further `u32` sample ID can be committed without reuse |
| `4` | `CURAG_ELITTLEFS_INIT` | LittleFS initialization failed |
| `5` | `CURAG_EINVALID_ARGUMENT` | Caller violated an interface precondition |
| `6` | `CURAG_EIO` | A backend file operation or synchronization failed |
| `7` | `CURAG_ELOG_FULL` | A logical file quota rejected an append |
| `8` | `CURAG_ECORRUPT_RECORD` | Record framing or CRC could not be recovered safely |
| `9` | `CURAG_EUNSUPPORTED_RECORD` | A structurally valid record has an unsupported version or type |
| `10` | `CURAG_ERECORD_MISMATCH` | The newest pending record is not the expected semantic record |

Every ordinary persistence method returns only `err_curag_t` and accepts an
optional caller-owned `diagn_context_t *out_diag`. The public method clears a
non-null output on entry. On failure, the private code at the failure site
populates its operation and canonical context. On success it remains:

```text
operation       NONE
context_schema  NONE
context_length  0
context         zeroed
```

Passing `out_diag = NULL` discards diagnostic detail without changing the
operation or returned error. The component never retains the pointer. An outer
helper does not overwrite a more precise context already populated by a deeper
helper.

Every nonzero persistence result with a non-null diagnostic output must set a
non-`NONE` operation and `CURAG_PERSISTENCE_CONTEXT_V1` with length seven,
including semantic failures that use `NO_ERROR`. This makes a mismatched error
and context an interface-contract violation rather than something `node_core`
must repair.

`node_core` does not interpret component context. It extracts the domain and
code from `err_curag_t`, copies the component-produced operation, schema and
context bytes, adds the application offset and cycle ID with their validity
flags, and calls `append_diagnostic_event` best-effort. Failure of that append
is never logged recursively.

### Persistence context V1

Context-schema values are scoped by error domain. For persistence:

| Value | Schema |
|---:|---|
| `0` | `NONE`; requires `context_length = 0` |
| `1` | `CURAG_PERSISTENCE_CONTEXT_V1`; requires `context_length = 7` |

`CURAG_PERSISTENCE_CONTEXT_V1` has one fixed encoding for every persistence
method:

| Field | Encoding |
|---|---:|
| `resource` | `u8` |
| `stage` | `u8` |
| `backend_status_kind` | `u8` |
| `backend_status` | `i32`, little-endian |

Assigned resource values are:

| Value | Resource |
|---:|---|
| `0` | `NONE` |
| `1` | `NVS_SAMPLE_COUNTER` |
| `2` | `LITTLEFS` |
| `3` | `PENDING_LOG` |
| `4` | `PENDING_COMPACT` |
| `5` | `QUARANTINE_LOG` |
| `6` | `DIAGNOSTIC_LOG` |
| `7` | `DELIVERY_LOG` |

Assigned stage values are:

| Value | Stage |
|---:|---|
| `0` | `NONE` |
| `1` | `INITIALIZE` |
| `2` | `GET` |
| `3` | `SET` |
| `4` | `COMMIT` |
| `5` | `OPEN` |
| `6` | `STAT` |
| `7` | `READ` |
| `8` | `WRITE` |
| `9` | `SYNC` |
| `10` | `TRUNCATE` |
| `11` | `RENAME` |
| `12` | `REMOVE` |
| `13` | `CLOSE` |
| `14` | `TAIL_SCAN` |
| `15` | `QUOTA_CHECK` |
| `16` | `ENCODE` |
| `17` | `DECODE` |

Assigned backend-status kinds are:

| Value | Kind |
|---:|---|
| `0` | `NO_ERROR`; requires `backend_status = 0` |
| `1` | `ERRNO` |
| `2` | `ESP_ERR` |
| `3` | `LITTLEFS_ERROR` |

The raw status is always normalized to signed 32 bits before little-endian
encoding. Semantic failures such as exhaustion, quota, unsupported data and
record mismatch use `NO_ERROR` because there is no underlying backend error.

The mappings below use this shorthand:

```text
PCTX(operation, resource, stage, status-kind)
```

`NO_ERROR` implies status zero. `ERRNO`, `ESP_ERR` and `LITTLEFS_ERROR` carry
the exact status observed at the failure site.

For backend failures during recovery, `operation` remains `RECOVER` while
`stage` names the exact failed primitive such as `OPEN`, `READ`, `TRUNCATE` or
`SYNC`. `TAIL_SCAN` is reserved for a semantic failure to establish a valid
record boundary when no backend status caused it. More generally, a backend
failure not listed separately below uses the exact resource and primitive
stage, while retaining the enclosing semantic operation.

## Shared persistence values

All pointers are borrowed. A persistence method copies or encodes inputs before
returning and never retains pointers. Outputs are valid only on success or the
explicitly documented empty result. Calls are single-threaded during one wake;
thread safety is not required.

Interfaces pass `sample_id` and `cura_lora_v2_reading_t` separately rather than
introducing a persistence-specific aggregate type. Persistence serializes the
reading through the generated protocol codec and never dumps its native
structure.

There is no public initialization method. NVS and LittleFS initialize
independently on first use and cache initialization failure for the remainder
of the wake.

Every append establishes or repairs a trustworthy tail before writing. If
bounded recovery discards an unusable newest record, the requested operation is
not executed and the method returns the original corruption or unsupported
error so `node_core` can persist a diagnostic. Caching per-file validation state
within a wake is only a possible future optimization.

## `node_persistence`

### `claim_sample_id`

Implemented shape:

```text
err_curag_t node_persistence_claim_sample_id(
    u32 *out_sample_id,
    diagn_context_t *out_diag)
```

**Purpose**

Atomically claim the next identity-lifetime sample ID before sensors or radio
are activated.

**Inputs and ownership**

`out_sample_id` is non-null writable caller memory. `out_diag` is optional.
Neither pointer is retained.

**Outputs**

On success, writes the claimed ID after its successor has been committed to
NVS. Fresh storage claims `0`. No output ID is valid on failure.

**Side effects**

Lazily initializes NVS, opens the sample-counter namespace, reads the next ID,
commits its successor and caches NVS initialization state. An ID may be skipped
after an ambiguous commit outcome, but is never deliberately reused.

**Failure results and diagnostic context**

| Failure | Error | `diagn_context_t` |
|---|---|---|
| Null output | `CURAG_EINVALID_ARGUMENT` | `PCTX(VALIDATE, NVS_SAMPLE_COUNTER, NONE, NO_ERROR)` |
| NVS initialization | `CURAG_ENVS_INIT` | `PCTX(INITIALIZE, NVS_SAMPLE_COUNTER, INITIALIZE, ESP_ERR)` |
| Namespace open | `CURAG_ENVS_ACCESS` | `PCTX(INITIALIZE, NVS_SAMPLE_COUNTER, OPEN, ESP_ERR)` |
| Counter read | `CURAG_ENVS_ACCESS` | `PCTX(READ, NVS_SAMPLE_COUNTER, GET, ESP_ERR)` |
| Counter set | `CURAG_ENVS_ACCESS` | `PCTX(WRITE, NVS_SAMPLE_COUNTER, SET, ESP_ERR)` |
| Counter commit | `CURAG_ENVS_ACCESS` | `PCTX(SYNC, NVS_SAMPLE_COUNTER, COMMIT, ESP_ERR)` |
| Counter exhausted | `CURAG_ESAMPLE_ID_EXHAUSTED` | `PCTX(VALIDATE, NVS_SAMPLE_COUNTER, NONE, NO_ERROR)` |

**`node_core` response**

Persist one best-effort diagnostic with no valid cycle sample ID, skip sensor
and radio activation, run normal cleanup and final synchronization, leave the
outgoing RTC record invalid, and enter deep sleep.

### `append_pending_reading`

Implemented shape:

```text
err_curag_t node_persistence_append_pending_reading(
    u32 sample_id,
    const cura_lora_v2_reading_t *reading,
    diagn_context_t *out_diag)
```

**Purpose**

Durably append the input reading.

**Inputs and ownership**

`sample_id` is the ID claimed for the current wake. `reading` is non-null,
caller-owned and borrowed only for the call. `out_diag` is optional.

**Outputs**

Success means the complete 46-byte pending record is synchronized and
recoverable after reset. Failure does not promise that no bytes were written;
tail recovery resolves a torn or ambiguously synchronized append before a later
operation can bury it.

**Side effects**

Lazily initializes LittleFS, performs tail recovery, encodes and appends the
record, synchronizes it, and may compact `pending.log` before appending when its
logical quota would be exceeded.

**Failure results and diagnostic context**

| Failure | Error | `diagn_context_t` |
|---|---|---|
| Invalid input or reading | `CURAG_EINVALID_ARGUMENT` | `PCTX(VALIDATE, PENDING_LOG, NONE, NO_ERROR)` |
| LittleFS initialization | `CURAG_ELITTLEFS_INIT` | `PCTX(INITIALIZE, LITTLEFS, INITIALIZE, ESP_ERR)` |
| Stale temporary-file removal | `CURAG_EIO` | `PCTX(RECOVER, PENDING_COMPACT, REMOVE, ERRNO)` |
| Tail recovery open/read | `CURAG_EIO` | `PCTX(RECOVER, PENDING_LOG, OPEN or READ, ERRNO)` |
| Tail recovery truncate/sync | `CURAG_EIO` | `PCTX(RECOVER, PENDING_LOG, TRUNCATE or SYNC, ERRNO)` |
| Torn, CRC-invalid or semantically invalid tail removed | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, PENDING_LOG, TRUNCATE, NO_ERROR)` |
| Unsupported tail removed | `CURAG_EUNSUPPORTED_RECORD` | `PCTX(RECOVER, PENDING_LOG, TRUNCATE, NO_ERROR)` |
| No trustworthy preceding boundary | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, PENDING_LOG, TAIL_SCAN, NO_ERROR)` |
| Reading encoding | `CURAG_EINVALID_ARGUMENT` | `PCTX(ENCODE, PENDING_LOG, ENCODE, NO_ERROR)` |
| Pending-file size query | `CURAG_EIO` | `PCTX(READ, PENDING_LOG, STAT, ERRNO)` |
| Compaction source read | `CURAG_EIO` | `PCTX(COMPACT, PENDING_LOG, READ, ERRNO)` |
| Compaction temporary open/write | `CURAG_EIO` | `PCTX(COMPACT, PENDING_COMPACT, OPEN or WRITE, ERRNO)` |
| Compaction temporary synchronization | `CURAG_EIO` | `PCTX(COMPACT, PENDING_COMPACT, SYNC, ERRNO)` |
| Compaction replacement | `CURAG_EIO` | `PCTX(COMPACT, PENDING_COMPACT, RENAME, ERRNO)` |
| Quota still cannot admit record | `CURAG_ELOG_FULL` | `PCTX(COMPACT, PENDING_LOG, QUOTA_CHECK, NO_ERROR)` |
| Pending-file open/write | `CURAG_EIO` | `PCTX(APPEND, PENDING_LOG, OPEN or WRITE, ERRNO)` |
| Pending-record synchronization | `CURAG_EIO` | `PCTX(SYNC, PENDING_LOG, SYNC, ERRNO)` |

`OPEN or WRITE` denotes two distinct emitted contexts using the listed stage,
not one ambiguous stage value.

**`node_core` response**

Persist the returned error and context best-effort and transmit the in-RAM
current reading. For the pilot, do not access backlog storage again in that
wake: failed append durability may be uncertain, and a peek could immediately
select the same current reading. Final `sync_all` is still called.

### `peek_most_recent_pending`

Implemented shape:

```text
err_curag_t node_persistence_peek_most_recent_pending(
    u32 *out_sample_id,
    cura_lora_v2_reading_t *out_reading,
    bool *out_found,
    diagn_context_t *out_diag)
```

**Purpose**

Return the newest complete pending reading for backlog delivery without
removing it.

**Inputs and ownership**

`out_sample_id`, `out_reading` and `out_found` are non-null writable caller
memory. `out_diag` is optional. No pointer is retained.

**Outputs**

On success, `out_found = true`, `out_sample_id` contains the stored ID and
`out_reading` contains the decoded newest reading, or `out_found = false` when
the log is empty. Empty backlog is normal, not an error. The other outputs have
no meaning when empty or on failure.

**Side effects**

Lazily initializes LittleFS and establishes a trustworthy tail. Destructive
recovery synchronizes the truncation and returns an error without returning a
reading; an ordinary successful peek is read-only.

**Failure results and diagnostic context**

| Failure | Error | `diagn_context_t` |
|---|---|---|
| Invalid output | `CURAG_EINVALID_ARGUMENT` | `PCTX(VALIDATE, PENDING_LOG, NONE, NO_ERROR)` |
| LittleFS initialization | `CURAG_ELITTLEFS_INIT` | `PCTX(INITIALIZE, LITTLEFS, INITIALIZE, ESP_ERR)` |
| Pending-file open/read | `CURAG_EIO` | `PCTX(READ, PENDING_LOG, OPEN or READ, ERRNO)` |
| Tail recovery open/read | `CURAG_EIO` | `PCTX(RECOVER, PENDING_LOG, OPEN or READ, ERRNO)` |
| Tail recovery truncate/sync | `CURAG_EIO` | `PCTX(RECOVER, PENDING_LOG, TRUNCATE or SYNC, ERRNO)` |
| Torn, CRC-invalid or semantically invalid tail removed | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, PENDING_LOG, TRUNCATE, NO_ERROR)` |
| Unsupported tail removed | `CURAG_EUNSUPPORTED_RECORD` | `PCTX(RECOVER, PENDING_LOG, TRUNCATE, NO_ERROR)` |
| No trustworthy preceding boundary | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, PENDING_LOG, TAIL_SCAN, NO_ERROR)` |

**`node_core` response**

An empty result ends backlog drainage normally. A failure is persisted
best-effort and stops backlog drainage without changing an already accepted
current-reading result.

### `remove_newest_reading`

Implemented shape:

```text
err_curag_t node_persistence_remove_newest_reading(
    u32 expected_sample_id,
    diagn_context_t *out_diag)
```

**Purpose**

Durably remove the newest pending reading after authenticated `ACCEPTED`, or
after a permanent rejection has triggered a quarantine attempt.

**Inputs and ownership**

`expected_sample_id` identifies the exact pending reading processed by
`node_core`. `out_diag` is optional and is not retained.

**Outputs**

Success means the newest supported `PENDING_READING` had the expected ID and
was removed with its truncation synchronized.

**Side effects**

Lazily initializes LittleFS, establishes a trustworthy tail, validates its
record type and decoded sample ID, truncates exactly that record and
synchronizes the truncation. Private recovery may instead remove one unusable
tail and return an error without removing the caller's expected reading.

**Failure results and diagnostic context**

| Failure | Error | `diagn_context_t` |
|---|---|---|
| LittleFS initialization | `CURAG_ELITTLEFS_INIT` | `PCTX(INITIALIZE, LITTLEFS, INITIALIZE, ESP_ERR)` |
| Tail open/read | `CURAG_EIO` | `PCTX(READ, PENDING_LOG, OPEN or READ, ERRNO)` |
| Missing or different supported tail ID/type | `CURAG_ERECORD_MISMATCH` | `PCTX(VALIDATE, PENDING_LOG, TAIL_SCAN, NO_ERROR)` |
| Torn, CRC-invalid or semantically invalid tail removed | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, PENDING_LOG, TRUNCATE, NO_ERROR)` |
| Unsupported tail removed | `CURAG_EUNSUPPORTED_RECORD` | `PCTX(RECOVER, PENDING_LOG, TRUNCATE, NO_ERROR)` |
| No trustworthy preceding boundary | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, PENDING_LOG, TAIL_SCAN, NO_ERROR)` |
| Recovery truncate/sync failure | `CURAG_EIO` | `PCTX(RECOVER, PENDING_LOG, TRUNCATE or SYNC, ERRNO)` |
| Expected-reading truncation | `CURAG_EIO` | `PCTX(REMOVE, PENDING_LOG, TRUNCATE, ERRNO)` |
| Expected-reading synchronization | `CURAG_EIO` | `PCTX(SYNC, PENDING_LOG, SYNC, ERRNO)` |

**`node_core` response**

After `ACCEPTED`, retain the authenticated result and metrics, persist the
returned diagnostic best-effort, and stop further radio/backlog work on
failure. After a quarantine attempt, persist removal failure and stop drainage;
for a backlog entry, successful removal permits drainage to continue even when
the separate quarantine append failed. A permanently rejected current reading
still ends the radio phase.

### `quarantine_reading`

Implemented shape:

```text
err_curag_t node_persistence_quarantine_reading(
    u32 sample_id,
    const cura_lora_v2_reading_t *reading,
    diagn_context_t *out_diag)
```

**Purpose**

Durably append a permanently rejected reading to `quarantine.log`. It never
reads or modifies `pending.log`.

**Inputs and ownership**

`sample_id` and `reading` identify the rejected reading already held in RAM.
`reading` is non-null, caller-owned and borrowed only for the call. `out_diag`
is optional. No pointer is retained.

**Outputs**

Success means the complete quarantine record is synchronized and recoverable
after reset.

**Side effects**

Lazily initializes LittleFS, establishes a trustworthy quarantine tail,
encodes and appends the record, and synchronizes it before returning success.
Bounded destructive recovery returns an error without appending the requested
quarantine record.

**Failure results and diagnostic context**

| Failure | Error | `diagn_context_t` |
|---|---|---|
| Invalid input or reading | `CURAG_EINVALID_ARGUMENT` | `PCTX(VALIDATE, QUARANTINE_LOG, NONE, NO_ERROR)` |
| LittleFS initialization | `CURAG_ELITTLEFS_INIT` | `PCTX(INITIALIZE, LITTLEFS, INITIALIZE, ESP_ERR)` |
| Full quarantine log | `CURAG_ELOG_FULL` | `PCTX(APPEND, QUARANTINE_LOG, QUOTA_CHECK, NO_ERROR)` |
| Tail recovery open/read | `CURAG_EIO` | `PCTX(RECOVER, QUARANTINE_LOG, OPEN or READ, ERRNO)` |
| Tail recovery truncate/sync | `CURAG_EIO` | `PCTX(RECOVER, QUARANTINE_LOG, TRUNCATE or SYNC, ERRNO)` |
| Torn, CRC-invalid or semantically invalid tail removed | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, QUARANTINE_LOG, TRUNCATE, NO_ERROR)` |
| Unsupported tail removed | `CURAG_EUNSUPPORTED_RECORD` | `PCTX(RECOVER, QUARANTINE_LOG, TRUNCATE, NO_ERROR)` |
| No trustworthy preceding boundary | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, QUARANTINE_LOG, TAIL_SCAN, NO_ERROR)` |
| Quarantine encoding | `CURAG_EINVALID_ARGUMENT` | `PCTX(ENCODE, QUARANTINE_LOG, ENCODE, NO_ERROR)` |
| Quarantine open/write | `CURAG_EIO` | `PCTX(APPEND, QUARANTINE_LOG, OPEN or WRITE, ERRNO)` |
| Quarantine synchronization | `CURAG_EIO` | `PCTX(SYNC, QUARANTINE_LOG, SYNC, ERRNO)` |

**`node_core` response**

Persist failure and context best-effort. For a known persisted rejected
reading, call `remove_newest_reading(sample_id)` afterward even when quarantine
failed. A rejected current reading ends radio work. For backlog, continue only
when that separate removal succeeds. For a RAM-only current reading whose
pending append failed, do not attempt removal.

### `append_diagnostic_event`

Implemented shape:

```text
err_curag_t node_persistence_append_diagnostic_event(
    const node_diagnostic_event_t *event,
    diagn_context_t *out_diag)
```

`node_diagnostic_event` contains the originating `err_curag_t`, the two
validity flags, application offset, cycle ID, and a borrowed pointer to the
originating `diagn_context_t`. A null originating context encodes operation
`NONE`, context schema `NONE` and zero context length.

**Purpose**

Best-effort append of the canonical diagnostic event defined in
[`ARCHITECTURE.md`](ARCHITECTURE.md).

**Inputs and ownership**

`event`, its originating diagnostic context and all context bytes are borrowed
only for the call. `out_diag` describes failure of this append itself and is
optional. No input may contain protocol keys.

**Outputs**

Success means the event bytes were accepted by the file/backend buffer.
Ordinary diagnostics are not guaranteed durable until `sync_all` succeeds.

**Side effects**

Lazily initializes LittleFS, establishes a trustworthy tail, validates and
canonically encodes the event, and appends it to `diagnostic.log`. It does not
synchronize solely for an ordinary diagnostic event.

**Failure results and diagnostic context**

| Failure | Error | `out_diag` |
|---|---|---|
| Invalid event, flags or context | `CURAG_EINVALID_ARGUMENT` | `PCTX(VALIDATE, DIAGNOSTIC_LOG, NONE, NO_ERROR)` |
| LittleFS initialization | `CURAG_ELITTLEFS_INIT` | `PCTX(INITIALIZE, LITTLEFS, INITIALIZE, ESP_ERR)` |
| Full diagnostic log | `CURAG_ELOG_FULL` | `PCTX(APPEND, DIAGNOSTIC_LOG, QUOTA_CHECK, NO_ERROR)` |
| Tail recovery open/read | `CURAG_EIO` | `PCTX(RECOVER, DIAGNOSTIC_LOG, OPEN or READ, ERRNO)` |
| Tail recovery truncate/sync | `CURAG_EIO` | `PCTX(RECOVER, DIAGNOSTIC_LOG, TRUNCATE or SYNC, ERRNO)` |
| Torn, CRC-invalid or semantically invalid tail removed | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, DIAGNOSTIC_LOG, TRUNCATE, NO_ERROR)` |
| Unsupported tail removed | `CURAG_EUNSUPPORTED_RECORD` | `PCTX(RECOVER, DIAGNOSTIC_LOG, TRUNCATE, NO_ERROR)` |
| No trustworthy preceding boundary | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, DIAGNOSTIC_LOG, TAIL_SCAN, NO_ERROR)` |
| Event encoding | `CURAG_EINVALID_ARGUMENT` | `PCTX(ENCODE, DIAGNOSTIC_LOG, ENCODE, NO_ERROR)` |
| Diagnostic open/write | `CURAG_EIO` | `PCTX(APPEND, DIAGNOSTIC_LOG, OPEN or WRITE, ERRNO)` |

**`node_core` response**

Discard the error and `out_diag`. Never invoke `append_diagnostic_event` to
diagnose itself, and never change sensing, delivery or sleep behavior because
diagnostic logging failed.

### `append_delivery_event`

Implemented shape:

```text
err_curag_t node_persistence_append_delivery_event(
    const node_delivery_event_t *event,
    diagn_context_t *out_diag)
```

`node_delivery_event` is a tagged caller-owned value containing exactly one of
the `DELIVERY_STARTED` or `DELIVERY_FINISHED` schemas in
[`ARCHITECTURE.md`](ARCHITECTURE.md).

**Purpose**

Durably bracket one complete current or backlog delivery episode.

**Inputs and ownership**

`event` is non-null and borrowed only for the call. `out_diag` is optional. No
pointer is retained.

**Outputs**

Success means the complete event is synchronized and recoverable after reset.

**Side effects**

Lazily initializes LittleFS, establishes a trustworthy tail, validates and
appends the selected record type to `delivery.log`, and synchronizes it before
success.

**Failure results and diagnostic context**

| Failure | Error | `out_diag` |
|---|---|---|
| Invalid event tag or fields | `CURAG_EINVALID_ARGUMENT` | `PCTX(VALIDATE, DELIVERY_LOG, NONE, NO_ERROR)` |
| LittleFS initialization | `CURAG_ELITTLEFS_INIT` | `PCTX(INITIALIZE, LITTLEFS, INITIALIZE, ESP_ERR)` |
| Full delivery log | `CURAG_ELOG_FULL` | `PCTX(APPEND, DELIVERY_LOG, QUOTA_CHECK, NO_ERROR)` |
| Tail recovery open/read | `CURAG_EIO` | `PCTX(RECOVER, DELIVERY_LOG, OPEN or READ, ERRNO)` |
| Tail recovery truncate/sync | `CURAG_EIO` | `PCTX(RECOVER, DELIVERY_LOG, TRUNCATE or SYNC, ERRNO)` |
| Torn, CRC-invalid or semantically invalid tail removed | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, DELIVERY_LOG, TRUNCATE, NO_ERROR)` |
| Unsupported tail removed | `CURAG_EUNSUPPORTED_RECORD` | `PCTX(RECOVER, DELIVERY_LOG, TRUNCATE, NO_ERROR)` |
| No trustworthy preceding boundary | `CURAG_ECORRUPT_RECORD` | `PCTX(RECOVER, DELIVERY_LOG, TAIL_SCAN, NO_ERROR)` |
| Event encoding | `CURAG_EINVALID_ARGUMENT` | `PCTX(ENCODE, DELIVERY_LOG, ENCODE, NO_ERROR)` |
| Delivery-file open/write | `CURAG_EIO` | `PCTX(APPEND, DELIVERY_LOG, OPEN or WRITE, ERRNO)` |
| Delivery synchronization | `CURAG_EIO` | `PCTX(SYNC, DELIVERY_LOG, SYNC, ERRNO)` |

**`node_core` response**

Persist the returned diagnostic best-effort when possible, but never delay or
cancel transmission because `DELIVERY_STARTED` failed and never alter a
terminal result because `DELIVERY_FINISHED` failed.

### `sync_all`

Implemented shape:

```text
err_curag_t node_persistence_sync_all(diagn_context_t *out_diag)
```

**Purpose**

Perform the single final synchronization of persistence state still buffered,
especially ordinary diagnostics.

**Inputs and ownership**

`out_diag` is optional and is not retained.

**Outputs**

Success means every initialized backend with buffered state completed its
required synchronization and all persistence-owned handles were closed.

**Side effects**

Synchronizes only backends initialized during the wake. It does not initialize
merely to finalize and does not retry an initialization failure cached earlier.
Sample-ID commits, pending transitions and delivery events are already durable
before this call. It closes persistence-owned LittleFS file handles and the NVS
handle, but does not unregister or unmount LittleFS; deep sleep discards the
remaining in-memory mount state.

**Failure results and diagnostic context**

| Failure | Error | `out_diag` |
|---|---|---|
| Diagnostic-log synchronization | `CURAG_EIO` | `PCTX(SYNC, DIAGNOSTIC_LOG, SYNC, ERRNO)` |
| Other initialized LittleFS synchronization | `CURAG_EIO` | `PCTX(SYNC, LITTLEFS, SYNC, ERRNO)` |
| Persistence-owned file close | `CURAG_EIO` | `PCTX(SYNC, affected resource, CLOSE, ERRNO)` |

**`node_core` response**

Do not append another persistent diagnostic because its durability could not
be established without a second synchronization. Continue RTC finalization
and deep sleep. Development firmware may still emit the error and context to a
non-persistent platform log.

## Open persistence-interface decisions

- Recovery or reclamation for full quarantine, diagnostic and delivery logs is
  intentionally undefined for the pilot. Until physical intervention, each
  full log rejects new records under its documented best-effort policy.
- The physical workflow for a log whose tail has no trustworthy preceding
  boundary is intentionally undefined for the pilot. The interface behavior is
  not open: preserve every byte and return `CURAG_ECORRUPT_RECORD`. In
  particular, persistence must not guess a boundary, reformat LittleFS or use
  `pending.compact` as a recovery source.

## `node_sensors`

Status: implemented pilot contract. The provisional GPIO defaults and physical
sensor identities must be replaced when the pilot wiring is assembled. The
five logical sensor groups and all-or-none enclosure validity are fixed.

### Public values

The component reports five independently usable logical groups. The BME280
group covers one coherent enclosure measurement and is valid only when its
temperature, pressure and humidity outputs are all available:

```c
typedef enum {
    NODE_SENSOR_SOIL_0_VALID        = 1U << 0,
    NODE_SENSOR_SOIL_1_VALID        = 1U << 1,
    NODE_SENSOR_SOIL_TEMP_0_VALID   = 1U << 2,
    NODE_SENSOR_SOIL_TEMP_1_VALID   = 1U << 3,
    NODE_SENSOR_ENCLOSURE_ENV_VALID = 1U << 4,
} node_sensor_validity_t;

typedef struct {
    uint16_t soil_0_mv;
    uint16_t soil_1_mv;
    int16_t soil_temp_0_centi_c;
    int16_t soil_temp_1_centi_c;
    int16_t enclosure_centi_c;
    uint32_t enclosure_pressure_pa;
    uint16_t enclosure_humidity_centi_pct;
    uint8_t validity;
} node_sensor_sample_t;
```

The structure is an in-memory API value and is never serialized directly.
Channel numbers identify stable connectors or configured identities, not soil
depth. `BME280`/`ENV` is used instead of `BMP`: the pilot enclosure sensor also
measures humidity.

`out_sample` is always zeroed before hardware access. A failed group remains
zero with its validity bit clear; zero is also a permitted valid measurement,
so the bit is authoritative. When `NODE_SENSOR_ENCLOSURE_ENV_VALID` is clear,
all three enclosure fields are zero. `node_core` maps it to all three protocol
flags `ENCLOSURE_TEMP_VALID`, `ENCLOSURE_PRESSURE_VALID` and
`ENCLOSURE_HUMIDITY_VALID`, which are therefore always equal in node-generated
pilot readings.

A set validity bit means that acquisition and structural conversion succeeded;
it does not certify physical or agronomic plausibility. Representable outliers
remain valid pilot data for downstream anomaly analysis.

### `node_sensors_sample_all`

Implemented shape:

```text
err_curag_t node_sensors_sample_all(
    node_sensor_sample_t *out_sample,
    diagn_context_t *out_diag)
```

**Purpose**

Collect the five logical sensor groups while preserving every successful group
when another one fails.

**Inputs and ownership**

`out_sample` is required and caller-owned. `out_diag` is optional. Neither
pointer is retained. Both non-null outputs are cleared on entry.

**Outputs and side effects**

The operation lazily initializes the required private buses, enables the shared
soil/DS18B20 rail, waits for its configured stabilization time, samples both
soil channels and both configured DS18B20 identities, and disables the shared
rail through one cleanup path before sampling the independent BME280. The
BME280 remains on its always-powered rail. Bounded operation and low-power
recovery of the current BME280 driver are known deferred work rather than a
guarantee of this pilot interface.

One sensor-group failure does not suppress attempts for independent groups.
The shared rail is disabled before return on every path after it may have been
enabled. A nonzero result does not invalidate fields whose validity bits are
set.

**Failure results**

| Condition | Error |
|---|---|
| Null `out_sample` | `CURAG_ESENSORS_EINVALID_ARGUMENT` |
| At least one but not all logical groups invalid | `CURAG_ESENSORS_EPARTIAL_SAMPLE` |
| All five logical groups invalid | `CURAG_ESENSORS_ECOMPLETE_SAMPLE` |
| Shared rail enable or disable cannot be established | `CURAG_ESENSORS_EPOWER_CONTROL` |
| Private sensor resource cleanup fails after acquisition | `CURAG_ESENSORS_ECLEANUP` |

Power-control failure takes precedence over sampling-summary errors because it
may affect the node's energy budget. Cleanup failure takes precedence over
complete/partial sampling but never clears an already valid field. Otherwise
the return describes complete versus partial sampling. The top-level
diagnostic operation describes the selected primary failure rather than every
populated context pair. Its precedence is `POWER_OFF`, `POWER_ON`, `CLEANUP`,
`VALIDATE`, `INITIALIZE`, then `READ`.

**`node_core` response**

Record a nonzero result and its component-produced diagnostic best-effort, but
still construct the current reading from every valid group. Map the enclosure
validity bit to all three enclosure protocol flags. Final wake cleanup still
calls `node_sensors_force_power_off` exactly once.

### `node_sensors_force_power_off`

Implemented shape:

```text
err_curag_t node_sensors_force_power_off(diagn_context_t *out_diag)
```

**Purpose**

Best-effort enforcement of the shared soil/DS18B20 rail's off state.

**Inputs and ownership**

`out_diag` is optional, is cleared on entry and is never retained.

**Outputs and side effects**

The operation is idempotent, never enables the rail and does not initialize
ADC, I2C, 1-Wire or any sensor driver. If the component never touched the gate
this wake, the board's hardware-default-off design makes this a successful
no-op. Otherwise it releases the open-drain gate control and returns the GPIO
to floating input mode with both internal pulls disabled. It does not use GPIO
hold, power-cycle or initialize the always-powered BME280.

Failure to enforce the off state returns `CURAG_ESENSORS_EPOWER_CONTROL` with
operation `POWER_OFF` and the component status pair populated.

**`node_core` response**

Append the returned diagnostic best-effort when persistence permits, then
continue radio and persistence cleanup and enter ESP32 deep sleep regardless
of the result.

### Sensor error domain

The third error domain is `CURAG_EDOM_SENSORS = 3`:

| Value | Error code | Meaning |
|---:|---|---|
| `0` | `NONE` | Success only |
| `1` | `CURAG_ESENSORS_EINVALID_ARGUMENT` | Caller violated an input invariant |
| `2` | `CURAG_ESENSORS_EPARTIAL_SAMPLE` | One or more, but not all, sensor groups are invalid |
| `3` | `CURAG_ESENSORS_ECOMPLETE_SAMPLE` | All five sensor groups are invalid |
| `4` | `CURAG_ESENSORS_EPOWER_CONTROL` | The shared gated rail could not be enabled or disabled reliably |
| `5` | `CURAG_ESENSORS_ECLEANUP` | Acquisition results may be valid, but a private sensor resource could not be released cleanly |

These codes summarize component behavior; they do not replace exact backend
statuses in the context.

### Sensor context V1

`CURAG_SENSOR_CONTEXT_V1 = 1` within the sensor domain has a fixed 48-byte
little-endian encoding. It consists of six pairs in this exact order:

| Offset | Pair | Meaning |
|---:|---|---|
| `0` | `component` | Shared initialization, power-gate or cleanup failure |
| `8` | `soil_0` | Soil channel 0 acquisition |
| `16` | `soil_1` | Soil channel 1 acquisition |
| `24` | `soil_temp_0` | Configured DS18B20 channel 0 acquisition |
| `32` | `soil_temp_1` | Configured DS18B20 channel 1 acquisition |
| `40` | `enclosure_env` | Atomic BME280 acquisition and compensation |

Each pair is:

| Pair offset | Field | Encoding | Meaning |
|---:|---|---:|---|
| `0` | `backend_status_kind` | `u32` | Interpretation of the following status |
| `4` | `backend_status` | `i32` | Exact normalized backend or internal result |

Backend-status kinds are:

| Value | Kind |
|---:|---|
| `0` | `NONE`; requires status zero |
| `1` | `ESP_ERR` |
| `2` | `DRIVER_STATUS`; the fixed pair position selects the driver |
| `3` | `NODE_SENSORS_INTERNAL` |

The initial component-internal status assignments are:

| Value | Status |
|---:|---|
| `0` | `NONE` |
| `1` | `BLOCKED_BY_SHARED_FAILURE` |
| `2` | `UNPROVISIONED_IDENTITY`; the configured DS18B20 ROM is all zeroes or invalid |
| `3` | `DUPLICATE_IDENTITY`; both configured nonzero DS18B20 ROM identities are equal |

A successful group uses `(NONE, 0)`. A direct failure stores the exact returned
status and its kind. A group not attempted because a shared prerequisite
failed uses `(NODE_SENSORS_INTERNAL, BLOCKED_BY_SHARED_FAILURE)`; the component
pair stores the selected underlying shared failure. Multiple independent group
pairs are retained. Because there is one component pair, a later
higher-precedence cleanup or power failure may replace an earlier exact shared
status; blocked group pairs preserve the fact that the earlier shared failure
occurred. A power-off or cleanup failure never clears already valid readings.

Every nonzero sensor result with non-null `out_diag` sets the selected primary
operation and this exact context. `CURAG_OP_CLEANUP = 19` is the stable operation
for private resource release failure. Successful calls leave the diagnostic
output empty. Implementations explicitly encode the 48 context bytes; they do
not copy a native C structure with compiler padding.

### Pilot board configuration

`node_sensors_sample_all` takes no GPIO parameters. Fixed board wiring is
selected at build time through `CONFIG_CURA_*`; `node_core` never owns it. The
initial ESP32-C6 defaults are provisional: soil ADC channels 0/1 use GPIO 0/1,
the active-low open-drain gate uses GPIO 2, 1-Wire uses GPIO 3, and BME280
SDA/SCL use GPIO 4/5. The BME280 address is fixed at `0x76`, I2C runs at 100
kHz, soil ADC attenuation is 12 dB, and rail stabilization starts at 200 ms.

The high-side P-MOSFET source connects to 3.3 V, its drain to the switched
sensor rail and its gate to 3.3 V through 47 kOhm. The GPIO pulls the gate low
only while the rail must be on. Before enabling open-drain output, firmware
loads level 1 so configuration cannot produce an unintended low pulse. To turn
the rail off it writes level 1, which means high impedance in open-drain mode,
then selects floating input mode. Every power-on reconfigures the output after
that transition; no cached output-mode assumption is permitted.

The two DS18B20 ROM strings default to all zeroes, which deliberately makes
those groups invalid until identities are provisioned. Enumeration order never
assigns logical channels. Equal nonzero configured identities are rejected as a
provisioning error before bus access and invalidate both temperature groups. An
external 1-Wire pull-up must be connected to the switched sensor rail; the
backend does not enable the ESP32 internal pull-up.

The current Espressif BME280 driver remains a deferred risk. The eventual
choice is a corrected and immutable pinned fork with upstream contributions or
replacement after evaluating another driver, initially Bosch's official
SensorAPI. Generated `managed_components` are not edited in place.

The public compatibility function
`soil_sensor_read_mv(int gpio_num, uint16_t *out_mv)` remains in the
`node_sensors` component for maintenance applications. It assumes its probe is
already powered and never changes the gate.

## `sx1262_radio`

Status: implemented pilot contract. The public declarations are in
`components/sx1262_radio/include/sx1262_radio.h`; the platform-independent
policy links either the ESP-IDF backend or the deterministic host fake.

### Public values

The receive result reserves the complete SX1262 payload range so a PHY-valid
but application-invalid packet cannot overflow a protocol-sized caller buffer:

```c
#define SX1262_RADIO_MAX_PAYLOAD_SIZE 255U

typedef struct {
    bool tx_started;
    bool tx_done;
    uint64_t set_tx_at_us;
    uint64_t tx_done_at_us;
} sx1262_radio_tx_result_t;

typedef enum {
    SX1262_RADIO_RX_INVALID = 0,
    SX1262_RADIO_RX_PACKET = 1,
    SX1262_RADIO_RX_DEADLINE = 2,
} sx1262_radio_rx_outcome_t;

typedef struct {
    sx1262_radio_rx_outcome_t outcome;
    uint64_t rx_done_at_us;
    int16_t rssi_dbm_x2;
    int16_t snr_db_x4;
    uint8_t payload_length;
    uint8_t payload[SX1262_RADIO_MAX_PAYLOAD_SIZE];
} sx1262_radio_rx_result_t;
```

These structures are in-memory API values, not serialized layouts. RSSI is
signed dBm multiplied by two and SNR is signed dB multiplied by four, matching
the SX1262 packet-status resolution without floating point. Packet metadata is
valid only for `SX1262_RADIO_RX_PACKET`; every field is zero for a deadline.

All operations are blocking, single-threaded and non-reentrant. Output
structures and a non-null `out_diag` are cleared on entry. Input pointers are
borrowed only for the call and are never retained.

`deadline_monotonic_us` is an absolute time, not a duration. It uses the same
unsigned 64-bit monotonic-microsecond domain as the controller's
`monotonic_us`. Reusing the same deadline after an invalid ACK cannot extend the
RX window. Production `node_core` and radio clock adapters must use the same
clock source.

### `sx1262_radio_transmit_uplink`

Interface:

```text
err_curag_t sx1262_radio_transmit_uplink(
    const u8 *payload,
    size_t payload_length,
    u64 deadline_monotonic_us,
    sx1262_radio_tx_result_t *out_result,
    diagn_context_t *out_diag)
```

**Purpose**

Transmit one caller-constructed LoRa uplink payload and report precisely how
far the physical attempt progressed.

**Inputs and ownership**

`payload` is non-null borrowed memory and `payload_length` is from 1 through
255. `out_result` is non-null caller memory. `out_diag` is optional. The
deadline is the latest monotonic time at which the operation may continue
waiting for completion; the component also applies its private SX1262 command
timeout no later than that bound.

**Outputs**

On success, both booleans are true. `set_tx_at_us` is sampled immediately before
issuing the `SetTx` command and becomes valid only if that command succeeds.
`tx_done_at_us` is the captured `TX_DONE` IRQ time. Required invariants are:

```text
tx_done -> tx_started
!tx_started -> set_tx_at_us == 0 && tx_done_at_us == 0
tx_started && !tx_done -> set_tx_at_us != 0 && tx_done_at_us == 0
tx_done -> tx_done_at_us >= set_tx_at_us
```

A nonzero result may therefore still report `tx_started = true`, allowing
`node_core` to count and charge an attempt whose later IRQ or cleanup failed.

**Side effects**

The first transmit performs cached lazy initialization and applies the complete
pilot profile. Each call leaves continuous RX through `STDBY_RC` when needed,
selects normal IQ, clears stale IRQs, writes the payload and issues `SetTx`.
It performs no encryption, retry, ACK validation or airtime-policy accounting.

**Failure classes**

- Invalid input or expired deadline before `SetTx`: no transmission starts.
- Cached/initial lazy-initialization failure: no transmission starts.
- GPIO, SPI, BUSY, command-status or pre-`SetTx` failure: no transmission
  starts.
- Deadline, IRQ, device-status or cleanup failure after successful `SetTx`:
  `tx_started` remains true; `tx_done` reflects whether the IRQ was captured.

**`node_core` response**

Persist the error/context best-effort and end the delivery operation. Count and
charge the attempt exactly when `tx_started` is true. The controller determines
from its global limits whether a deadline result maps to
`RADIO_CYCLE_DEADLINE`; all other local failures map to `LOCAL_RADIO_ERROR`.
Do not retry a local failure during the same wake.

### `sx1262_radio_receive_downlink_until`

Interface:

```text
err_curag_t sx1262_radio_receive_downlink_until(
    u64 deadline_monotonic_us,
    sx1262_radio_rx_result_t *out_result,
    diagn_context_t *out_diag)
```

**Purpose**

Keep inverted-IQ continuous RX open until the first PHY-valid packet completes
at or before the caller's absolute deadline.

**Inputs and ownership**

`out_result` is non-null caller memory and `out_diag` is optional. The component
must already be `READY` after a successful transmit; receive never initializes
the radio independently.

**Outputs**

`SX1262_RADIO_RX_PACKET` returns the complete SX1262 payload, its `RX_DONE`
timestamp, RSSI and SNR with `err_curag_t == 0`.
`SX1262_RADIO_RX_DEADLINE` is normal silence, also with `err_curag_t == 0`.
An `RX_DONE` timestamp at or before the deadline wins even if task scheduling
delays later processing. A packet completed after the deadline does not win.

**Side effects**

The operation selects inverted IQ and enters or continues continuous RX. PHY
header and payload-CRC failures are discarded internally while RX remains
open. A returned application packet is not parsed or authenticated. After
`node_core` rejects an application-invalid packet, it calls this operation
again with the unchanged absolute deadline. The radio remains available for a
normal-IQ retransmission without being put to sleep between attempts.

**Failure classes**

- Null output.
- `UNTOUCHED`, `FAILED` or `SLEEPING` component state.
- GPIO, SPI, BUSY, command-status, IRQ, buffer-read, packet-status or SX1262
  device failure.

**`node_core` response**

For `RX_PACKET`, validate the complete ACK in `node_core`; an invalid ACK is
diagnosed and RX resumes under the same deadline. For `RX_DEADLINE`, apply the
normal silence/retry policy. For a nonzero local error, diagnose it, retain the
reading and end radio work for the wake.

### `sx1262_radio_sleep`

Interface:

```text
err_curag_t sx1262_radio_sleep(diagn_context_t *out_diag)
```

**Purpose**

Best-effort final cleanup followed by SX1262 cold-start sleep.

**Inputs and ownership**

`out_diag` is optional and is never retained.

**Outputs and side effects**

- `UNTOUCHED`: return success without hardware access or lazy initialization.
- `READY`, or `FAILED` after hardware was touched: stop RX/other active mode
  through `SetStandby(STDBY_RC)`, then issue `SetSleep(COLD_START)` when it is
  safe to issue another command.
- `FAILED` before hardware was touched: return success without hardware access.
- `SLEEPING`: return success as an idempotent no-op.

Successful cleanup moves to `SLEEPING`. Failed cleanup remains `FAILED` so a
second best-effort call may retry. TX and RX calls in `SLEEPING` fail with
invalid state for the remainder of the wake. A standby or sleep failure is
returned even when a later cleanup step succeeds.

**`node_core` response**

Persist the returned diagnostic best-effort when possible, then continue
unconditionally to ESP32 deep sleep.

### Radio error domain

The second error domain is `CURAG_EDOM_RADIO = 2`. Its error-code values are
scoped by that domain:

| Value | Error code | Meaning |
|---:|---|---|
| `0` | `NONE` | Success only |
| `1` | `CURAG_ERADIO_EINVALID_ARGUMENT` | Caller violated an input invariant |
| `2` | `CURAG_ERADIO_EINVALID_STATE` | Operation is not valid in the singleton's current state |
| `3` | `CURAG_ERADIO_EIO` | GPIO, SPI, IRQ or other backend primitive failed |
| `4` | `CURAG_ERADIO_EBUSY_TIMEOUT` | SX1262 BUSY did not deassert within the command bound |
| `5` | `CURAG_ERADIO_ECOMMAND_STATUS` | SX1262 command status reported failure |
| `6` | `CURAG_ERADIO_EDEADLINE` | The absolute operation deadline was reached |
| `7` | `CURAG_ERADIO_EUNEXPECTED_IRQ` | IRQ status cannot represent a valid outcome for the active operation |
| `8` | `CURAG_ERADIO_EDEVICE_ERROR` | SX1262 device-error bits report an oscillator, PLL, calibration or PA fault |

Initialization is an operation, not a separate reason: an SPI failure while
initializing returns `CURAG_ERADIO_EIO` with operation `INITIALIZE` and the
precise context stage. An RX deadline is a successful outcome and never returns
`CURAG_ERADIO_EDEADLINE`; the deadline error is for TX or an internal operation
that could not complete normally.

### Radio context V1

The schema is `CURAG_RADIO_CONTEXT_V1 = 1` within the radio domain and has a
fixed 14-byte little-endian encoding:

| Field | Encoding | Meaning |
|---|---:|---|
| `state` | `u8` | Singleton state when failure was observed |
| `command_opcode` | `u8` | Raw SX1262 command opcode, or zero when none applies |
| `stage` | `u8` | Stable radio failure stage |
| `flags` | `u8` | Validity and hardware-touch bitmap |
| `backend_status_kind` | `u8` | Encoding of `backend_status` |
| `backend_status` | `i32` | Exact normalized backend result |
| `chip_status` | `u8` | Raw SX1262 status byte when valid |
| `irq_status` | `u16` | Raw SX1262 IRQ bits when valid |
| `device_errors` | `u16` | Raw SX1262 device-error bits when valid |

State values are:

| Value | State |
|---:|---|
| `0` | `UNTOUCHED` |
| `1` | `READY` |
| `2` | `FAILED` |
| `3` | `SLEEPING` |

Flag bits are:

| Bit | Flag |
|---:|---|
| `0` | `CHIP_STATUS_VALID` |
| `1` | `IRQ_STATUS_VALID` |
| `2` | `DEVICE_ERRORS_VALID` |
| `3` | `HARDWARE_TOUCHED` |
| `4`-`7` | Reserved; must be zero |

Backend-status kinds are:

| Value | Kind |
|---:|---|
| `0` | `NO_ERROR`; requires status zero |
| `1` | `ESP_ERR` |
| `2` | `SX1262_DRIVER_STATUS` |

Stable stage values are:

| Value | Stage |
|---:|---|
| `0` | `NONE` |
| `1` | `STATE_CHECK` |
| `2` | `VALIDATE_INPUT` |
| `3` | `CONFIGURE_GPIO` |
| `4` | `CONFIGURE_SPI` |
| `5` | `RESET` |
| `6` | `WAKE` |
| `7` | `WAIT_BUSY` |
| `8` | `WRITE_COMMAND` |
| `9` | `READ_COMMAND` |
| `10` | `CONFIGURE_IRQ` |
| `11` | `WAIT_IRQ` |
| `12` | `READ_IRQ` |
| `13` | `CLEAR_IRQ` |
| `14` | `WRITE_BUFFER` |
| `15` | `READ_BUFFER` |
| `16` | `READ_PACKET_STATUS` |
| `17` | `DETACH_IRQ` |
| `18` | `CAPTURE_TIME` |

Every nonzero radio result with a non-null diagnostic output uses shared
operation `VALIDATE`, `INITIALIZE`, `TRANSMIT`, `RECEIVE` or `SLEEP` plus this
context. Fields whose validity flag is clear must be zero. A semantic failure
uses `NO_ERROR`; a backend failure carries its exact status. The context stores
raw SX1262 status/IRQ/device values rather than copying a private driver object.

### Singleton construction and private seam

There is no public `sx1262_radio_t`, setter, getter or `init` function. The
component owns file-static state whose all-zero BSS representation is
`UNTOUCHED`. The private state contains at least:

- the state enum and `hardware_touched` marker;
- cached lazy-initialization error and diagnostic context;
- active operation/IRQ state and captured IRQ timestamps; and
- any private driver/HAL state required by the selected implementation.

Production links the component to a private ESP-IDF backend providing SPI,
GPIO, RESET, BUSY, DIO1 IRQ and the same monotonic clock source used by
`node_core`. Host component tests will link the same component to a fake
backend. Board pins, RF-switch control, oscillator/TCXO settings, regulator mode
and the fixed pilot PHY profile are compile-time board configuration, not
controller inputs.

`UNTOUCHED -> READY` occurs only after complete lazy initialization during the
first transmit. A hardware initialization or active radio-operation failure
moves to `FAILED`; input validation and a deadline reached before any radio
operation leave the current state unchanged. Subsequent calls return the cached
initialization failure where applicable. Successful final cleanup moves to
`SLEEPING`. Ordinary BSS is recreated on the next ESP32 wake, restoring
`UNTOUCHED` without a public reset operation.

The platform-independent policy calls a private backend interface for radio
mechanics. Host tests link a deterministic fake implementation and run each
scenario in a fresh process, so ordinary BSS restores `UNTOUCHED` without a
test-only singleton setter, getter, snapshot or reset function. None of those
operations is part of the public interface.

### Selected radio implementation

- The pilot module is the Waveshare Pico-LoRa-SX1262-868M. Semtech's
  Clear-BSD `sx126x_driver` v2.5.0 is vendored at a pinned upstream commit and
  used unchanged below the Cura backend adapter.
- The ESP32-C6 uses SPI2 at 8 MHz. Provisional SCLK, MOSI, MISO, CS, RESET,
  BUSY and DIO1 pins are component Kconfig values and must be revised against
  the assembled board. DIO1 uses a rising-edge GPIO interrupt.
- The module's onboard DIO2-controlled RF switch is enabled. Its DIO3-powered
  TCXO is configured for 1.7 V and a 5 ms startup. The regulator uses DC-DC
  mode.
- `esp_timer_get_time()` supplies both ordinary monotonic reads and the DIO1
  ISR timestamp. A component-owned static binary semaphore wakes the blocking
  caller; the component does not consume a task-notification slot.
- BUSY waits are bounded to 10 ms, with 20 ms allowed after hardware reset.
  The SX1262 TX watchdog is programmed to expire 5 ms before the caller's
  absolute deadline when enough time remains. The software deadline remains
  authoritative.
- The +14 dBm pilot output uses the SX1262 optimal PA row from datasheet table
  13-21: `paDutyCycle=0x02`, `hpMax=0x02`, `deviceSel=0x00`,
  `paLut=0x01`, followed by `SetTxParams(+22 dBm, 40 us)`. In this PA mode the
  requested radiated output is +14 dBm; `+22` is the required register value,
  not the claimed RF output.
- ESP-IDF failures retain their exact `esp_err_t` with backend kind `ESP_ERR`.
  Failures returned directly by the Semtech command layer use
  `SX1262_DRIVER_STATUS`. Semantic state, argument, unexpected-IRQ and
  deadline failures use `NO_ERROR`.
- After every non-sleep Semtech driver operation, the backend issues
  `GetStatus`. Command timeout, processing-error and execution-failure states
  return `CURAG_ERADIO_ECOMMAND_STATUS` and preserve the reconstructed raw
  chip-status byte in diagnostic context. `SetSleep` is excluded because the
  sleeping device cannot be queried afterward.

## Platform ports

Clock, retry randomness and terminal system operations are small injected ports
rather than stateful components. They have no `err_curag_t` or diagnostic
context: the clock, random source and reset-reason query are infallible
contracts, while deep-sleep failure is contained by the production system
adapter after ordinary firmware diagnostics have already been synchronized.

```c
typedef struct {
    void *context;
    uint64_t (*monotonic_us)(void *context);
} node_clock_port_t;

typedef struct {
    void *context;
    uint32_t (*uniform_u32_inclusive)(
        void *context,
        uint32_t minimum,
        uint32_t maximum);
} node_randomness_port_t;

typedef struct {
    void *context;
    uint8_t (*get_reset_reason)(void *context);
    void (*enter_deep_sleep_for)(
        void *context,
        uint64_t duration_us);
} node_system_port_t;

typedef struct {
    node_clock_port_t clock;
    node_randomness_port_t randomness;
    node_system_port_t system;
} node_platform_ports_t;
```

`node_core` borrows a `const node_platform_ports_t *` for one complete wake
invocation. Every callback is required; each context may be null and may be
mutated by its callbacks. The port table and non-null contexts remain valid
until `node_core` terminates. No callback or controller operation retains them
beyond that invocation. A missing callback or invalid caller argument is a
composition/programming error covered by host tests, not a recoverable field
condition.

### `monotonic_us`

```text
uint64_t monotonic_us(void *context)
```

The result is microseconds in an unspecified epoch. Values are nondecreasing
throughout one wake; consecutive calls may be equal. The clock includes time
spent in component calls and blocking operations and cannot fail. Its epoch
need not survive deep sleep because every comparison and subtraction is
between values from the current wake.

The concrete adapter and the private SX1262 backend use the same underlying
clock. The production ESP-IDF adapter uses `esp_timer_get_time()` with an
explicit nonnegative `u64` conversion. A backwards value within one wake is an
interface violation, not a runtime condition that `node_core` attempts to
repair. A host fake may return scripted values or expose an explicitly advanced
clock without real waiting.

This one operation supplies application offsets, `run_ms`, absolute radio
deadlines, retry scheduling, transmission timing and total awake duration. No
UTC or delay operation is part of the clock port.

### `uniform_u32_inclusive`

```text
uint32_t uniform_u32_inclusive(
    void *context,
    uint32_t minimum,
    uint32_t maximum)
```

`minimum <= maximum` is a caller precondition. The result is uniformly selected
from the inclusive interval. Equal bounds return that value. The implementation
uses rejection sampling or an equivalent unbiased mapping, calculates interval
width without `u32` overflow and supports the complete `[0, UINT32_MAX]`
interval. It cannot fail.

The pilot retry calculation is:

```text
random_us  = uniform_u32_inclusive(context, 100000, 500000)
retry_at   = tx_done_at_us + 500000 + random_us
```

This source does not need cryptographic guarantees and must not be used for
node identities, provisioning keys or any future cryptographic random value.
Host fakes may return scripted in-range values while recording the exact bounds
and call count.

### `get_reset_reason`

```text
uint8_t get_reset_reason(void *context)
```

This operation is called once near application start and cannot fail. The
production adapter wraps `esp_reset_reason()`, not
`esp_sleep_get_wakeup_cause()`: the protocol carries the reset reason and does
not carry a sleep wakeup source.

The adapter explicitly converts the numeric `esp_reset_reason_t` value to
`u8`. Assigned values keep their protocol-defined numbers and unassigned values
16 through 255 are retained. A negative or greater-than-255 platform result is
normalized to `ESP_RST_UNKNOWN` (`0`) rather than truncated. `node_core` sets
`DEEP_SLEEP_BOOT` and accepts incoming RTC metrics if and only if the result is
`ESP_RST_DEEPSLEEP` (`8`) and all other RTC invariants hold.

### `enter_deep_sleep_for`

```text
void enter_deep_sleep_for(
    void *context,
    uint64_t duration_us)
```

`duration_us` is a nonzero relative duration, not an absolute timestamp. The
pilot passes `900000000` microseconds. Sensor power, radio sleep, persistence
synchronization and RTC commit have completed before this terminal operation is
called.

From `node_core`'s perspective the operation cannot fail and production never
returns. The production implementation:

1. configures timer wakeup with `esp_sleep_enable_timer_wakeup(duration_us)`;
2. on success, calls the non-returning `esp_deep_sleep_start()`; and
3. on configuration failure, emits a development-console error, waits 60
   seconds without a tight busy-spin, and calls `esp_restart()`.

The failure path is deliberately contained here. It does not reopen
persistence or attempt a late persistent diagnostic after `sync_all`. Both the
sleep and restart paths are terminal; if the platform restart primitive were
unexpectedly to return, the adapter must abort or otherwise remain in a
non-returning fatal path.

The production concrete function may be declared `_Noreturn`, but the injected
function-pointer type is not. A host fake records the requested duration and
may return. `node_core` treats the invocation as terminal in either case: if a
test fake or broken adapter returns, it immediately returns to its caller and
must invoke no clock, randomness, component or system operation afterward.

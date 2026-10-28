"""Independent service.reading_delivery reconciliation of a consistent SQLite/node capture.

Sensor acquisition correctness is a separate sensor-carrier prerequisite.
No production peer policy or generated reading decoder supplies expected values.
"""
from collections import Counter
import sqlite3
import struct

from reading_baseline import new_readings

FIELDS = ("sample_id run_ms soil_0_mv soil_1_mv soil_temp_0_centi_c "
          "soil_temp_1_centi_c enclosure_centi_c enclosure_pressure_pa "
          "enclosure_humidity_centi_pct reset_reason previous_current_tx_attempts "
          "previous_awake_ms previous_current_delivery_ms previous_cycle_tx_attempts "
          "previous_cycle_accepted_readings flags").split()
BODY = struct.Struct("<IHHHhhhIHBBHHBBH")


def verify_service(database, node_id, decoded, instance_id, baseline=None):
    """Require two complete accelerated wakes of one isolated service instance.

    SQL supplies decoded authority and per-reception outcomes; node delivery logs
    confirm acceptance/attempt counts. Normal frames and ACK bytes are not stored,
    so this check cannot independently decrypt or compare those wire bytes.
    """
    with sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True) as db:
        db.row_factory = sqlite3.Row
        if db.execute("PRAGMA integrity_check").fetchall()[0][0] != "ok" or db.execute("PRAGMA foreign_key_check").fetchall():
            raise ValueError("inconsistent database snapshot")
        lifecycle = db.execute("SELECT * FROM receiver_instances WHERE receiver_instance_id=?", (instance_id,)).fetchall()
        if len(lifecycle) != 1 or lifecycle[0]["clean_stopped_at_monotonic_us"] is None:
            raise ValueError("missing clean service completion")
        profiles = [dict(r) for r in db.execute(
            "SELECT * FROM packet_receptions WHERE receiver_instance_id=? ORDER BY occurrence_sequence", (instance_id,))]
        readings = [dict(r) for r in db.execute(
            "SELECT * FROM reading_messages WHERE node_id=? ORDER BY message_id", (node_id,))]
        transports = {r["message_id"]: dict(r) for r in db.execute(
            "SELECT * FROM received_messages WHERE node_id=?", (node_id,))}
        exceptions = db.execute(
            "SELECT count(*) FROM packet_reception_exceptions WHERE receiver_instance_id=?", (instance_id,)).fetchone()[0]
    if exceptions:
        raise ValueError("unexpected reception exception")
    if baseline is not None:
        readings = new_readings(readings, baseline, node_id)
    if len(readings) != 2 or not profiles:
        raise ValueError("missing/extra canonical reading")
    counts, observed, order = Counter(), {}, []
    by_message = {r["message_id"]: r for r in readings}
    for p in profiles:
        message = p["message_id"]
        reading = by_message.get(message)
        transport = transports.get(message)
        if reading is None or transport is None or transport["domain"] != 1:
            raise ValueError("reception has no matching current reading authority")
        body = {name: reading[name] for name in FIELDS}
        if (p["domain"], p["node_id"], p["decoded_sample_id"],
            p["processing_result_id"], p["ack_tx_result_id"]) != (
                1, node_id, body["sample_id"], 11, 5):
            raise ValueError("reception outcome/identity mismatch")
        timestamps = [p[name] for name in (
            "received_at_monotonic_us", "t1_handler_started_monotonic_us",
            "t2_packet_copied_monotonic_us", "t3_authentication_completed_monotonic_us",
            "t4_set_tx_attempted_monotonic_us", "t5_tx_done_monotonic_us", "t6_set_rx_issued_monotonic_us")]
        if any(type(t) is not int for t in timestamps) or timestamps != sorted(timestamps):
            raise ValueError("incomplete/nonchronological successful profile")
        if p["persistence_classification_id"] != (2 if counts[message] else 1):
            raise ValueError("incorrect first-seen/retry classification")
        if message in observed:
            if order[-1] != message:
                raise ValueError("changed or cross-wake retry")
        else:
            if order and message <= order[-1]:
                raise ValueError("message counter regression")
            order.append(message)
            observed[message] = dict(body=body, profile=p)
        counts[message] += 1
    if len(order) != 2 or [r["message_id"] for r in readings] != order:
        raise ValueError("missing/extra production wake")
    first, second = [observed[m] for m in order]
    elapsed = second["profile"]["received_at_monotonic_us"] - first["profile"]["received_at_monotonic_us"]
    if not 9_500_000 <= elapsed <= 45_500_000:
        raise ValueError("accelerated 10-second wake not observed")
    if second["body"]["sample_id"] != first["body"]["sample_id"] + 1:
        raise ValueError("sample counter discontinuity")
    for r in readings:
        e = observed[r["message_id"]]
        if r["is_canonical"] != 1:
            raise ValueError("reading is not canonical")
        b = e["body"]
        if b["flags"] & 0x00fe != 0x00fe or not all(2000 <= b[n] <= 2700 for n in ("soil_0_mv", "soil_1_mv")):
            raise ValueError("nominal sensor flags/ranges not satisfied")
    b = second["body"]
    if (b["reset_reason"] != 8 or b["flags"] & 0x301 != 0x301 or
            b["previous_current_tx_attempts"] != counts[order[0]] or
            b["previous_cycle_tx_attempts"] != counts[order[0]] or
            b["previous_cycle_accepted_readings"] != 1 or
            not 0 < b["previous_current_delivery_ms"] <= b["previous_awake_ms"]):
        raise ValueError("missing/inconsistent previous-wake metrics")
    logs = decoded["logs"]
    if logs["pending.log"] != [] or logs["quarantine.log"] or logs["diagnostic.log"]:
        raise ValueError("unexpected pending/quarantine/diagnostic state")
    deliveries = logs["delivery.log"]
    if deliveries is None or len(deliveries) != 4:
        raise ValueError("missing durable node delivery boundaries")
    for index, m in enumerate(order):
        sample = observed[m]["body"]["sample_id"]
        start, finish = deliveries[2 * index:2 * index + 2]
        for record, kind in ((start, 4), (finish, 5)):
            if (record["type"], record["cycle_sample_id"], record["sample_id"], record["message_id"], record["domain"]) != (kind, sample, sample, m, 1):
                raise ValueError("node delivery identity mismatch")
        if (finish["final_result"], finish["attempt_count"]) != (1, counts[m]):
            raise ValueError("node outcome/attempts differ from receiver evidence")
    return dict(case="service.reading_delivery", status="PASS", scope="production_node_receiver_service", sleep_seconds=10, cadence_scope="accelerated functional RF",
                wakes=2, profiles=len(profiles), inter_wake_us=elapsed,
                sensor_conversion="separate sensor-carrier prerequisite",
                evidence_scope="decoded SQL authority and node delivery outcomes; no retained normal wire bytes")

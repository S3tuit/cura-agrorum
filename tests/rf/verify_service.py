"""Independent RF-020 reconciliation of a consistent SQLite/node capture.

Sensor acquisition correctness is a separate sensor-carrier prerequisite.
No production peer policy or generated reading decoder supplies expected values.
"""
from collections import Counter
import sqlite3
import struct

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
from reading_baseline import new_readings

FIELDS = ("sample_id run_ms soil_0_mv soil_1_mv soil_temp_0_centi_c "
          "soil_temp_1_centi_c enclosure_centi_c enclosure_pressure_pa "
          "enclosure_humidity_centi_pct reset_reason previous_current_tx_attempts "
          "previous_awake_ms previous_current_delivery_ms previous_cycle_tx_attempts "
          "previous_cycle_accepted_readings flags").split()
BODY = struct.Struct("<IHHHhhhIHBBHHBBH")


def opened(frame, key, node_id, domain, length):
    if len(frame) != length:
        raise ValueError("wrong frame length")
    control, actual_domain, actual_node, message = struct.unpack("<BB8sI", frame[:14])
    if (control, actual_domain, actual_node) != (32, domain, node_id):
        raise ValueError("wrong authenticated frame identity/domain")
    nonce = struct.pack("<8sIB", node_id, message, domain)
    return message, AESCCM(key, tag_length=8).decrypt(nonce, frame[14:], frame[:14])


def verify_service(database, node_id, key, decoded, instance_id, baseline=None):
    """Require two complete accelerated wakes of one isolated service instance."""
    with sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True) as db:
        db.row_factory = sqlite3.Row
        if db.execute("PRAGMA integrity_check").fetchall()[0][0] != "ok" or db.execute("PRAGMA foreign_key_check").fetchall():
            raise ValueError("inconsistent database snapshot")
        lifecycle = db.execute("SELECT * FROM receiver_instances WHERE receiver_instance_id=?", (instance_id,)).fetchall()
        if len(lifecycle) != 1 or lifecycle[0]["clean_stopped_at_monotonic_us"] is None:
            raise ValueError("missing clean service completion")
        profiles = [dict(r) for r in db.execute(
            "SELECT * FROM message_profiles WHERE receiver_instance_id=? ORDER BY occurrence_sequence", (instance_id,))]
        readings = [dict(r) for r in db.execute(
            "SELECT * FROM reading_messages WHERE node_id=? ORDER BY message_id", (node_id,))]
    if baseline is not None:
        readings = new_readings(readings, baseline, node_id)
    if len(readings) != 2 or not profiles:
        raise ValueError("missing/extra canonical reading")
    counts, observed, order = Counter(), {}, []
    for p in profiles:
        if p["received_frame_length"] != 54 or p["received_frame"] is None or len(p["received_frame"]) != 255:
            raise ValueError("missing exact received frame")
        frame = p["received_frame"][:54]
        if any(p["received_frame"][54:]):
            raise ValueError("noncanonical frame padding")
        message, raw = opened(frame, key, node_id, 1, 54)
        body = dict(zip(FIELDS, BODY.unpack(raw), strict=True))
        ack_message, status = opened(p["ack_frame"] or b"", key, node_id, 3, 23)
        if ack_message != message or status != b"\0":
            raise ValueError("ACK does not accept this message")
        if (p["claimed_control"], p["claimed_domain"], p["claimed_node_id"],
            p["claimed_message_id"], p["header_authenticated"], p["decoded_sample_id"],
            p["processing_result_id"], p["ack_selected_id"], p["ack_tx_result_id"]) != (
                32, 1, node_id, message, 1, body["sample_id"], 11, 3, 5):
            raise ValueError("profile classification/identity mismatch")
        timestamps = [p[name] for name in (
            "received_at_monotonic_us", "t1_handler_started_monotonic_us",
            "t2_packet_copied_monotonic_us", "t3_authentication_completed_monotonic_us",
            "t4_set_tx_attempted_monotonic_us", "t5_tx_done_monotonic_us", "t6_set_rx_issued_monotonic_us")]
        if any(type(t) is not int for t in timestamps) or timestamps != sorted(timestamps):
            raise ValueError("incomplete/nonchronological successful profile")
        if p["persistence_classification_id"] != (2 if counts[message] else 1):
            raise ValueError("incorrect first-seen/retry classification")
        if message in observed:
            if observed[message]["frame"] != frame or order[-1] != message:
                raise ValueError("changed or cross-wake retry")
        else:
            if order and message <= order[-1]:
                raise ValueError("message counter regression")
            order.append(message)
            observed[message] = dict(frame=frame, raw=raw, body=body, profile=p)
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
        if (r["reading_body"] != e["raw"] or r["is_canonical_for_sample"] != 1 or
                any(r[name] != e["body"][name] for name in FIELDS) or
                r["first_receiver_instance_id"] != instance_id or
                r["first_occurrence_sequence"] != e["profile"]["occurrence_sequence"]):
            raise ValueError("canonical persistence differs from authenticated reading")
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
    return dict(case="RF-020", status="PASS", scope="production_node_receiver_service", sleep_seconds=10, cadence_scope="accelerated functional RF",
                wakes=2, profiles=len(profiles), inter_wake_us=elapsed,
                sensor_conversion="separate sensor-carrier prerequisite")

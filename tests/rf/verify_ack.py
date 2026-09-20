"""RF-019 packet/storage reconciliation, separate from peer reply selection."""
from collections import Counter
import struct

from cryptography.hazmat.primitives.ciphers.aead import AESCCM

from cura_receiver.protocol_v2_lora_crypto import open_frame
from cura_receiver.generated.protocol_v2_lora_generated import decode_reading


def verify_ack_case(case, node_id, key, packets, decoded):
    scope, action = case.split(".")[1:]
    seed_count = 2 if scope == "backlog" else 1
    target_cycle = seed_count
    statuses = {"accepted": 1, "retry_later": 2, "unsupported": 3,
                "malformed": 4, "invalid_auth": 1, "wrong_message": 6, "domain_status": 6}
    if (not case.startswith("RF-019.") or scope not in ("current", "backlog") or action not in statuses or
            (scope == "backlog" and action not in ("accepted", "retry_later", "unsupported", "malformed"))):
        raise ValueError("unknown RF-019 assertion set")
    currents, messages, order = [], {}, []
    counts = Counter()
    for packet in packets:
        frame = bytes.fromhex(packet["frame"])
        opened = open_frame(key, frame)
        h, raw = opened.header, opened.plaintext_body
        if len(frame) != 54 or h.control != 32 or h.node_id != node_id or h.domain not in (1, 2):
            raise ValueError("wrong authenticated uplink identity/shape")
        reading = decode_reading(raw)
        if h.message_id in messages:
            if messages[h.message_id]["frame"] != frame:
                raise ValueError("changed exact retry")
        else:
            if messages and h.message_id <= max(messages):
                raise ValueError("message counter regression")
            if h.domain == 1:
                if currents and (reading.sample_id != currents[-1]["reading"].sample_id + 1 or
                                 reading.reset_reason != 8 or not reading.flags & 257 == 257):
                    raise ValueError("broken wake continuity")
                currents.append(dict(reading=reading, message=h.message_id, raw=raw))
            if not currents:
                raise ValueError("backlog before first current")
            cycle = len(currents) - 1
            messages[h.message_id] = dict(frame=frame, raw=raw, reading=reading,
                domain=h.domain, cycle=cycle, cycle_sample=currents[-1]["reading"].sample_id)
            order.append(h.message_id)
        if messages[h.message_id]["cycle"] != len(currents) - 1:
            raise ValueError("unexpected cross-wake retry")
        counts[h.message_id] += 1
    if len(currents) != seed_count + 2:
        raise ValueError("missing setup/target/metrics wake")
    target = currents[target_cycle]
    seeds = currents[:seed_count]
    backlog = [m for m in order if messages[m]["domain"] == 2]
    wanted_backlog = ([] if scope == "current" and statuses[action] != 1 else
                      [s["reading"].sample_id for s in reversed(seeds)])
    if scope == "backlog" and action == "retry_later":
        wanted_backlog = wanted_backlog[:1]
    if [messages[m]["reading"].sample_id for m in backlog] != wanted_backlog:
        raise ValueError("missing/extra/wrong-order backlog")
    for m in backlog:
        entry = messages[m]
        old = next(s for s in seeds if s["reading"].sample_id == entry["reading"].sample_id)
        if entry["cycle"] != target_cycle or entry["raw"] != old["raw"]:
            raise ValueError("backlog identity/body mismatch")
    expected = {}
    for m in order:
        entry = messages[m]
        if entry["cycle"] != target_cycle:
            outcome = 2
        elif scope == "current" and entry["domain"] == 1:
            outcome = statuses[action]
        elif scope == "backlog" and m == backlog[0]:
            outcome = statuses[action]
        else:
            outcome = 1
        if outcome == 6:
            if counts[m] < 2:
                raise ValueError("missing retry after invalid ACK and silence")
        elif counts[m] != 1:
            raise ValueError("terminal ACK did not complete first attempt")
        expected[m] = outcome
    logs = decoded["logs"]
    deliveries = logs["delivery.log"]
    if deliveries is None or len(deliveries) != 2 * len(order):
        raise ValueError("missing/extra durable delivery boundary")
    for i, m in enumerate(order):
        entry = messages[m]
        start, finish = deliveries[2 * i:2 * i + 2]
        for record, kind in ((start, 4), (finish, 5)):
            if (record["type"], record["cycle_sample_id"], record["sample_id"], record["message_id"], record["domain"]) != (
                    kind, entry["cycle_sample"], entry["reading"].sample_id, m, entry["domain"]):
                raise ValueError("delivery identity/order mismatch")
        if (finish["final_result"], finish["attempt_count"]) != (expected[m], counts[m]):
            raise ValueError("node outcome/attempts differ from RF expectations")
    # Reconstruct surviving readings from specified outcomes, not peer policy.
    pending = []
    quarantine = []
    bindings = {}
    for m in order:
        entry = messages[m]
        sample = entry["reading"].sample_id
        if entry["domain"] == 1:
            pending.append(sample)
        else:
            bindings[sample] = entry["frame"].hex()
        if expected[m] in (1, 3, 4):
            if not pending or pending[-1] != sample:
                raise ValueError("unexpected pending-stack transition")
            pending.pop()
            bindings.pop(sample, None)
        if expected[m] in (3, 4):
            quarantine.append(sample)
    actual_pending = logs["pending.log"]
    if actual_pending is None:
        raise ValueError("no durable pending file")
    readings = [r for r in actual_pending if r["type"] == 1]
    if [r["sample_id"] for r in readings] != pending:
        raise ValueError("pending removal/retention mismatch")
    bodies = {c["reading"].sample_id: c["raw"].hex() for c in currents}
    if any(r["reading_body"] != bodies[r["sample_id"]] for r in readings):
        raise ValueError("pending reading changed")
    actual_bindings = {r["sample_id"]: r["frame"] for r in actual_pending if r["type"] == 6}
    if actual_bindings != bindings:
        raise ValueError("durable backlog binding mismatch")
    quarantined = logs["quarantine.log"] or []
    if [r["sample_id"] for r in quarantined] != quarantine or any(
            r["reading_body"] != bodies[r["sample_id"]] for r in quarantined):
        raise ValueError("quarantine mismatch")
    diagnostics = logs["diagnostic.log"] or []
    wanted_error = {"invalid_auth": 7, "wrong_message": 10, "domain_status": 13}.get(action)
    if wanted_error is None:
        if diagnostics:
            raise ValueError("unexpected node diagnostic")
    elif len(diagnostics) != 1 or any(
            (r["error_domain"], r["error_code"], r["cycle_sample_id"], r["message_id"]) !=
            (4, wanted_error, target["reading"].sample_id, target["message"]) for r in diagnostics):
        raise ValueError("missing/wrong invalid-ACK diagnostic")
    for cycle, current in enumerate(currents[1:]):
        prior = currents[cycle]
        wake_messages = [m for m in order if messages[m]["cycle"] == cycle]
        reading = current["reading"]
        accepted = expected[prior["message"]] == 1
        if (reading.previous_current_tx_attempts, reading.previous_cycle_tx_attempts,
            reading.previous_cycle_accepted_readings, bool(reading.flags & 512)) != (
                counts[prior["message"]], sum(counts[m] for m in wake_messages),
                sum(expected[m] == 1 for m in wake_messages), accepted):
            raise ValueError("next-wake metrics mismatch")
    return dict(case=case, status="PASS", scope="production_node_controlled_peer",
                wakes=len(currents), packets=len(packets), host_deadline_assertion="separate prerequisite")


def verify_ack_transmissions(case, node_id, key, outcome, trace, attempts):
    """Check exact independently constructed ACK bytes against actual SPI writes."""
    scope, action = case.split(".")[1:]
    seeds = 2 if scope == "backlog" else 1
    seen, current_ids, backlog_ids, expected = set(), [], [], []
    for packet in outcome["packets"]:
        opened = open_frame(key, bytes.fromhex(packet["frame"]))
        h = opened.header
        if h.message_id in seen:
            continue
        seen.add(h.message_id)
        if h.domain == 1:
            current_ids.append(h.message_id)
        else:
            backlog_ids.append(h.message_id)
        index = len(current_ids) - 1
        selected = (index == seeds and
                    ((scope == "current" and h.domain == 1) or
                     (scope == "backlog" and h.domain == 2 and len(backlog_ids) == 1)))
        selected_action = action if selected else ("accepted" if index == seeds else "retry_later")
        status = {"accepted": 0, "retry_later": 1, "unsupported": 2, "malformed": 3}.get(selected_action, 0)
        domain = 4 if selected_action == "domain_status" else status + 3
        message = 0xffffffff if selected_action == "wrong_message" else h.message_id
        header = struct.pack("<BB8sI", 32, domain, node_id, message)
        nonce = struct.pack("<8sIB", node_id, message, domain)
        frame = header + AESCCM(key, tag_length=8).encrypt(nonce, bytes([status]), header)
        if selected_action == "invalid_auth":
            expected.append((frame[:-1] + bytes([frame[-1] ^ 1])).hex())
        expected.append(frame.hex())
    actual = [t["frame"] for t in outcome["transmissions"]]
    writes = [event["tx"][4:] for event in trace
              if event.get("operation") == "spi" and event.get("tx", "").startswith("0e00")]
    commands = [event for event in trace
                if event.get("operation") == "spi" and event.get("tx", "").startswith("83")]
    if actual != expected or writes != expected or attempts != len(expected) or len(commands) != len(expected):
        raise ValueError("ACK bytes or actual TX count differ from selected case")

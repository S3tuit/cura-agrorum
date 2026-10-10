"""Independent raw IRQ, interrupted-TX and persistent-window reconciliation.

No peer policy imports. Pi and C6 clocks are checked only within their domains.
"""
import json
import re
import struct

from verify import _verify_pi_command_confirmation


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify_summary(action, diagnostics, cycle, message):
    require(len(diagnostics) == 1, "missing/extra HeaderErr window diagnostic")
    record = diagnostics[0]
    require((record["error_domain"], record["error_code"], record["operation"],
             record["context_schema"], record["cycle_sample_id"], record["message_id"]) ==
            (4, 15, 17, 1, cycle, message), "wrong HeaderErr diagnostic identity/schema")
    context = bytes.fromhex(record["context"])
    require(len(context) == 14, "wrong HeaderErr context length")
    attempt, ack, first, header, crc = struct.unpack("<BBQHH", context)
    require((attempt, ack, header, crc) == (1, int(action == "header_error_rearm"), 2, 0),
            "wrong HeaderErr window counts/attempt/ACK")
    return first


def node_observations(uart, wakes=3):
    groups, pending, sealed = [], [], False
    for raw in uart.splitlines():
        if raw.startswith(b"RF_NODE_PHY "):
            require(not sealed, "PHY observation after end marker")
            item = json.loads(raw[len(b"RF_NODE_PHY "):])
            require(set(item) == {"index", "opcode", "value", "before_us", "after_us", "irq_at_us", "result"},
                    "malformed PHY observation")
            require(all(type(v) is int and v >= 0 for v in item.values()), "invalid PHY observation fields")
            require(item["index"] == len(pending) and len(pending) < 128 and item["result"] == 0 and
                    item["opcode"] in (0x12, 0x82, 0x83) and item["value"] <= 0xffff and
                    item["irq_at_us"] <= item["before_us"] <= item["after_us"],
                    "invalid PHY chronology/result")
            require(not pending or pending[-1]["after_us"] <= item["before_us"], "PHY trace reordered")
            pending.append(item)
        elif raw.startswith(b"RF_NODE_PHY_END"):
            require(not sealed, "duplicate PHY end marker")
            match = re.fullmatch(rb"RF_NODE_PHY_END count=(\d+) overflow=0", raw)
            require(match is not None and int(match[1]) == len(pending), "missing/overflowed PHY trace")
            sealed = True
        elif raw.startswith(b"RF_NODE_SLEEP"):
            require(raw == b"RF_NODE_SLEEP duration_us=10000000" and sealed and pending,
                    "sleep without complete PHY observation")
            groups.append(pending)
            pending, sealed = [], False
    require(len(groups) == wakes and not pending and not sealed, "incomplete PHY wake observations")
    return groups


def verify_header_ack_rf(case, outcome, trace, decoded, uart):
    action = case.split(".")[-1]
    require(action in ("header_error_rearm", "header_error_retry"), "unknown HeaderErr RF case")
    logs = decoded["logs"]
    diagnostics = logs["diagnostic.log"] or []
    target = [r for r in logs["delivery.log"] if r["type"] == 5 and r["domain"] == 1][1]
    first = verify_summary(action, diagnostics, target["cycle_sample_id"], target["message_id"])
    calls = target["tx_calls"]
    require(len(calls) == (1 if action == "header_error_rearm" else 2) and target["final_result"] == 1,
            "wrong HeaderErr delivery history")
    require(calls[-1]["outcome"] == "ACK_RECEIVED", "missing successful ACK after HeaderErr")
    if len(calls) == 2:
        require(calls[0]["outcome"] == "ACK_TIMEOUT" and
                400_000 <= calls[1]["set_tx_at_us"] - calls[0]["tx_done_at_us"] <= 710_000,
                "HeaderErr window ended early or retry was delayed")

    groups = node_observations(uart)
    starts = [e for e in groups[1] if e["opcode"] == 0x83]
    completions = [e for e in groups[1] if e["opcode"] == 0x12 and e["value"] == 1]
    require(len(starts) >= len(calls) and len(completions) >= len(calls),
            "missing raw node target transmission")
    for call, start, completion in zip(calls, starts, completions):
        require(call["tx_done_at_us"] == completion["irq_at_us"] and
                call["set_tx_at_us"] <= start["before_us"] <= call["set_tx_at_us"] + 1_000 and
                start["after_us"] < completion["irq_at_us"],
                "saved target transmission differs from raw SetTx/TX_DONE")
    rejects = [(wake, i, e) for wake, group in enumerate(groups) for i, e in enumerate(group)
               if e["opcode"] == 0x12 and e["value"] & 0x60]
    # RX_DONE does not establish CRC validity. A co-observed RX_DONE still
    # carries HeaderErr, and the driver discards it before packet processing.
    # Exclude payload CRC and every unrelated IRQ bit from this HeaderErr case.
    require(len(rejects) == 2 and all(w == 1 and e["value"] in (0x20, 0x22) for w, _, e in rejects),
            "missing/extra real node HeaderErr or payload CRC instead")
    require(first == rejects[0][2]["irq_at_us"], "saved first rejection differs from raw IRQ")
    for _, index, rejection in rejects:
        require(calls[0]["tx_done_at_us"] < rejection["irq_at_us"] < calls[0]["tx_done_at_us"] + 400_000,
                "HeaderErr outside first ACK window")
        following = next((e for e in groups[1][index + 1:] if e["opcode"] == 0x12 and e["value"]), None)
        require(following is not None and any(
            e["opcode"] == 0x82 and rejection["after_us"] <= e["before_us"] <= e["after_us"] < following["irq_at_us"]
            for e in groups[1][index + 1:]), "missing node rearm before next reception")
    ack_time = calls[-1]["ack_rx_done_at_us"]
    require(any(e["opcode"] == 0x12 and e["value"] == 2 and e["irq_at_us"] == ack_time for e in groups[1]),
            "saved ACK differs from raw RX_DONE")
    require(ack_time > rejects[-1][2]["irq_at_us"] and
            ack_time - calls[-1]["tx_done_at_us"] <= (400_000 if len(calls) == 1 else 300_000),
            "valid control ACK outside unchanged window")

    transmissions = outcome["transmissions"]
    require(len(transmissions) == 6 and [i for i, t in enumerate(transmissions) if t.get("interrupted")] == [1, 2],
            "missing/extra interrupted ACK stimuli")
    spi = [e for e in trace if e.get("operation") == "spi"]
    starts = [i for i, e in enumerate(spi) if e["tx"].startswith("83")]
    require(len(starts) == 6, "wrong actual Pi SetTx count")
    # Target uplink is the second received packet. The third is either its
    # retry or the backlog; only the retry case takes its control ACK origin.
    origin = outcome["packets"][1]["at"]
    expected_targets = [origin + 100_000, origin + 180_000,
                        (origin + 260_000 if len(calls) == 1 else outcome["packets"][2]["at"] + 150_000)]
    for index in (1, 2, 3):
        t = transmissions[index]
        start = starts[index]
        end = starts[index + 1] if index + 1 < len(starts) else len(spi)
        require(t["target"] == expected_targets[index - 1] and
                t["target"] <= t["set_tx"] <= t["target"] + 10_000 and
                t["set_tx"] <= spi[start]["before"] <= t["set_tx"] + 500,
                "wrong Pi ACK schedule/SetTx binding")
        if index == 3:
            require(0 < t["tx_done"] - t["set_tx"] <= 100_000, "unconfirmed control ACK timing")
            continue
        standby = next((j for j in range(start + 1, end) if spi[j]["tx"] == "8000"), None)
        require(standby is not None and standby + 1 < end, "missing Pi abort standby")
        require(t["abort_before"] <= spi[standby]["before"] <= spi[standby]["after"] <= t["abort_after"] and
                18_000 <= t["abort_before"] - t["set_tx"] <= 20_000 and
                17_500 <= spi[standby]["before"] - spi[start]["before"] <= 20_000,
                "Pi abort missed qualified cutoff bounds")
        _verify_pi_command_confirmation(spi[standby + 1], 2, "Pi abort standby unconfirmed")
        require(t["irq"] == t["device_errors"] == 0, "interrupted TX completed or failed")
        post = spi[standby + 2:end]
        for command, message in (("12000000", "IRQ"), ("17000000", "device errors")):
            raw = next((e for e in post if e["tx"] == command), None)
            require(raw is not None and len(bytes.fromhex(raw.get("result", ""))) == 4 and
                    int(raw["result"][-4:], 16) == 0, "missing raw Pi abort " + message)

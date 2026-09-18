"""Independent RF assertions and disposable capture checks; no hardware I/O."""
from __future__ import annotations
import argparse
import json
from pathlib import Path

# Literal catalogue oracles, independent of either endpoint's construction.
A = "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f303132333435"
U = "808182838485868788898a8b8c8d8e8f909192939495969798999a9b9c9d9e9fa0a1a2a3a4a5a6a7a8a9aaabacadaeafb0b1b2b3b4b5"
B = "a0a1a2a3a4a5a6a7a8a9aaabacadaeafb0b1b2b3b4b5b6"
D = "c0c1c2c3c4c5c6c7c8c9cacbcccdcecfd0d1d2d3d4d5d6"
X = "e0e1e2e3e4e5e6e7e8e9eaebecedeeeff0f1f2f3f4f5f6"
EXPECTED = {
    "RF-001.exchange": ([A], [B]), "RF-003.silence": ([A], [""]),
    "RF-006.invalid": ([A], ["00", "deadbeef", X, ""]),
    "RF-008.silence": ([A, U], [""]), "RF-008.exchange": ([A, U], [B, D]),
    "RF-009.untouched": ([A], []), "RF-009.initialized": ([A, A], []),
    "RF-010.wake": ([A, U], [B, D]), "RF-012.disconnected": ([A], []),
    "RF-013.absent": ([A], []),
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify_pi_profiles(trace, downlinks):
    """Decode recorded commands against the protocol and reviewed SX1262 vectors.

    Literal profile vectors match receiver/tests/host/test_sx1262.py's reviewed
    oracle, not imports from the driver. This proves command sequencing, not RF
    electrical timing or a register's physical retention after the run.
    """
    spi = [v for v in trace if v["operation"] == "spi"]
    require(spi and all("result" in v and "error" not in v for v in spi), "failed Pi SPI primitive")
    segment, tx_index, rx_count = [], 0, 0
    for event in spi:
        command = event["tx"]
        if command == "8a01":
            segment = []
        segment.append(command)
        if not command.startswith(("82", "83")):
            continue
        transmit = command.startswith("83")
        length = len(downlinks[tx_index]) // 2 if transmit and tx_index < len(downlinks) else 255
        expected = ["8a01", "863641999a", "8b07040100",
                    f"8c000800{length:02x}01{int(transmit):02x}", "8e0e02", "9320", "9f00", "a000",
                    "080263026300000000", "0d07401424", "0d08ac96"]
        positions = [segment.index(value) if value in segment else -1 for value in expected]
        require(-1 not in positions and positions == sorted(positions), "incomplete or wrong Pi direction profile")
        for prefix, bit in (("0d0889", 4), ("0d0736", 0 if transmit else 4)):
            writes = [v for v in segment if v.startswith(prefix)]
            require(len(writes) == 1 and len(writes[0]) == 8 and int(writes[0][6:], 16) & 4 == bit,
                    "wrong Pi modulation/IQ register workaround")
        require(command == ("83001900" if transmit else "82000000"), "wrong Pi watchdog/RX mode")
        tx_index += int(transmit)
        rx_count += int(not transmit)
        segment = []  # A later operation needs its own complete profile.
    require(tx_index == len(downlinks) and rx_count > 0, "missing Pi profile operations")
    operations = [v["tx"] for v in spi if v["tx"].startswith(("82", "83"))]
    require(operations[-1] == "82000000", "Pi did not restore receive after final TX")
    tail = spi[next(i for i in range(len(spi)-1, -1, -1) if spi[i]["tx"].startswith(("82", "83"))) + 1:]
    commands = [v["tx"] for v in tail]
    require("8000" in commands and "080000000000000000" in commands and
            "12000000" in commands and "02ffff" in commands, "missing Pi standby/IRQ cleanup")
    require(spi[-1]["tx"] == "c000" and (int(spi[-1]["result"][-2:], 16) >> 4) & 7 == 2,
            "Pi cleanup did not confirm standby")
    require(trace[-1]["operation"] == "close" and "result" in trace[-1] and "error" not in trace[-1],
            "Pi handles not released")


def verify_case(case, events, peer, run, fixture, elf):
    require(case in EXPECTED, "unimplemented case")
    boots = [v for v in events if v["kind"] == "boot"]
    begins = [v for v in events if v["kind"] == "begin"]
    ends = [v for v in events if v["kind"] == "end"]
    phases = 2 if case == "RF-010.wake" else 1
    commands = [v for v in events if v["kind"] == "command"]
    require(len(commands) == phases and all(v["boot"] == b["boot"] and
            0 <= v["elapsed_us"] <= 2_000_000 and 0 < v["bytes"] <= 159
            for v, b in zip(commands, begins)), "invalid command framing evidence")
    require(not any(v["kind"] == "reject" for v in events), "C6 rejected command")
    require(len(boots) == phases + 1 and len(begins) == len(ends) == phases, "incomplete wake/phase evidence")
    require(len({v["boot"] for v in boots}) == len(boots), "reused boot nonce")
    for index, boot in enumerate(boots):
        require(boot["dut"] == fixture["c6_dut"] and boot["elf"] == elf, "wrong C6 source/device")
        if index:
            require(boot["reset"] == 8 and boot["previous_boot"] == boots[index-1]["boot"] and
                    boot["previous_run"] == run and boot["previous_case"] == case, "missing actual deep sleep")
    for index, (begin, end) in enumerate(zip(begins, ends)):
        for value in (begin, end):
            require(value["run"] == run and value["case"] == case and value["phase"] == index and
                    value["boot"] == boots[index]["boot"], "phase identity mismatch")
        require(end["failed"] == 0, "C6 Unity failed")
        require(end["cleanup_error"] == 0 or case == "RF-013.absent", "C6 cleanup failed")
    results = [v for v in events if v["kind"] == "result"]
    tx = [v for v in results if v["tx"]]
    rx = [v for v in results if not v["tx"]]
    expected_tx, expected_rx = EXPECTED[case]
    require([v["payload"] for v in tx] == expected_tx, "wrong or missing C6 TX calls")
    require([v["payload"] for v in rx] == expected_rx, "wrong or missing C6 RX results")
    for index, value in enumerate(tx):
        require(0 < value["before"] <= value["after"] <= value["deadline"] + 50_000, "C6 TX bound")
        failed = case in {"RF-012.disconnected", "RF-013.absent"} or case == "RF-009.initialized" and index == 1
        if not failed:
            require(value["error"] == 0 and value["operation"] == 0 and not value["diagnostic"] and
                    value["started"] is True and value["done"] is True, "C6 TX outcome")
            require(value["before"] <= value["set_tx"] < value["tx_done"] <= value["after"], "C6 shared TX clock")
            require(102656 <= value["tx_done"] - value["set_tx"] <= 112922, "C6 TX airtime")
        else:
            require(value["error"] != 0 and value["done"] is False, "missing expected TX fault")
            if case == "RF-012.disconnected":
                diag = bytes.fromhex(value["diagnostic"])
                require(value["started"] is True and value["operation"] == 16 and len(diag) == 14 and
                        diag[2] == 11 and diag[3] & 8, "DIO1 certainty/diagnostic")
            elif case == "RF-013.absent":
                require(value["started"] is False and value["error"] == 0x20004 and value["operation"] == 1,
                        "absence must be an initialization BUSY failure")
            else:
                require(value["started"] is False and value["error"] == 0x20002, "post-sleep invalid state")
    for value in rx:
        require(value["error"] == value["operation"] == 0 and not value["diagnostic"], "C6 RX error")
        if value["payload"]:
            require(value["rx_outcome"] == 1 and value["before"] <= value["rx_done"] <= value["deadline"] and
                    value["rx_done"] <= value["after"] and -255 <= value["rssi"] <= 0 and
                    -128 <= value["snr"] <= 127, "C6 RX packet/clock metadata")
        else:
            require(value["rx_outcome"] == 2 and value["rx_done"] == value["rssi"] == value["snr"] == 0,
                    "nonzero deadline packet fields")
            require(value["deadline"] <= value["after"] <= value["deadline"] +
                    ((value["deadline"] - value["before"]) * 15 + 99) // 100, "C6 RX deadline extension")
    if case == "RF-006.invalid":
        require(len({v["deadline"] for v in rx}) == 1, "invalid packets extended RX deadline")
    trace = [v for v in events if v["kind"] == "trace"]
    starts = [v for v in trace if v["operation"] == "start_tx"]
    positive = len(tx) - int(case in {"RF-009.initialized", "RF-013.absent"})
    require(len(starts) == positive and all(v["result"] == 2 for v in starts), "C6 SetTx facts")
    initializes = [v for v in trace if v["operation"] == "initialize"]
    require(len(initializes) == phases, "C6 initialization count")
    require(all(v["result"] == int(case != "RF-013.absent") for v in initializes), "initialization outcome")
    sleep_calls = [v for v in trace if v["operation"] == "set_sleep_cold"]
    require(case == "RF-013.absent" or len(sleep_calls) == phases and all(v["result"] == 2 for v in sleep_calls),
            "missing cold-sleep evidence")
    if case == "RF-009.untouched":
        require(trace[0]["operation"] == "initialize", "untouched sleep performed I/O")
    if case == "RF-009.initialized":
        require(trace[-1]["operation"] == "set_sleep_cold" and trace[-1]["after"] <= tx[-1]["before"],
                "terminal sleep performed later I/O")
    require(peer["kind"] == "complete" and peer["case"] == case and peer["run"] == run and
            peer["failure"] is None and peer["cleanup"]["safe_shutdown"] is True, "peer failed or wrong identity")
    expected_air = [] if case == "RF-013.absent" else expected_tx[:positive]
    packets = peer["outcome"]["packets"]
    require([p["frame"] for p in packets] == expected_air, "Pi copied packet mismatch")
    for packet in packets:
        require(packet["irq_status"] == 2 and packet["device_errors"] == 0 and
                0 < packet["edge_timestamp_ns"] // 1000 <= packet["t2_packet_copied_monotonic_us"], "Pi RX metadata")
    pi_tx = [v for v in peer["trace"] if v["operation"] == "spi" and v["tx"].startswith("83")]
    expected_down = [value for value in expected_rx if value]
    verify_pi_profiles(peer["trace"], expected_down)
    require(peer["attempts"] == len(pi_tx) == len(expected_down), "Pi attempt ceiling/count")
    buffers = [v["tx"][4:] for v in peer["trace"] if v["operation"] == "spi" and v["tx"].startswith("0e00")]
    require(buffers == expected_down, "Pi downlink buffer mismatch")
    require(len(peer["outcome"]["transmissions"]) == len(expected_down), "missing Pi TX outcome")
    if case == "RF-006.invalid":
        require(peer["layer"] == "Sx1262/LinuxRadioIo", "wrong RF-006 layer claim")
        for value in peer["outcome"]["transmissions"]:
            require(value["event"]["irq_status"] == 1 and value["certainty"] == "CONFIRMED_APPLIED" and
                    value["set_tx"] <= value["tx_done"] <= value["set_tx"] + 250000, "burst TX incomplete")
    else:
        require(peer["layer"] == "Radio/Sx1262/LinuxRadioIo", "wrong production layer")
        for value in peer["outcome"]["transmissions"]:
            require(value["tx"]["ack_tx_result"] == "TX_DONE" and value["t6_set_rx_issued_monotonic_us"] is not None and
                    value["t6_set_rx_issued_monotonic_us"] >= value["tx"]["t5_tx_done_monotonic_us"], "Pi profile restoration")
    return dict(case=case, status="PASS", scope="raw_component", c6_attempts=len(starts), pi_attempts=len(pi_tx))


def verify_run(root):
    root = Path(root)
    run = json.loads((root / "run.json").read_text())
    require(run["pytest_exit"] == 0 and not run["failures"], "failed pytest or retained run failure")
    import xml.etree.ElementTree as ET
    xml = ET.parse(root / "junit.xml")
    require(bool(xml.findall(".//testcase")) and not any(xml.findall(".//" + kind) for kind in ("failure", "error", "skipped")),
            "missing or failed JUnit evidence")
    require(run["status"] == "PASS" and len(run["results"]) == len(run["selected"]) > 0, "incomplete requested run")
    require([v["case"] for v in run["results"]] == run["selected"], "requested/result selections differ")
    for case in run["selected"]:
        path = root / case
        events = json.loads((path / "c6-events.json").read_text())
        pi = [json.loads(line) for line in (path / "pi.jsonl").read_text().splitlines()]
        require([v["kind"] for v in pi] == ["ready", "armed", "complete"], "missing/extra Pi events")
        require(len({(v["pid"], v["boot"], v["source"], v["run"], v["case"]) for v in pi}) == 1 and
                pi[-1]["source"] == run["source"], "Pi provenance changed")
        verify_case(case, events, pi[-1], run["run"], run["fixture"], run["elf"])
        require(json.loads((path / "peer-exit.json").read_text())["exit"] == 0, "nonzero peer exit")
    return run


def verify_retained(root):
    """Recheck curated fault captures; restoration is a historical summary only."""
    root = Path(root)
    report = json.loads((root / "results.json").read_text())
    for case, record in report["cases"].items():
        require(record["status"] == "PASS" and record["pytest_exit"] == 0 and
                record["peer_exit"]["exit"] == 0, "failed retained execution")
        captures = record["capture"]
        events = json.loads((root / captures["c6"]).read_text())
        peer = [json.loads(line) for line in (root / captures["pi"]).read_text().splitlines()]
        require([v["kind"] for v in peer] == ["ready", "armed", "complete"], "incomplete retained peer")
        require(len({(v["pid"], v["boot"], v["source"], v["run"], v["case"]) for v in peer}) == 1 and
                peer[-1]["source"] == record["source"], "retained peer provenance changed")
        checked = verify_case(case, events, peer[-1], record["run"], record["fixture"], record["elf"])
        require(record["results"] == [checked], "retained result differs from observations")
    return len(report["cases"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    root = parser.parse_args().directory
    count = verify_retained(root) if (root / "results.json").exists() else len(verify_run(root)["selected"])
    print(f"Verified {count} raw component parameters; no full-service acceptance claim.")

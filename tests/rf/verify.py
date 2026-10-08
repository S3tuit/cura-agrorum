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
    "component.ack_exchange": ([A], [B]), "component.ack_timeout": ([A], [""]),
    "component.invalid_downlinks": ([A], ["00", "deadbeef", X, ""]),
    "component.repeat_timeout": ([A, U], [""]), "component.repeat_exchange": ([A, U], [B, D]),
    "component.cold_sleep": ([A], []), "component.initialized_sleep": ([A, A], []),
    "component.sleep_wake": ([A, U], [B, D]), "component.dio1_disconnected": ([A], []),
    "component.radio_absent": ([A], []),
    "component.header_error_rearm": ([A, A, U], []),
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


# Literal required command parameters, independent of the production driver.
_PI_PROFILE_COMMANDS = {
    0x8a: b"\x01",                       # LoRa
    0x86: b"\x36\x41\x99\x9a",          # 868.1 MHz
    0x8b: b"\x07\x04\x01\x00",          # SF7/BW125/CR4/5, LDRO off
    0x8c: b"\x00\x08\x00\xff\x01\x00",  # RX: preamble/header/length/CRC/IQ
    0x8e: b"\x0e\x02",                   # +14 dBm, 40 us ramp
    0x93: b"\x20",                       # STDBY_RC fallback
    0x9f: b"\x00",                       # RX timer continues at preamble
    0xa0: b"\x00",                       # No symbol-count timeout
    0x08: b"\x02\x63\x02\x63\x00\x00\x00\x00",  # IRQ/DIO1 mask
}


class _PiProfileReplay:
    """Known profile values and fresh installation coverage, not a chip emulator."""

    def __init__(self):
        self.reset_asserted = False
        self.invalidate()

    def invalidate(self):
        self.commands, self.registers = {}, {}
        self.fresh_commands, self.fresh_registers = set(), set()

    def reset(self, event):
        require("result" in event and "error" not in event and type(event.get("asserted")) is bool,
                "failed or invalid Pi reset primitive")
        self.reset_asserted = event["asserted"]
        self.invalidate()

    def apply(self, command):
        require(command and not self.reset_asserted, "empty Pi command or SPI during reset")
        opcode, parameters = command[0], command[1:]
        if opcode == 0x0d:
            require(len(parameters) >= 3, "malformed Pi register write")
            address = int.from_bytes(parameters[:2], "big")
            values = parameters[2:]
            require(address + len(values) <= 0x10000, "Pi register write exceeds address space")
            for offset, value in enumerate(values):
                self.registers[address + offset] = value
                self.fresh_registers.add(address + offset)
        elif opcode in _PI_PROFILE_COMMANDS:
            require(len(parameters) == len(_PI_PROFILE_COMMANDS[opcode]), "malformed Pi profile command")
            if opcode == 0x8a and self.commands.get(opcode) != parameters:
                self.invalidate()
            if opcode in (0x8b, 0x8c):
                require(self.commands.get(0x8a) == b"\x01", "Pi LoRa parameters before packet type")
                # Each associated parameter command needs its workaround reapplied.
                address = 0x0889 if opcode == 0x8b else 0x0736
                self.registers.pop(address, None)
                self.fresh_registers.discard(address)
            self.commands[opcode] = parameters
            self.fresh_commands.add(opcode)
        elif opcode not in (0x02, 0x07, 0x0e, 0x12, 0x13, 0x14, 0x17, 0x1d, 0x1e,
                            0x80, 0x82, 0x83, 0xc0):
            # Sleep and unmodeled commands cannot preserve known profile state.
            # Initialization's module-specific commands precede a full install.
            self.invalidate()

    def verify(self, transmit, length):
        expected = dict(_PI_PROFILE_COMMANDS)
        expected[0x8c] = b"\x00\x08\x00" + bytes((length, 1, int(transmit)))
        for opcode, parameters in expected.items():
            require(opcode in self.fresh_commands and self.commands.get(opcode) == parameters,
                    f"incomplete or wrong Pi profile command 0x{opcode:02x}")
        # Workaround checks preserve unrelated bits; sync/gain bytes are exact.
        registers = {0x0740: (0x14, 0xff), 0x0741: (0x24, 0xff), 0x08ac: (0x96, 0xff),
                     0x0889: (4, 4), 0x0736: (0 if transmit else 4, 4)}
        for address, (value, mask) in registers.items():
            require(address in self.fresh_registers and self.registers.get(address, -1) & mask == value,
                    f"incomplete or wrong Pi profile register 0x{address:04x}")
        self.fresh_commands.clear()
        self.fresh_registers.clear()


def verify_pi_profiles(trace, downlinks):
    """Replay effective required profile state at each recorded SetRx/SetTx.

    Literal oracles match the protocol and reviewed Semtech vectors, without
    importing the driver. This checks configuration evidence and freshness,
    not RF electrical timing, output power or physical register retention.
    """
    spi = [v for v in trace if v["operation"] == "spi"]
    require(spi and all("result" in v and "error" not in v for v in spi), "failed Pi SPI primitive")
    state, tx_index, rx_count = _PiProfileReplay(), 0, 0
    for event in trace:
        if event["operation"] == "reset":
            state.reset(event)
        if event["operation"] != "spi":
            continue
        command = bytes.fromhex(event["tx"])
        state.apply(command)
        if command[0] not in (0x82, 0x83):
            continue
        transmit = command[0] == 0x83
        length = len(downlinks[tx_index]) // 2 if transmit and tx_index < len(downlinks) else 255
        state.verify(transmit, length)
        require(command == (b"\x83\x00\x19\x00" if transmit else b"\x82\x00\x00\x00"),
                "wrong Pi watchdog/RX mode")
        tx_index += int(transmit)
        rx_count += int(not transmit)
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


def _verify_pi_command_confirmation(event, mode, message):
    """Literal GetStatus framing and ordinary command-result checks."""
    require(event["tx"] == "c000", message)
    reply = bytes.fromhex(event.get("result", ""))
    require(len(reply) == 2, message)
    status = reply[1]
    require(not status & 0x81 and (status >> 4) & 7 == mode and
            (status >> 1) & 7 not in (3, 4, 5, 7), message)


def verify_header_error(events, peer, tx):
    """Actual command replies and fresh complete rearming, independent of Radio."""
    cuts = [v for v in events if v.get("operation") == "header_abort"]
    require(len(cuts) == 1, "missing C6 abort timing")
    cut = cuts[0]
    require(cut["argument"] == 18000 and cut["result"] == 2 and
            cut["before"] == tx[1]["set_tx"] and
            cut["before"] <= cut["tx_hal_after"] <= cut["abort_hal_before"] <= cut["after"] and
            18000 <= cut["abort_hal_before"] - cut["before"] <= 18500 and
            cut["irq"] == cut["device_errors"] == 0, "wrong C6 abort stimulus")
    require(all(b["set_tx"] - a["set_tx"] >= 12200000 for a, b in zip(tx, tx[1:])), "C6 pacing")
    outcome = peer["outcome"]
    require(outcome["counters_before"] == outcome["counters_after"], "HeaderErr counters changed")
    require(outcome["counters_after"]["recovery_attempts"] == 0, "HeaderErr used recovery")
    handled = outcome["handled"]
    require(len(handled) == 1, "missing handled HeaderErr result")
    result = handled[0]
    event = result["receive_event"]
    require(result["state"] == "RX_SINGLE" and not result["episodes"] and
            event["disposition"] == "HANDLED_NO_PACKET" and event["frame"] is None and
            event["irq_status"] == 0x20 and event["device_errors"] == 0, "wrong HeaderErr disposition")
    packets = outcome["packets"]
    require(len(packets) == 2 and packets[0]["edge_timestamp_ns"] < event["edge_timestamp_ns"] <
            packets[1]["edge_timestamp_ns"] and
            event["edge_timestamp_ns"] // 1000 <= event["t1_handler_started_monotonic_us"] <=
            result["t6_set_rx_issued_monotonic_us"] <= packets[1]["edge_timestamp_ns"] // 1000,
            "HeaderErr/rearm/next-packet chronology")
    trace = peer["trace"]
    require(not any(v["operation"] == "reset" for v in trace
                    if v.get("before", 0) >= event["t1_handler_started_monotonic_us"]), "unexpected reset recovery")
    spi = [v for v in trace if v["operation"] == "spi"]
    observed = [i for i, v in enumerate(spi) if v["tx"] == "12000000" and
                len(bytes.fromhex(v.get("result", ""))) == 4 and int(v["result"][-4:], 16) == 0x20]
    require(len(observed) == 1, "missing/extra raw HeaderErr IRQ")
    i = observed[0]
    require(spi[i+1]["tx"] == "17000000" and len(bytes.fromhex(spi[i+1]["result"])) == 4 and
            int(spi[i+1]["result"][-4:], 16) == 0 and spi[i+2]["tx"] == "c000" and
            len(bytes.fromhex(spi[i+2]["result"])) == 2 and int(spi[i+2]["result"][-2:], 16) == 0x52,
            "wrong raw HeaderErr status/device evidence")
    require(spi[i+3]["tx"] == "8000", "HeaderErr standby not first")
    _verify_pi_command_confirmation(spi[i+4], 2, "HeaderErr standby unconfirmed")
    j = next((j for j in range(i+3, len(spi)) if spi[j]["tx"] == "020020"), None)
    require(j is not None and not any(v["tx"].startswith(("82", "83", "1e")) for v in spi[i+3:j]),
            "missing exact IRQ clearing or premature packet/mode operation")
    k = next((k for k in range(j+1, len(spi)) if spi[k]["tx"].startswith(("82", "83"))), None)
    require(k is not None and spi[k]["tx"] == "82000000" and
            spi[k-1]["after"] <= result["t6_set_rx_issued_monotonic_us"] <= spi[k]["before"],
            "missing correlated HeaderErr SetRx")
    _verify_pi_command_confirmation(spi[k+1], 5, "HeaderErr SetRx unconfirmed")
    state = _PiProfileReplay()
    for v in spi[j+1:k+1]:
        state.apply(bytes.fromhex(v["tx"]))
    state.verify(False, 255)


def verify_case(case, events, peer, run, fixture, elf):
    require(case in EXPECTED, "unimplemented case")
    boots = [v for v in events if v["kind"] == "boot"]
    begins = [v for v in events if v["kind"] == "begin"]
    ends = [v for v in events if v["kind"] == "end"]
    phases = 2 if case == "component.sleep_wake" else 1
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
        require(end["cleanup_error"] == 0 or case == "component.radio_absent", "C6 cleanup failed")
    results = [v for v in events if v["kind"] == "result"]
    tx = [v for v in results if v["tx"]]
    rx = [v for v in results if not v["tx"]]
    expected_tx, expected_rx = EXPECTED[case]
    require([v["payload"] for v in tx] == expected_tx, "wrong or missing C6 TX calls")
    require([v["payload"] for v in rx] == expected_rx, "wrong or missing C6 RX results")
    for index, value in enumerate(tx):
        require(0 < value["before"] <= value["after"] <= value["deadline"] + 50_000, "C6 TX bound")
        failed = case in {"component.dio1_disconnected", "component.radio_absent"} or case == "component.initialized_sleep" and index == 1
        if not failed:
            if case == "component.header_error_rearm" and index == 1:
                require(value["error"] == value["operation"] == 0 and not value["diagnostic"] and
                        value["started"] is True and value["done"] is False and value["tx_done"] == 0,
                        "C6 intentional interruption outcome")
                require(value["before"] <= value["set_tx"] <= value["after"], "C6 interruption clock")
                continue
            require(value["error"] == 0 and value["operation"] == 0 and not value["diagnostic"] and
                    value["started"] is True and value["done"] is True, "C6 TX outcome")
            require(value["before"] <= value["set_tx"] < value["tx_done"] <= value["after"], "C6 shared TX clock")
            require(102656 <= value["tx_done"] - value["set_tx"] <= 112922, "C6 TX airtime")
        else:
            require(value["error"] != 0 and value["done"] is False, "missing expected TX fault")
            if case == "component.dio1_disconnected":
                diag = bytes.fromhex(value["diagnostic"])
                require(value["started"] is True and value["operation"] == 16 and len(diag) == 14 and
                        diag[2] == 11 and diag[3] & 8, "DIO1 certainty/diagnostic")
            elif case == "component.radio_absent":
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
    if case == "component.invalid_downlinks":
        require(len({v["deadline"] for v in rx}) == 1, "invalid packets extended RX deadline")
    trace = [v for v in events if v["kind"] == "trace"]
    starts = [v for v in trace if v["operation"] == "start_tx"]
    positive = len(tx) - int(case in {"component.initialized_sleep", "component.radio_absent"})
    require(len(starts) == positive and all(v["result"] == 2 for v in starts), "C6 SetTx facts")
    initializes = [v for v in trace if v["operation"] == "initialize"]
    require(len(initializes) == phases, "C6 initialization count")
    require(all(v["result"] == int(case != "component.radio_absent") for v in initializes), "initialization outcome")
    sleep_calls = [v for v in trace if v["operation"] == "set_sleep_cold"]
    require(case == "component.radio_absent" or len(sleep_calls) == phases and all(v["result"] == 2 for v in sleep_calls),
            "missing cold-sleep evidence")
    if case == "component.cold_sleep":
        require(trace[0]["operation"] == "initialize", "untouched sleep performed I/O")
    if case == "component.initialized_sleep":
        require(trace[-1]["operation"] == "set_sleep_cold" and trace[-1]["after"] <= tx[-1]["before"],
                "terminal sleep performed later I/O")
    require(peer["kind"] == "complete" and peer["case"] == case and peer["run"] == run and
            peer["failure"] is None and peer["cleanup"]["safe_shutdown"] is True, "peer failed or wrong identity")
    expected_air = [] if case == "component.radio_absent" else expected_tx[:positive]
    if case == "component.header_error_rearm":
        expected_air = [A, U]
        verify_header_error(events, peer, tx)
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
    if case == "component.invalid_downlinks":
        require(peer["layer"] == "Sx1262/LinuxRadioIo", "wrong invalid downlinks layer claim")
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

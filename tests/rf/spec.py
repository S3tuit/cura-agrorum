"""RF test names and whole-test reservations. Read this file to select a test.

Charges are conservative microseconds under SF7/BW125/CR4/5, explicit header,
eight-symbol preamble and CRC, rounded up with a 10% margin. Modelled airtimes
for 54/23/4/1-byte packets are 102656/61696/30976/25856 us. Include all setup,
retry and observation wakes; these reservations are not observed TX counts.
"""
from dataclasses import dataclass
import re

CHARGE_US = {54: 112922, 23: 67866, 4: 34074, 1: 28442}


@dataclass(frozen=True)
class Episode:
    name: str
    fixture: str
    c6_packets: int
    pi_lengths: tuple[int, ...]
    purpose: str
    runner: str = "component"
    wakes: int = 0
    seed_wakes: int = 0

    @property
    def charge(self):
        return {"c6_us": self.c6_packets * CHARGE_US[54],
                "pi_us": sum(CHARGE_US[n] for n in self.pi_lengths)}


# Component runner: run.py --cases NAME --fixture-state FIXTURE ...
# Node runner: run_ack.py --case NAME ...
# Installed-service runner: run_service.py ... (one explicit scenario).
# Node bounds reserve 70 attempts per wake, including failed reception.
# Component expectations are independently checked by verify.py and the peer.
CASES = {e.name: e for e in (
    Episode("component.ack_exchange", "nominal", 1, (23,), "One uplink and ACK"),
    Episode("component.ack_timeout", "nominal", 1, (), "ACK timeout with no downlink"),
    Episode("component.invalid_downlinks", "nominal", 1, (1, 4, 23), "Reject short/invalid downlinks within one RX deadline"),
    Episode("component.repeat_timeout", "nominal", 2, (), "Repeat TX/RX after timeout"),
    Episode("component.repeat_exchange", "nominal", 2, (23, 23), "Repeat TX/RX after exchange"),
    Episode("component.cold_sleep", "nominal", 1, (), "Sleep untouched radio twice, then transmit"),
    Episode("component.initialized_sleep", "nominal", 1, (), "Sleep initialized radio; reject later TX before SetTx"),
    Episode("component.sleep_wake", "nominal", 2, (23, 23), "Exchange across timer deep sleep and a fresh command"),
    Episode("component.dio1_disconnected", "dio1_disconnected", 1, (), "Bound uncertain TX when TxDone is not observed"),
    Episode("component.radio_absent", "radio_absent", 1, (), "Reserve one attempt; require initialization failure before SetTx"),
    Episode("node.current.accepted", "nominal", 210, (23,) * 4, "Accept current then seeded backlog", "node", 3, 1),
    Episode("node.current.retry_later", "nominal", 210, (23,) * 3, "Retain current after RETRY_LATER", "node", 3, 1),
    Episode("node.current.unsupported", "nominal", 210, (23,) * 3, "Quarantine unsupported current", "node", 3, 1),
    Episode("node.current.malformed", "nominal", 210, (23,) * 3, "Quarantine malformed current", "node", 3, 1),
    Episode("node.current.invalid_auth", "nominal", 210, (23,) * 5, "Ignore corrupt ACK then accept valid ACK", "node", 3, 1),
    Episode("node.current.wrong_message", "nominal", 210, (23,) * 3, "Ignore wrong-message ACK then observe silence", "node", 3, 1),
    Episode("node.current.domain_status", "nominal", 210, (23,) * 3, "Ignore mismatched ACK domain/status", "node", 3, 1),
    Episode("node.backlog.accepted", "nominal", 280, (23,) * 6, "Accept seeded backlog newest first", "node", 4, 2),
    Episode("node.backlog.retry_later", "nominal", 280, (23,) * 5, "Retain backlog after RETRY_LATER", "node", 4, 2),
    Episode("node.backlog.unsupported", "nominal", 280, (23,) * 6, "Quarantine unsupported backlog", "node", 4, 2),
    Episode("node.backlog.malformed", "nominal", 280, (23,) * 6, "Quarantine malformed backlog", "node", 4, 2),
    Episode("service.reading_delivery", "nominal", 140, (23,) * 140, "Persist two production wakes through the installed receiver", "service", 2),
)}
EPISODES = {name: case for name, case in CASES.items() if case.runner == "component"}
NODE_CASES = tuple(name for name, case in CASES.items() if case.runner == "node")

# Preparation-only rejection vectors: no integrated hardware runner yet.
# Each reserves one maximum uplink and at most one ACK (including shorter
# invalid frames). The value is (vector name, Pi ACK count).
REJECTION_CASES = {
    "rejection.implausible": ("implausible_reading", 1),
    "rejection.control": ("unsupported_control", 1),
    "rejection.domain": ("unknown_domain", 1),
    "rejection.body_length": ("malformed_length", 1),
    "rejection.flags": ("malformed_flags", 1),
    "rejection.direction": ("wrong_direction", 0),
    "rejection.control_direction": ("unsupported_control_wrong_direction", 1),
    "rejection.bad_tag": ("bad_tag", 0),
    "rejection.unknown": ("unknown_node", 0),
    "rejection.short_header": ("short_header", 0),
    "rejection.before_revoke": ("revocation_baseline", 1),
    "rejection.revoked": ("revoked_node", 0),
}
REJECTION_EPISODES = {name: Episode(name, "nominal", 1, (23,) * replies, vector, "preparation")
                      for name, (vector, replies) in REJECTION_CASES.items()}


def node_episode(case, sleep_seconds=900):
    if sleep_seconds not in (10, 900):
        raise ValueError("unreviewed RF sleep duration")
    if case not in NODE_CASES:
        raise ValueError("unknown node case")
    plan = CASES[case]
    _, scope, action = case.split(".")
    return dict(case=case, scope=scope, action=action, seed_wakes=plan.seed_wakes,
                wakes=plan.wakes, sleep_seconds=sleep_seconds,
                lease_seconds=(plan.wakes - 1) * 945 + 60 if sleep_seconds == 900 else plan.wakes * 50 + 10,
                c6_max_packets=plan.c6_packets, pi_max_packets=len(plan.pi_lengths),
                charges=plan.charge)


def service_episode():
    plan = CASES["service.reading_delivery"]
    return dict(case=plan.name, wakes=plan.wakes, lease_seconds=110, sleep_seconds=10,
                c6_max_packets=plan.c6_packets, pi_max_packets=len(plan.pi_lengths),
                charges=plan.charge, final_observation="agreed sleep-entry marker")


def select_cases(value, fixture):
    names = value.split(",") if value else []
    if not names or len(names) != len(set(names)):
        raise ValueError("select a nonempty, nonduplicated exact case/parameter list")
    if any(name not in EPISODES for name in names):
        raise ValueError("unknown or unimplemented RF selection")
    if any(EPISODES[name].fixture != fixture for name in names):
        raise ValueError("selection does not match the C6 fixture")
    if fixture != "nominal" and len(names) != 1:
        raise ValueError("manual fault selections must run alone")
    return [EPISODES[name] for name in names]


def validate_fixture(value):
    fields = {"schema", "c6_fixture", "c6_dut", "c6_uart", "pi_user", "pi_board_id", "pi_fixture",
              "rtc_shunts_open", "receiver_service_stopped", "exclusive_radios", "wiring_checked",
              "power_checked", "antennas_checked", "operator", "c6_transmitter", "pi_transmitter",
              "operating_envelope_reference", "manual_fault_confirmed"}
    if type(value) is not dict or set(value) != fields or type(value["schema"]) is not int or value["schema"] != 1:
        raise ValueError("invalid fixture schema/fields")
    for field in ("rtc_shunts_open", "receiver_service_stopped", "exclusive_radios", "wiring_checked",
                  "power_checked", "antennas_checked"):
        if value[field] is not True:
            raise ValueError(f"operator confirmation missing: {field}")
    for field in ("operator", "pi_board_id", "c6_transmitter", "pi_transmitter", "operating_envelope_reference"):
        if type(value[field]) is not str or not value[field].strip():
            raise ValueError(f"missing {field}")
    if (value["c6_fixture"] not in {"nominal", "dio1_disconnected", "radio_absent"}
            or value["pi_fixture"] != "radio_nominal"
            or not re.fullmatch(r"[0-9a-f]{12}", value["c6_dut"])
            or not re.fullmatch(r"[a-z_][a-z0-9_-]*", value["pi_user"])
            or value["pi_user"] == "root" or not value["c6_uart"].startswith("/dev/")):
        raise ValueError("invalid DUT identity, UART, user or fixture")
    if type(value["manual_fault_confirmed"]) is not bool or (value["c6_fixture"] != "nominal" and
                                                           not value["manual_fault_confirmed"]):
        raise ValueError("manual fault requires powered-off wiring confirmation")
    return value

"""Local selection and episode declarations. No device access or airtime ledger."""
from dataclasses import dataclass
import re


@dataclass(frozen=True)
class Episode:
    name: str
    fixture: str
    c6_packets: int
    pi_lengths: tuple[int, ...]

    @property
    def charge(self):
        # Independent fixed-profile values in EPISODES.md.
        times = {1: 28442, 4: 34074, 23: 67866}
        return {"c6_us": self.c6_packets * 112922,
                "pi_us": sum(times[n] for n in self.pi_lengths)}


EPISODES = {e.name: e for e in (
    Episode("RF-001.exchange", "nominal", 1, (23,)),
    Episode("RF-003.silence", "nominal", 1, ()),
    Episode("RF-006.invalid", "nominal", 1, (1, 4, 23)),
    Episode("RF-008.silence", "nominal", 2, ()),
    Episode("RF-008.exchange", "nominal", 2, (23, 23)),
    Episode("RF-009.untouched", "nominal", 1, ()),
    Episode("RF-009.initialized", "nominal", 1, ()),
    Episode("RF-010.wake", "nominal", 2, (23, 23)),
    Episode("RF-012.disconnected", "dio1_disconnected", 1, ()),
    Episode("RF-013.absent", "radio_absent", 1, ()),
)}


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

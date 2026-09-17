"""Explicit SX1262 fixture selection and source/evidence provenance; no device I/O."""

from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import pwd
import sys

import pytest

from cura_receiver.ports.radio import RadioConfiguration


def validate_selection(config, *, required_state=None):
    supplied = config.getoption("receiver_radio_fixture")
    evidence = config.getoption("receiver_radio_evidence")
    if not supplied or not evidence:
        raise pytest.UsageError("radio tests require --receiver-radio-fixture and --receiver-radio-evidence")
    try:
        path = Path(supplied)
        if not path.is_absolute():
            raise ValueError("fixture path must be absolute")
        value = json.loads(path.read_text())
        required = {"schema", "board_id", "service_user", "wiring_checked", "power_checked",
                    "no_pico_fitted", "receiver_service_stopped", "fixture_state",
                    "busy_selector_checked", "configuration"}
        if type(value) is not dict or set(value) != required or type(value["schema"]) is not int or value["schema"] != 2:
            raise ValueError("invalid radio fixture fields/schema")
        for field in ("wiring_checked", "power_checked", "no_pico_fitted", "receiver_service_stopped",
                      "busy_selector_checked"):
            if value[field] is not True:
                raise ValueError(f"operator confirmation missing: {field}")
        for field in ("board_id", "service_user"):
            if type(value[field]) is not str or not value[field].strip():
                raise ValueError(f"invalid {field}")
        if value["fixture_state"] not in ("radio_nominal", "radio_busy_held"):
            raise ValueError("unknown manual radio fixture state")
        if required_state is not None and value["fixture_state"] != required_state:
            raise ValueError(f"selected tests require fixture_state {required_state}")
        if type(value["configuration"]) is not dict:
            raise ValueError("invalid radio configuration")
        configuration = RadioConfiguration(**value["configuration"])
        if configuration != RadioConfiguration():
            raise ValueError("fixture requires the documented Pi carrier pin allocation")
        if os.geteuid() == 0 or pwd.getpwuid(os.geteuid()).pw_name != value["service_user"]:
            raise ValueError("run as the configured non-root receiver service user")
        root = Path(evidence)
        if not root.is_absolute() or root.exists() or not root.parent.is_dir():
            raise ValueError("radio evidence must be a new absolute directory below an existing parent")
        return value, configuration, root
    except (OSError, ValueError, TypeError, KeyError) as error:
        raise pytest.UsageError(f"invalid radio fixture: {error}") from error


def create_evidence(config):
    value, configuration, root = validate_selection(config)
    root.mkdir(mode=0o700)
    repo = Path(__file__).resolve().parents[3]
    paths = [* (repo / "receiver/cura_receiver").rglob("*.py"),
             * (repo / "receiver/tests").rglob("*.py"),
             * (repo / "protocol/protocol-v2-lora/python").rglob("*.py")]
    paths += [repo / "receiver/requirements-radio.txt", repo / "receiver/requirements-test.txt"]
    paths += [repo / "receiver" / name for name in (
        "ARCHITECTURE.md", "INTERFACE.md", "INTERFACE_DIAGNOSTIC.md", "TESTING.md",
        "pytest.ini", "schemas/receiver_enums.json", "schemas/receiver_entities.json", "db/schema.sql",
    )]
    paths.append(repo / "protocol/protocol-v2-lora/README.md")
    paths += [repo / "receiver/hardware/TEST_CARRIER.md", repo / "receiver/tests/hardware/RADIO_TESTS.md"]
    manifest = {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(paths)}
    (root / "sources.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (root / "fixture.json").write_text(json.dumps(value, indent=2) + "\n")
    (root / "target.json").write_text(json.dumps({
        "uname": list(os.uname()), "python": sys.version, "uid": os.geteuid(),
        "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "configuration": asdict(configuration),
    }, indent=2) + "\n")
    return root, configuration

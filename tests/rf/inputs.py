"""Build/flash guards, source staging inputs and temporary RF prerequisites."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

from evidence import REPO, digest, write_json
APP = REPO / "firmware/test_apps/radio"


def tree_sources():
    paths = []
    for name in ("firmware/components", "firmware/test_apps/radio/main", "receiver/cura_receiver",
                 "receiver/test_apps/radio_peer", "receiver/tests", "receiver/schemas", "receiver/db",
                 "receiver/deploy", "receiver/native", "receiver/tools",
                 "protocol/protocol-v2-lora/python", "tests/rf"):
        paths.extend(p for p in (REPO / name).rglob("*") if p.is_file() and
                     not any(part in {"__pycache__", "runs", "raw", ".pytest_cache", "evidence"} for part in p.parts) and
                     (p.name.startswith("Kconfig") or p.suffix in {".c", ".h", ".py", ".md", ".txt", ".cmake", ".yml", ".json", ".ini", ".sql", ".service", ".conf", ".rules"}) and
                     not p.name.startswith(("WORKPLAN", "REVIEW")))
    paths.extend(APP / name for name in ("CMakeLists.txt", "sdkconfig.defaults", "partitions.csv", "dependencies.lock"))
    paths.extend(REPO / name for name in ("Makefile", "firmware/TESTING.md", "firmware/INTERFACE.md",
        "firmware/ARCHITECTURE.md", "receiver/ARCHITECTURE.md", "receiver/INTERFACE.md", "receiver/TESTING.md",
        "receiver/INTERFACE_DIAGNOSTIC.md", "receiver/requirements-runtime.txt", "receiver/requirements-radio.txt", "receiver/requirements-test.txt", "receiver/pytest.ini",
        "receiver/hardware/ds3231/chrony-runtime.conf",
        "receiver/hardware/TEST_CARRIER.md", "receiver/tests/hardware/RADIO_TESTS.md",
        "protocol/protocol-v2-lora/README.md", "deployment_remaining.notes.md"))
    return {str(p.relative_to(REPO)): digest(p) for p in sorted(set(paths))}


def firmware_sources(build, app=APP):
    """Hash the actual compiler dependencies, including IDF and fetched headers."""
    result = subprocess.run(["ninja", "-C", str(build), "-t", "deps"], check=True,
                            capture_output=True, text=True)
    files = set()
    for line in result.stdout.splitlines():
        if line.startswith("    "):
            path = Path(line.strip())
            path = path if path.is_absolute() else build / path
            if path.is_file():
                files.add(path.resolve())
    commands = json.loads((build / "compile_commands.json").read_text())
    files.update(Path(entry["file"]).resolve() for entry in commands)
    project = json.loads((build / "project_description.json").read_text())
    files.add(Path(project["config_file"]).resolve())
    for paths in project.get("config_environment", {}).values():
        files.update(Path(name).resolve() for name in paths.split(";") if name)
    files.update(app / name for name in ("CMakeLists.txt", "main/CMakeLists.txt",
                 "sdkconfig.defaults", "partitions.csv", "dependencies.lock"))
    main = "main/radio_app.c" if app == APP else "main/app_main.c"
    if app / main not in files or len(files) < 100:
        raise ValueError("missing actual build dependency records")
    return {str(p): digest(p) for p in sorted(files)}


def check_flash(build, application="cura_radio_component"):
    if application not in {"cura_radio_component", "cura_agrorum_firmware"}:
        raise ValueError("unreviewed application")
    args = json.loads((build / "flasher_args.json").read_text())
    expected = {"0x0": "bootloader/bootloader.bin", "0x8000": "partition_table/partition-table.bin",
                "0x10000": application + ".bin"}
    if args["flash_files"] != expected or args["extra_esptool_args"]["chip"] != "esp32c6":
        raise ValueError("unapproved flash files or target")
    limits = {"0x0": 0x8000, "0x8000": 0x1000, "0x10000": 0x100000}
    for offset, name in expected.items():
        if not 0 < (build / name).stat().st_size <= limits[offset]:
            raise ValueError("flash file overlaps retained storage")
    # Check the actual binary partition table, not only its source labels.
    import struct
    data = (build / expected["0x8000"]).read_bytes()
    partitions = []
    for offset in range(0, len(data), 32):
        block = data[offset:offset + 32]
        if len(block) != 32 or block[:2] != b"\xaa\x50":
            break
        _, kind, subtype, start, size, label, flags = struct.unpack("<HBBII16sI", block)
        partitions.append((kind, subtype, start, size, label.rstrip(b"\0").decode(), flags))
    if partitions != [(1, 2, 0x9000, 0x6000, "nvs", 0), (1, 1, 0xf000, 0x1000, "phy_init", 0),
                      (0, 0, 0x10000, 0x100000, "factory", 0), (1, 131, 0x110000, 0x2e0000, "storage", 0)]:
        raise ValueError("binary partition table differs from reviewed physical layout")
    config = json.loads((build / "config/sdkconfig.json").read_text())
    pins = dict(SCLK=6, MOSI=7, MISO=14, CS=23, RESET=18, BUSY=19, DIO1=20)
    if any(config.get(f"CURA_SX1262_{name}_GPIO") != pin for name, pin in pins.items()):
        raise ValueError("resolved C6 radio pins differ")
    if config.get("ESP_CONSOLE_UART_NUM") != 0 or config.get("ESP_CONSOLE_UART_BAUDRATE") != 115200:
        raise ValueError("wrong resolved UART console")
    return {name: digest(build / name) for name in (*expected.values(), application + ".elf",
              "flasher_args.json", "config/sdkconfig.json", "compile_commands.json", "project_description.json")}


def seal_build(build):
    build = Path(build).resolve()
    value = dict(schema=1, dependencies=firmware_sources(build), files=check_flash(build))
    write_json(build / "rf-build.json", value)
    return value


def verify_build(build):
    build = Path(build).resolve()
    value = json.loads((build / "rf-build.json").read_text())
    if value != dict(schema=1, dependencies=firmware_sources(build), files=check_flash(build)):
        raise ValueError("source/build seal mismatch; rebuild and seal before device access")
    return value


def source_manifest():
    return dict(schema=1, files=tree_sources(), head=subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=REPO, check=True, capture_output=True, text=True).stdout.strip())


def session_identity(manifest, seal, fixture):
    # Documentation changes cannot change executable test behavior. Build sealing
    # separately includes actual compiler dependencies and resolved configuration.
    files = {name: value for name, value in manifest["files"].items()
             if not name.endswith(".md")}
    return dict(source=hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest(),
                build=hashlib.sha256(json.dumps(seal, sort_keys=True).encode()).hexdigest(),
                devices={key: fixture[key] for key in
                         ("c6_dut", "pi_board_id", "c6_transmitter", "pi_transmitter")})


def start_session(path, identity, selected):
    """Admit prerequisites before staging; invalidate the receipt during execution.

    This is disposable local session state, never historical qualification.
    One owner runs the two devices sequentially. Start a new file each bench session.
    """
    path = Path(path)
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    now = time.monotonic()
    state = json.loads(path.read_text()) if path.exists() else {}
    if path.exists() and (not isinstance(state, dict) or
                         set(state) != {"schema", "identity", "boot", "started", "passed"} or
                         state["schema"] != 1 or type(state["started"]) not in (int, float) or
                         not isinstance(state["passed"], list) or
                         any(name not in {"RF-001.exchange", "RF-003.silence"} for name in state["passed"])):
        raise ValueError("not an RF session receipt; choose a new temporary file")
    valid = (state.get("identity") == identity and state.get("boot") == boot and
             0 <= now - state.get("started", -43201) <= 43200)
    passed = set(state.get("passed", [])) if valid else set()
    planned = set(passed)
    for name in selected:
        required = set() if name in {"RF-001.exchange", "RF-009.untouched", "RF-009.initialized"} else {"RF-001.exchange"}
        if name.startswith("RF-008."):
            required.add("RF-003.silence")
        if not required <= planned:
            raise ValueError(f"{name} requires fresh nominal prerequisites {sorted(required - planned)}")
        planned.add(name)
    state = dict(schema=1, identity=identity, boot=boot,
                 started=state["started"] if valid else now, passed=[])
    write_json(path, state)
    return state, passed


def finish_session(path, state, passed, run):
    # Failed/interrupted runs and all fault runs require nominal requalification.
    if run["status"] == "PASS" and run["fixture"]["c6_fixture"] == "nominal":
        state["passed"] = sorted((passed | set(run["selected"])) &
                                 {"RF-001.exchange", "RF-003.silence"})
    write_json(path, state)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seal-build", type=Path, required=True)
    seal_build(parser.parse_args().seal_build)

"""Pi-local read-only RF-020 service inspection and SQLite capture.

Administrative inspection runs separately from database access under the actual
service UID. This helper never installs packages, starts services or touches RF.
"""
import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import pwd
import re
import sqlite3
import stat
import subprocess
import sys
import time


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_config(value):
    if set(value) != {"schema", "unit", "package", "test_root", "user", "files"} or value["schema"] != 1:
        raise ValueError("invalid service configuration")
    if not re.fullmatch(r"cura-pilot-[a-z0-9-]+\.service", value["unit"]) or value["user"] != "cura-receiver":
        raise ValueError("not an isolated production service")
    for name, prefix in (("package", "/opt/cura-pilot-"), ("test_root", "/var/lib/cura-pilot-")):
        if not re.fullmatch(re.escape(prefix) + "[a-z0-9-]+", value[name]):
            raise ValueError("unreviewed service path")
    if not value["files"] or "receiver/cura_receiver/__main__.py" not in value["files"]:
        raise ValueError("missing production package manifest")
    for name, expected in value["files"].items():
        if Path(name).is_absolute() or ".." in Path(name).parts or not re.fullmatch("[0-9a-f]{64}", expected):
            raise ValueError("invalid package manifest entry")
    return value


def checked_path(path):
    path = Path(path)
    if not path.is_absolute() or path.resolve() != path or any(p.is_symlink() for p in (path, *path.parents)):
        raise ValueError("symlink/aliased deployment path")
    return path


def expected_unit(config, template):
    root, package = config["test_root"], config["package"]
    bootstrap = config["unit"].replace("cura-pilot-", "cura-pilot-rtc-", 1)
    return (template.replace("cura-rtc-bootstrap.service", bootstrap)
            .replace("/etc/cura-agrorum", root + "/config")
            .replace("/var/lib/cura-agrorum", root + "/data")
            .replace("/opt/cura-agrorum", package))


def dependencies():
    result = {name: version(name) for name in ("cryptography", "gpiod", "spidev")}
    if result["cryptography"] != "49.0.0":
        raise ValueError("unqualified crypto version")
    for name, minimum, maximum in (("gpiod", (2, 2), 3), ("spidev", (3, 8), 4)):
        parts = tuple(int(p) for p in result[name].split(".")[:2])
        if not minimum <= parts or parts[0] >= maximum:
            raise ValueError("unqualified radio dependency")
    return result


def inspect(config):
    config = validate_config(config)
    unit, package, root = config["unit"], Path(config["package"]), Path(config["test_root"])
    for path in (package, root, root / "config", root / "data", root / "data/tmp"):
        checked_path(path)
    for name, expected in config["files"].items():
        path = checked_path(package / name)
        info = path.stat()
        if (info.st_mode & 0o002 or (info.st_mode & 0o020 and info.st_gid != 0) or digest(path) != expected):
            raise ValueError("installed package differs: " + name)
    unit_path = checked_path(Path("/etc/systemd/system") / unit)
    template = (Path(__file__).resolve().parents[2] / "receiver/deploy/systemd/cura-receiver.service").read_text()
    if unit_path.read_text() != expected_unit(config, template):
        raise ValueError("installed unit differs from isolated production template")
    names = ("ActiveState", "SubState", "MainPID", "User", "Group", "FragmentPath",
             "DropInPaths", "NeedDaemonReload", "NRestarts", "InvocationID", "Result", "ExecMainStatus", "ExecMainCode", "ControlGroup")
    output = subprocess.check_output(["systemctl", "show", unit, *["--property=" + n for n in names]], text=True)
    state = dict(line.split("=", 1) for line in output.splitlines())
    if (state["User"] != config["user"] or state["Group"] != config["user"] or
            state["FragmentPath"] != str(unit_path) or state["DropInPaths"] or state["NeedDaemonReload"] != "no"):
        raise ValueError("wrong effective service ownership/unit")
    if state["ActiveState"] == "active":
        if Path("/proc", state["MainPID"]).stat().st_uid != pwd.getpwnam(config["user"]).pw_uid:
            raise ValueError("actual running process has wrong UID")
    environment_path = checked_path(root / "config/deployment.env")
    group_path = checked_path(root / "config/receiver-group.json")
    if group_path.stat().st_uid != pwd.getpwnam(config["user"]).pw_uid or stat.S_IMODE(group_path.stat().st_mode) != 0o600:
        raise ValueError("wrong group file owner/mode")
    environment = {}
    for line in environment_path.read_text().splitlines():
        if line and not line.startswith("#"):
            name, value = line.split("=", 1)
            if name in environment:
                raise ValueError("duplicate deployment environment")
            environment[name] = value
    expected = dict(CURA_RECEIVER_TEST_ROOT=str(root),
                    CURA_RECEIVER_CONFIGURATION=str(root / "config/receiver-group.json"),
                    CURA_RECEIVER_DATABASE=str(root / "data/receiver.sqlite3"),
                    SQLITE_TMPDIR=str(root / "data/tmp"))
    if (set(environment) != {*expected, "RTC_HELPER_SHA256", "RTC_KERNEL_BOUND_US"} or
            any(environment.get(k) != v for k, v in expected.items()) or
            not re.fullmatch("[0-9a-f]{64}", environment["RTC_HELPER_SHA256"]) or
            not environment["RTC_KERNEL_BOUND_US"].isdigit()):
        raise ValueError("unreviewed service environment")
    state.update(environment_sha256=digest(environment_path), unit_sha256=digest(unit_path),
                 configuration_sha256=digest(group_path), dependencies=dependencies(),
                 board_id=Path("/proc/device-tree/serial-number").read_bytes().rstrip(b"\0").decode(),
                 boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip())
    return state


def stopped_state(config):
    state = inspect(config)
    group = Path('/sys/fs/cgroup/system.slice') / config['unit']
    state['remaining_pids'] = [int(pid) for path in group.rglob('cgroup.procs')
                               for pid in path.read_text().split()]
    reply = json.loads(subprocess.check_output([
        'busctl', '--json=short', 'call', 'org.freedesktop.login1',
        '/org/freedesktop/login1', 'org.freedesktop.login1.Manager', 'ListInhibitors'], text=True))
    if reply['type'] != 'a(ssssuu)' or len(reply['data']) != 1:
        raise ValueError('unrecognized inhibitor response')
    uid = pwd.getpwnam(config['user']).pw_uid
    state['receiver_inhibitors'] = [row for row in reply['data'][0]
                                    if row[4] == uid or row[1] == config['user']]
    return state


def database_observation(database):
    """One read transaction; never infer readiness from systemd active alone."""
    with sqlite3.connect(checked_path(database).as_uri() + "?mode=ro", uri=True, timeout=1) as db:
        db.execute("BEGIN")
        db.row_factory = sqlite3.Row
        total_readings = db.execute("SELECT count(*) FROM reading_messages").fetchone()[0]
        instance = db.execute("SELECT * FROM receiver_instances ORDER BY instance_ordinal DESC LIMIT 1").fetchone()
        if instance is None:
            return dict(instance=None, health=None, profiles=0, samples=0, total_readings=total_readings)
        iid = instance["receiver_instance_id"]
        health = db.execute("SELECT radio_state_id,communicator_sampled_at_monotonic_us FROM receiver_health "
                            "WHERE receiver_instance_id=? ORDER BY health_sequence DESC LIMIT 1", (iid,)).fetchone()
        profiles = db.execute("SELECT count(*) FROM message_profiles WHERE receiver_instance_id=?", (iid,)).fetchone()[0]
        samples = db.execute("SELECT count(*) FROM reading_messages WHERE first_receiver_instance_id=?", (iid,)).fetchone()[0]
        latest = db.execute("SELECT max(received_at_monotonic_us) FROM message_profiles WHERE receiver_instance_id=?", (iid,)).fetchone()[0]
        clock = db.execute("SELECT clock_state_generation,system_time_quality_id,sampled_at_monotonic_us "
                           "FROM clock_observations WHERE receiver_instance_id=? "
                           "ORDER BY observation_sequence DESC LIMIT 1", (iid,)).fetchone()
        from cura_receiver.application_settings import ApplicationSettings
        from cura_receiver.communicator_state_persistence import classify_communicator_state_rows
        from cura_receiver.sqlite_repository import SqliteRepository
        repository = SqliteRepository(db)
        loaded = classify_communicator_state_rows(repository.read_communicator_state_rows(),
                                                  repository, ApplicationSettings().airtime_policy)
        airtime = dict(status=loaded.status.name, condition=loaded.state_condition.name,
                       total_charged_us=None, budget_us=None)
        if loaded.state is not None:
            airtime.update(total_charged_us=sum(b.charged_airtime_us for b in loaded.state.buckets),
                           budget_us=loaded.state.tx_airtime_budget_us)
        generations = db.execute("SELECT generation FROM communicator_state").fetchall()
        generation = generations[0][0] if len(generations) == 1 else (0 if not generations else None)
        return dict(instance=iid.hex(), ordinal=instance["instance_ordinal"],
                    started=instance["started_at_monotonic_us"],
                    clean_generation=instance["clean_stop_state_generation"], state_generation=generation,
                    linux_boot_id=instance["linux_boot_id"].hex(),
                    clean_stop=instance["clean_stopped_at_monotonic_us"],
                    clock=dict(clock) if clock else None, airtime=airtime,
                    health=dict(health) if health else None, profiles=profiles, samples=samples, total_readings=total_readings,
                    latest_packet_us=latest, observed_monotonic_us=time.monotonic_ns() // 1000)


def snapshot(database, destination):
    destination = Path(destination)
    # Exclusive creation prevents replacing an earlier capture.
    with destination.open("xb"):
        pass
    deadline = time.monotonic() + 10
    def progress(*_):
        if time.monotonic() > deadline:
            raise TimeoutError("SQLite snapshot deadline")
    with sqlite3.connect(checked_path(database).as_uri() + "?mode=ro", uri=True, timeout=1) as source:
        with sqlite3.connect(destination) as target:
            source.backup(target, pages=128, progress=progress, sleep=0.05)
            if target.execute("PRAGMA integrity_check").fetchone() != ("ok",) or target.execute("PRAGMA foreign_key_check").fetchall():
                raise ValueError("inconsistent SQLite snapshot")
    return dict(sha256=digest(destination), bytes=destination.stat().st_size)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("action", choices=("inspect", "stopped", "observe", "snapshot", "prepare-zero", "reading-baseline"))
    parser.add_argument("--node-id")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--silence-record", type=Path)
    args = parser.parse_args()
    config = validate_config(json.loads(args.config.read_text()))
    stage = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(stage / "receiver"), str(stage / "protocol/protocol-v2-lora/python")]
    manifest = json.loads((stage / "source-manifest.json").read_text())
    for name, expected in manifest["files"].items():
        if Path(name).is_absolute() or ".." in Path(name).parts or digest(checked_path(stage / name)) != expected:
            raise ValueError("staged sources differ")
    if any(manifest["files"].get(name) != expected for name, expected in config["files"].items()):
        raise ValueError("package expectation differs from current staged tree")
    if args.action in ("inspect", "stopped"):
        result = inspect(config) if args.action == "inspect" else stopped_state(config)
    else:
        if os.geteuid() != pwd.getpwnam(config["user"]).pw_uid:
            raise ValueError("database observation must run as actual service UID")
        database = Path(config["test_root"]) / "data/receiver.sqlite3"
        if args.action == "prepare-zero":
            sys.path[:0] = [str(stage / "receiver"), str(stage / "protocol/protocol-v2-lora/python")]
            from service_preparation import create_zero_airtime_database
            from cura_protocol_v2_lora.receiver_group import load_receiver_group
            state = stopped_state(config)
            if (state['ActiveState'], state['SubState'], state['MainPID']) != ('inactive', 'dead', '0') or state['remaining_pids'] or state['receiver_inhibitors']:
                raise ValueError('test preparation requires a stopped empty service')
            if args.output is None or checked_path(args.output.parent) != database.parent:
                raise ValueError('prepared candidate must be in isolated data directory')
            group = load_receiver_group(Path(config['test_root']) / 'config/receiver-group.json')
            result = create_zero_airtime_database(args.output, group.group_id,
                json.loads(args.silence_record.read_text()), board_id=state['board_id'],
                boot_id=state['boot_id'], now_monotonic_us=time.monotonic_ns() // 1000,
                utc_us=time.time_ns() // 1000)
        elif args.action == "reading-baseline":
            from reading_baseline import capture_readings
            result = capture_readings(database, bytes.fromhex(args.node_id))
        else:
            result = database_observation(database) if args.action == "observe" else snapshot(database, args.output)
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()

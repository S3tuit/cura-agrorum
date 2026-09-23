"""RF-020: an already installed production service and production node.

No installation, provisioning, flash writes or implicit scenario selection.
"""
import argparse
import json
import os
from pathlib import Path
import re
import runpy
import shlex
import shutil
import sys
import tempfile
import time

from evidence import REPO, admit_episode, digest, episode_capture, write_json
sys.path.insert(0, str(REPO / "receiver"))
sys.path.insert(0, str(REPO / "protocol/protocol-v2-lora/python"))
from cura_protocol_v2_lora.receiver_group import load_receiver_group
from cura_receiver.application import authentication_keys
from inputs import source_manifest
from node_capture import build_reader
from production_node import NodeUART, capture_storage, identify, verify_build, verify_installed
from service_probe import validate_config
from spec import validate_fixture
from transport import RemoteTransport
from verify_service import verify_service
from sleep_observation import sleep_count

EPISODE = dict(case="RF-020", wakes=2, lease_seconds=110, sleep_seconds=10,
               c6_max_packets=140, pi_max_packets=140,
               final_observation="agreed sleep-entry marker")


def package_files():
    with tempfile.TemporaryDirectory(prefix="cura-rf-package-") as temporary:
        target = Path(temporary) / "package"
        runpy.run_path(str(REPO / "receiver/tools/build_runtime_bundle.py"))["build"](target)
        return json.loads((target / "SOURCE_MANIFEST.json").read_text())["files_sha256"]


def same_service(before, current, *, running=None):
    for key in ("unit_sha256", "environment_sha256", "configuration_sha256", "boot_id"):
        if current[key] != before[key]:
            raise ValueError("service/configuration/boot changed: " + key)
    if running is not None and any(current[k] != running[k] for k in ("InvocationID", "MainPID", "NRestarts")):
        raise ValueError("service restarted during episode")
    if running is not None and (current["ActiveState"], current["SubState"]) != ("active", "running"):
        raise ValueError("service stopped during episode")


def verify_service_stop(running, instance, state, database):
    """Both process termination and this instance's durable shutdown are required."""
    if running is None or instance is None:
        raise ValueError('cannot verify clean stop without run binding')
    same_service(running, state)
    if (not running['InvocationID'] or state['InvocationID'] not in ('', running['InvocationID'])
            or state['NRestarts'] != running['NRestarts']):
        raise ValueError('service invocation changed before stop')
    if (state['ActiveState'], state['SubState'], state['MainPID'], state['Result']) != ('inactive', 'dead', '0', 'success'):
        raise ValueError('service did not restore stopped state')
    if ((state['ExecMainCode'], state['ExecMainStatus']) not in (('1', '0'), ('2', '15'))
            or state['ControlGroup'] or state['remaining_pids'] or state['receiver_inhibitors']):
        raise ValueError('unexpected wrapper exit or remaining service resources')
    if (any(database.get(k) != instance.get(k) for k in ('instance', 'ordinal', 'linux_boot_id', 'started'))
            or database.get('linux_boot_id') != running['boot_id'].replace('-', '')
            or type(database.get('clean_stop')) is not int
            or database['clean_stop'] < instance['started']
            or type(database.get('clean_generation')) is not int
            or database['clean_generation'] < 0
            or database['clean_generation'] != database.get('state_generation')):
        raise ValueError('missing or mismatched receiver clean-stop record')
    return dict(state=state, database=database, invocation=running['InvocationID'])


def service_prerequisite_reasons(observed):
    """Observable test prerequisites, never a claim about the RAM-only grant."""
    reasons = []
    now = observed['observed_monotonic_us']
    health, clock, airtime = (observed.get(k) for k in ('health', 'clock', 'airtime'))
    if (health is None or health['radio_state_id'] != 2 or
            not 0 <= now - health['communicator_sampled_at_monotonic_us'] <= 75_000_000):
        reasons.append('fresh_RX_SINGLE_health_required')
    if (clock is None or clock['system_time_quality_id'] != 2 or
            clock['clock_state_generation'] < 1 or
            not observed['started'] <= clock['sampled_at_monotonic_us'] <= now or
            now - clock['sampled_at_monotonic_us'] > 60_000_000):
        reasons.append('fresh_NETWORK_SYNCED_observation_required')
    if airtime is None or airtime['status'] != 'LOADED':
        reasons.append('validated_airtime_history_required')
    elif not 0 <= airtime['total_charged_us'] < airtime['budget_us']:
        reasons.append('conservative_airtime_budget_has_no_headroom')
    return reasons


class InstalledService:
    def __init__(self, remote, config):
        self.remote, self.config = remote, config
        self.python = config["package"] + "/venv/bin/python"
        self.script = remote.remote + "/tests/rf/service_probe.py"
        self.config_path = remote.remote + "/service-config.json"
        self.stopped = False
        self.invocation = None
        self.running = None
        self.instance = None
        self.prior = None

    def admin(self, command, timeout=30):
        # Password goes through stdin only, never a command/capture field.
        password = os.environ.get("CURA_PI_SUDO_PASSWORD", self.remote.env.get("SSHPASS"))
        prefix = ["sudo", "-S", "-p", ""] if password else ["sudo", "-n"]
        return self.remote.command(shlex.join([*prefix, "--", "sh", "-c", command]),
                                   timeout, input=password + "\n" if password else None)

    def probe(self, action, *extra):
        argv = [self.python, self.script, self.config_path, action, *extra]
        if action not in ("inspect", "stopped"):
            argv = ["sudo", "-u", self.config["user"], "--", *argv]
        return json.loads(self.admin(shlex.join(argv)).stdout)

    def prepare(self):
        self.remote.command("printf %s " + shlex.quote(json.dumps(self.config)) + " > " + shlex.quote(self.config_path))
        # Staged sources/configuration are public; credentials remain elsewhere.
        self.remote.command("chmod 755 " + shlex.quote(self.remote.remote))
        self.remote.command(shlex.join(["chmod", "644", self.config_path, self.remote.remote + "/source-manifest.json"]))
        self.admin(shlex.join(["install", "-d", "-m", "700", "-o", self.config["user"], self.remote.remote + "/captures"]))
        state = self.probe("inspect")
        if (state["ActiveState"], state["SubState"], state["MainPID"]) != ("inactive", "dead", "0"):
            raise ValueError("service must be stopped before RF-020")
        return state

    def start(self):
        self.prior = self.probe('observe')
        self.admin(shlex.join(["systemctl", "start", self.config["unit"]]))
        self.running = self.probe('inspect')
        self.invocation = self.running['InvocationID']
        if not self.invocation or self.running['NRestarts'] != '0':
            raise ValueError('service start lacks a unique invocation')

    def bind_instance(self, observed):
        if (self.prior is None or observed['instance'] is None
                or observed['instance'] == self.prior['instance']
                or observed['ordinal'] != self.prior.get('ordinal', 0) + 1
                or observed['linux_boot_id'] != self.running['boot_id'].replace('-', '')
                or observed['clean_stop'] is not None):
            raise ValueError('cannot bind current receiver instance')
        if self.instance is not None and self.instance != observed:
            raise ValueError('receiver instance binding already established')
        self.instance = observed

    def wait_for_prerequisites(self, *, timeout_seconds=45):
        if not 0 < timeout_seconds <= 60:
            raise ValueError('bounded prerequisite timeout required')
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            state = self.probe('inspect')
            same_service(self.running, state, running=self.running)
            observed = self.probe('observe')
            if self.instance is None:
                if observed['instance'] is None or observed['instance'] == self.prior['instance']:
                    write_json(self.remote.root / 'service-prerequisites.json',
                               dict(database=observed, reasons=['new_instance_required']))
                    time.sleep(1)
                    continue
                self.bind_instance(observed)
            elif any(observed.get(k) != self.instance.get(k)
                     for k in ('instance', 'ordinal', 'linux_boot_id', 'started')) or observed['clean_stop'] is not None:
                raise ValueError('receiver lifecycle changed during prerequisites')
            if observed['profiles'] or observed['samples']:
                raise ValueError('unexpected traffic before node start')
            reasons = service_prerequisite_reasons(observed)
            write_json(self.remote.root / 'service-prerequisites.json',
                       dict(database=observed, reasons=reasons,
                            scope='preconditions only; not live grant or ACK delivery proof'))
            if not reasons:
                return observed
            time.sleep(1)
        raise TimeoutError('service prerequisites incomplete; see service-prerequisites.json')

    def stop(self):
        if self.stopped:
            return
        # Always attempt the actual stop, even when pre-stop observation fails.
        try:
            state = self.probe('inspect')
            if self.running is None:
                raise ValueError('service stop has no run binding')
            same_service(self.running, state, running=self.running)
            if self.instance is None:
                self.bind_instance(self.probe('observe'))
        finally:
            self.admin(shlex.join(["systemctl", "stop", self.config["unit"]]))
        state = self.probe('stopped')
        database = self.probe('observe')
        report = dict(state=state, database=database, invocation=self.invocation)
        write_json(self.remote.root / 'service-stop.json', report)
        verify_service_stop(self.running, self.instance, state, database)
        self.stopped = True

    def capture_journal(self):
        if self.invocation:
            result = self.admin(shlex.join(["journalctl", "--no-pager", "-o", "short-monotonic",
                                            "_SYSTEMD_INVOCATION_ID=" + self.invocation]))
            (self.remote.root / "service-journal.txt").write_text(result.stdout)

    def capture_database(self):
        if not (self.remote.root / "receiver.sqlite3").exists():
            return self.snapshot()
        return self.remote.root / "receiver.sqlite3"

    def snapshot(self):
        source = self.remote.remote + "/captures/receiver.sqlite3"
        result = self.probe("snapshot", "--output", source)
        export = self.remote.remote + "/receiver.sqlite3"
        self.admin(shlex.join(["install", "-m", "600", "-o", self.remote.user, source, export]))
        destination = self.remote.root / "receiver.sqlite3"
        self.remote.fetch(export, destination)
        if digest(destination) != result["sha256"]:
            raise ValueError("SQLite snapshot transfer mismatch")
        write_json(self.remote.root / "snapshot.json", result)
        return destination


def run(args):
    fixture = validate_fixture(json.loads(args.fixture.read_text()))
    if fixture["c6_fixture"] != "nominal" or not args.confirm_isolated:
        raise ValueError("nominal isolated production fixture required")
    if not re.fullmatch("[0-9a-f]{32}", args.run) or args.ready_run not in (None, args.run):
        raise ValueError("invalid run/readiness identity")
    if not args.output.is_absolute() or args.output.exists() or not args.output.parent.is_dir():
        raise ValueError("choose a new absolute capture directory")
    for record in (args.manual_record, args.prerequisites):
        if not record.is_file() or not record.read_bytes().strip():
            raise ValueError("missing operator/prerequisite record")
    seal = verify_build(args.build)
    if seal.get("sleep_seconds") != 10 or not seal.get("sleep_observation"):
        raise ValueError("RF-020 requires observed accelerated10-second build")
    node_id = bytes.fromhex(seal["node_id"])
    group = load_receiver_group(args.local_group)
    keys = authentication_keys(group)
    if set(keys) != {node_id}:
        raise ValueError("RF-020 requires exactly the selected disposable node in its group")
    config = validate_config(dict(schema=1, unit=args.unit, package=args.package,
        test_root=args.test_root, user="cura-receiver", files=package_files()))
    root = args.output
    root.mkdir(mode=0o700)
    manifest = source_manifest()
    write_json(root / "source-manifest.json", manifest)
    write_json(root / "service-config.json", config)
    write_json(root / "production-build.json", seal)
    write_json(root / "episode.json", EPISODE)
    shutil.copyfile(args.manual_record, root / "operator-airtime-record.txt")
    shutil.copyfile(args.prerequisites, root / "prerequisites.txt")
    reader = root / "node-image-reader"
    write_json(root / "decoder-sources.json", build_reader(reader))
    record = dict(schema=1, run=args.run, selected=["RF-020"], status="INCOMPLETE", failures=[], results=[], fixture=fixture)
    remote = RemoteTransport(args.host, fixture["pi_user"], args.run, root, args.host_key_alias)
    with episode_capture(record, "RF-020", root, root) as cleanups:
        remote.stage(manifest, fixture)
        service = InstalledService(remote, config)
        before = service.prepare()
        if before["board_id"] != fixture["pi_board_id"]:
            raise ValueError("wrong physical Pi")
        if before["configuration_sha256"] != digest(args.local_group):
            raise ValueError("installed and local isolated group files differ")
        prior = service.probe("observe")
        reading_baseline = service.probe("reading-baseline", "--node-id", node_id.hex())
        write_json(root / "reading-baseline.json", reading_baseline)
        write_json(root / "service-before.json", dict(state=before, database=prior))
        print(json.dumps(EPISODE, indent=2), flush=True)
        admit_episode(args.run, "RF-020", root, root / "operator-airtime-record.txt", args.ready_run)
        verify_installed(fixture, args.build, root)
        baseline = capture_storage(fixture, root, "before", reader, seal, args.run)
        if any(baseline["logs"].values()):
            raise ValueError("RF-020 requires separately initialized empty node logs")
        # Reverse cleanup order stops the node/service before capturing database.
        cleanups.append(("consistent receiver snapshot", service.capture_database))
        cleanups.append(("service journal", service.capture_journal))
        cleanups.append(("production service stop", service.stop))
        service.start()
        ready = service.wait_for_prerequisites()
        running = service.running
        write_json(root / "service-ready.json", dict(state=running, database=ready))
        node = NodeUART(fixture["c6_uart"], root)
        node_closed = node_stopped = False
        def stop_node():
            nonlocal node_closed, node_stopped
            if node_stopped:
                return
            error = None
            if not node_closed:
                try:
                    node.close()
                except BaseException as e:
                    error = e
                node_closed = True
            try:
                identify(fixture, root, "final-stop-mac")
                node_stopped = True
            except BaseException:
                write_json(root / "node-restoration-required.json", dict(action="remove all node power; autonomous TX unconfirmed"))
                raise
            if error is not None:
                raise error
        cleanups.append(("autonomous node stop", stop_node))
        node.start()
        deadline = time.monotonic() + EPISODE["lease_seconds"]
        while time.monotonic() < deadline:
            if node.failure is not None:
                raise RuntimeError("UART capture failed: " + str(node.failure))
            same_service(before, service.probe("inspect"), running=running)
            observed = service.probe("observe")
            if observed["instance"] != ready["instance"] or observed["clean_stop"] is not None:
                raise ValueError("service lifecycle changed")
            if observed["samples"] > 2 or observed["profiles"] > EPISODE["pi_max_packets"]:
                raise ValueError("extra wake or episode ceiling exceeded")
            markers = sleep_count((root / "c6-uart.bin").read_bytes(), 10, 2)
            if observed["samples"] == 2 and markers == 2:
                write_json(root / "sleep-observation.json", dict(run=args.run, case="RF-020",
                    wakes=2, sleep_seconds=10, boundary="cycle finalized, timer configured, entering sleep"))
                break
            time.sleep(2)
        else:
            raise TimeoutError("complete production wake pair not observed")
        stop_node()
        if sleep_count((root / "c6-uart.bin").read_bytes(), 10, 2) != 2:
            raise ValueError("incomplete final UART observation")
        service.stop()
        write_json(root / "service-after.json", service.probe("inspect"))
        service.capture_journal()
        database = service.capture_database()
        captured = capture_storage(fixture, root, "after", reader, seal, args.run)
        record["results"].append(verify_service(database, node_id, keys[node_id], captured,
                                               bytes.fromhex(ready["instance"]), reading_baseline))
    record["status"] = "PASS"
    write_json(root / "run.json", record)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for field in ("fixture", "output", "manual-record", "prerequisites", "local-group", "build"):
        parser.add_argument("--" + field, type=Path, required=True)
    for field in ("run", "unit", "package", "test-root"):
        parser.add_argument("--" + field, required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument("--host-key-alias")
    parser.add_argument("--ready-run")
    parser.add_argument("--confirm-isolated", action="store_true")
    run(parser.parse_args())


if __name__ == "__main__":
    main()

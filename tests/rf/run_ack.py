"""Explicit RF-019 runner for an already provisioned production image.

Does not flash, format, erase or create identities. Preparation is separate.
"""
import argparse
import json
from pathlib import Path
import re
import shlex
import shutil
import sys
import time

from evidence import REPO, admit_episode, digest, episode_capture, write_json
sys.path.insert(0, str(REPO / "receiver"))
sys.path.insert(0, str(REPO / "protocol/protocol-v2-lora/python"))

from cura_protocol_v2_lora.receiver_group import load_receiver_group
from cura_receiver.application import authentication_keys
from test_apps.radio_peer.ack_cases import CASES, episode
from control import Peer
from inputs import source_manifest
from node_capture import build_reader
from production_node import verify_build, verify_installed, capture_storage, identify, NodeUART
from spec import validate_fixture
from transport import RemoteTransport
from verify_ack import verify_ack_case, verify_ack_transmissions
from sleep_observation import sleep_count



def preflight(args):
    fixture = validate_fixture(json.loads(args.fixture.read_text()))
    if fixture["c6_fixture"] != "nominal":
        raise ValueError("RF-019 requires nominal production fixture")
    if not re.fullmatch("[0-9a-f]{32}", args.run):
        raise ValueError("invalid run identity")
    if not args.output.is_absolute() or args.output.exists() or not args.output.parent.is_dir():
        raise ValueError("output must be a new absolute directory")
    if not args.manual_record.is_file() or not args.manual_record.read_bytes().strip():
        raise ValueError("missing operator batch/airtime record")
    if not args.confirm_isolated or args.ready_run not in (None, args.run):
        raise ValueError("isolated test state/readiness not confirmed")
    if not args.formatter_result.is_file() or "LittleFS format completed successfully; NVS was preserved" not in args.formatter_result.read_text():
        raise ValueError("missing successful formatter result")
    seal = verify_build(args.build)
    duration = seal.get("sleep_seconds", 900)
    if duration != 10 or not seal.get("sleep_observation", False):
        raise ValueError("RF-019 requires observed accelerated10s build; cadence belongs to bench/pilot")
    group = load_receiver_group(args.local_group)
    node_id = bytes.fromhex(seal["node_id"])
    keys = authentication_keys(group)
    if node_id not in keys:
        raise ValueError("production node is not active in isolated group")
    return fixture, seal, group.group_id.hex(), keys[node_id]


def run(args):
    fixture, seal, group_id, key = preflight(args)
    plan = episode(args.case, seal.get("sleep_seconds", 900))
    # Current-source host prerequisite; no target devices are requested here.
    import subprocess
    host = subprocess.run(["make", "test-host"], cwd=REPO, capture_output=True, text=True)
    if host.returncode:
        raise ValueError("firmware deadline prerequisite failed or absent: " + host.stdout + host.stderr)
    if "node_core.invalid_acks_share_one_receive_interval" not in host.stdout:
        raise ValueError("required firmware deadline test missing from host run")
    args.output.mkdir(mode=0o700)
    root = args.output
    write_json(root / "host-deadline.json", dict(exit=host.returncode, stdout=host.stdout,
               scope="firmware host prerequisite, not RF-019 result"))
    manifest = source_manifest()
    write_json(root / "source-manifest.json", manifest)
    write_json(root / "production-build.json", seal)
    write_json(root / "episode.json", plan)
    shutil.copyfile(args.manual_record, root / "operator-airtime-record.txt")
    shutil.copyfile(args.formatter_result, root / "formatter.txt")
    reader = root / "node-image-reader"
    write_json(root / "decoder-sources.json", build_reader(reader))
    run_record = dict(schema=1, run=args.run, selected=[args.case], status="INCOMPLETE",
                      failures=[], results=[], fixture=fixture, node_id=seal["node_id"], group_id=group_id)
    remote = RemoteTransport(args.host, fixture["pi_user"], args.run, root,
                             args.host_key_alias, args.peer_python)
    with episode_capture(run_record, args.case, root, root) as cleanups:
        remote.stage(manifest, fixture)
        peer_args = [args.peer_python, "receiver/test_apps/radio_peer/ack_peer.py",
                     "--case", args.case, "--run", args.run, "--manifest", "source-manifest.json",
                     "--fixture", "rf-fixture.json", "--group", args.remote_group, "--node-id", seal["node_id"]]
        peer_args.extend(["--sleep-seconds", str(plan["sleep_seconds"])])
        command = "cd " + shlex.quote(remote.remote) + " && exec env PYTHONDONTWRITEBYTECODE=1 " + shlex.join(peer_args)
        checked = json.loads(remote.command(command + " --check").stdout)
        if checked["group_id"] != group_id or checked["node_id"] != seal["node_id"]:
            raise ValueError("local/remote isolated identity mismatch")
        print(json.dumps(plan, indent=2), flush=True)
        admit_episode(args.run, args.case, root, root / "operator-airtime-record.txt", args.ready_run)
        verify_installed(fixture, args.build, root)
        baseline = capture_storage(fixture, root, "before", reader, seal, args.run)
        if any(records for records in baseline["logs"].values()):
            raise ValueError("RF-019 setup requires explicitly initialized empty logs; never auto-erase")
        peer = Peer(remote.ssh(command), remote.env, args.run, args.case, root,
                    digest(root / "source-manifest.json"), lease_seconds=plan["lease_seconds"])
        cleanups.append(("authenticated peer", peer.close))
        node = NodeUART(fixture["c6_uart"], root)
        node_closed = False
        node_stopped = False

        def stop_node():
            nonlocal node_closed, node_stopped
            if node_stopped:
                return
            close_error = None
            if not node_closed:
                try:
                    node.close()
                except BaseException as error:
                    close_error = error
                node_closed = True
            try:
                identify(fixture, root, "final-stop-mac")
                node_stopped = True
            except BaseException:
                write_json(root / "node-restoration-required.json", dict(
                    action="operator must remove all node power; autonomous TX state unconfirmed"))
                raise
            if close_error is not None:
                raise close_error

        cleanups.append(("autonomous node stop", stop_node))
        peer.arm()
        node.start()
        end = time.monotonic() + plan["lease_seconds"] + 3
        sleep_notified = False
        while peer.process.poll() is None:
            if node.failure is not None:
                raise RuntimeError("UART observation lost: " + str(node.failure))
            if time.monotonic() >= end:
                raise TimeoutError("authenticated peer exceeded its finite lease")
            if plan["sleep_seconds"] == 10:
                count = sleep_count((root / "c6-uart.bin").read_bytes(), 10, plan["wakes"])
                if count == plan["wakes"] and not sleep_notified:
                    message = f"SLEEP {args.run} {args.case} {count}\n"
                    peer.process.stdin.write(message.encode()); peer.process.stdin.flush()
                    write_json(root / "sleep-observation.json", dict(run=args.run, case=args.case,
                        wakes=count, sleep_seconds=10, boundary="cycle finalized, timer configured, entering sleep"))
                    sleep_notified = True
            time.sleep(0.25)
        result = peer.finish()
        if plan["sleep_seconds"] == 10 and (not sleep_notified or not result["outcome"].get("final_sleep_observed")):
            raise ValueError("missing matched final sleep observation")
        stop_node()
        if plan["sleep_seconds"] == 10:
            if sleep_count((root / "c6-uart.bin").read_bytes(), 10, plan["wakes"]) != plan["wakes"]:
                raise ValueError("incomplete final UART observation")
        captured = capture_storage(fixture, root, "after", reader, seal, args.run)
        verify_ack_transmissions(args.case, bytes.fromhex(seal["node_id"]), key,
                                  result["outcome"], result["trace"], result["attempts"])
        verified = verify_ack_case(args.case, bytes.fromhex(seal["node_id"]), key,
                                   result["outcome"]["packets"], captured)
        run_record["results"].append(verified)
        verified["sleep_seconds"] = plan["sleep_seconds"]
        verified["cadence_scope"] = "accelerated ACK policy" if plan["sleep_seconds"] == 10 else "production cadence"
    run_record["status"] = "PASS"
    write_json(root / "run.json", run_record)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, required=True)
    for field in ("fixture", "output", "manual-record", "formatter-result", "local-group", "build"):
        parser.add_argument("--" + field, type=Path, required=True)
    for field in ("run", "remote-group"):
        parser.add_argument("--" + field, required=True)
    parser.add_argument("--host", default="cura-receiver")
    parser.add_argument("--host-key-alias")
    parser.add_argument("--peer-python", default="python3")
    parser.add_argument("--ready-run")
    parser.add_argument("--confirm-isolated", action="store_true")
    run(parser.parse_args())


if __name__ == "__main__":
    main()

"""Bounded SSH/process control and C6 UART event collection; no RF timing here."""
from __future__ import annotations
import io
import json
import os
from pathlib import Path
import queue
import re
import shlex
import subprocess
import sys
import tarfile
import threading
import time

from inputs import REPO, digest, write_json


def verify_uart_identity(fixture, root):
    """Identify in the ROM/stub and leave it there; never boot the previous app."""
    command = [sys.executable, "-m", "esptool", "--chip", "esp32c6", "--port", fixture["c6_uart"],
               "--after", "no-reset", "read-mac"]
    result = subprocess.run(command, capture_output=True, text=True, timeout=20)
    write_json(Path(root) / "uart-identity.json", dict(command=command, exit=result.returncode,
               stdout=result.stdout, stderr=result.stderr))
    mac = ":".join(fixture["c6_dut"][i:i+2] for i in range(0, 12, 2))
    if result.returncode or not re.search(r"^(?:BASE )?MAC:\s+" + re.escape(mac) + r"\s*$", result.stdout, re.M):
        raise ValueError("UART factory MAC mismatch before flash")


class Remote:
    def __init__(self, host, user, run, root, host_key_alias=None, python="python3"):
        if not re.fullmatch(r"[a-zA-Z0-9.-]+", host) or not re.fullmatch(r"[0-9a-f]{32}", run):
            raise ValueError("invalid host/run")
        self.host, self.user, self.run, self.root = host, user, run, Path(root)
        self.remote = "/var/tmp/cura-rf-" + run
        if python != "python3" and not python.startswith("/"):
            raise ValueError("peer Python must be python3 or an absolute interpreter path")
        self.python = python
        self.env = os.environ.copy()
        self.prefix = []
        if password := self.env.pop("CURA_PI_PASSWORD", None):
            self.env["SSHPASS"] = password
            self.prefix = ["sshpass", "-e"]
        self.options = ["-F", "/dev/null", "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=5",
                        "-o", "ServerAliveCountMax=2"]
        if host_key_alias:
            if not re.fullmatch(r"[a-zA-Z0-9.-]+", host_key_alias):
                raise ValueError("invalid SSH host-key alias")
            self.options += ["-o", "HostKeyAlias=" + host_key_alias]
        self.counter = 0

    def ssh(self, command):
        return self.prefix + ["ssh", *self.options, self.user + "@" + self.host, command]

    def command(self, command, timeout=30):
        self.counter += 1
        result = subprocess.run(self.ssh(command), env=self.env, capture_output=True, text=True, timeout=timeout)
        write_json(self.root / f"remote-{self.counter}.json", dict(command=command, exit=result.returncode,
                   stdout=result.stdout, stderr=result.stderr))
        if result.returncode:
            raise RuntimeError(f"remote command exited {result.returncode}: {result.stderr.strip()}")
        return result

    def stage(self, manifest, fixture):
        archive = self.root / "source.tar.gz"
        with tarfile.open(archive, "w:gz") as tar:
            for name, expected in manifest["files"].items():
                if digest(REPO / name) != expected:
                    raise ValueError("source changed while staging")
                tar.add(REPO / name, arcname=name, recursive=False)
            for name, value in (("source-manifest.json", manifest), ("rf-fixture.json", fixture)):
                data = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode()
                entry = tarfile.TarInfo(name); entry.size = len(data); entry.mode = 0o600
                tar.addfile(entry, io.BytesIO(data))
        self.command("mkdir -m 700 " + shlex.quote(self.remote))
        result = subprocess.run(self.prefix + ["scp", *self.options, str(archive),
            f"{self.user}@{self.host}:{self.remote}/source.tar.gz"], env=self.env,
            capture_output=True, text=True, timeout=60)
        write_json(self.root / "scp.json", dict(exit=result.returncode, stdout=result.stdout, stderr=result.stderr))
        if result.returncode:
            raise RuntimeError("source transfer failed")
        self.command("tar -xzf " + shlex.quote(self.remote + "/source.tar.gz") + " -C " + shlex.quote(self.remote))

    def peer(self, case, root, source_hash):
        args = [self.python, "receiver/test_apps/radio_peer/peer.py", "--case", case, "--run", self.run,
                "--manifest", "source-manifest.json", "--fixture", "rf-fixture.json"]
        command = "cd " + shlex.quote(self.remote) + " && exec env PYTHONDONTWRITEBYTECODE=1 " + shlex.join(args)
        return Peer(self.ssh(command), self.env, self.run, case, root, source_hash)

    def check_peer(self, case):
        args = [self.python, "receiver/test_apps/radio_peer/peer.py", "--case", case, "--run", self.run,
                "--manifest", "source-manifest.json", "--fixture", "rf-fixture.json", "--check"]
        return self.command("cd " + shlex.quote(self.remote) + " && " + shlex.join(args))


class Peer:
    def __init__(self, argv, env, run, case, root, source_hash):
        self.run, self.case, self.source_hash = run, case, source_hash
        self.root = Path(root)
        self.events, self.identity = [], None
        self.lines = queue.Queue(maxsize=8)
        self.raw = (self.root / "pi.jsonl").open("xb")
        self.stderr = (self.root / "pi.stderr").open("xb")
        self.process = subprocess.Popen(argv, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.stderr, start_new_session=True)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        self.completed = False

    def _read(self):
        try:
            while line := self.process.stdout.readline(4_000_001):
                self.raw.write(line); self.raw.flush()
                if len(line) > 4_000_000:
                    self.lines.put(ValueError("oversized peer event"), timeout=1)
                    break
                self.lines.put(line, timeout=1)
        except BaseException as exc:
            try:
                self.lines.put(exc, timeout=1)
            except queue.Full:
                pass
        finally:
            try:
                self.lines.put(None, timeout=1)
            except queue.Full:
                pass

    def event(self, kind, timeout):
        try:
            line = self.lines.get(timeout=timeout)
        except queue.Empty as exc:
            raise TimeoutError(f"missing peer {kind}") from exc
        if line is None or isinstance(line, BaseException):
            raise RuntimeError(f"peer ended before {kind}: {line}")
        value = json.loads(line)
        if (value.get("kind") != kind or value.get("run") != self.run or value.get("case") != self.case or
                value.get("source") != self.source_hash):
            raise ValueError("stale, duplicate or mismatched peer event")
        identity = (value.get("pid"), value.get("boot"), value.get("layer"))
        if not isinstance(identity[0], int) or not identity[1] or self.identity is not None and self.identity != identity:
            raise ValueError("peer identity changed")
        self.identity = identity
        self.events.append(value)
        return value

    def arm(self):
        self.event("ready", 10)
        self.process.stdin.write(f"GO {self.run} {self.case}\n".encode()); self.process.stdin.flush()
        return self.event("armed", 5)

    def finish(self):
        result = self.event("complete", 48)
        code = self.process.wait(timeout=5)
        self.reader.join(timeout=2)
        trailing = self.lines.get(timeout=2)
        if trailing is not None:
            raise ValueError("extra peer event after completion")
        write_json(self.root / "peer-exit.json", dict(exit=code))
        if code or result["failure"] or result["cleanup"].get("safe_shutdown") is not True:
            raise RuntimeError(f"peer failed: {result['failure']}")
        self.completed = True
        return result

    def close(self):
        try:
            if self.process.poll() is None:
                try:
                    self.process.stdin.write(b"STOP\n"); self.process.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Remote process has its own finite lease. Killing SSH is not
                    # evidence of radio safety, so retain required restoration.
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self.process.kill(); self.process.wait(timeout=3)
            if not self.completed:
                write_json(self.root / "restoration-required.json", dict(
                    reason="run did not confirm complete peer cleanup", latest_peer_lease_seconds=45,
                    action="inhibit further triggers; confirm peer exit/safe state or remove power"))
        finally:
            self.reader.join(timeout=2)
            self.process.stdin.close(); self.process.stdout.close()
            self.raw.close(); self.stderr.close()


class Node:
    def __init__(self, dut, fixture, elf, run, case, root):
        self.dut, self.fixture, self.elf = dut, fixture, elf
        self.run, self.case, self.root = run, case, Path(root)
        self.events = []
        self.boot = None

    def event(self, timeout=12):
        match = self.dut.expect(rb"RF_(BOOT|BEGIN|COMMAND|REJECT|RESULT|TRACE|END) (\{[^\r\n]*\})\r?\n", timeout=timeout)
        value = dict(kind=match.group(1).decode().lower(), **json.loads(match.group(2)))
        self.events.append(value)
        write_json(self.root / "c6-events.json", self.events)
        if value["kind"] == "reject":
            raise RuntimeError(f"C6 rejected command (boot {value.get('boot')}): {value.get('reason')}; no resend")
        return value

    def booted(self, previous=None):
        value = self.event()
        if value["kind"] != "boot" or value["dut"] != self.fixture["c6_dut"] or value["elf"] != self.elf:
            raise ValueError("unexpected C6 boot/image/DUT")
        if previous is not None and (value["reset"] != 8 or value["previous_boot"] != previous or
                                    value["previous_run"] != self.run or value["previous_case"] != self.case):
            raise ValueError("C6 did not complete the selected deep-sleep progression")
        if previous == value["boot"]:
            raise ValueError("C6 boot nonce did not change")
        self.dut.expect_exact("RF_READY", timeout=3)
        self.boot = value["boot"]
        return value

    def phase(self, phase):
        boot = self.boot
        self.dut.write(f"RUN {self.run} {self.case} {boot} {phase}")
        value = self.event()
        if value != dict(kind="begin", run=self.run, case=self.case, boot=boot, phase=phase):
            raise ValueError("wrong C6 command identity")
        command = self.event()
        if (command["kind"] != "command" or command["boot"] != boot or
                not 0 <= command["elapsed_us"] <= 2_000_000 or not 0 < command["bytes"] <= 159):
            raise ValueError("invalid C6 command framing evidence")
        # Consume Unity separately so pytest-embedded retains its actual result.
        self.dut.expect_unity_test_output(timeout=15)
        while True:
            value = self.event()
            if value["kind"] == "end":
                if any(value.get(key) != expected for key, expected in
                       (("run", self.run), ("case", self.case), ("boot", boot), ("phase", phase))):
                    raise ValueError("wrong C6 completion identity")
                break
            if value["kind"] not in {"result", "trace"}:
                raise ValueError("unexpected reset or event during C6 case")
        self.dut.expect_exact("RF_SLEEP", timeout=3)
        wake = self.booted(previous=boot)
        if value["failed"] or (value["cleanup_error"] and self.case != "RF-013.absent"):
            raise RuntimeError("C6 assertion or cleanup failure")
        if phase == 0 and self.case == "RF-010.wake" and wake["continuation"] != 1:
            raise ValueError("missing RF-010 continuation")
        return value

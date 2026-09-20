"""Bounded SSH and verified source staging, independent of the RF test layer."""
import io
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import tarfile

from evidence import REPO, digest, write_json


class RemoteTransport:
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

    def command(self, command, timeout=30, *, input=None):
        self.counter += 1
        try:
            result = subprocess.run(self.ssh(command), env=self.env, input=input, capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired as error:
            def decoded(value):
                return value.decode(errors="replace") if isinstance(value, bytes) else value
            write_json(self.root / f"remote-{self.counter}.json", dict(
                command=command, exit=None, timeout_seconds=timeout,
                stdout=decoded(error.stdout), stderr=decoded(error.stderr)))
            raise
        write_json(self.root / f"remote-{self.counter}.json", dict(command=command, exit=result.returncode,
                   stdout=result.stdout, stderr=result.stderr))
        if result.returncode:
            raise RuntimeError(f"remote command exited {result.returncode}: {result.stderr.strip()}")
        return result

    def fetch(self, remote_path, destination):
        if not remote_path.startswith(self.remote + "/") or ".." in Path(remote_path).parts:
            raise ValueError("capture outside isolated stage")
        destination = Path(destination)
        if destination.exists():
            raise ValueError("capture destination already exists")
        result = subprocess.run(self.prefix + ["scp", *self.options,
            f"{self.user}@{self.host}:{remote_path}", str(destination)], env=self.env,
            capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise RuntimeError("capture transfer failed: " + result.stderr.strip())

    def stage(self, manifest, fixture):
        if manifest.get("schema") != 1 or not manifest.get("files"):
            raise ValueError("invalid source manifest")
        for name in manifest["files"]:
            path = REPO / name
            if (Path(name).is_absolute() or ".." in Path(name).parts or
                    path.is_symlink() or not path.resolve().is_relative_to(REPO)):
                raise ValueError("unsafe source manifest path")
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

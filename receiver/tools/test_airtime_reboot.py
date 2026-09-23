"""External, bounded controller for the Pi airtime component reboot test.

Run from the host, never from the Pi being rebooted. Credentials come from one
explicit environment variable and are never included in artifacts. Source must
already be staged with a source-manifest.json; each phase verifies every hash.
"""

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time


def dedicated_test_root(value):
    if not re.fullmatch(r"/var/tmp/cura-airtime-[A-Za-z0-9_-]+", value):
        raise argparse.ArgumentTypeError(
            "use one new /var/tmp/cura-airtime-NAME directory"
        )
    return value


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--remote-source", required=True)
    parser.add_argument("--remote-test-root", required=True, type=dedicated_test_root)
    parser.add_argument("--python", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--password-env", default="CURA_PI_PASSWORD")
    parser.add_argument(
        "--confirm-receiver-destructive", action="store_true", required=True
    )
    return parser.parse_args()


class Controller:
    def __init__(self, args):
        self.args = args
        self.password = os.environ[args.password_env]
        self.environment = {**os.environ, "SSHPASS": self.password}
        self.counter = 0
        args.output.mkdir(parents=True, exist_ok=False)

    def remote(self, argv, *, name, timeout=30, check=True):
        command = (
            "cd "
            + shlex.quote(self.args.remote_source)
            + " && sudo -S -p "
            + shlex.quote("")
            + " "
            + shlex.join(argv)
        )
        try:
            result = subprocess.run(
                [
                    "sshpass",
                    "-e",
                    "ssh",
                    "-F",
                    "/dev/null",
                    "-o",
                    "ConnectTimeout=8",
                    "-o",
                    "ServerAliveInterval=5",
                    "-o",
                    "ServerAliveCountMax=2",
                    self.args.host,
                    command,
                ],
                input=self.password + "\n",
                capture_output=True,
                text=True,
                timeout=timeout,
                env=self.environment,
            )
        except subprocess.TimeoutExpired as error:
            result = subprocess.CompletedProcess(
                argv, 124, "", "bounded remote command timeout"
            )
            if check:
                (self.args.output / (name + "-timeout.txt")).write_text(
                    str(error.cmd[-1]) + "\n"
                )
        self.counter += 1
        record = dict(
            argv=argv,
            returncode=result.returncode,
            stdout=result.stdout,
            stderr=result.stderr,
        )
        (self.args.output / f"{self.counter:03d}-{name}.json").write_text(
            json.dumps(record, indent=2) + "\n"
        )
        if check and result.returncode:
            raise RuntimeError(
                f"{name}: exit {result.returncode}; see {self.args.output}"
            )
        return result

    def inspect(self, name, *, check=True):
        code = """import hashlib,json,pathlib,subprocess,platform,sys,sqlite3
root=pathlib.Path.cwd()
manifest=json.loads((root/'source-manifest.json').read_text())
assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==digest for p,digest in manifest['files'].items())
run=lambda *args: subprocess.run(args,check=True,capture_output=True,text=True,timeout=10).stdout.strip()
print(json.dumps(dict(boot=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
 model=pathlib.Path('/proc/device-tree/model').read_text().strip('\\x00\\n'),
 config_sha256=hashlib.sha256(pathlib.Path('/etc/chrony/chrony.conf').read_bytes()).hexdigest(),
 chrony=run('systemctl','is-active','chrony'), tracking=run('/usr/bin/chronyc','-n','-c','-h','/run/chrony/chronyd.sock','tracking'),
 python=sys.version,sqlite=sqlite3.sqlite_version,platform=platform.platform(),
 source_manifest_sha256=hashlib.sha256((root/'source-manifest.json').read_bytes()).hexdigest())))
"""
        result = self.remote([self.args.python, "-c", code], name=name, check=check)
        return json.loads(result.stdout) if result.returncode == 0 else None

    def phase(self, mode, phase):
        observed = self.inspect(mode + "-" + phase + "-source")
        assert observed["source_manifest_sha256"] == self.source_manifest_sha256
        session = self.args.remote_test_root + "/" + mode
        argv = [
            "env",
            "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1",
            self.args.python,
            "-m",
            "pytest",
            "-c",
            "receiver/pytest.ini",
            "receiver/tests/hardware/test_tx_airtime_reboot.py",
            "--receiver-hardware",
            "--confirm-receiver-destructive",
            "--receiver-test-root=" + self.args.remote_test_root,
            "--airtime-reboot-session=" + session,
            "--airtime-reboot-mode=" + mode,
            "--airtime-reboot-phase=" + phase,
            "--basetemp=" + session + "/pytest-" + phase,
            "--junitxml=" + session + "/" + phase + ".xml",
            "-vv",
            "--tb=long",
        ]
        self.remote(argv, name=mode + "-" + phase, timeout=60)

    def run(self):
        args = self.args
        assert args.remote_source.startswith("/") and args.python.startswith("/")
        assert not args.host.startswith("-")
        assert re.fullmatch(
            r"/var/tmp/cura-airtime-[A-Za-z0-9_-]+", args.remote_test_root
        )
        baseline = self.inspect("preflight")
        self.source_manifest_sha256 = baseline["source_manifest_sha256"]
        assert "Raspberry Pi" in baseline["model"] and baseline["chrony"] == "active"
        assert baseline["tracking"].split(",")[-1] == "Normal"
        # Fixture settings/services are read only. The dedicated persistent root is new.
        setup = """import json,pathlib,subprocess,sys
root=pathlib.Path(sys.argv[1]); root.mkdir(mode=0o700)
(root/'.cura-receiver-test-root').write_text('CURA AGRORUM RECEIVER TEST ROOT\\n')
for mode in ('trusted','unavailable'):
    session=root/mode; session.mkdir(mode=0o700)
    (session/'controller.json').write_text(json.dumps({'workload':'receiver_tx_airtime','mode':mode})+'\\n')
"""
        self.remote(
            [args.python, "-c", setup, args.remote_test_root], name="fixture-setup"
        )
        for mode in ("trusted", "unavailable"):
            print(mode + ": preparing durable charge", flush=True)
            self.phase(mode, "prepare")
            before = self.inspect(mode + "-before-reboot")
            print(mode + ": rebooting Pi", flush=True)
            self.remote(["systemctl", "reboot"], name=mode + "-reboot", check=False)
            until = time.monotonic() + 180
            after = None
            while time.monotonic() < until:
                time.sleep(2)
                candidate = self.inspect(mode + "-reconnect", check=False)
                if (
                    candidate
                    and candidate["boot"] != before["boot"]
                    and candidate["tracking"].split(",")[-1] == "Normal"
                ):
                    after = candidate
                    break
            assert (
                after is not None
            ), "Pi did not restore trusted network time within 180 seconds"
            assert after["config_sha256"] == baseline["config_sha256"]
            assert after["source_manifest_sha256"] == baseline["source_manifest_sha256"]
            print(
                mode + ": verifying reconstruction after changed boot identity",
                flush=True,
            )
            self.phase(mode, "verify")
        restored = self.inspect("restoration")
        assert (
            restored["chrony"] == "active"
            and restored["tracking"].split(",")[-1] == "Normal"
        )
        assert restored["config_sha256"] == baseline["config_sha256"]
        (args.output / "result.json").write_text(
            json.dumps(
                {"passed": True, "before": baseline, "restored": restored}, indent=2
            )
            + "\n"
        )
        print(
            "Both reboot modes passed; Chrony configuration and service state verified.",
            flush=True,
        )


if __name__ == "__main__":
    controller = Controller(arguments())
    try:
        controller.run()
    finally:
        # Reboot is the only host mutation. Always record whether its existing time service recovered.
        try:
            final = controller.inspect("final-host-state", check=False)
            (controller.args.output / "final-host-state.json").write_text(
                json.dumps(final, indent=2) + "\n"
            )
        except Exception as error:
            (controller.args.output / "restoration-unverified.txt").write_text(
                type(error).__name__ + ": " + str(error) + "\n"
            )

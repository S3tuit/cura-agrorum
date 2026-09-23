"""Root-owned bounded storage fixture; receiver assertions run without privileges."""

from contextlib import contextmanager
import ctypes
import json
import multiprocessing
import os
from pathlib import Path
import pwd
import shutil
import subprocess
import tempfile
import time
import traceback


def _command(*arguments):
    subprocess.run(arguments, check=True, capture_output=True, text=True, timeout=20)


def _send(pipe, value):
    pipe.send_bytes(json.dumps(value).encode())


def _receive(pipe):
    return json.loads(pipe.recv_bytes(65536))


def _component(pipe, parent_pipe, account, mount, check, arguments):
    parent_pipe.close()
    try:
        os.setgroups([])
        os.setgid(account.pw_gid)
        os.setuid(account.pw_uid)
        # The child must not regain privilege through a setuid executable.
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(38, 1, 0, 0, 0) != 0:  # PR_SET_NO_NEW_PRIVS
            raise OSError(ctypes.get_errno(), "PR_SET_NO_NEW_PRIVS")
        status = dict(
            line.split(":", 1) for line in Path("/proc/self/status").read_text().splitlines()
            if ":" in line
        )
        assert os.getuid() == os.geteuid() == account.pw_uid != 0
        assert os.getgid() == os.getegid() == account.pw_gid
        assert os.getgroups() == []
        for field in ("CapInh", "CapPrm", "CapEff", "CapAmb"):
            assert int(status[field], 16) == 0, field
        assert int(status["NoNewPrivs"]) == 1
        _send(pipe, {"event": "identity", "uid": os.getuid(), "gid": os.getgid(),
                     "capabilities": {k: status[k].strip() for k in
                                      ("CapInh", "CapPrm", "CapEff", "CapAmb")},
                     "no_new_privileges": True})

        def remount(mode):
            assert mode in ("ro", "rw")
            _send(pipe, {"event": "remount", "mode": mode})
            assert pipe.poll(30), "storage supervisor did not reply"
            assert _receive(pipe) == {"event": "remounted", "mode": mode}

        check(mount, remount, *arguments)
        _send(pipe, {"event": "passed"})
    except BaseException:
        _send(pipe, {"event": "failed", "traceback": traceback.format_exc()})
        raise
    finally:
        pipe.close()


class StorageSupervisor:
    def __init__(self, mount, account, evidence):
        self.mount = mount
        self.account = account
        self.evidence = evidence

    def run(self, check, *arguments):
        context = multiprocessing.get_context("fork")
        parent, child = context.Pipe()
        process = context.Process(
            target=_component,
            args=(child, parent, self.account, self.mount, check, arguments),
        )
        process.start()
        child.close()
        deadline = time.monotonic() + 180
        try:
            while True:
                remaining = deadline - time.monotonic()
                assert remaining > 0 and parent.poll(remaining), "storage child timed out"
                event = _receive(parent)
                self.evidence["events"].append(event)
                if event["event"] == "identity":
                    assert len(self.evidence["events"]) == 1
                elif event["event"] == "remount":
                    mode = event["mode"]
                    assert mode in ("ro", "rw")
                    assert self.evidence["events"][0]["event"] == "identity"
                    _command("mount", "-o", "remount," + mode, str(self.mount))
                    assert bool(os.statvfs(self.mount).f_flag & os.ST_RDONLY) == (mode == "ro")
                    _send(parent, {"event": "remounted", "mode": mode})
                elif event["event"] == "failed":
                    raise AssertionError(event["traceback"])
                else:
                    assert event == {"event": "passed"}
                    assert self.evidence["events"][0]["event"] == "identity"
                    process.join(10)
                    assert process.exitcode == 0
                    self.evidence["passed"] = True
                    break
        except BaseException:
            self.evidence["failure"] = traceback.format_exc()
            raise
        finally:
            if process.is_alive():
                process.terminate()
                process.join(5)
            if process.is_alive():
                process.kill()
                process.join(5)
            parent.close()
            self.evidence["child_exitcode"] = process.exitcode
            assert not process.is_alive(), "storage child could not be reaped"


@contextmanager
def bounded_storage(root, user):
    assert os.geteuid() == 0, "storage fixture requires a root supervisor"
    account = pwd.getpwnam(user)
    assert account.pw_uid != 0, "storage component must be unprivileged"
    fixture = Path(tempfile.mkdtemp(prefix="ordinary-storage-", dir=root))
    fixture.chmod(0o711)
    mount = fixture / "mount"
    mount.mkdir()
    evidence = {"user": user, "events": [], "passed": False, "unmounted": False}
    mounted = False
    try:
        _command("mount", "-t", "tmpfs", "-o",
                 f"size=4m,mode=0700,uid={account.pw_uid},gid={account.pw_gid},nosuid,nodev,noexec",
                 "cura-receiver-test", str(mount))
        mounted = True
        assert mount.stat().st_dev != fixture.stat().st_dev
        assert os.statvfs(mount).f_blocks * os.statvfs(mount).f_frsize <= 4 * 1024 * 1024
        yield StorageSupervisor(mount, account, evidence)
    finally:
        try:
            if mounted:
                try:
                    shutil.copytree(mount, fixture / "artifacts")
                finally:
                    _command("umount", str(mount))
                    assert mount.stat().st_dev == fixture.stat().st_dev
                    evidence["unmounted"] = True
        finally:
            (fixture / "supervisor.json").write_text(json.dumps(evidence, indent=2) + "\n")

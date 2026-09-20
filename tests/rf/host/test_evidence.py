import json
import subprocess

import pytest

from evidence import episode_capture
from transport import RemoteTransport


@pytest.mark.parametrize("primary", [False, True])
def test_all_cleanup_runs_and_original_failure_survives(tmp_path, primary):
    run = {"failures": []}
    calls = []

    def broken():
        calls.append("broken")
        raise OSError("cleanup failed")

    expected = ValueError if primary else OSError
    with pytest.raises(expected):
        with episode_capture(run, "case", tmp_path, tmp_path) as cleanups:
            cleanups.extend([("last", lambda: calls.append("last")), ("broken", broken)])
            if primary:
                raise ValueError("original failure")
    assert calls == ["broken", "last"]
    saved = json.loads((tmp_path / "run.json").read_text())
    assert len(saved["failures"]) == 1 + primary
    if primary:
        assert "original failure" in saved["failures"][0]["error"]
    assert saved["failures"][-1]["cleanup"] == "broken"


def test_timeout_retains_partial_command_output(tmp_path, monkeypatch):
    remote = RemoteTransport("cura-receiver", "cura", "a" * 32, tmp_path)

    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 1, output=b"partial", stderr=b"failure")

    monkeypatch.setattr("transport.subprocess.run", timeout)
    with pytest.raises(subprocess.TimeoutExpired):
        remote.command("approved-command", timeout=1)
    result = json.loads((tmp_path / "remote-1.json").read_text())
    assert result["exit"] is None
    assert result["stdout"] == "partial"


@pytest.mark.parametrize("name", ["../outside", "/etc/passwd"])
def test_unsafe_manifest_rejected_before_remote_access(tmp_path, name):
    remote = RemoteTransport("cura-receiver", "cura", "a" * 32, tmp_path)
    with pytest.raises(ValueError, match="unsafe"):
        remote.stage({"schema": 1, "files": {name: "hash"}}, {})
    assert remote.counter == 0

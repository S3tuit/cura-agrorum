"""Exercise supervisor protocol and failed-child reaping without mounting anything."""

import os
from pathlib import Path

import pytest

from tests.hardware import storage_fixture as fixture


def _scripted_child(pipe, parent, account, mount, check, events):
    parent.close()
    for event in events[0]:
        fixture._send(pipe, event)
        if event.get("event") == "remount" and event.get("mode") in ("ro", "rw"):
            assert pipe.poll(5)
            assert fixture._receive(pipe) == {"event": "remounted", "mode": event["mode"]}
    pipe.close()


@pytest.mark.parametrize("events,expected_commands,error", [
    ([{"event": "identity"}, {"event": "passed"}], [], None),
    ([{"event": "identity"}, {"event": "failed", "traceback": "original assertion"}], [], "original assertion"),
    ([{"event": "identity"}, {"event": "remount", "mode": "rw"}, {"event": "passed"}],
     [("mount", "-o", "remount,rw", "/fixed/test/mount")], None),
    ([{"event": "identity"}, {"event": "remount", "mode": "/other/path"}], [], ""),
    ([{"event": "remount", "mode": "rw"}], [], ""),
    ([{"event": "arbitrary-command"}], [], ""),
])
def test_storage_supervisor_protocol(monkeypatch, events, expected_commands, error):
    monkeypatch.setattr(fixture, "_component", _scripted_child)
    commands = []
    monkeypatch.setattr(fixture, "_command", lambda *args: commands.append(args))
    stat = os.statvfs("/tmp")
    monkeypatch.setattr(fixture.os, "statvfs", lambda path: stat)
    evidence = {"events": [], "passed": False}
    supervisor = fixture.StorageSupervisor(Path("/fixed/test/mount"), None, evidence)
    if error is None:
        supervisor.run(None, events)
        assert evidence["passed"]
        assert evidence["child_exitcode"] == 0
    else:
        with pytest.raises(AssertionError, match=error or None):
            supervisor.run(None, events)
        assert not evidence["passed"]
        assert evidence["child_exitcode"] is not None
    assert commands == expected_commands


def test_storage_supervisor_rejects_root_component(monkeypatch, tmp_path):
    monkeypatch.setattr(fixture.os, "geteuid", lambda: 0)
    with pytest.raises(AssertionError, match="component must be unprivileged"):
        with fixture.bounded_storage(tmp_path, "root"):
            pytest.fail("root component admitted")
    assert list(tmp_path.iterdir()) == []

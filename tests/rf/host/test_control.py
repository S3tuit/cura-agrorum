import importlib.util
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from control import Node, Peer, verify_uart_identity


def process_script(mode):
    # A child process exercises actual pipes, exit handling and bounded cleanup.
    return f'''
import json, sys
identity = dict(run="a"*32, case="RF-001.exchange", source="s", pid=123, boot="boot", layer="Radio/Sx1262/LinuxRadioIo")
def emit(kind, **values):
    print(json.dumps(dict(kind=kind, **identity, **values)), flush=True)
mode = {mode!r}
if mode == "missing":
    sys.stdin.readline()
    sys.exit(1)
if mode == "stale": identity["run"] = "b"*32
emit("ready")
if sys.stdin.readline().startswith("GO"):
    emit("armed")
    if mode == "identity": identity["pid"] = 456
    emit("complete", failure=None, cleanup=dict(safe_shutdown=mode != "unsafe"))
    if mode == "duplicate": emit("complete")
'''


@pytest.mark.parametrize("mode", ["good", "stale", "identity", "unsafe", "duplicate", "missing"])
def test_real_process_handshake_and_teardown(tmp_path, mode):
    peer = Peer([sys.executable, "-u", "-c", process_script(mode)], os.environ.copy(),
                "a"*32, "RF-001.exchange", tmp_path, "s")
    try:
        if mode == "good":
            peer.arm(); peer.finish()
        else:
            with pytest.raises((ValueError, RuntimeError, TimeoutError)):
                if mode == "missing":
                    peer.event("ready", 0.1)
                else:
                    peer.arm(); peer.finish()
    finally:
        peer.close()
    assert peer.process.poll() is not None
    assert (tmp_path / "restoration-required.json").exists() == (mode != "good")
    assert (tmp_path / "pi.jsonl").is_file()


def test_failed_or_skipped_report_cannot_become_requested_run_pass(tmp_path):
    spec = importlib.util.spec_from_file_location("rf_report_hook", Path(__file__).resolve().parents[1] / "conftest.py")
    hook = importlib.util.module_from_spec(spec); spec.loader.exec_module(hook)
    xml = tmp_path / "junit.xml"
    xml.write_text('<testsuites><testsuite><testcase name="Unity passed"/><testcase name="peer failed"><failure/></testcase></testsuite></testsuites>')
    run = dict(selected=["RF-001.exchange"], results=[dict(case="RF-001.exchange", status="PASS")])
    config = SimpleNamespace(_rf_run=run, _rf_context={"output": tmp_path}, getoption=lambda _: str(xml))
    session = SimpleNamespace(config=config, exitstatus=0)
    execution = hook.pytest_sessionfinish(session, 0)
    next(execution)
    with pytest.raises(StopIteration):
        next(execution)
    assert session.exitstatus == 1
    assert json.loads((tmp_path / "run.json").read_text())["status"] == "FAIL"
    assert 'failures="1"' in xml.read_text()


@pytest.mark.parametrize("mac,exitcode", [("cc:8d:a2:fc:02:24", 0), ("00:00:00:00:00:01", 0), ("cc:8d:a2:fc:02:24", 1)])
def test_preflash_identity_probe_never_restarts_previous_app(tmp_path, monkeypatch, mac, exitcode):
    observed = []
    def execute(command, **kwargs):
        observed.append(command)
        return SimpleNamespace(returncode=exitcode, stdout="BASE MAC: " + mac + "\n", stderr="")
    monkeypatch.setattr("control.subprocess.run", execute)
    fixture = dict(c6_uart="/dev/ttyUSB0", c6_dut="cc8da2fc0224")
    if mac == "cc:8d:a2:fc:02:24" and exitcode == 0:
        verify_uart_identity(fixture, tmp_path)
    else:
        with pytest.raises(ValueError, match="MAC mismatch"):
            verify_uart_identity(fixture, tmp_path)
    assert observed[0][-3:] == ["--after", "no-reset", "read-mac"]


def test_c6_rejection_is_retained_and_fails_without_resending(tmp_path):
    import re
    writes = []
    event = b'RF_REJECT {"boot":11,"reason":"incomplete_timeout","bytes":3,"elapsed_us":2000001}\n'
    dut = SimpleNamespace(write=writes.append, expect=lambda pattern, **kw: re.search(pattern, event))
    node = Node(dut, {}, "elf", "a"*32, "RF-001.exchange", tmp_path)
    node.boot = 11
    with pytest.raises(RuntimeError, match="incomplete_timeout.*no resend"):
        node.phase(0)
    assert len(writes) == 1
    assert json.loads((tmp_path / "c6-events.json").read_text())[0]["kind"] == "reject"

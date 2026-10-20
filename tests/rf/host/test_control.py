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
identity = dict(run="a"*32, case="component.ack_exchange", source="s", pid=123, boot="boot", layer="Radio/Sx1262/LinuxRadioIo")
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
                "a"*32, "component.ack_exchange", tmp_path, "s")
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
    run = dict(selected=["component.ack_exchange"], results=[dict(case="component.ack_exchange", status="PASS")])
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
    fixture = dict(node_uart="/dev/ttyACM0", node_dut="cc8da2fc0224")
    if mac == "cc:8d:a2:fc:02:24" and exitcode == 0:
        verify_uart_identity(fixture, tmp_path)
    else:
        with pytest.raises(ValueError, match="MAC mismatch"):
            verify_uart_identity(fixture, tmp_path)
    assert observed[0][-3:] == ["--after", "no-reset", "read-mac"]


def test_node_rejection_is_retained_and_fails_without_resending(tmp_path):
    import re
    writes = []
    event = b'RF_REJECT {"boot":11,"reason":"incomplete_timeout","bytes":3,"elapsed_us":2000001}\n'
    dut = SimpleNamespace(write=writes.append, expect=lambda pattern, **kw: re.search(pattern, event))
    node = Node(dut, {}, "elf", "a"*32, "component.ack_exchange", tmp_path)
    node.boot = 11
    with pytest.raises(RuntimeError, match="incomplete_timeout.*no resend"):
        node.phase(0)
    assert len(writes) == 1
    assert json.loads((tmp_path / "node-events.json").read_text())[0]["kind"] == "reject"


@pytest.mark.parametrize("fault", [None, "lost_receipt", "missing_sleep", "wrong_end"])
def test_component_parses_completion_before_receipt_and_requires_real_wake(tmp_path, monkeypatch, fault):
    import re
    from finalization import Finalization
    clock = SimpleNamespace(now=0)
    monkeypatch.setattr("control.Finalization", lambda select, send: Finalization(select, send, clock=lambda: clock.now))
    sent, commands = [], []
    usb = SimpleNamespace(on_data=None, check_health=lambda: None)
    stream = bytearray()
    def event(kind, **values):
        return ("RF_" + kind + " " + json.dumps(values) + "\n").encode()
    def feed(data):
        stream.extend(data)
        if usb.on_data is not None:
            usb.on_data(data)
    def write_receipt(command):
        assert not stream, "completion records must be consumed before receipt"
        sent.append(command)
        if fault == "lost_receipt" and len(sent) == 1:
            return
        feed(event("BOOT", dut="001122334455", elf="image", boot=22, reset=8,
                   previous_boot=11, previous_run="a" * 32,
                   previous_case="component.ack_exchange", continuation=0) + b"RF_READY\n")
    usb.write = write_receipt
    def select(command):
        commands.append(command)
        feed(event("BEGIN", run="a" * 32, case="component.ack_exchange", boot=11, phase=0)
             + event("COMMAND", boot=11, elapsed_us=100, bytes=77)
             + event("END", run="b" * 32 if fault == "wrong_end" else "a" * 32,
                     case="component.ack_exchange", boot=11, phase=0, failed=0, cleanup_error=0)
             + (b"" if fault == "missing_sleep" else b"RF_SLEEP\n"))
    def expect(pattern, **kwargs):
        match = re.search(pattern, bytes(stream))
        if match is None and fault not in {"missing_sleep", "wrong_end"}:
            for _ in range(2):
                clock.now += 1
                usb.on_data(b"RF_FINALIZE 12345678\n")
                match = re.search(pattern, bytes(stream))
                if match is not None:
                    break
        if match is None:
            raise TimeoutError("missing component record")
        del stream[:match.end()]
        return match
    dut = SimpleNamespace(serial=SimpleNamespace(_cura_usb=usb), write=select, expect=expect,
        expect_exact=lambda text, **kwargs: expect(re.escape(text.encode()), **kwargs),
        expect_unity_test_output=lambda **kwargs: None)
    node = Node(dut, dict(node_dut="001122334455"), "image", "a" * 32,
                "component.ack_exchange", tmp_path)
    node.boot = 11
    if fault in {"missing_sleep", "wrong_end"}:
        with pytest.raises((ValueError, TimeoutError)):
            node.phase(0)
        assert sent == []
    else:
        node.phase(0)
        assert sent == [b"RF_FINALIZE_ACK 12345678 SLEEP\n"] * (2 if fault == "lost_receipt" else 1)
        assert usb.on_data is None and node.boot == 22
    assert len(commands) == 1

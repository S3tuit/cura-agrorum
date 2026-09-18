"""Temporary prerequisites must not outlive their source, devices or bench session."""
from types import SimpleNamespace

import pytest

import inputs
import pytest_radio
from inputs import finish_session, session_identity, start_session, write_json


@pytest.fixture
def bench(tmp_path):
    manifest = dict(files={"peer.py": "a", "README.md": "b"})
    seal = dict(files={"cura_radio_component.elf": "image"})
    fixture = dict(c6_dut="node", pi_board_id="pi", c6_transmitter="node radio",
                   pi_transmitter="pi radio", c6_fixture="nominal")
    return tmp_path / "session.json", manifest, seal, fixture


def qualify(bench, selected=("RF-001.exchange", "RF-003.silence")):
    path, manifest, seal, fixture = bench
    state, passed = start_session(path, session_identity(manifest, seal, fixture), selected)
    finish_session(path, state, passed, dict(status="PASS", fixture=fixture, selected=selected))


def test_nominal_to_fault_then_restoration(bench):
    qualify(bench)
    path, manifest, seal, fixture = bench
    fixture["c6_fixture"] = "dio1_disconnected"
    identity = session_identity(manifest, seal, fixture)
    state, passed = start_session(path, identity, ["RF-012.disconnected"])
    finish_session(path, state, passed, dict(status="PASS", fixture=fixture, selected=["RF-012.disconnected"]))
    with pytest.raises(ValueError, match="fresh nominal"):
        start_session(path, identity, ["RF-013.absent"])
    fixture["c6_fixture"] = "nominal"
    qualify(bench)
    assert start_session(path, identity, ["RF-013.absent"])[1] == {"RF-001.exchange", "RF-003.silence"}


@pytest.mark.parametrize("change", ["source", "build", "node", "pi", "radio", "expired", "reboot", "future"])
def test_stale_session_requires_new_nominal_run(bench, change, monkeypatch):
    qualify(bench)
    path, manifest, seal, fixture = bench
    if change == "source": manifest["files"]["peer.py"] = "changed"
    elif change == "build": seal["files"]["cura_radio_component.elf"] = "changed"
    elif change == "node": fixture["c6_dut"] = "other"
    elif change == "pi": fixture["pi_board_id"] = "other"
    elif change == "radio": fixture["c6_transmitter"] = "other"
    else:
        import json
        state = json.loads(path.read_text())
        if change == "reboot": state["boot"] = "other"
        else: state["started"] += 43201 * (-1 if change == "expired" else 1)
        write_json(path, state)
    with pytest.raises(ValueError, match="fresh nominal"):
        start_session(path, session_identity(manifest, seal, fixture), ["RF-012.disconnected"])
    qualify(bench)  # Explicit fresh nominal selection always recovers stale state.


def test_docs_do_not_invalidate_executable_prerequisite(bench):
    qualify(bench)
    path, manifest, seal, fixture = bench
    manifest["files"]["README.md"] = "updated results"
    assert start_session(path, session_identity(manifest, seal, fixture), ["RF-008.exchange"])[1]


@pytest.mark.parametrize("selection", [["RF-008.exchange"], ["RF-003.silence", "RF-001.exchange"],
                                      ["RF-001.exchange", "RF-008.exchange"]])
def test_missing_or_reordered_prerequisites_rejected(bench, selection):
    path, manifest, seal, fixture = bench
    with pytest.raises(ValueError, match="fresh nominal"):
        start_session(path, session_identity(manifest, seal, fixture), selection)
    assert not path.exists()


@pytest.mark.parametrize("outcome", ["FAIL", "interrupted"])
def test_failure_or_interruption_invalidates_previous_passes(bench, outcome):
    qualify(bench)
    path, manifest, seal, fixture = bench
    identity = session_identity(manifest, seal, fixture)
    state, passed = start_session(path, identity, ["RF-006.invalid"])
    if outcome == "FAIL":
        finish_session(path, state, passed, dict(status="FAIL", fixture=fixture, selected=["RF-006.invalid"]))
    with pytest.raises(ValueError, match="fresh nominal"):
        start_session(path, identity, ["RF-012.disconnected"])


def test_rejection_precedes_staging_and_device_access(bench, tmp_path, monkeypatch):
    path, manifest, seal, fixture = bench
    root = tmp_path / "run"; root.mkdir()
    manual = tmp_path / "sheet"; manual.write_text("operator sheet")
    ctx = dict(output=root, session=path, seal=seal, manual=manual, fixture=fixture,
               episodes=[SimpleNamespace(name="RF-012.disconnected")])
    config = SimpleNamespace(_rf_context=ctx, getoption={"rf_run": "a" * 32}.get)
    monkeypatch.setattr(pytest_radio, "source_manifest", lambda: manifest)
    def forbidden(*args, **kwargs):
        pytest.fail("device/staging reached before prerequisite acceptance")
    monkeypatch.setattr(pytest_radio, "Remote", forbidden)
    runner = pytest_radio.joint_run.__wrapped__(SimpleNamespace(config=config, getfixturevalue=forbidden))
    with pytest.raises(ValueError, match="fresh nominal"):
        next(runner)


@pytest.mark.parametrize("failure", [None, "junit", "missing_xml", "pytest", "incomplete"])
def test_only_completed_successful_pytest_publishes_prerequisites(bench, tmp_path, failure):
    import importlib.util
    from pathlib import Path
    definition = importlib.util.spec_from_file_location("rf_session_hook", Path(inputs.__file__).with_name("conftest.py"))
    guard = importlib.util.module_from_spec(definition); definition.loader.exec_module(guard)
    path, manifest, seal, fixture = bench
    identity = session_identity(manifest, seal, fixture)
    state = start_session(path, identity, ["RF-001.exchange"])
    xml = tmp_path / "junit.xml"
    if failure != "missing_xml":
        xml.write_text('<testsuites><testsuite><testcase>' +
                       ('<failure/>' if failure == "junit" else '') + '</testcase></testsuite></testsuites>')
    run = dict(fixture=fixture, selected=["RF-001.exchange"], results=[] if failure == "incomplete" else [{}])
    config = SimpleNamespace(_rf_session=state, _rf_run=run,
                             _rf_context=dict(session=path, output=tmp_path),
                             getoption={"xmlpath": str(xml)}.get)
    session = SimpleNamespace(config=config, exitstatus=1 if failure == "pytest" else 0)
    hook = guard.pytest_sessionfinish(session, session.exitstatus)
    next(hook)
    with pytest.raises(StopIteration):
        next(hook)
    if failure:
        assert run["status"] == "FAIL"
        with pytest.raises(ValueError, match="fresh nominal"):
            start_session(path, identity, ["RF-012.disconnected"])
    else:
        assert run["status"] == "PASS"
        assert start_session(path, identity, ["RF-012.disconnected"])[1] == {"RF-001.exchange"}


def test_session_path_cannot_overwrite_fixture_or_operator_input(bench):
    path, manifest, seal, fixture = bench
    write_json(path, fixture)
    original = path.read_bytes()
    with pytest.raises(ValueError, match="not an RF session receipt"):
        start_session(path, session_identity(manifest, seal, fixture), ["RF-001.exchange"])
    assert path.read_bytes() == original

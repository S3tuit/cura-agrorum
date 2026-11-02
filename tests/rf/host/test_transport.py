"""Public source archives remain readable by the isolated service account."""
import hashlib
import json
import stat
import tarfile
from types import SimpleNamespace

import transport


def test_staged_generated_sources_are_public_without_exposing_private_inputs(tmp_path, monkeypatch):
    repo = tmp_path / "repo"
    repo.mkdir()
    source = repo / "generated.py"
    source.write_bytes(b"VALUE = 1\n")
    source.chmod(0o600)
    private = repo / "receiver-group.json"
    private.write_bytes(b"private fixture credential")
    private.chmod(0o600)
    output = tmp_path / "capture"
    output.mkdir()
    manifest = dict(schema=1, files={"generated.py": hashlib.sha256(source.read_bytes()).hexdigest()})
    monkeypatch.setattr(transport, "REPO", repo)
    monkeypatch.setattr(transport.subprocess, "run", lambda *a, **kw:
                        SimpleNamespace(returncode=0, stdout="", stderr=""))
    remote = transport.RemoteTransport("pi", "user", "a" * 32, output)
    monkeypatch.setattr(remote, "command", lambda *a, **kw: None)
    remote.stage(manifest, dict(schema=2))
    with tarfile.open(output / "source.tar.gz") as archive:
        assert set(archive.getnames()) == {"generated.py", "source-manifest.json", "rf-fixture.json"}
        assert archive.getmember("generated.py").mode == 0o644
        assert archive.extractfile("generated.py").read() == source.read_bytes()
        assert json.load(archive.extractfile("source-manifest.json")) == manifest
    assert stat.S_IMODE(source.stat().st_mode) == 0o600
    assert stat.S_IMODE(private.stat().st_mode) == 0o600

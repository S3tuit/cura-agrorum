"""Explicit laptop entry point; default pytest targets never select this file."""
import shutil

import pytest

from control import Node, Remote, verify_uart_identity
from evidence import admit_episode, episode_capture
from inputs import digest, session_identity, start_session, source_manifest, write_json
from spec import EPISODES
from verify import verify_case

pytestmark = pytest.mark.rf_component


@pytest.fixture(scope="session")
def joint_run(request):
    config = request.config
    ctx = config._rf_context
    root = ctx["output"]
    # Collection created this fresh root before plugin log fixtures can use it.
    if not root.is_dir():
        raise ValueError("missing guarded evidence root")
    manifest = source_manifest()
    write_json(root / "source-manifest.json", manifest)
    write_json(root / "build.json", ctx["seal"])
    shutil.copyfile(ctx["manual"], root / "operator-airtime-record.txt")
    run = dict(schema=1, run=config.getoption("rf_run"), selected=[e.name for e in ctx["episodes"]],
               fixture=ctx["fixture"], elf=ctx["seal"]["files"]["cura_radio_component.elf"],
               status="INCOMPLETE", results=[], failures=[], source=digest(root / "source-manifest.json"))
    config._rf_run = run
    write_json(root / "run.json", run)
    state, passed = start_session(ctx["session"], session_identity(manifest, ctx["seal"], ctx["fixture"]), run["selected"])
    config._rf_session = state, passed
    remote = Remote(config.getoption("rf_host"), ctx["fixture"]["pi_user"], run["run"], root,
                    config.getoption("rf_host_key_alias"), config.getoption("rf_peer_python"))
    remote.stage(manifest, ctx["fixture"])
    remote.check_peer(run["selected"][0])
    yield ctx, run, remote


def test_rf_component(request, joint_run, episode_name, record_property):
    ctx, run, remote = joint_run
    episode = EPISODES[episode_name]
    root = ctx["output"] / episode_name
    root.mkdir()
    write_json(root / "episode.json", dict(case=episode_name, c6_max_packets=episode.c6_packets,
               pi_payload_lengths=episode.pi_lengths, charges=episode.charge, peer_lease_seconds=45))
    peer = None
    with episode_capture(run, episode_name, root, ctx["output"]) as cleanups:
        print(f"RF episode {episode_name}: {episode.charge}; operator owns admission/pacing.", flush=True)
        admit_episode(run["run"], episode_name, root,
                      ctx["output"] / "operator-airtime-record.txt",
                      request.config.getoption("rf_ready_run"))
        # All guards, identity/storage disclosure and source verification precede this fixture.
        verify_uart_identity(ctx["fixture"], root)
        dut = request.getfixturevalue("dut")
        node = Node(dut, ctx["fixture"], run["elf"], run["run"], episode_name, root)
        node.booted()
        peer = remote.peer(episode_name, root, run["source"])
        cleanups.append(("component peer", peer.close))
        peer.arm()
        node.phase(0)
        if episode_name == "RF-010.wake":
            node.phase(1)
        result = peer.finish()
        checked = verify_case(episode_name, node.events, result, run["run"], ctx["fixture"], run["elf"])
        run["results"].append(checked)
        record_property("rf_case", episode_name)
        record_property("rf_scope", "raw_component")

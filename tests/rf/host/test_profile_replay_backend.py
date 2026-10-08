"""F-002: replay actual peer/backend traces over the physical-port fake."""
import json
from pathlib import Path
import runpy
import threading

import pytest

from verify import EXPECTED, verify_pi_profiles

support = runpy.run_path(str(Path(__file__).with_name("test_peer.py")))
peer = support["peer"]


@pytest.mark.parametrize("case", tuple(EXPECTED))
def test_profile_replay_accepts_actual_peer_backend_traces(case, monkeypatch):
    clock = support["Clock"](monotonic_us=10000)
    air = support["Air"](clock)
    # Only physical I/O is replaced: retain TraceIo, Radio and Sx1262.
    for name in ("open", "close", "busy", "dio1", "set_reset", "transfer", "wait_edge", "resynchronize_events"):
        def forward(self, *args, _name=name, **kwargs):
            return getattr(air, _name)(*args, **kwargs)
        monkeypatch.setattr(peer.LinuxRadioIo, name, forward)
    downlinks = [frame for frame in EXPECTED[case][1] if frame]
    io = peer.TraceIo(clock, len(downlinks))
    io.transmit_deadline = clock.now_monotonic_us() + 45_000_000
    backend = peer.Sx1262(io, clock, clock)
    radio = None if case == "component.invalid_downlinks" else peer.Radio(backend)
    if radio:
        assert radio.initialize().state is peer.State.RX_SINGLE
    else:
        end = backend.deadline(2_000_000)
        backend.open(end)
        backend.initialize(end)
        backend.arm_receive(end)
    start = clock.now_monotonic_us()
    uplinks = EXPECTED[case][0]
    if case == "component.header_error_rearm": uplinks = [uplinks[0], None, uplinks[2]]
    elif case == "component.radio_absent": uplinks = []
    elif case == "component.initialized_sleep": uplinks = uplinks[:1]
    air.incoming.extend((start + 100_000 + i * 6_000_000, bytes.fromhex(frame) if frame is not None else None)
                        for i, frame in enumerate(uplinks))
    peer.execute(case, backend, radio, threading.Event(), start)
    if radio:
        assert radio.shutdown().safe_shutdown is True
    else:
        backend.safe_standby(backend.deadline(500_000))
        backend.close()
    # Use the same wire serialization as the recorded peer completion.
    trace = json.loads(json.dumps(io.trace, default=peer.encoded))
    assert sum(event["operation"] == "reset" for event in trace) == 2
    assert io.attempts == len(downlinks)
    verify_pi_profiles(trace, downlinks)

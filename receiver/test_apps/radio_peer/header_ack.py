"""Test-only interrupted downlinks; physical cutoff remains to be qualified."""
from test_apps.radio_peer import peer
from cura_receiver.sx1262 import IRQ_TX_DONE

CUT_US = 18_000
CUT_MAX_US = 20_000
START_LATE_US = 10_000


def send(backend, packet, frames, stop, *, interrupt_count=0):
    """Prepare before each target; no catch-up TX after a missed bound."""
    origin = packet["edge_timestamp_ns"] // 1000
    offsets = (100_000, 180_000, 260_000) if interrupt_count else (150_000,)
    peer.require(len(frames) <= len(offsets) and interrupt_count in (0, 2), "invalid HeaderErr burst")
    records = []
    for index, frame in enumerate(frames):
        target = origin + offsets[index]
        deadline = backend.deadline(500_000)
        backend.standby(deadline)
        backend.write_buffer(frame, deadline)
        backend.install_profile(transmit=True, payload_length=23, deadline=deadline)
        backend.account_stale_irqs(deadline)
        backend.clock.wait_until_monotonic_us(target)
        peer.require(not stop.is_set(), "peer cancelled before HeaderErr ACK")
        peer.require(backend.clock.now_monotonic_us() <= target + START_LATE_US,
                     "missed HeaderErr ACK target; no catch-up TX")
        # The lower-level SetTx uses the ordinary finite hardware watchdog.
        backend.start_tx(deadline)
        issued = backend.last_set_tx_issued_us
        peer.require(target <= issued <= target + START_LATE_US, "late HeaderErr SetTx")
        record = dict(frame=frame.hex(), target=target, set_tx=issued,
                      interrupted=index < interrupt_count)
        if record["interrupted"]:
            backend.clock.wait_until_monotonic_us(issued + CUT_US)
            before = backend.clock.now_monotonic_us()
            backend.standby(deadline)
            after = backend.clock.now_monotonic_us()
            # This timestamp brackets the test-peer call. Verification also
            # checks the actual SPI standby command within this bracket.
            peer.require(CUT_US <= before - issued <= CUT_MAX_US, "HeaderErr cutoff missed")
            irq = backend.read_irq(deadline)
            errors = backend.read_device_errors(deadline)
            peer.require(irq == 0 and errors == 0, "interrupted ACK completed or radio failed")
            backend.clear_irq(irq, deadline)
            record.update(abort_before=before, abort_after=after, irq=irq, device_errors=errors)
        else:
            edge = backend.wait_edge(deadline_monotonic_us=issued + 100_000)
            peer.require(edge is not None and issued <= edge.monotonic_us <= issued + 100_000,
                         "unconfirmed HeaderErr control ACK")
            event = backend.observe_event(deadline)
            backend.validate_event(event, transmit=True)
            peer.require(event.irq_status == IRQ_TX_DONE, "control ACK TX_DONE missing")
            backend.standby(deadline)
            backend.clear_irq(event.irq_status, deadline)
            record["tx_done"] = edge.monotonic_us
        records.append(record)
    deadline = backend.deadline(500_000)
    backend.install_profile(transmit=False, payload_length=255, deadline=deadline)
    backend.arm_receive(deadline)
    return records

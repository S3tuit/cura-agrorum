from types import SimpleNamespace

import pytest

from finalization import Finalization


def test_receipt_waits_for_retained_records_and_retries_only_fresh_greetings():
    clock = SimpleNamespace(now=0)
    sent, armed = [], []
    receipt = Finalization(lambda: (1, "SLEEP") if armed else None, sent.append, clock=lambda: clock.now)
    receipt.feed(b"RF_END {}\nRF_SLEEP\nRF_FINALIZE 1234")
    receipt.feed(b"5678\r\n")
    assert sent == []
    armed.append(True)
    receipt.feed(b"RF_FINALIZE 12345678\n")
    assert sent == [b"RF_FINALIZE_ACK 12345678 SLEEP\n"]
    clock.now = 1
    receipt.feed(b"")
    assert len(sent) == 1
    receipt.feed(b"RF_FINALIZE 12345678\n")
    assert len(sent) == 2


def test_stop_park_requires_current_receipt_and_blocks_another_boundary():
    sent = []
    receipt = Finalization(lambda: (1, "STOP"), sent.append)
    receipt.feed(b"RF_FINALIZE 12345678\nRF_PARKED 12345678\n")
    assert receipt.parked.is_set() and len(sent) == 1
    receipt.select = lambda: (2, "SLEEP")
    with pytest.raises(ValueError, match="progression"):
        receipt.feed(b"RF_FINALIZE 87654321\n")


@pytest.mark.parametrize("data", [b"RF_FINALIZE bad\n", b"RF_FINALIZE_FAILED 12345678\n",
    b"RF_PARKED 12345678\n", b"x" * 4097])
def test_incomplete_or_unbound_evidence_cannot_pass(data):
    receipt = Finalization(lambda: (1, "SLEEP"), lambda _: None)
    with pytest.raises((ValueError, RuntimeError)):
        receipt.feed(data)
    assert receipt.failure is not None and not receipt.parked.is_set()


def test_same_boundary_cannot_change_nonce_or_directive():
    for change in ("nonce", "action"):
        receipt = Finalization(lambda: (1, "SLEEP"), lambda _: None)
        receipt.feed(b"RF_FINALIZE 12345678\n")
        if change == "action":
            receipt.select = lambda: (1, "STOP")
        with pytest.raises(ValueError, match="changed"):
            receipt.feed(b"RF_FINALIZE " + (b"deadbeef" if change == "nonce" else b"12345678") + b"\n")

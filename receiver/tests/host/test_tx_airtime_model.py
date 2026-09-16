"""Reviewed primitive examples establish the independent model before differential tests."""

from tests.support.models.tx_airtime import LedgerModel


# A current-process increment is unusable during uncertainty and becomes a baseline after restart.
def test_model_unknown_increment_and_restart():
    model = LedgerModel()
    assert model.acquire("unknown_installed") == "PERSISTENCE_PENDING"
    assert model.available == 0 and model.total is None
    assert model.reconcile() == "ALLOWED"
    assert model.available == 8_000_000 and model.durable == (
        2,
        0,
        ((8_000_000, 3_780_000_000),),
    )
    model.restart(True, 0)
    assert model.available == 0 and model.acquire() == "BUDGET_EXHAUSTED"


# A settlement retains definite/possible attempts, refunds only non-start, and precharges a distinct minute.
def test_model_reviewed_certainty_and_settlement():
    model = LedgerModel()
    assert model.acquire() == "ALLOWED"
    for certainty in ("NOT_STARTED", "STARTED", "UNCERTAIN", "MISSING"):
        assert model.spend(certainty) == "ALLOWED"
    assert model.available == 7_796_402
    model.advance(60_000_000)
    assert model.settle(True) == "ALLOWED"
    assert model.durable == (
        3,
        60_000_000,
        ((203_598, 3_780_000_000), (8_000_000, 3_840_000_000)),
    )
    assert model.total == 8_203_598


# Opposite UTC offsets retain the old logical identity and cannot make an unacknowledged increment spendable.
def test_model_reviewed_trust_change():
    model = LedgerModel()
    model.acquire()
    model.spend("MISSING")
    model.advance(10_000_000)
    model.set_trust(True, -30_000_000)
    assert model.available == 0
    assert model.settle(True) == "ALLOWED"
    assert model.durable == (
        3,
        -20_000_000,
        ((8_000_000, 3_720_000_000), (67_866, 3_780_000_000)),
    )


# A nominal UTC expiration does not bypass conservative live retention or permit an invalid snapshot.
def test_model_reviewed_snapshot_deferral():
    model = LedgerModel(((1, 10_000_000),))
    model.advance(10_000_000)
    assert model.acquire() == "SNAPSHOT_DEFERRED"
    assert model.total == 1 and model.durable[0] == 1
    model.advance(37_000)
    assert model.acquire() == "ALLOWED"
    assert model.durable == (2, 10_037_000, ((8_000_000, 3_790_037_000),))

"""Literal reviewed examples anchor the independent rational/list oracle."""
from tests.support.models.tx_airtime import LedgerModel


def test_model_unknown_increment_and_restart():
    model = LedgerModel()
    assert model.acquire('unknown_installed') == 'PERSISTENCE_PENDING'
    assert model.available == 0 and model.total is None
    assert model.reconcile() == 'ALLOWED'
    assert model.available == 8_000_000
    assert model.durable == (2, (0, 1), (0,) * 61 + (8_000_000,))
    model.restart(False, 0)
    assert model.available == 0 and model.acquire() == 'BUDGET_EXHAUSTED'
    model.advance(60_222_000)
    assert model.acquire() == 'ALLOWED'
    assert model.total == 16_000_000


def test_model_reviewed_certainty_and_settlement():
    model = LedgerModel()
    assert model.acquire() == 'ALLOWED'
    for certainty in ('NOT_STARTED', 'STARTED', 'UNCERTAIN', 'MISSING'):
        assert model.spend(certainty) == 'ALLOWED'
    assert model.available == 7_796_402
    model.advance(60_222_000)
    assert model.settle(True) == 'ALLOWED'
    assert model.durable[2] == (0,) * 60 + (203_598, 8_000_000)
    assert model.total == 8_203_598


def test_model_reviewed_trust_change():
    model = LedgerModel()
    model.acquire()
    model.spend('MISSING')
    model.advance(10_000_000)
    model.set_trust(False, 0)
    assert model.available == 7_932_134
    assert model.settle(True) == 'ALLOWED'
    assert model.durable == (3, None, (0,) * 61 + (8_000_000,))


def test_model_partial_recovery_limits_top_up_lifetime():
    model = LedgerModel((1,))
    model.advance(30_000_000)
    model.restart(True, 0)
    assert model.acquire() == 'ALLOWED'
    assert model.grant['deadline'] == 29_667_101 + 30_000_000
    assert model.total == 8_000_000


def test_model_snapshot_does_not_restart_retention():
    model = LedgerModel()
    model.acquire()
    deadline = model.live[0][2]
    model.spend('STARTED')
    model.advance(20_000_000)
    model.settle(True)
    assert model.live[0][2] == deadline == 3_673_793_025

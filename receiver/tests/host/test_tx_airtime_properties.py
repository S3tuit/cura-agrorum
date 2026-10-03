"""Production policy + real SQLite, checked by a separate physical-history oracle."""
from pathlib import Path
import json
import random
import pytest
from cura_receiver.tx_airtime import TxCertainty
from tests.support.coordination.airtime import PhysicalRun


@pytest.mark.parametrize('seed', range(8))
def test_production_histories_against_physical_oracle(airtime_component, seed):
    run = PhysicalRun(airtime_component, longest=200_000)
    rng = random.Random(seed+4004)
    modes = ('committed', 'not_installed', 'unknown_installed', 'unknown_absent')
    for step in range(160):
        run.trust(rng.random() > .08, offset=rng.choice((-1_000_000,0,1_000_000)))
        action = rng.randrange(7)
        if action == 0:
            run.restart(trusted=rng.random() > .15, fresh_clock=rng.random() < .5)
        elif action == 1:
            run.channel.mode = rng.choice(modes)
            run.save(rtc=rng.random() < .5)
        elif action == 2:
            run.channel.fail_load = rng.random() < .25
            run.policy.maintain(deadline_monotonic_us=run.deadline())
            run.channel.fail_load = False
        else:
            for _ in range(rng.randrange(1, 35)):
                charge = rng.choice((50_000,67_866,100_000,200_000))
                certainty = rng.choice((TxCertainty.STARTED,TxCertainty.UNCERTAIN,TxCertainty.NOT_STARTED))
                if not run.transmit(charge, certainty):
                    break
                run.advance(251_000, rng.choice((996300,1_000_000,1_003_700)))
        run.advance(rng.choice((0,1,60_000_000,1_800_000_000,3_700_000_000)),
                    rng.choice((996300,1_000_000,1_003_700)))
        run.check()


def test_lost_save_restart_requests_against_production(airtime_component):
    data = json.loads((Path(__file__).parents[1]/'support/data/airtime_regression_requests.json').read_text())
    assert data['result']['contract_violated']
    run = PhysicalRun(airtime_component, empty=False)
    origin = data['events'][0][1]
    for kind, physical, *extra in data['events']:
        run.advance(physical-origin-int(run.physical))
        if kind == 'boot':
            run.restart(trusted=extra[0])
        elif kind == 'save':
            run.channel.fail_load = False
            if run.policy.owner.pending:
                run.policy.reconcile(deadline_monotonic_us=run.deadline())
            run.save()
        elif kind == 'lost_save':
            run.channel.mode = 'unknown_installed'
            run.save()
        elif kind == 'tx':
            run.transmit()
    assert run.sent < 609 and run.denied > 0
    run.check()

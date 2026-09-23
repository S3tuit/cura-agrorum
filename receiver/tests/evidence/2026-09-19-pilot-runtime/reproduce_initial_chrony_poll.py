from pathlib import Path
from types import SimpleNamespace
import hashlib,json,runpy
from cura_receiver.communicator_scheduler import CommunicatorScheduler
from cura_receiver.generated import receiver_enums_generated as E

factory=runpy.run_path('receiver/tests/host/test_runtime_time.py')['runtime']
def reproduce(advancing):
    rt,clock,_,_=factory()
    original=clock.now_monotonic_us
    def read():
        if advancing: clock.advance_elapsed_us(1)
        return original()
    clock.now_monotonic_us=read
    c=SimpleNamespace(time=rt,clock=clock,radio=SimpleNamespace(state=E.RadioState.RX_SINGLE),occurrence_sequence=0,
        receive_once=lambda **_:SimpleNamespace(finalization=None,radio_episodes=()),
        airtime=SimpleNamespace(update_time=lambda *a,**k:None,acquire_grant=lambda **_:None))
    scheduler=CommunicatorScheduler(c,chrony=None,rtc=None,health_interval_us=60_000_000)
    polls=[]
    scheduler._poll_time=lambda:polls.append(read())
    scheduler._rtc_due=lambda:None
    turns=[scheduler.run_once().work.name for _ in range(100)]
    return dict(clock_advances_each_read=advancing,turns=100,chrony_poll_dispatches=len(polls),work_counts={x:turns.count(x) for x in set(turns)})
paths=['receiver/cura_receiver/runtime_time.py','receiver/cura_receiver/communicator_scheduler.py']
print(json.dumps(dict(results=[reproduce(False),reproduce(True)],sources={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}),indent=2))

import sqlite3
import pytest
from reading_baseline import capture_readings, new_readings
from pathlib import Path
import runpy
_support = runpy.run_path(str(Path(__file__).with_name("test_service.py")))
transcript, NODE, KEY, INSTANCE = (_support[k] for k in ("transcript", "NODE", "KEY", "INSTANCE"))
from verify_service import verify_service


def test_readonly_compact_capture(tmp_path):
    p=tmp_path/'db'; transcript(p)
    original=p.read_bytes()
    before=capture_readings(p,NODE)
    assert set(before['rows']) == {'100','101'}
    assert all(len(h)==64 for h in before['rows'].values())
    assert p.read_bytes()==original
    with sqlite3.connect(p) as db:
        db.row_factory=sqlite3.Row
        rows=[dict(r) for r in db.execute('select * from reading_messages order by message_id')]
    assert new_readings(rows,before,NODE)==[]
    assert new_readings(rows[1:],dict(schema=1,node_id=NODE.hex(),rows={}),NODE)==rows[1:]
    with pytest.raises(ValueError,match='disappeared'):
        new_readings(rows[1:],before,NODE)
    rows[0]['soil_0_mv']+=1
    with pytest.raises(ValueError,match='changed'):
        new_readings(rows,before,NODE)
    with pytest.raises(ValueError,match='identity'):
        new_readings(rows,before,b'x'*8)


def test_existing_history_does_not_count_as_new_wakes(tmp_path):
    p=tmp_path/'db';decoded=transcript(p)
    # An old immutable row in a separate instance, while preserving FK bindings.
    with sqlite3.connect(p) as db:
        db.execute('PRAGMA foreign_keys=ON')
        db.execute("insert into receiver_instances(instance_ordinal,receiver_instance_id,linux_boot_id,started_at_monotonic_us) values(2,?,?,1)",(b'o'*16,b'b'*16))
        db.row_factory=sqlite3.Row
        profile=dict(db.execute('select * from message_profiles where occurrence_sequence=1').fetchone())
        profile.update(receiver_instance_id=b'o'*16, claimed_message_id=1, decoded_sample_id=1)
        cols=list(profile)
        db.execute('insert into message_profiles('+','.join(cols)+') values('+','.join('?' for _ in cols)+')',list(profile.values()))
        old=dict(db.execute('select * from reading_messages where message_id=100').fetchone())
        old.update(message_id=1,sample_id=1,first_receiver_instance_id=b'o'*16)
        cols=list(old)
        db.execute('insert into reading_messages('+','.join(cols)+') values('+','.join('?' for _ in cols)+')',list(old.values()))
    all_rows=capture_readings(p,NODE)
    before=dict(all_rows,rows={'1':all_rows['rows']['1']})
    assert verify_service(p,NODE,KEY,decoded,INSTANCE,before)['wakes']==2
    with pytest.raises(ValueError,match='missing/extra canonical'):
        verify_service(p,NODE,KEY,decoded,INSTANCE,all_rows)

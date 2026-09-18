import os, json, subprocess, hashlib
from pathlib import Path
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v2')
code=Path('/opt/cura-pilot-vuh7p781');root=Path('/var/lib/cura-pilot-vuh7p781')
env={**os.environ,'PYTHONPATH':str(code/'receiver')+':'+str(code/'protocol/protocol-v2-lora/python'),'CURA_RECEIVER_TEST_ROOT':str(root),'CURA_RECEIVER_CONFIGURATION':str(root/'config/receiver-group.json'),'CURA_RECEIVER_DATABASE':str(root/'data/receiver.sqlite3'),'SQLITE_TMPDIR':str(root/'data/tmp')}
python=str(code/'venv/bin/python');prefix=['runuser','-u','cura-receiver','--',python]
subprocess.run(prefix+['-m','cura_receiver.storage_preflight'],env=env,check=True)
script='''import os,json,sqlite3,sys
from pathlib import Path
from cura_receiver.application_environment import settings_from_environment
from cura_receiver.sqlite_database import open_receiver_database
from cura_protocol_v2_lora.receiver_group import load_receiver_group
import cryptography,gpiod,spidev
s=settings_from_environment(os.environ)
g=load_receiver_group(s.configuration_path)
r=open_receiver_database(s.database_path,g.group_id,minimum_free_bytes=s.minimum_free_bytes)
assert r.failure is None and r.database is not None, r.failure
db=r.database
pragmas={name:db.connection.execute('PRAGMA '+name).fetchone()[0] for name in ('journal_mode','synchronous','foreign_keys','busy_timeout')}
assert pragmas=={'journal_mode':'wal','synchronous':2,'foreign_keys':1,'busy_timeout':250},pragmas
assert db.connection.execute('PRAGMA integrity_check').fetchone()[0]=='ok'
v=os.statvfs(s.database_path.parent)
report={'uid':os.getuid(),'gid':os.getgid(),'groups':os.getgroups(),'python':sys.version,'sqlite':sqlite3.sqlite_version,'cryptography':cryptography.__version__,'pragmas':pragmas,'group_id':db.group_id.hex(),'free_bytes':v.f_bavail*v.f_frsize,'reserve_bytes':s.minimum_free_bytes,'real_preflight':'PASS','database_integrity':'ok','production_module':__import__('cura_receiver').__path__[0]}
db.close()
print(json.dumps(report,indent=2))
'''
r=subprocess.run(prefix+['-c',script],env=env,check=True,capture_output=True,text=True)
report=json.loads(r.stdout)
report['mounts']={str(p):subprocess.check_output(['findmnt','-T',str(p),'-n','-o','TARGET,SOURCE,FSTYPE,OPTIONS'],text=True).strip() for p in (code,root/'config',root/'data',root/'data/tmp')}
report['permissions']={str(p):{'uid':p.stat().st_uid,'gid':p.stat().st_gid,'mode':oct(p.stat().st_mode&0o777),'nonsymlink':not p.is_symlink()} for p in (root,root/'config',root/'config/receiver-group.json',root/'data',root/'data/receiver.sqlite3',root/'data/tmp')}
m=json.loads((code/'SOURCE_MANIFEST.json').read_text());assert all(hashlib.sha256((code/k).read_bytes()).hexdigest()==v for k,v in m['files_sha256'].items())
report['installed_source_files_verified']=len(m['files_sha256'])
(stage/'PACKAGE_VERIFICATION.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))

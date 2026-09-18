from pathlib import Path
import subprocess,time,json,sqlite3
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v5');unit='cura-pilot-vuh7p781.service'
def command(*args):return subprocess.check_output(args,text=True)
def rows():
 with sqlite3.connect('file:/var/lib/cura-pilot-vuh7p781/data/receiver.sqlite3?mode=ro',uri=True) as c:
  return [list(r) for r in c.execute('select instance_ordinal,hex(receiver_instance_id),hex(linux_boot_id),clean_stopped_at_monotonic_us from receiver_instances')]
report={'before':rows()}
start=time.monotonic();subprocess.run(['systemctl','stop',unit],check=True);report['stop_seconds']=time.monotonic()-start
report['after_stop']=rows();report['stop_properties']=command('systemctl','show',unit,'-p','ActiveState','-p','Result','-p','ExecMainStatus')
report['inhibitors_after_stop']=command('systemd-inhibit','--list','--no-pager')
report['journal']=command('journalctl','-u',unit,'-b','--no-pager','-o','short-monotonic')
(stage/'CLEAN_STOP.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
assert report['stop_seconds']<15
assert report['after_stop'][-1][-1] is not None,'missing clean stop marker'
assert 'cura-receiver' not in report['inhibitors_after_stop']
subprocess.run(['systemctl','start',unit],check=True);time.sleep(6)
report['after_restart']=rows()
assert report['after_restart'][-1][0]==report['before'][-1][0]+1
assert report['after_restart'][-1][1]!=report['before'][-1][1]
assert report['after_restart'][-1][2]==report['before'][-1][2]
assert command('systemctl','is-active',unit).strip()=='active'
(stage/'CLEAN_RESTART.json').write_text(json.dumps(report,indent=2)+'\n');print('Clean restart identity PASS')

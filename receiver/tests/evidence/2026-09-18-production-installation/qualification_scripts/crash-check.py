from pathlib import Path
import subprocess,time,json,sqlite3
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v5');unit='cura-pilot-vuh7p781.service'
def rows():
 with sqlite3.connect('file:/var/lib/cura-pilot-vuh7p781/data/receiver.sqlite3?mode=ro',uri=True) as c:
  return [list(r) for r in c.execute('select instance_ordinal,hex(receiver_instance_id),hex(linux_boot_id),clean_stopped_at_monotonic_us from receiver_instances')]
report={'before':rows()};start=time.monotonic()
subprocess.run(['systemctl','kill','--kill-whom=all','--signal=SIGKILL',unit],check=True)
time.sleep(1)
report['one_second']=subprocess.check_output(['systemctl','show',unit,'-p','ActiveState','-p','SubState','-p','NRestarts'],text=True)
assert 'SubState=auto-restart' in report['one_second']
for _ in range(20):
 time.sleep(1)
 after=rows()
 if len(after)>len(report['before']):break
report['after']=after;report['elapsed_s']=time.monotonic()-start
report['journal']=subprocess.check_output(['journalctl','-u',unit,'-b','--no-pager','-o','short-monotonic'],text=True)
(stage/'CRASH_RESTART.json').write_text(json.dumps(report,indent=2)+'\n')
assert len(after)==len(report['before'])+1
assert after[-2][-1] is None and after[-1][2]==after[-2][2]
assert after[-1][1]!=after[-2][1] and report['elapsed_s']>=5
print('Crash restart PASS',report['elapsed_s'])
subprocess.run(['systemctl','stop',unit],check=True)

from pathlib import Path
import subprocess,time,json,sqlite3
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v5');unit='cura-pilot-vuh7p781.service';bootunit='cura-pilot-rtc-vuh7p781.service'
def cmd(*a):return subprocess.check_output(a,text=True)
def data():
 with sqlite3.connect('file:/var/lib/cura-pilot-vuh7p781/data/receiver.sqlite3?mode=ro',uri=True) as c:
  rows=[list(r) for r in c.execute('select instance_ordinal,hex(receiver_instance_id),hex(linux_boot_id),clean_stopped_at_monotonic_us from receiver_instances')]
  observations=[list(r) for r in c.execute('select observation_sequence,code from clock_observations join system_time_quality_codes on system_time_quality_id=id where receiver_instance_id=(select receiver_instance_id from receiver_instances order by instance_ordinal desc limit 1) order by observation_sequence')]
 return {'instances':rows,'latest_clock_observations':observations}
report={'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'data':data(),'bootstrap':cmd('journalctl','-b','-u',bootunit,'--no-pager','-o','cat'),'ordering':cmd('systemctl','show',unit,'-p','After','-p','Requires','-p','Wants','-p','ActiveEnterTimestampMonotonic'),'bootstrap_properties':cmd('systemctl','show',bootunit,'-p','ExecMainStartTimestampMonotonic','-p','ExecMainExitTimestampMonotonic','-p','Result'),'chrony_tracking':cmd('chronyc','-n','-c','-h','/run/chrony/chronyd.sock','tracking')}
(stage/'OFFLINE_BOOT.json').write_text(json.dumps(report,indent=2)+'\n')
assert report['boot_id']!=json.loads((stage/'OFFLINE_BOOT_INPUTS.json').read_text())['old_boot_id']
assert 'MISSING' in report['bootstrap'],report['bootstrap']
assert cmd('systemctl','is-active',unit).strip()=='active'
assert report['data']['latest_clock_observations'] and all(row[1]=='UNTRUSTED' for row in report['data']['latest_clock_observations'])
assert 'network-online.target' not in report['ordering']
before=cmd('systemctl','show',bootunit,'-p','ExecMainStartTimestampMonotonic')
subprocess.run(['systemctl','restart',unit],check=True);time.sleep(6)
assert cmd('systemctl','show',bootunit,'-p','ExecMainStartTimestampMonotonic')==before
report['after_receiver_restart']=data()
subprocess.run(['systemctl','restart',bootunit],check=True)
report['bootstrap_after_explicit_restart']=cmd('journalctl','-b','-u',bootunit,'--no-pager','-o','cat')
assert 'ALREADY_ATTEMPTED' in report['bootstrap_after_explicit_restart']
(stage/'OFFLINE_BOOT.json').write_text(json.dumps(report,indent=2)+'\n');print('Offline missing-RTC boot, receiver restart and bootstrap once-per-boot PASS')

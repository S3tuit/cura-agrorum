from pathlib import Path
import subprocess,time,json,sqlite3,hashlib
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v5');unit='cura-pilot-vuh7p781.service';bootunit='cura-pilot-rtc-vuh7p781.service'
def cmd(*a):return subprocess.check_output(a,text=True)
inputs=json.loads((stage/'UNCLEAN_REBOOT_INPUTS.json').read_text());boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip();assert boot!=inputs['old_boot_id']
assert cmd('systemctl','is-active',unit).strip()=='active'
time.sleep(6)
with sqlite3.connect('file:/var/lib/cura-pilot-vuh7p781/data/receiver.sqlite3?mode=ro',uri=True) as c:
 rows=[list(r) for r in c.execute('select instance_ordinal,hex(receiver_instance_id),hex(linux_boot_id),clean_stopped_at_monotonic_us from receiver_instances')]
prior=[r for r in rows if r[1]==inputs['killed_instance'][1]];assert len(prior)==1 and prior[0][-1] is None
assert rows[-1][2]!=prior[0][2] and rows[-1][1]!=prior[0][1]
journal=cmd('journalctl','-b','-u',bootunit,'--no-pager','-o','cat')
assert 'COPIED' in journal or 'ALREADY_SYNCHRONIZED' in journal
assert Path('/etc/chrony/chrony.conf').read_bytes()==(stage/'chrony-before-offline.conf').read_bytes()
assert not Path('/etc/systemd/system/'+bootunit+'.d/90-test-missing-rtc.conf').exists()
assert not Path('/run/systemd/system/'+unit+'.d/90-test-device-denial.conf').exists()
report={'boot_id':boot,'instances':rows,'bootstrap_journal':journal,'inhibitors_while_active':cmd('systemd-inhibit','--list','--no-pager'),'service_properties':cmd('systemctl','show',unit,'-p','ActiveState','-p','NRestarts','-p','After','-p','ReadWritePaths'),'nominal_configuration_restored':True}
assert 'cura-receiver' in report['inhibitors_while_active']
subprocess.run(['python3',str(stage/'check-time-root.py')],check=True)
start=time.monotonic();subprocess.run(['systemctl','stop',unit],check=True);report['final_stop_seconds']=time.monotonic()-start
subprocess.run(['systemctl','disable',unit],check=True)
with sqlite3.connect('file:/var/lib/cura-pilot-vuh7p781/data/receiver.sqlite3?mode=ro',uri=True) as c:
 report['final_clean_marker']=c.execute('select clean_stopped_at_monotonic_us from receiver_instances order by instance_ordinal desc limit 1').fetchone()[0]
 report['integrity_check']=c.execute('pragma integrity_check').fetchone()[0]
assert report['final_clean_marker'] is not None and report['integrity_check']=='ok'
report['final_inhibitors']=cmd('systemd-inhibit','--list','--no-pager');assert 'cura-receiver' not in report['final_inhibitors']
report['final_service']=cmd('systemctl','show',unit,'-p','ActiveState','-p','UnitFileState','-p','Result')
report['chrony_active']=cmd('systemctl','is-active','chrony.service').strip()
code=Path('/opt/cura-pilot-vuh7p781');m=json.loads((stage/'SOURCE_MANIFEST.json').read_text());assert all(hashlib.sha256((code/k).read_bytes()).hexdigest()==v for k,v in m['files_sha256'].items())
report['installed_source_files_verified']=len(m['files_sha256'])
(stage/'FINAL_BOOT_RESTORATION.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

from pathlib import Path
import subprocess,time,json,sqlite3,shutil
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v5');unit='cura-pilot-vuh7p781.service'
subprocess.run(['systemctl','stop',unit],check=True)
subprocess.run(['systemctl','stop','chrony.service'],check=True)
shutil.copyfile(stage/'chrony-before-offline.conf','/etc/chrony/chrony.conf')
Path('/etc/systemd/system/cura-pilot-rtc-vuh7p781.service.d/90-test-missing-rtc.conf').unlink()
subprocess.run(['systemctl','daemon-reload'],check=True)
subprocess.run(['systemctl','start',unit],check=True);time.sleep(6)
with sqlite3.connect('file:/var/lib/cura-pilot-vuh7p781/data/receiver.sqlite3?mode=ro',uri=True) as c:
 row=list(c.execute('select instance_ordinal,hex(receiver_instance_id),hex(linux_boot_id),clean_stopped_at_monotonic_us from receiver_instances order by instance_ordinal desc limit 1').fetchone())
assert row[-1] is None
report={'old_boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'killed_instance':row,'nominal_time_configuration_restored':True,'fixture':'SIGKILL entire test receiver control group immediately followed by orderly Pi reboot; not physical power removal'}
(stage/'UNCLEAN_REBOOT_INPUTS.json').write_text(json.dumps(report,indent=2)+'\n')
subprocess.run(['systemctl','kill','--kill-whom=all','--signal=SIGKILL',unit],check=True)
subprocess.run(['systemctl','reboot'],check=True)

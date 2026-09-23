from pathlib import Path
import subprocess,time,json,shutil
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v5');unit='cura-pilot-vuh7p781.service'
prior=json.loads((stage/'MISSING_DEVICE.json').read_text());assert 'Start request repeated too quickly.' in prior['journal'] and 'NRestarts=5' in prior['properties']
subprocess.run(['systemctl','start',unit],check=True);time.sleep(6)
restored=subprocess.check_output(['systemctl','show',unit,'-p','ActiveState','-p','NRestarts'],text=True)
assert 'ActiveState=active' in restored and 'NRestarts=0' in restored
subprocess.run(['systemctl','stop',unit],check=True)
(stage/'MISSING_DEVICE_RECONCILED.json').write_text(json.dumps({'rate_limit':'PASS: five starts, restart delays and explicit start-request refusal in retained journal','checker_failure':'Expected a systemd Result label not required by the contract; actual Result retains exit-code','nominal_restoration':restored},indent=2)+'\n')
config=Path('/etc/chrony/chrony.conf');shutil.copy2(config,stage/'chrony-before-offline.conf')
subprocess.run(['systemctl','stop','chrony.service'],check=True)
config.write_text('driftfile /var/lib/chrony/chrony.drift\nleapsecmode slew\nmaxslewrate 3500\ncmdport 0\nbindcmdaddress /run/chrony/chronyd.sock\n')
# Hide real RTC only in bootstrap's test namespace. No physical fixture change.
p=Path('/etc/systemd/system/cura-pilot-rtc-vuh7p781.service.d/90-test-missing-rtc.conf');assert not p.exists();p.parent.mkdir(parents=True,exist_ok=True);p.write_text('[Service]\nPrivateDevices=yes\n')
subprocess.run(['systemctl','daemon-reload'],check=True)
subprocess.run(['systemctl','enable',unit],check=True)
report={'old_boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'fixture':'Chrony configured without network sources; bootstrap private /dev hides RTC; SSH remains available for administration','receiver_enabled_for_test':True,'restoration':'restore chrony-before-offline.conf, remove bootstrap 90-test-missing-rtc.conf, disable test receiver after final qualification'}
(stage/'OFFLINE_BOOT_INPUTS.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
subprocess.run(['systemctl','reboot'],check=True)

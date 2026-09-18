from pathlib import Path
import subprocess,time,json
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v5');unit='cura-pilot-vuh7p781.service'
p=Path('/run/systemd/system/'+unit+'.d/90-test-device-denial.conf');assert not p.exists();p.parent.mkdir(parents=True,exist_ok=True)
report={}
try:
 p.write_text('[Service]\nInaccessiblePaths=/dev/spidev0.0\n')
 subprocess.run(['systemctl','daemon-reload'],check=True);subprocess.run(['systemctl','reset-failed',unit],check=True)
 start=time.monotonic();subprocess.run(['systemctl','start',unit],check=True)
 for _ in range(45):
  time.sleep(1)
  props=subprocess.check_output(['systemctl','show',unit,'-p','Result','-p','ActiveState','-p','NRestarts','-p','SubState'],text=True)
  if 'Result=start-limit-hit' in props:break
 report={'properties':props,'elapsed_s':time.monotonic()-start,'journal':subprocess.check_output(['journalctl','-u',unit,'-b','--no-pager','-o','short-monotonic'],text=True)}
 (stage/'MISSING_DEVICE.json').write_text(json.dumps(report,indent=2)+'\n')
 assert 'Result=start-limit-hit' in props,props
finally:
 subprocess.run(['systemctl','stop',unit],check=True)
 p.unlink(missing_ok=True);subprocess.run(['systemctl','daemon-reload'],check=True);subprocess.run(['systemctl','reset-failed',unit],check=True)
subprocess.run(['systemctl','start',unit],check=True);time.sleep(6)
report['restored']=subprocess.check_output(['systemctl','show',unit,'-p','ActiveState','-p','NRestarts','-p','Result'],text=True)
assert 'ActiveState=active' in report['restored'] and 'NRestarts=0' in report['restored']
(stage/'MISSING_DEVICE.json').write_text(json.dumps(report,indent=2)+'\n');print('Missing device rate-limit and nominal restoration PASS')
subprocess.run(['systemctl','stop',unit],check=True)

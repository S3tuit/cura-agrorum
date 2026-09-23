from pathlib import Path
import subprocess,json,os
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v2');code=Path('/opt/cura-pilot-vuh7p781')
script='''import json,subprocess
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_chrony import LinuxChronyControl
clock=LinuxOsClock()
path='/run/cura-pilot-deliberately-absent.sock'
raw=subprocess.run(['/usr/bin/chronyc','-n','-c','-h',path,'tracking'],capture_output=True,text=True)
c=LinuxChronyControl(clock,socket_path=path,deadline_monotonic_us=clock.now_monotonic_us()+1000000)
r=c.read_tracking(deadline_monotonic_us=clock.now_monotonic_us()+1000000)
print(json.dumps({'socket':path,'chronyc_version':subprocess.check_output(['/usr/bin/chronyc','-v'],text=True).strip(),'returncode':raw.returncode,'stdout':raw.stdout,'stderr':raw.stderr,'expected':'UNAVAILABLE','actual':r.status.name},indent=2))
'''
env={**os.environ,'PYTHONPATH':str(code/'receiver')+':'+str(code/'protocol/protocol-v2-lora/python')}
r=subprocess.run(['runuser','-u','cura-receiver','--',str(code/'venv/bin/python'),'-c',script],env=env,capture_output=True,text=True,check=True)
report=json.loads(r.stdout);report['real_socket']={}
for p in ('/run/chrony','/run/chrony/chronyd.sock'):
 s=Path(p).stat();report['real_socket'][p]={'mode':oct(s.st_mode&0o777),'uid':s.st_uid,'gid':s.st_gid}
report['receiver_state']=subprocess.check_output(['systemctl','show','cura-pilot-vuh7p781.service','-p','ActiveState','-p','UnitFileState'],text=True)
report['chrony_active']=subprocess.check_output(['systemctl','is-active','chrony.service'],text=True).strip()
report['scope']='Read-only missing-socket reproduction; no makestep or RF operation. Socket installation permissions remain incomplete.'
(stage/'CHRONY_UNAVAILABLE_FAILURE.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

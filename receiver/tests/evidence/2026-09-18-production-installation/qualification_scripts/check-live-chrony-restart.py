from pathlib import Path
import subprocess,time,json
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v5');unit='cura-pilot-vuh7p781.service'
report={}
try:
 subprocess.run(['systemctl','start',unit],check=True);time.sleep(6)
 pid=subprocess.check_output(['systemctl','show',unit,'--value','-p','MainPID'],text=True).strip()
 def probe():
  command=['nsenter','--target',pid,'--mount','--','runuser','-u','cura-receiver','--','/usr/bin/chronyc','-n','-c','-h','/run/chrony/chronyd.sock','tracking']
  r=subprocess.run(command,capture_output=True,text=True,timeout=4)
  return {'returncode':r.returncode,'stdout':r.stdout,'stderr':r.stderr}
 report['before']=probe();report['directory_inode_before']=Path('/run/chrony').stat().st_ino
 subprocess.run(['systemctl','restart','chrony.service'],check=True)
 report['after']=probe();report['directory_inode_after']=Path('/run/chrony').stat().st_ino
 report['same_receiver_pid']=subprocess.check_output(['systemctl','show',unit,'--value','-p','MainPID'],text=True).strip()==pid
 report['scope']='Real chronyc under receiver UID in the running receiver mount namespace, before and after Chrony restart; no step or RF command'
 (stage/'LIVE_CHRONY_RESTART.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2))
 assert report['before']['returncode']==0 and report['after']['returncode']==0 and report['same_receiver_pid']
finally:subprocess.run(['systemctl','stop',unit],check=True)

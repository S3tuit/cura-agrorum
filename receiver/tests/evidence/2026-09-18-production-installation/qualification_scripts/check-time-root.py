from pathlib import Path
import subprocess,json,sys
stage=Path('/var/tmp/cura-pilot-20260918-vuh7p781-v5');code=Path('/opt/cura-pilot-vuh7p781')
sys.path.insert(0,str(code/'receiver'));from tools.check_chrony import check_configuration
raw=subprocess.check_output(['systemctl','show','chrony.service','-p','ActiveState','-p','ExecStartEx','-p','ExecStartPre','-p','MainPID','-p','User'],text=True)
p=dict(line.split('=',1) for line in raw.splitlines())
assert p['ActiveState']=='active';assert 'flags=no-setuid' in p['ExecStartEx']
args=Path('/proc/'+p['MainPID']+'/cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
assert args==['/usr/sbin/chronyd','-F','1','-f','/etc/chrony/chrony.conf']
assert 'ignore_errors=no' in p['ExecStartPre'] and '/usr/libexec/cura-agrorum/check-chrony.py' in p['ExecStartPre']
policy=check_configuration('/etc/chrony/chrony.conf')
report={'systemd':p,'actual_arguments':args,'policy':policy,'bootstrap_journal':subprocess.check_output(['journalctl','-u','cura-pilot-rtc-vuh7p781.service','-b','--no-pager','-o','cat'],text=True)}
(stage/'CHRONY_INSTALLED_STARTUP.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
test=code/'check-time-user.py';test.write_bytes((stage/'check-time-user.py').read_bytes());test.chmod(0o644)
cmd=['systemd-run','--unit=cura-pilot-time-check-vuh7p781','--wait','--pipe','--collect','-p','User=cura-receiver','-p','Group=cura-receiver','-p','SupplementaryGroups=gpio spi','-p','CapabilityBoundingSet=CAP_SYS_TIME','-p','AmbientCapabilities=','-p','NoNewPrivileges=no','-p','ProtectSystem=strict','-p','ProtectHome=yes','-p','ReadWritePaths=/var/lib/cura-pilot-vuh7p781/data -/run/chrony','-p','UMask=0077','-p','Environment=PYTHONPATH='+str(code/'receiver')+':'+str(code/'protocol/protocol-v2-lora/python'),str(code/'venv/bin/python'),str(test)]
import pwd,os
sentinel=Path('/run/chrony/cura-pilot-protected-sentinel');sentinel.touch(exist_ok=False);os.chown(sentinel,pwd.getpwnam('_chrony').pw_uid,os.stat('/run/chrony').st_gid)
try:r=subprocess.run(cmd,capture_output=True,text=True)
finally:
 sentinel.unlink(missing_ok=True)
 Path('/run/chrony/cura-pilot-renamed-sentinel').unlink(missing_ok=True)
(stage/'TIME_USER_CHECK.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr)
raise SystemExit(r.returncode)

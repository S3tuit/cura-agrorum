import os,json,subprocess
from pathlib import Path
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_ds3231 import LinuxDs3231Control
from cura_receiver.platform.linux_chrony import LinuxChronyControl
from cura_receiver.platform.linux_kernel_clock import LinuxKernelClock
clock=LinuxOsClock()
env=dict(line.split('=',1) for line in Path('/var/lib/cura-pilot-vuh7p781/config/deployment.env').read_text().splitlines())
rtc=LinuxDs3231Control(clock,kernel_operation_bound_us=3000000,helper_sha256=bytes.fromhex(env['RTC_HELPER_SHA256']),receiver_gid=os.getegid())
r=rtc.read_time(deadline_monotonic_us=clock.now_monotonic_us()+5000000)
assert r.status.name=='OK',r
chrony=LinuxChronyControl(clock,socket_path='/run/chrony/chronyd.sock',deadline_monotonic_us=clock.now_monotonic_us()+1000000)
t=chrony.read_tracking(deadline_monotonic_us=clock.now_monotonic_us()+1000000)
assert t.status.name=='OK',t
k=LinuxKernelClock(clock).sample(deadline_monotonic_us=clock.now_monotonic_us()+250000)
assert k.status.name=='OK',k
status=dict(line.split(':',1) for line in Path('/proc/self/status').read_text().splitlines() if ':' in line)
for p in ('/dev/spidev0.0','/dev/gpiochip0','/dev/rtc-ds3231'):
 assert os.access(p,os.R_OK|os.W_OK),p
assert not os.access('/opt/cura-pilot-vuh7p781/receiver/cura_receiver/__main__.py',os.W_OK)
assert not os.access('/usr/libexec/cura-agrorum/ds3231-set',os.W_OK)
assert not os.access('/var/lib/cura-pilot-vuh7p781/config',os.W_OK)
# Sticky directory permits own reply cleanup but protects daemon-owned entries.
before=set(Path('/run/chrony').glob('chronyc.*.sock'))
for _ in range(2):
 assert chrony.read_tracking(deadline_monotonic_us=clock.now_monotonic_us()+1000000).status.name=='OK'
assert set(Path('/run/chrony').glob('chronyc.*.sock'))==before
sentinel=Path('/run/chrony/cura-pilot-protected-sentinel')
for action in (lambda:sentinel.unlink(),lambda:sentinel.rename('/run/chrony/cura-pilot-renamed-sentinel')):
 try:action()
 except PermissionError:pass
 else:raise AssertionError('daemon-owned entry removal permitted')
for location in ('/etc/cura-pilot-write-test','/opt/cura-pilot-vuh7p781/write-test','/run/cura-pilot-write-test'):
 try:Path(location).write_text('test')
 except OSError:pass
 else:raise AssertionError('unexpected write permitted')
assert Path('/run/chrony').stat().st_mode & 0o7777 == 0o1770
assert Path('/run/chrony/chronyd.sock').stat().st_mode & 0o7777 == 0o660
report={'reply_cleanup':'PASS','daemon_entry_protection':'PASS','sandbox_write_rejection':'PASS','uid':os.getuid(),'gid':os.getgid(),'groups':os.getgroups(),'rtc_status':r.status.name,'chrony_status':t.status.name,'kernel_status':k.status.name,'helper_validation':'PASS','helper_sha256':env['RTC_HELPER_SHA256'],'capabilities':{n:status[n].strip() for n in ('CapInh','CapPrm','CapEff','CapAmb','CapBnd','NoNewPrivs')},'device_access':'PASS','trusted_path_write_rejection':'PASS'}
print(json.dumps(report,indent=2))
Path('/var/lib/cura-pilot-vuh7p781/data/TIME_USER_VERIFICATION.json').write_text(json.dumps(report,indent=2)+'\n')

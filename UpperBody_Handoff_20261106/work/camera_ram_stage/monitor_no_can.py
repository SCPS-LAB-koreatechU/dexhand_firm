from pathlib import Path
import os,time,json,subprocess,re
BASE=Path('/run/user/1000/bhl_hand_camera_session')
PATTERN=re.compile(r'I/O error|tuning execution failed|All sample phases|EXT4-fs error|Remounting filesystem read-only|Out of memory|Killed process',re.I)

def kernel_errors():
    r=subprocess.run(['dmesg','--color=never'],capture_output=True,text=True,timeout=4)
    return [line for line in r.stdout.splitlines() if PATTERN.search(line)]

def health():
    d=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),uptime=float(Path('/proc/uptime').read_text().split()[0]),root=[s for s in Path('/proc/mounts').read_text().splitlines() if s.split()[1]=='/'],can_statistics={},temperature_c={})
    for net in Path('/sys/class/net').glob('can*'):
        d['can_statistics'][net.name]={k:int((net/'statistics'/k).read_text()) for k in ('tx_packets','rx_packets')}
    for p in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:d['temperature_c'][p.parent.name]=int(p.read_text())/1000
        except OSError:pass
    return d

initial=health();before=kernel_errors();print(json.dumps(dict(event='before',health=initial,kernel_errors=before)),flush=True)
start=time.monotonic();last={}
while time.monotonic()-start<100:
    try:last=json.loads((BASE/'no_can_status.json').read_text())
    except (OSError,ValueError):last={}
    print(json.dumps(dict(event='sample',health=health(),gui=last)),flush=True)
    if last and (last.get('callback_errors') or last.get('blocked_can_socket_attempts')):break
    if last and last.get('elapsed_s',0)>=75:break
    if last and not (Path('/proc')/str(last['pid'])).exists():break
    time.sleep(2)
print(json.dumps(dict(event='after',health=health(),kernel_errors=kernel_errors(),last_gui=last)),flush=True)

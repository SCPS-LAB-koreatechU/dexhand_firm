"""Read-only SSH inspection; no CAN sockets, link resets, processes or firmware changes."""
from pathlib import Path
import argparse,subprocess,json,datetime
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--host',default='sunrise@172.18.49.21');parser.add_argument('--out',type=Path);args=parser.parse_args()
remote=r'''
from pathlib import Path
import json,subprocess,time
b=Path('/run/user/1000/bhl_hand_camera_session')
d={'utc':time.time(),'base_exists':b.exists(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
for n in ('status.json','data/index_trial_state.json','data/tracking_preview.json'):
 p=b/n
 if p.exists():
  try:d[n]=json.loads(p.read_text())
  except (ValueError,OSError) as error:d[n]={'read_error':str(error)}
r=subprocess.run(['ip','-j','-details','-statistics','link','show'],capture_output=True,text=True)
d['can_interfaces']=[x for x in json.loads(r.stdout) if x.get('link_type')=='can'] if r.returncode==0 else {'error':r.stderr}
print(json.dumps(d,ensure_ascii=False,indent=2))
'''
result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',args.host,'python3','-'],input=remote,capture_output=True,text=True)
if result.returncode:
 print(result.stderr.strip());raise SystemExit('SSH inspection failed. Set up SSH access manually first. No robot state was changed.')
d=json.loads(result.stdout);text=json.dumps(d,ensure_ascii=False,indent=2)+'\n';print(text)
if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(text,encoding='utf-8')

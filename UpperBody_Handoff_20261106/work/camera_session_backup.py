"""Session-bound data recorder, exits when this camera GUI closes."""
import base64,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
DEST=ROOT/'outputs/hand_camera_session_20261105'
REMOTE='/run/user/1000/bhl_hand_camera_session/export_session.py'

def main():
    DEST.mkdir(parents=True,exist_ok=True);attempts=0
    while attempts<60:
        state={}
        for p in DEST.rglob('*'):
            if p.is_file() and p.name!='backup_status.json':
                stat=p.stat();state[str(p.relative_to(DEST))]=dict(size=stat.st_size,mtime_ns=0)
        process=subprocess.Popen(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=4','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3','sunrise@172.18.49.21','python3',REMOTE],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        try:
            process.stdin.write(json.dumps(state)+'\n');process.stdin.flush()
            for line in process.stdout:
                packet=json.loads(line)
                for item in packet['files']:
                    relative=Path(item['name'])
                    if relative.is_absolute() or '..' in relative.parts:raise ValueError('Unsafe session export path')
                    path=DEST/relative;path.parent.mkdir(parents=True,exist_ok=True);chunk=base64.b64decode(item['data'])
                    if item['append']:
                        with path.open('r+b' if path.exists() else 'w+b') as target:
                            target.seek(item['offset']);target.write(chunk);target.truncate();target.flush();os.fsync(target.fileno())
                    else:
                        temporary=path.with_name(path.name+'.tmp');temporary.write_bytes(chunk);temporary.replace(path)
                ack=dict(utc=time.time(),pc_backup_dir=str(DEST),session_done=packet['done'])
                temporary=DEST/'backup_status.tmp';temporary.write_text(json.dumps(ack));temporary.replace(DEST/'backup_status.json')
                process.stdin.write(json.dumps(ack)+'\n');process.stdin.flush()
                if packet['done']:print('Session closed; final backup saved',flush=True);return
            print('SSH recorder disconnected; retrying',flush=True)
        except (OSError,ValueError) as error:print('Backup error:',error,flush=True)
        finally:
            try:process.stdin.close()
            except OSError:pass
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=4)
                except subprocess.TimeoutExpired:process.kill();process.wait()
        attempts+=1;time.sleep(3)
    print('Backup stopped after repeated connection failures',flush=True)

if __name__=='__main__':main()

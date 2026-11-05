"""Stream RAM session data to the controlling PC; never reads/writes SD files."""
import base64,json,os,select,sys,time
from pathlib import Path
BASE=Path('/run/user/1000/bhl_hand_camera_session')

def main():
    state=json.loads(sys.stdin.readline() or '{}')
    while True:
        started=time.monotonic();items=[]
        paths=[p for p in (BASE/'data').rglob('*') if p.is_file() and p.suffix in ('.json','.jsonl')]
        paths += [BASE/'status.json',BASE/'gui.log']
        for path in paths:
            if not path.exists():continue
            name=str(path.relative_to(BASE));stat=path.stat();previous=state.get(name,{})
            append=path.suffix=='.jsonl' or path.name=='gui.log'
            offset=int(previous.get('size',0)) if append else 0
            if offset>stat.st_size:offset=0
            if append and offset==stat.st_size:continue
            if not append and previous.get('mtime_ns')==stat.st_mtime_ns:continue
            with path.open('rb') as source:
                source.seek(offset);chunk=source.read(262144 if append else -1)
            if not chunk:continue
            items.append(dict(name=name,append=append,offset=offset,data=base64.b64encode(chunk).decode()))
            state[name]=dict(size=offset+len(chunk),mtime_ns=stat.st_mtime_ns)
        try:
            pid=int((BASE/'gui.pid').read_text());cmdline=(Path('/proc')/str(pid)/'cmdline').read_bytes()
            alive=str(BASE/'launch_session.py').encode() in cmdline
        except (OSError,ValueError):alive=False
        print(json.dumps(dict(utc=time.time(),files=items,done=not alive)),flush=True)
        ready,_,_=select.select([sys.stdin],[],[],5)
        if not ready:continue
        raw=sys.stdin.readline()
        if not raw:break
        ack=json.loads(raw)
        tmp=BASE/'backup_ack.tmp';tmp.write_text(json.dumps(ack));tmp.replace(BASE/'backup_ack.json')
        if not alive:break
        time.sleep(max(0.,2-(time.monotonic()-started)))

if __name__=='__main__':main()

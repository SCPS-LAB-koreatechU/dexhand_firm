"""Nonblocking UI/worker event trace; no motor or CAN access."""
from datetime import datetime,timezone,timedelta
from pathlib import Path
import json,queue,threading,time
from .data_paths import data_path

class UiTrace:
    def __init__(self):
        folder=data_path('logs')
        self.path=folder/(datetime.now(timezone(timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')
        self.q=queue.Queue(maxsize=2000);self.dropped=0;self.error='';self.closed=False
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def submit(self,event):
        if self.closed or self.error:self.dropped+=1;return
        entry={'utc':datetime.now(timezone.utc).isoformat(),'monotonic_s':time.monotonic(),**event}
        try:self.q.put_nowait(entry)
        except queue.Full:self.dropped+=1
    def run(self):
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            f=self.path.open('a',encoding='utf-8')
        except OSError as exc:
            self.error=str(exc);return
        with f:
            while True:
                entry=self.q.get()
                try:
                    if entry is None:return
                    f.write(json.dumps(entry,ensure_ascii=False,allow_nan=False)+'\n');f.flush()
                except Exception as exc:self.error=str(exc)
                finally:self.q.task_done()
    def close(self):
        self.closed=True
        if not self.thread.is_alive():return
        try:self.q.put_nowait(None)
        except queue.Full:
            try:self.q.get_nowait();self.q.task_done();self.dropped+=1
            except queue.Empty:pass
            self.q.put_nowait(None)
        self.thread.join(timeout=2)

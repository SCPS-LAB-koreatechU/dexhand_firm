"""Minimal hand UI. Explicit Open starts output; all CAN I/O stays in GuiWorker."""
import argparse
import json
import queue
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox
from .gui_worker import GuiWorker
from .config import load_config
from .ui_trace import UiTrace


class SimpleHandGui:
    def __init__(self, root, config, project, hand='right', channel='can1', dry_run=False):
        self.root=root;self.config_path=Path(config);self.config=load_config(config);self.project=Path(project)
        self.side=hand;self.channel=channel;self.dry_run=dry_run
        self.channel_var=tk.StringVar(value=channel)
        self.identity_text=tk.StringVar(value=f'{channel} · ID {self.config.hand(hand).node}'+(' · 모의 실행' if dry_run else ' · 실물'))
        self.profile=json.loads((self.project/f'dexhand_control/profiles/{hand}_hand.json').read_text())
        self.motors=self.profile['motors']
        self.pending_resume=False
        self.connected=False;self.active=False;self.pending_open=False;self.arming=False;self.open_request_id=0
        self.refreshing=False;self.closing=False;self.error='';self.last_status=None
        self.dragging=set();self.dirty_until={};self.values=[];self.percent=[];self.targets=[];self.sent=[];self.applied=[];self.sliders=[]
        self.trace=UiTrace()
        self.worker=GuiWorker(config)
        self.connecting=False
        self.can_text=tk.StringVar(value='CAN: 미연결 · 연결 확인은 모터를 시작하지 않습니다.')
        self.diagnostic_text=tk.StringVar(value='마지막 응답 / 전송 통계: 대기')
        root.title('손 간단 제어 · 오른손 수정값');root.geometry('1320x950');root.minsize(1200,900)
        style=ttk.Style(root);style.theme_use('clam')
        style.configure('.',font=('Noto Sans CJK KR',11),foreground='#101828',background='#f8fafc')
        style.configure('TButton',padding=(14,9));style.configure('Open.TButton',font=('Noto Sans CJK KR',13,'bold'))
        outer=ttk.Frame(root,padding=18);outer.pack(fill='both',expand=True)
        bar=ttk.Frame(outer);bar.pack(fill='x')
        ttk.Label(bar,text=('오른손' if hand=='right' else '왼손')+' · 손가락 제어',font=('Noto Sans CJK KR',20,'bold')).pack(side='left')
        ttk.Label(bar,textvariable=self.identity_text).pack(side='right')
        connection=ttk.Frame(outer);connection.pack(fill='x',pady=(8,0))
        ttk.Label(connection,text='CAN').pack(side='left',padx=(0,4))
        self.channel_box=ttk.Combobox(connection,textvariable=self.channel_var,width=7,state='readonly',values=self.available_channels(),postcommand=lambda:self.channel_box.configure(values=self.available_channels()));self.channel_box.pack(side='left',padx=(0,7))
        self.can_button=ttk.Button(connection,text='CAN 연결 확인',command=self.check_can);self.can_button.pack(side='left')
        self.reconnect_button=ttk.Button(connection,text='CAN 재연결',command=self.reconnect);self.reconnect_button.pack(side='left',padx=6)
        self.disconnect_button=ttk.Button(connection,text='연결 해제',command=self.disconnect);self.disconnect_button.pack(side='left',padx=8)
        ttk.Label(connection,textvariable=self.can_text,wraplength=670).pack(side='left',padx=10)
        actions=ttk.Frame(outer);actions.pack(fill='x',pady=12)
        self.open_button=ttk.Button(actions,text='손 펼치기',style='Open.TButton',command=self.open_hand);self.open_button.pack(side='left')
        self.resume_button=ttk.Button(actions,text='현재 자세 이어받기',command=self.resume_current);self.resume_button.pack(side='left',padx=7)
        ttk.Button(actions,text='멈춤 · HOLD (Esc)',command=self.hold).pack(side='left',padx=10)
        ttk.Label(outer,text='검지 위 GPIO32 / 아래 GPIO25 · 재연결은 출력 시작과 분리됩니다. 이어받기는 보드의 마지막 PWM을 읽어 사용합니다.',foreground='#475467').pack(anchor='w',pady=(0,6))
        self.status=tk.StringVar(value='먼저 CAN 연결 확인을 누르세요. 연결 확인만으로는 손가락을 움직이지 않습니다.')
        self.status_label=ttk.Label(outer,textvariable=self.status,wraplength=1110,font=('Noto Sans CJK KR',11,'bold'));self.status_label.pack(fill='x',pady=(0,10))
        grid=ttk.Frame(outer);grid.pack(fill='both',expand=True);grid.columnconfigure(1,weight=1)
        for i,text in enumerate(('손가락 / GPIO','당김 슬라이더','목표 %','목표 PWM µs','전송 명령 µs','마지막 확인 µs')):
            ttk.Label(grid,text=text,font=('Noto Sans CJK KR',10,'bold')).grid(row=0,column=i,padx=6,pady=8,sticky='w')
        for i,m in enumerate(self.motors):
            ttk.Label(grid,text=f"{i+1:02} {m['name']} / {m['gpio']}",width=22).grid(row=i+1,column=0,sticky='w',pady=2)
            v=tk.DoubleVar(value=0);percent=tk.StringVar(value='0%');target=tk.StringVar(value=str(m['open_us']));sent=tk.StringVar(value='—');applied=tk.StringVar(value='—')
            slider=ttk.Scale(grid,from_=0,to=100,variable=v,command=lambda value,index=i:self.move(index,float(value)))
            slider.grid(row=i+1,column=1,sticky='ew',padx=10,pady=2)
            slider.bind('<ButtonPress-1>',lambda event,index=i:self.dragging.add(index),add='+')
            slider.bind('<ButtonRelease-1>',lambda event,index=i:self.dragging.discard(index),add='+')
            for col,var in enumerate((percent,target,sent,applied),2):ttk.Label(grid,textvariable=var,width=12).grid(row=i+1,column=col,padx=5)
            self.values.append(v);self.percent.append(percent);self.targets.append(target);self.sent.append(sent);self.applied.append(applied);self.sliders.append(slider)
        self.delivery=tk.StringVar(value='전송 대기 · 실제 손가락 위치를 측정하는 화면은 아닙니다.')
        ttk.Label(outer,textvariable=self.delivery,wraplength=1110).pack(fill='x',pady=(12,4))
        ttk.Label(outer,textvariable=self.diagnostic_text,wraplength=1180,foreground='#475467').pack(fill='x',pady=(4,3))
        ttk.Label(outer,text='목표=슬라이더 설정 / 전송 명령=속도 제한을 거친 PWM / 확인 PWM=ESP32 응답으로 확인한 마지막 명령\n전송 주기 최대 50Hz · 값은 µs(펄스폭), 각도가 아닙니다. 멈춤은 마지막 출력을 유지합니다.',foreground='#475467',font=('Noto Sans CJK KR',9)).pack(fill='x')
        ttk.Label(outer,text='로그: '+str(self.trace.path),foreground='#475467',font=('Noto Sans CJK KR',8)).pack(fill='x',pady=(3,0))
        self.update_controls();root.bind('<Escape>',lambda event:self.hold());root.protocol('WM_DELETE_WINDOW',self.close);root.after(30,self.pump)

    def available_channels(self):
        channels=sorted(p.name for p in Path('/sys/class/net').glob('can*'))
        return tuple(channels) or (self.channel,)

    def choose_channel(self):
        self.channel=self.channel_var.get().strip()
        self.identity_text.set(f'{self.channel} · ID {self.config.hand(self.side).node}'+(' · 모의 실행' if self.dry_run else ' · 실물'))

    def update_controls(self):
        for s in self.sliders:s.configure(state='normal' if self.active and not self.error and not self.pending_open and not self.pending_resume else 'disabled')
        self.open_button.configure(state='normal' if self.connected and not self.connecting and not self.pending_open and not self.pending_resume else 'disabled')
        self.can_button.configure(state='disabled' if self.connecting or self.pending_open or self.pending_resume or self.active else 'normal')
        self.disconnect_button.configure(state='normal' if (self.connected or self.error) and not self.pending_open and not self.pending_resume else 'disabled')
        self.reconnect_button.configure(state='disabled' if self.active or self.connecting or self.pending_open or self.pending_resume else 'normal')
        self.resume_button.configure(state='normal' if self.connected and not self.active and not self.error and not self.pending_resume and not self.pending_open else 'disabled')
        self.channel_box.configure(state='disabled' if self.connected or self.active or self.connecting or self.pending_open or self.pending_resume else 'readonly')

    def check_can(self):
        if self.active or self.pending_open or self.pending_resume or self.connecting:return
        self.choose_channel()
        self.error='';self.connecting=True
        self.status.set('통신만 확인 중 · PWM 출력 시작 명령은 보내지 않습니다.')
        self.trace.submit({'ui_action':'check_can','already_connected':self.connected})
        if self.connected:self.worker.post('check')
        else:self.worker.post('connect',hand=self.side,channel=self.channel,dry_run=self.dry_run)
        self.update_controls()

    def disconnect(self):
        self.pending_open=False;self.pending_resume=False;self.active=False;self.last_status=None
        self.dragging.clear();self.dirty_until.clear()
        for v in self.sent+self.applied:v.set("—")
        self.worker.post('disconnect');self.update_controls()
        self.trace.submit({'ui_action':'disconnect'})

    def reconnect(self):
        if self.active or self.pending_open or self.pending_resume or self.connecting:return
        self.choose_channel()
        self.connected=False;self.active=False;self.error='';self.last_status=None
        self.dragging.clear();self.dirty_until.clear()
        for v in self.sent+self.applied:v.set('—')
        self.connecting=True;self.status.set('기존 소켓을 정리하고 CAN을 다시 확인합니다. 자동 출력은 하지 않습니다.')
        self.trace.submit({'ui_action':'reconnect'})
        self.worker.post('connect',hand=self.side,channel=self.channel,dry_run=self.dry_run);self.update_controls()

    def resume_current(self):
        if not self.connected or self.error:
            self.status.set('CAN 연결 확인 또는 재연결을 먼저 실행하세요. 출력은 시작하지 않았습니다.');return
        if self.active or self.pending_open or self.pending_resume:return
        self.pending_resume=True;self.arming=False;self.open_request_id+=1
        self.status.set('보드의 마지막 PWM을 읽어 같은 자세로 이어받는 중…')
        self.trace.submit({'ui_action':'resume_current'})
        self.worker.post('arm',initial_open=False,request_id='resume-'+str(self.open_request_id));self.update_controls()

    def open_hand(self):
        if not self.connected:
            self.status.set('먼저 CAN 연결 확인을 누르세요. 손 출력은 시작하지 않았습니다.')
            return
        if self.pending_open or self.pending_resume:return
        self.trace.submit({'ui_action':'open_hand'})
        self.error='';self.pending_open=True;self.arming=False;self.open_request_id+=1
        self.status.set('손 펼침 출력 준비 중…');self.update_controls()
        self.start_output()

    def start_output(self):
        self.arming=True;self.status.set('펼침 출력 준비 중…')
        self.worker.post('arm',initial_open=True,request_id='open-'+str(self.open_request_id))

    def send_open_goal(self):
        self.arming=False
        self.refreshing=True
        for i,m in enumerate(self.motors):
            self.values[i].set(0);self.percent[i].set('0%');self.targets[i].set(str(m['open_us']));self.dirty_until[i]=time.monotonic()+.3
        self.refreshing=False
        self.worker.post('open_all',request_id=self.open_request_id);self.update_controls()

    def move(self,index,value):
        if self.refreshing or not self.active or self.error or self.pending_open or self.pending_resume:return
        m=self.motors[index];pulse=round(m['open_us']+value/100*(m['close_us']-m['open_us']))
        self.percent[index].set(f'{value:.0f}%');self.targets[index].set(str(pulse));self.dirty_until[index]=time.monotonic()+.3
        self.trace.submit({'ui_action':'slider','index':index,'percent':value,'target_us':pulse})
        self.worker.move(index,pulse)

    def hold(self):
        self.trace.submit({'ui_action':'hold'})
        self.pending_open=False;self.pending_resume=False;self.arming=False;self.active=False;self.dragging.clear();self.dirty_until.clear();self.worker.post('hold');self.update_controls()
        self.status.set('멈춤 요청 · 마지막 출력 유지 응답을 확인 중입니다.')

    def pump(self):
        self.worker.ui_seen=time.monotonic()
        for _ in range(64):
            try:event=self.worker.events.get_nowait()
            except queue.Empty:break
            self.trace.submit({'worker_event':event})
            kind=event['kind']
            if kind in ('connected','checked'):
                self.connected=True;self.connecting=False;self.error=''
                self.can_text.set(('모의 연결' if self.dry_run else 'CAN 응답 확인')+f": ID {event['node']} · 프로필 {event['tag']}")
                self.status.set('연결 확인 완료 · 손 출력은 시작하지 않았습니다. 펼치려면 손 펼치기를 누르세요.')
            elif kind=='armed':
                if self.pending_resume and event.get('request_id')=='resume-'+str(self.open_request_id):
                    self.pending_resume=False;self.active=True;self.status.set('보드의 마지막 PWM으로 이어받았습니다. 이전 목표는 재전송하지 않았습니다.')
                elif self.pending_open and event.get('request_id')=='open-'+str(self.open_request_id):
                    self.active=True;self.send_open_goal()
            elif kind=='warning':
                self.diagnostic_text.set(event['message'])
            elif kind=='open_goal_set':
                if event.get('request_id')==self.open_request_id:self.pending_open=False
            elif kind=='disconnected':
                self.connected=False;self.active=False;self.connecting=False;self.pending_resume=False;self.last_status=None
                self.can_text.set('CAN: 연결 해제')
                self.status.set('연결 해제됨 · 다시 CAN 연결 확인을 누르세요.')
            elif kind=='error':
                self.error=event.get('stage','unknown')+' · '+event['message'];self.active=False;self.pending_open=False;self.pending_resume=False;self.arming=False;self.connecting=False;self.connected=False
                self.can_text.set('통신/출력 확인 필요 · 오류 로그 확인 후 CAN 재연결')
            elif kind=='status':
                self.last_status=event;self.active=event['active']
                self.refreshing=True
                for i,pulse in enumerate(event['goal_us']):
                    self.sent[i].set(str(event['command_us'][i]) if event['command_known'] else '—')
                    self.applied[i].set(str(event['applied_us'][i]) if event['applied_us'] is not None else '—')
                    if i not in self.dragging and time.monotonic()>self.dirty_until.get(i,0):
                        m=self.motors[i];pct=max(0.,min(100.,100*(pulse-m['open_us'])/(m['close_us']-m['open_us'])))
                        self.values[i].set(pct);self.percent[i].set(f'{pct:.0f}%');self.targets[i].set(str(pulse))
                self.refreshing=False
                diag=event.get('diagnostics',{});age=diag.get('last_response_age_s');stats=diag.get('stats',{})
                self.diagnostic_text.set('마지막 응답 '+('미확인' if age is None else f'{age:.2f}초 전')+' | 단계 '+str(diag.get('stage','—'))+f" | 전송 {stats.get('sets_sent',0)} · 적용 ACK {stats.get('applied',0)} · 미응답 {stats.get('no_reply',0)} · 지연 ACK {stats.get('late',0)} | 응답 대기 {diag.get('reply_timeout_s',0)*1000:.0f}ms")
                d=event['delivery'];self.delivery.set(f"{'모의 응답' if event['dry_run'] else 'CAN 응답'}: {d['result']}  |  {'미확인' if d['latency_ms']<0 else str(round(d['latency_ms'],1))+' ms'}  |  실제 손가락 위치는 미측정")
                if not self.error and not self.pending_open and not self.pending_resume:self.status.set(('모의 실행 · ' if self.dry_run else '')+('조절 가능 · 슬라이더를 움직이면 값이 실시간 전송됩니다.' if self.active else '출력 대기 · 현재 자세 이어받기 또는 손 펼치기를 선택하세요.'))
            elif kind=='closed':
                if self.worker.shutdown_error:messagebox.showwarning('HOLD 응답 미확인',self.worker.shutdown_error,parent=self.root)
                self.trace.close();self.root.destroy();return
        if self.trace.error or self.trace.dropped:
            self.diagnostic_text.set('로그 저장 오류: '+self.trace.error+f' / 누락 {self.trace.dropped}')
        if self.error:
            self.active=False
            self.status.set('응답 오류 · '+self.error+' · 실제 적용 상태 미확인. 슬라이더 잠김.')
        self.status_label.configure(foreground='#9b1c1c' if self.error else '#101828');self.update_controls();self.root.after(30,self.pump)

    def close(self):
        if not self.closing:
            self.closing=True
            self.worker.close()


def main():
    from .gui import project_default
    p=argparse.ArgumentParser();p.add_argument('--project',default=str(project_default()));p.add_argument('--config');p.add_argument('--hand',choices=('left','right'),default='right');p.add_argument('--channel',default='can1');p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    config=a.config or str(Path(a.project)/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand.yaml')
    root=tk.Tk();SimpleHandGui(root,config,a.project,a.hand,a.channel,a.dry_run);root.mainloop()

if __name__=='__main__':main()

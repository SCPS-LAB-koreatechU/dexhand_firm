"""Full-hand camera correspondence editor. Preview is the default; CAN needs --live."""
import argparse,collections,copy,json,math,multiprocessing as mp,os,queue,statistics,time
from pathlib import Path
import tkinter as tk
from tkinter import ttk,filedialog,font as tkfont
from .config import load_config
from .gui_worker import GuiWorker
from .ui_trace import UiTrace
from .data_paths import data_path
from .camera_source import camera_worker
from .hand_retarget import CorrespondenceProfile,GROUPS,LABELS,JOINTS,extract_features,estimate_joints


def synthetic_hand(curl=0.,splay=0.,thumb_across=0.,thumb_fold=0.):
    p=[[0.,0.,0.] for _ in range(21)]
    for m,x,y in ((5,.04,.06),(9,.012,.07),(13,-.018,.065),(17,-.045,.052)):
        p[m]=[x,y,0.]
        for k,L in enumerate((.035,.025,.018),1):
            theta=curl*k*.48;v=[math.sin(splay)*math.cos(theta)*L,math.cos(splay)*math.cos(theta)*L,-math.sin(theta)*L]
            p[m+k]=[a+b for a,b in zip(p[m+k-1],v)]
    p[1]=[.037,.012,0.];p[2]=[.057,.025,0.]
    for k,L in ((3,.027),(4,.022)):
        theta=thumb_fold*(k-1)*.6;a=.7-thumb_across*1.1
        p[k]=[p[k-1][0]+math.sin(a)*math.cos(theta)*L,p[k-1][1]+math.cos(a)*math.cos(theta)*L,p[k-1][2]-math.sin(theta)*L]
    return p


class TrackingGui:
    def __init__(self,root,project,config,*,live=False,source_hand='Right',demo=False,max_frame_age=.25):
        if not 0 < max_frame_age <= .35:raise ValueError('영상 유효시간은 0~350ms 범위여야 합니다')
        self.max_frame_age=max_frame_age
        if live and demo:raise ValueError('데모 영상은 실물 출력과 함께 사용할 수 없습니다')
        self.root=root;self.project=Path(project).resolve();self.config_path=Path(config);self.config=load_config(config);self.spec=self.config.hand('right')
        self.demo_curl=tk.DoubleVar(value=0);self.demo_across=tk.DoubleVar(value=0);self.demo_fold=tk.DoubleVar(value=0)
        self.live=live;self.demo=demo;self.source_hand=tk.StringVar(value=source_hand);self.model=CorrespondenceProfile(self.spec,source_hand)
        self.worker=GuiWorker(config);self.trace=UiTrace();self.connected=False;self.active=False;self.connecting=False;self.pending=None;self.request=0;self.tracking=False;self.last_status=None;self.error='';self.closing=False
        self.camera_problem='';self.camera=None;self.frames=None;self.images=None;self.photo=None;self.latest=None;self.features=None;self.filtered=None;self.last_frame_time=0.;self.stable=collections.deque(maxlen=60);self.prediction=None;self.target=[0.]*15;self.refreshing=False;self.dragging=set();self.dirty_until=0.
        labels={'전체':'all',**{v:k for k,v in LABELS.items()}}
        self.scope_label=tk.StringVar(value='전체');self.group_label=tk.StringVar(value='전체');self.link_label=tk.StringVar(value='검지')
        self.scope_label.trace_add('write',lambda *a:self.scope.set(labels.get(self.scope_label.get(),'all')))
        self.group_label.trace_add('write',lambda *a:self.group.set(labels.get(self.group_label.get(),'all')))
        self.link_label.trace_add('write',lambda *a:self.link_group.set(labels.get(self.link_label.get(),'index')))
        self.channel=tk.StringVar(value=self.spec.channel);self.camera_device=tk.StringVar(value='0');self.scope=tk.StringVar(value='all');self.group=tk.StringVar(value='all');self.pose_name=tk.StringVar(value='펴기');self.confirmed=tk.BooleanVar(value=False);self.time_verified=tk.BooleanVar(value=False);self.times=tk.StringVar(value='.5, .5, .5')
        self.note=tk.StringVar(value='1 카메라 미리보기 → 2 로봇 자세를 수동 조절 → 3 같은 사람 자세를 유지하고 등록. 기본은 모의 출력입니다.')
        self.camera_note=tk.StringVar(value='카메라를 시작하세요');self.connection=tk.StringVar(value='CAN 미접속');self.delivery=tk.StringVar(value='보드 수신/실제 위치는 측정하지 않습니다');self.counts=tk.StringVar(value='대응 자세 없음');self.values=[];self.pulses=[];self.sent=[];self.sliders=[]
        self.preview_file=data_path('preview');self.calibration_folder=data_path('calibration');self.last_preview_write=0.;self.frames_received=0;self.previews_received=0;self.storage_error='';self.last_camera_trace=0.
        self._build();root.protocol('WM_DELETE_WINDOW',self.close);root.bind('<Escape>',lambda e:self.hold());root.after(30,self.pump)
    def _build(self):
        r=self.root;r.title('손 전체 · 카메라 추종과 자세 등록'+(' · 실물' if self.live else ' · 모의 출력')+(' · 합성 입력 데모' if self.demo else ''));r.geometry('1480x970');r.minsize(1300,900)
        # Fontconfig/desktop scaling can multiply Tk point sizes; fit measured line heights.
        font_cache={}
        def ui_font(height,bold=False):
            key=(height,bold)
            if key in font_cache:return font_cache[key]
            candidates=[]
            for size in range(1,33):
                spec=('Noto Sans CJK KR',size,'bold' if bold else 'normal')
                probe=tkfont.Font(root=r,font=spec)
                candidates.append((abs(probe.metrics('linespace')-height),size))
            font_cache[key]=('Noto Sans CJK KR',min(candidates)[1],'bold' if bold else 'normal')
            return font_cache[key]
        for name in ('TkDefaultFont','TkTextFont','TkMenuFont','TkHeadingFont','TkCaptionFont','TkSmallCaptionFont','TkIconFont','TkTooltipFont'):
            tkfont.nametofont(name).configure(family='Noto Sans CJK KR',size=ui_font(20)[1])
        style=ttk.Style(r);style.theme_use('clam');style.configure('.',font=ui_font(20),foreground='#101828',background='#f8fafc');style.configure('TButton',padding=(10,7));style.configure('Treeview',rowheight=28);style.configure('Treeview.Heading',font=ui_font(20,True))
        outer=ttk.Frame(r,padding=12);outer.pack(fill='both',expand=True)
        ttk.Label(outer,text='손 전체 카메라 추종 · 사용자 손 ↔ 모터 자세 대응',font=ui_font(34,True)).pack(anchor='w')
        ttk.Label(outer,text=('실물 송신 모드 · 새 수신 전용 펌웨어 업로드 후 사용' if self.live else '모의 출력 모드 · CAN을 열거나 모터를 움직이지 않습니다')+(' · 합성 랜드마크 데모, 실제 카메라 아님' if self.demo else '')+' · 기존 GPIO/PWM 끝점 유지 · 보드 확인 없음',wraplength=1400,foreground='#9b1c1c' if self.live else '#175cd3').pack(anchor='w')
        bar=ttk.Frame(outer);bar.pack(fill='x',pady=7)
        ttk.Label(bar,text='CAN').pack(side='left');ttk.Entry(bar,textvariable=self.channel,width=8).pack(side='left',padx=4)
        self.connect_button=ttk.Button(bar,text='CAN 소켓 접속',command=self.connect);self.connect_button.pack(side='left')
        ttk.Button(bar,text='접속 해제',command=self.disconnect).pack(side='left',padx=4)
        self.open_button=ttk.Button(bar,text='명시적 전체 펼침',command=lambda:self.arm(True));self.open_button.pack(side='left',padx=4)
        self.resume_button=ttk.Button(bar,text='마지막 송신값 재개',command=lambda:self.arm(False));self.resume_button.pack(side='left',padx=4)
        ttk.Button(bar,text='HOLD / Esc',command=self.hold).pack(side='left',padx=4)
        ttk.Label(bar,textvariable=self.connection,wraplength=500).pack(side='left',padx=8)
        cameras=ttk.Frame(outer);cameras.pack(fill='x',pady=4)
        ttk.Label(cameras,text='카메라 번호/영상 경로').pack(side='left');ttk.Entry(cameras,textvariable=self.camera_device,width=18).pack(side='left',padx=4)
        ttk.Label(cameras,text='사용자 손').pack(side='left');ttk.Combobox(cameras,textvariable=self.source_hand,values=('Right','Left'),width=8,state='readonly').pack(side='left',padx=4)
        ttk.Button(cameras,text='영상 시작',command=self.start_camera).pack(side='left',padx=4);ttk.Button(cameras,text='영상 종료',command=self.stop_camera).pack(side='left',padx=4)
        ttk.Label(cameras,text='추종 대상').pack(side='left',padx=(12,0));ttk.Combobox(cameras,textvariable=self.scope_label,values=('전체',)+tuple(LABELS.values()),width=9,state='readonly').pack(side='left',padx=4)
        self.track_button=ttk.Button(cameras,text='카메라 추종 시작',command=self.start_tracking);self.track_button.pack(side='left',padx=4)
        ttk.Button(cameras,text='추종 정지/HOLD',command=self.hold).pack(side='left',padx=4)
        if self.demo:
            demo_bar=ttk.Frame(outer);demo_bar.pack(fill='x')
            for title,var in (('데모 손가락 굽힘',self.demo_curl),('엄지 방향',self.demo_across),('엄지 굽힘',self.demo_fold)):
                ttk.Label(demo_bar,text=title).pack(side='left');ttk.Scale(demo_bar,from_=0,to=1,variable=var,length=180).pack(side='left',padx=6)
        body=ttk.Frame(outer);body.pack(fill='both',expand=True);body.columnconfigure(0,weight=3);body.columnconfigure(1,weight=2);body.rowconfigure(0,weight=1)
        tabs=ttk.Notebook(body);tabs.grid(row=0,column=0,sticky='nsew',padx=(0,10));manual=ttk.Frame(tabs,padding=8);cal=ttk.Frame(tabs,padding=10);tabs.add(manual,text='1 실물 자세 수동 조절');tabs.add(cal,text='2 카메라 자세 등록')
        motor_data=json.loads((self.project/'dexhand_control/profiles/right_hand.json').read_text())['motors']
        manual.columnconfigure(1,weight=1)
        for c,title in enumerate(('모터 / GPIO','당김 비율 0~100%','목표 µs','송신 µs')):ttk.Label(manual,text=title,font=ui_font(20,True)).grid(row=0,column=c,padx=4,pady=8,sticky='w')
        for i,m in enumerate(motor_data):
            ttk.Label(manual,text=f"{i+1:02} {m['name']} / {m['gpio']}",width=23).grid(row=i+1,column=0,sticky='w',pady=3)
            v=tk.DoubleVar(value=0);pulse=tk.StringVar(value=str(m['open_us']));sent=tk.StringVar(value='—');s=ttk.Scale(manual,from_=0,to=100,variable=v,command=lambda x,j=i:self.move(j,float(x)/100))
            s.grid(row=i+1,column=1,sticky='ew',padx=6,pady=3);s.bind('<ButtonPress-1>',lambda e,j=i:self.dragging.add(j));s.bind('<ButtonRelease-1>',lambda e,j=i:self.dragging.discard(j))
            ttk.Label(manual,textvariable=pulse,width=7).grid(row=i+1,column=2);ttk.Label(manual,textvariable=sent,width=7).grid(row=i+1,column=3)
            self.values.append(v);self.pulses.append(pulse);self.sent.append(sent);self.sliders.append(s)
        linked=ttk.LabelFrame(manual,text='위·아래 모터 함께 당기기 · TIP은 별도 슬라이더',padding=8);linked.grid(row=16,column=0,columnspan=4,sticky='ew',pady=9);linked.columnconfigure(1,weight=1)
        self.link_group=tk.StringVar(value='index');ttk.Combobox(linked,textvariable=self.link_label,values=('검지','중지','약지','새끼'),width=9,state='readonly').grid(row=0,column=0)
        self.linked=ttk.Scale(linked,from_=0,to=100,command=lambda v:self.move_linked(float(v)/100));self.linked.grid(row=0,column=1,sticky='ew',padx=8)
        ttk.Label(manual,text='각 모터 범위는 기존 확정값 그대로입니다. 모든 명령은 공통 진행률로 보간합니다.\n실제 서보의 도착 시간은 영상으로 비교해야 합니다. 첫 출력은 보드 자세를 읽지 못합니다.',wraplength=750).grid(row=17,column=0,columnspan=4,sticky='w')
        ttk.Label(cal,text='로봇과 사람 손을 같은 자세로 맞춘 다음 등록합니다.\n처음에는 전체 펴기를 등록하세요. 이후 손가락별로 자세를 추가합니다.',wraplength=700,font=ui_font(22,True)).pack(anchor='w',pady=6)
        select=ttk.Frame(cal);select.pack(fill='x',pady=8);ttk.Label(select,text='등록 대상').pack(side='left');ttk.Combobox(select,textvariable=self.group_label,values=('전체',)+tuple(LABELS.values()),width=9,state='readonly').pack(side='left',padx=5)
        ttk.Label(select,text='자세 이름').pack(side='left');self.names=ttk.Combobox(select,textvariable=self.pose_name,width=29,values=('펴기','25% 굽힘','50% 굽힘','75% 굽힘','접기','왼쪽 벌림','오른쪽 벌림','끝마디 접기','검지 접촉-중간','중지 접촉-중간','약지 접촉-중간','새끼 접촉-중간'));self.names.pack(side='left',padx=5)
        ttk.Checkbutton(cal,text='실물 로봇 자세를 눈으로 확인했고 사람 손과 맞췄음',variable=self.confirmed).pack(anchor='w')
        ttk.Label(cal,text='모의 출력에서 등록한 샘플은 실물 확인으로 표시되지 않으며 실물 추종에 사용되지 않습니다.',wraplength=710,foreground='#475467').pack(anchor='w')
        ttk.Button(cal,text='현재 사람 손 + 현재 모터값 대응 등록',command=self.record).pack(anchor='w',pady=8)
        table_frame=ttk.Frame(cal);table_frame.pack(fill='both',expand=True)
        self.table=ttk.Treeview(table_frame,columns=('g','name','pwm','verified'),show='headings',height=6)
        for c,title,w in (('g','손가락',65),('name','자세',150),('pwm','모터 3개 PWM µs',210),('verified','실물 확인',85)):
            self.table.heading(c,text=title);self.table.column(c,width=w)
        self.table.pack(side='left',fill='both',expand=True)
        scroll=ttk.Scrollbar(table_frame,orient='vertical',command=self.table.yview);scroll.pack(side='right',fill='y');self.table.configure(yscrollcommand=scroll.set)
        files=ttk.Frame(cal);files.pack(fill='x',pady=8)
        for title,fn in (('선택 샘플 삭제',self.delete),('대응표 저장',self.save),('대응표 불러오기',self.load),('새 대응표',self.new_profile)):ttk.Button(files,text=title,command=fn).pack(side='left',padx=3)
        timing=ttk.LabelFrame(cal,text='선택 손가락 모터 3개의 전체 범위 이동시간(초)',padding=8);timing.pack(fill='x')
        ttk.Entry(timing,textvariable=self.times,width=22).pack(side='left');ttk.Checkbutton(timing,text='영상으로 확인한 값',variable=self.time_verified).pack(side='left',padx=4);ttk.Button(timing,text='시간값 기록',command=self.set_times).pack(side='left')
        ttk.Label(cal,text='입력 순서: 위/아래/끝 · 엄지는 왼쪽/TIP/오른쪽. 기본 0.5초는 명령 기준 추정입니다. 실제 시간 측정값을 넣으면 느린 모터에 맞춰 함께 이동합니다.',wraplength=710,foreground='#475467').pack(anchor='w',pady=5)
        right=ttk.Frame(body);right.grid(row=0,column=1,sticky='nsew')
        image_frame=tk.Frame(right,height=360,background='#172531');image_frame.pack(fill='x');image_frame.pack_propagate(False)
        self.demo_canvas=tk.Canvas(image_frame,background='#172531',highlightthickness=0) if self.demo else None
        if self.demo:self.demo_canvas.pack(fill='both',expand=True)
        self.image=tk.Label(image_frame,text='영상 미리보기 · 카메라만 켜서는 실물이 움직이지 않습니다',background='#172531',foreground='white');self.image.pack(fill='both',expand=True) if not self.demo else None
        ttk.Label(right,textvariable=self.camera_note,wraplength=510).pack(anchor='w',pady=8)
        ttk.Label(right,textvariable=self.counts,wraplength=510).pack(anchor='w',pady=5)
        self.feature_text=tk.StringVar(value='손 특징값 대기');ttk.Label(right,textvariable=self.feature_text,wraplength=510).pack(anchor='w',pady=6)
        ttk.Label(right,text='미리보기 목표는 등록한 자세의 보간 결과입니다.\nRViz는 별도 실행하며 예상 관절 자세를 표시합니다. 실제 위치 센서값이 아닙니다.',wraplength=510,foreground='#475467').pack(anchor='w',pady=6)
        ttk.Label(right,text='등록 순서: 전체 펴기 → 각 손가락 굽힘 25/50/75/접기 → 좌우 벌림 → 엄지 네 손가락 접촉×굽힘 단계',wraplength=510).pack(anchor='w',pady=10)
        self.support_note=tk.StringVar(value='추종 전에 각 손가락에 서로 다른 자세 3개 이상이 필요합니다. 미등록 영역에서는 HOLD합니다.');ttk.Label(right,textvariable=self.support_note,wraplength=510).pack(anchor='w',pady=6)
        ttk.Label(outer,textvariable=self.note,wraplength=1420,font=ui_font(20,True)).pack(fill='x',pady=6)
        self.storage_note=tk.StringVar(value='자세 등록 시 자동 저장: '+str(self.calibration_folder))
        ttk.Label(outer,textvariable=self.storage_note,wraplength=1420,foreground='#175cd3').pack(fill='x')
        ttk.Label(outer,textvariable=self.delivery,wraplength=1420).pack(fill='x');ttk.Label(outer,text='로그: '+str(self.trace.path),foreground='#475467').pack(anchor='w')
        self.update_controls();self.refresh_samples()
    def connect(self):
        if self.connecting or self.active or self.pending:return
        self.hold();self.error='';self.connecting=True;self.worker.post('connect',hand='right',channel=self.channel.get(),dry_run=not self.live)
        self.note.set('로컬 CAN 소켓 접속 중 · 단방향이므로 보드 응답을 확인하지 않습니다')
    def disconnect(self):
        self.tracking=False;self.active=False;self.pending=None;self.connected=False;self.last_status=None;self.worker.post('disconnect')
    def arm(self,open_hand):
        if not self.connected or self.pending or self.tracking:return
        self.request+=1;self.pending='open' if open_hand else 'resume'
        if open_hand and self.active:self.worker.post('open_all',request_id=self.request)
        else:self.worker.post('arm',initial_open=open_hand,request_id=self.request)
        self.note.set('명시적으로 출력을 준비합니다. 첫 출력은 현재 실물 자세를 읽을 수 없습니다.')
    def hold(self):
        self.tracking=False;self.active=False;self.pending=None;self.worker.post('hold')
        self.note.set('HOLD 요청 · 현재 송신값 유지 요청이며 보드 적용 확인은 없습니다. 자동 재개하지 않습니다.')
    def move(self,index,value):
        if self.refreshing or not self.active or self.pending or self.tracking:return
        proposed=self.target[:];proposed[index]=max(0.,min(1.,value))
        try:self.model.validate_motor_goal(proposed)
        except ValueError as e:
            self.note.set(str(e));self.refreshing=True;self.values[index].set(self.target[index]*100);self.refreshing=False;return
        self.target=proposed;self.dirty_until=time.monotonic()+.3;self.worker.move_pose(proposed,{},time.monotonic(),streaming=False);self.pulses[index].set(str(self.spec.to_us(proposed)[index]))
        self.trace.submit({'ui_action':'manual_motor','index':index,'target':proposed[index]})
    def move_linked(self,value):
        if not self.active or self.pending or self.tracking:return
        indices=GROUPS[self.link_group.get()][:2];proposed=self.target[:]
        for i in indices:proposed[i]=value
        self.target=proposed;self.worker.move_pose(proposed,{},time.monotonic(),streaming=False);self.refreshing=True
        for i in indices:self.values[i].set(100*value);self.pulses[i].set(str(self.spec.to_us(proposed)[i]))
        self.refreshing=False;self.dirty_until=time.monotonic()+.3
    def start_camera(self):
        self.stop_camera();self.camera_problem=''
        if self.demo:self.camera_note.set('데모 입력 · 실물 확인으로 저장되지 않습니다');return
        model=self.project/'dexhand_control/.cache/hand_landmarker.task'
        if not model.is_file():self.note.set('MediaPipe 모델이 없습니다. setup_hand_camera.sh를 실행하세요');return
        context=mp.get_context('spawn');self.frames=context.Queue(maxsize=1);self.images=context.Queue(maxsize=1);stop=context.Event()
        process=context.Process(target=camera_worker,kwargs=dict(output=self.frames,stop=stop,camera=self.camera_device.get(),model=str(model),headless=True,preview=self.images,include_landmarks=True,control_sample_path=os.environ.get('BHL_HAND_CAMERA_SAMPLE_PATH'),observation_path=os.environ.get('BHL_HAND_CAMERA_OBSERVATION_PATH'),input_role=os.environ.get('BHL_HAND_CAMERA_ROLE','human_control')),daemon=True);process.start();self.camera=(process,stop)
        self.camera_note.set('카메라 시작 중')
    def stop_camera(self):
        if self.tracking:self.hold()
        self.latest=self.features=self.filtered=None;self.stable.clear()
        if self.camera:
            process,stop=self.camera;stop.set();process.join(.5)
            if process.is_alive():process.terminate();process.join(.5)
            for q in (self.frames,self.images):q.close();q.cancel_join_thread()
            self.camera=None
    def accept_frame(self,sample):
        now=time.monotonic();self.latest=sample;self.frames_received+=1
        if now-self.last_camera_trace>=.2:
            self.last_camera_trace=now
            self.trace.submit({'camera_frame':sample,'motors_measured':False})
        if (not sample.get('detection_valid') or sample.get('source_hand')!=self.source_hand.get() or sample.get('handedness_score',0)<.65 or not 0<=now-sample['captured_at']<=self.max_frame_age):
            self.feature_text.set('현재 유효한 손 특징값 없음 · 이전 각도값을 현재값으로 사용하지 않습니다')
            self.features=None;self.filtered=None;self.stable.clear();self.camera_note.set('선택한 손 하나를 보여주세요 · 인식/영상 시간 확인 필요')
            if self.tracking:self.hold();return
            return
        try:f=extract_features(sample['landmarks'])
        except ValueError as e:
            self.features=None;self.stable.clear();self.note.set(str(e))
            if self.tracking:self.hold()
            return
        self.features=f;dt=max(0.,min(.1,now-self.last_frame_time));alpha=1-math.exp(-dt/.06) if self.last_frame_time else 1.;self.last_frame_time=now
        if self.filtered is None:self.filtered=copy.deepcopy(f)
        else:self.filtered={g:[a+alpha*(b-a) for a,b in zip(self.filtered[g],f[g])] for g in GROUPS}
        self.stable.append((now,f));self.camera_note.set(f"{sample['source_hand']} · {sample.get('view_mode','full')} 영상 · 영상 지연 {(now-sample['captured_at'])*1000:.0f}ms · 기록 프레임 {len(self.stable)}")
        self.feature_text.set(' / '.join(f'{LABELS[g]} 굽힘 {f[g][0]*90:.0f}°' for g in ('index','middle','ring','pinky'))+' · 관측 특징값, 로봇 관절각 아님')
        try:
            goal,q,support=self.model.predict(self.filtered,self.scope.get(),self.target,verified_only=self.live)
            self.prediction=(goal,q,support);self.support_note.set('등록 자세와의 거리: '+', '.join(f'{LABELS[g]} {v:.3f}' for g,v in support.items()))
            if self.tracking:
                self.worker.move_pose(goal,q,sample['captured_at'],streaming=True)
                self.trace.submit({'tracking_frame':sample['frame_id'],'captured_at':sample['captured_at'],'target_us':self.spec.to_us(goal),'support':support})
            if not self.tracking:self.write_preview(q,False)
        except (ValueError,KeyError) as e:
            self.prediction=None;self.support_note.set(str(e))
            if not self.tracking:self.write_preview(estimate_joints(f,self.model.reference),False,source='camera reference angles; not calibrated motor prediction')
            if self.tracking:self.hold();self.note.set('추종 정지: '+str(e))
    def stable_features(self):
        now=time.monotonic();recent=[(t,f) for t,f in self.stable if 0<=now-t<=1.0];samples=[f for _,f in recent]
        # A time interval works at both desktop and board frame rates.
        if (self.features is None or not self.latest or not 0<=now-self.latest['captured_at']<=self.max_frame_age
                or len(recent)<5 or recent[-1][0]-recent[0][0]<.45
                or any(b[0]-a[0]>.3 for a,b in zip(recent,recent[1:]))):
            raise ValueError('같은 손 자세를 약 0.5초 이상 유지하세요')
        groups=tuple(GROUPS) if self.group.get()=='all' else (self.group.get(),)
        for g in groups:
            for j in range(len(samples[0][g])):
                if statistics.pstdev(f[g][j] for f in samples)>.035:raise ValueError('손이 움직이고 있습니다. 자세를 잠시 고정하세요')
        return {g:[statistics.mean(f[g][j] for f in samples) for j in range(len(samples[0][g]))] for g in GROUPS}
    def record(self):
        try:
            if self.tracking:raise ValueError('추종을 멈추고 수동으로 로봇 자세를 맞추세요')
            if self.source_hand.get()!=self.model.source_hand:raise ValueError('다른 사용자 손을 사용하려면 새 대응표를 만드세요')
            if self.model.reference is None and (self.group.get()!='all' or self.pose_name.get()!='펴기'):raise ValueError('처음에는 전체 / 펴기를 등록하세요')
            status=self.last_status
            if not status or not status['command_known']:raise ValueError('명시적 펼침으로 출력을 시작한 뒤 로봇 자세를 맞추세요')
            if any(abs(a-b)>2 for a,b in zip(status['command_us'],status['goal_us'])):raise ValueError('PC 명령이 아직 이동 중입니다. 목표에 도착한 뒤 등록하세요')
            if self.live and not self.confirmed.get():raise ValueError('실물 자세를 눈으로 확인한 뒤 확인 표시를 선택하세요')
            f=self.stable_features();groups=tuple(GROUPS) if self.group.get()=='all' else (self.group.get(),)
            proposed=copy.deepcopy(self.model)
            for g in groups:proposed.add(g,self.pose_name.get(),f,status['command_us'],verified=self.live and self.confirmed.get())
            self.model=proposed;self.confirmed.set(False);self.refresh_samples();saved=self.autosave();self.note.set('대응 자세를 기록했습니다. 모터 끝값과 펌웨어는 바뀌지 않았습니다.'+(' 자동 저장 완료' if saved else ' · 저장 오류: '+self.storage_error))
            self.trace.submit({'ui_action':'record_correspondence','groups':groups,'name':self.pose_name.get(),'operator_confirmed':self.live,'command_us':status['command_us'],'features':f})
        except (ValueError,KeyError) as e:self.note.set(str(e))
    def refresh_samples(self):
        for row in self.table.get_children():self.table.delete(row)
        for i,a in enumerate(self.model.anchors):
            us=[round(self.spec.open_us[j]+v*(self.spec.close_us[j]-self.spec.open_us[j])) for j,v in zip(GROUPS[a['group']],a['pull'])]
            self.table.insert('', 'end',iid=str(i),values=(LABELS[a['group']],a['name'],str(us),'사용자 확인' if a['operator_confirmed'] else '모의/초안'))
        c=self.model.counts();v=self.model.counts(True);self.counts.set('대응 자세: '+' / '.join(f'{LABELS[g]} {c[g]}개(실물 {v[g]})' for g in GROUPS))
    def delete(self):
        if self.tracking:self.hold()
        selected=self.table.selection()
        for i in sorted((int(x) for x in selected),reverse=True):del self.model.anchors[i]
        self.refresh_samples();self.autosave()
    def autosave(self):
        try:
            self.model.save(self.calibration_folder/'autosave_correspondence.json');self.storage_error='';return True
        except (OSError,ValueError) as e:self.storage_error=str(e);self.trace.submit({'storage_error':str(e)});return False
    def save(self):
        folder=self.calibration_folder
        try:folder.mkdir(parents=True,exist_ok=True)
        except OSError as e:self.note.set('저장 폴더 오류: '+str(e));return
        path=filedialog.asksaveasfilename(parent=self.root,initialdir=str(folder),initialfile='right_hand_correspondence.json',defaultextension='.json')
        if path:
            try:self.model.save(path);self.note.set('저장: '+path)
            except (OSError,ValueError) as e:self.note.set(str(e))
    def load(self):
        path=filedialog.askopenfilename(parent=self.root,filetypes=[('JSON','*.json')])
        if path:
            try:
                proposed=CorrespondenceProfile.load(path,self.spec,self.source_hand.get());self.hold();self.model=proposed;self.refresh_samples();self.autosave();self.note.set('대응표를 불러왔습니다. 추종은 자동 시작하지 않습니다.')
            except (OSError,ValueError,KeyError,TypeError) as e:self.note.set(str(e))
    def new_profile(self):
        self.hold();self.model=CorrespondenceProfile(self.spec,self.source_hand.get());self.stable.clear();self.refresh_samples();self.autosave();self.note.set('새 대응표입니다. 전체 펴기부터 등록하세요.')
    def set_times(self):
        try:
            if self.group.get()=='all':raise ValueError('이동시간은 손가락 하나를 선택해 입력하세요')
            values=[float(v.strip()) for v in self.times.get().split(',')]
            if len(values)!=3 or any(not math.isfinite(v) or not .05<=v<=10 for v in values):raise ValueError('0.05~10초 범위의 시간 3개를 쉼표로 구분하세요')
            for i,v in zip(GROUPS[self.group.get()],values):self.model.full_stroke_s[i]=v;self.model.timing_measured[i]=self.live and self.time_verified.get()
            self.autosave();self.note.set('시간값을 기록했습니다. 다음 추종 시작 때 적용됩니다. 실제값이 아닌 기본값은 추정으로 남습니다.')
        except ValueError as e:self.note.set(str(e))
    def start_tracking(self):
        try:
            if not self.active or self.pending:raise ValueError('먼저 소켓 접속 후 명시적 펼침 또는 마지막 송신값 재개로 출력을 시작하세요')
            if not self.latest or self.features is None or time.monotonic()-self.latest['captured_at']>self.max_frame_age:raise ValueError('새로운 카메라 인식 결과가 필요합니다')
            if self.source_hand.get()!=self.model.source_hand:raise ValueError('사용자 손과 대응표의 손이 다릅니다')
            self.model.predict(self.features,self.scope.get(),self.target,verified_only=self.live)
            self.worker.post('tracking_times',times=self.model.full_stroke_s)
            self.tracking=True;self.note.set('카메라 연속 추종 중 · 인식 상실/미등록 자세는 HOLD, 자동 재개하지 않습니다.')
            self.trace.submit({'ui_action':'start_continuous_tracking','scope':self.scope.get(),'live':self.live})
        except ValueError as e:self.note.set(str(e))
    def write_preview(self,joints,streaming,source='operator-correspondence joint estimate, NOT measured robot state'):
        if not joints or time.monotonic()-self.last_preview_write<.05:return
        data=dict(utc=time.time(),monotonic_s=time.monotonic(),joints=joints,source=source,motor_streaming=streaming)
        try:
            self.preview_file.parent.mkdir(parents=True,exist_ok=True)
            tmp=self.preview_file.with_name(self.preview_file.name+'.tmp');tmp.write_text(json.dumps(data,allow_nan=False));tmp.replace(self.preview_file);self.last_preview_write=time.monotonic()
        except OSError as e:
            self.last_preview_write=time.monotonic();self.storage_error=str(e);self.note.set('미리보기 저장 오류: '+str(e))
    def update_controls(self):
        manual='normal' if self.active and not self.pending and not self.tracking else 'disabled'
        known=bool(self.last_status and self.last_status.get('same_session_command_known'))
        states=(manual,'disabled' if self.connecting or self.active or self.pending else 'normal','normal' if self.connected and not self.pending and not self.tracking else 'disabled','normal' if known and self.connected and not self.active and not self.pending else 'disabled')
        if getattr(self,'_control_states',None)==states:return
        self._control_states=states
        for widget in (*self.sliders,self.linked):widget.configure(state=manual)
        self.connect_button.configure(state=states[1]);self.open_button.configure(state=states[2]);self.resume_button.configure(state=states[3])
    def pump(self):
        self.worker.ui_seen=time.monotonic()
        incoming=[]
        for _ in range(256):
            try:incoming.append(self.worker.events.get_nowait())
            except queue.Empty:break
        newest_status=max((i for i,e in enumerate(incoming) if e['kind']=='status'),default=-1)
        for i,e in enumerate(incoming):
            self.trace.submit({'worker_event':e});kind=e['kind']
            # Preserve every log/event; repaint only the latest status in this batch.
            if kind=='status' and i!=newest_status:continue
            if kind in ('connected','checked'):
                self.connected=True;self.connecting=False;self.error='';self.connection.set(('실물 단방향 소켓' if self.live else '모의 소켓')+' 접속 · 보드 수신/프로필 미확인')
            elif kind=='armed' and e.get('request_id')==self.request and self.pending:
                self.active=True
                if self.pending=='open':self.worker.post('open_all',request_id=self.request)
                else:self.pending=None
            elif kind=='open_goal_set' and e.get('request_id')==self.request:self.pending=None
            elif kind=='disconnected':self.connected=self.active=self.connecting=False;self.tracking=False;self.pending=None;self.last_status=None;self.connection.set('접속 해제')
            elif kind=='error':
                self.error=e['message'];self.connected=self.active=self.connecting=self.tracking=False;self.pending=None;self.note.set('오류: '+self.error);self.connection.set('오류 후 재접속 필요')
            elif kind=='status':
                self.last_status=e;self.active=e['active'] and not self.error;self.refreshing=True
                if not self.dragging and time.monotonic()>self.dirty_until:
                    self.target=self.spec.to_fractions(e['goal_us'])
                    for i,v in enumerate(self.target):
                        if abs(self.values[i].get()-v*100)>1e-7:self.values[i].set(v*100)
                        if self.pulses[i].get()!=str(e['goal_us'][i]):self.pulses[i].set(str(e['goal_us'][i]))
                for i in range(15):
                    value=str(e['command_us'][i]) if e['command_known'] else '—'
                    if self.sent[i].get()!=value:self.sent[i].set(value)
                self.refreshing=False;self.delivery.set(f"{e['delivery']['result']} · 단방향 송신 결과 · 명령 동기화 기준, 실제 도착/보드 적용 미확인")
                if e['estimated_joint_command']:self.write_preview(e['estimated_joint_command'],e['active'])
            elif kind=='closed':self.trace.close();self.root.destroy();return
        if self.demo:
            level=self.demo_curl.get()
            points=synthetic_hand(level,thumb_across=self.demo_across.get(),thumb_fold=self.demo_fold.get())
            self.draw_demo(points)
            self.accept_frame(dict(detection_valid=True,source_hand=self.source_hand.get(),handedness_score=1.,captured_at=time.monotonic(),frame_id=int(time.monotonic()*30),landmarks=synthetic_hand(level,thumb_across=self.demo_across.get(),thumb_fold=self.demo_fold.get())))
        if self.camera:
            process,stop=self.camera
            for _ in range(3):
                try:sample=self.frames.get_nowait()
                except queue.Empty:break
                if sample.get('kind')=='frame':self.accept_frame(sample)
                elif sample.get('kind')=='error':
                    self.camera_problem=sample['message'];self.trace.submit({'camera_error':self.camera_problem});self.camera_note.set(self.camera_problem);self.hold()
            if stop.is_set() or not process.is_alive():
                if self.tracking:self.hold()
                self.camera_note.set(self.camera_problem or '카메라 종료/오류 · 입력 경로를 확인하세요')
            try:
                data=self.images.get_nowait();self.previews_received+=1
                if self.photo is None:
                    self.photo=tk.PhotoImage(data=data['ppm'],format='PPM');self.image.configure(image=self.photo,text='')
                else:self.photo.configure(data=data['ppm'],format='PPM')
            except queue.Empty:pass
        if self.tracking and (not self.latest or time.monotonic()-self.latest['captured_at']>self.max_frame_age):self.hold();self.note.set('영상 만료로 HOLD했습니다. 다시 인식돼도 자동 추종하지 않습니다.')
        self.update_controls();self.root.after(30,self.pump)
    def draw_demo(self,points):
        c=self.demo_canvas
        if c is None:return
        c.delete('all');w=max(c.winfo_width(),480);x0=w*.46
        xy=[(x0+p[0]*2000,335-p[1]*2000-p[2]*600) for p in points]
        c.create_text(w/2,18,text='합성 랜드마크 데모 · 실제 카메라 영상 아님',fill='white')
        for chain in ((0,1,2,3,4),(0,5,6,7,8),(5,9,10,11,12),(9,13,14,15,16),(13,17,18,19,20),(0,17)):
            for a,b in zip(chain,chain[1:]):c.create_line(*xy[a],*xy[b],fill='#69d9a0',width=3)
        for i,(x,y) in enumerate(xy):c.create_oval(x-3,y-3,x+3,y+3,fill='#ffd36a',outline='');c.create_text(x+7,y-7,text=str(i),fill='#d0d5dd',font=('Arial',8))
    def close(self):
        if self.closing:return
        self.closing=True;self.hold();self.stop_camera();self.worker.close()


def main():
    from .gui import project_default
    parser=argparse.ArgumentParser();parser.add_argument('--project',default=str(project_default()));parser.add_argument('--config');parser.add_argument('--live',action='store_true');parser.add_argument('--demo',action='store_true');parser.add_argument('--source-hand',choices=('Right','Left'),default='Right');args=parser.parse_args()
    if args.live and args.demo:parser.error('--demo cannot be used with --live')
    config=args.config or str(Path(args.project)/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand_camera.yaml')
    window=tk.Tk();TrackingGui(window,args.project,config,live=args.live,source_hand=args.source_hand,demo=args.demo);window.mainloop()
if __name__=='__main__':main()

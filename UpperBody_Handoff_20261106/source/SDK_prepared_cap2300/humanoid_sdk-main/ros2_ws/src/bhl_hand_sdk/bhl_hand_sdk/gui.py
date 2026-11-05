"""DexHand desktop: named motors, embedded USB camera, calibration deployment.

No CAN is opened until Connect. Tk lives only on the main thread. All CAN
access belongs to GuiWorker; MediaPipe runs in a separate spawned process.
"""
import argparse
import json
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .cli import default_config
from .config import load_config
from .gui_model import CalibrationDraft
from .gui_worker import GuiWorker


def project_default():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'dexhand_control/profiles/left_hand.json').is_file():
            return parent
    return Path.cwd()


class HandGui:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.project = Path(args.project).resolve()
        self.config = load_config(args.config)
        self.drafts = {side: CalibrationDraft(json.loads((self.project /
            f'dexhand_control/profiles/{side}_hand.json').read_text(encoding='utf-8')), self.config.hand(side))
            for side in ('left', 'right')}
        self.worker = GuiWorker(args.config)
        self.connected = self.active = self.refreshing = self.closing = False
        self.dirty = set()
        self.dragging = set()
        self.move_tokens = {}
        self.guide_text = tk.StringVar(value="")
        self.last_status = None
        self.photo = None
        self.mode = 'manual'
        self.exporting = False
        self.side = tk.StringVar(value=args.hand)
        self.channel = tk.StringVar(value=args.channel)
        self.simulated = tk.BooleanVar(value=args.dry_run)
        self.initial_open = tk.BooleanVar(value=False)
        self.connection_text = tk.StringVar(value='연결 안 됨 · 모터 출력 없음')
        self.result_text = tk.StringVar(value='CAN 응답 없음')
        self.camera_text = tk.StringVar(value='카메라 미리보기 대기')
        self.camera_device = tk.StringVar(value='0')
        self.cal_pulse = tk.StringVar(value='1500')
        self.cal_note = tk.StringVar(value='기록한 값은 초안입니다. 현재 펌웨어와 실시간 허용 범위는 바뀌지 않습니다.')
        self.scales, self.targets, self.applied, self.row_labels, self.motor_widgets = [], [], [], [], []
        self._build()
        self._populate()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind('<Escape>', lambda _: self.post('hold'))
        self.root.after(50, self.pump)

    @property
    def draft(self):
        return self.drafts[self.side.get()]

    def _build(self):
        root = self.root
        root.title('DexHand · 개별 모터 / 카메라 / 캘리브레이션')
        root.geometry('1420x940')
        root.minsize(1180, 850)
        style = ttk.Style(root)
        if 'clam' in style.theme_names():
            style.theme_use('clam')
        style.configure('Title.TLabel', font=('Arial', 16, 'bold'))
        style.configure('Status.TLabel', font=('Arial', 11, 'bold'))
        outer = ttk.Frame(root, padding=14)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='DexHand 손 제어', style='Title.TLabel').pack(anchor='w')
        ttk.Label(outer, text='µs와 적용 응답은 PWM 명령입니다. 손가락의 실제 각도·도달 위치는 측정하지 않습니다.').pack(anchor='w', pady=(3, 10))
        guide = ttk.LabelFrame(outer, text='사용 순서 · 슬라이더가 잠긴 이유', padding=10)
        guide.pack(fill='x', pady=(0, 10))
        ttk.Label(guide, textvariable=self.guide_text, font=('Arial', 12, 'bold'), foreground='#172b4d', wraplength=1320).pack(anchor='w')
        top = ttk.Frame(outer)
        top.pack(fill='x')
        ttk.Label(top, text='손').pack(side='left')
        self.side_box = ttk.Combobox(top, textvariable=self.side, values=('left', 'right'), width=7, state='readonly')
        self.side_box.pack(side='left', padx=6)
        self.side_box.bind('<<ComboboxSelected>>', lambda _: self._populate())
        ttk.Label(top, text='CAN').pack(side='left')
        self.channel_entry = ttk.Entry(top, textvariable=self.channel, width=9)
        self.channel_entry.pack(side='left', padx=6)
        self.sim_box = ttk.Checkbutton(top, text='모의 실행 · 실제 CAN 연결 없음', variable=self.simulated, command=self._guide)
        self.sim_box.pack(side='left', padx=8)
        self.connect_button = ttk.Button(top, text='1. CAN 연결 확인', command=self.connect)
        self.connect_button.pack(side='left', padx=5)
        self.disconnect_button = ttk.Button(top, text='연결 해제', command=lambda: self.post('disconnect'))
        self.disconnect_button.pack(side='left')
        actions = ttk.Frame(outer)
        actions.pack(fill='x', pady=10)
        self.cold_box = ttk.Checkbutton(actions, text='처음 켠 빈손: 펼침으로 시작 (확인 후 체크)', variable=self.initial_open)
        self.cold_box.pack(side='left')
        self.arm_button = ttk.Button(actions, text='2. 출력 시작 / 재개', command=lambda: self.post('arm', initial_open=self.initial_open.get()))
        self.arm_button.pack(side='left', padx=8)
        self.open_all_button = ttk.Button(actions, text='손 전체 펼침', command=self.open_all)
        self.open_all_button.pack(side='left', padx=5)
        ttk.Button(actions, text='유지 HOLD  [Esc]', command=lambda: self.post('hold')).pack(side='left', padx=5)
        ttk.Button(actions, text='PWM 끄기 · 유지력 해제', command=lambda: self.post('off')).pack(side='left', padx=5)
        ttk.Label(outer, textvariable=self.connection_text, style='Status.TLabel').pack(anchor='w')
        ttk.Label(outer, textvariable=self.result_text).pack(anchor='w', pady=(2, 10))
        body = ttk.Frame(outer)
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)
        self.tabs = ttk.Notebook(body)
        self.tabs.grid(row=0, column=0, sticky='nsew', padx=(0, 12))
        manual, camera, calibration = [ttk.Frame(self.tabs, padding=10) for _ in range(3)]
        for frame, label in zip((manual, camera, calibration), ('개별 모터', '카메라 제어', '캘리브레이션')):
            self.tabs.add(frame, text=label)
        self.tabs.bind('<<NotebookTabChanged>>', self.mode_changed)
        headers = ('손가락·위치 / GPIO', '당김 비율 0 → 100%', '당김% / 목표 µs', '마지막 적용 µs')
        for column, label in enumerate(headers):
            ttk.Label(manual, text=label).grid(row=0, column=column, sticky='w', padx=4, pady=(0, 8))
        manual.columnconfigure(1, weight=1)
        for index in range(15):
            name = ttk.Label(manual, width=20)
            name.grid(row=index+1, column=0, sticky='w', pady=3)
            fraction, target, applied = tk.DoubleVar(value=0), tk.StringVar(value='—'), tk.StringVar(value='—')
            slider = ttk.Scale(manual, from_=0, to=100, variable=fraction,
                               command=lambda value, i=index: self.slider(i, float(value)/100.0))
            slider.bind("<ButtonPress-1>", lambda event, i=index: self.dragging.add(i), add="+")
            slider.bind("<ButtonRelease-1>", lambda event, i=index: self.dragging.discard(i), add="+")
            slider.grid(row=index+1, column=1, sticky='ew', padx=5)
            ttk.Label(manual, textvariable=target, width=12).grid(row=index+1, column=2)
            ttk.Label(manual, textvariable=applied, width=13).grid(row=index+1, column=3)
            self.row_labels.append(name)
            self.scales.append(fraction)
            self.targets.append(target)
            self.applied.append(applied)
            self.motor_widgets.append(slider)
        ttk.Label(manual, text='3. 손잡이를 드래그하거나 ← → 키로 1%씩 조절하세요. 0%=풀림, 100%=보정된 최대 당김.\n슬라이더를 움직이면 해당 모터의 목표만 변경합니다.\n나머지 모터는 기존 목표를 유지합니다. 속도 제한: 초당 전체 범위의 25%.\n엄지 양쪽 당김의 합이 허용치를 넘으면 출력을 유지하고 오류를 표시합니다.',
                  wraplength=650).grid(row=17, column=0, columnspan=4, sticky='w', pady=14)
        ttk.Label(camera, text='USB 카메라 → MediaPipe → CAN → 선택한 손', style='Status.TLabel').pack(anchor='w', pady=8)
        ttk.Label(camera, text='현재 카메라 모드는 손 전체의 펴짐 / 주먹을 따라갑니다.\n개별 손가락 각도를 그대로 복사하는 모드는 아닙니다.\n주먹을 쥐면 보정 범위의 최대 25%까지 당깁니다.\n\n1. 미리보기를 시작합니다.\n2. 카메라에 편 손 하나를 보여줍니다.\n3. 위의 출력 시작 버튼을 누릅니다.\n\n손이 사라지거나 둘 이상 보이거나 영상이 늦어지면 HOLD합니다.\n다시 인식돼도 자동으로 움직이지 않습니다.', wraplength=640).pack(anchor='w')
        camera_bar = ttk.Frame(camera)
        camera_bar.pack(anchor='w', pady=15)
        ttk.Label(camera_bar, text='카메라 번호 / 경로').pack(side='left')
        ttk.Entry(camera_bar, textvariable=self.camera_device, width=14).pack(side='left', padx=6)
        ttk.Button(camera_bar, text='미리보기 시작', command=lambda: self.post('camera_start', camera=self.camera_device.get(), model=self.args.model)).pack(side='left')
        ttk.Button(camera_bar, text='카메라 종료', command=lambda: self.post('camera_stop')).pack(side='left', padx=6)
        self.cal_table = ttk.Treeview(calibration, columns=('name', 'gpio', 'open', 'close', 'bounds'), show='headings', height=15, selectmode='browse')
        for key, label, width in (('name', '손가락·위치', 160), ('gpio', 'GPIO', 50), ('open', '초안 풀림', 80), ('close', '초안 당김', 80), ('bounds', '현재 허용 µs', 125)):
            self.cal_table.heading(key, text=label)
            self.cal_table.column(key, width=width, stretch=key == 'name')
        self.cal_table.pack(fill='x')
        self.cal_table.bind('<<TreeviewSelect>>', self.cal_selected)
        bar = ttk.Frame(calibration)
        bar.pack(fill='x', pady=8)
        ttk.Label(bar, text='선택 모터 PWM µs').pack(side='left')
        ttk.Entry(bar, textvariable=self.cal_pulse, width=8).pack(side='left', padx=5)
        for step in (-25, -5, 5, 25):
            ttk.Button(bar, text=f'{step:+}', width=4, command=lambda s=step: self.jog(s)).pack(side='left', padx=2)
        ttk.Button(bar, text='선택 모터로 전송', command=self.cal_send).pack(side='left', padx=6)
        record = ttk.Frame(calibration)
        record.pack(fill='x')
        ttk.Button(record, text='이 값을 풀림으로 기록', command=lambda: self.record('open_us')).pack(side='left')
        ttk.Button(record, text='이 값을 당김으로 기록', command=lambda: self.record('close_us')).pack(side='left', padx=5)
        files = ttk.Frame(calibration)
        files.pack(fill='x', pady=(7, 0))
        ttk.Button(files, text='JSON 저장', command=self.save_draft).pack(side='left')
        ttk.Button(files, text='JSON 불러오기', command=self.load_draft).pack(side='left', padx=5)
        ttk.Button(files, text='펌웨어 + PC 설정 내보내기', command=self.export).pack(side='left', padx=5)
        ttk.Label(calibration, textvariable=self.cal_note, wraplength=640).pack(anchor='w', pady=8)
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky='nsew')
        ttk.Label(right, text='카메라 영상', style='Status.TLabel').pack(anchor='w')
        self.preview = tk.Label(right, text='카메라 모드에서 미리보기를 시작하세요', background='#172531', foreground='#c9d5df', width=60, height=23)
        self.preview.pack(fill='x', pady=8)
        ttk.Label(right, textvariable=self.camera_text, wraplength=470).pack(anchor='w')
        ttk.Label(right, text='동작 상태 / 오류 기록').pack(anchor='w', pady=(12, 4))
        self.log = tk.Text(right, height=9, wrap='word', state='disabled')
        self.log.pack(fill='both', expand=True)
        self._set_connected(False)

    def log_message(self, message):
        self.log.configure(state='normal')
        self.log.insert('end', message+'\n')
        if int(self.log.index('end-1c').split('.')[0]) > 160:
            self.log.delete('1.0', '30.0')
        self.log.see('end')
        self.log.configure(state='disabled')

    def _populate(self):
        self.refreshing = True
        motors = self.draft.original['motors']
        self.cal_table.delete(*self.cal_table.get_children())
        for i, motor in enumerate(motors):
            self.row_labels[i].configure(text=f"{i+1:02}  {motor['name']}  / {motor['gpio']}")
            self.scales[i].set(0)
            self.targets[i].set(f"0% | {motor['open_us']}")
            self.applied[i].set('—')
            draft = self.draft.profile['motors'][i]
            self.cal_table.insert('', 'end', iid=str(i), values=(motor['name'], motor['gpio'],
                draft['open_us'], draft['close_us'], f"{min(motor['open_us'], motor['close_us'])}–{max(motor['open_us'], motor['close_us'])}"))
        self.cal_table.selection_set('0')
        self.cal_pulse.set(str(motors[0]['open_us']))
        self.refreshing = False

    def _set_connected(self, connected):
        self.connected = connected
        self.side_box.configure(state='disabled' if connected else 'readonly')
        for widget in (self.channel_entry, self.sim_box, self.connect_button):
            widget.configure(state='disabled' if connected else 'normal')
        self.disconnect_button.configure(state='normal' if connected else 'disabled')
        self.arm_button.configure(state='normal' if connected else 'disabled')
        self._motor_states()

    def _motor_states(self):
        for slider in self.motor_widgets:
            slider.configure(state='normal' if self.connected and self.active and self.mode == 'manual' else 'disabled')
        self.open_all_button.configure(state='normal' if self.connected and self.active and self.mode == 'manual' else 'disabled')
        self._guide()

    def _guide(self):
        prefix = '모의 실행 · 실물 손은 움직이지 않음\n' if self.simulated.get() else ''
        if not self.connected:
            text = '① 손 보드 전원 연결 → ② [1. CAN 연결 확인]을 누르세요.\n슬라이더 잠김: 아직 CAN 연결이 확인되지 않았습니다.'
        elif not self.active:
            text = 'CAN 연결 확인됨 · 아직 슬라이더는 잠겨 있습니다.\n처음 켠 빈손만 [펼침으로 시작]을 체크하고 [2. 출력 시작 / 재개]를 누르세요. 이미 구동 중이던 손은 체크 없이 이어받습니다.'
        elif self.mode != 'manual':
            text = '현재 카메라/캘리브레이션 모드입니다. 개별 슬라이더는 [개별 모터] 탭에서 사용하세요. 탭 변경 시 HOLD되므로 출력 시작을 다시 눌러야 합니다.'
        else:
            text = '슬라이더 사용 가능 · 손잡이를 드래그하거나 ← → 키로 1%씩 조절하세요.\n0%=풀림, 100%=보정된 최대 당김. 실제 이동은 속도 제한에 따라 천천히 진행됩니다. 멈추려면 [유지 HOLD] 또는 Esc.'
        self.guide_text.set(prefix+text)

    def _finish_move(self, index, token):
        if self.move_tokens.get(index) != token:
            return
        if index in self.dragging:
            self.root.after(100, lambda: self._finish_move(index, token))
            return
        self.dirty.discard(index)

    def post(self, action, **kwargs):
        try:
            self.worker.post(action, **kwargs)
        except queue.Full:
            self.worker.post('hold')
            self.log_message('명령 대기열이 가득 참 · 유지 요청')

    def connect(self):
        # Lock identity before the asynchronous PONG arrives. A side change
        # during connection must never pair the left UI with the right CAN node.
        for widget in (self.side_box, self.channel_entry, self.sim_box, self.connect_button):
            widget.configure(state='disabled')
        self.guide_text.set('CAN 응답 확인 중… 손가락 출력은 아직 시작하지 않습니다.')
        self.post('connect', hand=self.side.get(), channel=self.channel.get(), dry_run=self.simulated.get())

    def mode_changed(self, _=None):
        self.mode = ('manual', 'camera', 'calibration')[self.tabs.index(self.tabs.select())]
        self.active = False
        self._motor_states()
        if self.connected:
            self.post('mode', mode=self.mode)
        self.preview.configure(image='', text='카메라 모드에서 미리보기를 시작하세요', width=60, height=23)
        self.photo = None
        self.camera_text.set('카메라 미리보기 대기')

    def slider(self, index, fraction):
        if self.refreshing or not self.connected or not self.active or self.mode != 'manual':
            return
        motor = self.draft.original['motors'][index]
        pulse = round(motor['open_us']+fraction*(motor['close_us']-motor['open_us']))
        self.targets[index].set(f"{fraction*100:.0f}% | {pulse}")
        self.dirty.add(index)
        self.worker.move(index, pulse)
        token = self.move_tokens.get(index, 0)+1
        self.move_tokens[index] = token
        self.root.after(250, lambda: self._finish_move(index, token))

    def open_all(self):
        if not self.connected or not self.active or self.mode != 'manual':
            self.log_message('손 전체 펼침: CAN 연결 → 출력 시작 → 개별 모터 탭 순서로 준비하세요.')
            return
        self.dragging.clear()
        self.refreshing = True
        try:
            for i, motor in enumerate(self.draft.original['motors']):
                self.scales[i].set(0)
                self.targets[i].set(f"0% | {motor['open_us']}")
                self.dirty.add(i)
                token = self.move_tokens.get(i, 0)+1
                self.move_tokens[i] = token
                self.root.after(250, lambda i=i, token=token: self._finish_move(i, token))
        finally:
            self.refreshing = False
        self.post('open_all')
        self.log_message('전체 펼침 목표 요청 · 15개 모터를 각자의 풀림 값으로 이동합니다. 실제 도달 위치는 측정하지 않습니다.')

    def selected(self):
        selection = self.cal_table.selection()
        if not selection:
            raise ValueError('모터를 선택하세요')
        return int(selection[0])

    def cal_selected(self, _=None):
        if self.refreshing:
            return
        index = self.selected()
        pulse = (self.last_status or {}).get('applied_us')
        self.cal_pulse.set(str(pulse[index] if pulse else self.draft.original['motors'][index]['open_us']))

    def jog(self, increment):
        try:
            index = self.selected()
            pulse = int(self.cal_pulse.get())+increment
            spec = self.config.hand(self.side.get())
            if not spec.low_us[index] <= pulse <= spec.high_us[index]:
                raise ValueError(f'현재 허용 범위: {spec.low_us[index]}..{spec.high_us[index]} µs')
            self.cal_pulse.set(str(pulse))
            self.cal_send()
        except ValueError as error:
            self.log_message(str(error))

    def cal_send(self):
        try:
            if not self.connected or not self.active or self.mode != 'calibration':
                raise ValueError('캘리브레이션 모드에서 출력을 먼저 시작하세요')
            self.worker.move(self.selected(), int(self.cal_pulse.get()))
        except ValueError as error:
            self.log_message(str(error))

    def record(self, endpoint):
        try:
            index = self.selected()
            self.draft.record(index, endpoint, int(self.cal_pulse.get()))
            selection = str(index)
            self._populate()
            self.cal_table.selection_set(selection)
            motor = self.draft.profile['motors'][index]
            self.cal_pulse.set(str(motor[endpoint]))
            self.cal_note.set(f"{motor['name']} {'풀림' if endpoint == 'open_us' else '당김'} {motor[endpoint]} µs 기록. 새 펌웨어 + PC 설정 배포 전에는 적용되지 않습니다.")
        except ValueError as error:
            self.log_message(str(error))

    def save_draft(self):
        self.post('hold')
        path = filedialog.asksaveasfilename(parent=self.root, title='캘리브레이션 JSON 초안 저장',
                    initialdir=str(self.project.parent), initialfile=f'{self.side.get()}_hand.json', defaultextension='.json', filetypes=[('JSON', '*.json')])
        if path:
            try:
                resolved = Path(path).resolve()
                protected = [self.project/'dexhand_control/profiles/left_hand.json', self.project/'dexhand_control/profiles/right_hand.json', Path(self.args.config)]
                if resolved in {p.resolve() for p in protected}:
                    raise ValueError('실행 중 프로필 대신 별도 JSON 파일로 저장하세요. 적용 파일은 내보내기를 사용하세요.')
                self.draft.save(path)
                self.log_message('초안 저장: '+path)
            except Exception as error:
                messagebox.showerror('저장 실패', str(error), parent=self.root)

    def load_draft(self):
        self.post('hold')
        path = filedialog.askopenfilename(parent=self.root, title='저장한 캘리브레이션 JSON 불러오기',
                    initialdir=str(self.project.parent), filetypes=[('JSON', '*.json')])
        if path:
            try:
                self.draft.load(path)
                self._populate()
                self.cal_note.set('초안 불러옴 · 실시간 허용 범위는 그대로이며 새 적용 파일을 내보내야 합니다.')
                self.log_message('초안 불러오기: '+path)
            except Exception as error:
                messagebox.showerror('불러오기 실패', str(error), parent=self.root)

    def export(self):
        if self.exporting:
            return
        self.post('hold')
        try:
            for draft in self.drafts.values():
                draft.validate()
        except ValueError as error:
            messagebox.showerror('끝점 확인', str(error), parent=self.root)
            return
        path = filedialog.asksaveasfilename(parent=self.root, title='적용 파일을 만들 새 폴더 이름', initialdir=str(self.project.parent), initialfile='DexHandCalibration', defaultextension='')
        if not path:
            return
        # Snapshot on Tk thread; the deployment worker never reads widgets.
        profiles = {side: json.dumps(draft.profile, ensure_ascii=False) for side, draft in self.drafts.items()}
        self.exporting = True
        def run_export():
            try:
                with tempfile.TemporaryDirectory(prefix='dexhand-calibration-') as work:
                    command = [sys.executable, '-B', str(self.project / 'dexhand_control/tools/export_calibration.py'), '--output', path]
                    for side, content in profiles.items():
                        draft_path = Path(work) / f'{side}_hand.json'
                        draft_path.write_text(content, encoding='utf-8')
                        command += ['--'+side, str(draft_path)]
                    result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', timeout=30,
                        env={**__import__('os').environ, 'PYTHONUTF8': '1'},
                        **({'creationflags': subprocess.CREATE_NO_WINDOW} if sys.platform == 'win32' else {}))
                    if result.returncode:
                        raise RuntimeError(result.stderr.strip())
                self.worker.events.put(dict(kind='exported', path=path))
            except Exception as error:
                self.worker.events.put(dict(kind='export_error', message=str(error)))
        threading.Thread(target=run_export, daemon=True, name='DexHand-export').start()

    def pump(self):
        self.worker.ui_seen = time.monotonic()
        for _ in range(64):
            try:
                event = self.worker.events.get_nowait()
            except queue.Empty:
                break
            kind = event['kind']
            if kind == 'connected':
                self._set_connected(True)
                self.post('mode', mode=self.mode)
                self.log_message(f"{'모의 연결' if event['dry_run'] else 'CAN 연결'}: {event['hand']} / ID {event['node']} / {event['tag']}")
            elif kind == 'disconnected':
                self.active = False
                self._set_connected(False)
                self.last_status = None
                self.connection_text.set('연결 해제')
            elif kind == 'status':
                self.last_status = event
                self.active = event['active']
                self._motor_states()
                prefix = '모의 실행 · ' if event['dry_run'] else ''
                self.connection_text.set(prefix+event['reason'])
                delivery = event['delivery']
                self.result_text.set(f"{delivery['result']} / ACK {delivery['latency_ms']:.1f} ms · 실제 위치 피드백 없음")
                if self.mode == 'camera':
                    vision = event['camera']
                    label = ('주먹' if vision['closed'] else '펴짐') if vision['valid'] else '손 인식 없음 / 영상 만료'
                    age = '—' if vision['age'] is None else f"{vision['age']:.2f}s"
                    self.camera_text.set(f'{label} · 입력 나이 {age} · 시작은 편 손 하나가 보일 때 누르세요')
                self.refreshing = True
                spec = self.config.hand(self.side.get())
                for i, pulse in enumerate(event['goal_us']):
                    self.applied[i].set('—' if event['applied_us'] is None else str(event['applied_us'][i]))
                    if i not in self.dirty and i not in self.dragging:
                        fraction = (pulse-spec.open_us[i])/(spec.close_us[i]-spec.open_us[i])
                        self.targets[i].set(f'{fraction*100:.0f}% | {pulse}' if event['command_known'] else '—')
                        self.scales[i].set(fraction*100)
                self.refreshing = False
            elif kind in ('error', 'export_error'):
                self.log_message(event['message'])
                if kind == 'error':
                    self.active = False
                    self._motor_states()
                    friendly = ('손 보드 응답 없음 · 손 보드/트랜시버 전원, CAN 배선, can13 펌웨어를 확인한 뒤 다시 연결하세요.' if 'NO_PONG' in event['message'] else
                                '펌웨어와 PC 프로필이 다릅니다. 같은 can13 버전으로 맞춰주세요.' if 'PROFILE_MISMATCH' in event['message'] else event['message'])
                    self.connection_text.set(friendly)
                if kind == 'error' and not self.connected:
                    self._set_connected(False)
                if kind == 'export_error':
                    self.exporting = False
            elif kind == 'exported':
                self.exporting = False
                self.log_message('적용 파일 생성: '+event['path'])
                self.cal_note.set('내보내기 완료. 변경한 손 펌웨어를 업로드하고 profiles JSON + dexhand.yaml을 함께 갱신한 뒤 GUI를 다시 실행하세요.')
            elif kind == 'closed':
                if self.worker.shutdown_error:
                    messagebox.showwarning('출력 유지 확인 실패', self.worker.shutdown_error, parent=self.root)
                self.root.destroy()
                return
        try:
            image = self.worker.images.get_nowait()
            if time.monotonic()-image['captured_at'] <= .35 and self.mode == 'camera':
                self.photo = tk.PhotoImage(data=image['ppm'], format='PPM')
                self.preview.configure(image=self.photo, text='', width=480, height=360)
        except queue.Empty:
            pass
        if self.mode == 'camera' and self.worker.vision is None:
            self.camera_text.set('미리보기 중단 / 시작 전 · 모터는 자동으로 시작하지 않습니다')
        self.root.after(50, self.pump)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.connection_text.set('출력 유지 요청 후 종료 중…')
        self.worker.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=default_config())
    parser.add_argument('--project', default=str(project_default()))
    parser.add_argument('--hand', choices=('left', 'right'), default='right')
    parser.add_argument('--channel', default='can0')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--model', default=str(project_default() / 'dexhand_control/.cache/hand_landmarker.task'))
    args = parser.parse_args(argv)
    root = tk.Tk()
    try:
        HandGui(root, args)
    except Exception:
        root.destroy()
        raise
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

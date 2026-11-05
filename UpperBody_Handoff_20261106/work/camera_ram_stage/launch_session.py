from pathlib import Path
import os,sys,time,json,signal,tkinter as tk

BASE=Path('/run/user/1000/bhl_hand_camera_session')

def atomic(path,data):
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False));temporary.replace(path)

def main():
    output_mode=os.environ.get('BHL_HAND_OUTPUT_MODE','live')
    if output_mode not in ('live','preview'):raise ValueError('Unknown output mode')
    if output_mode=='preview':
        import socket
        def block_can(event,args):
            if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:raise PermissionError('Preview GUI cannot open CAN sockets')
        sys.addaudithook(block_can)
    os.environ['BHL_HAND_DATA_DIR']=str(BASE/'data')
    os.environ['BHL_HAND_CAMERA_SAMPLE_PATH']=str(BASE/'camera_control_sample.json')
    os.environ['BHL_HAND_CAMERA_OBSERVATION_PATH']=str(BASE/'latest_observation.json')
    os.environ['BHL_HAND_CAMERA_ROLE']=os.environ.get('BHL_HAND_CAMERA_ROLE','robot_observation')
    os.environ['FONTCONFIG_FILE']=str(BASE/'fonts.conf')
    for key,folder in (('MPLCONFIGDIR','matplotlib'),('TMPDIR','tmp'),('XDG_CACHE_HOME','cache')):
        path=BASE/folder;path.mkdir(exist_ok=True);os.environ[key]=str(path)
    project=BASE/'project';package=project/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk'
    sys.path.insert(0,str(package))
    from bhl_hand_sdk.hand_tracking_gui import TrackingGui
    class TrialGui(TrackingGui):
        def accept_frame(self,sample):
            super().accept_frame(sample)
            if os.environ['BHL_HAND_CAMERA_ROLE']=='robot_observation':
                self.features=self.filtered=self.prediction=None;self.stable.clear()
                self.feature_text.set('로봇 손 관찰 영상 · 표시 PWM은 명령값 · 관절각/장력 실측값 없음')
        def start_tracking(self):
            if os.environ['BHL_HAND_CAMERA_ROLE']=='robot_observation':
                self.note.set('현재 카메라는 로봇 손 관찰용입니다. 사람 손 추종 입력으로 사용하지 않습니다.');return
            super().start_tracking()
        def hold(self):
            (BASE/'stop_index_trial').touch()
            super().hold()
    config_path=Path(os.environ.get('BHL_HAND_CAMERA_CONFIG',str(package/'config/dexhand_camera.yaml')))
    root=tk.Tk();app=TrialGui(root,project,config_path,live=output_mode=='live',demo=False,max_frame_age=.35)
    root.title('네 손가락 사람 손 추적 · 비구동 미리보기 · CAN 송신 0' if output_mode=='preview' else 'ZED 사람 손 추종 · Esc/HOLD 중단' if os.environ['BHL_HAND_CAMERA_ROLE']=='human_control' else 'ZED 로봇 손 관찰 · PWM 명령/영상 비교 · Esc/HOLD 중단')
    errors=[]
    def callback_error(kind,value,tb):
        import traceback
        message=''.join(traceback.format_exception(kind,value,tb));errors.append(message);print(message,flush=True)
        app.note.set('화면 처리 오류: '+str(value))
    root.report_callback_exception=callback_error
    signal.signal(signal.SIGTERM,lambda *_:root.after(0,app.close))
    app.camera_device.set('/dev/video0')
    existing=app.calibration_folder/'autosave_correspondence.json'
    if existing.exists() and os.environ['BHL_HAND_CAMERA_ROLE']=='human_control':
        from bhl_hand_sdk.hand_retarget import CorrespondenceProfile
        app.model=CorrespondenceProfile.load(existing,app.spec,app.source_hand.get());app.refresh_samples()
    if os.environ['BHL_HAND_CAMERA_ROLE']=='human_control':app.autosave()
    root.lift();root.attributes('-topmost',True);root.after(1800,lambda:root.attributes('-topmost',False))
    def status():
        latest=app.latest or {};age=time.monotonic()-latest.get('captured_at',time.monotonic())
        ack=None
        try:ack=json.loads((BASE/'backup_ack.json').read_text())
        except (OSError,ValueError):pass
        backed_up=bool(ack and 0<=time.time()-ack.get('utc',0)<8)
        trial={}
        try:trial=json.loads((BASE/'data/index_trial_state.json').read_text())
        except (OSError,ValueError):pass
        preview={}
        if output_mode=='preview':
            try:preview=json.loads((BASE/'data/tracking_preview.json').read_text())
            except (OSError,ValueError):pass
        message=preview.get('message') if output_mode=='preview' else trial.get('message')
        if message and app.note.get()!=message:app.note.set(message)
        command=trial.get('command_us');goal=trial.get('goal_us') or command
        if output_mode=='preview':goal=preview.get('goal_us')
        if command and goal:
            app.refreshing=True
            try:
                for i,fraction in enumerate(app.spec.to_fractions(goal)):
                    if abs(app.values[i].get()-fraction*100)>1e-6:app.values[i].set(fraction*100)
                    if app.pulses[i].get()!=str(goal[i]):app.pulses[i].set(str(goal[i]))
                    if app.sent[i].get()!=str(command[i]):app.sent[i].set(str(command[i]))
            finally:app.refreshing=False
        elif output_mode=='preview':
            for value in app.pulses:
                if value.get()!='—':value.set('—')
        if output_mode=='preview':app.delivery.set('CAN 송신 0 · 목표는 예상값 · 송신 열은 직전 실물 명령이며 현재 위치 실측값이 아닙니다')
        app.connect_button.configure(state='disabled')  # The dedicated trial process owns CAN.
        output_note=('외부 실물 시험 진행 중' if trial.get('active') else '추종 중지 · HOLD 명령 · 실제 PWM 상태 미측정' if trial.get('phase') in ('HOLD','COMPLETE') else '추종 대기 · 실제 PWM 상태 미측정')
        if output_mode=='preview':output_note='비구동 목표 미리보기 · CAN 송신 0 · 직전 실물 PWM 유지 가능'
        banner='임시 메모리 저장 · '+('제어 PC로 자동 백업 중' if backed_up else 'PC 백업 연결 확인 중 · 아직 전원을 끄지 마세요')+' · 손 CAN '+app.channel.get()+' · '+output_note
        if app.storage_note.get()!=banner:app.storage_note.set(banner)
        atomic(BASE/'status.json',dict(utc=time.time(),pid=os.getpid(),visible=bool(root.winfo_viewable()),camera_role=os.environ['BHL_HAND_CAMERA_ROLE'],output_mode=output_mode,tracking_preview=preview,camera_worker_alive=bool(app.camera and app.camera[0].is_alive()),frame_id=latest.get('frame_id'),captured_at=latest.get('captured_at'),external_index_trial=trial,frames_received=app.frames_received,previews_received=app.previews_received,frame_age_s=age if latest else None,detection_valid=latest.get('detection_valid'),source_hand=latest.get('source_hand'),handedness_score=latest.get('handedness_score'),timing_ms=latest.get('timing_ms'),view_mode=latest.get('view_mode'),frame_size=latest.get('frame_size'),recognized_features=app.features,camera_note=app.camera_note.get(),camera_problem=app.camera_problem,camera_image_loaded=bool(app.photo),canvas_size=[app.photo.width(),app.photo.height()] if app.photo else None,live_output=app.live,can_connected=app.connected or bool(trial.get('active')),trial_streaming=bool(trial.get('active')),motors_active=None,physical_pwm_measured=False,max_frame_age_s=app.max_frame_age,anchors=len(app.model.anchors),callback_errors=errors,trace_error=app.trace.error,storage_error=app.storage_error,local_backup_fresh=backed_up))
        root.after(50,status)
    root.after(400,app.start_camera);root.after(50,status)
    root.mainloop()

if __name__=='__main__':main()

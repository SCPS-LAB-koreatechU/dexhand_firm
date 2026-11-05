"""Production camera/GUI with simulated controller and a hard AF_CAN audit block."""
from pathlib import Path
import os,sys,json,time,signal,socket,tkinter as tk
BASE=Path('/run/user/1000/bhl_hand_camera_session')
BLOCKED=[]

def audit(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==getattr(socket,'AF_CAN',29):
        BLOCKED.append({'utc':time.time(),'event':event});raise RuntimeError('Diagnostic mode: AF_CAN socket creation is forbidden')
sys.addaudithook(audit)

def atomic(path,data):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False));tmp.replace(path)

def main():
    os.environ['BHL_HAND_DATA_DIR']=str(BASE/'no_can_data');os.environ['FONTCONFIG_FILE']=str(BASE/'fonts.conf')
    for key,folder in (('MPLCONFIGDIR','matplotlib'),('TMPDIR','tmp'),('XDG_CACHE_HOME','cache')):
        p=BASE/folder;p.mkdir(exist_ok=True);os.environ[key]=str(p)
    project=BASE/'project';package=project/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk';sys.path.insert(0,str(package))
    from bhl_hand_sdk.hand_tracking_gui import TrackingGui
    from bhl_hand_sdk.index_trial import trial_goal
    root=tk.Tk();app=TrackingGui(root,project,package/'config/dexhand_camera.yaml',live=False,demo=False)
    root.title('CAN 미연결 코드 진단 · 실제 카메라 + 가상 제어 · 실제 CAN 송신 차단')
    errors=[];started=time.monotonic();camera_started=False;armed=False;reference=None
    def error(kind,value,tb):
        import traceback
        message=''.join(traceback.format_exception(kind,value,tb));errors.append(message);print(message,flush=True)
    root.report_callback_exception=error;signal.signal(signal.SIGTERM,lambda *_:root.after(0,app.close))
    app.camera_device.set('/dev/video0');app.preview_file=BASE/'no_can_data/preview.json'
    root.lift();root.attributes('-topmost',True);root.after(1800,lambda:root.attributes('-topmost',False))
    def tick():
        nonlocal camera_started,armed,reference
        elapsed=time.monotonic()-started
        if elapsed>=10 and not camera_started:camera_started=True;app.start_camera()
        if app.connected and not armed and not app.pending:armed=True;app.arm(True)
        if app.active and app.features is not None and app.latest and time.monotonic()-app.latest['captured_at']<.25:
            if reference is None:reference=app.features
            app.worker.move_pose(trial_goal(app.features,reference),{},app.latest['captured_at'],streaming=False)
        app.storage_note.set('진단 모드 · 실제 CAN 소켓 생성 차단 · 가상 제어만 실행 · 자료는 RAM에 기록')
        latest=app.latest or {}
        atomic(BASE/'no_can_status.json',dict(utc=time.time(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),pid=os.getpid(),elapsed_s=elapsed,visible=bool(root.winfo_viewable()),live_output=app.live,physical_motor_output=False,simulated_connected=app.connected,simulated_controller_active=app.active,phase='camera_and_simulated_controller' if camera_started else 'gui_and_simulated_controller',camera_worker_alive=bool(app.camera and app.camera[0].is_alive()),frames=app.frames_received,previews=app.previews_received,frame_age_s=time.monotonic()-latest['captured_at'] if latest else None,detected=latest.get('detection_valid'),callback_errors=errors,trace_error=app.trace.error,storage_error=app.storage_error,blocked_can_socket_attempts=BLOCKED,camera_problem=app.camera_problem))
        root.after(250,tick)
    root.after(200,app.connect);root.after(300,tick);root.mainloop()

if __name__=='__main__':main()

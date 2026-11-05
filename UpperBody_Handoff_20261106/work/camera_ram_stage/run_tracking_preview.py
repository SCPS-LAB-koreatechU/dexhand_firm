"""Human four-finger target preview. CAN socket creation is explicitly blocked."""
import sys,socket,time,json,signal
from pathlib import Path


def block_can(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:
        raise PermissionError('Preview process cannot open CAN sockets')


sys.addaudithook(block_can)
BASE=Path('/run/user/1000/bhl_hand_camera_session')
sys.path.insert(0,str(BASE/'project/humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk'))
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.hand_retarget import extract_features
from bhl_hand_sdk.four_finger_trial import fresh_sample,trial_goal,validate_references


def main():
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    config=load_config(BASE/'project/humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand_camera.yaml')
    saved=json.loads((BASE/'data/calibration/four_finger_camera_trial.json').read_text())
    opened,closed=saved['opened'],saved['closed'];validate_references(opened,closed)
    path=BASE/'data/tracking_preview.json';last_log=0.;deadline=time.monotonic()+300
    while not stopped and time.monotonic()<deadline:
        now=time.monotonic();data={}
        try:
            data=json.loads((BASE/'camera_control_sample.json').read_text())
            data['recognized_features']=extract_features(data['landmarks']) if data.get('detection_valid') else None
        except (OSError,ValueError,KeyError,TypeError):pass
        valid=fresh_sample(data,now)
        goal=trial_goal(data['recognized_features'],opened,closed) if valid else None
        status=dict(utc=time.time(),valid=valid,frame_id=data.get('frame_id'),camera_age_s=now-data['captured_at'] if 'captured_at' in data else None,
            source_hand=data.get('source_hand'),goal_fraction=goal,goal_us=config.hand('right').to_us(goal) if goal else None,
            motor_commands_sent=0,physical_output=False,profile_tag=config.hand('right').profile_tag,
            message='비구동 미리보기 · 네 손가락 목표 계산 중 · CAN 송신 0' if valid else '비구동 미리보기 · 오른손 하나를 손목부터 손끝까지 보여주세요 · CAN 송신 0')
        temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(status,ensure_ascii=False));temporary.replace(path)
        if now-last_log>=.2:print(json.dumps(status,ensure_ascii=False),flush=True);last_log=now
        time.sleep(.05)
    status.update(valid=False,goal_fraction=None,goal_us=None,message='비구동 미리보기 종료 · 실물 모터 명령 없음')
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(status,ensure_ascii=False));temporary.replace(path)


if __name__=='__main__':main()

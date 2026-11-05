"""Operator-requested full calibrated PWM range, four-finger camera trial."""
from pathlib import Path
import argparse,os,sys,time,json,statistics,collections,subprocess,signal
BASE=Path('/run/user/1000/bhl_hand_camera_session')
os.environ['TMPDIR']=str(BASE/'tmp');os.environ['BHL_HAND_DATA_DIR']=str(BASE/'data')
sys.path.insert(0,str(BASE/'project/humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk'))
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.controller import HandController
from bhl_hand_sdk.demo_session import DemoSession
from bhl_hand_sdk.hand_retarget import extract_features
from bhl_hand_sdk.four_finger_trial import FINGERS,fresh_sample,pose_matches,validate_references,trial_goal,TrackingContinuity,recalibrate_finger
from bhl_hand_sdk.ui_trace import UiTrace

STOP=BASE/'stop_index_trial';STATE=BASE/'data/index_trial_state.json'
DURATION=45.;SPEED=.25

def report(phase,**fields):
    data=dict(utc=time.time(),phase=phase,active=phase in ('ARMING','ACTIVE'),scope='four_fingers',
        mapping='two human poses; three tendons per finger coupled; not independent DIP calibration',**fields)
    p=STATE.with_suffix('.tmp');p.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False));p.replace(STATE)
    print(json.dumps({'event':'state',**data},ensure_ascii=False),flush=True)

def sample():
    try:
        data=json.loads((BASE/'camera_control_sample.json').read_text())
        data['recognized_features']=extract_features(data['landmarks']) if data.get('detection_valid') else None
        return data
    except (OSError,ValueError,KeyError,TypeError):return {}

def link():
    data=json.loads(subprocess.check_output(['ip','-j','-details','-statistics','link','show','can1'],text=True,timeout=1))[0]
    return data['linkinfo']['info_data']['state']

def collect(phase,message,closed=False,groups=FINGERS):
    report(phase,message=message)
    frames=collections.deque();deadline=time.monotonic()+90;last_id=-1
    while time.monotonic()<deadline:
        if STOP.exists():raise RuntimeError('사용자 HOLD')
        data=sample();now=time.monotonic()
        if fresh_sample(data,now) and pose_matches(data['recognized_features'],closed,groups):
            if data['frame_id']!=last_id:
                last_id=data['frame_id'];frames.append((data['captured_at'],data['recognized_features']))
                while frames and now-frames[0][0]>1.5:frames.popleft()
                if (len(frames)>=6 and frames[-1][0]-frames[0][0]>=.7
                        and all(statistics.pstdev(f[g][j] for _,f in frames)<.075 for g in groups for j in (0,1))):
                    return {g:[statistics.mean(f[g][j] for _,f in frames) for j in range(4)] for g in FINGERS}
        else:frames.clear()
        time.sleep(.02)
    raise RuntimeError(message+' · 인식 대기 만료, 이번 구동은 시작하지 않았습니다')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse-calibration',action='store_true',help='Reuse the saved human open/fist references, then require a fresh open hand')
    parser.add_argument('--recalibrate',choices=FINGERS,help='Replace only this finger human references; keep PWM endpoints unchanged')
    parser.add_argument('--calibration-only',action='store_true',help='Save human references without opening CAN or moving any motors')
    parser.add_argument('--fingers',nargs='+',choices=FINGERS,default=list(FINGERS),help='Only these fingers track; other fingers stay open')
    parser.add_argument('--duration',type=float,default=DURATION,help='Finite trial length in seconds, 1..120')
    args=parser.parse_args()
    if not 1<=args.duration<=120:parser.error('--duration must be 1..120 seconds')
    duration=args.duration;tracked=tuple(dict.fromkeys(args.fingers))
    tracked_label='·'.join({'index':'검지','middle':'중지','ring':'약지','pinky':'소지'}[g] for g in tracked)
    STOP.unlink(missing_ok=True);trace=UiTrace();session=None;hold_result=None
    signal.signal(signal.SIGTERM,lambda *_:STOP.touch());signal.signal(signal.SIGINT,lambda *_:STOP.touch())
    try:
        if sample().get('input_role')=='robot_observation':raise RuntimeError('카메라가 로봇 관찰 모드입니다. 사람 손 추종/보정 입력으로 사용하지 않습니다.')
        if not args.calibration_only and link()!='ERROR-ACTIVE':raise RuntimeError('CAN 상태 오류 · 시작하지 않음')
        calibration_path=BASE/'data/calibration/four_finger_camera_trial.json'
        if args.reuse_calibration or args.recalibrate:
            saved=json.loads(calibration_path.read_text())
            opened,closed=saved['opened'],saved['closed']
            if args.recalibrate:
                group=args.recalibrate;label={'index':'검지','middle':'중지','ring':'약지','pinky':'소지'}[group]
                new_open=collect('WAIT_OPEN',label+' 기준 ① · 사람 오른손을 펴고 1초 유지하세요 · 모터 구동 없음',groups=(group,))
                new_closed=collect('WAIT_FIST',label+' 기준 ② · 사람 '+label+'를 충분히 접고 1초 유지하세요 · 모터 구동 없음',True,groups=(group,))
                opened,closed=recalibrate_finger(opened,closed,new_open,new_closed,group)
        else:
            opened=collect('WAIT_OPEN','① 오른손 하나를 펴서 1초 멈추세요 · 사람 손 기준만 측정 중')
            closed=collect('WAIT_FIST','② 네 손가락으로 주먹을 쥐고 1초 멈추세요 · 아직 이번 구동은 시작하지 않음',True)
        validate_references(opened,closed)
        calibration=dict(opened=opened,closed=closed,max_fraction=1.,speed_fraction_s=SPEED,
            mapping='three tendons share each finger curl',robot_pose_calibration_verified=False,recalibrated_finger=args.recalibrate,recorded_utc=time.time())
        if args.recalibrate or not args.reuse_calibration:
            if calibration_path.exists():
                backup=calibration_path.with_name(calibration_path.stem+'.before_'+str(time.time_ns())+'.json')
                backup.write_text(calibration_path.read_text())
            temp=calibration_path.with_suffix('.tmp');temp.write_text(json.dumps(calibration,ensure_ascii=False,indent=2));temp.replace(calibration_path)
        trace.submit({'four_finger_human_references':calibration,'references_reused':args.reuse_calibration,'tracked_fingers':tracked,'duration_s':duration})
        if args.calibration_only:
            report('CALIBRATED',message='사람 손 기준 저장 완료 · 모터 구동 명령 없음 · 기존 PWM 범위 유지',recalibrated_finger=args.recalibrate)
            return
        collect('WAIT_RETURN_OPEN',f'오른손을 펴서 1초 유지하면 {tracked_label} {duration:g}초 추종 시작 · 나머지 손가락 펼침 유지')
        if link()!='ERROR-ACTIVE':raise RuntimeError('CAN 상태 오류 · 시작하지 않음')
        cfg=load_config(BASE/'project/humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand_camera.yaml')
        cfg.hand('right').channel='can1';cfg.max_speed=SPEED;cfg.command_timeout=.3
        continuity=TrackingContinuity()
        if continuity.update(sample(),time.monotonic())[0]!='TRACK':raise RuntimeError('시작 직전 손 인식 없음 · 구동하지 않음')
        hand=HandController(cfg,'right',dry_run=False);session=DemoSession(hand)
        report('ARMING',message='전체 펼침 기준 적용 · '+tracked_label+' 추종 시작 · 나머지 손가락 펼침 유지',tracked_fingers=tracked)
        session.arm(initial_open=True)
        start=last=time.monotonic();last_check=last_report=last
        report('ACTIVE',message=tracked_label+'를 천천히 접었다 펴세요 · 범위 100% · Esc/HOLD 중단',remaining_s=duration,tracked_fingers=tracked)
        while time.monotonic()-start<duration:
            now=time.monotonic()
            if STOP.exists():raise RuntimeError('사용자 HOLD')
            data=sample()
            tracking_mode,tracking_reason=continuity.update(data,now)
            if tracking_mode=='HOLD':
                evidence={k:v for k,v in data.items() if k!='landmarks'}
                evidence['age_at_trial_s']=now-data.get('captured_at',now)
                trace.submit({'invalid_camera_input':evidence});print(json.dumps({'event':'invalid_camera_input',**evidence}),flush=True)
                raise RuntimeError(tracking_reason+' · HOLD (현재 명령 유지)')
            goal=(list(hand.command) if tracking_mode=='FREEZE' else trial_goal(data['recognized_features'],opened,closed,groups=tracked))
            hand.set_goal(goal,streaming=True)
            delivery=session.step(max(1e-6,min(.05,now-last)));last=now
            entry=dict(event='command',frame_id=data.get('frame_id'),captured_at=data.get('captured_at'),
                camera_age_s=now-data['captured_at'] if 'captured_at' in data else None,features=data.get('recognized_features'),tracking_mode=tracking_mode,
                goal_fraction=goal,goal_us=hand.spec.to_us(goal),command_us=hand.command_us,
                delivery=delivery.as_dict(),physical_measured=False)
            trace.submit(entry);print(json.dumps(entry),flush=True)
            if now-last_check>.2:
                if link()!='ERROR-ACTIVE':raise RuntimeError('CAN 오류 · HOLD')
                last_check=now
            if now-last_report>.15:
                report('ACTIVE',message=('일시 인식 누락 · 새 동작 정지/현재 명령 유지 (최대 0.3초)' if tracking_mode=='FREEZE' else tracked_label+'를 천천히 움직이세요 · 나머지 손가락 펼침 유지 · Esc/HOLD 중단'),tracking_mode=tracking_mode,tracked_fingers=tracked,
                    remaining_s=round(duration-(now-start),1),goal_fraction=goal,goal_us=hand.spec.to_us(goal),command_us=hand.command_us,delivery=delivery.result)
                last_report=now
            time.sleep(max(0.,.02-(time.monotonic()-now)))
        hold_result=session.stop(f'{duration:g}초 {tracked_label} 시험 종료')
        report('COMPLETE',message=f'{duration:g}초 {tracked_label} 시험 종료 · 추종 중지/마지막 PWM HOLD · 실제 움직임 확인 필요',tracked_fingers=tracked,
            command_us=hand.command_us,hold_delivery=hold_result.as_dict() if hold_result else None)
    except Exception as error:
        if session:
            try:hold_result=session.stop(str(error))
            except Exception as hold_error:trace.submit({'hold_error':str(hold_error)})
            try:failed=link()!='ERROR-ACTIVE'
            except Exception:failed=True
            if failed:subprocess.run(['sudo','-n','ip','link','set','can1','down'],capture_output=True,text=True)
        report('HOLD' if session else 'NOT_STARTED',message=str(error),command_us=session.hand.command_us if session else None,
            hold_delivery=hold_result.as_dict() if hold_result else None)
    finally:
        if session:
            trace.submit({'four_finger_session_summary':session.summary()});session.close()
        trace.close()

if __name__=='__main__':main()

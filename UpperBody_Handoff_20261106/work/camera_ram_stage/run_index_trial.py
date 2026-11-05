"""Explicit, finite, low-range physical trial requested by the operator."""
from pathlib import Path
import os,sys,time,json,statistics,collections,subprocess,signal
BASE=Path('/run/user/1000/bhl_hand_camera_session')
os.environ['TMPDIR']=str(BASE/'tmp');os.environ['BHL_HAND_DATA_DIR']=str(BASE/'data')
sys.path.insert(0,str(BASE/'project/humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk'))
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.controller import HandController
from bhl_hand_sdk.demo_session import DemoSession
from bhl_hand_sdk.index_trial import trial_goal,fresh_index_sample,index_is_open
from bhl_hand_sdk.ui_trace import UiTrace

STOP=BASE/'stop_index_trial';STATE=BASE/'data/index_trial_state.json'

def report(phase,**fields):
    d=dict(utc=time.time(),phase=phase,active=phase in ('ARMING','ACTIVE'),mapping='uncalibrated 15% index-only trial',**fields)
    p=STATE.with_suffix('.tmp');p.write_text(json.dumps(d,ensure_ascii=False));p.replace(STATE)
    print(json.dumps({'event':'state',**d},ensure_ascii=False),flush=True)

def sample():
    return json.loads((BASE/'status.json').read_text())

def link():
    d=json.loads(subprocess.check_output(['ip','-j','-details','-statistics','link','show','can1'],text=True,timeout=1))[0]
    return d['linkinfo']['info_data']['state']

def main():
    STOP.unlink(missing_ok=True);trace=UiTrace();session=None
    def stop(*_):STOP.touch()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        assert link()=='ERROR-ACTIVE','CAN is not error-active'
        report('WAIT_OPEN',message='오른손 하나의 검지를 펴고 1초 유지하세요 · 아직 모터 출력 꺼짐')
        frames=collections.deque();deadline=time.monotonic()+60;last_id=-1;reference=None
        while time.monotonic()<deadline:
            if STOP.exists():raise RuntimeError('OPERATOR_HOLD')
            d=sample();now=time.monotonic()
            if fresh_index_sample(d,now) and index_is_open(d['recognized_features']):
                ident=d.get('frame_id')
                if ident!=last_id:
                    last_id=ident;frames.append((d['captured_at'],d['recognized_features']))
                    while frames and now-frames[0][0]>1.2:frames.popleft()
                    if len(frames)>=7 and frames[-1][0]-frames[0][0]>=.7 and all(statistics.pstdev(f['index'][j] for _,f in frames)<.035 for j in range(3)):
                        reference={k:[statistics.mean(f[k][j] for _,f in frames) for j in range(len(frames[0][1][k]))] for k in frames[0][1]};break
            else:frames.clear()
            time.sleep(.02)
        if reference is None:raise RuntimeError('편 오른손 인식 대기 만료 · 모터는 시작하지 않았습니다')
        assert link()=='ERROR-ACTIVE','CAN failed before enabling'
        cfg=load_config(BASE/'project/humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand_camera.yaml')
        cfg.hand('right').channel='can1';cfg.max_speed=.10;cfg.command_timeout=.3
        hand=HandController(cfg,'right',dry_run=False);session=DemoSession(hand)
        trace.submit({'index_trial_reference':reference,'max_pull_fraction':.15,'max_speed_fraction_s':.10,'initial_pose':'explicit whole-hand open','physical_measured':False})
        report('ARMING',message='전체 펼침 기준 적용 중 · 검지만 시험합니다')
        session.arm(initial_open=True)
        report('ACTIVE',message='실물 검지 시험 · 최대 15% · 검지를 천천히 접었다 펴세요 · Esc/HOLD로 중단',goal_us=hand.command_us,command_us=hand.command_us)
        start=last=time.monotonic();last_check=last;last_report=last
        while time.monotonic()-start<20:
            now=time.monotonic()
            if STOP.exists():raise RuntimeError('OPERATOR_HOLD')
            d=sample()
            if not fresh_index_sample(d,now):
                evidence={k:d.get(k) for k in ('frame_id','captured_at','detection_valid','source_hand','handedness_score','recognized_features','frame_age_s','camera_note')}
                evidence['age_at_trial_s']=now-d.get('captured_at',now)
                trace.submit({'invalid_camera_input':evidence});print(json.dumps({'event':'invalid_camera_input',**evidence},ensure_ascii=False),flush=True)
                raise RuntimeError('손 인식 상실/잘못된 손/영상 만료로 HOLD')
            goal=trial_goal(d['recognized_features'],reference)
            hand.set_goal(goal,streaming=True)
            delivery=session.step(max(1e-6,min(.05,now-last)));last=now
            entry={'index_trial_frame':d['frame_id'],'captured_at':d['captured_at'],'goal_us':hand.spec.to_us(goal),'command_us':hand.command_us,'delivery':delivery.as_dict(),'physical_measured':False}
            trace.submit(entry);print(json.dumps({'event':'command',**entry}),flush=True)
            if now-last_check>.2:
                if link()!='ERROR-ACTIVE':raise RuntimeError('CAN 오류로 HOLD')
                last_check=now
            if now-last_report>.1:
                report('ACTIVE',message='검지를 천천히 접었다 펴세요 · Esc/HOLD로 중단',remaining_s=round(20-(now-start),1),goal_us=hand.spec.to_us(goal),command_us=hand.command_us,delivery=delivery.result)
                last_report=now
            time.sleep(max(0.,.02-(time.monotonic()-now)))
        session.stop('20초 검지 시험 종료');report('COMPLETE',message='20초 검지 시험 종료 · 현재 명령값 HOLD · 실제 움직임은 사용자 확인 필요',command_us=hand.command_us)
    except Exception as e:
        if session:
            try:session.stop(str(e))
            except Exception as hold_error:trace.submit({'hold_error':str(hold_error)})
            try:failed_link=link()!='ERROR-ACTIVE'
            except Exception:failed_link=True
            if failed_link:
                subprocess.run(['sudo','-n','ip','link','set','can1','down'],capture_output=True,text=True)
                trace.submit({'cancelled_failed_bus_tx_queue':True})
        report('HOLD' if session else 'NOT_STARTED',message=str(e),command_us=session.hand.command_us if session else None)
        trace.submit({'index_trial_stop':str(e),'motor_enable_attempted':bool(session and session.attempted_enable)})
    finally:
        if session:session.close()
        trace.close()

if __name__=='__main__':main()

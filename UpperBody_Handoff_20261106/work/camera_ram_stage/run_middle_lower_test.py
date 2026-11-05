"""Finite diagnostic of the existing GPIO22 range; no endpoint increases."""
from run_four_finger_trial import BASE,STOP,report,link,load_config,HandController,DemoSession,UiTrace
from bhl_hand_sdk.four_finger_trial import middle_lower_test_goal
from bhl_hand_sdk.observation_capture import ObservationRecorder
import argparse,json,time,signal,subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--countdown',type=int,default=8)
    parser.add_argument('--time-scale',type=float,default=1.)
    args=parser.parse_args()
    if not 0<=args.countdown<=30:parser.error('--countdown must be 0..30')
    if not 1<=args.time_scale<=3:parser.error('--time-scale must be 1..3 (slower only)')
    duration=14.*args.time_scale
    STOP.unlink(missing_ok=True);session=None;trace=UiTrace();held=None
    recorder=ObservationRecorder(BASE/'data/observations'/('middle_lower_'+str(time.time_ns())))
    signal.signal(signal.SIGTERM,lambda *_:STOP.touch());signal.signal(signal.SIGINT,lambda *_:STOP.touch())
    try:
        observation=json.loads((BASE/'latest_observation.json').read_text())
        if not 0<=time.monotonic()-observation['captured_at']<=.8:raise RuntimeError('관찰 영상 만료 · 구동하지 않음')
        if link()!='ERROR-ACTIVE':raise RuntimeError('CAN 오류 · 구동하지 않음')
        countdown_end=time.monotonic()+args.countdown;shown=-1
        while time.monotonic()<countdown_end:
            if STOP.exists():raise RuntimeError('사용자 HOLD · 시작하지 않음')
            remaining=int(countdown_end-time.monotonic()+.999)
            if remaining!=shown:
                report('PREPARING',message=f'{remaining}초 후 중지 아래만 시험 · 로봇 손과 ZED 화면을 봐 주세요 · Esc/HOLD 취소',countdown_s=remaining)
                shown=remaining
            time.sleep(.05)
        observation=json.loads((BASE/'latest_observation.json').read_text())
        if not 0<=time.monotonic()-observation['captured_at']<=.8:raise RuntimeError('관찰 영상 만료 · 구동하지 않음')
        if link()!='ERROR-ACTIVE':raise RuntimeError('CAN 오류 · 구동하지 않음')
        cfg=load_config(BASE/'project/humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand_camera.yaml')
        cfg.hand('right').channel='can1';cfg.max_speed=.25;cfg.command_timeout=.3
        hand=HandController(cfg,'right',dry_run=False);session=DemoSession(hand)
        report('ARMING',message='중지 아래 GPIO22 분리 시험 · 전체 펼침 기준 · 카메라 추종 아님')
        session.arm(initial_open=True)
        start=last=last_report=last_check=time.monotonic();last_image=0.
        recorder.command(start,hand.command_us)
        while time.monotonic()-start<duration:
            now=time.monotonic();elapsed=now-start
            if STOP.exists():raise RuntimeError('사용자 HOLD')
            phase_time=elapsed/args.time_scale
            goal=middle_lower_test_goal(phase_time);hand.set_goal(goal,streaming=True)
            delivery=session.step(max(1e-6,min(.05,now-last)));last=now
            recorder.command(time.monotonic(),hand.command_us)
            if now-last_image>.1:
                image=recorder.capture(BASE/'latest_observation.json')
                if image:
                    trace.submit(image);print(json.dumps(image),flush=True)
                last_image=now
            entry=dict(event='command',elapsed_s=elapsed,goal_fraction=goal,goal_us=hand.spec.to_us(goal),command_us=hand.command_us,delivery=delivery.as_dict(),physical_measured=False)
            trace.submit(entry);print(json.dumps(entry),flush=True)
            if now-last_check>.2:
                if link()!='ERROR-ACTIVE':raise RuntimeError('CAN 오류 · HOLD')
                last_check=now
            if now-last_report>.1:
                phase='펼침 기준 대기' if phase_time<2 else '아래만 천천히 당기는 중' if phase_time<6 else '최대 당김 유지 · 옆 손가락과 닿는지 보세요' if phase_time<8 else '아래를 천천히 푸는 중' if phase_time<12 else '펼침 복귀 대기'
                report('ACTIVE',message='중지 아래 단독 시험 · '+phase+' · Esc/HOLD 중단',remaining_s=round(duration-elapsed,1),command_us=hand.command_us,goal_us=hand.spec.to_us(goal),tracked_channels=[6],time_scale=args.time_scale)
                last_report=now
            time.sleep(max(0.,.02-(time.monotonic()-now)))
        held=session.stop(f'{duration:g}초 중지 아래 분리 시험 종료')
        report('COMPLETE',message='중지 아래 단독 시험 종료 · 펼침 명령값 HOLD · 실물 관찰 필요',command_us=hand.command_us,hold_delivery=held.as_dict() if held else None)
    except Exception as error:
        if session:
            try:held=session.stop(str(error))
            except Exception as hold_error:trace.submit({'hold_error':str(hold_error)})
            try:failed=link()!='ERROR-ACTIVE'
            except Exception:failed=True
            if failed:subprocess.run(['sudo','-n','ip','link','set','can1','down'],capture_output=True,text=True)
        report('HOLD' if session else 'NOT_STARTED',message=str(error),hold_delivery=held.as_dict() if held else None)
    finally:
        if session:trace.submit({'middle_lower_session':session.summary()});session.close()
        trace.close()

if __name__=='__main__':main()

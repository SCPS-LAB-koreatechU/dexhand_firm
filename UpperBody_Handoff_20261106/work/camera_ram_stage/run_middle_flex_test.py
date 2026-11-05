"""Existing-firmware middle flexion only; coordinated tendons, no automatic extension."""
from run_four_finger_trial import BASE,STOP,report,link,load_config,HandController,DemoSession,UiTrace
from bhl_hand_sdk.observation_capture import ObservationRecorder
import argparse,json,time,signal,subprocess


def middle_goal(elapsed):
    fraction=max(0.,min(1.,(elapsed-2.)/8.))
    goal=[0.]*15
    for i in (4,5,9):goal[i]=fraction
    return goal


def validate_flex_profile(base,spec,lower):
    if lower not in (1750,1850):raise RuntimeError('지원하지 않는 중지 아래 목표')
    if base.profile_tag not in ('79610de6','bdad07a4') or base.close_us[5]!=1750:
        raise RuntimeError('기준 설정과 다름 · 구동하지 않음')
    if lower==1850 and (base.profile_tag!='bdad07a4' or getattr(base,'hardware_max_us',None)!=2300):
        raise RuntimeError('1850us는 새 수신 상한 펌웨어 업로드와 SDK 적용 후 사용합니다. 기존 설정에서는 보내지 않습니다.')
    if spec.node!=61 or spec.profile_tag!=base.profile_tag or spec.open_us!=base.open_us:
        raise RuntimeError('기준 핀/프로필과 다른 운용 설정')
    if [spec.close_us[i] for i in (4,5,9)]!=[825,lower,2000]:raise RuntimeError('중지 목표 설정과 다름')
    if any(a!=b for i,(a,b) in enumerate(zip(spec.close_us,base.close_us)) if i!=5):
        raise RuntimeError('다른 모터의 목표가 변경됨')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--middle-lower',type=int,choices=(1750,1850),default=1750)
    args=parser.parse_args()
    STOP.unlink(missing_ok=True);session=None;trace=UiTrace();held=None
    recorder=ObservationRecorder(BASE/'data/observations'/('middle_flex_'+str(time.time_ns())))
    signal.signal(signal.SIGTERM,lambda *_:STOP.touch());signal.signal(signal.SIGINT,lambda *_:STOP.touch())
    def camera_ready():
        data=json.loads((BASE/'latest_observation.json').read_text())
        return 0<=time.monotonic()-data['captured_at']<=.8
    try:
        if not camera_ready() or link()!='ERROR-ACTIVE':raise RuntimeError('영상/CAN 상태 오류 · 구동하지 않음')
        folder=BASE/'project/humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config'
        base=load_config(folder/'dexhand_camera.yaml')
        cfg=load_config(folder/'dexhand_camera_middle1850.yaml') if args.middle_lower==1850 else base
        spec=cfg.hand('right')
        validate_flex_profile(base.hand('right'),spec,args.middle_lower)
        end=time.monotonic()+8;shown=-1
        while time.monotonic()<end:
            if STOP.exists():raise RuntimeError('사용자 HOLD · 시작하지 않음')
            remaining=int(end-time.monotonic()+.999)
            if remaining!=shown:
                report('PREPARING',message=f'{remaining}초 후 중지 세 모터 함께 굽힘 · 기존 범위 · 자동 펼침 없음 · Esc/HOLD 취소',countdown_s=remaining)
                shown=remaining
            time.sleep(.05)
        if not camera_ready() or link()!='ERROR-ACTIVE':raise RuntimeError('영상/CAN 상태 오류 · 구동하지 않음')
        cfg.hand('right').channel='can1';cfg.max_speed=.125;cfg.command_timeout=.3
        hand=HandController(cfg,'right',dry_run=False);session=DemoSession(hand)
        report('ARMING',message=f'중지 굽힘 시험 · 아래 목표 {args.middle_lower}us · 다른 손가락 펼침 유지')
        session.arm(initial_open=True)
        start=last=last_report=last_check=time.monotonic();last_image=0.
        recorder.command(start,hand.command_us)
        while time.monotonic()-start<13.:
            now=time.monotonic();elapsed=now-start
            if STOP.exists():raise RuntimeError('사용자 HOLD')
            goal=middle_goal(elapsed);hand.set_goal(goal,streaming=True)
            delivery=session.step(max(1e-6,min(.05,now-last)));last=now
            recorder.command(time.monotonic(),hand.command_us)
            if now-last_image>.1:
                image=recorder.capture(BASE/'latest_observation.json')
                if image:trace.submit(image);print(json.dumps(image),flush=True)
                last_image=now
            entry=dict(event='command',elapsed_s=elapsed,goal_fraction=goal,goal_us=hand.spec.to_us(goal),command_us=hand.command_us,delivery=delivery.as_dict(),physical_measured=False)
            trace.submit(entry);print(json.dumps(entry),flush=True)
            if now-last_check>.2:
                if link()!='ERROR-ACTIVE':raise RuntimeError('CAN 오류 · HOLD')
                last_check=now
            if now-last_report>.1:
                phase='펼침 기준 대기' if elapsed<2 else '8초에 걸쳐 위·아래·TIP 함께 굽히는 중' if elapsed<10 else '접힘 명령 도달 · 모양을 확인하세요'
                report('ACTIVE',message='중지 굽힘 시험 · '+phase+' · 자동 펼침 없음 · Esc/HOLD 중단',remaining_s=round(13-elapsed,1),command_us=hand.command_us,goal_us=hand.spec.to_us(goal),tracked_channels=[5,6,10],automatic_extension=False)
                last_report=now
            time.sleep(max(0.,.02-(time.monotonic()-now)))
        held=session.stop('중지 굽힘 시험 종료 · 자동 펼침 없음')
        report('COMPLETE',message='중지 굽힘 시험 종료 · 접힘 명령값 HOLD · 자동 펼침 없음 · PWM 유지 중',command_us=hand.command_us,hold_delivery=held.as_dict() if held else None,automatic_extension=False)
    except Exception as error:
        if session:
            try:held=session.stop(str(error))
            except Exception as hold_error:trace.submit({'hold_error':str(hold_error)})
            try:failed=link()!='ERROR-ACTIVE'
            except Exception:failed=True
            if failed:subprocess.run(['sudo','-n','ip','link','set','can1','down'],capture_output=True,text=True)
        report('HOLD' if session else 'NOT_STARTED',message=str(error),hold_delivery=held.as_dict() if held else None)
    finally:
        if session:trace.submit({'middle_flex_session':session.summary()});session.close()
        trace.close()

if __name__=='__main__':main()

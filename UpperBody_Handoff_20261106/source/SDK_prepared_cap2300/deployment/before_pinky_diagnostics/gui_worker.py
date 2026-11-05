"""Single CAN owner thread for the Tk GUI; vision stays in a spawned process."""
import importlib.util
import multiprocessing as mp
from pathlib import Path
import queue
import threading
import time

from .config import load_config
from .controller import HandController
from .demo_session import DemoSession
from .gui_model import GuiControl
from .camera_source import camera_worker, offer_latest


class GuiWorker:
    def __init__(self, config_path, *, factory=HandController):
        self.config_path, self.factory = config_path, factory
        self.commands = queue.Queue(maxsize=32)
        self.events = queue.SimpleQueue()
        self.images = queue.Queue(maxsize=1)
        self.quit = threading.Event()
        self.hold_requested = threading.Event()
        self.lock = threading.Lock()
        self.pending_motors = {}
        self.generation = 0
        self.link_stale_reported = False
        self.ui_seen = time.monotonic()
        self.control = None
        self.vision = None
        self.shutdown_error = None
        self.stage = "idle"
        self.thread = threading.Thread(target=self._run, name='DexHand-CAN', daemon=True)
        self.thread.start()

    def _cancel_pending(self):
        with self.lock:
            self.generation += 1
            self.pending_motors.clear()
            while True:
                try:self.commands.get_nowait()
                except queue.Empty:break

    def post(self, action, **values):
        if action in ('hold','connect','disconnect'):
            self._cancel_pending()
        if action == 'hold':
            self.hold_requested.set()
            return
        self.commands.put_nowait(dict(action=action, stamp=time.monotonic(), generation=self.generation, **values))

    def move(self, index, pulse):
        with self.lock:
            self.pending_motors[index] = (pulse, time.monotonic(), self.generation)

    def _clear_motion(self):
        with self.lock:
            self.pending_motors.clear()

    def _camera_stop(self):
        if self.vision is None:
            return
        worker, stop, output, preview = self.vision
        self.vision = None
        stop.set()
        worker.join(timeout=.5)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=.5)
        for pipe in (output, preview):
            pipe.close()
            pipe.cancel_join_thread()
        while True:
            try:
                self.images.get_nowait()
            except queue.Empty:
                break

    def _camera_start(self, camera, model):
        if self.control.mode != 'camera':
            raise ValueError('카메라 모드를 선택하세요')
        if not Path(model).is_file() or any(importlib.util.find_spec(m) is None for m in ('cv2', 'mediapipe')):
            raise ValueError('카메라 설치 필요: bash setup_hand_test.sh --camera')
        self.control.hold('카메라 미리보기 · 시작 버튼 대기')
        self._camera_stop()
        # A restarted worker starts its frame sequence from zero.
        self.control.camera = type(self.control.camera)(self.control.session, clock=time.monotonic, log=lambda _: None)
        context = mp.get_context('spawn')
        output, preview, stop = context.Queue(maxsize=1), context.Queue(maxsize=1), context.Event()
        worker = context.Process(target=camera_worker,
            kwargs=dict(output=output, stop=stop, camera=camera, model=model,
                        headless=True, auto_arm=False, preview=preview), daemon=True)
        worker.start()
        self.vision = worker, stop, output, preview

    def _disconnect(self):
        try:
            if self.control:
                session = self.control.session
                session.close()
                if session.last_hold is not None and not session.last_hold.ok:
                    raise RuntimeError('종료 HOLD 응답 없음 · 실제 PWM 유지 상태는 확인되지 않았습니다')
        finally:
            self.control = None
            self._camera_stop()
            self._clear_motion()

    def _execute(self, command):
        generation = command.get('generation',self.generation)
        if generation != self.generation:return
        action = command['action']
        self.stage = action
        if action == 'disconnect':
            try:
                self._disconnect()
            finally:
                self.events.put(dict(kind='disconnected'))
            return
        if action == 'connect':
            try:self._disconnect()
            except Exception as error:
                # The old socket/lease is closed in _disconnect's finally. A failed
                # HOLD must be reported, but must not trap a fresh read-only connect.
                self.events.put(dict(kind='warning',message='기존 연결 HOLD 미확인: '+str(error)))
            config = load_config(self.config_path)
            config.max_speed = min(config.max_speed, .25)
            config.reply_timeout = max(.05, config.reply_timeout)  # measured APPLIED delays 21-26 ms on this USB CAN
            config.ping_timeout = max(.2, config.ping_timeout)
            config.hand(command['hand']).channel = command['channel']
            hand = self.factory(config, command['hand'], dry_run=command['dry_run'])
            session = DemoSession(hand)
            try:
                pong = session.ping()
            except BaseException:
                session.close()
                raise
            if generation != self.generation:
                session.close();return
            self.link_stale_reported = False
            self.control = GuiControl(session, clock=time.monotonic)
            self.events.put(dict(kind='connected', hand=hand.spec.name, node=pong.node,
                                 tag=pong.profile_tag, dry_run=hand.dry_run))
            return
        if action == 'check' and self.control:
            if self.control.session.active:
                raise ValueError('동작 중 통신 점검은 HOLD 후 실행하세요')
            pong = self.control.session.ping()
            hand = self.control.session.hand
            self.events.put(dict(kind='checked', node=pong.node, version=pong.version,
                                 tag=pong.profile_tag, dry_run=hand.dry_run))
            return
        if self.control is None:
            raise ValueError('먼저 CAN 연결 확인을 누르세요')
        if action == 'mode':
            self._clear_motion()
            self.control.switch_mode(command['mode'])
            self._camera_stop()
        elif action == 'arm':
            if time.monotonic()-command['stamp'] > .35:
                raise ValueError('시작 요청이 만료됨 · 다시 누르세요')
            self.control.arm(command['initial_open'])
            self._clear_motion()
            if generation != self.generation:
                self.control.hold('시작 중 요청 취소');return
            self.events.put(dict(kind='armed',request_id=command.get('request_id')))
        elif action == 'open_all':
            if time.monotonic()-command['stamp']>.35:raise ValueError('펼침 요청이 만료됐습니다. 다시 눌러주세요')
            if not self.control.session.active or self.control.mode != 'manual':
                raise ValueError('전체 펼침은 출력 시작 후 개별 모터 탭에서 사용하세요')
            self._clear_motion()
            self.control.session.hand.set_grip('open')
            self.control.reason = '출력 중 · 전체 펼침 목표'
            self.events.put(dict(kind='open_goal_set', request_id=command.get('request_id')))
        elif action == 'off':
            self.control.hold('PWM 끄기 요청')
            result = self.control.session.hand.off()
            if not result.ok:
                raise RuntimeError('OFF 응답 없음: ' + result.result)
            self.control.reason = 'PWM 꺼짐 · 유지력 해제'
        elif action == 'camera_start':
            self._camera_start(command['camera'], command['model'])
        elif action == 'camera_stop':
            self.control.hold('카메라 종료')
            self._camera_stop()
        else:
            raise ValueError('unknown GUI command')

    def _sample(self):
        if self.vision is None:
            return None
        worker, stop, output, preview = self.vision
        sample = None
        for _ in range(4):
            try:
                sample = output.get_nowait()
            except queue.Empty:
                break
        for _ in range(2):
            try:
                offer_latest(self.images, preview.get_nowait())
            except queue.Empty:
                break
        if sample and sample.get('kind') == 'error':
            raise RuntimeError('카메라: ' + sample.get('message', 'error'))
        if stop.is_set() or not worker.is_alive():
            raise RuntimeError('카메라 중단 · 다시 시작 필요')
        return sample

    def _run(self):
        last, published = time.monotonic(), 0.0
        try:
            while not self.quit.is_set():
                now = time.monotonic()
                cycle_generation = self.generation
                try:
                    if self.hold_requested.is_set():
                        self.hold_requested.clear()
                        self._clear_motion()
                        if self.control:
                            self.control.hold()
                    # One command per cycle limits CAN work delaying a HOLD.
                    try:
                        self._execute(self.commands.get_nowait())
                    except queue.Empty:
                        pass
                    if cycle_generation != self.generation or self.hold_requested.is_set():
                        last=time.monotonic();self.quit.wait(.002);continue
                    if self.control:
                        if self.control.session.active and now-self.ui_seen > .7:
                            self.control.hold('GUI 응답 만료 · 출력을 다시 시작하세요')
                            self._clear_motion()
                        with self.lock:
                            motors, self.pending_motors = self.pending_motors, {}
                        for index, (pulse, stamp, generation) in motors.items():
                            if generation == self.generation and time.monotonic()-stamp <= .35:
                                self.control.motor(index, pulse)
                        self.stage = 'stream'
                        self.control.tick(max(1e-6, now-last), self._sample())
                        if not self.control.session.active and not self.control.session.hand.dry_run:
                            hand = self.control.session.hand
                            hand.poll()
                            if hand.last_heard > 0 and time.monotonic()-hand.last_heard > 2 and not self.link_stale_reported:
                                self.link_stale_reported = True
                                self._cancel_pending()
                                self.events.put(dict(kind='error',stage='CAN/HEARTBEAT',message='2초간 손 응답 없음. CAN 재연결 후 현재 자세 이어받기를 명시적으로 실행하세요.'))
                        if now-published >= .1:
                            hand = self.control.session.hand
                            diagnostics = dict(stage=getattr(hand,'io_stage',self.stage),
                                stats=dict(getattr(hand,'stats',{})),
                                reply_timeout_s=hand.config.reply_timeout,
                                last_response_age_s=None if not getattr(hand,'last_heard',0) else max(0,time.monotonic()-hand.last_heard),
                                firmware_fault=getattr(hand,'fault',''), boot_id=getattr(hand,'boot_id',None))
                            self.events.put(dict(kind='status', diagnostics=diagnostics, **self.control.status()))
                            published = now
                except Exception as error:
                    failed_stage = getattr(error,'io_stage',None) or (getattr(self.control.session.hand,'io_stage',self.stage) if self.control else self.stage)
                    self._cancel_pending()
                    try:
                        if self.control:
                            self.control.hold('오류 · ' + str(error))
                    except Exception as hold_error:
                        self.events.put(dict(kind='error', message=str(hold_error)))
                    if self.vision:
                        self._camera_stop()
                    self.events.put(dict(kind='error', message=str(error), stage=failed_stage, failure_stats=getattr(error,'evidence',{})))
                last = now
                self.quit.wait(max(0, .02-(time.monotonic()-now)))
        finally:
            try:
                self._disconnect()
            except Exception as error:
                self.shutdown_error = str(error)
                self.events.put(dict(kind='error', message='종료 유지 확인 실패: ' + str(error)))
            self.events.put(dict(kind='closed'))

    def close(self):
        self._cancel_pending()
        self.quit.set()

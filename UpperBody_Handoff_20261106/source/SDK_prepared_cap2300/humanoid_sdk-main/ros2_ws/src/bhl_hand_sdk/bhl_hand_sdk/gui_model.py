"""GUI-independent single-motor control and calibration drafts.

Live bounds and profile identity never change during a session. A draft is an
offline deployment artifact, not a calibration command sent to an ESP32.
"""
import copy
import hashlib
import json
from pathlib import Path

from .config import HandSpec
from .demo_session import CameraControl


def profile_tag(profile):
    canonical = json.dumps(profile, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(canonical.encode('utf-8')).hexdigest()[:8]


class CalibrationDraft:
    def __init__(self, profile, spec):
        self.original = copy.deepcopy(profile)
        self.profile = copy.deepcopy(profile)
        if (profile['hand'] != spec.name or profile['device_id'] != spec.node or
                profile_tag(profile) != spec.profile_tag[:8].lower()):
            raise ValueError('GUI profile and SDK config differ; use matching deployed files')
        if ([m['open_us'] for m in profile['motors']] != spec.open_us or
                [m['close_us'] for m in profile['motors']] != spec.close_us):
            raise ValueError('GUI pulse calibration differs from SDK config')

    def record(self, index, endpoint, pulse):
        if type(index) is not int or not 0 <= index < 15 or endpoint not in ('open_us', 'close_us'):
            raise ValueError('invalid motor or endpoint')
        if type(pulse) is not int or not 500 <= pulse <= 2500:
            raise ValueError('PWM must be an integer in 500..2500 us')
        self.profile['motors'][index][endpoint] = pulse

    def validate(self):
        motors = self.profile['motors']
        HandSpec(self.profile['hand'], self.profile['device_id'], 'can0',
                 [m['open_us'] for m in motors], [m['close_us'] for m in motors])

    def save(self, path):
        self.validate()
        Path(path).write_text(json.dumps(self.profile, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

    def load(self, path):
        candidate = json.loads(Path(path).read_text(encoding='utf-8'))
        original_meta, candidate_meta = copy.deepcopy(self.original), copy.deepcopy(candidate)
        for data in (original_meta, candidate_meta):
            for motor in data['motors']:
                motor.pop('open_us')
                motor.pop('close_us')
        if candidate_meta != original_meta:
            raise ValueError('보정 JSON의 손·GPIO·모터 순서가 현재 배선과 다릅니다')
        proposed = copy.deepcopy(self)
        for i, motor in enumerate(candidate['motors']):
            for key in ('open_us', 'close_us'):
                proposed.record(i, key, motor[key])
        proposed.validate()
        self.profile = proposed.profile


class GuiControl:
    """Owns one DemoSession; called only from the CAN worker thread."""
    def __init__(self, session, *, clock, camera_timeout=.35):
        self.session = session
        self.clock = clock
        self.mode = 'manual'
        self.camera = CameraControl(session, timeout=camera_timeout, clock=clock, log=lambda _: None)
        self.reason = '연결됨 · 출력 시작 전'

    def hold(self, reason='사용자 유지'):
        result = self.session.stop(reason)
        self.camera.gate.block(self.clock(), reason)
        self.camera.last_closed = None
        self.reason = reason
        if result is not None and not result.ok:
            raise RuntimeError('HOLD 응답 없음: 실제 PWM 상태 확인 필요')

    def switch_mode(self, mode):
        if mode not in ('manual', 'camera', 'calibration'):
            raise ValueError('unknown mode')
        self.hold('모드 변경 · 출력을 다시 시작하세요')
        self.mode = mode

    def arm(self, initial_open=False):
        if self.mode == 'camera':
            now = self.clock()
            gate = self.camera.gate
            if not gate.fresh_hand(now) or gate.sample['closed'] is not False:
                raise ValueError('카메라에 편 손 하나를 보여준 뒤 시작하세요')
            # Arm event must refer to this fresh frame, never a queued old click.
            sample = dict(gate.sample, arm_at=now)
            self.camera.initial_open = initial_open
            self.camera.tick(sample=None)
            gate.sample = sample
            self.camera.tick()
            if not self.session.active:
                raise RuntimeError('카메라 입력이 만료되어 시작을 취소했습니다')
        else:
            self.session.arm(initial_open=initial_open)
        self.reason = '출력 중'

    def motor(self, index, pulse):
        if self.mode == 'camera' or not self.session.active:
            raise ValueError('개별/캘리브레이션 모드에서 출력을 먼저 시작하세요')
        if type(index) is not int or not 0 <= index < 15 or type(pulse) is not int:
            raise ValueError('invalid motor / integer PWM required')
        hand = self.session.hand
        target = list(hand.goal)
        lo, hi = hand.spec.low_us[index], hand.spec.high_us[index]
        if not lo <= pulse <= hi:
            raise ValueError(f'현재 펌웨어 허용 범위: {lo}..{hi} us; 범위 밖은 새 펌웨어 필요')
        target[index] = (pulse-hand.spec.open_us[index]) / (hand.spec.close_us[index]-hand.spec.open_us[index])
        # Check the complete target, including the opposing thumb tendon.
        hand.set_goal(target)

    def tick(self, dt, sample=None):
        if self.mode == 'camera':
            self.camera.tick(sample)
            if not self.session.active:
                self.reason = self.camera.gate.state
        elif self.session.active:
            self.session.step(dt)

    def status(self):
        hand = self.session.hand
        sample = self.camera.gate.sample
        camera = dict(valid=self.camera.gate.fresh_hand(self.clock()),
                      closed=None if sample is None else sample.get('closed'),
                      age=None if sample is None else round(max(0, self.clock()-sample['captured_at']), 3))
        return dict(mode=self.mode, active=self.session.active, reason=self.reason,
                    command_us=hand.command_us, goal_us=hand.spec.to_us(hand.goal),
                    command_known=hand._known_command, delivery=hand.last_delivery.as_dict(),
                    applied_us=None if not hand.spec.feedback or hand.last_applied_command is None else list(hand.last_applied_command['pwm_us']),
                    feedback=hand.spec.feedback, same_session_command_known=hand._known_command,
                    estimated_joint_command=dict(hand.preview_joint_command), estimated_joint_goal=dict(hand.preview_joint_goal),
                    camera=camera, dry_run=hand.dry_run, physical_reached=None)

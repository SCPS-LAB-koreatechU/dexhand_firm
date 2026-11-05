"""One-hand bench session, without ROS, camera imports or automatic motion."""
from collections import Counter
import time

from .hand_can import Flag


class DemoSession:
    def __init__(self, hand, *, level=0.25):
        self.hand, self.level = hand, level
        self.active = False
        self.attempted_enable = False
        self.reason = "NOT_ARMED"
        self.last_hold = None
        self.results = Counter()
        self.max_latency_ms = 0.0

    def ping(self):
        if not self.hand.spec.feedback:
            raise RuntimeError('단방향 설정: ESP32 응답을 사용하지 않습니다. CAN 소켓 접속은 보드 응답 확인이 아닙니다')
        pong = self.hand.ping()
        if pong is None:
            raise RuntimeError(f"NO_PONG: {self.hand.spec.channel} · ID {self.hand.spec.node} 응답 없음 · 보드 전원, CAN H/L/GND, 비트레이트를 확인하세요")
        if pong.version != 3 or self.hand.protocol != 3:
            raise RuntimeError(f"PROTOCOL_MISMATCH: device={pong.version}, config={self.hand.protocol}; use protocol 3")
        if not self.hand.spec.profile_tag or pong.profile_tag != self.hand.spec.profile_tag[:8].lower():
            raise RuntimeError(f"PROFILE_MISMATCH: 보드={pong.profile_tag}, SDK={self.hand.spec.profile_tag} · 같은 보정 JSON을 반영한 펌웨어가 필요합니다")
        return pong

    def arm(self, *, initial_open=False):
        if not self.hand.spec.feedback:
            if initial_open:
                self.hand.set_grip('open')
            elif self.hand._known_command and self.hand.held:
                self.hand.goal=list(self.hand.command)
                self.hand._explicit_goal=True  # operator explicitly requested same-session resume
            else:
                raise RuntimeError('단방향 재접속: 현재 보드 자세를 읽을 수 없습니다. 명시적인 손 펼치기가 필요합니다')
        else:
            pong = self.ping()
            if not pong.flags & Flag.PWM_ON:
                if not initial_open:
                    raise RuntimeError("INITIAL_TARGET_REQUIRED: 보드에 이어받을 PWM이 없습니다. 빈손 상태에서 의도적으로 손 펼치기를 실행한 뒤 조절하세요")
                self.hand.set_grip("open")
            else:
                # Never inject an open goal before recovering a hand that is already driven.
                self.hand._explicit_goal = False
        self.attempted_enable = True
        result = self.hand.enable()
        self._note(result)
        if not result.ok:
            stage = getattr(self.hand, 'io_stage', 'ENABLE')
            evidence = dict(getattr(self.hand, 'stats', {}))
            self.stop("ENABLE: " + result.result)
            error = RuntimeError("enable failed: " + result.result + " " + result.message)
            error.io_stage, error.evidence = stage, evidence
            raise error
        self.active, self.reason = True, "ARMED"
        return result

    def set_closed(self, closed):
        if not self.active:
            raise RuntimeError("not armed")
        self.hand.set_grip("power" if closed else "open", self.level if closed else 1.0)

    def _note(self, result):
        self.results[result.result] += 1
        self.max_latency_ms = max(self.max_latency_ms, result.latency_ms)

    def step(self, dt):
        if not self.active:
            return None
        result = self.hand.step(dt)
        if result is None:
            self.stop("CONTROLLER_STOPPED: " + self.hand.fault)
            raise RuntimeError(self.reason)
        self._note(result)
        if not result.ok:
            stage = getattr(self.hand, 'io_stage', 'POSE/APPLIED')
            evidence = dict(getattr(self.hand, 'stats', {}))
            self.stop("CAN_RESULT: " + result.result)
            error = RuntimeError(self.reason)
            error.io_stage, error.evidence = stage, evidence
            raise error
        return result

    def stop(self, reason):
        self.active, self.reason = False, reason
        if self.attempted_enable:
            self.last_hold = self.hand.hold()
        return self.last_hold

    def close(self):
        try:
            if self.attempted_enable and not self.hand.held:
                self.stop("EXIT")
        finally:
            if self.hand.bus is not None:
                self.hand.bus.close()

    def summary(self):
        return dict(hand=self.hand.spec.name, node=self.hand.spec.node, dry_run=self.hand.dry_run,
                    profile_tag=self.hand.spec.profile_tag, feedback_enabled=self.hand.spec.feedback, board_profile_verified=bool(self.hand.spec.feedback and self.hand.last_pong), results=dict(self.results),
                    max_latency_ms=round(self.max_latency_ms, 3), reason=self.reason,
                    hold=None if self.last_hold is None else self.last_hold.as_dict(),
                    physical_reached=None)


class CameraGate:
    """Freshness and explicit re-arm gate, independent of camera/CAN implementation.

    The selected source is one visible hand. Missing, ambiguous or stale frames
    latch a stop; reacquisition alone never resumes output.
    """
    def __init__(self, timeout=0.35):
        self.timeout = timeout
        self.sample = None
        self.last_frame_id = -1
        self.blocked_at = 0.0
        self.last_arm_at = 0.0
        self.state = "WAIT_FOR_HAND"

    def update(self, sample, now):
        if not isinstance(sample, dict) or sample.get("kind") != "frame":
            return False
        stamp, sequence = sample.get("captured_at"), sample.get("frame_id")
        if type(stamp) not in (int, float) or not 0 <= now-stamp <= self.timeout:
            return False
        if type(sequence) is not int or sequence <= self.last_frame_id:
            return False
        if type(sample.get("valid")) is not bool or (sample["valid"] and type(sample.get("closed")) is not bool):
            return False
        self.last_frame_id, self.sample = sequence, sample
        return True

    def fresh_hand(self, now):
        return bool(self.sample is not None and 0 <= now-self.sample["captured_at"] <= self.timeout
                    and self.sample.get("valid") is True)

    def block(self, now, reason):
        self.blocked_at = now
        self.state = reason

    def arm_requested(self, now):
        if not self.fresh_hand(now) or self.sample.get("closed") is not False:
            return False
        stamp = self.sample.get("arm_at", 0.0)
        if (type(stamp) not in (int, float) or not self.blocked_at < stamp <= now or
                now-stamp > self.timeout or stamp <= self.last_arm_at):
            return False
        self.last_arm_at, self.state = stamp, "ARMED"
        return True


class CameraControl:
    """The same camera-to-CAN policy used by the runner and offline tests."""
    def __init__(self, session, *, initial_open=False, timeout=.35, clock=time.monotonic, log=print):
        self.session, self.initial_open, self.clock, self.log = session, initial_open, clock, log
        self.gate = CameraGate(timeout)
        self.last_tick = clock()
        self.last_closed = None

    def tick(self, sample=None):
        now = self.clock()
        if sample is not None:
            self.gate.update(sample, now)
        fresh = self.gate.fresh_hand(now)
        manual_hold = bool(fresh and self.gate.sample.get("arm_at", 0) < 0)
        if self.session.active and (not fresh or manual_hold):
            reason = "OPERATOR_HOLD" if manual_hold else "TRACKING_LOST_OR_FRAME_EXPIRED"
            self.session.stop(reason)
            self.gate.block(now, reason)
            self.last_closed = None
            self.log("HOLD: " + reason + "; a new explicit arm action is required")
        if not self.session.active and self.gate.arm_requested(now):
            self.session.arm(initial_open=self.initial_open)
            self.last_closed = None
            self.log("ARMED: CAN acknowledged; camera follows open/closed only")
        result = None
        if self.session.active:
            checked_at = self.clock()
            if not self.gate.fresh_hand(checked_at):
                self.session.stop("FRAME_EXPIRED_DURING_ENABLE")
                self.gate.block(checked_at, self.session.reason)
                self.last_closed = None
            else:
                closed = self.gate.sample["closed"]
                if closed != self.last_closed:
                    self.session.set_closed(closed)
                    self.last_closed = closed
                    self.log("CLOSE" if closed else "OPEN")
                result = self.session.step(max(1e-6, checked_at-self.last_tick))
        self.last_tick = self.clock()
        return result

    @property
    def activity(self):
        if self.session.active:
            return 1
        return 0 if self.gate.state == "WAIT_FOR_HAND" else -1

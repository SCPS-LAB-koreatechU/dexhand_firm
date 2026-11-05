"""One DexHand over CAN, without ROS: slew-limited pull fractions, 50 Hz pose sets,
and -- with protocol v2 firmware -- a delivery result for every set and command.

What a result can and cannot say (the hardware decides this, not the code):

    SENT        the five frames were queued on the PC adapter (v1: all you get)
    APPLIED     the ESP32 received all five, every check passed, PWM duties written
    REJECT_*    the ESP32 received the set and refused it, with the reason
    NO_REPLY    nothing came back within reply_timeout: frames lost, hand off the
                bus, or the reply lost or slow -- an APPLIED for it that arrives
                later is counted as "late" and clears the miss, and the applied
                counter in the next reply tells a lost reply from a lost set

No result says where a finger is: the ES3352 servos have no feedback, and
"command_us" here is always a command, never a measurement.

Safety behaviour, in the spirit of bhl_arm_sdk:

  * enable pings first (v2) and refuses a hand that does not answer, answers
    with another protocol, or carries a different calibration profile
  * a hand already driving its PWM is resumed (RESUME), never restarted (START
    drops the PWM for a moment and would let go of what it holds)
  * `max_missed` sets without a reply in a row -> fault LOST, streaming stops;
    the hand keeps its last PWM by itself (firmware watchdog)
  * a REJECT_INHIBITED we did not ask for (someone else sent HOLD / OFF, or the
    hand latched after an output failure) stops streaming: fault HELD_ELSEWHERE
  * poses that pull antagonist tendons together are refused before sending
  * dry_run (default) opens no bus: every set is "APPLIED" at once
"""

from __future__ import annotations

import time
import math
import secrets
import struct
from dataclasses import asdict, dataclass
import copy

from .hand_retarget import synchronized_step
from .config import HandConfig, HandSpec
from .hand_can import (CHANNELS, Flag, HandBus, Mode, Pong, Result, Status, control_frame,
                       ping_frame, pose_frames, Boot, StatePart, Snapshot, F_PING, can_id)

PRESENT_FOR = 1.5      # [s] heard from within this -> present (heartbeat is 0.5 s)
MODE_NAMES = {Mode.OFF: "OFF", Mode.START: "START", Mode.HOLD: "HOLD", Mode.RESUME: "RESUME", Mode.CLEAR_FLAGS: "CLEAR_FLAGS"}


@dataclass
class Delivery:
    """What happened to one pose set or control command."""

    ok: bool
    result: str               # APPLIED, CONTROL_ACK, SENT, NO_REPLY, REJECT_..., SEND_FAILED, ...
    sequence: int = -1
    latency_ms: float = -1.0  # send -> reply; -1 without a reply
    detail: int = 0xFF        # channel (1-based in `message`) or mode, as the hand sent it
    message: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


class HandController:
    def __init__(self, config: HandConfig, hand: str, dry_run: bool = True, bus=None) -> None:
        # rightfix1: isolated per-hand rules; raw goals remain in actual CAN order.
        self.config = copy.deepcopy(config)
        self.spec: HandSpec = self.config.hand(hand)
        if hand == "right":
            self.config.antagonist_pairs = [(13, 15)]
            self.config.grips = {name: list(g[:12]) + [g[13], g[14], g[12]]
                                 for name, g in self.config.grips.items()}
        self.dry_run = dry_run
        self.protocol = self.spec.protocol
        self.bus = None
        if not dry_run:
            self.bus = bus if bus is not None else HandBus(self.spec.channel, [self.spec.node])
        open_pose = list(self.config.grips["open"])
        self.goal = list(open_pose)
        self.command = list(open_pose)
        self.live = False             # streaming pose sets every step
        self.held = False             # we sent HOLD (or OFF) and are not streaming
        self.estopped = False
        self.fault = ""
        self.message = ""
        self._sequence = 0
        self.last_delivery = Delivery(False, "NONE")
        self.last_status: Status | None = None
        self.last_pong: Pong | None = None
        self.last_heard = 0.0
        self.stats = {"sets_sent": 0, "applied": 0, "rejected": 0, "no_reply": 0, "acks_lost": 0,
                      "send_failed": 0, "late": 0, "max_latency_ms": 0.0}
        self.missed = 0
        self._last_applied_count: int | None = None
        self._explicit_goal = False
        self._known_command = False
        self._pending = {}
        self.boot_id = None
        self._last_uptime = None
        self._stream_deadline = None
        self._command_owner = None
        self._awaiting = -1           # sequence whose STATUS _send_set is waiting for
        self.command_context = None  # optional PC session/sequence at send time
        self.last_applied_command = None
        self.last_io_timing = None
        self.preview_joint_goal = {}
        self.preview_joint_command = {}

    # ------------------------------------------------------------------ helpers
    @property
    def confirmed(self) -> bool:
        """True when the hand can tell us what it did (protocol v2 on a real bus)."""
        return not self.dry_run and self.protocol >= 2 and self.spec.feedback

    @property
    def present(self) -> bool:
        if self.dry_run:
            return True
        return self.confirmed and time.monotonic() - self.last_heard < PRESENT_FOR

    @property
    def command_us(self) -> list[int]:
        return self.spec.to_us(self.command)

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFF
        return self._sequence

    def _note(self, frame) -> bool:
        if frame.node != self.spec.node:
            return False
        if isinstance(frame, Boot):
            changed = self.boot_id is not None and self.boot_id != frame.boot_id
            if self._last_uptime is not None and self.boot_id == frame.boot_id:
                elapsed = (frame.uptime_ms - self._last_uptime) & 0xFFFFFFFF
                if elapsed >= 0x80000000:
                    return False  # stale boot telemetry
            self.boot_id, self._last_uptime = frame.boot_id, frame.uptime_ms
            if changed:
                self._pending.clear()
                self._last_applied_count = None
                self._explicit_goal = self._known_command = False
                self.last_applied_command = None
                self._fail("DEVICE_REBOOT", "ESP32 rebooted; supply a fresh initial target and explicitly enable")
            self.last_heard = frame.stamp
            return True
        if isinstance(frame, StatePart):
            return True
        if isinstance(frame, Pong):
            self.last_pong = frame
            self.last_heard = frame.stamp
            return True
        if not isinstance(frame, Status) or frame.result not in Result.NAMES:
            return False
        if frame.result not in (Result.HEARTBEAT, Result.CONTROL_ACK):
            pending = self._pending.get(frame.sequence)
            if pending is None or not 0 <= frame.stamp - pending[0] <= 0.5:
                return False
        jump = 0 if self._last_applied_count is None else (frame.applied - self._last_applied_count) & 0xFFFF
        if jump > 128:
            self._pending.clear()
            self._last_applied_count = None
            self._explicit_goal = self._known_command = False
            self.last_applied_command = None
            self._fail("COUNTER_RESET", "unexpected applied counter; resynchronize before enabling")
            return False
        if frame.result not in (Result.HEARTBEAT, Result.CONTROL_ACK):
            pending = self._pending.get(frame.sequence)
            if pending is None or not 0 <= frame.stamp - pending[0] <= 0.5:
                return False  # duplicate, replay, expired or another sender
            if frame.result == Result.APPLIED and pending[1] is not None:
                progress = (frame.applied - pending[1]) & 0xFFFF
                if not 1 <= progress <= 128:
                    return False
            if frame.result == Result.APPLIED:
                self._record_applied(pending[2], pending[3], frame.sequence, frame.stamp, "firmware_ack")
            del self._pending[frame.sequence]
            if frame.result == Result.APPLIED and frame.sequence != self._awaiting:
                self.stats["late"] += 1
                self.stats["no_reply"] = max(0, self.stats["no_reply"] - 1)
                self.missed = 0
        if jump:
            self.stats["acks_lost"] += max(0, jump - (1 if frame.result == Result.APPLIED else 0))
        self._last_applied_count = frame.applied
        self.last_status = frame
        self.last_heard = frame.stamp
        return True

    def _await(self, match, timeout: float):
        """Read replies for up to `timeout`; the first one `match` accepts, or None."""
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            frame = self.bus.recv(left)
            if frame is None:
                return None
            accepted = self._note(frame)
            if accepted and match(frame):
                return frame

    def poll(self) -> None:
        """Drain replies that arrived on their own (heartbeats, late acks)."""
        if self.confirmed:
            for _ in range(256):
                frame = self.bus.recv(0.0)
                if frame is None:
                    break
                self._note(frame)

    # --------------------------------------------------------------- commands
    def ping(self) -> Pong | None:
        self.io_stage = "PING/PONG"
        if not self.spec.feedback:return None
        if self.dry_run:
            return Pong(self.spec.node, self.protocol, Flag.PWM_ON if self.live or self._known_command else 0, self.spec.profile_tag,
                        time.monotonic())
        if not self.confirmed:
            return None
        self.bus.send(*ping_frame(self.spec.node))
        return self._await(lambda f: isinstance(f, Pong) and f.node == self.spec.node,
                           self.config.ping_timeout)

    def read_snapshot(self) -> Snapshot | None:
        """Protocol 3: atomic last-command readback, never an actual measurement."""
        self.io_stage = "STATE/SNAPSHOT"
        if self.protocol < 3 or self.dry_run or not self.spec.feedback:
            return None
        self.poll()
        nonce = secrets.randbelow(256)
        if not self.bus.send(can_id(F_PING, self.spec.node), bytes([0xCB, nonce])):
            return None
        parts = {}
        deadline = time.monotonic() + self.config.ping_timeout
        while time.monotonic() < deadline:
            frame = self.bus.recv(max(0, deadline - time.monotonic()))
            if frame is None:
                break
            if not isinstance(frame, StatePart):
                self._note(frame)
                continue
            if frame.node != self.spec.node or frame.nonce != nonce:
                continue
            if frame.group in parts and parts[frame.group] != frame.data:
                return None
            parts[frame.group] = frame.data
            if 0xF0 not in parts or 0xF1 not in parts:
                continue
            boot, count = struct.unpack("<IH", parts[0xF0])
            flags, sequence = parts[0xF1][:2]
            if not flags & Flag.HAVE_POSE:
                return Snapshot(boot, count, flags, sequence, None)
            if all(g in parts for g in range(5)):
                pulses = tuple(v for g in range(5) for v in struct.unpack("<3H", parts[g]))
                return Snapshot(boot, count, flags, sequence, pulses)
        return None

    def _control(self, mode: int) -> Delivery:
        name = MODE_NAMES[mode]
        self.io_stage = "CONTROL/" + name
        if self.dry_run:
            return Delivery(True, "CONTROL_ACK" if self.spec.feedback else "SENT", detail=mode, message=f"{name} (dry run)")
        self.poll()  # discard already queued ACKs before the fresh correlated request
        sent_at = time.monotonic()
        token = secrets.randbelow(256) if self.protocol >= 3 else None
        if not self.bus.send(*control_frame(self.spec.node, mode, token)):
            self.stats["send_failed"] += 1
            return Delivery(False, "SEND_FAILED", detail=mode, message=f"{name}: CAN send failed")
        if not self.confirmed:
            return Delivery(True, "SENT", detail=mode, message=f"{name} sent (v1: no confirmation)")
        reply = self._await(lambda f: isinstance(f, Status) and f.result == Result.CONTROL_ACK
                            and f.detail == mode and (token is None or f.sequence == token), self.config.ping_timeout)
        if reply is None:
            return Delivery(False, "NO_REPLY", detail=mode, message=f"{name}: no CONTROL_ACK")
        return Delivery(True, "CONTROL_ACK", detail=mode,
                        latency_ms=(reply.stamp - sent_at) * 1e3, message=name)

    def _send_set(self) -> Delivery:
        self.io_stage = "POSE/APPLIED" if self.spec.feedback else "POSE/SEND_ONLY"
        problem = self.config.check_pose(self.command)
        if problem:
            return self._fail("INVALID_COMMAND", problem)
        sequence = self._next_sequence()
        pulses = self.command_us
        self.stats["sets_sent"] += 1
        if self.dry_run:
            if not self.spec.feedback:return Delivery(True,"SENT",sequence,message="dry run; no confirmation")
            self.stats["applied"] += 1
            self._record_applied(pulses, self.command_context, sequence, time.monotonic(), "simulated")
            return Delivery(True, "APPLIED", sequence, 0.0, message="dry run")
        sent_at = time.monotonic()
        self.last_io_timing = dict(sequence=sequence, queue_started_monotonic_s=sent_at,
            queue_finished_monotonic_s=None, frames_queued=0,
            ack_decoded_monotonic_s=None, wait_finished_monotonic_s=None,
            result='WAITING', timestamp_scope='host enqueue/decode; not CAN-wire or physical-position timestamps')
        self._pending = {seq: p for seq, p in self._pending.items() if sent_at - p[0] <= 0.5}
        self._pending[sequence] = (sent_at, self._last_applied_count, tuple(pulses), copy.deepcopy(self.command_context))
        for ident, data in pose_frames(self.spec.node, sequence, pulses):
            if not self.bus.send(ident, data):
                # The sequence is spent: a partial set is never completed under it.
                self.stats["send_failed"] += 1
                self.last_io_timing.update(result='SEND_FAILED', queue_finished_monotonic_s=time.monotonic())
                return Delivery(False, "SEND_FAILED", sequence, message="CAN send failed mid-set")
            self.last_io_timing['frames_queued'] += 1
        self.last_io_timing['queue_finished_monotonic_s'] = time.monotonic()
        if not self.confirmed:
            self.last_io_timing['result'] = 'SENT'
            return Delivery(True, "SENT", sequence, message="v1 firmware: delivery not confirmed")
        self._awaiting = sequence
        try:
            reply = self._await(lambda f: isinstance(f, Status) and f.sequence == sequence and
                                f.result not in (Result.HEARTBEAT, Result.CONTROL_ACK),
                                self.config.reply_timeout)
        finally:
            # Late ACKs during HOLD/poll must not be mistaken for an active wait.
            self._awaiting = -1
        self.last_io_timing.update(wait_finished_monotonic_s=time.monotonic(),
            ack_decoded_monotonic_s=None if reply is None else reply.stamp,
            result='NO_REPLY' if reply is None else reply.name)
        if self.fault in ("DEVICE_REBOOT", "COUNTER_RESET"):
            return Delivery(False, self.fault, sequence, message=self.message)
        if reply is None:
            self.stats["no_reply"] += 1
            return Delivery(False, "NO_REPLY", sequence)
        latency = (reply.stamp - sent_at) * 1e3
        self.stats["max_latency_ms"] = max(self.stats["max_latency_ms"], round(latency, 3))
        if reply.result == Result.APPLIED:
            self.stats["applied"] += 1
            return Delivery(True, "APPLIED", sequence, latency)
        self.stats["rejected"] += 1
        message = ""
        if reply.detail != 0xFF and reply.result in (Result.REJECT_RANGE, Result.REJECT_CONFLICT, Result.REJECT_PAIR):
            message = f"channel {reply.detail + 1}"
        return Delivery(False, reply.name, sequence, latency, reply.detail, message)

    def _record_applied(self, pulses, context, sequence, stamp, source):
        previous = self.last_applied_command
        if previous is not None and stamp < previous["observed_monotonic_s"]:
            return
        self.last_applied_command = dict(pwm_us=list(pulses), context=copy.deepcopy(context),
                                         can_sequence=sequence, observed_monotonic_s=stamp, source=source)

    # ---------------------------------------------------------------- lifecycle
    def enable(self) -> Delivery:
        """Preserve an already driven hand; a cold hand requires an explicit goal."""
        if self.estopped:
            return Delivery(False, "ESTOP", message="reset E-STOP before enable")
        if self.live:
            return Delivery(True, "ALREADY_LIVE")
        self.fault = ""
        self.missed = 0
        self.command_context = None  # initialization/readback is not a new upper-body goal
        mode = Mode.START
        initial = None
        if self.confirmed:
            pong = self.ping()
            if pong is None:
                return self._fail("NO_REPLY", "no PONG")
            if pong.version != self.protocol:
                return self._fail("PROTOCOL", f"firmware {pong.version}, config {self.protocol}")
            if self.spec.profile_tag and pong.profile_tag != self.spec.profile_tag.lower()[:8]:
                return self._fail("PROFILE_MISMATCH", "flashed calibration differs from SDK")
            if self.protocol >= 3 and not pong.flags & Flag.PWM_ON:
                # Establish boot/counter identity even on a cold board. Its first
                # unsolicited BOOT may have preceded opening our CAN socket.
                cold = self.read_snapshot()
                if cold is None:
                    return self._fail("STATE_UNAVAILABLE", "cannot establish cold boot identity")
                if cold.flags & Flag.PWM_ON:
                    self.hold()
                    return self._fail("STATE_CHANGED", "PWM changed during initialization; explicitly resynchronize")
                self.boot_id, self._last_uptime = cold.boot_id, None
                self._last_applied_count = cold.applied
                self._pending.clear()
                self.last_applied_command = None
            if pong.flags & Flag.PWM_ON:
                if self.protocol >= 3:
                    hold = self._control(Mode.HOLD)
                    if not hold.ok:
                        return self._fail(hold.result, "cannot hold before state synchronization")
                    snap = self.read_snapshot()
                    if snap is None or snap.pulses is None or not snap.flags & Flag.PWM_ON:
                        return self._fail("STATE_UNAVAILABLE", "cannot read last commanded pose; hand left held")
                    try:
                        initial = self.spec.to_fractions(snap.pulses)
                    except ValueError as error:
                        return self._fail("STATE_INVALID", str(error))
                    if self.config.check_pose(initial):
                        # Up to half a microsecond per servo is lost in PWM rounding.
                        for a, b in self.config.antagonist_pairs:
                            total = initial[a - 1] + initial[b - 1]
                            tolerance = 0.51 / abs(self.spec.close_us[a - 1] - self.spec.open_us[a - 1]) + 0.51 / abs(self.spec.close_us[b - 1] - self.spec.open_us[b - 1])
                            if total > self.config.max_pair_sum + tolerance:
                                return self._fail("STATE_INVALID", "held pose violates tendon constraint")
                            if total > self.config.max_pair_sum:
                                initial[a - 1] *= self.config.max_pair_sum / total
                                initial[b - 1] *= self.config.max_pair_sum / total
                    self.boot_id = snap.boot_id
                    self._record_applied(snap.pulses, None, snap.sequence, time.monotonic(), "firmware_snapshot")
                    self._last_uptime = None
                    self._last_applied_count = snap.applied
                    self._pending.clear()
                elif self._known_command:
                    initial = list(self.command)
                else:
                    return self._fail("STATE_UNAVAILABLE", "v1/v2 cannot safely recover an unknown held pose; use v3 firmware")
                mode = Mode.RESUME
        if not self.spec.feedback and self._known_command and self.held:
            # Only same-process explicit resume from the last SENT command.
            # This is not board readback and never restores across reconnects.
            initial=list(self.command);mode=Mode.RESUME
        if initial is None:
            if not self._explicit_goal and not self.dry_run:
                return self._fail("INITIAL_TARGET_REQUIRED", "set a deliberate initial pose before enabling an undriven hand")
            initial = list(self.goal)
        elif not self._explicit_goal:
            self.goal = list(initial)
        self.command = list(initial)
        control = self._control(mode)
        if not control.ok:
            return self._fail(control.result, control.message)
        delivery = self._send_set()
        self.last_delivery = delivery
        if not delivery.ok:
            return self._fail(delivery.result, f"first set: {delivery.result}")
        self._known_command = True
        self.live, self.held = True, False
        self.message = f"live ({MODE_NAMES[mode]}); command synchronized" if self.spec.feedback else f"streaming ({MODE_NAMES[mode]}); CAN queued only, board unconfirmed"
        return delivery

    def _fail(self, fault: str, message: str) -> Delivery:
        self.fault, self.message, self.live = fault, message, False
        return Delivery(False, fault, message=message)

    def hold(self) -> Delivery:
        """Stop streaming and tell the hand to keep its last PWM and ignore poses."""
        self.live = False
        self.held = True
        self._stream_deadline = None
        self._command_owner = None
        self._explicit_goal = False
        if self._known_command:
            self.goal = list(self.command)
        delivery = self._control(Mode.HOLD)
        self.message = "held" if delivery.ok else f"HOLD {delivery.result}"
        return delivery

    def resume(self) -> Delivery:
        """After hold: continue from the pose being held (no PWM drop)."""
        if self.estopped:
            return Delivery(False, "ESTOP", message="E-STOP is latched; reset it first")
        if self._known_command:
            self.goal = list(self.command)
        # A last host command may never have reached PWM. Protocol 3 enable
        # reads the held PWM; resume must adopt it rather than replay that goal.
        self._explicit_goal = False
        return self.enable()

    def off(self) -> Delivery:
        """Stop the PWM. Not a torque-off guarantee, and the grip is released."""
        self.live = False
        self.held = True
        self._known_command = self._explicit_goal = False
        self.last_applied_command = None
        self._pending.clear()
        self._command_owner = None
        self.command_context = None
        self._stream_deadline = None
        delivery = self._control(Mode.OFF)
        self.message = "PWM off" if delivery.ok else f"OFF {delivery.result}"
        return delivery

    def estop(self, reason: str = "E-STOP") -> Delivery:
        delivery = self.off() if self.config.estop_action == "off" else self.hold()
        self.estopped = True
        self.message = f"{reason} ({self.config.estop_action}: {delivery.result})"
        return delivery

    def reset_estop(self) -> bool:
        # Explicit operator acknowledgement. Verify the hand before clearing a
        # fault, without moving it; coordinated arm/hand holds must be recoverable.
        if self.confirmed:
            pong = self.ping()
            if pong is None or pong.version != self.protocol or (
                self.spec.profile_tag and pong.profile_tag != self.spec.profile_tag.lower()[:8]
            ):
                self.message = "cannot reset: hand/profile is not verified"
                return False
        self.estopped = False
        self.fault = ""
        self._stream_deadline = None
        self._command_owner = None
        self._explicit_goal = False
        self.message = "fault acknowledged; output unchanged, explicitly resume or set a cold initial target"
        return True

    def clear_flags(self) -> Delivery:
        return self._control(Mode.CLEAR_FLAGS)

    def close(self) -> None:
        if self.bus is not None:
            if self.live:
                self.hold()        # leave the hand holding, not limp
            self.bus.close()
        self.live = False

    # ------------------------------------------------------------------ targets
    def set_goal(self, fractions, streaming=False, source="api") -> list[float]:
        values = [float(v) for v in fractions]
        problem = self.config.check_pose(values)
        if problem:
            raise ValueError(problem)
        now = time.monotonic()
        if self._stream_deadline is not None and now < self._stream_deadline and self._command_owner is not None:
            if not streaming or source != self._command_owner:
                raise ValueError("another input owns streaming control; HOLD before changing source")
        self._command_owner = source if streaming else None
        self.goal = values
        self._explicit_goal = True
        self._stream_deadline = time.monotonic() + self.config.command_timeout if streaming else None
        return self.goal

    def set_goal_us(self, pulses_us, streaming=False, source="api") -> list[float]:
        return self.set_goal(self.spec.to_fractions(pulses_us), streaming=streaming, source=source)

    def set_grip(self, name: str, level: float = 1.0) -> list[float]:
        """Goal between open (level 0) and the named grip (level 1)."""
        if name not in self.config.grips:
            raise KeyError(f"unknown grip {name}; known: {sorted(self.config.grips)}")
        level = float(level)
        if not math.isfinite(level) or not 0 <= level <= 1:
            raise ValueError("grip level must be finite and in 0..1")
        start, end = self.config.grips["open"], self.config.grips[name]
        return self.set_goal([a + level * (b - a) for a, b in zip(start, end)])

    def settled(self) -> bool:
        return all(abs(c - g) <= 1e-6 for c, g in zip(self.command, self.goal))

    # --------------------------------------------------------------------- loop
    def step(self, dt: float) -> Delivery | None:
        """One cycle: slew the command, send one pose set, judge the reply."""
        if not self.live or self.estopped:
            self.poll()
            return None
        if not math.isfinite(dt) or dt <= 0 or not math.isfinite(self.config.max_speed) or self.config.max_speed <= 0:
            return self._fail("INVALID_TIMING", "dt and max_speed must be finite and positive")
        if self._stream_deadline is not None and time.monotonic() > self._stream_deadline:
            self.hold()
            return self._fail("COMMAND_TIMEOUT", "upper-level streaming command expired; explicit resume required")
        dt = min(dt, 2 / self.config.control_rate_hz)
        max_step = self.config.max_speed * dt
        if self.config.synchronized_motion:
            self.command,phase=synchronized_step(self.command,self.goal,dt,self.config.max_speed,self.config.motor_full_stroke_time_s)
            self.preview_joint_command={k:self.preview_joint_command.get(k,0.)+phase*(v-self.preview_joint_command.get(k,0.)) for k,v in self.preview_joint_goal.items()}
        else:
            self.command = [c + max(-max_step, min(max_step, g - c)) for c, g in zip(self.command, self.goal)]
        delivery = self._send_set()
        self.last_delivery = delivery
        if delivery.ok:
            self.missed = 0
            return delivery
        if delivery.result in ("NO_REPLY", "SEND_FAILED", "REJECT_DEADLINE"):
            self.missed += 1
            if self.missed >= self.config.max_missed:
                self._fail("LOST", f"{self.missed} sets in a row without APPLIED "
                                   f"(last {delivery.result}); hand keeps its last PWM")
        elif delivery.result == "REJECT_INHIBITED":
            self._fail("HELD_ELSEWHERE", "hand refuses poses (HOLD/OFF from someone else, "
                                         "or latched after an output failure)")
        else:
            self._fail(delivery.result, f"set {delivery.sequence}: {delivery.result} {delivery.message}".strip())
        return delivery

    def status(self) -> dict:
        s = self.last_status
        return {
            "hand": self.spec.name, "node": self.spec.node, "channel": self.spec.channel,
            "dry_run": self.dry_run, "protocol": self.protocol, "confirmed": self.confirmed,
            "present": self.present, "live": self.live, "held": self.held, "estop": self.estopped,
            "fault": self.fault, "message": self.message, "missed": self.missed,
            "settled": self.settled(),  # legacy alias: command interpolation only
            "command_settled": self.settled(),
            "measured_position": None, "measured_force": None, "physical_reached": None,
            "units": {"goal": "tendon_pull_fraction", "command": "tendon_pull_fraction", "command_us": "microseconds"},
            "feedback_source": "none", "delivery_semantics": "PWM_command_applied_not_physical_arrival",
            "last_applied_command": None if self.last_applied_command is None else {
                **self.last_applied_command,
                "age_s": round(max(0, time.monotonic() - self.last_applied_command["observed_monotonic_s"]), 3)},
            "goal": [round(v, 4) for v in self.goal],
            "command": [round(v, 4) for v in self.command],
            "command_us": self.command_us,         # a command, never a measurement
            "last_delivery": self.last_delivery.as_dict(),
            "hand_reported": None if s is None else {
                "result": s.name, "sequence": s.sequence, "flags": s.flag_names(),
                "applied_count": s.applied, "tx_errors": s.tx_errors, "rx_errors": s.rx_errors,
                "age_s": round(time.monotonic() - s.stamp, 3)},
            "profile_tag": None if self.last_pong is None else self.last_pong.profile_tag,
            "stats": dict(self.stats), "boot_id": self.boot_id, "command_known": self._known_command, "command_owner": self._command_owner,
        }

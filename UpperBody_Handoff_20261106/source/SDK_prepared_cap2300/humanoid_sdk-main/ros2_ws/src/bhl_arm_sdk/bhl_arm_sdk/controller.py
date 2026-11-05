"""Per-joint position control of the BHL arm actuators, without ROS.

The control flow follows Unitree's g1_arm7_sdk_dds_example.cpp:

  * read the current joint positions first, and start every command there
  * move each joint towards its goal by at most `max_speed_deg * dt` per step
    (the example's max_joint_velocity / max_joint_delta), at a fixed rate
  * keep sending the final target until explicitly disabled or a fault occurs

and the actuator side follows the upstream Bimanual class: per joint, IDLE ->
write kp / kd / torque limit -> feed the watchdog -> POSITION, the pose at
start taken as joint zero, then one PDO_2 exchange per joint per cycle.

On top of that, the safety behaviour this lab needed on its own arms:

  * a joint that does not answer the ping is never enabled
  * a joint that stops answering for `max_missed` cycles is dropped (fault LOST)
  * a joint lagging its command by more than `max_tracking_error_deg` stops
    every joint (fault TRACKING) -- a reversed direction or a bad encoder shows
    up here instead of as a runaway
  * dry_run (default) opens no bus: the measured angle simply follows the command
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .config import ArmConfig, JointSpec
from .recoil_can import Mode, Param, RecoilBus


SUPPORTED_FIRMWARE = 0x20250226
MODE_NAMES = {value: name for name, value in vars(Mode).items() if name.isupper()}


@dataclass
class JointState:
    spec: JointSpec
    zero: float = 0.0               # motor reading at joint 0
    measured_deg: float = 0.0
    velocity_dps: float = 0.0
    command_deg: float = 0.0        # the slew-limited setpoint actually sent
    goal_deg: float = 0.0           # where the user asked it to go
    live: bool = False              # host is sending position commands; see hardware_mode
    fault: str = ""
    missed: int = 0
    last_reply: float = 0.0
    firmware_version: int | None = None
    firmware_gear_ratio: float | None = None
    current_limit_a: float | None = None
    torque_constant: float | None = None
    startup_bus_voltage: float | None = None
    hardware_mode: int | None = None
    hardware_error: int | None = None
    mode_read_at: float = 0.0
    error_read_at: float = 0.0
    health_failure: str = ""
    preflight_complete: bool = False
    command_velocity_dps: float = 0.0
    bus_voltage_v: float | None = None
    voltage_read_at: float = 0.0


class ArmController:
    def __init__(self, config: ArmConfig, dry_run: bool = True, bus_factory=RecoilBus,
                 max_missed: int = 25, max_tracking_error_deg: float = 20.0) -> None:
        self.config = config
        self.dry_run = dry_run
        self.max_missed = max_missed
        self.max_tracking_error_deg = max_tracking_error_deg
        self.max_speed_deg = config.max_speed_deg
        self.joints = {spec.name: JointState(spec=spec) for spec in config.joints}
        self.buses: dict[str, RecoilBus] = {}
        self.estopped = False
        self.message = ""
        self._last_feed = 0.0
        self._health_cursor = 0
        self.motion_stopped = False
        self._stream_deadline = None
        self._command_owner = None
        self.loop_stats = {"cycles": 0, "deadline_misses": 0, "max_step_ms": 0.0, "last_period_ms": 0.0}
        self._last_step_at = None
        if not dry_run:
            for key, channel in config.buses.items():
                if any(j.bus == key and j.enabled for j in config.joints) and channel not in \
                        {b.channel for b in self.buses.values()}:
                    self.buses[key] = bus_factory(channel, reply_timeout=config.reply_timeout)
            # several bus keys may share one channel (one adapter for both arms)
            for key, channel in config.buses.items():
                if key not in self.buses:
                    shared = next((b for b in self.buses.values() if b.channel == channel), None)
                    if shared is not None:
                        self.buses[key] = shared

        try:
            for bus in {id(b): b for b in self.buses.values()}.values():
                if hasattr(bus, "reserve_nodes"):
                    bus.reserve_nodes([j.device_id for j in config.joints if j.enabled and config.buses[j.bus] == bus.channel])
        except Exception:
            for bus in {id(b): b for b in self.buses.values()}.values():
                bus.close()
            raise

    # ------------------------------------------------------------------ helpers
    def _bus(self, state: JointState) -> RecoilBus | None:
        return self.buses.get(state.spec.bus)

    def _select(self, names) -> list[JointState]:
        if names is None:
            return [s for s in self.joints.values() if s.spec.enabled]
        unknown = [n for n in names if n not in self.joints]
        if unknown:
            raise KeyError(f"unknown joints: {unknown}")
        return [self.joints[n] for n in names if self.joints[n].spec.enabled]

    @property
    def any_live(self) -> bool:
        return any(s.live for s in self.joints.values())

    def _preflight(self, state: JointState, bus: RecoilBus) -> bool:
        """Check units and read the board current clamp without changing it."""
        state.preflight_complete = False
        node = state.spec.device_id
        self._service_heartbeat()
        state.firmware_version = bus.read_u32(node, Param.FIRMWARE_VERSION)
        if state.firmware_version != SUPPORTED_FIRMWARE:
            state.fault = "FIRMWARE_UNVERIFIED"
            return False
        for attr, param in (("firmware_gear_ratio", Param.GEAR_RATIO),
                            ("current_limit_a", Param.CURRENT_LIMIT),
                            ("torque_constant", Param.TORQUE_CONSTANT),
                            ("startup_bus_voltage", Param.BUS_VOLTAGE)):
            self._service_heartbeat()
            value = bus.read_f32(node, param)
            if value is None or not math.isfinite(value):
                setattr(state, attr, None)
                state.fault = f"PREFLIGHT_READ {attr}"
                return False
            setattr(state, attr, value)
        if not math.isclose(state.firmware_gear_ratio, state.spec.firmware_gear_ratio, rel_tol=1e-5):
            state.fault = f"GEAR_MISMATCH board={state.firmware_gear_ratio:g} expected={state.spec.firmware_gear_ratio:g}"
            return False
        if state.current_limit_a <= 0 or state.torque_constant <= 0 or state.startup_bus_voltage <= 0:
            state.fault = "INVALID_BOARD_LIMITS"
            return False
        self._service_heartbeat()
        state.hardware_error = bus.read_u32(node, Param.ERROR)
        state.error_read_at = time.monotonic() if state.hardware_error is not None else 0.0
        if state.hardware_error != 0:
            state.fault = "BOARD_ERROR_UNREAD" if state.hardware_error is None else f"BOARD_ERROR 0x{state.hardware_error:08X}"
            return False
        state.preflight_complete = True
        return True

    def _poll_health(self) -> None:
        """One bounded SDO read per step: 10 joints => each MODE every 30 steps.

        ERROR may be latched history. Report it without cutting holding torque.
        A confirmed departure from POSITION drops that joint from host control,
        like LOST; never automatically send POSITION again. An SDO failure is a
        visible loss of verification, not a reason by itself to release the arm.
        """
        candidates = [s for s in self.joints.values() if s.preflight_complete and not s.health_failure]
        if not candidates:
            return
        slot = self._health_cursor % (3 * len(candidates))
        self._health_cursor += 1
        state = candidates[slot // 3]
        param = (Param.MODE, Param.ERROR, Param.BUS_VOLTAGE)[slot % 3]
        bus = self._bus(state)
        reader = bus.read_f32 if param == Param.BUS_VOLTAGE else bus.read_u32
        try:
            value = reader(state.spec.device_id, param, timeout=self.config.health_timeout)
        except Exception as error:
            state.health_failure = f"SDO_UNVERIFIED {type(error).__name__}"
            return
        if value is None or not math.isfinite(value):
            state.health_failure = "SDO_UNVERIFIED; restart controller to recheck"
            return
        now = time.monotonic()
        if param == Param.MODE:
            state.hardware_mode, state.mode_read_at = value, now
            if state.live and value != Mode.POSITION:
                state.fault = f"MODE_LOST {MODE_NAMES.get(value, hex(value))}"
                state.live = False
                state.goal_deg = state.command_deg = state.measured_deg
                self.message = f"{state.spec.name}: {state.fault}; no automatic re-enable"
        elif param == Param.ERROR:
            state.hardware_error, state.error_read_at = value, now
        else:
            state.bus_voltage_v, state.voltage_read_at = value, now
        if state.fault.startswith("MODE_LOST") and self.config.stop_on_joint_fault:
            self.hold("JOINT_FAULT: " + state.spec.name)

    # ----------------------------------------------------------------- lifecycle
    def enable(self, names=None) -> list[str]:
        """Put joints in position mode holding where they are. Returns the joints
        that came up; the others carry a fault saying why."""
        if self.estopped or self.motion_stopped:
            self.message = "stop is latched; explicitly reset/resume first"
            return []
        up = []
        for state in self._select(names):
            if state.live:
                up.append(state.spec.name)
                continue
            state.fault = ""
            state.missed = 0
            state.health_failure = ""
            if self.dry_run:
                if self.config.zero == "config" and state.spec.motor_zero is not None:
                    state.zero = state.spec.motor_zero
                state.command_deg = state.goal_deg = state.measured_deg
                state.live = True
                up.append(state.spec.name)
                continue
            self._service_heartbeat()
            bus = self._bus(state)
            spec = state.spec
            if bus is None or not bus.ping(spec.device_id):
                state.fault = "NO_REPLY"
                continue
            if not self._preflight(state, bus):
                continue
            motor = bus.read_f32(spec.device_id, Param.POSITION_MEASURED)
            if motor is None or not math.isfinite(motor):
                state.fault = "NO_POSITION"
                continue
            state.zero = motor if self.config.zero == "start" else float(spec.motor_zero)
            state.measured_deg = math.degrees(spec.to_joint(motor, state.zero))
            if not spec.lower_deg - 5.0 <= state.measured_deg <= spec.upper_deg + 5.0:
                # Enabling would pull the joint straight to its limit.
                state.fault = f"OUT_OF_RANGE {state.measured_deg:+.1f} deg"
                continue
            state.command_deg = state.goal_deg = spec.clamp_deg(state.measured_deg)
            bus.set_mode(spec.device_id, Mode.IDLE)
            bus.write_gains(spec.device_id, spec.kp, spec.kd, spec.torque_limit)
            for param, requested in ((Param.POSITION_KP, spec.kp), (Param.VELOCITY_KP, spec.kd),
                                     (Param.TORQUE_LIMIT, spec.torque_limit)):
                self._service_heartbeat()
                actual = bus.read_f32(spec.device_id, param)
                if actual is None or not math.isfinite(actual) or not math.isclose(actual, requested, rel_tol=1e-5, abs_tol=1e-7):
                    state.fault = f"GAIN_READBACK 0x{param:03X}"
                    break
            if state.fault:
                continue  # stays in IDLE; no POSITION command was sent
            bus.feed(spec.device_id)
            bus.set_mode(spec.device_id, Mode.POSITION)
            reply = bus.exchange_pdo_2(spec.device_id, spec.to_motor(math.radians(state.command_deg), state.zero))
            if reply is None or not all(math.isfinite(v) for v in reply):
                bus.set_mode(spec.device_id, Mode.IDLE)
                state.fault = "NO_PDO_REPLY"
                continue
            state.hardware_mode = bus.read_u32(spec.device_id, Param.MODE)
            state.mode_read_at = time.monotonic() if state.hardware_mode is not None else 0.0
            if state.hardware_mode != Mode.POSITION:
                bus.set_mode(spec.device_id, Mode.DAMPING)
                state.fault = "POSITION_NOT_CONFIRMED"
                continue
            state.last_reply = time.monotonic()
            state.command_velocity_dps = 0.0
            state.live = True
            up.append(spec.name)
        self.message = f"enabled {len(up)} joint(s)"
        return up

    def disable(self, names=None, damping: bool = True) -> None:
        """Stop position control. DAMPING opposes motion; it does NOT hold a pose."""
        for state in self._select(names) if names is not None else list(self.joints.values()):
            if state.live and not self.dry_run:
                bus = self._bus(state)
                if bus is not None:
                    try:
                        bus.set_mode(state.spec.device_id, Mode.DAMPING if damping else Mode.IDLE)
                    except Exception:
                        state.fault = "CAN_DISABLE_FAILED"
                    state.hardware_mode, state.mode_read_at = None, 0.0
            state.live = False
            state.goal_deg = state.command_deg = state.measured_deg
        self.message = "disabled"
        if names is None:
            self.motion_stopped = False
            self._stream_deadline = None

    def estop(self, reason: str = "E-STOP") -> None:
        self.disable(damping=True)
        self.estopped = True
        self.message = reason

    def reset_estop(self) -> None:
        if not self.estopped:
            self.message = "E-STOP was not latched; use resume for a motion hold"
            return
        self.estopped = False
        self.motion_stopped = False
        self.message = "E-STOP cleared; joints are still disabled"

    def close(self) -> None:
        if not self.dry_run:
            for state in self.joints.values():
                bus = self._bus(state)
                if bus is not None and state.spec.enabled:
                    try:
                        bus.set_mode(state.spec.device_id, Mode.IDLE)
                    except Exception:  # noqa: BLE001
                        pass
            for bus in {id(b): b for b in self.buses.values()}.values():
                bus.close()
        for state in self.joints.values():
            state.live = False

    # ------------------------------------------------------------------ targets
    def set_goal(self, name: str, degrees: float) -> float:
        return self.set_goals({name: degrees})[name]

    def set_goals(self, goals: dict[str, float], streaming=False, source="api") -> dict[str, float]:
        if self.motion_stopped or self.estopped:
            raise ValueError("stop is latched; resume/reset before supplying new goals")
        if streaming and self.config.require_fixed_zero_for_streaming and self.config.zero != "config":
            raise ValueError("streaming control requires calibrated fixed motor_zero")
        now = time.monotonic()
        if self._stream_deadline is not None and now < self._stream_deadline and self._command_owner is not None:
            if not streaming or source != self._command_owner:
                raise ValueError("another input owns streaming control; HOLD before changing source")
        checked = {}
        for name, degrees in goals.items():
            if name not in self.joints or not math.isfinite(degrees):
                raise ValueError(f"invalid joint target: {name}")
            checked[name] = self.joints[name].spec.clamp_deg(float(degrees))
        for name, value in checked.items():
            self.joints[name].goal_deg = value
        if checked:
            self._stream_deadline = now + self.config.command_timeout if streaming else None
            self._command_owner = source if streaming else None
        return checked

    def feedback_valid(self, state, now=None):
        now = time.monotonic() if now is None else now
        return state.live and (self.dry_run or (state.last_reply > 0 and now - state.last_reply <= self.config.feedback_timeout))

    def hold(self, reason="HOLD"):
        """Latch a motion stop; only verified healthy position loops keep holding.

        No POSITION/re-enable is sent. This is not a brake or guaranteed support.
        E-STOP retains its separate DAMPING policy.
        """
        self.motion_stopped = True
        self._stream_deadline = None
        self._command_owner = None
        now = time.monotonic()
        for state in self.joints.values():
            if not state.live:
                continue
            healthy = self.feedback_valid(state, now) and (self.dry_run or (
                state.hardware_mode == Mode.POSITION and state.hardware_error == 0))
            if healthy:
                # An SDO diagnostic failure alone must not release holding torque.
                verified = self.dry_run or (not state.health_failure and now - min(state.mode_read_at, state.error_read_at) < 1.5)
                target = state.measured_deg if verified else state.command_deg
                state.goal_deg = state.command_deg = state.spec.clamp_deg(target)
                state.command_velocity_dps = 0.0
            else:
                if not self.dry_run:
                    try:
                        self._bus(state).set_mode(state.spec.device_id, Mode.DAMPING)
                    except Exception:
                        pass
                state.live = False
                if not state.fault:
                    state.fault = "HOLD_UNVERIFIED"
        self.message = reason + "; healthy joints hold, unverified joints are not re-enabled"

    def resume_motion(self):
        if self.estopped:
            return False
        active = [s for s in self.joints.values() if s.live]
        if not active or any(s.fault and not s.live for s in self.joints.values()):
            return False
        for state in active:
            if not self.feedback_valid(state) or (not self.dry_run and (
                state.hardware_mode != Mode.POSITION or state.hardware_error != 0 or state.health_failure)):
                return False
        for state in active:
            state.goal_deg = state.command_deg = state.measured_deg
            state.command_velocity_dps = 0
            if state.fault == "TRACKING":
                state.fault = ""
        self.motion_stopped = False
        self._stream_deadline = None
        self.message = "motion resumed at current verified pose; supply a new goal"
        return True

    def _service_heartbeat(self):
        now = time.monotonic()
        if self.dry_run or self.estopped or now - self._last_feed < self.config.heartbeat_period:
            return
        self._last_feed = now
        for state in self.joints.values():
            if state.live:
                try:
                    self._bus(state).feed(state.spec.device_id)
                except Exception:
                    state.fault = "CAN_SEND_FAILED"
                    state.live = False

    def settled(self, tolerance_deg: float = 0.5) -> bool:
        return all(abs(s.command_deg - s.goal_deg) <= 1e-6 and abs(s.measured_deg - s.goal_deg) <= tolerance_deg
                   for s in self.joints.values() if s.live)

    # --------------------------------------------------------------------- loop
    def step(self, dt: float) -> None:
        started = time.monotonic()
        self.loop_stats["cycles"] += 1
        if self._last_step_at is not None:
            period = started - self._last_step_at
            self.loop_stats["last_period_ms"] = round(period * 1000, 3)
            if period > 1.5 / self.config.control_rate_hz:
                self.loop_stats["deadline_misses"] += 1
        self._last_step_at = started
        try:
            self._step(dt)
        finally:
            elapsed = (time.monotonic() - started) * 1000
            self.loop_stats["max_step_ms"] = max(self.loop_stats["max_step_ms"], round(elapsed, 3))

    def _step(self, dt):
        if not math.isfinite(dt) or dt <= 0 or not math.isfinite(self.max_speed_deg) or self.max_speed_deg <= 0:
            self.hold("INVALID_TIMING")
            return
        dt = min(dt, 2 / self.config.control_rate_hz)
        if not self.dry_run:
            self._poll_health()
        if self.estopped:
            return
        if self._stream_deadline is not None and time.monotonic() > self._stream_deadline:
            self.hold("COMMAND_TIMEOUT")
        self._service_heartbeat()
        if self.config.stop_on_joint_fault and any(s.fault == "CAN_SEND_FAILED" for s in self.joints.values()):
            self.hold("CAN_SEND_FAILED")
        active = [s for s in self.joints.values() if s.live]
        for state in active:
            delta = state.goal_deg - state.command_deg
            max_step = self.max_speed_deg * dt
            if self.config.max_acceleration_deg > 0:
                acc = self.config.max_acceleration_deg
                desired = math.copysign(min(self.max_speed_deg, math.sqrt(2 * acc * abs(delta))), delta) if delta else 0
                change = max(-acc * dt, min(acc * dt, desired - state.command_velocity_dps))
                state.command_velocity_dps += change
                move = state.command_velocity_dps * dt
                if delta * move >= 0 and abs(move) >= abs(delta):
                    move = delta
                    state.command_velocity_dps = 0
                state.command_deg += move
            else:
                state.command_deg += max(-max_step, min(max_step, delta))
            clamped = state.spec.clamp_deg(state.command_deg)
            if clamped != state.command_deg:
                state.command_velocity_dps = 0
            state.command_deg = clamped
        if self.dry_run:
            for state in active:
                previous = state.measured_deg
                state.measured_deg = state.command_deg
                state.velocity_dps = (state.measured_deg - previous) / dt
                state.last_reply = time.monotonic()
            return
        groups = {}
        for state in active:
            groups.setdefault(id(self._bus(state)), []).append(state)
        for states in groups.values():
            bus = self._bus(states[0])
            targets = {s.spec.device_id: s.spec.to_motor(math.radians(s.command_deg), s.zero) for s in states}
            try:
                if hasattr(bus, "exchange_many"):
                    replies = bus.exchange_many(targets, timeout=self.config.reply_timeout)
                else:  # deterministic test bus / legacy adapter
                    replies = {node: bus.exchange_pdo_2(node, target) for node, target in targets.items()}
            except Exception as error:
                for state in states:
                    state.fault = "CAN_IO " + type(error).__name__
                    state.live = False
                if self.config.stop_on_joint_fault:
                    self.hold("CAN_IO")
                continue
            for state in states:
                spec = state.spec
                reply = replies.get(spec.device_id)
                if reply is None:
                    state.missed += 1
                    if state.missed >= self.max_missed or not self.feedback_valid(state):
                        state.fault = "LOST"
                        state.live = False
                        try:
                            bus.set_mode(spec.device_id, Mode.DAMPING)
                        except Exception:
                            pass
                        if self.config.stop_on_joint_fault:
                            self.hold("JOINT_LOST: " + spec.name)
                    continue
                if not all(math.isfinite(v) for v in reply):
                    state.fault = "INVALID_FEEDBACK"
                    self.estop(spec.name + ": non-finite position/velocity")
                    return
                state.missed = 0
                state.last_reply = time.monotonic()
                state.measured_deg = math.degrees(spec.to_joint(reply[0], state.zero))
                state.velocity_dps = math.degrees(spec.direction * reply[1] / spec.gear_ratio)
                if abs(state.measured_deg - state.command_deg) > self.max_tracking_error_deg:
                    state.fault = "TRACKING"
                    reason = f"{spec.name}: measured {state.measured_deg:+.1f} vs command {state.command_deg:+.1f} deg"
                    if self.config.tracking_action == "hold":
                        if not self.motion_stopped:
                            self.hold("TRACKING: " + reason)
                        else:
                            # Holding failed too: do not keep issuing an unachievable target.
                            state.live = False
                            try:
                                bus.set_mode(spec.device_id, Mode.DAMPING)
                            except Exception:
                                pass
                    else:
                        self.estop(reason)
                    return

    def status(self) -> dict:
        now = time.monotonic()
        stale_after = max(1.0, 4 * len(self.joints) / self.config.control_rate_hz)
        def health_warning(state):
            if self.dry_run or not state.live:
                return state.health_failure
            if state.health_failure:
                return state.health_failure
            if now - min(state.mode_read_at, state.error_read_at) > stale_after:
                return "HEALTH_STALE"
            if state.hardware_error:
                return f"BOARD_ERROR 0x{state.hardware_error:08X}"
            return ""
        return {
            "dry_run": self.dry_run, "estop": self.estopped, "message": self.message,
            "units": {"measured_deg": "degrees", "command_deg": "degrees", "goal_deg": "degrees"},
            "output_encoder_available": False,
            "motion_stopped": self.motion_stopped, "timing": dict(self.loop_stats),
            "zero_mode": self.config.zero, "fixed_zero_configured": self.config.zero == "config",
            "fixed_zero_calibrated": self.config.zero == "config" and all(
                self.config.extra.get("commissioning", {}).get(f) is True for f in
                ("reference_pose_verified", "encoder_reference_persistent_verified")),
            "command_owner": self._command_owner,
            "max_speed_deg": self.max_speed_deg,
            "joints": {name: {
                "bus": s.spec.bus, "device_id": s.spec.device_id, "enabled": s.spec.enabled,
                "live": s.live, "fault": s.fault,
                "feedback_valid": self.feedback_valid(s, now), "feedback_source": "simulated" if self.dry_run else "rotor_encoder_scaled",
                "feedback_age_s": None if s.last_reply <= 0 else round(now - s.last_reply, 3),
                "bus_voltage_v": s.bus_voltage_v,
                "voltage_age_s": None if s.voltage_read_at <= 0 else round(now - s.voltage_read_at, 3),
                "health_warning": health_warning(s),
                "hardware_mode": s.hardware_mode,
                "hardware_mode_name": MODE_NAMES.get(s.hardware_mode, "UNKNOWN"),
                "hardware_error": s.hardware_error,
                "mode_age_s": round(now - s.mode_read_at, 3) if s.mode_read_at else None,
                "error_age_s": round(now - s.error_read_at, 3) if s.error_read_at else None,
                "firmware_version": s.firmware_version,
                "firmware_gear_ratio": s.firmware_gear_ratio,
                "current_limit_a": s.current_limit_a,
                "torque_constant_nm_per_a": s.torque_constant,
                "startup_bus_voltage_v": s.startup_bus_voltage,
                "configured_torque_limit": s.spec.torque_limit,
                "measured_deg": round(s.measured_deg, 3), "command_deg": round(s.command_deg, 3),
                "goal_deg": round(s.goal_deg, 3),
                "limits_deg": [s.spec.lower_deg, s.spec.upper_deg],
            } for name, s in self.joints.items()},
        }

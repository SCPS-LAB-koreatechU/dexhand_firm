"""Validate without motion, then use existing local controllers on commit."""
import math
import time


class ArmAdapter:
    def __init__(self, arm):
        self.arm = arm

    def validate(self, values):
        arm = self.arm
        if arm.estopped or arm.motion_stopped:
            raise ValueError("arm stop latched")
        if not arm.dry_run and arm.config.zero != "config":
            raise ValueError("live upper-body control requires measured fixed motor_zero")
        record = arm.config.extra.get("commissioning", {})
        if not arm.dry_run and any(record.get(f) is not True for f in
                                   ("reference_pose_verified", "encoder_reference_persistent_verified")):
            raise ValueError("fixed-zero commissioning record not verified")
        names = {n for n, s in arm.joints.items() if s.spec.enabled}
        if set(values) != names:
            raise ValueError("all enabled arm joints are required exactly once")
        for name, radians in values.items():
            state = arm.joints[name]
            if not state.live or state.fault or not arm.feedback_valid(state):
                raise ValueError(name + ": not live/fresh/healthy")
            if not arm.dry_run and (state.hardware_mode != 0x13 or state.hardware_error != 0 or state.health_failure):
                raise ValueError(name + ": POSITION/error verification failed")
            if not arm.dry_run and time.monotonic() - min(state.mode_read_at, state.error_read_at) > max(1, 4*len(arm.joints)/arm.config.control_rate_hz):
                raise ValueError(name + ": MODE/ERROR verification stale")
            if not state.spec.lower_deg <= math.degrees(radians) <= state.spec.upper_deg:
                raise ValueError(name + ": outside calibrated range")
        if arm._command_owner not in (None, "upper_body"):
            raise ValueError("arm already owned by another input")

    def apply(self, values):
        self.arm.set_goals({n: math.degrees(v) for n, v in values.items()}, streaming=True, source="upper_body")

    def hold(self, reason):
        self.arm.hold(reason)


class HandAdapter:
    def __init__(self, hand):
        self.hand = hand

    def validate(self, values):
        hand = self.hand
        if not hand.live or hand.held or hand.estopped or hand.fault or not hand.present:
            raise ValueError("hand not live/fresh/healthy; explicitly initialize and enable first")
        if not hand.dry_run and hand.protocol != 3:
            raise ValueError("upper-body integration requires hand protocol 3")
        if hand._command_owner not in (None, "upper_body"):
            raise ValueError("hand already owned by another input")
        problem = hand.config.check_pose(values)
        if problem:
            raise ValueError(problem)

    def apply(self, values):
        self.hand.set_goal(values, streaming=True, source="upper_body")

    def hold(self, reason):
        delivery = self.hand.hold()
        if not delivery.ok:
            raise ValueError(reason + ": HOLD not acknowledged: " + delivery.result)


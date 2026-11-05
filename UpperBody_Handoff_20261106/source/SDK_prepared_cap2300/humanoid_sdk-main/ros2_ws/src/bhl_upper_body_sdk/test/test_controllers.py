from pathlib import Path
import math

import pytest

from bhl_arm_sdk.config import load_config as arm_config
from bhl_arm_sdk.controller import ArmController
from bhl_hand_sdk.config import load_config as hand_config
from bhl_hand_sdk.controller import HandController
from bhl_upper_body_sdk.adapters import ArmAdapter, HandAdapter
from bhl_upper_body_sdk.contract import CommandBuilder
from bhl_upper_body_sdk.transaction import Coordinator, Endpoint

SRC = Path(__file__).resolve().parents[2]


def controllers():
    arm = ArmController(arm_config(SRC / "bhl_arm_sdk/config/bhl_arms.yaml"))
    cfg = hand_config(SRC / "bhl_hand_sdk/config/dexhand.yaml")
    left, right = HandController(cfg, "left"), HandController(cfg, "right")
    arm.enable()
    left.enable()
    right.enable()
    return arm, left, right


def test_actual_controllers_receive_full_pose_and_keep_own_limits():
    arm, left, right = controllers()
    adapters = dict(arms=ArmAdapter(arm), hand_left=HandAdapter(left), hand_right=HandAdapter(right))
    queue, results = [], []
    core = Coordinator(adapters, queue.append, results.append)
    endpoints = {p: Endpoint(p, a.validate, a.apply, a.hold, lambda r: core.reply(r, 10.01, 1.01))
                 for p, a in adapters.items()}
    angles = {n: math.radians(5 if s.spec.lower_deg >= 0 else -5) for n, s in arm.joints.items()}
    command = CommandBuilder().build(stamp_s=10, arms_rad=angles, hand_left=[.2]*15, hand_right=[.1]*15)
    core.submit(command, 10, 1)
    prepare = queue.pop()
    for e in endpoints.values():
        e.receive(prepare, 10, 1)
    assert all(s.goal_deg == 0 for s in arm.joints.values()) and left.goal == [0]*15
    commit = queue.pop()
    for e in endpoints.values():
        e.receive(commit, 10.01, 1.01)
    assert results[-1]["status"] == "ACCEPTED"
    assert all(s.goal_deg == pytest.approx(math.degrees(angles[n])) for n, s in arm.joints.items())
    assert left.goal == [.2]*15 and right.goal == [.1]*15
    arm.step(.02); left.step(.02); right.step(.02)
    assert 0 < left.command[0] < .2
    assert left.status()["measured_position"] is None and left.status()["physical_reached"] is None
    assert left.status()["last_delivery"]["result"] == "APPLIED"
    assert left.status()["feedback_source"] == "none"
    for e in endpoints.values():
        e.tick(10.26, 1.26)
    assert arm.motion_stopped and left.held and right.held


def test_arm_full_set_and_limits_are_rejected_before_mutation():
    arm, *_ = controllers()
    adapter = ArmAdapter(arm)
    with pytest.raises(ValueError, match="all enabled"):
        adapter.validate({next(iter(arm.joints)): 0})
    values = {n: 0 for n in arm.joints}
    values[next(iter(values))] = 100
    with pytest.raises(ValueError, match="range"):
        adapter.validate(values)
    assert all(s.goal_deg == 0 for s in arm.joints.values())


def test_live_arm_integration_requires_fixed_zero_and_verified_record():
    arm, *_ = controllers()
    arm.dry_run = False  # validation only; never steps or opens CAN
    adapter = ArmAdapter(arm)
    with pytest.raises(ValueError, match="fixed"):
        adapter.validate({n: 0 for n in arm.joints})
    arm.config.zero = "config"
    with pytest.raises(ValueError, match="record"):
        adapter.validate({n: 0 for n in arm.joints})


def test_independent_source_cannot_be_taken_over_by_upper_body():
    arm, left, _ = controllers()
    arm.set_goals({n: 0 for n in arm.joints}, streaming=True, source="planner")
    with pytest.raises(ValueError, match="owned"):
        ArmAdapter(arm).validate({n: 0 for n in arm.joints})
    left.set_goal([0]*15, streaming=True, source="target")
    with pytest.raises(ValueError, match="owned"):
        HandAdapter(left).validate([0]*15)

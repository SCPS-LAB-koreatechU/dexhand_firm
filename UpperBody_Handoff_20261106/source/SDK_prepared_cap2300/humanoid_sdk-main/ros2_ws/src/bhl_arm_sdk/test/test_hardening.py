import math
import struct
import time
import pytest

from bhl_arm_sdk.controller import ArmController
from bhl_arm_sdk.recoil_can import Function, Mode
from bhl_arm_sdk.ownership import NodeLease
from bhl_arm_sdk.coordination import PeerMonitor
from test_core import FakeMotor, live, two_joint_config, make_bus, reply, run


def test_tracking_stop_keeps_verified_position_loops_without_rezero():
    motors = {1: FakeMotor(follows=False), 2: FakeMotor()}
    cfg = two_joint_config(tracking_action="hold", stop_on_joint_fault=True)
    arm, _ = live(cfg, motors)
    arm.enable()
    zero = arm.joints["a"].zero
    arm.set_goal("a", 40)
    run(arm, 1)
    assert arm.motion_stopped and not arm.estopped
    assert all(m.mode == Mode.POSITION for m in motors.values())
    assert arm.joints["a"].command_deg == arm.joints["a"].measured_deg
    with pytest.raises(ValueError):
        arm.set_goal("a", 10)
    assert arm.resume_motion() and arm.joints["a"].zero == zero


def test_hold_never_reenables_faulty_mode_or_unverified_feedback():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, _ = live(two_joint_config(), motors)
    arm.enable()
    arm.joints["a"].last_reply = time.monotonic() - 1
    arm.hold()
    assert not arm.joints["a"].live and motors[1].mode == Mode.DAMPING
    assert arm.joints["b"].live and motors[2].mode == Mode.POSITION
    assert not arm.resume_motion()


def test_stream_timeout_latches_hold_and_keeps_healthy_torque():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, _ = live(two_joint_config(tracking_action="hold"), motors)
    arm.enable()
    arm.set_goals({"a": 10}, streaming=True)
    arm._stream_deadline = time.monotonic() - 1
    arm.step(.02)
    assert arm.motion_stopped and "COMMAND_TIMEOUT" in arm.message
    assert all(m.mode == Mode.POSITION for m in motors.values())


def test_atomic_targets_and_fixed_zero_streaming_guard():
    arm = ArmController(two_joint_config(require_fixed_zero_for_streaming=True), dry_run=True)
    arm.enable()
    with pytest.raises(ValueError):
        arm.set_goals({"a": 10, "missing": 4})
    assert arm.joints["a"].goal_deg == 0
    with pytest.raises(ValueError, match="fixed"):
        arm.set_goals({"a": 10}, streaming=True)


def test_acceleration_bound_and_large_dt_cannot_jump_to_goal():
    cfg = two_joint_config(max_acceleration_deg=60)
    arm = ArmController(cfg, dry_run=True)
    arm.enable(); arm.set_goal("a", 40)
    arm.step(.02)
    assert arm.joints["a"].command_deg == pytest.approx(.024)
    arm.step(10)
    assert arm.joints["a"].command_deg < 1


def test_pdo_batch_sends_all_nodes_before_receiving_and_rejects_old_queue():
    bus = make_bus([reply(Function.TRANSMIT_PDO_2, 1, struct.pack("<ff", 99, 0))])
    sends = []
    def send(message, timeout=None):
        sends.append(message)
        if len(sends) == 2:
            for node in (2, 1):
                msg = reply(Function.TRANSMIT_PDO_2, node, struct.pack("<ff", node, 0))
                msg.timestamp = time.time()
                bus._bus.replies.append(msg)
    bus._bus.send = send
    values = bus.exchange_many({1: .1, 2: .2})
    assert values == {1: (1, 0), 2: (2, 0)}
    assert len(sends) == 2


def test_freshness_expires_without_republishing_new_measurements():
    arm, _ = live(two_joint_config(), {1: FakeMotor(), 2: FakeMotor()})
    arm.enable()
    arm.joints["a"].last_reply -= 1
    status = arm.status()["joints"]["a"]
    assert not status["feedback_valid"] and status["feedback_age_s"] >= 1


def test_node_ownership_prevents_two_sdk_writers():
    lease = NodeLease("unit-test-node", [61])
    try:
        with pytest.raises(RuntimeError, match="already owned"):
            NodeLease("unit-test-node", [61])
        other = NodeLease("unit-test-node", [60]); other.close()
    finally:
        lease.close()
    retry = NodeLease("unit-test-node", [61]); retry.close()


def test_peer_status_expiry_faults_and_dry_run_mismatch():
    peers = PeerMonitor(["left", "right"], dry_run=False)
    assert "no status" in peers.issue(now=10)
    assert not peers.update("left", {}, now=10)
    for name in ("left", "right"):
        assert peers.update(name, {"hand": name, "dry_run": False}, now=10)
    assert not peers.issue(now=11)
    assert "expired" in peers.issue(now=12)
    peers.update("left", {"hand": "left", "dry_run": True}, now=10)
    assert "mismatch" in peers.issue(now=11)
    peers.update("left", {"hand": "left", "dry_run": False, "fault": "LOST"}, now=10)
    assert "faulted" in peers.issue(now=11)


def test_reset_estop_does_not_bypass_motion_hold_latch():
    arm = ArmController(two_joint_config(), dry_run=True)
    arm.enable(); arm.hold()
    arm.reset_estop()
    assert arm.motion_stopped
    assert arm.resume_motion()


def test_diagnostic_timeout_alone_does_not_release_held_position():
    arm, _ = live(two_joint_config(), {1: FakeMotor(), 2: FakeMotor()})
    arm.enable()
    arm.joints["a"].health_failure = "SDO_UNVERIFIED"
    arm.hold()
    assert arm.joints["a"].live and arm.joints["a"].hardware_mode == Mode.POSITION


def test_stream_owner_prevents_gui_or_other_source_from_overwriting_goal():
    arm = ArmController(two_joint_config(), dry_run=True)
    arm.enable()
    arm.set_goals({"a": 10}, streaming=True, source="planner")
    with pytest.raises(ValueError, match="owns"):
        arm.set_goal("a", 30)
    with pytest.raises(ValueError, match="owns"):
        arm.set_goals({"a": 30}, streaming=True, source="other")
    assert arm.joints["a"].goal_deg == 10
    arm.hold(); assert arm.resume_motion()
    arm.set_goals({"a": 20}, streaming=True, source="other")

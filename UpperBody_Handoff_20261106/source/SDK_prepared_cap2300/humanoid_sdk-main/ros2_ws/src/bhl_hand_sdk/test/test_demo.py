import math
from pathlib import Path

import pytest

from bhl_hand_sdk.camera_source import angle, fist_score
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.controller import HandController
from bhl_hand_sdk.demo import cycle, main
from bhl_hand_sdk.demo_session import DemoSession, CameraGate, CameraControl
from test_hand import config, rig  # noqa: F401


def test_ping_is_passive_and_cold_motion_needs_explicit_initial_target(rig):
    hand, fake = rig(protocol=3)
    session = DemoSession(hand)
    assert session.ping().version == 3
    assert not fake.modes and fake.pulses is None
    with pytest.raises(RuntimeError, match="INITIAL_TARGET"):
        session.arm()
    assert not fake.modes and fake.pulses is None
    assert session.arm(initial_open=True).ok
    assert fake.pulses == hand.spec.open_us
    session.close()
    assert fake.inhibited and fake.pwm_on


def test_existing_pwm_is_preserved_even_with_initial_open_flag(rig):
    hand, fake = rig(protocol=3)
    held = hand.spec.to_us([.2]*15)
    fake.pulses, fake.pwm_on = held[:], True
    session = DemoSession(hand)
    session.arm(initial_open=True)
    assert fake.pulses == held and hand.goal == hand.command
    assert fake.modes == [2, 0x13]
    session.stop("test")


def test_unconfirmed_set_immediately_requests_hold(rig):
    hand, fake = rig(protocol=3)
    session = DemoSession(hand)
    session.arm(initial_open=True)
    session.set_closed(True)
    fake.drop_status = 1
    with pytest.raises(RuntimeError, match="NO_REPLY"):
        session.step(.02)
    assert not session.active and fake.inhibited and fake.pwm_on
    assert session.last_hold.ok
    assert session.summary()["physical_reached"] is None


def test_wrong_profile_never_enables(rig):
    hand, fake = rig(protocol=3, tag="deadbeef")
    session = DemoSession(hand)
    with pytest.raises(RuntimeError, match="PROFILE_MISMATCH"):
        session.arm(initial_open=True)
    assert not session.attempted_enable and not fake.modes


def frame(number=1, captured_at=10.0, arm_at=10.1, closed=False, valid=True):
    return dict(kind="frame", frame_id=number, captured_at=captured_at, arm_at=arm_at,
                closed=closed, valid=valid)


def test_tracking_loss_latches_and_reacquisition_cannot_replay_old_arm_event():
    gate = CameraGate(.35)
    assert gate.update(frame(), 10.1) and gate.arm_requested(10.1)
    assert not gate.fresh_hand(10.5)
    gate.block(10.5, "LOST")
    assert gate.update(frame(2, 10.6), 10.6)
    assert not gate.arm_requested(10.6)
    assert gate.update(frame(3, 10.7, 10.7), 10.7)
    assert gate.arm_requested(10.7)
    assert not gate.arm_requested(10.7)  # a consumed key cannot re-arm twice


def test_fist_or_ambiguous_hand_cannot_arm_and_stale_inference_is_rejected():
    gate = CameraGate(.35)
    assert gate.update(frame(closed=True), 10.1)
    assert not gate.arm_requested(10.1)
    assert gate.update(frame(2, valid=False), 10.1)
    assert not gate.fresh_hand(10.1)
    assert not gate.update(frame(3, captured_at=9), 10.1)
    assert not gate.update(frame(1), 10.1)
    assert not gate.update(frame(4, captured_at=math.nan), 10.1)
    assert not gate.update(frame(4, closed=None), 10.1)
    assert gate.update(frame(5, captured_at=10.1, arm_at=9.0), 10.1)
    assert not gate.arm_requested(10.1)  # old key event cannot authorize new input


def test_camera_policy_drives_real_sdk_then_holds_on_missing_frames_without_auto_resume(rig):
    hand, fake = rig(protocol=3)
    hand.config.max_speed = .25
    session = DemoSession(hand)
    now = [10.0]
    control = CameraControl(session, initial_open=True, clock=lambda: now[0], log=lambda s: None)
    now[0] = 10.1
    control.tick(frame(captured_at=10.1))
    assert session.active and fake.pwm_on
    now[0] = 10.2
    control.tick(frame(2, captured_at=10.2, closed=True))
    assert hand.goal == [.25*v for v in hand.config.grips["power"]]
    held = fake.pulses[:]
    now[0] = 10.7
    control.tick()
    assert not session.active and fake.inhibited and fake.pulses == held
    now[0] = 10.8
    control.tick(frame(3, captured_at=10.8))
    assert not session.active  # old Space event cannot resume
    now[0] = 10.9
    control.tick(frame(4, captured_at=10.9, arm_at=10.9))
    assert session.active and not fake.inhibited
    session.stop("test")


def test_camera_input_expiring_during_enable_is_held_before_vision_goal(rig):
    hand, fake = rig(protocol=3)
    session = DemoSession(hand)
    now = [10.1]
    control = CameraControl(session, initial_open=True, clock=lambda: now[0], log=lambda s: None)
    original = session.arm
    def slow_arm(**kwargs):
        result = original(**kwargs)
        now[0] += 1
        return result
    session.arm = slow_arm
    control.tick(frame(captured_at=10.1))
    assert not session.active and fake.inhibited
    assert session.reason == "FRAME_EXPIRED_DURING_ENABLE"


def points(closed=False):
    values = [(0.0, 0.0, 0.0)]*21
    for x, index in enumerate((5, 9, 13, 17)):
        if closed:
            values[index:index+4] = [(x, 0, 0), (x, 1, 0), (x, 1, 1), (x, .5, 1)]
        else:
            values[index:index+4] = [(x, 0, 0), (x, 1, 0), (x, 2, 0), (x, 3, 0)]
    return values


def test_geometry_open_closed_and_rotation_scale_translation_invariance():
    assert fist_score(points(False)) == 0
    assert fist_score(points(True)) > .6
    transformed = [(3*z+8, 3*x-1, 3*y+5) for x, y, z in points(True)]
    assert fist_score(transformed) == pytest.approx(fist_score(points(True)))
    with pytest.raises(ValueError):
        fist_score([(0, 0, 0)]*21)
    with pytest.raises(ValueError):
        angle((0, 0, 0), (0, 0, 0), (1, 2, 3))


def test_limited_cycle_uses_existing_slew_and_ends_in_hold_without_releasing_pwm():
    config = load_config(Path(__file__).resolve().parents[1]/"config/dexhand.yaml")
    config.max_speed = .25
    hand = HandController(config, "right", dry_run=True)
    session = DemoSession(hand)
    now = [1.0]
    def clock(): return now[0]
    def sleep(dt): now[0] += max(dt, .001)
    cycle(session, 1, .1, initial_open=True, clock=clock, sleep=sleep, log=lambda s: None)
    assert hand.held and not hand.live and session.results["APPLIED"] > 100
    assert hand.command == config.grips["open"]
    assert session.last_hold.ok and session.reason == "CYCLE_COMPLETE"


@pytest.mark.parametrize("arguments", [
    ["--level", "nan"], ["--level", ".9"], ["--speed", "-1"], ["--cycles", "0"],
    ["--camera-timeout", ".01"], ["--headless"], ["--enable"],
])
def test_invalid_cli_options_fail_before_opening_can(arguments):
    with pytest.raises(SystemExit) as result:
        main(["right", "camera", *arguments])
    assert result.value.code == 2

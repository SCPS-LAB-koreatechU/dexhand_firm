import time
import pytest
import yaml

from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.hand_can import Boot, Result, Status, Flag
from test_hand import CONFIG, config, rig


def test_unknown_held_pose_is_read_back_without_opening(rig):
    ctrl, fake = rig(protocol=3)
    held = ctrl.spec.to_us(ctrl.config.grips["power"])
    fake.pulses, fake.pwm_on, fake.applied = held[:], True, 12
    ctrl._explicit_goal = False
    result = ctrl.enable()
    assert result.ok and fake.pulses == held
    assert ctrl.command_us == held and ctrl.goal == ctrl.command
    assert fake.modes == [2, 0x13]  # HOLD snapshot, then RESUME; no START/open


def test_explicit_new_goal_slews_from_readback_instead_of_jumping(rig):
    ctrl, fake = rig(protocol=3)
    held = ctrl.spec.to_us(ctrl.config.grips["power"])
    fake.pulses, fake.pwm_on = held[:], True
    ctrl.set_grip("open")
    assert ctrl.enable().ok and fake.pulses == held
    ctrl.step(.02)
    assert ctrl.command[0] == pytest.approx(.96)


def test_cold_hand_has_no_implicit_initial_motion(rig):
    ctrl, fake = rig(protocol=3)
    ctrl._explicit_goal = False
    assert ctrl.enable().result == "INITIAL_TARGET_REQUIRED"
    assert not fake.modes and fake.pulses is None


def test_applied_pwm_is_correlated_with_pc_command_and_survives_lost_reply(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    ctrl.command_context = dict(session="test-session", sequence=10)
    ctrl.set_goal([.2]*15)
    assert ctrl.step(.02).ok
    applied = ctrl.status()["last_applied_command"]
    assert applied["context"]["sequence"] == 10 and applied["pwm_us"] == fake.pulses
    assert applied["source"] == "firmware_ack"
    # A new queued command must not masquerade as a confirmed PWM application.
    fake.drop_status = 1
    ctrl.command_context = dict(session="test-session", sequence=11)
    assert ctrl.step(.02).result == "NO_REPLY"
    assert ctrl.status()["last_applied_command"]["context"]["sequence"] == 10
    ctrl._note(Boot(61, 999, 0, time.monotonic()))
    assert ctrl.status()["last_applied_command"] is None


def test_resume_adopts_held_pwm_without_replaying_a_lost_host_target(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    held = fake.pulses[:]
    fake.lose_group = 2
    ctrl.set_goal([.3]*15)
    assert ctrl.step(.02).result == "NO_REPLY"
    assert ctrl.command_us != held and fake.pulses == held
    ctrl.hold()
    fake.lose_group = None
    assert ctrl.resume().ok
    assert ctrl.goal == ctrl.command and ctrl.command_us == held
    assert ctrl.step(.02).ok and fake.pulses == held


def test_duplicate_old_ack_cannot_reset_misses_or_count_as_late(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    frame = Status(61, 1, Result.APPLIED, Flag.PWM_ON, 255, 1, 0, 0, time.monotonic())
    ctrl.missed = 7
    for _ in range(20):
        assert not ctrl._note(frame)
    assert ctrl.missed == 7 and ctrl.stats["late"] == 0


def test_boot_change_stops_without_automatic_replay(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    ctrl.boot_id = 1
    ctrl._note(Boot(61, 2, 0, time.monotonic()))
    assert not ctrl.live and ctrl.fault == "DEVICE_REBOOT"
    assert not ctrl._explicit_goal and not ctrl._known_command


def test_counter_reset_is_not_65036_lost_acknowledgements(rig):
    ctrl, fake = rig(protocol=3)
    ctrl._last_applied_count = 500
    ctrl._note(Status(61, 0, Result.HEARTBEAT, 0, 255, 0, 0, 0, time.monotonic()))
    assert ctrl.stats["acks_lost"] == 0 and ctrl.fault == "COUNTER_RESET"


@pytest.mark.parametrize("key,value", [("max_speed", -2), ("max_speed", float("nan")),
    ("control_rate_hz", 0), ("reply_timeout", -1), ("max_missed", 0), ("command_timeout", 0)])
def test_invalid_control_config_is_rejected(tmp_path, key, value):
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    data["control"][key] = value
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(path)


def test_stream_expiry_holds_grip_and_requires_explicit_resume(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    ctrl.set_goal([0] * 15, streaming=True)
    ctrl._stream_deadline = time.monotonic() - 1
    assert ctrl.step(.02).result == "COMMAND_TIMEOUT"
    assert not ctrl.live and fake.inhibited and fake.pwm_on


def test_internal_command_is_validated_again_before_transmit(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    ctrl.command[12] = ctrl.command[14] = .6
    before = fake.applied
    assert ctrl._send_set().result == "INVALID_COMMAND"
    assert fake.applied == before


def test_off_requires_a_new_deliberate_initial_target(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    assert ctrl.off().ok
    assert ctrl.enable().result == "INITIAL_TARGET_REQUIRED"
    assert not fake.pwm_on


def test_clear_flags_control_is_correlated_without_pose_change(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    held = list(fake.pulses)
    assert ctrl.clear_flags().ok and fake.modes[-1] == 3
    assert fake.pulses == held


def test_operator_can_acknowledge_recovered_fault_without_moving_hand(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    held = list(fake.pulses)
    ctrl._fail("LOST", "test link outage")
    ctrl.estopped = True
    assert ctrl.reset_estop()
    assert not ctrl.live and not ctrl.fault and not ctrl.estopped
    assert fake.pulses == held
    assert ctrl.resume().ok and fake.pulses == held


def test_cannot_acknowledge_fault_when_hand_is_still_absent(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    ctrl._fail("LOST", "test")
    fake.silent = True
    assert not ctrl.reset_estop() and ctrl.fault == "LOST"


def test_stream_source_lease_prevents_fraction_us_and_manual_conflicts(rig):
    ctrl, fake = rig(protocol=3)
    assert ctrl.enable().ok
    ctrl.set_goal([0] * 15, streaming=True, source="target")
    with pytest.raises(ValueError, match="owns"):
        ctrl.set_goal_us(ctrl.spec.open_us, streaming=True, source="target_us")
    with pytest.raises(ValueError, match="owns"):
        ctrl.set_grip("power")
    assert ctrl.goal == [0] * 15
    ctrl.hold(); assert ctrl.resume().ok
    ctrl.set_goal_us(ctrl.spec.open_us, streaming=True, source="target_us")

"""Regression tests for loss of holding mode and untagged SDO replies."""

import math
import struct

import pytest

from bhl_arm_sdk.config import load_config
from bhl_arm_sdk.diagnostics import WARN, diagnostics
from bhl_arm_sdk.recoil_can import Function, Mode, Param
from test_core import CONFIG, FakeMotor, live, make_bus, reply, run, two_joint_config


def test_target_arrival_keeps_position_control_and_current_cap_is_read_only():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, _ = live(two_joint_config(), motors)
    arm.enable()
    arm.set_goal("a", 10)
    run(arm, 2)
    assert arm.settled() and arm.joints["a"].live
    assert motors[1].mode == Mode.POSITION
    assert arm.status()["joints"]["a"]["current_limit_a"] == 22.0
    assert motors[1].current_limit == 22.0


def test_board_damping_is_detected_even_while_pdo_replies_continue():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, _ = live(two_joint_config(), motors)
    arm.enable()
    motors[1].mode = Mode.DAMPING
    motors[1].error = 0x40
    run(arm, 0.12)
    state = arm.status()["joints"]["a"]
    assert not state["live"] and state["fault"] == "MODE_LOST DAMPING"
    assert state["hardware_error"] == 0x40
    assert motors[1].mode == Mode.DAMPING  # no automatic re-arm
    assert motors[2].mode == Mode.POSITION and arm.joints["b"].live


@pytest.mark.parametrize("field,value,fault", [
    ("gear_ratio", -15, "GEAR_MISMATCH"),
    ("firmware", 0x1234, "FIRMWARE_UNVERIFIED"),
    ("current_limit", float("nan"), "PREFLIGHT_READ"),
    ("error", 0x2000, "BOARD_ERROR"),
    ("accept_gains", False, "GAIN_READBACK"),
])
def test_unverified_units_or_settings_prevent_enable(field, value, fault):
    motors = {1: FakeMotor(), 2: FakeMotor()}
    setattr(motors[1], field, value)
    arm, _ = live(two_joint_config(), motors)
    assert arm.enable() == ["b"]
    assert arm.joints["a"].fault.startswith(fault)
    assert motors[1].mode == Mode.IDLE


def test_health_timeout_warns_without_releasing_position_hold():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, _ = live(two_joint_config(), motors)
    arm.enable()
    motors[1].sdo_answers = False
    run(arm, 0.12)
    status = arm.status()
    assert status["joints"]["a"]["live"]
    assert "SDO_UNVERIFIED" in status["joints"]["a"]["health_warning"]
    assert motors[1].mode == Mode.POSITION
    assert diagnostics(status)[0]["level"] == WARN


def test_pdo_reply_does_not_prove_position_enable_was_accepted():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, buses = live(two_joint_config(), motors)
    bus = buses["can0"]
    original_set_mode = bus.set_mode
    def ignore_enable(node, mode):
        if not (node == 1 and mode == Mode.POSITION):
            original_set_mode(node, mode)
    bus.set_mode = ignore_enable
    assert arm.enable() == ["b"]
    assert arm.joints["a"].fault == "POSITION_NOT_CONFIRMED"
    assert not arm.joints["a"].live


def test_error_register_is_reported_without_auto_clear_or_torque_release():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, _ = live(two_joint_config(), motors)
    arm.enable()
    motors[1].error = 0x40
    run(arm, 0.12)
    assert "00000040" in arm.status()["joints"]["a"]["health_warning"]
    assert motors[1].error == 0x40 and motors[1].mode == Mode.POSITION


def test_board_errors_remain_readable_after_estop_without_reenabling():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, buses = live(two_joint_config(), motors)
    arm.enable()
    arm.estop()
    feeds = buses["can0"].feeds
    motors[1].error = 0x2000
    run(arm, 0.12)
    assert arm.status()["joints"]["a"]["hardware_error"] == 0x2000
    assert all(m.mode == Mode.DAMPING for m in motors.values())
    assert buses["can0"].feeds == feeds and arm.estopped


def test_nan_feedback_triggers_existing_estop_instead_of_publishing_it():
    motors = {1: FakeMotor(follows=False), 2: FakeMotor()}
    arm, _ = live(two_joint_config(), motors)
    arm.enable()
    motors[1].position = float("nan")
    arm.step(0.02)
    assert arm.estopped and arm.joints["a"].fault == "INVALID_FEEDBACK"
    assert math.isfinite(arm.joints["a"].measured_deg)


def test_sdo_timeout_quarantines_node_so_late_mode_cannot_become_current():
    bus = make_bus()
    assert bus.read_u32(1, Param.MODE) is None
    bus._bus.replies.append(reply(Function.TRANSMIT_SDO, 1, struct.pack("<I", Mode.POSITION)))
    sent_before = len(bus._bus.sent)
    assert bus.read_f32(1, Param.CURRENT_LIMIT) is None
    assert len(bus._bus.sent) == sent_before
    # PDO control remains available despite the diagnostic-session failure.
    bus._bus.replies.append(reply(Function.TRANSMIT_PDO_2, 1, struct.pack("<ff", 1, 0)))
    assert bus.exchange_pdo_2(1, 1) == (1, 0)


def test_sdo_drains_stale_reply_and_accepts_only_response_after_request():
    bus = make_bus([reply(Function.TRANSMIT_SDO, 1, struct.pack("<I", Mode.IDLE))])
    original_send = bus._bus.send
    def respond(message, timeout=None):
        original_send(message)
        bus._bus.replies.append(reply(Function.TRANSMIT_SDO, 1, struct.pack("<I", Mode.POSITION)))
    bus._bus.send = respond
    assert bus.read_u32(1, Param.MODE) == Mode.POSITION


def test_external_sdo_request_makes_correlation_unverified():
    bus = make_bus([reply(Function.RECEIVE_SDO, 1, struct.pack("<BH", 0x40, Param.ERROR))])
    assert bus.read_u32(1, Param.MODE) is None
    assert not bus._bus.sent
    assert "another SDO client" in bus.sdo_failures[1]


def test_reference_profile_generates_original_joint_torque_for_same_error():
    config = load_config(CONFIG.with_name("bhl_arms_berkeley_reference.yaml"))
    for spec in config.joints:
        # Compare both full PD expressions, not only the literal config numbers.
        for error, velocity in ((0.01, 0.02), (-0.03, 0.1)):
            rotor_torque = spec.kp * spec.gear_ratio * error - spec.kd * spec.gear_ratio * velocity
            joint_torque = rotor_torque * spec.gear_ratio
            assert joint_torque == pytest.approx(30 * error - 2 * velocity)
        assert spec.torque_limit * spec.gear_ratio == pytest.approx(2.0)
    assert config.extra["payload"]["mass_kg"] is None

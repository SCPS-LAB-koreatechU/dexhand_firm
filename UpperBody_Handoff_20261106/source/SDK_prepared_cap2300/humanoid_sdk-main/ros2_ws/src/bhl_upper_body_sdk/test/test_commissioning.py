import math
import pytest

from bhl_upper_body_sdk.commissioning import fixed_zero_config, audit, model_issues


def sample():
    config = dict(defaults=dict(gear_ratio=15), joints={
        "a": dict(direction=-1, lower_deg=-90, upper_deg=90),
        "b": dict(direction=1, lower_deg=-90, upper_deg=90)})
    samples = dict(reference_pose_verified=True, encoder_reference_persistent_verified=True,
                   joints=dict(a=dict(motor_position=5, joint_rad=.2),
                               b=dict(motor_position=-2, joint_rad=-.1)))
    return config, samples


def test_fixed_zero_conversion_preserves_gains_and_marks_measured_provenance():
    config, samples = sample()
    config["joints"]["a"]["kp"] = .3
    result = fixed_zero_config(config, samples)
    assert result["joints"]["a"]["motor_zero"] == pytest.approx(8)
    assert result["joints"]["b"]["motor_zero"] == pytest.approx(-.5)
    assert result["joints"]["a"]["kp"] == .3 and "motor_zero" not in config["joints"]["a"]
    assert result["control"]["require_fixed_zero_for_streaming"]
    assert audit(result, {})["fixed_zero_ready"]
    assert not audit(result, {})["gravity_compensation_implemented"]


@pytest.mark.parametrize("field", ["reference_pose_verified", "encoder_reference_persistent_verified"])
def test_cannot_invent_verified_zero(field):
    config, samples = sample()
    samples[field] = False
    with pytest.raises(ValueError, match="verify"):
        fixed_zero_config(config, samples)


def test_missing_nan_or_out_of_range_measurement_fails():
    for data in ({}, {"motor_position": math.nan, "joint_rad": 0}, {"motor_position": 0, "joint_rad": 4}):
        config, samples = sample()
        samples["joints"]["a"] = data
        with pytest.raises(ValueError):
            fixed_zero_config(config, samples)


def test_model_records_remain_unready_until_real_fields_exist(tmp_path):
    config, _ = sample()
    assert model_issues({}, config)
    (tmp_path / "robot.urdf").write_text('<robot name="test"><link name="test_mount"/><joint name="a"/><joint name="b"/></robot>')
    payload = dict(mass_kg=.5, com_m=[0, 0, .1], mount_frame="test_mount", verified=True)
    model = dict(urdf="robot.urdf", payloads=dict(left=payload, right=payload))
    assert not model_issues(model, config, tmp_path)
    (tmp_path / "robot.urdf").write_text('<robot><joint name="a"/></robot>')
    assert "misses" in model_issues(model, config, tmp_path)[0]

from dataclasses import replace
from pathlib import Path
import pytest
from bhl_hand_sdk.config import load_config


def test_raw_receive_ceiling_is_independent_of_calibrated_motion():
    cfg=load_config(Path(__file__).resolve().parents[1]/'config/dexhand_camera.yaml')
    spec=cfg.hand('right')
    assert spec.hardware_max_us==2300 and spec.hardware_high_us==[2300]*15
    assert spec.to_us([0.]*15)==spec.open_us
    assert spec.to_us([1.]*15)==spec.close_us
    assert spec.close_us[5]==1750 and spec.high_us[5]==1750
    raw=spec.open_us[:];raw[5]=2300
    assert spec.validate_hardware_pulses(raw)==raw
    with pytest.raises(ValueError):spec.to_fractions(raw)
    raw[5]=2301
    with pytest.raises(ValueError):spec.validate_hardware_pulses(raw)
    with pytest.raises(ValueError):replace(spec,hardware_max_us=2299)


def test_legacy_profiles_retain_per_channel_receive_bounds():
    cfg=load_config(Path(__file__).resolve().parents[1]/'config/dexhand_camera.yaml')
    left=cfg.hand('left')
    assert left.hardware_max_us is None and left.hardware_high_us==left.high_us
    raw=left.open_us[:];raw[0]=left.high_us[0]+1
    with pytest.raises(ValueError):left.validate_hardware_pulses(raw)

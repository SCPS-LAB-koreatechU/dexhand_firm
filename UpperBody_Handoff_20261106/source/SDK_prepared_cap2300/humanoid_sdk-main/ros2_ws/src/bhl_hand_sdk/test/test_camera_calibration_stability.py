import copy,time
from types import SimpleNamespace
import pytest
from bhl_hand_sdk.hand_tracking_gui import TrackingGui
from bhl_hand_sdk.hand_retarget import GROUPS,DIMS


def state(ages):
    now=time.monotonic();features={g:[0.]*DIMS[g] for g in GROUPS}
    return SimpleNamespace(max_frame_age=.25,stable=[(now-a,copy.deepcopy(features)) for a in ages],features=features,latest={'captured_at':now},group=SimpleNamespace(get=lambda:'all'))


def test_stability_uses_elapsed_time_instead_of_desktop_frame_count():
    obj=state([.6,.45,.3,.15,0.])
    assert TrackingGui.stable_features(obj)==obj.features


@pytest.mark.parametrize('ages',[[.1,.075,.05,.025,0.],[.9,.8,.7,.1,0.],[.6,.4,.2,0.]])
def test_short_interval_gaps_or_too_few_observations_are_not_stable(ages):
    with pytest.raises(ValueError):TrackingGui.stable_features(state(ages))


def test_moving_pose_is_not_recorded_as_stable():
    obj=state([.6,.45,.3,.15,0.]);obj.stable[-1][1]['index'][0]=1.
    with pytest.raises(ValueError):TrackingGui.stable_features(obj)

import json
import pytest
from bhl_hand_sdk.four_finger_trial import FINGERS,fresh_sample,pose_matches,trial_goal,TrackingContinuity,recalibrate_finger,middle_lower_test_goal
from bhl_hand_sdk.hand_retarget import GROUPS,synchronized_step
from bhl_hand_sdk.camera_source import publish_sample


def features(value):return {g:[value,value,.45,0.] for g in FINGERS}


def test_independent_full_stroke_and_three_tendon_arrival():
    opened,closed=features(.1),features(.8)
    f=features(.1);f['ring']=[.8,.8,.45,0.]
    goal=trial_goal(f,opened,closed)
    assert [goal[i] for i in GROUPS['ring']]==[1.]*3
    assert all(goal[i]==0 for g in ('index','middle','pinky','thumb') for i in GROUPS[g])
    assert trial_goal(opened,opened,closed)==[0.]*15
    assert trial_goal(features(2),opened,closed)==[1.]*12+[0.]*3
    stepped,_=synchronized_step([0.]*15,goal,.02,.25)
    assert max(stepped)<=.005+1e-12
    assert len({stepped[i] for i in GROUPS['ring']})==1
    with pytest.raises(ValueError):trial_goal(f,opened,opened)


def test_stale_wrong_hand_and_ambiguous_input_are_rejected():
    d=dict(captured_at=1.,detection_valid=True,source_hand='Right',handedness_score=.99,recognized_features=features(.1))
    assert fresh_sample(d,1.3)
    for changed in ({'captured_at':0.},{'source_hand':'Left'},{'detection_valid':False},{'handedness_score':.5},{'input_role':'robot_observation'}):
        assert not fresh_sample({**d,**changed},1.3)
    assert pose_matches(features(.1)) and pose_matches(features(.8),True)
    assert not pose_matches(features(.8))


def test_control_sample_latest_write_is_independent_of_gui(tmp_path):
    path=tmp_path/'camera.json'
    publish_sample(path,{'frame_id':1})
    publish_sample(path,{'frame_id':2,'detection_valid':False})
    assert json.loads(path.read_text())=={'frame_id':2,'detection_valid':False}
    assert not path.with_suffix('.tmp').exists()


def test_one_missing_frame_freezes_then_recovers_without_rearming():
    gate=TrackingContinuity()
    valid=dict(captured_at=1.,detection_valid=True,source_hand='Right',handedness_score=.99,recognized_features=features(.1))
    assert gate.update(valid,1.1)[0]=='TRACK'
    assert gate.update({'detection_valid':False},1.25)[0]=='FREEZE'
    assert gate.update({'detection_valid':False},1.40)[0]=='FREEZE'
    assert gate.update({**valid,'captured_at':1.4},1.45)[0]=='TRACK'


def test_persistent_loss_and_wrong_hand_latch_hold_without_auto_resume():
    valid=dict(captured_at=1.,detection_valid=True,source_hand='Right',handedness_score=.99,recognized_features=features(.1))
    gate=TrackingContinuity();assert gate.update(valid,1.1)[0]=='TRACK'
    assert gate.update({},1.2)[0]=='FREEZE'
    assert gate.update({},1.51)[0]=='HOLD'
    assert gate.update({**valid,'captured_at':1.55},1.6)[0]=='HOLD'
    gate=TrackingContinuity();gate.update(valid,1.1);gate.update({},1.2)
    assert gate.update({**valid,'captured_at':1.5},1.51)[0]=='HOLD'
    gate=TrackingContinuity();gate.update(valid,1.1)
    assert gate.update({**valid,'source_hand':'Left'},1.2)==('HOLD','WRONG_HAND')
    assert TrackingContinuity().update({},1.1)[0]=='HOLD'


def test_middle_reference_can_reach_full_goal_without_changing_other_fingers():
    opened,closed=features(.1),features(.8)
    current_open,current_closed=features(.2),features(.7)
    assert trial_goal(current_closed,opened,closed)[4]<.9
    new_open,new_closed=recalibrate_finger(opened,closed,current_open,current_closed,'middle')
    assert trial_goal(current_closed,new_open,new_closed)[4:6]==[1.,1.]
    assert new_open['middle']==current_open['middle'] and new_closed['middle']==current_closed['middle']
    for group in ('index','ring','pinky'):
        assert new_open[group]==opened[group] and new_closed[group]==closed[group]
    assert opened==features(.1) and closed==features(.8)
    with pytest.raises(ValueError):recalibrate_finger(opened,closed,current_open,current_open,'middle')


def test_single_finger_calibration_does_not_require_other_fingers_to_close():
    f=features(.1);f['middle']=[.7,.7,.45,0.]
    assert pose_matches(f,True,groups=('middle',))
    assert not pose_matches(f,True)
    with pytest.raises(ValueError):pose_matches(f,groups=())


def test_middle_only_trial_excludes_other_finger_targets_even_in_fist():
    goal=trial_goal(features(.8),features(.1),features(.8),groups=('middle',))
    assert [goal[i] for i in GROUPS['middle']]==[1.,1.,1.]
    assert all(goal[i]==0 for g in ('index','ring','pinky','thumb') for i in GROUPS[g])
    with pytest.raises(ValueError):trial_goal(features(.8),features(.1),features(.8),groups=())


def test_lower_motor_isolation_has_a_finite_return_to_open():
    for t,v in [(0,0),(2,0),(4,.5),(6,1),(8,1),(10,.5),(12,0),(14,0),(20,0)]:
        goal=middle_lower_test_goal(t)
        assert goal[5]==v and all(x==0 for i,x in enumerate(goal) if i!=5)
    with pytest.raises(ValueError):middle_lower_test_goal(float('nan'))

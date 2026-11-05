import pytest
from bhl_hand_sdk.index_trial import trial_goal,fresh_index_sample,index_is_open,INDEX


def test_trial_moves_only_index_and_pairs_the_main_tendons():
    ref={'index':[.1,.1,.1,0.]}
    goal=trial_goal({'index':[2.,2.,2.,1.]},ref)
    assert all(goal[i]==0 for i in range(15) if i not in INDEX)
    assert [goal[i] for i in INDEX]==[.15,.15,.15]
    assert trial_goal(ref,ref)==[0.]*15
    assert trial_goal({'index':[-1.,-1.,-1.,0.]},ref)==[0.]*15
    with pytest.raises(ValueError):trial_goal(ref,ref,.5)


def test_trial_rejects_stale_ambiguous_and_wrong_hand_input():
    d={'detection_valid':True,'source_hand':'Right','captured_at':1.,'recognized_features':{'index':[0.,0.,0.,0.]}}
    assert fresh_index_sample(d,1.1) and index_is_open(d['recognized_features'])
    assert not fresh_index_sample(d,1.4)
    assert not fresh_index_sample({**d,'source_hand':'Left'},1.1)
    assert not fresh_index_sample({**d,'detection_valid':False},1.1)
    assert not fresh_index_sample({**d,'recognized_features':{'index':[float('nan')]*4}},1.1)


def test_live_open_bias_and_frame_interval_keep_bounded_guards():
    features={'index':[.20,.09,.45,0.]}
    d={'detection_valid':True,'source_hand':'Right','captured_at':1.,'recognized_features':features}
    assert index_is_open(features)
    assert fresh_index_sample(d,1.30)
    assert not fresh_index_sample(d,1.36)
    assert not index_is_open({'index':[.40,.09,.45,0.]})
    assert not index_is_open({'index':[.20,.40,.45,0.]})
    assert not index_is_open({'index':[.20,.09,.60,0.]})
    assert trial_goal(features,features)==[0.]*15

import copy,json,math
from pathlib import Path
import pytest
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.controller import HandController
from bhl_hand_sdk.hand_retarget import CorrespondenceProfile,GROUPS,extract_features,synchronized_step
from bhl_hand_sdk.hand_tracking_gui import synthetic_hand

CONFIG=Path(__file__).resolve().parents[1]/'config/dexhand_camera.yaml'

def trained(verified=True):
    spec=load_config(CONFIG).hand('right');p=CorrespondenceProfile(spec)
    for name,level in (('펴기',0.),('중간',.4),('접기',.8)):
        f=extract_features(synthetic_hand(level,thumb_across=level*.3,thumb_fold=level))
        v=[level]*15;v[-3:]=[level*.4,level*.8,level*.4]
        for group in GROUPS:p.add(group,name,f,spec.to_us(v),verified)
    return p

def test_features_ignore_camera_rotation_translation_and_hand_size():
    p=synthetic_hand(.4,.1,.2,.3);a=extract_features(p)
    angle=.8;c,s=math.cos(angle),math.sin(angle)
    moved=[[2*(c*x-s*y)+1,2*(s*x+c*y)-2,2*z+.3] for x,y,z in p]
    b=extract_features(moved)
    for group in GROUPS:assert b[group]==pytest.approx(a[group],abs=1e-7)
    with pytest.raises(ValueError):extract_features([[0,0,0]]*21)
    bad=copy.deepcopy(p);bad[0][0]=float('nan')
    with pytest.raises(ValueError):extract_features(bad)

def test_common_phase_arrives_together_despite_different_pwm_strokes_and_directions():
    cfg=load_config(CONFIG);hand=HandController(cfg,'right',dry_run=True)
    start=[0.]*15;goal=[0.]*15;goal[0]=1.;goal[1]=.35;goal[11]=.6
    hand.set_goal(start);hand.enable();hand.set_goal(goal)
    phases=[]
    for _ in range(25):
        assert hand.step(.02).ok
        ratios=[hand.command[i]/goal[i] for i in (0,1,11)]
        assert max(ratios)-min(ratios)<1e-9
        phases.append(ratios[0])
    assert phases[-2]<1 and phases[-1]==pytest.approx(1.)
    assert hand.command_us[0]==1775 and hand.command_us[1]==1385 and hand.command_us[11]==1280

def test_measured_slow_motor_sets_the_common_duration_and_preserves_thumb_constraint():
    start=[0.]*15;start[12]=.8
    goal=[0.]*15;goal[14]=.8;times=[.5]*15;times[14]=2.
    cmd=start[:]
    for _ in range(79):
        cmd,phase=synchronized_step(cmd,goal,.02,2.,times)
        assert cmd[12]+cmd[14]<=1+1e-9
        assert cmd[12]>.0001
    cmd,phase=synchronized_step(cmd,goal,.02,2.,times)
    assert cmd==pytest.approx(goal,abs=1e-9)

def test_anchor_interpolation_is_exact_at_samples_and_bounded_between_samples(tmp_path):
    p=trained();f=extract_features(synthetic_hand(.4,thumb_across=.12,thumb_fold=.4))
    values,joints,support=p.predict(f,verified_only=True)
    expected=p.spec.to_fractions(p.spec.to_us([.4]*12+[.16,.32,.16]))
    assert values==pytest.approx(expected,abs=.001)
    assert len(joints)==21 and max(support.values())<1e-8
    middle=extract_features(synthetic_hand(.6,thumb_across=.18,thumb_fold=.6))
    out,_,_=p.predict(middle)
    assert all(0<=x<=.801 for x in out) and out[12]+out[14]<=1
    path=tmp_path/'profile.json';p.save(path);again=CorrespondenceProfile.load(path,p.spec)
    assert again.predict(f)[0]==pytest.approx(values)
    broken=json.loads(path.read_text());broken['open_us'][0]+=1;path.write_text(json.dumps(broken))
    with pytest.raises(ValueError,match='범위'):CorrespondenceProfile.load(path,p.spec)

def test_dry_run_samples_cannot_enable_live_mapping_and_unknown_poses_are_rejected():
    p=trained(False);f=extract_features(synthetic_hand(.3))
    with pytest.raises(ValueError,match='실물 확인'):p.predict(f,verified_only=True)
    f['index']=[9.]*4
    with pytest.raises(ValueError,match='학습하지 않은'):p.predict(f)

def test_single_finger_mapping_preserves_other_twelve_motor_targets():
    p=trained();f=extract_features(synthetic_hand(.4,thumb_across=.12,thumb_fold=.4))
    base=[.1]*15;out,q,_=p.predict(f,'index',base)
    assert all(out[i]==.1 for i in range(15) if i not in GROUPS['index'])
    assert len(q)==4

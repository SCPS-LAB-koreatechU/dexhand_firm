"""Two human poses map four independent finger curls to existing PWM endpoints.

Each finger's three tendons share one fraction in this gross-motion trial.
This is not independent distal-joint calibration or measured servo feedback.
"""
import copy,math
from .hand_retarget import GROUPS

FINGERS=('index','middle','ring','pinky')
MAX_AGE=.35


def middle_lower_test_goal(elapsed):
    """14-second diagnostic: open 2s, close 4s, hold 2s, open 4s, rest 2s."""
    if not math.isfinite(elapsed) or elapsed<0:raise ValueError('잘못된 시험 시간')
    value=max(0.,min(1.,(elapsed-2.)/4.,(12.-elapsed)/4.))
    goal=[0.]*15;goal[5]=value
    return goal


class TrackingContinuity:
    """Freeze motion for brief detection gaps; persistent loss latches HOLD.

    FREEZE must use the current command as its goal, never extrapolate or
    continue advancing the last camera goal. HOLD never resumes itself.
    """
    def __init__(self,grace=.3):
        if not 0<grace<=.3:raise ValueError('Detection grace must be at most 300 ms')
        self.grace=grace;self.missing_since=None;self.seen_valid=False;self.latched=False

    def update(self,sample,now):
        if self.latched:return 'HOLD','LATCHED'
        if sample.get('source_hand') not in (None,'Right'):
            self.latched=True;return 'HOLD','WRONG_HAND'
        if self.missing_since is not None and now-self.missing_since>=self.grace:
            self.latched=True;return 'HOLD','PERSISTENT_INPUT_LOSS'
        if fresh_sample(sample,now):
            self.seen_valid=True;self.missing_since=None;return 'TRACK',''
        if not self.seen_valid:
            self.latched=True;return 'HOLD','NO_INITIAL_HAND'
        if self.missing_since is None:self.missing_since=now
        return 'FREEZE','TRANSIENT_INPUT_LOSS'


def valid_features(features):
    return (isinstance(features,dict) and all(
        g in features and isinstance(features[g],(list,tuple)) and len(features[g])==4
        and all(type(v) in (int,float) and math.isfinite(v) for v in features[g])
        for g in FINGERS))


def fresh_sample(sample,now):
    stamp=sample.get('captured_at')
    return (sample.get('input_role','human_control')=='human_control'
        and sample.get('detection_valid') is True and sample.get('source_hand')=='Right'
        and sample.get('handedness_score',0)>=.65
        and type(stamp) in (int,float) and 0<=now-stamp<=MAX_AGE
        and valid_features(sample.get('recognized_features')))


def pose_matches(features,closed=False,groups=FINGERS):
    if not groups or any(g not in FINGERS for g in groups):raise ValueError('잘못된 보정 대상')
    if not valid_features(features):return False
    if closed:return all(sum(features[g][:2])>.85 for g in groups)
    return all(features[g][0]<.4 and features[g][1]<.55 for g in groups)


def validate_references(opened,closed):
    if not valid_features(opened) or not valid_features(closed):raise ValueError('잘못된 손 기준값')
    for g in FINGERS:
        delta=[b-a for a,b in zip(opened[g][:2],closed[g][:2])]
        if math.hypot(*delta)<.2 or sum(delta)<.2:
            raise ValueError(g+': 펼침/주먹 차이가 부족합니다')


def recalibrate_finger(opened,closed,new_open,new_closed,group):
    if group not in FINGERS:raise ValueError('잘못된 보정 대상')
    if not valid_features(new_open) or not valid_features(new_closed):raise ValueError('잘못된 새 기준값')
    updated_open,updated_closed=copy.deepcopy(opened),copy.deepcopy(closed)
    updated_open[group]=list(new_open[group]);updated_closed[group]=list(new_closed[group])
    validate_references(updated_open,updated_closed)
    return updated_open,updated_closed


def trial_goal(features,opened,closed,groups=FINGERS):
    if not groups or any(g not in FINGERS for g in groups):raise ValueError('잘못된 추종 대상')
    if not valid_features(features):raise ValueError('잘못된 손 인식값')
    validate_references(opened,closed)
    goal=[0.]*15
    for g in groups:
        axis=[b-a for a,b in zip(opened[g][:2],closed[g][:2])]
        value=sum((v-a)*d for v,a,d in zip(features[g][:2],opened[g][:2],axis))/sum(d*d for d in axis)
        value=max(0.,min(1.,value))
        if value<.03:value=0.
        if value>.97:value=1.
        for i in GROUPS[g]:goal[i]=value
    return goal

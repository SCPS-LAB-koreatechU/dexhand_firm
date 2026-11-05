"""Small, uncalibrated index-only trial; never creates verified calibration anchors."""
import math

INDEX=(6,7,8)


def trial_goal(features,reference,limit=.15):
    if not 0<limit<=.15:raise ValueError('Initial trial is limited to 15%')
    values=list(features['index']);baseline=list(reference['index'])
    if len(values)!=4 or len(baseline)!=4 or any(not math.isfinite(v) for v in values+baseline):raise ValueError('Invalid index features')
    # Both main tendons pull together; TIP follows the distal bend separately.
    main=max(0.,min(1.,((values[0]-baseline[0])+(values[1]-baseline[1]))/1.5))*limit
    tip=max(0.,min(1.,values[2]-baseline[2]))*limit
    goal=[0.]*15
    for i,v in zip(INDEX,(main,main,tip)):goal[i]=v
    return goal


def fresh_index_sample(data,now):
    if not isinstance(data,dict) or not data.get('detection_valid') or data.get('source_hand')!='Right':return False
    stamp=data.get('captured_at');features=data.get('recognized_features')
    # At ~7 Hz, capture-to-result latency plus one frame interval exceeds 250 ms.
    # Keep a finite 350 ms cutoff; loss still latches HOLD in the trial runner.
    return (type(stamp) in (int,float) and 0<=now-stamp<=.35 and isinstance(features,dict)
            and 'index' in features and len(features['index'])==4
            and all(isinstance(v,(int,float)) and math.isfinite(v) for v in features['index']))


def index_is_open(features):
    # Live held-open observations have a distal estimate bias of ~0.45.
    # Baseline subtraction removes that bias; proximal/middle joints must be open.
    proximal,middle,distal=features['index'][:3]
    return max(proximal,middle)<.35 and distal<.55

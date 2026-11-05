"""Measured-pose correspondence for 15 PWM motors. No CAN or camera access.

Interpolation is a convex combination of operator-recorded motor combinations.
The predicted joint angles are visualization estimates, never measured states.
"""
import copy,datetime,json,math,os
from pathlib import Path

GROUPS={'index':(6,7,8),'middle':(4,5,9),'ring':(2,3,10),'pinky':(0,1,11),'thumb':(12,13,14)}
LABELS={'index':'검지','middle':'중지','ring':'약지','pinky':'새끼','thumb':'엄지'}
DIMS={'index':4,'middle':4,'ring':4,'pinky':4,'thumb':5}
JOINTS={g:tuple('R_'+g.title()+'_'+j for j in ('Pitch','Flexor','DIP','Yaw')) for g in GROUPS if g!='thumb'}
JOINTS['thumb']=('R_Thumb_Yaw','R_Thumb_Roll','R_Thumb_Pitch','R_Thumb_Flexor','R_Thumb_DIP')

def sub(a,b):return [x-y for x,y in zip(a,b)]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def norm(a):return math.sqrt(dot(a,a))
def unit(a):
    n=norm(a)
    if not math.isfinite(n) or n<1e-8:raise ValueError('손 기준 좌표가 불안정합니다')
    return [v/n for v in a]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def bend(a,b,c):
    u,v=unit(sub(b,a)),unit(sub(c,b))
    return math.acos(max(-1.,min(1.,dot(u,v))))
def clamp(x,a,b):return max(a,min(b,x))

def extract_features(points):
    if len(points)!=21 or any(len(p)!=3 or any(not math.isfinite(float(x)) for x in p) for p in points):
        raise ValueError('유효한 3D 손 랜드마크 21개가 필요합니다')
    x=unit(sub(points[5],points[17]));dy=sub(points[9],points[0]);y=unit(sub(dy,[v*dot(dy,x) for v in x]));z=unit(cross(x,y))
    scale=norm(sub(points[5],points[17]))
    local=[[dot(sub(p,points[0]),axis)/scale for axis in (x,y,z)] for p in points]
    result={}
    for group,m in (('index',5),('middle',9),('ring',13),('pinky',17)):
        proximal=sub(points[m+1],points[m]);base=sub(points[m],points[0])
        flex=math.atan2(abs(dot(proximal,z)),math.hypot(dot(proximal,x),dot(proximal,y)))
        splay=math.atan2(dot(proximal,x),dot(proximal,y))-math.atan2(dot(base,x),dot(base,y))
        splay=math.atan2(math.sin(splay),math.cos(splay))
        result[group]=[flex/(math.pi/2),bend(points[m],points[m+1],points[m+2])/(math.pi/2),bend(points[m+1],points[m+2],points[m+3])/(math.pi/2),splay/(math.pi/2)]
    result['thumb']=list(local[4])+[bend(points[1],points[2],points[3])/math.pi,bend(points[2],points[3],points[4])/math.pi]
    return result

def estimate_joints(features,reference=None):
    q={}
    for g in ('index','middle','ring','pinky'):
        f=features[g];names=JOINTS[g]
        for j,v in zip(names,(clamp(f[0]*math.pi/2,0,1.308997),clamp(f[1]*math.pi/2,0,1.047198),clamp(f[2]*math.pi/2,0,1.047198),clamp(f[3]*math.pi/2,-.349066,.349066))):q[j]=v
    f=features['thumb'];r=reference['thumb'] if reference else f
    yaw=math.atan2(f[0],f[1])-math.atan2(r[0],r[1]);yaw=math.atan2(math.sin(yaw),math.cos(yaw))
    roll=math.atan2(f[2],math.hypot(f[0],f[1]))-math.atan2(r[2],math.hypot(r[0],r[1]))
    pitch=clamp((f[3]-r[3])*math.pi,0,1.047198);tip=clamp((f[4]-r[4])*math.pi,0,1.047198)
    for name,value in zip(JOINTS['thumb'],(clamp(yaw,-.523599,.523599),clamp(roll,-.349066,.349066),pitch,tip,tip)):q[name]=value
    return q

def synchronized_step(command,goal,dt,max_speed,full_stroke_s=None):
    delta=[g-c for c,g in zip(command,goal)]
    duration=max([abs(v)/max_speed for v in delta]+([abs(v)*t for v,t in zip(delta,full_stroke_s)] if full_stroke_s else [0.]))
    phase=1. if duration<=dt else dt/duration
    return [c+phase*d for c,d in zip(command,delta)],phase

class CorrespondenceProfile:
    def __init__(self,spec,source_hand='Right'):
        if spec.name!='right':raise ValueError('현재 연속 추종 모델은 오른손용입니다')
        self.spec=spec;self.source_hand=source_hand;self.anchors=[];self.support_radius=.35
        self.full_stroke_s=[.5]*15;self.timing_measured=[False]*15;self.reference=None
    def validate_motor_goal(self,fractions):
        if len(fractions)!=15 or any(not math.isfinite(float(v)) or not 0<=v<=1 for v in fractions):raise ValueError('모터 목표는 0~100% 범위여야 합니다')
        if fractions[12]+fractions[14]>1+1e-9:raise ValueError('엄지 좌우 당김 합이 현재 펌웨어 제한(100%)을 넘습니다')
    def add(self,group,name,features,pwm_us,verified=False,joints=None):
        if group not in GROUPS or not name.strip():raise ValueError('손가락과 자세 이름을 지정하세요')
        f=list(features[group])
        if len(f)!=DIMS[group] or any(not math.isfinite(float(v)) for v in f):raise ValueError('손 특징값이 잘못됐습니다')
        fractions=self.spec.to_fractions(pwm_us);self.validate_motor_goal(fractions)
        if self.reference is None:
            if name!='펴기':raise ValueError('먼저 편 손의 펴기 자세를 등록하세요')
            reference=copy.deepcopy(features)
        else:reference=self.reference
        js=estimate_joints(features,reference) if joints is None else joints
        group_joints={k:float(js[k]) for k in JOINTS[group]}
        if any(not math.isfinite(v) or abs(v)>math.pi for v in group_joints.values()):raise ValueError('시뮬 관절값이 잘못됐습니다')
        a=dict(group=group,name=name.strip(),features=f,pull=[fractions[i] for i in GROUPS[group]],joints=group_joints,operator_confirmed=bool(verified),recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
        keep=[x for x in self.anchors if not (x['group']==group and x['name']==a['name'])]
        for other in keep:
            if other['group']==group and self.distance(f,other['features'])<1e-4 and any(abs(x-y)>.03 for x,y in zip(a['pull'],other['pull'])):
                raise ValueError('같은 사람 손 자세에 서로 다른 모터값이 기록됩니다. 이전 샘플을 확인하세요')
        self.reference=reference;self.anchors=keep+[a]
    @staticmethod
    def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b))/len(a))
    def counts(self,verified_only=False):return {g:sum(a['group']==g and (not verified_only or a['operator_confirmed']) for a in self.anchors) for g in GROUPS}
    def predict(self,features,scope='all',base=None,verified_only=False):
        groups=tuple(GROUPS) if scope=='all' else (scope,)
        target=[0.]*15 if base is None else list(base);q={};support={}
        for g in groups:
            if g not in features or len(features[g])!=DIMS[g] or any(not math.isfinite(float(v)) for v in features[g]):raise ValueError('잘못된 손 특징값')
            eligible=[a for a in self.anchors if a['group']==g and (not verified_only or a['operator_confirmed'])]
            if len(eligible)<3:raise ValueError(f'{LABELS[g]}: 서로 다른 자세가 최소 3개 필요합니다'+(' (실물 확인 샘플만 사용)' if verified_only else ''))
            nearest=sorted([(self.distance(features[g],a['features']),a) for a in eligible],key=lambda v:v[0])[:4]
            support[g]=nearest[0][0]
            if nearest[0][0]>self.support_radius:raise ValueError(f'{LABELS[g]}: 아직 학습하지 않은 자세입니다. 가까운 샘플을 추가하세요')
            if nearest[0][0]<1e-6:weighted=[(1.,nearest[0][1])]
            else:
                w=[1/max(d,.01)**2 for d,a in nearest];total=sum(w);weighted=[(v/total,a) for v,(d,a) in zip(w,nearest)]
            for j,i in enumerate(GROUPS[g]):target[i]=sum(w*a['pull'][j] for w,a in weighted)
            for name in JOINTS[g]:q[name]=sum(w*a['joints'][name] for w,a in weighted)
        self.validate_motor_goal(target)
        return target,q,support
    def save(self,path):
        data=dict(format_version=1,hand=self.spec.name,source_hand=self.source_hand,profile_tag=self.spec.profile_tag,open_us=self.spec.open_us,close_us=self.spec.close_us,support_radius=self.support_radius,full_stroke_s=self.full_stroke_s,timing_measured=self.timing_measured,reference=self.reference,anchors=self.anchors,joint_values_are='operator-matched visualization estimates, not measured robot positions')
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_name(path.name+'.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n');os.replace(tmp,path)
    @classmethod
    def load(cls,path,spec,source_hand='Right'):
        d=json.loads(Path(path).read_text());obj=cls(spec,source_hand)
        if d.get('format_version')!=1 or d.get('hand')!=spec.name or d.get('source_hand')!=source_hand or d.get('profile_tag')!=spec.profile_tag or d.get('open_us')!=spec.open_us or d.get('close_us')!=spec.close_us:raise ValueError('손/카메라 손/모터 보정 범위가 다른 파일입니다')
        radius=float(d['support_radius'])
        if not .02<=radius<=1:raise ValueError('샘플 허용 거리가 잘못됐습니다')
        obj.support_radius=radius
        times=d.get('full_stroke_s',[.5]*15)
        if len(times)!=15 or any(not math.isfinite(float(t)) or not .05<=t<=10 for t in times):raise ValueError('모터 이동 시간은 0.05~10초입니다')
        obj.full_stroke_s=list(times);obj.timing_measured=d.get('timing_measured',[False]*15)
        if len(obj.timing_measured)!=15 or any(type(v) is not bool for v in obj.timing_measured):raise ValueError('이동시간 확인 표시가 잘못됐습니다')
        ref=d.get('reference');obj.reference=ref
        if ref is not None and (set(ref)!=set(GROUPS) or any(len(ref[g])!=DIMS[g] or any(not math.isfinite(float(x)) for x in ref[g]) for g in GROUPS)):raise ValueError('기준 손 특징값이 잘못됐습니다')
        for a in d['anchors']:
            g=a['group'];values=[0.]*15
            for i,v in zip(GROUPS[g],a['pull']):values[i]=v
            if len(a['pull'])!=3:raise ValueError('모터 샘플 수가 잘못됐습니다')
            features=copy.deepcopy(ref)
            if features is None:raise ValueError('기준 손 자세가 없습니다')
            features[g]=a['features']
            if type(a['operator_confirmed']) is not bool:raise ValueError('실물 확인 표시가 잘못됐습니다')
            obj.add(g,a['name'],features,spec.to_us(values),a['operator_confirmed'],a['joints'])
        return obj

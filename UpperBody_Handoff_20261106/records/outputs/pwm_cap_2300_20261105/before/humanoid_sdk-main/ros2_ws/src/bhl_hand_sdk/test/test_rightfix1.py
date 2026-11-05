import copy,hashlib,json
from pathlib import Path
import pytest
from bhl_hand_sdk.config import load_config,channel_names
from bhl_hand_sdk.controller import HandController
from bhl_upper_body_sdk.contract import CommandBuilder,validate
from bhl_upper_body_sdk.adapters import HandAdapter
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'dexhand_control/profiles').is_dir())
CONFIG=ROOT/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand.yaml'


def test_current_profile_hash_ring_and_corrected_thumb_endpoints():
 p=json.loads((ROOT/'dexhand_control/profiles/right_hand.json').read_text())
 assert hashlib.sha256(json.dumps(p,ensure_ascii=False,sort_keys=True,separators=(',',':')).encode()).hexdigest()[:8]=='79610de6'
 cfg=load_config(CONFIG)
 assert cfg.hand('right').profile_tag=='79610de6'
 assert cfg.hand('left').profile_tag=='a5b23746'
 assert [(m['id'],m['gpio'],m['open_us'],m['close_us']) for m in p['motors'][2:4]]==[(3,23,850,1725),(4,26,1700,800)]
 assert [m['name'] for m in p['motors'][12:]]==['엄지 왼쪽 당김','엄지 끝','엄지 오른쪽 당김']
 assert [m['gpio'] for m in p['motors'][12:]]==[13,14,18]
 assert [(m['open_us'],m['close_us']) for m in p['motors'][12:]]==[(2100,700),(700,1900),(2300,700)]
 assert cfg.hand('right').open_us==[m['open_us'] for m in p['motors']]
 assert cfg.hand('right').close_us==[m['close_us'] for m in p['motors']]


def test_per_controller_config_and_grips_are_isolated_and_not_double_rotated():
 cfg=load_config(CONFIG);original=copy.deepcopy(cfg)
 right=HandController(cfg,'right',dry_run=True);left=HandController(cfg,'left',dry_run=True);second=HandController(cfg,'right',dry_run=True)
 assert cfg==original
 assert left.config.antagonist_pairs==[(13,14)]
 assert right.config.antagonist_pairs==[(13,15)]
 for name,g in cfg.grips.items():
  assert right.config.grips[name]==g[:12]+[g[13],g[14],g[12]]
  assert second.config.grips[name]==right.config.grips[name]
  assert left.config.grips[name]==g
 right.config.grips['power'][0]=.2
 assert cfg==original and left.config.grips['power'][0]==original.grips['power'][0]


@pytest.mark.parametrize('side,pair,allowed',[('right',(12,14),False),('right',(12,13),True),('right',(13,14),True),('left',(12,13),False),('left',(12,14),True)])
def test_raw_goals_adapters_and_contract_use_actual_hand_order(side,pair,allowed):
 hand=HandController(load_config(CONFIG),side,dry_run=True);hand.enable();values=[0.]*15
 values[pair[0]]=values[pair[1]]=.7
 command=CommandBuilder().build(stamp_s=100,**{'hand_'+side:[0.]*15})
 command['targets']['hand_'+side]=values
 if allowed:
  hand.set_goal(values);assert hand.goal==values
  HandAdapter(hand).validate(values);validate(command,100,{'hand_'+side})
 else:
  with pytest.raises(ValueError):hand.set_goal(values)
  with pytest.raises(ValueError):HandAdapter(hand).validate(values)
  with pytest.raises(ValueError):validate(command,100,{'hand_'+side})


def test_named_grip_only_transforms_once_but_raw_targets_never_transform():
 cfg=load_config(CONFIG);cfg.grips['power']=[0.]*12+[1.,0.,1.]
 right=HandController(cfg,'right',dry_run=True)
 right.set_grip('power');assert right.goal[-3:]==[0.,1.,1.]
 right.set_grip('power',.25);assert right.goal[-3:]==[0.,.25,.25]
 raw=[0.]*12+[.2,.7,.3];right.set_goal(raw);assert right.goal==raw
 assert channel_names('right')[-3:]==('thumb_pull_left','thumb_tip','thumb_pull_right')
 assert channel_names('left')[-3:]==('thumb_pull_right','thumb_pull_left','thumb_tip')
 assert right.spec.channel_names==channel_names('right')

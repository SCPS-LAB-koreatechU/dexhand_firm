import json,time,xml.etree.ElementTree as ET
from pathlib import Path
import pytest
from bhl_hand_sdk.preview_state import read_estimate
from bhl_hand_sdk.hand_retarget import JOINTS

def test_preview_is_labeled_estimate_and_expires(tmp_path):
    p=tmp_path/'q.json';now=time.time()
    data=dict(utc=now,joints={'R_Thumb_Yaw':.1},source='operator-correspondence estimate, NOT measured',motor_streaming=False)
    p.write_text(json.dumps(data));assert read_estimate(p,now)['joints']==data['joints']
    assert read_estimate(p,now+1) is None
    data['source']='measured';p.write_text(json.dumps(data))
    with pytest.raises(ValueError):read_estimate(p,now)

def test_preview_assets_include_moving_thumb_and_valid_meshes():
    pkg=Path(__file__).resolve().parents[1]
    robot=ET.parse(pkg/'preview_model/estimated_right_hand.urdf').getroot()
    joints={j.get('name'):j for j in robot.findall('joint')}
    assert all(joints[j].get('type')=='revolute' for names in JOINTS.values() for j in names)
    assert not list(robot.iter('mimic'))
    assert all(l.get('name').startswith('camera_hand_') for l in robot.findall('link'))
    for mesh in robot.iter('mesh'):
        prefix='package://bhl_hand_sdk/'
        assert mesh.get('filename').startswith(prefix)
        assert (pkg/mesh.get('filename')[len(prefix):]).is_file()
    assert (pkg/'preview_model/LICENSE').is_file()

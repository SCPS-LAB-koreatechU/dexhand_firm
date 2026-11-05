import base64,json
from bhl_hand_sdk.observation_capture import ObservationRecorder


def test_capture_uses_command_at_exposure_not_later_read_time(tmp_path):
    recorder=ObservationRecorder(tmp_path/'frames')
    recorder.command(1.,[850]*15);recorder.command(1.2,[1000]*15);recorder.command(1.4,[1750]*15)
    source=tmp_path/'latest.json'
    payload=dict(frame_id=7,captured_at=1.25,input_role='robot_observation',image_jpeg_base64=base64.b64encode(b'\xff\xd8test\xff\xd9').decode())
    source.write_text(json.dumps(payload))
    frame=recorder.capture(source,now=1.5)
    assert frame['command_at_capture_us']==[1000]*15
    assert frame['command_stamp']==1.2
    assert frame['camera']['frame_id']==7
    assert (tmp_path/'frames/frame_00000007.jpg').read_bytes()==b'\xff\xd8test\xff\xd9'
    assert recorder.capture(source,now=1.6) is None


def test_stale_and_precommand_frames_are_never_assigned_a_future_command(tmp_path):
    recorder=ObservationRecorder(tmp_path/'frames');recorder.command(2.,[1750]*15)
    source=tmp_path/'latest.json';source.write_text(json.dumps(dict(frame_id=1,captured_at=1.5,image_jpeg_base64=base64.b64encode(b'\xff\xd8test').decode())))
    assert recorder.capture(source,now=2.4) is None
    assert recorder.capture(source,now=1.6)['command_at_capture_us'] is None

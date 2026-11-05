import json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from bhl_hand_sdk.camera_source import selected_view,crop_view
from bhl_hand_sdk.data_paths import data_path
from bhl_hand_sdk.ui_trace import UiTrace
from bhl_hand_sdk.hand_tracking_gui import TrackingGui


def test_zed_uses_one_eye_and_other_cameras_keep_full_view(tmp_path):
    node=tmp_path/'video0';node.mkdir();(node/'name').write_text('ZED 2i: ZED 2i')
    assert selected_view('0',sysfs_root=tmp_path)=='left'
    assert selected_view('/dev/video0',sysfs_root=tmp_path)=='left'
    assert selected_view('0','right',sysfs_root=tmp_path)=='right'
    (node/'name').write_text('USB Camera')
    assert selected_view('0',sysfs_root=tmp_path)=='full'
    assert selected_view('99',sysfs_root=tmp_path)=='full'
    frame=np.zeros((4,8,3),np.uint8);frame[:,:4]=17;frame[:,4:]=93
    assert crop_view(frame,'left').shape==(4,4,3)
    assert np.all(crop_view(frame,'left')==17)
    assert np.all(crop_view(frame,'right')==93)
    assert crop_view(frame,'full') is frame
    with pytest.raises(ValueError):crop_view(frame[:,:7],'left')


def test_session_storage_and_trace_are_outside_home(tmp_path,monkeypatch):
    monkeypatch.setenv('BHL_HAND_DATA_DIR',str(tmp_path))
    assert data_path('calibration')==tmp_path/'calibration'
    assert data_path('preview')==tmp_path/'preview.json'
    trace=UiTrace();trace.submit({'test':1});trace.close()
    assert trace.path.parent==tmp_path/'logs'
    assert json.loads(trace.path.read_text())['test']==1
    assert not trace.error


def test_readonly_log_does_not_crash_or_block_gui(tmp_path,monkeypatch):
    monkeypatch.setenv('BHL_HAND_DATA_DIR',str(tmp_path))
    original=Path.open
    def readonly(path,*a,**kw):
        if path.suffix=='.jsonl':raise OSError(30,'Read-only file system')
        return original(path,*a,**kw)
    monkeypatch.setattr(Path,'open',readonly)
    trace=UiTrace();trace.thread.join(2)
    assert trace.error and not trace.thread.is_alive()
    for _ in range(3000):trace.submit({'test':1})
    start=time.monotonic();trace.close()
    assert time.monotonic()-start<.3 and trace.q.qsize()==0


def test_preview_write_failure_preserves_event_loop(tmp_path):
    block=tmp_path/'block';block.write_text('not a directory');notes=[]
    obj=SimpleNamespace(preview_file=block/'preview.json',last_preview_write=0.,storage_error='',note=SimpleNamespace(set=notes.append))
    TrackingGui.write_preview(obj,{'R_Thumb_Yaw':.1},False)
    assert obj.storage_error and notes and obj.last_preview_write>0


def test_cpu_options_selects_threads_without_changing_model():
    from bhl_hand_sdk.camera_source import cpu_base_options
    class Base:
        def __init__(self,model_asset_path):self.path=model_asset_path
        def to_pb2(self):return SimpleNamespace(acceleration=SimpleNamespace(xnnpack=SimpleNamespace(num_threads=1)))
    options=cpu_base_options(Base,'model.task',4)
    assert options.path=='model.task' and options.to_pb2().acceleration.xnnpack.num_threads==4
    class Native:
        def __init__(self,model_asset_path):self.path=model_asset_path
    assert type(cpu_base_options(Native,'model.task')) is Native
    with pytest.raises(ValueError):cpu_base_options(Base,'model.task',0)

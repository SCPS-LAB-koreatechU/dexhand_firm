import importlib.util
import json
from pathlib import Path
import time

import pytest

from bhl_hand_sdk.gui_model import CalibrationDraft, GuiControl, profile_tag
from bhl_hand_sdk.demo_session import DemoSession
from bhl_hand_sdk.gui_worker import GuiWorker
from bhl_hand_sdk.config import load_config
from test_hand import config, rig  # noqa: F401

PROJECT = next(p for p in Path(__file__).resolve().parents if (p/'dexhand_control/profiles').is_dir())
CONFIG = PROJECT/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand.yaml'


def profile(side='right'):
    return json.loads((PROJECT/f'dexhand_control/profiles/{side}_hand.json').read_text(encoding='utf-8'))


def test_single_motor_updates_preserve_all_other_goals(rig):
    hand, fake = rig(protocol=3)
    session = DemoSession(hand)
    control = GuiControl(session, clock=time.monotonic)
    control.arm(initial_open=True)
    hand.set_goal([.1]*15)
    control.motor(0, 1775)
    assert hand.goal[0] == 1
    assert hand.goal[1:] == [.1]*14
    control.tick(.02)
    assert fake.pwm_on
    assert control.status()['applied_us'] == fake.pulses


def test_mode_change_holds_and_requires_explicit_rearm(rig):
    hand, fake = rig(protocol=3)
    control = GuiControl(DemoSession(hand), clock=time.monotonic)
    control.arm(initial_open=True)
    before = fake.pulses[:]
    control.switch_mode('calibration')
    assert fake.inhibited and not control.session.active and fake.pulses == before
    with pytest.raises(ValueError, match='출력을 먼저'):
        control.motor(0, 1000)
    control.tick(.02)
    assert fake.pulses == before
    control.arm()
    assert not fake.inhibited and fake.pulses == before


@pytest.mark.parametrize('index,pulse', [(0, 849), (0, 1776), (0, 1000.5), (0, True), (-1, 1000), (15, 1000)])
def test_live_calibration_refuses_out_of_bounds_or_malformed_command(rig, index, pulse):
    hand, fake = rig(protocol=3)
    control = GuiControl(DemoSession(hand), clock=time.monotonic)
    control.arm(initial_open=True)
    before = list(hand.goal)
    with pytest.raises(ValueError):
        control.motor(index, pulse)
    assert hand.goal == before


def test_thumb_antagonist_check_is_not_bypassed_by_single_motor_mode(rig):
    hand, fake = rig(protocol=3)
    control = GuiControl(DemoSession(hand), clock=time.monotonic)
    control.arm(initial_open=True)
    control.motor(12, 700)
    with pytest.raises(ValueError, match='pull against'):
        control.motor(14, hand.spec.close_us[14])
    assert hand.goal[12] == 1 and hand.goal[14] == 0


def test_gui_camera_requires_open_fresh_hand_and_does_not_auto_resume(rig):
    hand, fake = rig(protocol=3)
    now = [10.0]
    control = GuiControl(DemoSession(hand), clock=lambda: now[0])
    control.switch_mode('camera')
    with pytest.raises(ValueError, match='편 손'):
        control.arm(initial_open=True)
    def frame(n, closed=False, valid=True):
        return dict(kind='frame', captured_at=now[0], frame_id=n, valid=valid, closed=closed, arm_at=0)
    now[0] = 10.1
    control.tick(.02, frame(1))
    control.arm(initial_open=True)
    assert control.session.active and fake.pwm_on
    now[0] = 10.2
    control.tick(.02, frame(2, True))
    assert hand.goal == [.25*v for v in hand.config.grips['power']]
    now[0] = 10.7
    control.tick(.02)
    assert not control.session.active and fake.inhibited
    now[0] = 10.8
    control.tick(.02, frame(3))
    assert not control.session.active
    control.arm()
    assert control.session.active


@pytest.mark.parametrize('side', ['left', 'right'])
def test_calibration_draft_has_15_named_motors_and_cannot_mutate_live_spec(tmp_path, side):
    cfg = load_config(CONFIG)
    original = profile(side)
    draft = CalibrationDraft(original, cfg.hand(side))
    draft.record(0, 'open_us', 1500)
    draft.save(tmp_path/'draft.json')
    assert original == profile(side)
    assert cfg.hand(side).open_us[0] == original['motors'][0]['open_us']
    saved = json.loads((tmp_path/'draft.json').read_text(encoding='utf-8'))
    assert saved['motors'][0]['open_us'] == 1500
    assert len(saved['motors']) == 15 and all(m['name'] for m in saved['motors'])
    assert saved['motors'][14]['gpio'] == 18


def test_draft_rejects_equal_endpoints_and_profile_mismatch(tmp_path):
    cfg = load_config(CONFIG)
    draft = CalibrationDraft(profile(), cfg.hand('right'))
    draft.record(0, 'open_us', draft.profile['motors'][0]['close_us'])
    with pytest.raises(ValueError, match='nonzero'):
        draft.save(tmp_path/'bad.json')
    assert not (tmp_path/'bad.json').exists()
    wrong = profile()
    wrong['motors'][0]['gpio'] = 32
    with pytest.raises(ValueError, match='differ'):
        CalibrationDraft(wrong, cfg.hand('right'))


def test_load_saved_draft_changes_only_endpoints_and_rejects_wiring_atomically(tmp_path):
    cfg = load_config(CONFIG)
    draft = CalibrationDraft(profile(), cfg.hand('right'))
    candidate = profile()
    candidate['motors'][0]['open_us'] = 925
    path = tmp_path/'load.json'
    path.write_text(json.dumps(candidate), encoding='utf-8')
    draft.load(path)
    assert draft.profile['motors'][0]['open_us'] == 925
    candidate['motors'][0]['gpio'] = 32
    path.write_text(json.dumps(candidate), encoding='utf-8')
    with pytest.raises(ValueError, match='배선'):
        draft.load(path)
    assert draft.profile['motors'][0]['open_us'] == 925 and draft.profile['motors'][0]['gpio'] == 19


def exporter():
    spec = importlib.util.spec_from_file_location('gui_calibration_export', PROJECT/'dexhand_control/tools/export_calibration.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_old_export_template_refuses_rightfix1_without_partial_output(tmp_path):
    module = exporter()
    before = {path: path.read_bytes() for path in [CONFIG, PROJECT/'dexhand_control/profiles/right_hand.json',
        PROJECT/'dexhand_control/firmware/DexHandRight/BoardConfig.h']}
    profiles = {side: profile(side) for side in ('left', 'right')}
    profiles['right']['motors'][0]['open_us'] = 900
    with pytest.raises(ValueError, match='rightfix1'):
        module.export_bundle(tmp_path/'new', profiles)
    assert not (tmp_path/'new').exists()
    assert all(path.read_bytes() == data for path,data in before.items())


def test_invalid_export_creates_no_partial_deployment(tmp_path):
    profiles = {side: profile(side) for side in ('left', 'right')}
    profiles['right']['motors'][6]['gpio'] = 34
    with pytest.raises(ValueError):
        exporter().export_bundle(tmp_path/'bad', profiles)
    assert not (tmp_path/'bad').exists()


def wait_for(worker, predicate, timeout=2, heartbeat=True):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if heartbeat:
            worker.ui_seen = time.monotonic()
        try:
            event = worker.events.get(timeout=.025)
        except __import__('queue').Empty:
            continue
        if predicate(event):
            return event
    raise AssertionError('worker event timed out')


def test_worker_keeps_can_off_until_connect_and_holds_when_gui_stalls(rig):
    hand, fake = rig(protocol=3)
    opened = []
    def factory(*args, **kwargs):
        opened.append(True)
        return hand
    worker = GuiWorker(CONFIG, factory=factory)
    try:
        time.sleep(.03)
        assert not opened and not fake.modes
        worker.post('connect', hand='right', channel='unused', dry_run=False)
        wait_for(worker, lambda e: e['kind'] == 'connected')
        assert not fake.modes and fake.pulses is None
        worker.post('arm', initial_open=True)
        wait_for(worker, lambda e: e['kind'] == 'status' and e['active'])
        worker.move(0, 1000)
        wait_for(worker, lambda e: e['kind'] == 'status' and e['goal_us'][0] == 1000)
        worker.ui_seen = time.monotonic()-1
        event = wait_for(worker, lambda e: e['kind'] == 'status' and not e['active'], heartbeat=False)
        assert 'GUI 응답 만료' in event['reason']
        assert fake.inhibited and fake.pwm_on
    finally:
        worker.close()
        worker.thread.join(timeout=2)
        assert not worker.thread.is_alive()


def test_disconnect_reports_unacknowledged_hold_instead_of_claiming_success(rig):
    hand, fake = rig(protocol=3)
    worker = GuiWorker(CONFIG, factory=lambda *args, **kwargs: hand)
    try:
        worker.post('connect', hand='right', channel='unused', dry_run=False)
        wait_for(worker, lambda e: e['kind'] == 'connected')
        worker.post('arm', initial_open=True)
        wait_for(worker, lambda e: e['kind'] == 'status' and e['active'])
        fake.silent = True
        worker.post('disconnect')
        event = wait_for(worker, lambda e: e['kind'] == 'error')
        assert '종료 HOLD 응답 없음' in event['message']
        assert worker.control is None
    finally:
        fake.silent = False
        worker.close()
        worker.thread.join(timeout=2)

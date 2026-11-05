import time,queue
from pathlib import Path
import pytest

from test_hand import config, rig
from bhl_hand_sdk.io_diagnostics import io_diagnostics
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.controller import HandController
from bhl_hand_sdk.gui_worker import GuiWorker

CONFIG=Path(__file__).resolve().parents[1]/'config/dexhand.yaml'


def test_pose_timing_distinguishes_queue_from_ack_and_copies_evidence(rig):
    hand, fake = rig(protocol=3)
    assert hand.enable().ok
    fake.delay = .01
    assert hand.step(.02).ok
    timing = hand.last_io_timing
    assert timing['frames_queued'] == 5 and timing['result'] == 'APPLIED'
    assert timing['queue_started_monotonic_s'] <= timing['queue_finished_monotonic_s']
    assert timing['queue_finished_monotonic_s'] <= timing['ack_decoded_monotonic_s'] <= timing['wait_finished_monotonic_s']
    assert timing['ack_decoded_monotonic_s']-timing['queue_finished_monotonic_s'] >= .008
    evidence=io_diagnostics(hand,'stream')
    assert evidence['firmware_status']['result']=='APPLIED'
    assert evidence['firmware_status']['tx_errors']==0
    assert evidence['last_confirmed_command']['pwm_us']==hand.command_us
    hand.last_io_timing['frames_queued']=0
    assert evidence['last_pose_io']['frames_queued']==5


def test_timeout_timing_does_not_claim_an_ack(rig):
    hand, fake = rig(protocol=3)
    assert hand.enable().ok
    fake.drop_status=1
    result=hand.step(.02)
    assert result.result=='NO_REPLY'
    timing=hand.last_io_timing
    assert timing['frames_queued']==5 and timing['result']=='NO_REPLY'
    assert timing['ack_decoded_monotonic_s'] is None
    assert timing['wait_finished_monotonic_s']>timing['queue_finished_monotonic_s']


def test_gui_uses_sdk_speed_without_a_hidden_cap():
    worker=GuiWorker(CONFIG,factory=lambda cfg,side,dry_run:HandController(cfg,side,dry_run=True))
    try:
        worker.post('connect',hand='right',channel='can1',dry_run=True)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            worker.ui_seen=time.monotonic()
            try:event=worker.events.get(timeout=.05)
            except queue.Empty:continue
            if event['kind']=='connected':break
        else:raise AssertionError('Connect timed out')
        assert worker.control.session.hand.config.max_speed==load_config(CONFIG).max_speed
        assert not worker.control.session.attempted_enable
    finally:
        worker.close();worker.thread.join(3)
    assert not worker.thread.is_alive()


def test_hidden_cap_explains_eightfold_little_finger_command_delay():
    durations=[]
    for speed in (.25,2.):
        cfg=load_config(CONFIG);cfg.max_speed=speed
        hand=HandController(cfg,'right',dry_run=True)
        initial=[0.]*15
        for i in (0,1,11):initial[i]=1.
        hand.set_goal(initial);assert hand.enable().ok
        hand.set_grip('open');steps=0
        while any(abs(c-g)>1e-9 for c,g in zip(hand.command,hand.goal)):
            assert hand.step(.02).ok
            steps+=1
            assert steps<=201
        durations.append(steps*.02)
    assert durations==pytest.approx([4.,.5])

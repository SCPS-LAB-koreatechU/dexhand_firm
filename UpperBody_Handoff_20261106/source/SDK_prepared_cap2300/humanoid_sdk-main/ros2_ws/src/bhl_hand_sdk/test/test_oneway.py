from pathlib import Path
import queue,time
import pytest
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.controller import HandController
from bhl_hand_sdk.demo_session import DemoSession
from bhl_hand_sdk.gui_worker import GuiWorker

CONFIG=Path(__file__).resolve().parents[1]/'config/dexhand_oneway.yaml'

class NoReplyBus:
    def __init__(self):self.frames=[];self.closed=False
    def send(self,ident,data,**kw):self.frames.append((ident,bytes(data)));return True
    def recv(self,*args):raise AssertionError('Receive-only firmware must not cause an ACK wait')
    def close(self):self.closed=True

def make():
    bus=NoReplyBus();hand=HandController(load_config(CONFIG),'right',dry_run=False,bus=bus)
    return hand,bus,DemoSession(hand)

def test_socket_connect_and_cold_start_never_invent_feedback():
    hand,bus,session=make()
    assert not hand.confirmed and not hand.spec.feedback
    assert hand.ping() is None and hand.read_snapshot() is None and not bus.frames
    with pytest.raises(RuntimeError,match='단방향'):session.ping()
    with pytest.raises(RuntimeError,match='재접속'):session.arm()
    assert not bus.frames
    session.arm(initial_open=True)
    assert session.active and hand.last_delivery.result=='SENT'
    assert len(bus.frames)==6 and hand.stats['applied']==0
    assert hand.last_applied_command is None
    for _ in range(30):assert session.step(.02).result=='SENT'
    assert hand.stats['no_reply']==0 and session.active
    session.stop('test');assert session.last_hold.result=='SENT'
    hand.close()

def test_same_session_resume_uses_resume_control_but_reconnect_cannot_resume():
    hand,bus,session=make();session.arm(initial_open=True)
    values=[0.]*15;values[0]=.2;hand.set_goal(values);session.step(.02)
    held=hand.command_us[:];session.stop('hold');bus.frames.clear()
    session.arm(initial_open=False)
    assert bus.frames[0][1][0]==0x13 and hand.command_us==held
    session.close()
    other,newbus,newsession=make()
    with pytest.raises(RuntimeError,match='재접속'):newsession.arm()
    assert not newbus.frames;other.close()

def test_worker_connection_is_local_only_and_does_not_raise_missing_heartbeat():
    buses=[]
    def factory(cfg,side,dry_run):
        bus=NoReplyBus();buses.append(bus);return HandController(cfg,side,dry_run=False,bus=bus)
    worker=GuiWorker(CONFIG,factory=factory)
    def wait(kind,predicate=lambda e:True):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            worker.ui_seen=time.monotonic()
            try:e=worker.events.get(timeout=.05)
            except queue.Empty:continue
            if e['kind']=='error':raise AssertionError(e)
            if e['kind']==kind and predicate(e):return e
        raise AssertionError('No '+kind)
    try:
        worker.post('connect',hand='right',channel='can1',dry_run=False)
        e=wait('connected');assert not e['feedback'] and not buses[0].frames
        worker.post('arm',initial_open=True,request_id='open')
        wait('armed')
        e=wait('status',lambda e:e['active'])
        assert e['applied_us'] is None and not e['feedback']
        assert e['delivery']['result']=='SENT'
        worker.post('hold');wait('status',lambda e:not e['active'])
        worker.post('connect',hand='right',channel='can1',dry_run=False)
        wait('connected');assert len(buses)==2 and not buses[1].frames
    finally:
        worker.close();worker.thread.join(3)
    assert not worker.thread.is_alive()

def test_oneway_calibration_keeps_confirmed_pins_and_endpoints():
    spec=load_config(CONFIG).hand('right')
    assert spec.open_us[-3:]==[2100,700,2300]
    assert spec.close_us[-3:]==[700,1900,700]
    assert spec.profile_tag=='bdad07a4' and spec.protocol==3

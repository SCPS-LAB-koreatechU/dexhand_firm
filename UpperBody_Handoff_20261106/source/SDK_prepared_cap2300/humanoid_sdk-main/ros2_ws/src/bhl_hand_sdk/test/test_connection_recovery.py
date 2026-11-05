import queue,threading,time
from pathlib import Path
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.gui_worker import GuiWorker
from bhl_hand_sdk.controller import HandController
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'dexhand_control/profiles').is_dir())
CONFIG=ROOT/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand.yaml'

def test_cancel_generation_drops_queued_old_arm_and_moves():
 w=GuiWorker.__new__(GuiWorker);w.lock=threading.Lock();w.commands=queue.Queue();w.pending_motors={};w.generation=0;w.hold_requested=threading.Event()
 w.post('arm',initial_open=True);old=w.commands.queue[0];w.move(0,900)
 w.post('disconnect')
 assert w.generation==1 and not w.pending_motors
 assert w.commands.qsize()==1 and w.commands.queue[0]['action']=='disconnect'
 w._execute(old)  # stale work is rejected before touching any controller
 w.post('hold');assert w.hold_requested.is_set() and w.commands.empty()
 w.post('connect',hand='right',channel='can1',dry_run=True)
 assert w.commands.qsize()==1 and w.commands.queue[0]['generation']==3

def test_reconnect_survives_previous_close_hold_failure_without_auto_enable():
 w=GuiWorker(CONFIG,factory=lambda cfg,side,dry_run:HandController(cfg,side,dry_run=True))
 def wait_event(kind,timeout=3):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   w.ui_seen=time.monotonic()
   try:e=w.events.get_nowait()
   except queue.Empty:time.sleep(.005);continue
   if e['kind']==kind:return e
  raise AssertionError('No '+kind)
 try:
  w.post('connect',hand='right',channel='can1',dry_run=True);wait_event('connected')
  old=w.control;close=old.session.close
  def failed_close():close();raise RuntimeError('old HOLD reply unavailable')
  old.session.close=failed_close
  w.post('connect',hand='right',channel='can1',dry_run=True)
  wait_event('warning');wait_event('connected')
  assert w.control is not old and not w.control.session.active
  assert not w.control.session.attempted_enable
  assert w.control.session.hand.stats['sets_sent']==0
 finally:
  w.close();w.thread.join(3)
 assert not w.thread.is_alive()

def test_dry_hold_preserves_pwm_on_but_off_clears_it():
 from bhl_hand_sdk.hand_can import Flag
 hand=HandController(load_config(CONFIG),'right',dry_run=True)
 hand.set_grip('open');hand.enable();hand.hold()
 assert hand.ping().flags & Flag.PWM_ON
 hand.off();assert not hand.ping().flags & Flag.PWM_ON

def test_virtual_reconnect_uses_firmware_snapshot_not_old_unsent_goal():
 from test_hand import FakeHand
 from bhl_hand_sdk.hand_can import HandBus,Mode
 import uuid
 cfg=load_config(CONFIG);cfg.reply_timeout=.05;cfg.ping_timeout=.2
 spec=cfg.hand('right');channel='reconnect_'+uuid.uuid4().hex
 fake=FakeHand(channel,spec.node,spec.low_us,spec.high_us,protocol=3,tag=spec.profile_tag)
 first=HandController(cfg,'right',dry_run=False,bus=HandBus(channel,[61],interface='virtual'))
 second=None
 try:
  first.set_grip('open');assert first.enable().ok
  pose=[0.]*15;pose[0]=.2;first.set_goal(pose)
  for _ in range(8):assert first.step(.02).ok
  held=list(fake.pulses)
  stale=[0.]*15;stale[0]=.9;first.set_goal(stale)  # never sent; must not replay
  first.close()
  second=HandController(cfg,'right',dry_run=False,bus=HandBus(channel,[61],interface='virtual'))
  assert second.ping() is not None and not second.live
  assert fake.pulses==held
  assert second.enable().ok
  assert fake.pulses==held and second.command_us==held
  assert fake.modes[-1]==Mode.RESUME
 finally:
  first.close()
  if second:second.close()
  fake.close()

def test_virtual_link_loss_stops_stream_and_readonly_reconnect_never_replays():
 from test_hand import FakeHand
 from bhl_hand_sdk.hand_can import HandBus,Mode
 import uuid
 cfg=load_config(CONFIG);spec=cfg.hand('right');channel='loss_'+uuid.uuid4().hex
 fake=FakeHand(channel,spec.node,spec.low_us,spec.high_us,protocol=3,tag=spec.profile_tag)
 w=GuiWorker(CONFIG,factory=lambda c,side,dry_run:HandController(c,side,dry_run=False,bus=HandBus(channel,[61],interface='virtual')))
 def wait(kind,timeout=4):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   w.ui_seen=time.monotonic()
   try:e=w.events.get_nowait()
   except queue.Empty:time.sleep(.005);continue
   if e['kind']==kind:return e
  raise AssertionError('missing '+kind)
 try:
  w.post('connect',hand='right',channel='can1',dry_run=False);wait('connected')
  w.post('arm',initial_open=True,request_id='start');wait('armed')
  fake.silent=True;wait('error')
  assert not w.control.session.active
  fake.silent=False
  w.post('connect',hand='right',channel='can1',dry_run=False);wait('connected')
  assert not w.control.session.active and not w.control.session.attempted_enable
  assert w.control.session.hand.stats['sets_sent']==0
  assert fake.modes.count(Mode.START)==1 and Mode.RESUME not in fake.modes
 finally:
  w.close();w.thread.join(3);fake.close()
 assert not w.thread.is_alive()

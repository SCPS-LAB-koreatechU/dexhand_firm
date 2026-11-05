"""Offline Python SDK <-> actual native C++ firmware. Set BHL_WIRE_HARNESS_DIR."""
import collections
import os
from pathlib import Path
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk"
sys.path.insert(0, str(PACKAGE))
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.controller import HandController
from bhl_hand_sdk.hand_can import decode, pose_frames, Result
from bhl_hand_sdk.demo_session import DemoSession


class WireBus:
    def __init__(self, path):
        self.process = subprocess.Popen([str(path)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.frames = collections.deque()
        self.pulses = None
        self.history = []
        self.read()

    def read(self):
        while True:
            line = self.process.stdout.readline().strip()
            if line == "END":
                return
            assert line, "native harness exited unexpectedly"
            fields = line.split()
            if fields[0] == "CAN":
                self.frames.append(decode(int(fields[1]), bytes(map(int, fields[2:]))))
            elif fields[0] == "PWM":
                self.pulses = list(map(int, fields[1:])); self.history.append(self.pulses)
            elif fields[0] == "OFF":
                self.pulses = None

    def send(self, ident, data, timeout=None):
        self.process.stdin.write(" ".join(map(str, [ident, len(data), *data])) + "\n")
        self.process.stdin.flush(); self.read()
        return True

    def recv(self, timeout=0):
        return self.frames.popleft() if self.frames else None

    def close(self):
        pass  # shared across two SDK lifetimes in the handover test

    def terminate(self):
        self.process.stdin.close()
        self.process.wait(timeout=3)
        self.process.stdout.close()
        assert self.process.returncode == 0


@pytest.mark.parametrize("side", ["left", "right"])
def test_simple_demo_against_actual_cpp_firmware(side):
    directory = os.environ.get("BHL_WIRE_HARNESS_DIR")
    if not directory:
        pytest.skip("build native wire harnesses and set BHL_WIRE_HARNESS_DIR")
    path = Path(directory)/("wire_"+side+(".exe" if os.name == "nt" else ""))
    bus = WireBus(path)
    try:
        cfg = load_config(PACKAGE/"config/dexhand.yaml")
        cfg.max_speed, cfg.reply_timeout, cfg.ping_timeout = .25, .2, .2
        hand = HandController(cfg, side, dry_run=False, bus=bus)
        session = DemoSession(hand)
        assert session.ping().profile_tag == cfg.hand(side).profile_tag
        assert bus.pulses is None
        session.arm(initial_open=True)
        opened = bus.pulses[:]
        session.set_closed(True)
        for _ in range(25):
            assert session.step(.02).result == "APPLIED"
        assert bus.pulses != opened
        assert max(hand.command) <= .25
        held = bus.pulses[:]
        assert session.stop("tracking lost").ok
        assert bus.pulses == held
    finally:
        bus.terminate()


@pytest.mark.parametrize("side", ["left", "right"])
def test_gui_motor_and_mode_hold_against_actual_cpp_firmware(side):
    from bhl_hand_sdk.gui_model import GuiControl
    import time
    directory = os.environ.get('BHL_WIRE_HARNESS_DIR')
    if not directory:
        pytest.skip('build native wire harnesses and set BHL_WIRE_HARNESS_DIR')
    bus = WireBus(Path(directory)/('wire_'+side+('.exe' if os.name == 'nt' else '')))
    try:
        cfg = load_config(PACKAGE/'config/dexhand.yaml')
        cfg.max_speed, cfg.reply_timeout, cfg.ping_timeout = .25, .2, .2
        hand = HandController(cfg, side, dry_run=False, bus=bus)
        control = GuiControl(DemoSession(hand), clock=time.monotonic)
        control.arm(initial_open=True)
        opened = bus.pulses[:]
        control.motor(0, hand.spec.close_us[0])
        for _ in range(15):
            control.tick(.02)
        assert bus.pulses[0] != opened[0]
        assert bus.pulses[1:] == opened[1:]
        held = bus.pulses[:]
        control.switch_mode('calibration')
        assert not control.session.active and bus.pulses == held
        control.arm()
        assert bus.pulses == held
    finally:
        bus.terminate()


@pytest.mark.parametrize("side", ["left", "right"])
def test_sdk_and_real_firmware_agree_on_boot_grip_handover_and_rejection(side):
    directory = os.environ.get("BHL_WIRE_HARNESS_DIR")
    if not directory:
        pytest.skip("build native wire harnesses and set BHL_WIRE_HARNESS_DIR")
    path = Path(directory) / ("wire_" + side + (".exe" if os.name == "nt" else ""))
    bus = WireBus(path)
    try:
        cfg = load_config(PACKAGE / "config/dexhand.yaml")
        cfg.ping_timeout = cfg.reply_timeout = .2
        first = HandController(cfg, side, dry_run=False, bus=bus)
        assert first.enable().result == "INITIAL_TARGET_REQUIRED" and bus.pulses is None
        first.set_grip("open")
        assert first.enable().ok
        first.set_grip("power", .4)
        for _ in range(12):
            assert first.step(.02).ok
        held = list(bus.pulses)
        first.hold(); first.live = False
        second = HandController(cfg, side, dry_run=False, bus=bus)
        before = len(bus.history)
        assert second.enable().ok
        assert bus.pulses == held and bus.history[before:] == [held]
        assert second.boot_id == 0x12345678
        second.set_grip("open")
        assert second.step(.02).ok and bus.pulses != held
        second.live = False
        # Bypass the Python guard: the actual C++ firmware must reject the pair.
        bad = list(cfg.hand(side).close_us)
        for ident, data in pose_frames(cfg.hand(side).node, 100, bad):
            bus.send(ident, data)
        assert any(getattr(f, "result", None) == Result.REJECT_PAIR for f in bus.frames)
    finally:
        bus.terminate()

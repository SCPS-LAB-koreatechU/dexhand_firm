"""A DexHand ESP32 (protocol v1 or v2) on a python-can virtual bus, for tests.

It follows firmware/common/HandController.h + PoseReceiver.h. Knobs:
    protocol  1: never answers (v1 firmware), 2: STATUS / PONG
    silent    True: off the bus (no answers at all)
    drop_status  number of upcoming STATUS frames to lose on the way back
    lose_group   group number to lose on the way in (set never completes)
"""

from __future__ import annotations

import struct
import threading
import time

import can

from bhl_hand_sdk.hand_can import F_CONTROL, F_PING, F_PONG, F_POSE, F_STATUS, F_STATE, F_BOOT, Flag, Result, can_id


class FakeHand:
    def __init__(self, channel: str, node: int, low, high, protocol: int = 2, tag: str = "c7b282e4") -> None:
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.node, self.low, self.high = node, list(low), list(high)
        self.protocol, self.tag = protocol, bytes.fromhex(tag)
        self.silent = False
        self.drop_status = 0
        self.lose_group: int | None = None
        self.delay = 0.0              # [s] before each STATUS
        self.pwm_on = False
        self.inhibited = False
        self.applied = 0
        self.pulses: list[int] | None = None
        self.modes: list[int] = []
        self.boot_id = 1
        self._boot_time = time.monotonic()
        self._last_boot = 0.0
        self._started = 0.0
        self._seq = None
        self._mask = 0
        self._pose = [0] * 15
        self._blocked = self._complete = False
        self._inhibited_seq = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.bus.shutdown()

    def flags(self) -> int:
        return (Flag.PWM_ON if self.pwm_on else 0) | (Flag.INHIBITED if self.inhibited else 0) | (Flag.HAVE_POSE if self.pulses is not None else 0)

    def _emit(self, function, data):
        self.bus.send(can.Message(arbitration_id=can_id(function, self.node), data=data, is_extended_id=False))

    def reboot(self):
        self.boot_id += 1
        self.applied = 0
        self.pwm_on = self.inhibited = False
        self.pulses = None
        self._reset()
        self._boot_time = time.monotonic()
        self._last_boot = 0

    def _status(self, seq: int, result: int, detail: int = 0xFF) -> None:
        if self.protocol < 2 or self.silent:
            return
        if self.drop_status > 0:
            self.drop_status -= 1
            return
        if self.delay:
            import time
            time.sleep(self.delay)
        data = struct.pack("<BBBBHBB", seq & 0xFF, result, self.flags(), detail, self.applied & 0xFFFF, 0, 0)
        self.bus.send(can.Message(arbitration_id=can_id(F_STATUS, self.node), data=data, is_extended_id=False))

    def _reset(self) -> None:
        self._seq, self._mask, self._blocked, self._complete = None, 0, False, False
        self._inhibited_seq = None

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            if self.protocol >= 3 and not self.silent and now - self._last_boot >= 1:
                self._last_boot = now
                self._emit(F_BOOT, struct.pack("<II", self.boot_id, int((now - self._boot_time) * 1000)))
            message = self.bus.recv(timeout=0.01)
            if message is None or self.silent:
                continue
            function, node, data = message.arbitration_id >> 7, message.arbitration_id & 0x7F, bytes(message.data)
            if node != self.node:
                continue
            if function == F_PING and len(data) == 2 and data[0] == 0xCB and self.protocol >= 3:
                nonce = data[1]
                self._emit(F_STATE, bytes([nonce, 0xF0]) + struct.pack("<IH", self.boot_id, self.applied & 65535))
                self._emit(F_STATE, bytes([nonce, 0xF1, self.flags(), self._seq or 0, 0, 0, 0, 0]))
                if self.pulses is not None:
                    for g in range(5):
                        self._emit(F_STATE, bytes([nonce, g]) + struct.pack("<3H", *self.pulses[g*3:g*3+3]))
            elif function == F_PING and data[:1] == b"\xCA" and self.protocol >= 2:
                pong = bytes([0xCA, self.protocol, self.node, self.flags()]) + self.tag
                self.bus.send(can.Message(arbitration_id=can_id(F_PONG, self.node), data=pong, is_extended_id=False))
            elif function == F_CONTROL and len(data) in (2, 3) and data[1] == self.node:
                mode = data[0]
                self.modes.append(mode)
                if mode == 3:
                    self._status(data[2] if len(data) == 3 else 0, Result.CONTROL_ACK, mode)
                    continue
                if mode in (0x00, 0x01):
                    self.pwm_on = False
                    self.pulses = None
                self.inhibited = mode in (0x00, 0x02)
                self._reset()
                self._status(data[2] if len(data) == 3 else 0, Result.CONTROL_ACK, mode)
            elif function == F_POSE and len(data) == 8:
                self._pose_frame(data)

    def _pose_frame(self, data: bytes) -> None:
        seq, tag = data[0], data[1]
        group = tag & 0x0F
        if self.inhibited:
            if self._inhibited_seq != seq:
                self._inhibited_seq = seq
                self._status(seq, Result.REJECT_INHIBITED)
            return
        if group == self.lose_group:
            return
        delta = None if self._seq is None else (seq - self._seq) & 0xFF
        if delta is not None and delta >= 128:
            return                                   # old or ambiguous sequence
        if delta != 0:
            self._seq, self._mask, self._blocked, self._complete = seq, 0, False, False
        if self._complete or self._blocked:
            return
        if tag >> 4 != 1 or group >= 5:
            self._blocked = True
            self._status(seq, Result.REJECT_FORMAT)
            return
        if self._mask == 0:
            self._started = time.monotonic()
        elif time.monotonic() - self._started > 0.02:
            self._blocked = True
            self._status(seq, Result.REJECT_DEADLINE)
            return
        values = struct.unpack("<3H", data[2:8])
        if self._mask & (1 << group):
            if self._pose[group*3:group*3+3] != list(values):
                self._blocked = True
                self._status(seq, Result.REJECT_CONFLICT, group*3)
            return
        for i, v in enumerate(values):
            index = group * 3 + i
            if not self.low[index] <= v <= self.high[index]:
                self._blocked = True
                self._status(seq, Result.REJECT_RANGE, index)
                return
            self._pose[index] = v
        self._mask |= 1 << group
        if self._mask == 0x1F:
            self._complete = True
            self.pwm_on = True
            self.applied += 1
            self.pulses = list(self._pose)
            self._status(seq, Result.APPLIED)

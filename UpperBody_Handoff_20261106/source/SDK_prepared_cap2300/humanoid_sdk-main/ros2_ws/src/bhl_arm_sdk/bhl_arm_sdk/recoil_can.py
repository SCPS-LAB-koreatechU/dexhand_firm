"""Recoil motor controller CAN protocol (Berkeley Humanoid Lite actuators).

Frame layout and parameter ids follow berkeley_humanoid_lite_lowlevel/recoil/core.py
at HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel@652777c:

    CAN id = (function << 7) | device_id          (standard 11-bit id)
    NMT             set_mode:  [mode, device_id]
    RECEIVE_SDO     write:     [0x20, param_lo, param_hi, 0, value(4)]
                    read:      [0x40, param_lo, param_hi]  -> TRANSMIT_SDO [value(4), ...]
    RECEIVE_PDO_2   [position_target f32, velocity_target f32]
                    -> TRANSMIT_PDO_2 [position_measured f32, velocity_measured f32]
    HEARTBEAT       empty frame, feeds the firmware watchdog (default 1 s)

Two things are deliberately different from the upstream Bus, both found on
this lab's arms:

  * the PDO reply is filtered by function code TRANSMIT_PDO_2. Upstream takes
    the first frame from the device, so a late SDO reply was read as a position.
  * replies wait on an absolute deadline (default 4 ms) instead of 1 ms, and a
    run of error frames does not restart the wait.

python-can (socketcan) is the only dependency.
"""

from __future__ import annotations

import struct
import threading
import time

import can


class Function:
    NMT = 0b0000
    TRANSMIT_PDO_1 = 0b0011
    RECEIVE_PDO_1 = 0b0100
    TRANSMIT_PDO_2 = 0b0101
    RECEIVE_PDO_2 = 0b0110
    TRANSMIT_SDO = 0b1011
    RECEIVE_SDO = 0b1100
    FLASH = 0b1101
    HEARTBEAT = 0b1110


class Mode:
    DISABLED = 0x00
    IDLE = 0x01
    DAMPING = 0x02
    CALIBRATION = 0x05
    CURRENT = 0x10
    TORQUE = 0x11
    VELOCITY = 0x12
    POSITION = 0x13


class Param:
    DEVICE_ID = 0x000
    FIRMWARE_VERSION = 0x004
    WATCHDOG_TIMEOUT = 0x008
    MODE = 0x010
    ERROR = 0x014
    GEAR_RATIO = 0x01C
    POSITION_KP = 0x020
    VELOCITY_KP = 0x028      # the "kd" of the position loop (upstream write_position_kd)
    TORQUE_LIMIT = 0x030
    POSITION_MEASURED = 0x060
    CURRENT_LIMIT = 0x074  # phase dq current clamp, NOT battery current
    BUS_VOLTAGE = 0x100
    TORQUE_CONSTANT = 0x108


DEVICE_ID_MASK = 0x7F
FUNC_SHIFT = 7


class RecoilBus:
    """One socketcan channel shared by every actuator on it."""

    def __init__(self, channel: str, bitrate: int = 1_000_000, reply_timeout: float = 0.004) -> None:
        self.channel = channel
        self.reply_timeout = reply_timeout
        self._transaction = threading.RLock()
        self.sdo_failures: dict[int, str] = {}
        self._lease = None
        self._bus = can.interface.Bus(interface="socketcan", channel=channel, bitrate=bitrate,
                                      receive_own_messages=False)

    def close(self) -> None:
        try:
            self._bus.shutdown()
        except Exception:  # noqa: BLE001 - shutting down a dead adapter must not raise
            pass
        finally:
            if self._lease: self._lease.close()

    def reserve_nodes(self, nodes):
        from .ownership import NodeLease
        self._lease = NodeLease(self.channel, nodes)

    # -- raw frames -------------------------------------------------------------
    def send(self, device_id: int, function: int, data: bytes = b"") -> None:
        if not 0 <= device_id <= DEVICE_ID_MASK:
            raise ValueError(f"device_id {device_id} out of range")
        with self._transaction:
            self._bus.send(can.Message(arbitration_id=(function << FUNC_SHIFT) | device_id,
                                       is_extended_id=False, data=data), timeout=0.001)

    def exchange_many(self, positions: dict[int, float], timeout=None) -> dict[int, tuple[float, float]]:
        """Send the batch first, collect replies against one shared deadline.

        Recoil PDO has no transaction token. Draining and RX timestamps reject
        queued old data, but cannot prove causality for a delayed same-node PDO.
        """
        with self._transaction:
            for _ in range(512):
                message = self._bus.recv(timeout=0)
                if message is None:
                    break
                if not (message.is_error_frame or message.is_extended_id or message.is_remote_frame):
                    self._observe_sdo_client(message)
            else:
                raise RuntimeError("CAN receive queue did not drain before PDO batch")
            sent_wall = time.time()
            for node, position in positions.items():
                self.send(node, Function.RECEIVE_PDO_2, struct.pack("<ff", position, 0.0))
            deadline = time.monotonic() + (self.reply_timeout if timeout is None else timeout)
            replies = {}
            while len(replies) < len(positions):
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                message = self._bus.recv(timeout=left)
                if message is None:
                    break
                if message.is_error_frame or message.is_extended_id or message.is_remote_frame:
                    continue
                self._observe_sdo_client(message)
                node = message.arbitration_id & DEVICE_ID_MASK
                function = message.arbitration_id >> FUNC_SHIFT
                if node not in positions or node in replies or function != Function.TRANSMIT_PDO_2 or len(message.data) != 8:
                    continue
                if message.timestamp and message.timestamp < sent_wall:
                    continue
                replies[node] = struct.unpack("<ff", message.data)
            return replies

    def receive(self, device_id: int, function: int, timeout: float | None = None) -> bytes | None:
        """First frame from `device_id` with `function`, or None at the deadline."""
        deadline = time.monotonic() + (self.reply_timeout if timeout is None else timeout)
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            message = self._bus.recv(timeout=left)
            if message is None:
                return None
            if message.is_error_frame or message.is_extended_id or message.is_remote_frame:
                continue
            self._observe_sdo_client(message)
            if function == Function.TRANSMIT_SDO and device_id in self.sdo_failures:
                return None
            if (message.arbitration_id & DEVICE_ID_MASK) == device_id and \
                    (message.arbitration_id >> FUNC_SHIFT) == function:
                return bytes(message.data)

    def flush(self) -> None:
        while self._bus.recv(timeout=0) is not None:
            pass

    def _observe_sdo_client(self, message) -> None:
        # This socket does not receive its own TX. SDO responses contain no
        # register address: another writer/reader makes correlation ambiguous.
        if message.arbitration_id >> FUNC_SHIFT == Function.RECEIVE_SDO:
            node = message.arbitration_id & DEVICE_ID_MASK
            nodes = range(1, DEVICE_ID_MASK + 1) if node == 0 else (node,)
            for target in nodes:
                self.sdo_failures[target] = "another SDO client detected; restart after stopping it"

    # -- commands ---------------------------------------------------------------
    def set_mode(self, device_id: int, mode: int) -> None:
        self.send(device_id, Function.NMT, struct.pack("<BB", mode, device_id))

    def feed(self, device_id: int) -> None:
        self.send(device_id, Function.HEARTBEAT)

    def ping(self, device_id: int, timeout: float = 0.05) -> bool:
        with self._transaction:
            self.send(device_id, Function.RECEIVE_PDO_1, b"\xCA")
            reply = self.receive(device_id, Function.TRANSMIT_PDO_1, timeout)
        return bool(reply) and reply[0] == 0xCA

    def write_f32(self, device_id: int, param: int, value: float) -> None:
        self.send(device_id, Function.RECEIVE_SDO, struct.pack("<BHB", 0x01 << 5, param, 0) + struct.pack("<f", value))

    def read_raw(self, device_id: int, param: int, timeout: float = 0.05) -> bytes | None:
        with self._transaction:
            if device_id in self.sdo_failures:
                return None
            # Drop already queued replies before issuing a new read. A timeout
            # permanently quarantines this node's SDO session; a late MODE reply
            # must never be interpreted as e.g. CURRENT_LIMIT on the next read.
            for _ in range(512):
                message = self._bus.recv(timeout=0)
                if message is None:
                    break
                if not (message.is_error_frame or message.is_extended_id or message.is_remote_frame):
                    self._observe_sdo_client(message)
            else:
                self.sdo_failures[device_id] = "receive queue did not drain"
            if device_id in self.sdo_failures:
                return None
            self.send(device_id, Function.RECEIVE_SDO, struct.pack("<BH", 0x02 << 5, param))
            reply = self.receive(device_id, Function.TRANSMIT_SDO, timeout)
            if reply is None or len(reply) not in (4, 8):
                self.sdo_failures.setdefault(device_id, "SDO timeout/invalid reply; restart required")
                return None
            return reply[:4]

    def read_f32(self, device_id: int, param: int, timeout: float = 0.05) -> float | None:
        raw = self.read_raw(device_id, param, timeout)
        return struct.unpack("<f", raw)[0] if raw else None

    def read_u32(self, device_id: int, param: int, timeout: float = 0.05) -> int | None:
        raw = self.read_raw(device_id, param, timeout)
        return struct.unpack("<L", raw)[0] if raw else None

    def write_gains(self, device_id: int, kp: float, kd: float, torque_limit: float) -> None:
        self.write_f32(device_id, Param.POSITION_KP, kp)
        self.write_f32(device_id, Param.VELOCITY_KP, kd)
        self.write_f32(device_id, Param.TORQUE_LIMIT, torque_limit)

    def exchange_pdo_2(self, device_id: int, position: float, velocity: float = 0.0
                       ) -> tuple[float, float] | None:
        """Send a position target, return (measured position, velocity) or None."""
        with self._transaction:
            self.send(device_id, Function.RECEIVE_PDO_2, struct.pack("<ff", position, velocity))
            reply = self.receive(device_id, Function.TRANSMIT_PDO_2)
        if not reply or len(reply) < 8:
            return None
        return struct.unpack("<ff", reply[:8])

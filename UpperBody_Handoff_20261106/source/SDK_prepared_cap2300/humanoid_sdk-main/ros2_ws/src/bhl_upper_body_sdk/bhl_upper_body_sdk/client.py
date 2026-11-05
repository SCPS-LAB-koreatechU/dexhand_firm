"""Reusable nonblocking ROS client. Caller must spin its rclpy node."""
import json
import math
import time

from .contract import CommandBuilder, decode


class UpperBodyClient:
    def __init__(self, node):
        from std_msgs.msg import String
        self.node, self.builder = node, CommandBuilder()
        self._message_type = String
        self.publisher = node.create_publisher(String, "/upper_body/target", 1)
        self.last_result = self.state = None
        self.results = {}
        self._state_received = None
        self._subscriptions = [
            node.create_subscription(String, "/upper_body/result", self._result, 30),
            node.create_subscription(String, "/upper_body/state", self._state, 1)]

    def _result(self, message):
        result = decode(message.data)
        if result.get("session") == self.builder.session:
            self.last_result = result
            sequence = result.get("sequence")
            if type(sequence) is int:
                self.results[sequence] = result
                while len(self.results) > 128:
                    del self.results[next(iter(self.results))]

    def _state(self, message):
        self.state = decode(message.data, max_bytes=65536)
        self._state_received = time.monotonic()

    def get_state(self, max_age_s=0.5):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("positive finite max_age_s required")
        wall = self.node.get_clock().now().nanoseconds / 1e9
        stamp = None if self.state is None else self.state.get("stamp_s")
        if (self._state_received is None or time.monotonic() - self._state_received > max_age_s or
                type(stamp) not in (int, float) or not -0.02 <= wall - stamp <= max_age_s):
            raise RuntimeError("upper-body state missing, expired or clock not synchronized")
        return self.state

    def get_result(self, sequence):
        return self.results.get(sequence)

    def send(self, *, arms_rad=None, hand_left=None, hand_right=None, ttl_s=0.25):
        command = self.builder.build(stamp_s=self.node.get_clock().now().nanoseconds / 1e9,
                                     arms_rad=arms_rad, hand_left=hand_left, hand_right=hand_right, ttl_s=ttl_s)
        self.publisher.publish(self._message_type(data=json.dumps(command, allow_nan=False)))
        return (command["session"], command["sequence"])

    def hold(self):
        """Asynchronous request; caller must inspect the returned future."""
        from std_srvs.srv import Trigger
        if not hasattr(self, "_hold_client"):
            self._hold_client = self.node.create_client(Trigger, "/upper_body/hold")
        if not self._hold_client.service_is_ready():
            raise RuntimeError("upper-body HOLD service unavailable")
        return self._hold_client.call_async(Trigger.Request())


"""ROS coordinator: one full target -> prepare all -> commit all -> receipt.

No CAN ownership, motor enable, automatic fault clearing, or blocking service wait.
"""
from functools import partial
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .contract import decode, PARTS
from .transaction import Coordinator


class UpperBodyNode(Node):
    def __init__(self):
        super().__init__("upper_body")
        self.declare_parameter("parts", "arms,hand_left,hand_right")
        self.declare_parameter("dry_run", True)
        self.parts = tuple(self.get_parameter("parts").value.split(","))
        if not set(self.parts) <= PARTS:
            raise ValueError("invalid upper-body parts")
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.states = {}
        self.tx = self.create_publisher(String, "~/transaction", 30)
        self.result = self.create_publisher(String, "~/result", 30)
        self.state_pub = self.create_publisher(String, "~/state", 1)
        self.core = Coordinator(self.parts, self._emit, self._report)
        self.target = self.create_subscription(String, "~/target", self._target, 1)
        self.create_subscription(String, "~/receipt", self._receipt, 30)
        for part in self.parts:
            topic = "/arm_controller/status" if part == "arms" else f"/{part}/status"
            self.create_subscription(String, topic, partial(self._state, part), 1)
        self.create_service(Trigger, "~/hold", self._hold)
        self.create_service(Trigger, "~/clear_fault", self._clear)
        self.create_timer(0.01, self._tick)
        self.create_timer(0.1, self._publish_state)

    def _wall(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _emit(self, value):
        self.tx.publish(String(data=json.dumps(value, allow_nan=False)))

    def _report(self, value):
        self.result.publish(String(data=json.dumps(value, allow_nan=False)))

    def _state(self, part, message):
        try:
            value = decode(message.data)
            stamp = value.get("stamp_s")
            if type(stamp) not in (int, float) or not -0.02 <= self._wall() - stamp <= 0.5:
                return
            self.states[part] = (time.monotonic(), value)
        except (ValueError, TypeError):
            return

    def issue(self):
        for part in self.parts:
            if part not in self.states:
                return part + ": no status"
            received, value = self.states[part]
            if time.monotonic() - received > 0.5 or not -0.02 <= self._wall() - value["stamp_s"] <= 0.5:
                return part + ": stale status"
            if value.get("dry_run") != self.dry_run or not value.get("upper_body", {}).get("enabled"):
                return part + ": mode mismatch or endpoint not enabled"
            if value.get("estop") or value.get("motion_stopped") or value.get("fault"):
                return part + ": stop/fault latched"
            if part == "arms":
                if not self.dry_run and not value.get("fixed_zero_calibrated"):
                    return "arms: fixed zero not commissioned"
                joints = [s for s in value.get("joints", {}).values() if s.get("enabled")]
                if not joints or any(not s.get("live") or not s.get("feedback_valid") or s.get("fault") or s.get("health_warning") for s in joints):
                    return "arms: joints not all live/fresh/healthy"
            elif not value.get("live") or value.get("held") or not value.get("present"):
                return part + ": not live/healthy"
        return ""

    def _target(self, message):
        command = None
        try:
            if self.count_publishers(self.target.topic_name) != 1:
                self.core.stop("MULTIPLE_TARGET_PUBLISHERS")
                return
            command = decode(message.data)
            issue = self.issue()
            if issue:
                raise ValueError(issue)
            self.core.submit(command, self._wall(), time.monotonic())
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            # No target changed on a rejected input. If already moving, stop the old stream too.
            if self.core.active or self.core.pending:
                self.core.stop("INVALID_INPUT: " + str(error))
            self._report(dict(status="REJECTED", reason=str(error),
                              session=None if command is None else command.get("session"),
                              sequence=None if command is None else command.get("sequence")))

    def _receipt(self, message):
        try:
            self.core.reply(decode(message.data), self._wall(), time.monotonic())
        except (ValueError, TypeError, KeyError):
            return

    def _hold(self, request, response):
        self.core.stop("OPERATOR_HOLD")
        response.success, response.message = True, "HOLD broadcast; inspect per-part state for confirmation"
        return response

    def _clear(self, request, response):
        issue = self.issue()
        if issue:
            response.success, response.message = False, issue
        elif self.core.active or self.core.pending:
            response.success, response.message = False, "HOLD before changing owner"
        else:
            self.core.clear()
            response.success, response.message = True, "cleared without motion; send a fresh command"
        return response

    def _tick(self):
        self.core.tick(self._wall(), time.monotonic())
        if self.core.active or self.core.pending:
            issue = self.issue()
            if issue:
                self.core.stop("PEER_STATE: " + issue)

    def _publish_state(self):
        now = time.monotonic()
        self.state_pub.publish(String(data=json.dumps(dict(
            schema="bhl.upper_body.state/1", stamp_s=self._wall(), dry_run=self.dry_run,
            ready=not bool(self.issue()) and not self.core.fault, fault=self.core.fault,
            active_command=None if self.core.active is None else dict(session=self.core.active["session"], sequence=self.core.active["sequence"]),
            readiness_issue=self.issue(), result=self.core.last_result,
            parts={p: dict(age_s=now-t, state=v) for p, (t, v) in self.states.items()},
            physical_simultaneity_guaranteed=False), allow_nan=False)))

    def destroy_node(self):
        self.core.stop("COORDINATOR_SHUTDOWN")
        super().destroy_node()


def main():
    rclpy.init()
    node = UpperBodyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

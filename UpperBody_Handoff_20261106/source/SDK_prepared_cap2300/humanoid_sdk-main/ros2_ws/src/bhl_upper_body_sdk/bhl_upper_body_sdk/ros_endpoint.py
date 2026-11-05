"""Attach an optional integrated command endpoint to an existing ROS node."""
import json
import time

from std_msgs.msg import String

from .contract import decode
from .transaction import Endpoint


class RosEndpoint:
    def __init__(self, node, part, adapter):
        self.node = node
        node.declare_parameter("upper_body_only", False)
        self.enabled = bool(node.get_parameter("upper_body_only").value)
        self.endpoint = None
        if not self.enabled:
            return
        self.pub = node.create_publisher(String, "/upper_body/receipt", 30)
        self.endpoint = Endpoint(part, adapter.validate, adapter.apply, adapter.hold,
                                 lambda value: self.pub.publish(String(data=json.dumps(value, allow_nan=False))))
        self.sub = node.create_subscription(String, "/upper_body/transaction", self.receive, 30)

    def receive(self, message):
        try:
            if self.node.count_publishers(self.sub.topic_name) != 1:
                self.endpoint.receive(dict(phase="hold", reason="MULTIPLE_COORDINATORS"), 0, 0)
                return
            self.endpoint.receive(decode(message.data), self.node.get_clock().now().nanoseconds / 1e9, time.monotonic())
        except (ValueError, TypeError, KeyError) as error:
            self.node.get_logger().error("upper-body transaction: " + str(error))

    def tick(self):
        if self.endpoint:
            try:
                self.endpoint.tick(self.node.get_clock().now().nanoseconds / 1e9, time.monotonic())
            except ValueError as error:
                self.node.get_logger().error(str(error))

    def status(self):
        return {"enabled": self.enabled, **({} if self.endpoint is None else self.endpoint.status())}

    def command_identity(self):
        if self.endpoint is None or self.endpoint.active is None:
            return None
        command = self.endpoint.active["command"]
        return dict(session=command["session"], sequence=command["sequence"])


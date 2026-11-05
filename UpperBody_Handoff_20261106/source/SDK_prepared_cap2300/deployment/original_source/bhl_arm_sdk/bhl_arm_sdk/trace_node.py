"""Read-only ROS status recorder, separate process so file I/O cannot block control."""
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TraceNode(Node):
    def __init__(self):
        super().__init__("upper_body_trace")
        self.declare_parameter("output", "~/.local/state/bhl/upper_body.jsonl")
        path = Path(self.get_parameter("output").value).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.sink = logging.getLogger("bhl_status_trace")
        self.sink.setLevel(logging.INFO)
        self.sink.propagate = False
        self.handler = RotatingFileHandler(path, maxBytes=5_000_000, backupCount=3, encoding="utf-8")
        self.handler.setFormatter(logging.Formatter("%(message)s"))
        self.sink.addHandler(self.handler)
        for name, topic in (("arm", "/arm_controller/status"), ("left", "/hand_left/status"), ("right", "/hand_right/status")):
            self.create_subscription(String, topic, lambda m, n=name: self.record(n, m), 20)
        self.get_logger().info("Read-only status trace: " + str(path))

    def record(self, name, message):
        try:
            status = json.loads(message.data)
            self.sink.info(json.dumps({"received_unix": time.time(), "received_monotonic": time.monotonic(),
                                       "source": name, "status": status}, ensure_ascii=False, allow_nan=False))
        except (ValueError, TypeError):
            self.get_logger().warn("invalid status from " + name)

    def destroy_node(self):
        self.sink.removeHandler(self.handler)
        self.handler.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TraceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

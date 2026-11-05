"""Dry-run only: stream the simulated current pose through the integrated API.

Enable simulated controllers first. No implicit enable or fault reset is issued.
Run after sourcing ROS and the built workspace. Never sends to live-mode nodes.
"""
import math

import rclpy
from rclpy.node import Node

from bhl_upper_body_sdk.client import UpperBodyClient


class Demo(Node):
    def __init__(self):
        super().__init__("upper_body_example")
        self.client = UpperBodyClient(self)
        self.pose = None
        self.create_timer(0.02, self.tick)

    def tick(self):
        try:
            state = self.client.get_state()
        except RuntimeError:
            return
        if not state.get("dry_run") or not state.get("ready"):
            return
        if self.pose is None:
            parts = state["parts"]
            self.pose = dict(
                arms_rad={n: math.radians(s["command_deg"]) for n, s in parts["arms"]["state"]["joints"].items()},
                hand_left=parts["hand_left"]["state"]["command"],
                hand_right=parts["hand_right"]["state"]["command"])
        self.client.send(**self.pose, ttl_s=0.25)


def main():
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Scripted move, the same phases as Unitree's g1_arm7_sdk_dds_example.cpp:

    1. enable    -- position mode holding the pose the arm is in (joint 0 = here)
    2. target    -- go to the requested angles, slew-limited by the controller
    3. hold      -- stay there for --hold seconds
    4. release   -- back to 0 deg on every joint that moved
    5. stop      -- damping, nothing commanded

Each step waits until the measured angles reach the goal (or --timeout).
Talks to a running arm_controller, so dry run / live is decided there.

    ros2 run bhl_arm_sdk arm_sequence --pose arm_left_elbow_pitch_joint=30 \\
        arm_right_elbow_pitch_joint=-30 --hold 2
    ros2 run bhl_arm_sdk arm_sequence --pose arm_left_shoulder_pitch_joint=-20 --no-release
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import String
from std_srvs.srv import SetBool


class Sequence(Node):
    def __init__(self, controller: str) -> None:
        super().__init__("arm_sequence")
        self.status: dict | None = None
        self.create_subscription(String, f"{controller}/status",
                                 lambda m: setattr(self, "status", json.loads(m.data)), 10)
        self.target_client = self.create_client(SetParametersAtomically, f"{controller}/set_parameters_atomically")
        self.enable_client = self.create_client(SetBool, f"{controller}/enable")

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call_enable(self, on: bool) -> str:
        if not self.enable_client.wait_for_service(timeout_sec=5.0):
            raise SystemExit("arm_controller not found (ros2 launch bhl_arm_sdk arm_sdk.launch.py)")
        future = self.enable_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            raise SystemExit("enable/disable failed: " + (result.message if result else "no reply"))
        return result.message

    def send(self, goals: dict[str, float]) -> None:
        if not self.target_client.wait_for_service(timeout_sec=5.0):
            raise SystemExit("target parameter service unavailable")
        parameters = [ParameterMsg(name=f"target.{name}", value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(degrees))) for name, degrees in goals.items()]
        future = self.target_client.call_async(SetParametersAtomically.Request(parameters=parameters))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result() if future.done() else None
        if response is None or not response.result.successful:
            raise SystemExit("target rejected: " + (response.result.reason if response else "no reply"))

    def wait_reached(self, goals: dict[str, float], tolerance: float, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin_for(0.05)
            joints = (self.status or {}).get("joints", {})
            if (self.status or {}).get("estop") or (self.status or {}).get("motion_stopped"):
                print(f"E-STOP: {self.status.get('message')}")
                return False
            if all(n in joints and joints[n]["live"] and
                   not joints[n].get("fault") and
                   abs(joints[n]["goal_deg"] - goals[n]) <= 1e-3 and
                   abs(joints[n]["measured_deg"] - goals[n]) <= tolerance and
                   abs(joints[n]["goal_deg"] - joints[n]["command_deg"]) <= 1e-3 for n in goals):
                return True
        return False


def parse_pose(items: list[str]) -> dict[str, float]:
    pose = {}
    for item in items:
        name, _, value = item.partition("=")
        if not value:
            raise SystemExit(f"--pose wants joint=degrees, got '{item}'")
        pose[name] = float(value)
    return pose


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--pose", nargs="+", required=True, help="joint=degrees ...")
    parser.add_argument("--hold", type=float, default=2.0, help="[s] at the target")
    parser.add_argument("--tolerance", type=float, default=1.0, help="[deg] counts as reached")
    parser.add_argument("--timeout", type=float, default=20.0, help="[s] per move")
    parser.add_argument("--no-release", action="store_true", help="stay at the target, torque on")
    parser.add_argument("--controller", default="/arm_controller")
    args, _ros = parser.parse_known_args()
    pose = parse_pose(args.pose)

    rclpy.init(args=sys.argv)
    node = Sequence(args.controller)
    try:
        node.spin_for(0.5)
        if node.status is None:
            raise SystemExit("no status from arm_controller")
        unknown = [n for n in pose if n not in node.status["joints"]]
        if unknown:
            raise SystemExit(f"unknown joints: {unknown}\nknown: {list(node.status['joints'])}")
        mode = "dry run" if node.status["dry_run"] else "LIVE"
        print(f"[1/5] enable ({mode}): {node.call_enable(True)}")
        node.spin_for(0.3)
        print(f"[2/5] target: {pose}")
        node.send(pose)
        reached = node.wait_reached(pose, args.tolerance, args.timeout)
        joints = node.status["joints"]
        print("      " + ", ".join(f"{n} {joints[n]['measured_deg']:+.1f}°" for n in pose)
              + ("" if reached else "  (not reached in time)"))
        if not reached:
            raise SystemExit("requested pose was not reached; no automatic return/release")
        print(f"[3/5] hold {args.hold:.1f} s")
        node.spin_for(args.hold)
        if args.no_release:
            print("[4/5] --no-release: staying here, torque on")
            return
        home = {n: 0.0 for n in pose}
        print("[4/5] release: back to 0 deg")
        node.send(home)
        if not node.wait_reached(home, args.tolerance, args.timeout):
            raise SystemExit("return pose was not reached; refusing automatic output release")
        print(f"[5/5] stop: {node.call_enable(False)}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

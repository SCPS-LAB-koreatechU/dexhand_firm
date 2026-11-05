"""ROS 2 node: per-motor angle control of the BHL arms.

Set a joint's angle by parameter (degrees, clamped to the joint's limits):

    ros2 param set /arm_controller target.arm_left_elbow_pitch_joint 30.0

or several at once from a launch / YAML file (`target.<joint>: <deg>`), or by
topic (sensor_msgs/JointState, radians, any subset of joints):

    ros2 topic pub --once /arm_controller/target sensor_msgs/msg/JointState \\
        "{name: [arm_left_elbow_pitch_joint], position: [0.5]}"

Nothing moves until the joints are enabled:

    ros2 service call /arm_controller/enable std_srvs/srv/SetBool "{data: true}"

Each joint then slews to its target at most `max_speed_deg` deg/s.

  ~/enable        std_srvs/SetBool   true: position mode holding the current pose
                                     false: damping, then nothing is commanded
  ~/estop         std_srvs/Trigger   damping on every joint, latched
  ~/reset_estop   std_srvs/Trigger
  ~/home          std_srvs/Trigger   every target to 0 deg
  ~/target        sensor_msgs/JointState  (in)  goals [rad]
  /joint_states   sensor_msgs/JointState  (out) measured [rad]
  ~/status        std_msgs/String    (out) JSON: per joint measured / command / goal / fault
  /diagnostics    diagnostic_msgs/DiagnosticArray (out, 1 Hz) the same, one status per
                                     joint, for rqt_robot_monitor / diagnostic_aggregator

Parameters: config (YAML path), dry_run (default true: no CAN is opened, the
angle follows the command), max_speed_deg, target.<joint> [deg].

Under bhl_bringup robot.launch.py, /joint_states is remapped to
/arm_controller/joint_states and joint_state_publisher + robot_state_publisher
turn it into the full-URDF /joint_states and /tf.
"""

from __future__ import annotations

from functools import partial
import json
import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, FloatingPointRange
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .config import load_config
from .controller import ArmController
from .diagnostics import diagnostics
from .coordination import PeerMonitor
from bhl_upper_body_sdk.adapters import ArmAdapter
from bhl_upper_body_sdk.ros_endpoint import RosEndpoint

DIAGNOSTIC_LEVELS = (DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR, DiagnosticStatus.STALE)


class ArmControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_controller")
        default_config = Path(get_package_share_directory("bhl_arm_sdk")) / "config" / "bhl_arms.yaml"
        self.declare_parameter("config", str(default_config))
        self.declare_parameter("dry_run", True)
        self.declare_parameter("required_hands", "")
        config = load_config(self.get_parameter("config").value)
        self.declare_parameter("max_speed_deg", config.max_speed_deg,
                               ParameterDescriptor(description="per-joint slew limit [deg/s]"))
        dry_run = bool(self.get_parameter("dry_run").value)
        config.max_speed_deg = float(self.get_parameter("max_speed_deg").value)
        if not math.isfinite(config.max_speed_deg) or config.max_speed_deg <= 0:
            raise ValueError("max_speed_deg must be finite and positive")

        self.arm = ArmController(config, dry_run=dry_run)
        self.upper = RosEndpoint(self, "arms", ArmAdapter(self.arm))
        self.dt = 1.0 / config.control_rate_hz
        self._last_tick = None
        self._last_stop_state = (False, False)
        self._syncing_targets = False
        expected = [n.strip() for n in str(self.get_parameter("required_hands").value).split(",") if n.strip()]
        if any(n not in ("left", "right") for n in expected):
            raise ValueError("required_hands must contain left/right")
        self.peers = PeerMonitor(expected, dry_run=dry_run)
        for side in expected:
            self.create_subscription(String, f"/hand_{side}/status", partial(self._on_peer, side), 10)

        # One parameter per joint: target.<joint> in degrees, with the joint's limits.
        for spec in config.joints:
            self.declare_parameter(
                f"target.{spec.name}", 0.0,
                ParameterDescriptor(
                    description=f"{spec.name} target [deg] (CAN {spec.device_id} on {spec.bus})",
                    floating_point_range=[FloatingPointRange(
                        from_value=float(spec.lower_deg), to_value=float(spec.upper_deg), step=0.0)]))
        self.add_on_set_parameters_callback(self._on_parameters)

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.target_sub = self.create_subscription(JointState, "~/target", self._on_target, 1)
        self.create_service(SetBool, "~/enable", self._on_enable)
        self.create_service(Trigger, "~/estop", self._on_estop)
        self.create_service(Trigger, "~/reset_estop", self._on_reset_estop)
        self.create_service(Trigger, "~/home", self._on_home)
        self.create_service(Trigger, "~/hold", self._on_hold)
        self.create_service(Trigger, "~/resume", self._on_resume)
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.1, self._publish_status)
        self.create_timer(1.0, self._publish_diagnostics)

        mode = "DRY RUN (no CAN opened)" if dry_run else f"LIVE on {sorted(set(config.buses.values()))}"
        self.get_logger().info(
            f"{mode}: {len(config.joints)} joints, {config.control_rate_hz:.0f} Hz, "
            f"max {config.max_speed_deg:.0f} deg/s, zero at {config.zero}. "
            "Output is off -- call ~/enable.")

    # ------------------------------------------------------------- parameters
    def _on_parameters(self, params: list[Parameter]) -> SetParametersResult:
        goals, speed = {}, None
        for param in params:
            if param.name.startswith("target."):
                name = param.name[len("target."):]
                if name not in self.arm.joints or param.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    return SetParametersResult(successful=False, reason="invalid target name/type")
                spec, value = self.arm.joints[name].spec, float(param.value)
                if not math.isfinite(value) or not spec.lower_deg <= value <= spec.upper_deg:
                    return SetParametersResult(successful=False, reason=f"{name}: target outside calibrated range")
                goals[name] = value
            elif param.name == "max_speed_deg":
                speed = float(param.value)
                if not math.isfinite(speed) or speed <= 0:
                    return SetParametersResult(successful=False, reason="max_speed_deg must be positive")
            elif param.name in ("config", "dry_run", "required_hands", "upper_body_only"):
                return SetParametersResult(successful=False, reason=f"{param.name} is fixed at start-up")
        try:
            if goals and not self._syncing_targets:
                if self.upper.enabled:
                    raise ValueError("integrated mode: use UpperBodyClient, not target parameters")
                self.arm.set_goals(goals)
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))
        if speed is not None:
            self.arm.max_speed_deg = speed
        return SetParametersResult(successful=True)

    def _sync_target_params(self, goals: dict[str, float]) -> None:
        """Keep target.<joint> equal to what was actually commanded by topic / service."""
        self._syncing_targets = True
        try:
            self.set_parameters([Parameter(f"target.{n}", Parameter.Type.DOUBLE, float(v)) for n, v in goals.items()])
        finally:
            self._syncing_targets = False

    # ------------------------------------------------------------------ inputs
    def _on_target(self, message: JointState) -> None:
        if self.upper.enabled:
            self.get_logger().warn("integrated mode: individual arm targets are disabled")
            return
        if self.count_publishers(self.target_sub.topic_name) > 1:
            self.arm.hold("MULTIPLE_TARGET_PUBLISHERS")
            self.get_logger().error("Multiple target publishers: motion held")
            return
        if (not message.name or len(message.name) != len(message.position) or
                len(set(message.name)) != len(message.name) or
                any(n not in self.arm.joints for n in message.name) or
                not all(math.isfinite(p) for p in message.position)):
            self.get_logger().warn("Rejected malformed arm target; no partial update")
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        if stamp and not -0.1 <= now - stamp <= self.arm.config.command_timeout:
            self.get_logger().warn("Rejected expired/future arm target")
            return
        goals = {n: math.degrees(p) for n, p in zip(message.name, message.position)}
        try:
            applied = self.arm.set_goals(goals, streaming=True, source="target")
            self._sync_target_params(applied)
        except ValueError as error:
            self.get_logger().warn(str(error))

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            issue = self.peers.issue()
            if issue:
                response.success, response.message = False, "required hand unavailable: " + issue
                return response
            try:
                up = self.arm.enable()
            except Exception as error:
                self.arm.hold("ENABLE_IO_FAILURE")
                response.success, response.message = False, "enable failed: " + str(error)
                return response
            # start from where the arm is: targets follow the measured pose
            self._sync_target_params({n: self.arm.joints[n].goal_deg for n in up})
            faults = {n: s.fault for n, s in self.arm.joints.items() if s.fault}
            expected = [s.spec.name for s in self.arm.joints.values() if s.spec.enabled]
            response.success = set(up) == set(expected) and bool(up)
            if not response.success and self.arm.config.stop_on_joint_fault:
                self.arm.hold("PARTIAL_ENABLE")
            response.message = f"enabled {up}" + (f"; not enabled {faults}" if faults else "")
        else:
            self.arm.disable()
            response.success = True
            response.message = "disabled (damping)"
        self.get_logger().warn(response.message)
        return response

    def _on_estop(self, _request, response: Trigger.Response) -> Trigger.Response:
        self.arm.estop()
        response.success, response.message = True, "E-STOP: every joint in damping, latched"
        self.get_logger().error(response.message)
        return response

    def _on_reset_estop(self, _request, response: Trigger.Response) -> Trigger.Response:
        self.arm.reset_estop()
        response.success, response.message = True, self.arm.message
        return response

    def _on_home(self, _request, response: Trigger.Response) -> Trigger.Response:
        if self.upper.enabled:
            response.success, response.message = False, "integrated mode: use UpperBodyClient"
            return response
        try:
            goals = self.arm.set_goals({n: 0.0 for n in self.arm.joints})
        except ValueError as error:
            response.success, response.message = False, str(error)
            return response
        self._sync_target_params(goals)
        response.success, response.message = True, "every target set to 0 deg"
        return response

    def _on_peer(self, name, message):
        try:
            self.peers.update(name, json.loads(message.data))
        except (ValueError, TypeError, AttributeError):
            return

    def _on_hold(self, _request, response):
        self.arm.hold("HOLD service / coordinated stop")
        response.success, response.message = True, self.arm.message
        return response

    def _on_resume(self, _request, response):
        issue = self.peers.issue()
        response.success = not issue and self.arm.resume_motion()
        response.message = self.arm.message if response.success else "cannot resume: verify feedback, mode and faults"
        return response

    # -------------------------------------------------------------------- loop
    def _tick(self) -> None:
        self.upper.tick()
        was_estopped = self.arm.estopped
        now = time.monotonic()
        elapsed = self.dt if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        issue = self.peers.issue()
        if issue and self.arm.any_live and not self.arm.motion_stopped:
            self.arm.hold("PEER_FAULT: " + issue)
        self.arm.step(elapsed)
        stop_state = (self.arm.motion_stopped, self.arm.estopped)
        if stop_state != self._last_stop_state:
            self._sync_target_params({n: s.goal_deg for n, s in self.arm.joints.items()})
            self._last_stop_state = stop_state
        if self.arm.estopped and not was_estopped:
            self.get_logger().error(self.arm.message)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        for name, state in self.arm.joints.items():
            if not self.arm.feedback_valid(state):
                continue
            message.name.append(name)
            message.position.append(math.radians(state.measured_deg))
            message.velocity.append(math.radians(state.velocity_dps))
        self.joint_pub.publish(message)

    def _publish_status(self) -> None:
        status = self.arm.status()
        status.update(stamp_s=self.get_clock().now().nanoseconds / 1e9, upper_body=self.upper.status())
        events = {name: info["fault"] or info["health_warning"]
                  for name, info in status["joints"].items()}
        previous = getattr(self, "_reported_health", {})
        for name, event in events.items():
            if event and event != previous.get(name):
                self.get_logger().warn(f"{name}: {event}")
        self._reported_health = events
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def _publish_diagnostics(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for entry in diagnostics(self.arm.status()):
            status = DiagnosticStatus(level=DIAGNOSTIC_LEVELS[entry["level"]], name=entry["name"],
                                      message=entry["message"], hardware_id=entry["hardware_id"])
            status.values = [KeyValue(key=k, value=v) for k, v in entry["values"].items()]
            message.status.append(status)
        self.diagnostics_pub.publish(message)

    def destroy_node(self) -> None:
        self.arm.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

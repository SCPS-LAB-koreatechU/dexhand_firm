"""Execute the real node callbacks in a small queued ROS stand-in.

This tests application wiring and semantics, not DDS discovery/QoS/executor timing.
No ROS installation, socket, motor or CAN access is used.
"""
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


@pytest.fixture
def ros(monkeypatch):
    queue, pubs, subs = [], {}, {}
    SRC = Path(__file__).resolve().parents[2]

    class Message:
        def __init__(self, **kw):
            self.__dict__.update(kw)
            self.header = NS(stamp=NS(sec=0, nanosec=0))
            if not hasattr(self, "data"): self.data = []
            self.name, self.position, self.velocity = [], [], []
    class Param:
        class Type:
            DOUBLE = 1
            INTEGER = 2
        def __init__(self, name, type_=None, value=None):
            self.name, self.type_, self.value = name, type_, value
    class Service:
        Request = Message
        Response = Message
    class Node:
        overrides = {}
        def __init__(self, name):
            self.name = self.overrides.get("node_name", name)
            self.params, self.callbacks, self.services = {}, [], {}
        def resolve(self, name): return name.replace("~", "/" + self.name)
        def declare_parameter(self, name, value, *args): self.params[name] = self.overrides.get(name, value)
        def get_parameter(self, name): return NS(value=self.params[name])
        def add_on_set_parameters_callback(self, callback): self.callbacks.append(callback)
        def set_parameters(self, parameters):
            result = [cb(parameters) for cb in self.callbacks]
            if not result or all(r.successful for r in result):
                self.params.update({p.name: p.value for p in parameters})
            return result
        def get_clock(self): return NS(now=lambda: NS(nanoseconds=100_000_000_000, to_msg=lambda: NS(sec=100, nanosec=0)))
        def get_logger(self): return NS(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)
        def create_publisher(self, msgtype, name, depth):
            name = self.resolve(name)
            pubs[name] = pubs.get(name, 0) + 1
            return NS(publish=lambda m: queue.append((name, m)))
        def create_subscription(self, msgtype, name, cb, depth):
            name = self.resolve(name)
            subs.setdefault(name, []).append(cb)
            return NS(topic_name=name)
        def count_publishers(self, name): return pubs.get(name, 0)
        def create_service(self, srv, name, cb): self.services[self.resolve(name)] = cb
        def create_client(self, *args): return NS(service_is_ready=lambda: False)
        def create_timer(self, *args): pass
        def destroy_node(self): pass

    def module(name, **attrs):
        value = ModuleType(name)
        value.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, value)
    module("rclpy", init=lambda: None)
    module("rclpy.node", Node=Node)
    module("rclpy.parameter", Parameter=Param)
    module("std_msgs.msg", String=Message, Float64MultiArray=Message, UInt16MultiArray=Message)
    module("std_srvs.srv", Trigger=Service, SetBool=Service)
    module("sensor_msgs.msg", JointState=Message)
    module("diagnostic_msgs.msg", DiagnosticArray=Message, KeyValue=Message,
           DiagnosticStatus=NS(OK=0, WARN=1, ERROR=2, STALE=3))
    module("rcl_interfaces.msg", FloatingPointRange=Message, ParameterDescriptor=Message, SetParametersResult=Message)
    module("ament_index_python.packages", get_package_share_directory=lambda name: str(SRC / name))
    # ros_endpoint may have been imported by another fixture; replace its message class too.
    import bhl_upper_body_sdk.ros_endpoint as endpoint_module
    monkeypatch.setattr(endpoint_module, "String", Message)

    def load(package):
        spec = importlib.util.spec_from_file_location(package + "._test_ros_node", SRC / package / package / "node.py")
        value = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(value)
        return value
    def drain():
        for _ in range(200):
            if not queue: return
            name, message = queue.pop(0)
            for callback in subs.get(name, []): callback(message)
        raise AssertionError("ROS stand-in message loop did not drain")
    def make(package, cls, **overrides):
        Node.overrides = dict(upper_body_only=True, **overrides)
        return getattr(load(package), cls)()
    return NS(make=make, drain=drain, Message=Message, NS=NS, pubs=pubs)


def test_real_ros_callbacks_route_both_hands_and_arms_and_block_legacy_inputs(ros):
    from bhl_upper_body_sdk.client import UpperBodyClient
    arm = ros.make("bhl_arm_sdk", "ArmControllerNode")
    left = ros.make("bhl_hand_sdk", "HandControllerNode", hand="left", node_name="hand_left")
    right = ros.make("bhl_hand_sdk", "HandControllerNode", hand="right", node_name="hand_right")
    upper = ros.make("bhl_upper_body_sdk", "UpperBodyNode")
    arm.arm.enable(); left.hand.enable(); right.hand.enable()
    for node in (arm, left, right): node._publish_status()
    ros.drain()
    assert not upper.issue()
    client = UpperBodyClient(upper)
    upper._publish_state()
    ros.drain()
    assert client.get_state()["ready"]
    session, sequence = client.send(arms_rad={n: 0 for n in arm.arm.joints},
                                    hand_left=[.2]*15, hand_right=[.1]*15)
    ros.drain()
    assert upper.core.last_result["status"] == "ACCEPTED"
    assert client.get_result(sequence)["status"] == "ACCEPTED"
    left._tick(); right._tick()
    assert left.hand.status()["last_applied_command"]["context"] == dict(session=session, sequence=sequence)
    assert left.hand.goal == [.2]*15 and right.hand.goal == [.1]*15
    left._on_target(ros.Message(data=[.8]*15))
    assert left.hand.goal == [.2]*15
    assert left._open().ok is False
    arm._on_target(ros.Message())
    upper._hold(None, ros.NS())
    ros.drain()
    assert left.hand.held and right.hand.held and arm.arm.motion_stopped


def test_client_state_expiry_is_checked_at_read_time(ros):
    from bhl_upper_body_sdk.client import UpperBodyClient
    upper = ros.make("bhl_upper_body_sdk", "UpperBodyNode")
    client = UpperBodyClient(upper)
    with pytest.raises(RuntimeError):
        client.get_state()
    client._state(ros.Message(data=json.dumps(dict(stamp_s=99))))
    with pytest.raises(RuntimeError):
        client.get_state()


def test_cold_open_is_allowed_but_not_implicit_and_off_requires_new_goal(ros):
    left = ros.make("bhl_hand_sdk", "HandControllerNode", hand="left", node_name="hand_left")
    assert left._open().ok
    left.hand.enable()
    assert not left._open().ok
    left.hand.off()
    assert left._open().ok


def test_status_exposes_unknown_finger_feedback_and_command_receipt(ros):
    left = ros.make("bhl_hand_sdk", "HandControllerNode", hand="left", node_name="hand_left")
    left._publish_status()
    status = left.hand.status()
    assert status["physical_reached"] is None and status["measured_position"] is None
    assert status["command_settled"] == status["settled"]

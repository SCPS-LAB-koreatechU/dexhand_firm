"""Linux ROS acceptance runner. Launches ONLY dry_run=true, in a separate ROS domain.

Tests actual DDS/callback/service routing and goal/PWM confirmation semantics.
Does not test real CAN, motor feedback, physical holding or hardware timing.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=187)
    parser.add_argument("--report", default="ros_dry_run_report.json")
    args = parser.parse_args()
    if os.name != "posix" or not 0 <= args.domain <= 200:
        raise SystemExit("Run on Linux with ROS 2 sourced; domain must be 0..200")
    if Path(args.report).exists():
        raise SystemExit("Report exists; choose a new --report path")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger, SetBool
    from bhl_upper_body_sdk.client import UpperBodyClient
    rclpy.init()
    node = Node("upper_body_acceptance")
    client = UpperBodyClient(node)
    process = None
    report = dict(passed=False, hardware_accessed=False, real_can_tested=False,
                  domain=args.domain, tests=[])
    log = tempfile.TemporaryFile(mode="w+t", encoding="utf-8")

    def wait(predicate, timeout=10):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate(): return
            if process is not None and process.poll() is not None:
                raise RuntimeError("launch exited before acceptance completed")
        raise TimeoutError("acceptance condition not reached")

    def call(name, service, **values):
        connection = node.create_client(service, name)
        wait(connection.service_is_ready)
        request = service.Request()
        for key, value in values.items(): setattr(request, key, value)
        future = connection.call_async(request)
        wait(future.done)
        response = future.result()
        if not response.success: raise RuntimeError(name + ": " + response.message)
        node.destroy_client(connection)

    try:
        # Give DDS discovery a chance before refusing an occupied domain.
        until = time.monotonic() + 2
        while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=0.1)
        if any(name in {"upper_body", "arm_controller", "hand_left", "hand_right"}
               for name, namespace in node.get_node_names_and_namespaces()):
            raise RuntimeError("domain already has robot nodes; choose an unused --domain")
        process = subprocess.Popen(["ros2", "launch", "bhl_bringup", "robot.launch.py",
            "hands:=both", "upper_body:=true", "dry_run:=true", "publish_tf:=false"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        wait(lambda: client.state is not None)
        call("/hand_left/open", Trigger)
        call("/hand_right/open", Trigger)
        call("/hand_left/enable", SetBool, data=True)
        call("/hand_right/enable", SetBool, data=True)
        call("/arm_controller/enable", SetBool, data=True)
        wait(lambda: client.get_state().get("ready"))
        state = client.get_state()
        if not state["dry_run"]: raise RuntimeError("refusing live-mode state")
        names = state["parts"]["arms"]["state"]["joints"]
        session, sequence = client.send(arms_rad={name: 0.0 for name in names},
                                        hand_left=[0.0]*15, hand_right=[0.0]*15, ttl_s=0.5)
        wait(lambda: client.get_result(sequence) is not None, timeout=2)
        result = client.get_result(sequence)
        if result["status"] != "ACCEPTED": raise RuntimeError("goal not accepted: " + str(result))
        report["tests"].append("all three PC controllers accepted the same command")
        def pwm_confirmed():
            current = client.get_state()
            for part in ("hand_left", "hand_right"):
                hand = current["parts"][part]["state"]
                applied = hand.get("last_applied_command") or {}
                if applied.get("context") != dict(session=session, sequence=sequence): return False
                if applied.get("source") != "simulated" or hand.get("physical_reached") is not None:
                    raise RuntimeError("incorrect simulated/physical state semantics")
            return True
        wait(pwm_confirmed, timeout=2)
        report["tests"].append("both simulated applied PWM states correlated with PC command; physical arrival unknown")
        wait(lambda: bool(client.get_state().get("fault")), timeout=2)
        wait(lambda: client.get_state()["parts"]["arms"]["state"]["motion_stopped"] and
                     all(client.get_state()["parts"][p]["state"]["held"] for p in ("hand_left", "hand_right")), timeout=2)
        report["tests"].append("expired upper input latched HOLD in all controllers")
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.seek(0)
        report["launch_log_tail"] = log.read()[-12000:]
        log.close()
        Path(args.report).write_text(json.dumps(report, indent=2), encoding="utf-8")
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(report, indent=2))
    if not report["passed"]: raise SystemExit(1)


if __name__ == "__main__":
    main()

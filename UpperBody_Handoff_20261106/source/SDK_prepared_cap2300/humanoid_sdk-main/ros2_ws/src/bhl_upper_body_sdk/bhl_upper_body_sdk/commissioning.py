"""Offline commissioning records. Never reads/writes CAN or invents measurements."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from .contract import number


def fixed_zero_config(config, samples):
    result = copy.deepcopy(config)
    joints = result.get("joints", {})
    defaults = result.get("defaults", {})
    enabled = {n for n, v in joints.items() if {**defaults, **(v or {})}.get("enabled", True)}
    if not enabled or set(samples.get("joints", {})) != enabled:
        raise ValueError("samples must contain every enabled joint exactly once")
    if samples.get("reference_pose_verified") is not True or samples.get("encoder_reference_persistent_verified") is not True:
        raise ValueError("verify physical reference pose and encoder reference persistence across reboot first")
    for name in enabled:
        settings = {**defaults, **(joints[name] or {})}
        sample = samples["joints"][name]
        if not isinstance(sample, dict) or set(sample) != {"motor_position", "joint_rad"}:
            raise ValueError(name + ": motor_position and independently known joint_rad required")
        if not all(number(v) for v in sample.values()):
            raise ValueError(name + ": finite measurements required")
        ratio, direction = settings.get("gear_ratio", 15.0), settings.get("direction", 1)
        if not number(ratio) or ratio <= 0 or direction not in (-1, 1):
            raise ValueError(name + ": invalid direction/ratio")
        angle_deg = math.degrees(sample["joint_rad"])
        if not settings.get("lower_deg", -45) <= angle_deg <= settings.get("upper_deg", 45):
            raise ValueError(name + ": reference joint angle outside configured limits")
        joints[name] = {**(joints[name] or {}),
                        "motor_zero": sample["motor_position"] - direction * ratio * sample["joint_rad"]}
    result.setdefault("control", {}).update(zero="config", require_fixed_zero_for_streaming=True)
    result["commissioning"] = dict(reference_pose_verified=True, encoder_reference_persistent_verified=True,
        recorded_at=datetime.now(timezone.utc).isoformat(),
        measurements_sha256=hashlib.sha256(json.dumps(samples, sort_keys=True, allow_nan=False).encode()).hexdigest(),
        note="Operator supplied measurements; not automatically certified by SDK")
    return result


def model_issues(model, arm_config, base=Path(".")):
    issues = []
    link_names = None
    if not isinstance(model, dict):
        return ["robot model must be a mapping"]
    urdf = model.get("urdf")
    if not isinstance(urdf, str) or not urdf:
        issues.append("measured URDF path not set")
    else:
        path = Path(urdf)
        path = path if path.is_absolute() else base / path
        try:
            root = ET.parse(path).getroot()
            if root.tag != "robot":
                raise ValueError("root is not robot")
            names = {j.get("name") for j in root.findall("joint")}
            link_names = {link.get("name") for link in root.findall("link")}
            missing = set(arm_config.get("joints", {})) - names
            if missing:
                issues.append("URDF misses arm joints: " + ",".join(sorted(missing)))
        except (OSError, ET.ParseError, ValueError) as error:
            issues.append("URDF unavailable/invalid: " + str(error))
    for side in ("left", "right"):
        payload = (model.get("payloads") or {}).get(side) or {}
        if not number(payload.get("mass_kg")) or payload["mass_kg"] <= 0:
            issues.append(side + ": measured payload mass missing")
        com = payload.get("com_m")
        if not isinstance(com, list) or len(com) != 3 or not all(number(v) for v in com):
            issues.append(side + ": measured payload COM missing")
        if not isinstance(payload.get("mount_frame"), str) or not payload["mount_frame"]:
            issues.append(side + ": payload coordinate frame missing")
        elif link_names is not None and payload["mount_frame"] not in link_names:
            issues.append(side + ": payload frame is not a URDF link")
        if payload.get("verified") is not True:
            issues.append(side + ": payload measurements not verified")
    return issues


def audit(config, model, base=Path(".")):
    control, commissioning = config.get("control", {}), config.get("commissioning", {})
    defaults = config.get("defaults", {})
    enabled = {n: {**defaults, **(v or {})} for n, v in config.get("joints", {}).items()
               if {**defaults, **(v or {})}.get("enabled", True)}
    zero_issues = []
    if control.get("zero") != "config":
        zero_issues.append("zero is relative/start")
    if not enabled or any(not number(s.get("motor_zero")) for s in enabled.values()):
        zero_issues.append("fixed motor_zero missing")
    for field in ("reference_pose_verified", "encoder_reference_persistent_verified"):
        if commissioning.get(field) is not True:
            zero_issues.append(field + " not confirmed")
    return dict(fixed_zero_ready=not zero_issues, zero_issues=zero_issues,
                model_issues=model_issues(model, config, base), gravity_compensation_implemented=False,
                collision_checking_implemented=False, hardware_tested=False,
                note="This checks records, not hardware. A valid URDF name list does not validate geometry or inertias.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    make = actions.add_parser("make-fixed-zero")
    make.add_argument("--arm-config", required=True)
    make.add_argument("--samples", required=True)
    make.add_argument("--output", required=True)
    check = actions.add_parser("audit")
    check.add_argument("--arm-config", required=True)
    check.add_argument("--model", required=True)
    check.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.arm_config).read_text(encoding="utf-8"))
    if args.action == "make-fixed-zero":
        samples = json.loads(Path(args.samples).read_text(encoding="utf-8"))
        result = fixed_zero_config(config, samples)
        output = Path(args.output)
        with output.open("x", encoding="utf-8", newline="\n") as file:
            yaml.safe_dump(result, file, allow_unicode=True, sort_keys=False)
        print("Created measured fixed-zero config; no hardware accessed: " + str(output))
    else:
        model_path = Path(args.model)
        result = audit(config, yaml.safe_load(model_path.read_text(encoding="utf-8")), model_path.resolve().parent)
        print(json.dumps(result, indent=2, allow_nan=False))
        if args.strict and (not result["fixed_zero_ready"] or result["model_issues"]):
            raise SystemExit(1)


if __name__ == "__main__":
    main()

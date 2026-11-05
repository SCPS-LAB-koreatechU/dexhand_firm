"""Generate SDK calibration from the same JSON profiles as the ESP32 sketches."""
import argparse
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from hand_can import load_profile, profile_hash

OUTPUT = ROOT.parent / "humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand.yaml"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    config = yaml.safe_load(OUTPUT.read_text(encoding="utf-8"))
    config["defaults"]["protocol"] = 3
    config["control"].setdefault("command_timeout", 0.5)
    for side in ("left", "right"):
        profile = load_profile(ROOT / f"profiles/{side}_hand.json")
        config["hands"][side].update({
            "node": profile["device_id"], "profile_tag": profile_hash(profile)[:8],
            "open_us": [m["open_us"] for m in profile["motors"]],
            "close_us": [m["close_us"] for m in profile["motors"]],
        })
        if 'hardware_max_us' in profile:config['hands'][side]['hardware_max_us']=profile['hardware_max_us']
        else:config['hands'][side].pop('hardware_max_us',None)
    content = "# Generated calibration from dexhand_control/profiles; regenerate with tools/generate_sdk_config.py.\n"
    content += yaml.safe_dump(config, allow_unicode=True, sort_keys=False)
    if args.check:
        if OUTPUT.read_text(encoding="utf-8") != content:
            raise SystemExit("SDK calibration/config is stale; run generate_sdk_config.py")
        print("PASS: SDK profiles match firmware profile source")
    else:
        OUTPUT.write_text(content, encoding="utf-8", newline="\n")
        print(OUTPUT)


if __name__ == "__main__":
    main()

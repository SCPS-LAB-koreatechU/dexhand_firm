"""Keep the user's central documentation folder and installed SDK docs identical."""
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PAIRS = (
    (ROOT / "humanoid_sdk-main/ros2_ws/src/bhl_upper_body_sdk/README.md",
     ROOT / "dexhand_control/firmware/안내사항/UPPER_BODY_SDK_v020.md"),
    (ROOT / "dexhand_control/firmware/안내사항/PROTOCOL_v3.md",
     ROOT / "humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/firmware/PROTOCOL_v3.md"),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for source, output in PAIRS:
        data = source.read_bytes()
        if not output.exists() or output.read_bytes() != data:
            if args.check:
                raise SystemExit("Stale document: " + str(output))
            output.write_bytes(data)
    print("PASS: central and installed SDK documents match")


if __name__ == "__main__":
    main()

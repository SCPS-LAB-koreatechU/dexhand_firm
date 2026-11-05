"""Download the official MediaPipe hand model once, checking a fixed SHA256.

Source: official Google hand-landmarker Python example. No camera or CAN opened.
"""
import argparse
import hashlib
from pathlib import Path
import urllib.request

MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
MODEL_SHA256 = "fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        if hashlib.sha256(args.output.read_bytes()).hexdigest() != MODEL_SHA256:
            raise SystemExit("Existing model checksum differs; choose a new --output or remove that file explicitly")
        print("Model already verified: " + str(args.output))
        return
    with urllib.request.urlopen(MODEL_URL, timeout=30) as response:
        data = response.read(20_000_001)
    if len(data) > 20_000_000 or hashlib.sha256(data).hexdigest() != MODEL_SHA256:
        raise SystemExit("Model download checksum/size mismatch; no model saved")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as output:
        output.write(data)
    print("Verified model saved: " + str(args.output))


if __name__ == "__main__":
    main()

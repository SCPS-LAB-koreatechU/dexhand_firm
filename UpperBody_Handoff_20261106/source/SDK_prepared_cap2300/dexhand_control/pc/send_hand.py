"""Linux SocketCAN example. Default prints frames without opening CAN."""
import argparse
import json
import time
from pathlib import Path
from hand_can import DEFAULT_PROFILE, HandSender, PythonCanTransport, load_profile, pose_payloads, profile_hash


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--targets", type=Path, required=True, help="JSON array of 15 integer pulse widths (us)")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--send", action="store_true", help="Open CAN and actually send; omitted = preview only")
    args = parser.parse_args()
    profile = load_profile(args.profile)
    pulses = json.loads(args.targets.read_text(encoding="utf-8"))
    frames = pose_payloads(profile, pulses, 0)
    if not 0 < args.seconds <= 3600:
        parser.error("--seconds must be in (0, 3600]")
    print(f"Profile SHA256: {profile_hash(profile)}")
    print(f"Node {profile['device_id']}, ID 0x{(6 << 7) | profile['device_id']:03X}")
    for data in frames:
        print(data.hex(" "))
    if not args.send:
        print("Preview only: no CAN device was opened and no motor command was sent.")
        return

    import can
    from ownership import NodeLease
    lease = NodeLease(args.channel, [profile["device_id"]])
    bus = can.Bus(interface="socketcan", channel=args.channel, bitrate=1_000_000)
    sender = HandSender(PythonCanTransport(bus), profile)
    try:
        sender.start()
        deadline = time.monotonic() + args.seconds
        next_tick = time.monotonic()
        while time.monotonic() < deadline:
            sender.write_us(pulses)
            next_tick += 0.02
            remaining = next_tick - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()  # do not replay a burst of obsolete ticks
    finally:
        try:
            sender.hold()
        finally:
            bus.shutdown()
            lease.close()


if __name__ == "__main__":
    main()


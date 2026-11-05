"""Generate self-contained Arduino sketches for each hand from shared sources."""
from pathlib import Path
import argparse
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_board_config import render
from hand_can import load_profile

TEMPLATE = ROOT / "firmware" / "DexHandCan"
HANDS = {"left": ("DexHandLeft", 60), "right": ("DexHandRight", 61)}
HEADERS = ("HandController.h", "PoseReceiver.h", "ServoOutput.h", "StatusReport.h")


def expected_files(hand: str, *, profile=None) -> dict[Path, str]:
    if hand == 'right' and (ROOT/'firmware/DexHandRight/LatestPoseMailbox.h').exists():
        raise ValueError('Receive-only async right firmware is standalone. Use its update_config.py; do not overwrite it from the old shared template.')
    sketch_name, node = HANDS[hand]
    if profile is None:
        profile = load_profile(ROOT / "profiles" / f"{hand}_hand.json")
    if profile["hand"] != hand or profile["device_id"] != node:
        raise ValueError(f"{hand}: profile must identify hand={hand}, node={node}")
    output = ROOT / "firmware" / sketch_name
    sketch = (TEMPLATE / "DexHandCan.ino").read_text(encoding="utf-8")
    marker = "using namespace hand_config;"
    if sketch.count(marker) != 1:
        raise ValueError("Shared sketch no longer has a unique configuration marker")
    sketch = sketch.replace("Battery-powered left DexHand", f"Battery-powered {hand} DexHand", 1)
    start_marker, end_marker = "// CAN_CONFIG_BEGIN", "// CAN_CONFIG_END"
    if sketch.count(start_marker) != 1 or sketch.count(end_marker) != 1:
        raise ValueError("Shared sketch must contain exactly one CAN settings block")
    start, end = sketch.index(start_marker), sketch.index(end_marker) + len(end_marker)
    side = "왼손" if hand == "left" else "오른손"
    settings = (f"{start_marker}\n"
                f"constexpr int CAN_TX_GPIO = {profile['can']['tx_gpio']};          // ESP32 -> transceiver TXD/CTX\n"
                f"constexpr int CAN_RX_GPIO = {profile['can']['rx_gpio']};          // ESP32 <- transceiver RXD/CRX\n"
                f"constexpr uint8_t CAN_NODE_ID = {node};        // {side} 장치 번호\n"
                f"constexpr uint32_t CAN_CONTROL_ID = 0x{node:03X}; // START / HOLD / RESUME / OFF\n"
                f"constexpr uint32_t CAN_POSE_ID = 0x{(6 << 7) | node:03X};    // 손가락 목표 프레임\n"
                f"constexpr uint32_t CAN_PING_ID = 0x{(4 << 7) | node:03X};    // PING / 마지막 목표 조회\n"
                f"constexpr uint32_t CAN_PONG_ID = 0x{(3 << 7) | node:03X};    // ESP32 → PC 장치 응답\n"
                f"constexpr uint32_t CAN_STATUS_ID = 0x{(5 << 7) | node:03X};  // ESP32 → PC 적용 결과\n"
                f"constexpr uint32_t CAN_STATE_ID = 0x{(8 << 7) | node:03X};   // ESP32 → PC 마지막 PWM 목표\n"
                f"constexpr uint32_t CAN_BOOT_ID = 0x{(9 << 7) | node:03X};    // ESP32 → PC 부팅 식별\n"
                "constexpr uint32_t HEARTBEAT_MS = 500;\n"
                f"{end_marker}")
    sketch = sketch[:start] + settings + sketch[end:]
    sketch = sketch.replace(marker, marker + f'\nstatic_assert(kNodeId == {node}, "Wrong CAN node for {sketch_name}");', 1)
    banner = (f"// Generated {hand} sketch: node {node}. Edit profiles/{hand}_hand.json\n"
              "// or the shared firmware/DexHandCan sources, then run tools/prepare_firmware.py.\n")
    files = {output / f"{sketch_name}.ino": banner + sketch,
             output / "BoardConfig.h": render(profile)}
    files.update({output / name: (TEMPLATE / name).read_text(encoding="utf-8") for name in HEADERS})
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand", choices=("both", "left", "right"), default="both")
    parser.add_argument("--check", action="store_true", help="Verify generated sketches without modifying files")
    args = parser.parse_args()
    selected = HANDS if args.hand == "both" else (args.hand,)
    planned = {}
    for hand in selected:
        planned.update(expected_files(hand))
    differing = [path for path, content in planned.items()
                 if not path.exists() or path.read_text(encoding="utf-8") != content]
    if args.check:
        if differing:
            raise SystemExit("Generated sketch missing/stale: " + ", ".join(str(p.relative_to(ROOT)) for p in differing))
        print(f"PASS: {args.hand} sketch(es) match shared sources and their own profiles")
        return
    for path in differing:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(planned[path], encoding="utf-8", newline="\n")
    for hand in selected:
        name, node = HANDS[hand]
        print(f"{hand}: node {node}, {ROOT / 'firmware' / name / (name + '.ino')}")


if __name__ == "__main__":
    main()

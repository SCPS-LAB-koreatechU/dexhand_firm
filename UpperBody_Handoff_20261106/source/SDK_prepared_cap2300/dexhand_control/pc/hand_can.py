"""PC calibration and CAN sender. No serial connection, ROS dependency, or reply wait."""
from __future__ import annotations

import hashlib
import json
import math
from numbers import Integral, Real
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / "profiles" / "left_hand.json"
OUTPUT_GPIOS = {0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19,
                21, 22, 23, 25, 26, 27, 32, 33}
INPUT_GPIOS = OUTPUT_GPIOS | {34, 35, 36, 39}
FUNCTION_CONTROL = 0
FUNCTION_POSE = 6
MODE_OFF, MODE_START, MODE_HOLD, MODE_RESUME = 0x00, 0x01, 0x02, 0x13


def load_profile(path=DEFAULT_PROFILE) -> dict:
    profile = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_profile(profile)
    return profile


def validate_profile(profile: dict) -> None:
    if profile["profile_version"] != 1 or type(profile["device_id"]) is not int \
            or not 1 <= profile["device_id"] <= 127:
        raise ValueError("Unsupported profile version or node ID")
    motors = profile["motors"]
    if len(motors) != 15 or [m["id"] for m in motors] != list(range(1, 16)):
        raise ValueError("Exactly 15 motors in ID order 1..15 are required")
    pins = [m["gpio"] for m in motors]
    if len(set(pins)) != 15 or any(type(p) is not int or p not in OUTPUT_GPIOS for p in pins):
        raise ValueError("PWM GPIOs must be distinct ESP32 output pins")
    hardware_max=profile.get('hardware_max_us')
    if 'hardware_max_us' in profile and (type(hardware_max) is not int or not 500<=hardware_max<=2500):
        raise ValueError('hardware_max_us must be an integer 500..2500 us')
    for motor in motors:
        for key in ("open_us", "close_us"):
            value = motor[key]
            if type(value) is not int or not 500 <= value <= 2500:
                raise ValueError(f"Motor {motor['id']}: invalid {key}")
            if hardware_max is not None and value>hardware_max:
                raise ValueError(f"Motor {motor['id']}: {key} exceeds hardware_max_us")
        if motor["open_us"] == motor["close_us"]:
            raise ValueError("Open and closed positions must differ")
    tx, rx = profile["can"]["tx_gpio"], profile["can"]["rx_gpio"]
    if type(tx) is not int or type(rx) is not int or tx not in OUTPUT_GPIOS \
            or rx not in INPUT_GPIOS or tx == rx or tx in pins or rx in pins:
        raise ValueError("CAN pins are invalid or conflict with PWM pins")
    if profile["can"]["bitrate"] != 1_000_000 or profile["pwm_frequency_hz"] != 50:
        raise ValueError("This firmware uses 1 Mbps CAN and 50 Hz PWM")
    if type(profile["assembly_timeout_ms"]) is not int or type(profile["watchdog_ms"]) is not int \
            or not 1 <= profile["assembly_timeout_ms"] < profile["watchdog_ms"] <= 60_000:
        raise ValueError("Invalid timeout settings")


def profile_hash(profile: dict) -> str:
    canonical = json.dumps(profile, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def validate_pulses(profile: dict, pulses) -> list[int]:
    values = list(pulses)
    if len(values) != 15:
        raise ValueError("A complete pose contains exactly 15 pulse widths")
    for motor, value in zip(profile["motors"], values):
        low, high = sorted((motor["open_us"], motor["close_us"]))
        high=profile.get('hardware_max_us',high)  # Explicit raw PWM envelope; fractions retain calibrated endpoints.
        if isinstance(value, bool) or not isinstance(value, Integral) or not low <= value <= high:
            raise ValueError(f"Motor {motor['id']} ({motor['name']}): {value} outside {low}..{high} us")
    return [int(value) for value in values]


def fractions_to_us(profile: dict, fractions) -> list[int]:
    values = list(fractions)
    if len(values) != 15:
        raise ValueError("Exactly 15 fractions are required")
    pulses = []
    for motor, value in zip(profile["motors"], values):
        if isinstance(value, bool) or not isinstance(value, Real) \
                or not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError("Fractions must be finite numbers in 0..1")
        pulse = motor["open_us"] + (motor["close_us"] - motor["open_us"]) * value
        pulses.append(int(math.floor(pulse + 0.5)))
    return validate_pulses(profile, pulses)


def pose_payloads(profile: dict, pulses, sequence: int) -> list[bytes]:
    values = validate_pulses(profile, pulses)
    if type(sequence) is not int or not 0 <= sequence <= 255:
        raise ValueError("Sequence must be an integer in 0..255")
    return [struct.pack("<BBHHH", sequence, 0x10 | group, *values[group * 3:group * 3 + 3])
            for group in range(5)]


class PythonCanTransport:
    """Expose the existing SDK's send(node, function, data) interface on python-can."""
    def __init__(self, bus):
        self.bus = bus

    def send(self, device_id: int, function: int, data: bytes = b""):
        import can
        self.bus.send(can.Message(arbitration_id=(function << 7) | device_id,
                                  is_extended_id=False, data=data), timeout=0.02)


class HandSender:
    """Accepts RecoilBus or PythonCanTransport. All methods only transmit CAN."""
    def __init__(self, transport, profile: dict):
        validate_profile(profile)
        self.transport = transport
        self.profile = profile
        self.node_id = profile["device_id"]
        self.sequence = 0

    def _mode(self, mode: int):
        self.transport.send(self.node_id, FUNCTION_CONTROL, bytes((mode, self.node_id)))

    def start(self):
        """Once per PC session: output OFF, clear old sequence, permit new poses."""
        self._mode(MODE_START)
        self.sequence = 0

    def write_us(self, pulses):
        payloads = pose_payloads(self.profile, pulses, self.sequence)
        # A failed partial transmission must not be reused for another target.
        self.sequence = (self.sequence + 1) & 0xFF
        for payload in payloads:
            self.transport.send(self.node_id, FUNCTION_POSE, payload)

    def write_fractions(self, fractions):
        self.write_us(fractions_to_us(self.profile, fractions))

    def hold(self):
        """Keep last PWM, inhibit new poses until start()/resume(). Not damping."""
        self._mode(MODE_HOLD)

    def off(self):
        """Stop PWM pulses; this is not a motor power switch."""
        self._mode(MODE_OFF)

    def resume(self):
        """Keep last output, permit a fresh pose stream with a new sequence origin."""
        self._mode(MODE_RESUME)
        self.sequence = 0


import copy
import importlib.util
import math
from pathlib import Path
import struct
import subprocess
import sys
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from hand_can import (HandSender, PythonCanTransport, fractions_to_us, load_profile,
                      pose_payloads, validate_profile, validate_pulses)


class CaptureTransport:
    def __init__(self):
        self.frames = []
        self.fail_at = None

    def send(self, node, function, data=b""):
        self.frames.append((node, function, data))
        if self.fail_at == len(self.frames):
            raise OSError("simulated CAN transmit failure")


class HandTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile()
        self.open = [motor["open_us"] for motor in self.profile["motors"]]

    def test_final_user_pin_map(self):
        self.assertEqual([m["gpio"] for m in self.profile["motors"]],
                         [32, 33, 25, 26, 21, 22, 23, 19, 16, 17, 4, 27, 13, 14, 18])

    def test_deployed_can_wiring_uses_confirmed_tx0_rx0_for_both_hands(self):
        # Regression for the mismatch with the existing soldered GPIO1/3 test rig.
        for side in ('left', 'right'):
            profile = load_profile(ROOT / f'profiles/{side}_hand.json')
            self.assertEqual(profile['can'], {'tx_gpio': 1, 'rx_gpio': 3, 'bitrate': 1_000_000})
            name = 'DexHandLeft' if side == 'left' else 'DexHandRight'
            sketch = (ROOT / 'firmware' / name / (name+'.ino')).read_text(encoding='utf-8')
            self.assertIn('constexpr int CAN_TX_GPIO = 1;', sketch)
            self.assertIn('constexpr int CAN_RX_GPIO = 3;', sketch)
            self.assertIn('static_cast<gpio_num_t>(CAN_TX_GPIO), static_cast<gpio_num_t>(CAN_RX_GPIO)', sketch)
            self.assertIn('Serial.end();', sketch)
            self.assertNotIn('Serial.begin(', sketch)

    def test_pc_calibration_both_directions(self):
        self.assertEqual(fractions_to_us(self.profile, [0] * 15), self.open)
        self.assertEqual(fractions_to_us(self.profile, [1] * 15),
                         [m["close_us"] for m in self.profile["motors"]])
        self.assertEqual(fractions_to_us(self.profile, [0.5] * 15)[14], 1300)

    def test_fractions_reject_nonfinite_and_outside_values(self):
        for value in (-0.001, 1.001, math.nan, math.inf, True, "0.5"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                fractions_to_us(self.profile, [value] * 15)

    def test_wire_format_exact_little_endian_and_group_order(self):
        frames = pose_payloads(self.profile, self.open, 7)
        self.assertEqual(len(frames), 5)
        self.assertTrue(all(len(frame) == 8 for frame in frames))
        self.assertEqual(frames[0], bytes.fromhex("07 10 ef 06 bc 02 d6 06"))
        self.assertEqual(frames[4], bytes.fromhex("07 14 e8 03 34 08 bc 02"))
        decoded = [value for frame in frames for value in struct.unpack("<BBHHH", frame)[2:]]
        self.assertEqual(decoded, self.open)

    def test_per_motor_limits_not_only_global_envelope(self):
        for index, motor in enumerate(self.profile["motors"]):
            low, high = sorted((motor["open_us"], motor["close_us"]))
            for bad in (low - 1, high + 1, 1500.0, True):
                values = self.open.copy()
                values[index] = bad
                with self.subTest(index=index, bad=bad), self.assertRaises(ValueError):
                    validate_pulses(self.profile, values)

    def test_short_pose_and_invalid_sequence(self):
        for count in (0, 14, 16):
            with self.assertRaises(ValueError):
                pose_payloads(self.profile, [1500] * count, 0)
        for sequence in (-1, 256, True):
            with self.assertRaises(ValueError):
                pose_payloads(self.profile, self.open, sequence)

    def test_bad_profile_rejects_duplicate_gpio_and_can_conflict(self):
        changed = copy.deepcopy(self.profile)
        changed["motors"][14]["gpio"] = 19
        with self.assertRaises(ValueError):
            validate_profile(changed)
        for pin in (34, 35):
            changed = copy.deepcopy(self.profile)
            changed["motors"][6]["gpio"] = pin
            with self.subTest(input_only_gpio=pin), self.assertRaisesRegex(ValueError, "output pins"):
                validate_profile(changed)
        changed = copy.deepcopy(self.profile)
        changed["can"]["tx_gpio"] = 18
        with self.assertRaises(ValueError):
            validate_profile(changed)

    def test_generated_firmware_matches_single_profile(self):
        spec = importlib.util.spec_from_file_location("generator", ROOT / "tools/generate_board_config.py")
        generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(generator)
        self.assertEqual(generator.OUTPUT.read_text(encoding="utf-8"), generator.render(self.profile))

    def test_right_hand_user_calibration_and_anatomical_pin_map(self):
        right = load_profile(ROOT / "profiles/right_hand.json")
        self.assertEqual(right["device_id"], 61)
        self.assertEqual([m["gpio"] for m in right["motors"]],
                         [19, 21, 23, 26, 33, 22, 32, 25, 16, 17, 4, 27, 13, 14, 18])
        self.assertEqual([m["calibration_motor_id"] for m in right["motors"][:8]], [7, 8, 5, 6, 3, 4, 1, 2])
        self.assertEqual(fractions_to_us(right, [0] * 15),
                         [850, 1700, 850, 1700, 1750, 850, 1750, 700, 700, 700, 2000, 2000, 2100, 700, 2300])
        self.assertEqual(fractions_to_us(right, [1] * 15),
                         [1775, 800, 1725, 800, 825, 1750, 700, 1725, 2100, 2000, 750, 800, 700, 1900, 700])

    def test_right_raw_hardware_ceiling_does_not_stretch_fraction_targets(self):
        right=load_profile(ROOT/'profiles/right_hand.json')
        self.assertEqual(right['hardware_max_us'],2300)
        opened=fractions_to_us(right,[0]*15);closed=fractions_to_us(right,[1]*15)
        self.assertEqual(closed[5],1750)
        raw=opened[:];raw[5]=2300
        self.assertEqual(validate_pulses(right,raw),raw)
        raw[5]=2301
        with self.assertRaises(ValueError):validate_pulses(right,raw)
        legacy=copy.deepcopy(right);legacy.pop('hardware_max_us')
        raw[5]=1751
        with self.assertRaises(ValueError):validate_pulses(legacy,raw)
        invalid=copy.deepcopy(right);invalid['hardware_max_us']=2299
        with self.assertRaises(ValueError):validate_profile(invalid)

    def test_right_header_cannot_overwrite_left_template_by_default(self):
        header = ROOT / "firmware/DexHandCan/BoardConfig.h"
        before = header.read_bytes()
        result = subprocess.run([sys.executable, "-X", "utf8", "-B",
                                 str(ROOT / "tools/generate_board_config.py"),
                                 "--profile", str(ROOT / "profiles/right_hand.json")],
                                capture_output=True, text=True, encoding="utf-8")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("default header belongs to the left", result.stderr)
        self.assertEqual(header.read_bytes(), before)

    def test_left_and_right_sessions_coexist_on_one_virtual_bus(self):
        import can
        channel = "dexhand-left-right-unit-test"
        buses = [can.Bus(interface="virtual", channel=channel) for _ in range(3)]
        profiles = [self.profile, load_profile(ROOT / "profiles/right_hand.json")]
        try:
            for bus, profile in zip(buses, profiles):
                sender = HandSender(PythonCanTransport(bus), profile)
                sender.start()
                sender.write_fractions([0] * 15)
            received = [buses[2].recv(timeout=0.1) for _ in range(12)]
            self.assertTrue(all(message is not None for message in received))
            for profile in profiles:
                node = profile["device_id"]
                control = [m for m in received if m.arbitration_id == node]
                self.assertEqual([bytes(m.data) for m in control], [bytes([1, node])])
                poses = [m for m in received if m.arbitration_id == (6 << 7) | node]
                self.assertEqual(len(poses), 5)
                pulses = [value for m in poses for value in struct.unpack("<BBHHH", bytes(m.data))[2:]]
                self.assertEqual(pulses, [m["open_us"] for m in profile["motors"]])
        finally:
            for bus in buses:
                bus.shutdown()

    def test_sdk_transport_start_pose_and_control(self):
        transport = CaptureTransport()
        sender = HandSender(transport, self.profile)
        sender.start()
        sender.write_us(self.open)
        sender.hold()
        sender.resume()
        sender.off()
        self.assertEqual(transport.frames[0], (60, 0, b"\x01\x3c"))
        self.assertTrue(all(node == 60 and function == 6 for node, function, data in transport.frames[1:6]))
        self.assertEqual([frame[2] for frame in transport.frames[-3:]],
                         [b"\x02\x3c", b"\x13\x3c", b"\x00\x3c"])

    def test_sequence_wrap_and_failed_transmission_advance(self):
        transport = CaptureTransport()
        sender = HandSender(transport, self.profile)
        sender.sequence = 255
        sender.write_us(self.open)
        sender.write_us(self.open)
        self.assertEqual(transport.frames[0][2][0], 255)
        self.assertEqual(transport.frames[5][2][0], 0)
        transport.fail_at = len(transport.frames) + 2
        with self.assertRaises(OSError):
            sender.write_us(self.open)
        self.assertEqual(sender.sequence, 2)

    def test_invalid_target_sends_nothing(self):
        transport = CaptureTransport()
        with self.assertRaises(ValueError):
            HandSender(transport, self.profile).write_us([0] * 15)
        self.assertEqual(transport.frames, [])

    def test_python_can_virtual_transport(self):
        import can
        channel = "dexhand-unit-test"
        tx = can.Bus(interface="virtual", channel=channel)
        rx = can.Bus(interface="virtual", channel=channel)
        try:
            HandSender(PythonCanTransport(tx), self.profile).write_us(self.open)
            messages = [rx.recv(timeout=0.1) for _ in range(5)]
            self.assertTrue(all(m is not None and m.arbitration_id == 0x33C
                                and not m.is_extended_id and m.dlc == 8 for m in messages))
            self.assertEqual(bytes(messages[4].data), pose_payloads(self.profile, self.open, 0)[4])
        finally:
            tx.shutdown()
            rx.shutdown()

    def test_existing_humanoid_sdk_recoil_bus_integration(self):
        sdk = ROOT.parent / "humanoid_sdk-main/ros2_ws/src/bhl_arm_sdk"
        if not sdk.exists():
            self.skipTest("Adjacent humanoid SDK is not present")
        sys.path.insert(0, str(sdk))
        from bhl_arm_sdk.recoil_can import RecoilBus

        class RawCapture:
            def __init__(self):
                self.messages = []

            def send(self, message, timeout=None):
                self.messages.append(message)

        recoil = RecoilBus.__new__(RecoilBus)  # actual SDK class; never opens a real bus
        recoil._transaction = threading.RLock()
        recoil._bus = RawCapture()
        sender = HandSender(recoil, self.profile)
        sender.start()
        sender.write_us(self.open)
        self.assertEqual(recoil._bus.messages[0].arbitration_id, 0x03C)
        self.assertEqual(bytes(recoil._bus.messages[0].data), b"\x01\x3c")
        self.assertTrue(all(m.arbitration_id == 0x33C and not m.is_extended_id
                            and len(m.data) == 8 for m in recoil._bus.messages[1:]))
        self.assertEqual(len(recoil._bus.messages), 6)


if __name__ == "__main__":
    unittest.main()


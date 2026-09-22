import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


MODULE_PATH = Path(__file__).with_name("ver9_d8_sender.py")
SPEC = importlib.util.spec_from_file_location("ver9_d8_sender", MODULE_PATH)
sender = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = sender
SPEC.loader.exec_module(sender)


class SenderCleanupTests(unittest.TestCase):
    def test_ramp_targets_start_midpoint_and_finish(self):
        start = (0.0,) * len(sender.H_CAN_IDS)
        target = tuple(float(index) for index in range(len(sender.H_CAN_IDS)))
        self.assertEqual(sender.ramp_targets(start, target, 0.0, 2.0), start)
        self.assertEqual(sender.ramp_targets(start, target, 1.0, 2.0),
                         tuple(value / 2 for value in target))
        self.assertEqual(sender.ramp_targets(start, target, 3.0, 2.0), target)

    def test_ramp_rejects_nonpositive_duration(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            sender.ramp_targets((0.0,) * 10, (0.0,) * 10, 0.0, 0.0)

    def test_slew_target_prevents_opposite_sign_policy_step(self):
        # At 0.25 deg/s, a 20 ms tick cannot jump from -1.5 to +1.7 deg.
        prior = np.deg2rad(-1.5)
        desired = np.deg2rad(+1.7)
        got = sender.slew_target(prior, desired, np.deg2rad(0.25), 0.02)
        self.assertAlmostEqual(got, prior + np.deg2rad(0.005), places=10)

    def test_slew_target_rejects_negative_inputs(self):
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            sender.slew_target(0.0, 1.0, -1.0, 0.1)

    def test_tracking_summary_identifies_stall_from_current_without_motion(self):
        rows = [
            (0.0, "ramp", "0x1C", 0.0, -0.03, -0.03, 0.0, 0.0, -0.24),
            (0.1, "policy", "0x1C", 0.0, -0.04, -0.04, 0.0, 0.0, -0.33),
        ]
        movement, current, verdict = sender.tracking_summary(rows, 0.0)
        self.assertEqual(movement, 0.0)
        self.assertAlmostEqual(current, 0.33)
        self.assertIn("stalled under load", verdict)

    def test_tracking_summary_identifies_missing_torque_response(self):
        rows = [(0.0, "ramp", "0x1C", 0.0, -0.03, -0.03, 0.0, 0.0, 0.03)]
        _movement, current, verdict = sender.tracking_summary(rows, 0.0)
        self.assertAlmostEqual(current, 0.03)
        self.assertIn("no meaningful current", verdict)

    def test_frames_can_limit_transmission_to_one_registered_motor(self):
        frames = sender.frames((0.0,) * len(sender.H_CAN_IDS), (0x1C,))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].arbitration_id & 0xFF, 0x1C)

    def test_wire_command_reports_the_quantized_position_that_is_sent(self):
        requested = np.deg2rad(-1.5)
        kp, kd, wire_target, velocity, torque = sender.wire_command(0x1C, requested)
        self.assertGreater(kp, 0.0)
        self.assertGreater(kd, 0.0)
        self.assertLess(wire_target, 0.0)
        # Zero velocity/torque use the nearest 12-bit bin, so each may be a
        # half-LSB either side of zero on the wire.
        self.assertLess(abs(velocity), 0.01)
        self.assertLess(abs(torque), 0.02)

    def test_dual_bus_counts_frames_across_channels(self):
        bus = sender.DualBus.__new__(sender.DualBus)
        bus.bus_by_channel = {
            0: SimpleNamespace(tx_count=3),
            1: SimpleNamespace(tx_count=5),
        }
        self.assertEqual(bus.tx_count, 8)

    def test_main_arm_explicitly_requests_transmission(self):
        argv = [
            "ver9_d8_sender.py", "--arm", "--motor-id", "0x1C",
            "--ramp-seconds", "6", "--package", "package", "--duration", "2",
            "--csv", "out.csv",
        ]
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run:
            sender.main()
        self.assertTrue(run.call_args.kwargs["transmit"])
        self.assertEqual(run.call_args.kwargs["motor_ids"], (0x1C,))
        self.assertEqual(run.call_args.kwargs["ramp_seconds"], 6.0)

    def test_main_preflight_explicitly_forbids_transmission(self):
        argv = ["ver9_d8_sender.py", "--preflight", "--package", "package"]
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run:
            sender.main()
        self.assertFalse(run.call_args.kwargs["transmit"])

    def test_default_buses_open_both_physical_channels(self):
        bus = sender.DualBus()
        self.assertEqual(tuple(bus.bus_by_channel), (0, 1))

    def test_failed_open_retries_without_transmission(self):
        events = []

        class FakeMotorBus:
            def __init__(self, channel):
                self.channel = channel
                self.open_calls = 0

            def open(self):
                self.open_calls += 1
                events.append(("open", self.channel, self.open_calls))
                if self.channel == 0 and self.open_calls == 1:
                    raise OSError("reset failed")

            def close(self, stop_motors=False):
                events.append(("close", self.channel, stop_motors))

        with patch.object(sender.cm, "MotorBus", FakeMotorBus), patch.object(sender.time, "sleep"):
            bus = sender.DualBus()
            bus.open()
        self.assertEqual(bus.bus_by_channel[0].open_calls, 2)
        self.assertEqual(bus.bus_by_channel[1].open_calls, 1)
        self.assertFalse(any(event[0] == "send" for event in events))

    def test_route_discovery_accepts_swapped_gs_usb_enumeration(self):
        class FreshState:
            def age(self):
                return 0.0

        class FakeMotorBus:
            def __init__(self, channel):
                self.channel = channel

            def state(self, motor_id):
                # Deliberately reverse the earlier process's ch assignment.
                ids = (0x13, 0x1B, 0x2A, 0x12, 0x22) if self.channel == 0 else (0x1C, 0x11, 0x21, 0x1A, 0x2B)
                return FreshState() if motor_id in ids else None

        with patch.object(sender.cm, "MotorBus", FakeMotorBus):
            bus = sender.DualBus()
            bus.discover_routes(timeout_s=0.0)
        self.assertIs(bus.route_by_motor_id[0x1C], bus.bus_by_channel[1])
        self.assertIs(bus.route_by_motor_id[0x2A], bus.bus_by_channel[0])

    def test_feedback_rejects_unzeroed_joint_before_mit_send(self):
        class State:
            t = __import__("time").time()
            err = 0
            pos = 170.0
            spd = cur = 0.0

        bus = sender.DualBus.__new__(sender.DualBus)
        route_bus = type("Bus", (), {"state": lambda _self, _mid: State()})()
        bus.bus_by_channel = {0: route_bus, 1: object()}
        bus.route_by_motor_id = {motor_id: route_bus for motor_id in sender.H_CAN_IDS}
        with self.assertRaisesRegex(RuntimeError, "origin/pre-arm pose abort"):
            bus.feedback()

    def test_feedback_abort_reports_measured_current_and_route(self):
        class State:
            t = __import__("time").time()
            err = 0
            pos = spd = 0.0
            cur = 1.25

        buses = {channel: type("Bus", (), {"state": lambda _self, _mid: State()})()
                 for channel in (0, 1)}
        bus = sender.DualBus.__new__(sender.DualBus)
        bus.bus_by_channel = buses
        bus.route_by_motor_id = {motor_id: buses[0] for motor_id in sender.H_CAN_IDS}
        with self.assertRaisesRegex(RuntimeError, r"0x1C ch=0: cur=\+1.25A"):
            bus.feedback()

    def test_stale_feedback_still_zeros_closes_can_and_t265(self):
        instances = {}

        class FakeBus:
            def __init__(self):
                instances["bus"] = self
                self.opened = self.zeroed = self.closed = False

            def open(self):
                self.opened = True

            def feedback(self):
                raise RuntimeError("stale feedback 0x2A")

            def discover_routes(self):
                return None

            def zero(self):
                self.zeroed = True

            def close(self):
                self.closed = True

        class FakeT265:
            def __init__(self, _offset):
                instances["t265"] = self
                self.closed = False

            def start(self):
                return None

            def latest(self):
                return object()

            def close(self):
                self.closed = True

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "out.csv"
            with patch.object(sender, "HPolicy"), patch.object(sender, "DualBus", FakeBus), patch.object(sender, "RealT265", FakeT265):
                with self.assertRaisesRegex(RuntimeError, "stale feedback 0x2A"):
                    sender.run(Path(directory), 0.1, output, 0.0, 0.0, 0.0)
            self.assertTrue(output.exists())
            self.assertEqual(len(output.read_text(encoding="utf-8").splitlines()), 1)
        self.assertTrue(instances["bus"].opened)
        self.assertTrue(instances["bus"].zeroed)
        self.assertTrue(instances["bus"].closed)
        self.assertTrue(instances["t265"].closed)

    def test_receive_only_preflight_never_uses_mit_cleanup(self):
        bus = sender.DualBus.__new__(sender.DualBus)
        bus.route_by_motor_id = {0x1C: object()}
        bus.mit_frames_sent = False
        with patch.object(sender, "zero_frames") as zero_frames:
            bus.zero()
        zero_frames.assert_not_called()

    def test_preflight_uses_motor_feedback_position_without_transmitting(self):
        instances = {}

        class FakeBus:
            def __init__(self):
                instances["bus"] = self
                self.zero_called = self.closed = False

            def open(self):
                return None

            def discover_routes(self):
                return None

            def feedback(self):
                return {
                    motor_id: sender.MotorFeedback(motor_id, 0.0, 0.1, 0.0)
                    for motor_id in sender.H_CAN_IDS
                }

            def zero(self):
                self.zero_called = True

            def close(self):
                self.closed = True

        class FakeT265:
            def __init__(self, _offset):
                instances["t265"] = self
                self.closed = False

            def start(self):
                return None

            def latest(self):
                return object()

            def close(self):
                self.closed = True

        output = SimpleNamespace(joint_target_h_order=(0.2,) * len(sender.H_CAN_IDS))
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(sender, "HPolicy"), patch.object(sender, "DualBus", FakeBus), \
                 patch.object(sender, "RealT265", FakeT265), patch.object(
                     sender, "evaluate_cycle", return_value=(None, None, output, None)
                 ):
                sender.run(Path(directory), 0.0, Path(directory) / "out.csv", 0.0, 0.0, 0.0, transmit=False)
        self.assertTrue(instances["bus"].closed)
        self.assertTrue(instances["bus"].zero_called)
        self.assertTrue(instances["t265"].closed)


if __name__ == "__main__":
    unittest.main()

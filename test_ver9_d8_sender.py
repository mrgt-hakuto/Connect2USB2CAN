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


class TorquePathTests(unittest.TestCase):
    """The Kt-free reading of a stationary axis: |I| against commanded error."""

    @staticmethod
    def _stationary_rows(kp_cmd, slope_ratio, count=60):
        # The axis never moves, so the commanded error is exactly the wire
        # target minus the fixed feedback position.
        rows = []
        for index in range(count):
            error = np.deg2rad(0.5 + 0.1 * index)
            current = -slope_ratio * kp_cmd * error
            rows.append((0.02 * index, "ramp", "0x1C", -0.09, -error, -error,
                         0.0, 0.0, current))
        return rows

    def test_slope_matching_kp_verifies_the_mit_torque_path(self):
        rows = self._stationary_rows(7.937, 1.0)
        slope, ratio, verdict = sender.torque_path_summary(rows, 7.937)
        self.assertAlmostEqual(ratio, 1.0, places=6)
        self.assertAlmostEqual(slope, 7.937, places=4)
        self.assertIn("verified", verdict)

    def test_absent_current_response_is_not_called_a_mechanical_stall(self):
        rows = self._stationary_rows(7.937, 0.02)
        _slope, _ratio, verdict = sender.torque_path_summary(rows, 7.937)
        self.assertIn("absent", verdict)

    def test_slope_far_from_kp_is_reported_as_anomalous(self):
        rows = self._stationary_rows(7.937, 0.5)
        _slope, _ratio, verdict = sender.torque_path_summary(rows, 7.937)
        self.assertIn("anomalous", verdict)

    def test_too_few_loaded_samples_stay_undetermined(self):
        rows = self._stationary_rows(7.937, 1.0, count=5)
        slope, ratio, verdict = sender.torque_path_summary(rows, 7.937)
        self.assertIsNone(slope)
        self.assertIsNone(ratio)
        self.assertIn("undetermined", verdict)

    def test_deadband_lower_bound_uses_current_and_kp_only(self):
        # 0.66 A that did not move the axis, at Kp = 7.937, is 4.76 deg.
        bound = sender.deadband_lower_bound_rad(0.66, 7.937)
        self.assertAlmostEqual(np.rad2deg(bound), 4.764, places=2)


class ProbeAngleLimitTests(unittest.TestCase):
    """The probe angle is bounded by the current abort, per axis."""

    def test_ak10_hr_limit_leaves_room_for_a_feedback_noise_peak(self):
        # Kp 7.937 commands 0.92 A at 6.64 deg; a +0.08 A noise peak on top of
        # that still sits at the 1.0 A abort rather than through it.
        limit = sender.max_probe_angle_deg(7.937)
        self.assertAlmostEqual(limit, 6.641, places=2)
        self.assertLessEqual(limit, sender.STATIC_PROBE_MAX_DEG)
        commanded = np.deg2rad(limit) * 7.937
        self.assertLessEqual(
            commanded + sender.FEEDBACK_CURRENT_NOISE_A, sender.CURRENT_ABORT_A + 1e-9
        )

    def test_the_angle_limit_still_admits_the_approved_6_5_deg(self):
        # The noise allowance tightens the angle limit.  It must not be so
        # tight that the already-approved D9-3 angle becomes unrepresentable;
        # refusing a repeat is the stall guard's job, not the parser's.
        self.assertGreaterEqual(sender.max_probe_angle_deg(7.937), 6.5)

    def test_a_stiffer_axis_gets_a_smaller_limit(self):
        # AK80-9 at stiffness 15 commands Kp 28.7; 7 deg there would be 3.5 A.
        limit = sender.max_probe_angle_deg(28.7)
        self.assertLess(limit, 2.0)
        self.assertAlmostEqual(
            np.deg2rad(limit) * 28.7,
            sender.CURRENT_ABORT_A - sender.FEEDBACK_CURRENT_NOISE_A,
            places=6,
        )

    def test_d9_3_cannot_be_repeated_now_that_0x1c_held_0_86a(self):
        # D9-3 itself raised the demonstrated stall current, so the same run
        # can no longer produce new information and must be refused.
        self.assertAlmostEqual(sender.DEMONSTRATED_STALL_CURRENT_A[0x1C], 0.86)
        with self.assertRaisesRegex(RuntimeError, "repeat-probe abort"):
            sender.stall_guard(0x1C, 7.937, np.deg2rad(-6.5))


class StallGuardTests(unittest.TestCase):
    """A run that cannot move a healthy axis must not reach the CAN bus."""

    def test_probe_ceiling_is_kp_times_the_requested_error(self):
        self.assertAlmostEqual(
            sender.probe_ceiling_current_a(7.937, np.deg2rad(4.524)), 0.6266, places=3
        )

    def test_probe_ceiling_is_capped_by_the_current_abort_limit(self):
        self.assertEqual(
            sender.probe_ceiling_current_a(7.937, np.deg2rad(90.0)),
            sender.CURRENT_ABORT_A,
        )

    def test_repeat_of_a_known_stall_is_refused_before_any_send(self):
        with self.assertRaisesRegex(RuntimeError, "repeat-probe abort"):
            sender.stall_guard(0x1C, 7.937, np.deg2rad(-4.524))

    def test_guard_passes_an_axis_with_no_recorded_stall(self):
        ceiling, detail = sender.stall_guard(0x13, 7.937, np.deg2rad(-4.524))
        self.assertGreater(ceiling, 0.0)
        self.assertIn("probe ceiling", detail)


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

class ProbeScoringTests(unittest.TestCase):
    """The static probe is scored, not aborted: standing still is a result."""

    @staticmethod
    def _rows(positions_deg, currents, stage="probe-hold"):
        return [
            (0.02 * index, stage, "0x1C", np.deg2rad(-6.5), np.deg2rad(-6.5),
             np.deg2rad(-6.9), np.deg2rad(position), 0.0, -current)
            for index, (position, current) in enumerate(zip(positions_deg, currents))
        ]

    def test_quantization_dither_is_not_breakaway(self):
        # The measured D9-3 shape: 0.1 deg feedback increments flickering up
        # and down, net 0.2 deg after 10 s.  Two increments are wind-up and
        # backlash take-up, not rotation, so this is C and not B.
        positions = [-0.4, -0.5, -0.4, -0.5, -0.6, -0.5, -0.6, -0.6]
        result = sender.probe_result(
            self._rows(positions, [0.87] * len(positions)),
            np.deg2rad(-0.4), np.deg2rad(-6.5),
        )
        self.assertLess(np.rad2deg(result.net_rad), sender.BREAKAWAY_MIN_DEG)
        self.assertEqual(result.letter, "C")
        self.assertIsNone(result.breakaway_current_a)

    def test_real_breakaway_short_of_target_is_b_and_reports_its_current(self):
        positions = [-0.4, -0.9, -1.4, -1.9, -1.9, -1.9]
        currents = [0.30, 0.55, 0.60, 0.62, 0.62, 0.62]
        result = sender.probe_result(
            self._rows(positions, currents), np.deg2rad(-0.4), np.deg2rad(-6.5)
        )
        self.assertEqual(result.letter, "B")
        # The first sample past the 0.5 deg floor was reading 0.55 A.
        self.assertAlmostEqual(result.breakaway_current_a, 0.55, places=2)

    def test_covering_half_the_request_is_a(self):
        positions = [-0.4, -2.0, -3.9, -3.9, -3.9]
        result = sender.probe_result(
            self._rows(positions, [0.4] * len(positions)),
            np.deg2rad(-0.4), np.deg2rad(-6.5),
        )
        self.assertEqual(result.letter, "A")
        self.assertAlmostEqual(np.rad2deg(result.required_rad), 3.25, places=2)

    def test_no_current_and_no_motion_is_d_not_a_mechanical_verdict(self):
        positions = [-0.4] * 6
        result = sender.probe_result(
            self._rows(positions, [0.02] * 6), np.deg2rad(-0.4), np.deg2rad(-6.5)
        )
        self.assertEqual(result.letter, "D")

    def test_sustained_current_ignores_a_single_noise_peak(self):
        positions = [-0.4] * 7
        currents = [0.85, 0.87, 0.93, 0.86, 0.88, 0.84, 0.87]
        result = sender.probe_result(
            self._rows(positions, currents), np.deg2rad(-0.4), np.deg2rad(-6.5)
        )
        self.assertAlmostEqual(result.peak_current_a, 0.93, places=2)
        self.assertAlmostEqual(result.sustained_current_a, 0.87, places=2)
        # The deadband bound follows the sustained current, so the peak cannot
        # inflate it: 0.87/7.937 is 6.28 deg, not 0.93/7.937 = 6.71 deg.
        bound = sender.deadband_lower_bound_rad(result.sustained_current_a, 7.937)
        self.assertAlmostEqual(np.rad2deg(bound), 6.28, places=1)

    def test_the_ramp_stage_alone_still_scores(self):
        positions = [-0.4, -0.4, -0.4, -0.4]
        result = sender.probe_result(
            self._rows(positions, [0.5] * 4, stage="probe-ramp"),
            np.deg2rad(-0.4), np.deg2rad(-6.5),
        )
        self.assertEqual(result.letter, "C")

    def test_an_unverified_torque_path_overrides_the_letter(self):
        result = sender.ProbeResult(0.0, 0.0, 0.5, 0.5, 0.05, None, "C", "stalled")
        self.assertEqual(sender.d9_3_letter(result, "MIT torque absent (...)"), "D")
        self.assertEqual(
            sender.d9_3_letter(result, "MIT torque path verified (...)"), "C"
        )

    def test_every_letter_has_a_next_step_and_an_exit_code(self):
        for letter in "ABCD":
            self.assertIn(letter, sender.D9_3_NEXT_STEP)
            self.assertIn(letter, sender.D9_3_EXIT_CODE)
        self.assertEqual(sender.D9_3_EXIT_CODE["A"], 0)
        self.assertEqual(sender.D9_3_EXIT_CODE["B"], 0)
        self.assertNotEqual(sender.D9_3_EXIT_CODE["C"], 0)
        self.assertNotEqual(sender.D9_3_EXIT_CODE["D"], 0)


class SenderCleanupTests2(unittest.TestCase):

    def test_policy_ramp_requires_meaningful_fraction_of_initial_target(self):
        rows = [(0.0, "ramp", "0x1C", -0.11, -0.11, -0.11,
                 np.deg2rad(-0.3), 0.0, -0.81)]
        movement, current, required, verdict = sender.policy_ramp_summary(
            rows, np.deg2rad(-0.3), np.deg2rad(-6.4)
        )
        self.assertAlmostEqual(np.rad2deg(movement), 0.0)
        self.assertAlmostEqual(current, 0.81)
        self.assertAlmostEqual(np.rad2deg(required), 3.05)
        self.assertIn("stalled during initial policy ramp", verdict)

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

    def test_main_static_probe_requires_arm_and_uses_no_policy_package(self):
        argv = [
            # 0x13 has no recorded stall, so the repeat guard stays out of
            # the way of what this test is about.
            "ver9_d8_sender.py", "--arm", "--static-probe", "--motor-id", "0x13",
            "--probe-target-deg", "-2.5", "--ramp-seconds", "6", "--duration", "2",
            "--csv", "out.csv",
        ]
        with patch.object(sys, "argv", argv), patch.object(sender, "run_static_probe") as probe:
            probe.return_value = "C"
            exit_code = sender.main()
        self.assertEqual(probe.call_args.args[1:], (0x13, -2.5, 6.0, 2.0))
        # A scored, non-moving probe leaves a legible exit code, not a traceback.
        self.assertEqual(exit_code, sender.D9_3_EXIT_CODE["C"])

    def test_main_refuses_a_repeat_probe_before_opening_the_bus(self):
        argv = [
            "ver9_d8_sender.py", "--arm", "--static-probe", "--motor-id", "0x1C",
            "--probe-target-deg", "-6.5", "--ramp-seconds", "8", "--duration", "2",
            "--csv", "out.csv",
        ]
        with patch.object(sys, "argv", argv), \
                patch.object(sender, "run_static_probe") as probe, \
                patch.object(sender, "DualBus") as bus:
            with self.assertRaises(SystemExit):
                sender.main()
        probe.assert_not_called()
        bus.assert_not_called()

    def test_main_static_probe_exits_zero_when_the_axis_moves(self):
        argv = [
            # 0x13 has no recorded stall, so the repeat guard stays out of
            # the way of what this test is about.
            "ver9_d8_sender.py", "--arm", "--static-probe", "--motor-id", "0x13",
            "--probe-target-deg", "-2.5", "--ramp-seconds", "6", "--duration", "2",
            "--csv", "out.csv",
        ]
        with patch.object(sys, "argv", argv), patch.object(sender, "run_static_probe") as probe:
            probe.return_value = "B"
            self.assertEqual(sender.main(), 0)

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

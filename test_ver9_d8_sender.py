import importlib.util
import io
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

    def test_no_axis_is_currently_recorded_as_stalled(self):
        # D9-6 broke 0x1C away at 0.90 A, so the record that refused a repeat
        # is gone.  The guard itself is still tested below with a fake entry.
        self.assertEqual(sender.DEMONSTRATED_STALL_CURRENT_A, {})
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
        with patch.dict(sender.DEMONSTRATED_STALL_CURRENT_A, {(0x1C, -1): 0.86}):
            with self.assertRaisesRegex(RuntimeError, "repeat-probe abort"):
                sender.stall_guard(0x1C, 7.937, np.deg2rad(-4.524))

    def test_guard_passes_an_axis_with_no_recorded_stall(self):
        ceiling, detail = sender.stall_guard(0x13, 7.937, np.deg2rad(-4.524))
        self.assertGreater(ceiling, 0.0)
        self.assertIn("probe ceiling", detail)


class DirectionalStallRecordTests(unittest.TestCase):
    """D9-4: a stall is evidence about one axis in one direction only.

    D9-3 drove 0x1C only to -6.50 deg, which for LL_HR is toe-inward, toward
    the other foot -- and the two feet were found touching.  The untested
    positive side is new information, so the guard must not refuse it.
    """

    def test_recorded_stall_is_keyed_by_motor_and_direction(self):
        with patch.dict(sender.DEMONSTRATED_STALL_CURRENT_A, {(0x1C, -1): 0.86}):
            self.assertEqual(
                set(sender.DEMONSTRATED_STALL_CURRENT_A), {(0x1C, -1)}
            )

    def test_direction_sign_of_a_probe(self):
        self.assertEqual(sender.probe_direction(np.deg2rad(-6.5)), -1)
        self.assertEqual(sender.probe_direction(np.deg2rad(6.5)), 1)

    def test_lookup_only_matches_the_probed_direction(self):
        with patch.dict(sender.DEMONSTRATED_STALL_CURRENT_A, {(0x1C, -1): 0.86}):
            self.assertAlmostEqual(
                sender.demonstrated_stall_current_a(0x1C, np.deg2rad(-6.5)), 0.86
            )
            self.assertIsNone(
                sender.demonstrated_stall_current_a(0x1C, np.deg2rad(6.5))
            )
            self.assertIsNone(
                sender.demonstrated_stall_current_a(0x13, np.deg2rad(-6.5))
            )

    def test_opposite_direction_probe_is_allowed(self):
        with patch.dict(sender.DEMONSTRATED_STALL_CURRENT_A, {(0x1C, -1): 0.86}):
            ceiling, detail = sender.stall_guard(0x1C, 7.937, np.deg2rad(6.5))
        self.assertGreater(ceiling, 0.86)
        self.assertIn("probe ceiling", detail)

    def test_same_direction_probe_is_still_refused_and_names_the_side(self):
        with patch.dict(sender.DEMONSTRATED_STALL_CURRENT_A, {(0x1C, -1): 0.86}):
            with self.assertRaisesRegex(RuntimeError, "negative direction"):
                sender.stall_guard(0x1C, 7.937, np.deg2rad(-6.5))

    def test_refusal_points_at_the_unprobed_direction(self):
        with patch.dict(sender.DEMONSTRATED_STALL_CURRENT_A, {(0x1C, -1): 0.86}):
            with self.assertRaisesRegex(
                RuntimeError, "positive direction has not been probed"
            ):
                sender.stall_guard(0x1C, 7.937, np.deg2rad(-6.5))

    def test_probe_angle_limit_is_unchanged_by_this_patch(self):
        # The current abort and the noise margin still bound the angle; the
        # directional record must not become a way to ask for a bigger one.
        self.assertAlmostEqual(
            sender.max_probe_angle_deg(7.937), 6.643, places=2
        )


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
        # The current it was HOLDING just before it let go, not the current at
        # the first moved sample: by then the error, and so the current, is
        # already collapsing as the axis runs toward the target.
        self.assertAlmostEqual(result.breakaway_current_a, 0.30, places=2)

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
        with patch.dict(sender.DEMONSTRATED_STALL_CURRENT_A, {(0x1C, -1): 0.86}), \
                patch.object(sys, "argv", argv), \
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
        bus.current_limit_a = sender.default_current_limits()
        bus.route_by_motor_id = {motor_id: buses[0] for motor_id in sender.H_CAN_IDS}
        with self.assertRaisesRegex(RuntimeError, r"0x1C ch=0: cur=\+1.25A"):
            bus.feedback()

    def test_stale_feedback_still_zeros_closes_can_and_t265(self):
        instances = {}

        class FakeBus:
            def __init__(self, current_limits=None):
                instances["bus"] = self
                self.current_limit_a = dict(current_limits or {})
                self.opened = self.zeroed = self.closed = False

            def snapshot(self):
                return []

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
            def __init__(self, current_limits=None):
                instances["bus"] = self
                self.current_limit_a = dict(current_limits or {})
                self.zero_called = self.closed = False

            def snapshot(self):
                return []

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


class LateBreakawayTests(unittest.TestCase):
    """D9-6: an axis that lets go in the last second of the hold moved.

    All three D9-6 runs broke away near the end of the hold and then ran
    toward the target while the current collapsed.  A median over the whole
    hold sits on the stationary part before that and scored 0x1C's real
    3.5 deg move as 0.2 deg of dither.
    """

    def _rows(self, positions, currents, hold_from=0):
        return [
            (index * 0.02,
             "probe-hold" if index >= hold_from else "probe-ramp",
             "0x1C", 0.0, 0.0, 0.0,
             np.deg2rad(position), 0.0, current)
            for index, (position, current) in enumerate(zip(positions, currents))
        ]

    def test_breakaway_in_the_final_quarter_of_the_hold_scores_as_motion(self):
        # Eight samples stationary at 0.2 deg holding 0.87 A, then it lets go
        # and runs to 3.5 deg while the current falls away.
        positions = [0.2] * 8 + [0.5, 1.9, 2.7] + [3.5] * 5
        currents = [0.87] * 8 + [0.31, 0.34, 0.07] + [0.13] * 5
        result = sender.probe_result(
            self._rows(positions, currents), 0.0, np.deg2rad(6.5)
        )
        self.assertAlmostEqual(np.rad2deg(result.net_rad), 3.5, places=1)
        self.assertEqual(result.letter, "A")
        self.assertAlmostEqual(result.breakaway_current_a, 0.87, places=2)

    def test_genuine_dither_is_still_scored_c(self):
        # D9-3: two increments, non-monotone, no run at the end.
        positions = [-0.4, -0.5, -0.4, -0.5, -0.6, -0.6, -0.6, -0.6]
        currents = [0.86] * 8
        result = sender.probe_result(
            self._rows(positions, currents), np.deg2rad(-0.4), np.deg2rad(-6.5)
        )
        self.assertAlmostEqual(np.rad2deg(result.net_rad), 0.2, places=1)
        self.assertEqual(result.letter, "C")

    def test_hold_tail_is_the_last_quarter_of_the_hold_stage(self):
        rows = self._rows([0.0] * 12, [0.0] * 12, hold_from=4)
        self.assertEqual(len(sender.held_rows(rows)), 8)
        self.assertEqual(len(sender.hold_tail_rows(rows)), 2)


class AllAxesFlagTests(unittest.TestCase):
    """D10: the whole-body step must be explicit, never a slip of the wrist."""

    _BASE = [
        "ver9_d8_sender.py", "--arm", "--package", "pkg",
        "--ramp-seconds", "15", "--duration", "2", "--csv", "out.csv",
    ]

    def test_all_axes_drives_every_registered_axis(self):
        with patch.object(sys, "argv", self._BASE + ["--all-axes", "--policy-slew-dps", "30"]), \
                patch.object(sender, "run") as run:
            sender.main()
        self.assertEqual(run.call_args.kwargs["motor_ids"], sender.H_CAN_IDS)

    def test_all_axes_cannot_be_combined_with_a_single_motor_id(self):
        argv = self._BASE + ["--all-axes", "--motor-id", "0x1C"]
        with patch.object(sys, "argv", argv), \
                patch.object(sender, "run") as run:
            with self.assertRaises(SystemExit):
                sender.main()
        run.assert_not_called()

    def test_arm_without_all_axes_still_requires_one_motor_id(self):
        with patch.object(sys, "argv", self._BASE), \
                patch.object(sender, "run") as run:
            with self.assertRaises(SystemExit):
                sender.main()
        run.assert_not_called()

    def test_single_axis_arm_is_unchanged(self):
        argv = self._BASE + ["--motor-id", "0x1C"]
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run:
            sender.main()
        self.assertEqual(run.call_args.kwargs["motor_ids"], (0x1C,))


class PerAxisLimitTests(unittest.TestCase):
    """The current abort is per axis, and the default is unchanged."""

    def test_default_limits_are_the_unchanged_one_axis_value(self):
        limits = sender.default_current_limits()
        self.assertEqual(set(limits), set(sender.H_CAN_IDS))
        for value in limits.values():
            self.assertEqual(value, sender.CURRENT_ABORT_A)

    def test_gravity_limits_clear_the_measured_static_gravity_load(self):
        # Worst-case static gravity current per axis, from
        # onshape_export/myrobot_dummy/robot_sim.urdf with the measured c_p.
        required_a = {"HR": 1.95, "HAA": 3.78, "HFE": 7.36, "KFE": 0.93, "FFE": 0.20}
        limits = sender.gravity_current_limits()
        for motor_id, limit in limits.items():
            suffix = sender.joint_suffix(motor_id)
            self.assertGreater(limit, required_a[suffix],
                               f"0x{motor_id:02X} cannot hold itself up")

    def test_gravity_limits_stay_far_below_the_trained_policy_effort_limit(self):
        # AK10-9 53 N*m, AK80-9 13.5 N*m (H_eff13p5_2999/actuator_table.md).
        effort_nm = {"AK10-9": 53.0, "AK80-9": 13.5}
        for index, motor_id in enumerate(sender.H_CAN_IDS):
            model = sender.H_MODELS[index]
            policy_current = effort_nm[model] / sender.CP[model]
            self.assertLess(sender.gravity_current_limits()[motor_id],
                            0.5 * policy_current,
                            f"0x{motor_id:02X} limit is no longer a guard")

    def test_feedback_uses_the_per_axis_limit_not_the_flat_one(self):
        class State:
            t = __import__("time").time()
            err = 0
            pos = spd = 0.0
            cur = 5.0

        buses = {channel: type("Bus", (), {"state": lambda _self, _mid: State()})()
                 for channel in (0, 1)}
        bus = sender.DualBus.__new__(sender.DualBus)
        bus.bus_by_channel = buses
        bus.route_by_motor_id = {motor_id: buses[0] for motor_id in sender.H_CAN_IDS}
        # 5.0 A trips the flat limit on every axis ...
        bus.current_limit_a = sender.default_current_limits()
        with self.assertRaisesRegex(RuntimeError, "motion/current abort"):
            bus.feedback()
        # ... and passes under the gravity table, whose smallest entry is above it
        # for the axes that actually carry load.  HFE is the binding one.
        bus.current_limit_a = {motor_id: 6.0 for motor_id in sender.H_CAN_IDS}
        bus.feedback()


class AxisLoggingTests(unittest.TestCase):
    """A whole-body run logs every driven axis, not only H_CAN_IDS[0]."""

    class FakeBus:
        def __init__(self, current_limits=None):
            self.current_limit_a = dict(current_limits or {})

        def state(self, motor_id):
            return SimpleNamespace(pos=1.0, spd=0.0, cur=0.25, err=0,
                                   t=__import__("time").time())

    def test_axis_rows_emits_one_row_per_driven_axis(self):
        targets = tuple(0.01 * index for index in range(10))
        rows = sender.axis_rows(1.0, "ramp", targets, targets, self.FakeBus(),
                                sender.H_CAN_IDS)
        self.assertEqual(len(rows), 10)
        self.assertEqual([row[2] for row in rows],
                         [f"0x{mid:02X}" for mid in sender.H_CAN_IDS])
        self.assertTrue(all(row[1] == "ramp" for row in rows))

    def test_axis_rows_follows_the_driven_set_when_one_axis_is_selected(self):
        targets = tuple(0.0 for _ in range(10))
        rows = sender.axis_rows(1.0, "ramp", targets, targets, self.FakeBus(),
                                (0x2A,))
        self.assertEqual([row[2] for row in rows], ["0x2A"])

    def test_rows_for_axis_selects_only_that_axis(self):
        targets = tuple(0.0 for _ in range(10))
        rows = sender.axis_rows(1.0, "ramp", targets, targets, self.FakeBus(),
                                sender.H_CAN_IDS)
        self.assertEqual(len(sender.rows_for_axis(rows, 0x1C)), 1)
        self.assertEqual(sender.rows_for_axis(rows, 0x1C)[0][2], "0x1C")


class HoldPoseTests(unittest.TestCase):
    """Hold-pose measures what each axis needs to hold itself."""

    def test_report_hold_pose_names_the_tightest_axis(self):
        rows = []
        # 0x2A holds 0.9 A against a 1.0 A limit; everyone else holds 0.1 A.
        for tick in range(40):
            for motor_id in sender.H_CAN_IDS:
                current = 0.9 if motor_id == 0x2A else 0.1
                rows.append((0.02 * tick, "hold-pose", f"0x{motor_id:02X}",
                             0.0, 0.0, 0.0, 0.0, 0.0, current))
        worst = sender.report_hold_pose(rows, tuple(0.0 for _ in range(10)),
                                        sender.H_CAN_IDS,
                                        sender.default_current_limits())
        self.assertEqual(min(worst)[1], 0x2A)

    def test_hold_pose_requires_all_axes(self):
        with patch.object(sys, "argv",
                          ["ver9_d8_sender.py", "--arm", "--hold-pose",
                           "--csv", "x.csv", "--duration", "2"]):
            with self.assertRaises(SystemExit):
                sender.main()

    def test_hold_pose_refuses_a_velocity_command(self):
        with patch.object(sys, "argv",
                          ["ver9_d8_sender.py", "--arm", "--all-axes", "--hold-pose",
                           "--csv", "x.csv", "--duration", "2", "--vx", "0.2"]):
            with self.assertRaises(SystemExit):
                sender.main()

    def test_hold_pose_refuses_a_policy_package(self):
        with patch.object(sys, "argv",
                          ["ver9_d8_sender.py", "--arm", "--all-axes", "--hold-pose",
                           "--csv", "x.csv", "--duration", "2", "--package", "p"]):
            with self.assertRaises(SystemExit):
                sender.main()


class GravityLimitFlagTests(unittest.TestCase):
    """--gravity-limits raises an abort, so it has to be typed deliberately."""

    def test_gravity_limits_refuses_without_all_axes(self):
        with patch.object(sys, "argv",
                          ["ver9_d8_sender.py", "--arm", "--gravity-limits",
                           "--motor-id", "0x1C", "--csv", "x.csv",
                           "--duration", "2", "--ramp-seconds", "15",
                           "--package", "p"]):
            with self.assertRaises(SystemExit):
                sender.main()

    def test_gravity_limits_refuses_with_static_probe(self):
        with patch.object(sys, "argv",
                          ["ver9_d8_sender.py", "--arm", "--all-axes",
                           "--gravity-limits", "--static-probe",
                           "--probe-target-deg", "2", "--csv", "x.csv",
                           "--duration", "2", "--ramp-seconds", "5"]):
            with self.assertRaises(SystemExit):
                sender.main()


class IncompleteRunTests(unittest.TestCase):
    """A CSV that holds ramp samples only says so, instead of being scored."""

    def test_analyze_flags_a_ramp_only_multi_axis_csv(self):
        import contextlib
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ramp_only.csv"
            with path.open("w", newline="", encoding="utf-8") as file:
                file.write("tick,stage,sent_motor_id,desired_target_rad,"
                           "requested_target_rad,wire_target_rad,"
                           "feedback_position_rad,feedback_velocity_rad_s,"
                           "feedback_current_a\n")
                for tick in range(20):
                    for motor_id in sender.H_CAN_IDS:
                        file.write(f"{0.02 * tick},ramp,0x{motor_id:02X},"
                                   "0.01,0.005,0.005,0.0,0.0,0.02\n")
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                sender.analyze_csv(path)
        printed = buffer.getvalue()
        self.assertIn("INCOMPLETE", printed)
        self.assertIn("PER-AXIS RESULT", printed)


class HoldPoseRunTests(unittest.TestCase):
    """run_hold_pose writes every axis and never sends a policy target."""

    def test_hold_pose_writes_ten_axes_and_commands_only_the_start_pose(self):
        import contextlib

        sent = []

        class FakeBus:
            tx_count = 0

            def __init__(self, current_limits=None):
                self.current_limit_a = dict(current_limits or {})
                self.zeroed = self.closed = False

            def open(self):
                return None

            def discover_routes(self):
                return None

            def feedback(self):
                return {motor_id: sender.MotorFeedback(motor_id, 0.0, 0.05, 0.0)
                        for motor_id in sender.H_CAN_IDS}

            def state(self, motor_id):
                return SimpleNamespace(pos=2.8648, spd=0.0, cur=0.3, err=0,
                                       t=__import__("time").time())

            def send(self, frame):
                sent.append(frame.arbitration_id & 255)

            def snapshot(self):
                return []

            def zero(self):
                self.zeroed = True

            def close(self):
                self.closed = True

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hold.csv"
            with patch.object(sender, "DualBus", FakeBus):
                buffer = io.StringIO()
                with contextlib.redirect_stdout(buffer):
                    sender.run_hold_pose(path, 0.15, sender.H_CAN_IDS,
                                         sender.gravity_current_limits())
            text = path.read_text(encoding="utf-8").splitlines()
        self.assertIn("HOLD POSE RESULT", buffer.getvalue())
        # Every tick writes one row per axis, and every axis appears.
        body = text[1:]
        self.assertTrue(body)
        self.assertEqual(len(body) % len(sender.H_CAN_IDS), 0)
        self.assertEqual({line.split(",")[2] for line in body},
                         {f"0x{motor_id:02X}" for motor_id in sender.H_CAN_IDS})
        self.assertTrue(all(line.split(",")[1] == "hold-pose" for line in body))
        # The commanded target is the measured start position on every row,
        # so nothing is asked to move anywhere.
        self.assertEqual({line.split(",")[3] for line in body},
                         {line.split(",")[4] for line in body})
        self.assertEqual(set(sent), set(sender.H_CAN_IDS))


class AllAxesSlewTests(unittest.TestCase):
    """D10-4: every axis is slew-limited after the ramp, and the gate refuses
    a frozen target the open-loop ramp must not chase."""

    _BASE = [
        "ver9_d8_sender.py", "--arm", "--all-axes", "--package", "pkg",
        "--ramp-seconds", "15", "--duration", "3", "--csv", "out.csv",
    ]

    def test_all_axes_policy_run_requires_policy_slew(self):
        with patch.object(sys, "argv", self._BASE), \
                patch.object(sender, "run") as run:
            with self.assertRaises(SystemExit):
                sender.main()
        run.assert_not_called()

    def test_policy_slew_is_passed_in_radians(self):
        with patch.object(sys, "argv", self._BASE + ["--policy-slew-dps", "30"]), \
                patch.object(sender, "run") as run:
            sender.main()
        self.assertAlmostEqual(run.call_args.kwargs["policy_slew_rad_s"],
                               np.deg2rad(30.0))

    def test_policy_slew_range_is_enforced(self):
        for value in ("0", "-5", "61"):
            with patch.object(sys, "argv", self._BASE + ["--policy-slew-dps", value]), \
                    patch.object(sender, "run") as run:
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()

    def test_policy_slew_refused_without_all_axes_or_with_hold_pose(self):
        single = ["ver9_d8_sender.py", "--arm", "--package", "pkg", "--motor-id", "0x1C",
                  "--ramp-seconds", "15", "--duration", "2", "--csv", "o.csv",
                  "--policy-slew-dps", "30"]
        hold = ["ver9_d8_sender.py", "--arm", "--all-axes", "--hold-pose",
                "--duration", "2", "--csv", "o.csv", "--policy-slew-dps", "30"]
        for argv in (single, hold):
            with patch.object(sys, "argv", argv), \
                    patch.object(sender, "run") as run, \
                    patch.object(sender, "run_hold_pose") as hold_run:
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()
            hold_run.assert_not_called()

    def test_single_axis_run_keeps_its_old_behaviour(self):
        argv = ["ver9_d8_sender.py", "--arm", "--package", "pkg", "--motor-id", "0x1C",
                "--ramp-seconds", "15", "--duration", "2", "--csv", "o.csv"]
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run:
            sender.main()
        self.assertIsNone(run.call_args.kwargs["policy_slew_rad_s"])

    def test_slew_all_limits_every_axis_independently(self):
        previous = tuple(0.0 for _ in range(10))
        desired = tuple((-1.0) ** i * 0.5 for i in range(10))
        got = sender.slew_all(previous, desired, np.deg2rad(30.0), 0.02)
        step = np.deg2rad(30.0) * 0.02
        for value, want in zip(got, desired):
            self.assertAlmostEqual(abs(value), step)
            self.assertEqual(np.sign(value), np.sign(want))

    def test_prearm_gate_flags_large_target_or_delta(self):
        positions = tuple(0.0 for _ in range(10))
        ok = tuple(np.deg2rad(20.0) for _ in range(10))
        self.assertEqual(sender.all_axes_prearm_violations(ok, positions), [])
        big_delta = list(ok); big_delta[9] = np.deg2rad(-37.3)   # D10-3 R4, 0x22
        self.assertEqual(len(sender.all_axes_prearm_violations(big_delta, positions)), 1)
        far = list(positions); far_t = list(ok)
        far[8] = np.deg2rad(-30.0); far_t[8] = np.deg2rad(-45.0)  # |target|>40, delta 15
        self.assertEqual(len(sender.all_axes_prearm_violations(far_t, far)), 1)

    def _fake_bus(self, sent):
        class FakeBus:
            tx_count = 0

            def __init__(self, current_limits=None):
                self.current_limit_a = dict(current_limits or {})

            def open(self):
                return None

            def discover_routes(self):
                return None

            def feedback(self):
                return {mid: sender.MotorFeedback(mid, 0.0, 0.0, 0.0)
                        for mid in sender.H_CAN_IDS}

            def state(self, _mid):
                return SimpleNamespace(pos=0.0, spd=0.0, cur=0.1, err=0,
                                       t=__import__("time").time())

            def send(self, frame):
                sent.append(frame)

            def snapshot(self):
                return []

            def zero(self):
                return None

            def close(self):
                return None
        return FakeBus

    class _FakeT265:
        def __init__(self, _offset):
            pass

        def start(self):
            return None

        def latest(self):
            return object()

        def close(self):
            return None

    def test_policy_stage_never_steps_any_axis(self):
        import contextlib
        sent = []
        first = SimpleNamespace(joint_target_h_order=(0.1,) * 10, action_raw=np.zeros(10))
        # The live policy then asks for a 23 deg step on every axis.
        step = SimpleNamespace(joint_target_h_order=(0.5,) * 10, action_raw=np.zeros(10))
        outputs = iter([first] + [step] * 1000)
        rate = np.deg2rad(30.0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slew.csv"
            buffer = io.StringIO()
            with patch.object(sender, "HPolicy"), \
                    patch.object(sender, "DualBus", self._fake_bus(sent)), \
                    patch.object(sender, "RealT265", self._FakeT265), \
                    patch.object(sender, "evaluate_cycle",
                                 side_effect=lambda *a, **k: (None, None, next(outputs), None)), \
                    contextlib.redirect_stdout(buffer):
                sender.run(Path(directory), 0.2, path, 0.0, 0.0, 0.0, transmit=True,
                           ramp_seconds=0.1, motor_ids=sender.H_CAN_IDS,
                           current_limits=sender.gravity_current_limits(),
                           policy_slew_rad_s=rate)
            import csv as _csv
            rows = list(_csv.DictReader(path.open(encoding="utf-8")))
        policy = [r for r in rows if r["stage"] == "policy"]
        self.assertTrue(policy)
        self.assertEqual({r["sent_motor_id"] for r in policy},
                         {f"0x{m:02X}" for m in sender.H_CAN_IDS})
        by_axis = {}
        for r in policy:
            by_axis.setdefault(r["sent_motor_id"], []).append(r)
        for axis in by_axis.values():
            previous_tick, previous = None, 0.1
            for r in axis:
                tick, requested = float(r["tick"]), float(r["requested_target_rad"])
                # First policy tick starts from the frozen ramp target.
                bound = rate * ((tick - previous_tick) if previous_tick else 0.2) + 1e-9
                self.assertLessEqual(abs(requested - previous), bound)
                self.assertLess(requested, 0.5)
                previous_tick, previous = tick, requested
        self.assertIn("SLEW GAP", buffer.getvalue())

    def test_prearm_gate_refuses_before_any_mit_frame(self):
        import contextlib
        sent = []
        bad = SimpleNamespace(joint_target_h_order=(0.1,) * 9 + (np.deg2rad(-63.8),),
                              action_raw=np.zeros(10))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gate.csv"
            with patch.object(sender, "HPolicy"), \
                    patch.object(sender, "DualBus", self._fake_bus(sent)), \
                    patch.object(sender, "RealT265", self._FakeT265), \
                    patch.object(sender, "evaluate_cycle",
                                 return_value=(None, None, bad, None)), \
                    contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError, "PRE-ARM GATE"):
                    sender.run(Path(directory), 1.0, path, 0.0, 0.0, 0.0, transmit=True,
                               ramp_seconds=15, motor_ids=sender.H_CAN_IDS,
                               current_limits=sender.gravity_current_limits(),
                               policy_slew_rad_s=np.deg2rad(30.0))
        self.assertEqual(sent, [])

    def test_analyze_uses_the_frozen_ramp_target(self):
        import contextlib
        rows = ["tick,stage,sent_motor_id,desired_target_rad,requested_target_rad,"
                "wire_target_rad,feedback_position_rad,feedback_velocity_rad_s,feedback_current_a"]
        for tick in range(5):
            for mid in sender.H_CAN_IDS:
                rows.append(f"{tick*0.02},ramp,0x{mid:02X},-0.2,{-0.04*tick},0,0,0,0.1")
        for mid in sender.H_CAN_IDS:
            rows.append(f"0.2,policy,0x{mid:02X},0.3,-0.2,0,0,0,0.1")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "a.csv"
            path.write_text("\n".join(rows) + "\n", encoding="utf-8")
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                sender.analyze_csv(path)
        text = buffer.getvalue()
        self.assertIn("asked= -11.46deg", text)
        self.assertIn("POLICY STAGE: 1 tick(s)", text)


class StandPoseTests(unittest.TestCase):
    """D10-5: ramp to the sim default pose, hold it, then hand over to the
    slew-limited policy from there (not from the policy's first output)."""

    _fake_bus = AllAxesSlewTests._fake_bus
    _FakeT265 = AllAxesSlewTests._FakeT265

    _STAND = [
        "ver9_d8_sender.py", "--arm", "--all-axes", "--package", "pkg",
        "--ramp-seconds", "10", "--csv", "s.csv", "--stand-seconds", "2",
    ]

    def test_stand_target_is_the_sim_default_pose(self):
        deg = [round(float(np.rad2deg(v)), 1) for v in sender.STAND_TARGET]
        self.assertEqual(deg, [0.0, 0.0, 0.0, 0.0, -10.0, -10.0, 20.0, 20.0, -10.0, -10.0])

    def test_stand_only_needs_no_slew_or_duration(self):
        with patch.object(sys, "argv", self._STAND + ["--stand-only"]), \
                patch.object(sender, "run") as run:
            sender.main()
        self.assertTrue(run.call_args.kwargs["stand_only"])
        self.assertEqual(run.call_args.kwargs["stand_seconds"], 2.0)

    def test_stand_only_refuses_policy_arguments(self):
        for extra in (["--duration", "3"], ["--policy-slew-dps", "20"], ["--vx", "0.2"]):
            with patch.object(sys, "argv", self._STAND + ["--stand-only"] + extra), \
                    patch.object(sender, "run") as run:
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()

    def test_stand_seconds_range_and_scope(self):
        bad = [
            ["ver9_d8_sender.py", "--arm", "--all-axes", "--package", "pkg", "--ramp-seconds", "10",
             "--csv", "s.csv", "--stand-seconds", "6", "--stand-only"],
            ["ver9_d8_sender.py", "--arm", "--all-axes", "--hold-pose", "--duration", "2",
             "--csv", "s.csv", "--stand-seconds", "2"],
            ["ver9_d8_sender.py", "--arm", "--package", "pkg", "--motor-id", "0x1C", "--ramp-seconds",
             "10", "--duration", "2", "--csv", "s.csv", "--stand-seconds", "2"],
        ]
        for argv in bad:
            with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                    patch.object(sender, "run_hold_pose") as hold:
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()
            hold.assert_not_called()

    def _run_stand(self, stand_only):
        import contextlib
        import csv as _csv
        sent = []
        first = SimpleNamespace(joint_target_h_order=(0.3,) * 10, action_raw=np.ones(10))
        step = SimpleNamespace(joint_target_h_order=(1.2,) * 10, action_raw=np.zeros(10))
        outputs = iter([first] + [step] * 1000)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stand.csv"
            buffer = io.StringIO()
            with patch.object(sender, "HPolicy"), \
                    patch.object(sender, "DualBus", self._fake_bus(sent)), \
                    patch.object(sender, "RealT265", self._FakeT265), \
                    patch.object(sender, "evaluate_cycle",
                                 side_effect=lambda *a, **k: (None, None, next(outputs), None)), \
                    contextlib.redirect_stdout(buffer):
                sender.run(Path(directory), 0.0 if stand_only else 0.2, path, 0.0, 0.0, 0.0,
                           transmit=True, ramp_seconds=0.1, motor_ids=sender.H_CAN_IDS,
                           current_limits=sender.gravity_current_limits(),
                           policy_slew_rad_s=None if stand_only else np.deg2rad(20.0),
                           stand_seconds=0.1, stand_only=stand_only)
            rows = list(_csv.DictReader(path.open(encoding="utf-8")))
        return rows, buffer.getvalue()

    def test_stand_only_ramps_to_default_and_runs_no_policy(self):
        rows, text = self._run_stand(True)
        stages = {r["stage"] for r in rows}
        self.assertEqual(stages, {"stand-ramp", "stand-hold"})
        for r in rows:
            index = [f"0x{m:02X}" for m in sender.H_CAN_IDS].index(r["sent_motor_id"])
            self.assertAlmostEqual(float(r["desired_target_rad"]), sender.STAND_TARGET[index])
        self.assertIn("HOLD POSE RESULT", text)
        self.assertIn("STAND ONLY", text)

    def test_policy_starts_from_the_stand_pose_not_its_first_output(self):
        rows, text = self._run_stand(False)
        self.assertIn("stand-hold", {r["stage"] for r in rows})
        policy = [r for r in rows if r["stage"] == "policy"]
        self.assertTrue(policy)
        ids = [f"0x{m:02X}" for m in sender.H_CAN_IDS]
        first_tick = min(float(r["tick"]) for r in policy)
        for r in policy:
            if float(r["tick"]) != first_tick:
                continue
            index = ids.index(r["sent_motor_id"])
            # One slew step (20 deg/s over at most ~one stand period) from the stand pose.
            self.assertLess(abs(float(r["requested_target_rad"]) - sender.STAND_TARGET[index]),
                            np.deg2rad(20.0) * 0.2)
        self.assertIn("SLEW GAP", text)


from motor_console_ver8_2 import unpack_mit


class JointSignRoundTripTests(unittest.TestCase):
    """D10-6 (2026-09-23): a sign -1 joint is flipped on BOTH sides of the wire."""

    def test_h_plus_10_goes_out_as_motor_minus_10_and_comes_back(self):
        mid = 0x2A  # LR_HFE, sign -1
        index = sender.H_CAN_IDS.index(mid)
        targets = [0.0] * 10
        targets[index] = np.deg2rad(10.0)
        (sent,) = sender.frames(targets, (mid,))
        on_wire = unpack_mit(bytes(sent.data), "AK80-9")["pos"]
        self.assertAlmostEqual(np.rad2deg(on_wire), -10.0, places=1)
        wire = sender.wire_command(mid, np.deg2rad(10.0))[2]
        self.assertAlmostEqual(np.rad2deg(wire), -10.0, places=1)
        state = SimpleNamespace(pos=-10.0, spd=-31.5, cur=-0.5)
        position, velocity, current = sender.h_feedback(mid, state)
        self.assertAlmostEqual(np.rad2deg(position), 10.0)
        self.assertAlmostEqual(velocity, np.deg2rad(1.0))
        self.assertAlmostEqual(current, 0.5)

    def test_plus_one_joint_is_unchanged(self):
        mid = 0x21  # LL_HFE, sign +1
        index = sender.H_CAN_IDS.index(mid)
        targets = [0.0] * 10
        targets[index] = np.deg2rad(10.0)
        (sent,) = sender.frames(targets, (mid,))
        on_wire = unpack_mit(bytes(sent.data), "AK80-9")["pos"]
        self.assertAlmostEqual(np.rad2deg(on_wire), 10.0, places=1)
        position, _v, _c = sender.h_feedback(mid, SimpleNamespace(pos=10.0, spd=0.0, cur=0.0))
        self.assertAlmostEqual(np.rad2deg(position), 10.0)

    def test_stand_pose_is_mirrored_on_the_wire_for_flipped_joints(self):
        for mid in sender.H_CAN_IDS:
            index = sender.H_CAN_IDS.index(mid)
            wire = sender.wire_command(mid, sender.STAND_TARGET[index])[2]
            sign = sender.H_BINDING_BY_ID[mid].sign
            self.assertAlmostEqual(wire, sign * sender.STAND_TARGET[index], places=2)


class SignPoseTests(unittest.TestCase):
    """D10-7 (2026-09-23): a stand-only pose in which every joint sign shows."""

    def test_sign_pose_adds_outward_hr_and_haa_only(self):
        for index, mid in enumerate(sender.H_CAN_IDS):
            suffix = sender.H_BINDING_BY_ID[mid].name.split("_", 1)[1]
            extra = np.rad2deg(sender.SIGN_POSE_TARGET[index] - sender.STAND_TARGET[index])
            self.assertAlmostEqual(extra, 8.0 if suffix in ("HR", "HAA") else 0.0)
            self.assertLessEqual(abs(np.rad2deg(sender.SIGN_POSE_TARGET[index])),
                                 sender.ALL_AXES_MAX_TARGET_DEG)

    def test_sign_pose_is_mirrored_on_the_wire_for_lr_hr(self):
        index = sender.H_CAN_IDS.index(0x13)  # LR_HR, sign -1
        wire = sender.wire_command(0x13, sender.SIGN_POSE_TARGET[index])[2]
        self.assertAlmostEqual(np.rad2deg(wire), -8.0, places=1)

    def test_sign_pose_requires_stand_only(self):
        argv = ["ver9_d8_sender.py", "--arm", "--all-axes", "--stand-seconds", "2",
                "--sign-pose", "--policy-slew-dps", "20", "--duration", "2",
                "--package", "x", "--ramp-seconds", "10", "--csv", "x.csv"]
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
            with self.assertRaises(SystemExit):
                sender.main()
        run.assert_not_called()

    def test_sign_pose_reaches_run(self):
        argv = ["ver9_d8_sender.py", "--arm", "--all-axes", "--gravity-limits",
                "--stand-seconds", "3", "--stand-only", "--sign-pose",
                "--package", "x", "--ramp-seconds", "10", "--csv", "x.csv"]
        out = io.StringIO()
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stdout", out):
            sender.main()
        self.assertEqual(run.call_args.kwargs["stand_target"], sender.SIGN_POSE_TARGET)
        self.assertIn("JOINT SIGNS", out.getvalue())
        self.assertIn("0x2A LR_HFE=-1", out.getvalue())

    def test_analyze_reads_the_sign_pose_from_the_csv(self):
        import contextlib
        rows = ["tick,stage,sent_motor_id,desired_target_rad,requested_target_rad,"
                "wire_target_motor_rad,feedback_position_rad,feedback_velocity_rad_s,"
                "feedback_current_h_a"]
        for tick in range(10):
            for index, mid in enumerate(sender.H_CAN_IDS):
                target = sender.SIGN_POSE_TARGET[index]
                rows.append(f"{tick*0.02},stand-hold,0x{mid:02X},{target},{target},0,{target},0,0.2")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sign.csv"
            path.write_text("\n".join(rows) + "\n", encoding="utf-8")
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                sender.analyze_csv(path)
        text = buffer.getvalue()
        self.assertIn("STAND HOLD (sign pose)", text)
        self.assertIn("LOOK CHECK", text)
        self.assertIn("toe turned OUTWARD", text)
        self.assertIn("droop=  +0.00deg", text)


class ObservationSidecarTests(unittest.TestCase):
    """D10-8: what the policy saw is saved next to the run CSV and summarised."""

    def test_sidecar_round_trip_and_report(self):
        import contextlib
        with tempfile.TemporaryDirectory() as directory:
            run_csv = Path(directory) / "r.csv"
            sidecar = sender.obs_csv_path(run_csv)
            self.assertEqual(sidecar.name, "r_obs.csv")
            obs = [0.0] * 42
            obs[8] = -1.0
            rows = [(0.0, "initial", *obs, *([0.0] * 10))]
            rows += [(0.02 * t, "policy", *obs, *([0.1] * 10)) for t in range(5)]
            sender.write_obs_csv(sidecar, rows)
            self.assertEqual(len(sender.obs_header()), 2 + 42 + 10)
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                tilt = sender.report_observation(sidecar)
        self.assertAlmostEqual(tilt, 0.0, places=3)
        self.assertIn("5 policy tick(s)", buffer.getvalue())
        self.assertIn("LL_KFE=+0.0", buffer.getvalue())


class HaaSweepTests(unittest.TestCase):
    """D10-9 (2026-09-23): close both HAA from the stand pose and log where the feet meet."""

    _BASE = ["ver9_d8_sender.py", "--arm", "--all-axes", "--gravity-limits",
             "--stand-seconds", "2", "--stand-only", "--package", "x",
             "--ramp-seconds", "10", "--csv", "x.csv"]

    def test_targets_move_only_haa_inward_and_stop_at_the_end(self):
        indices = sender.haa_indices()
        self.assertEqual([sender.H_BINDING_BY_ID[sender.H_CAN_IDS[i]].name for i in indices],
                         ["LL_HAA", "LR_HAA"])
        half = sender.haa_sweep_targets(sender.STAND_TARGET, np.deg2rad(16), 4.0)
        end = sender.haa_sweep_targets(sender.STAND_TARGET, np.deg2rad(16), 100.0)
        for index, (h, e, s) in enumerate(zip(half, end, sender.STAND_TARGET)):
            if index in indices:
                self.assertAlmostEqual(np.rad2deg(h - s), -4.0 * sender.HAA_SWEEP_DPS)
                self.assertAlmostEqual(np.rad2deg(e - s), -16.0)
            else:
                self.assertEqual(h, s)
                self.assertEqual(e, s)

    def test_block_rule(self):
        four = np.deg2rad(4.0)
        droop = [-np.deg2rad(3.0)] * 50           # gravity pulls inward: never a block
        self.assertIsNone(sender.haa_block_tick(droop))
        nine = [0.0] * 5 + [four * 1.1] * 9 + [0.0]
        self.assertIsNone(sender.haa_block_tick(nine))
        ten = [0.0] * 5 + [four * 1.1] * 10
        self.assertEqual(sender.haa_block_tick(ten), 5)

    def test_flag_scope_and_range(self):
        bad = [
            [a for a in self._BASE if a != "--stand-only"] + ["--haa-close-deg", "10",
                                                               "--policy-slew-dps", "20", "--duration", "2"],
            self._BASE + ["--haa-close-deg", "10", "--sign-pose"],
            self._BASE + ["--haa-close-deg", "0"],
            self._BASE + ["--haa-close-deg", "17"],
        ]
        for argv in bad:
            with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                    patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()

    def test_flag_reaches_run_in_radians(self):
        out = io.StringIO()
        with patch.object(sys, "argv", self._BASE + ["--haa-close-deg", "16"]), \
                patch.object(sender, "run") as run, patch("sys.stdout", out):
            sender.main()
        self.assertAlmostEqual(run.call_args.kwargs["haa_close_rad"], np.deg2rad(16.0))
        self.assertEqual(run.call_args.kwargs["stand_target"], sender.STAND_TARGET)
        self.assertIn("HAA SWEEP planned", out.getvalue())

    def test_without_the_flag_nothing_changes(self):
        with patch.object(sys, "argv", self._BASE), patch.object(sender, "run") as run, \
                patch("sys.stdout", io.StringIO()):
            sender.main()
        self.assertIsNone(run.call_args.kwargs["haa_close_rad"])

    def test_run_freezes_at_the_first_block_and_analyze_finds_it(self):
        import contextlib
        import csv as _csv
        sent = []
        first = SimpleNamespace(joint_target_h_order=(0.0,) * 10, action_raw=np.zeros(10))
        # The fake bus always reports every axis at 0 rad, so the legs "stop" at 0
        # while the HAA target keeps moving inward: a block at feedback 0.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sweep.csv"
            buffer = io.StringIO()
            with patch.object(sender, "HPolicy"), \
                    patch.object(sender, "DualBus", AllAxesSlewTests._fake_bus(None, sent)), \
                    patch.object(sender, "RealT265", AllAxesSlewTests._FakeT265), \
                    patch.object(sender, "evaluate_cycle",
                                 side_effect=lambda *a, **k: (None, None, first, None)), \
                    patch.object(sender, "HAA_SWEEP_DPS", 400.0), \
                    patch.object(sender, "HAA_SWEEP_HOLD_S", 0.5), \
                    contextlib.redirect_stdout(buffer):
                sender.run(Path(directory), 0.0, path, 0.0, 0.0, 0.0, transmit=True,
                           ramp_seconds=0.1, motor_ids=sender.H_CAN_IDS,
                           current_limits=sender.gravity_current_limits(),
                           stand_seconds=0.1, stand_only=True, haa_close_rad=np.deg2rad(16.0))
                with path.open(encoding="utf-8") as handle:
                    rows = list(_csv.DictReader(handle))
                analyzed = io.StringIO()
                with contextlib.redirect_stdout(analyzed):
                    sender.analyze_csv(path)
        text = buffer.getvalue()
        stages = [r["stage"] for r in rows]
        self.assertIn("haa-sweep", stages)
        self.assertIn("haa-hold", stages)
        self.assertIn("HAA CONTACT", text)
        self.assertRegex(text, r"BLOCKED at feedback [+-]0\.0deg")
        haa_ids = {f"0x{sender.H_CAN_IDS[i]:02X}" for i in sender.haa_indices()}
        last_tick = max(float(r["tick"]) for r in rows)
        ids = [f"0x{m:02X}" for m in sender.H_CAN_IDS]
        for r in rows:
            index = ids.index(r["sent_motor_id"])
            target = float(r["requested_target_rad"])
            if r["sent_motor_id"] in haa_ids:
                self.assertGreaterEqual(target, -np.deg2rad(16.0) - 1e-9)
                if float(r["tick"]) == last_tick:
                    # Frozen where the legs are (0), not pressed on toward the target.
                    self.assertAlmostEqual(target, 0.0)
            elif r["stage"].startswith("haa-"):
                self.assertAlmostEqual(target, sender.STAND_TARGET[index])
        self.assertIn("HAA SWEEP (D10-9)", analyzed.getvalue())
        self.assertIn("BLOCKED", analyzed.getvalue())


class FloorStandTests(unittest.TestCase):
    """D10-10 (2026-09-23): first loaded stand, hoist rope attached."""

    _BASE = ["ver9_d8_sender.py", "--arm", "--all-axes", "--floor-limits",
             "--stand-seconds", "30", "--stand-only", "--package", "x",
             "--ramp-seconds", "10", "--csv", "x.csv"]

    def test_floor_table_raises_only_kfe_and_ffe(self):
        floor = sender.floor_current_limits()
        gravity = sender.gravity_current_limits()
        for mid in sender.H_CAN_IDS:
            suffix = sender.joint_suffix(mid)
            if suffix in ("KFE", "FFE"):
                self.assertEqual(floor[mid], 6.0)
                self.assertGreater(floor[mid], gravity[mid])
            else:
                self.assertEqual(floor[mid], gravity[mid])

    def test_floor_limits_reach_run_with_a_long_hold(self):
        out = io.StringIO()
        with patch.object(sys, "argv", self._BASE), patch.object(sender, "run") as run, \
                patch("sys.stdout", out):
            sender.main()
        self.assertEqual(run.call_args.kwargs["current_limits"], sender.floor_current_limits())
        self.assertEqual(run.call_args.kwargs["stand_seconds"], 30.0)
        self.assertIn("FLOOR LIMITS", out.getvalue())

    def test_floor_policy_run_is_allowed(self):
        argv = [a for a in self._BASE if a != "--stand-only"] + [
            "--policy-slew-dps", "30", "--duration", "5"]
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stdout", io.StringIO()):
            sender.main()
        self.assertFalse(run.call_args.kwargs["stand_only"])
        self.assertEqual(run.call_args.kwargs["current_limits"], sender.floor_current_limits())

    def test_long_hold_needs_floor_limits_and_scope_is_enforced(self):
        long_hold = [a if a != "--floor-limits" else "--gravity-limits" for a in self._BASE]
        bad = [
            long_hold,
            self._BASE + ["--gravity-limits"],
            [a for a in self._BASE if a != "--all-axes"],
            self._BASE[:4] + ["--stand-seconds", "31"] + self._BASE[6:],
            self._BASE + ["--haa-close-deg", "10"],
        ]
        for argv in bad:
            with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                    patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()

    def test_progress_line_reports_each_pair(self):
        rows = []
        for mid in sender.H_CAN_IDS:
            rows.append((0.0, "stand-hold", f"0x{mid:02X}", 0, 0, 0, 0, 0,
                         2.5 if sender.H_BINDING_BY_ID[mid].name == "LR_KFE" else 0.5))
        line = sender.stand_progress_line(4.0, 30.0, rows)
        self.assertIn("KFE 0.50/2.50A", line)
        self.assertIn("4/30s", line)


class LeanSweepTests(unittest.TestCase):
    """D10-11 (2026-09-23): stand-pose knobs and the floor lean sweep."""

    _BASE = ["ver9_d8_sender.py", "--arm", "--all-axes", "--floor-limits",
             "--stand-seconds", "30", "--stand-only", "--package", "x",
             "--ramp-seconds", "10", "--csv", "x.csv"]

    def _deg(self, target):
        return {sender.H_BINDING_BY_ID[m].name: round(float(np.rad2deg(t)), 1)
                for m, t in zip(sender.H_CAN_IDS, target)}

    def test_stand_pose_knobs(self):
        self.assertEqual(sender.stand_pose_target(), sender.STAND_TARGET)
        self.assertEqual(self._deg(sender.stand_pose_target(20.0)), self._deg(sender.STAND_TARGET))
        d = self._deg(sender.stand_pose_target(10.0, 4.0))
        self.assertEqual((d["LL_HFE"], d["LR_KFE"], d["LL_FFE"], d["LR_FFE"], d["LL_HAA"]),
                         (-5.0, 10.0, -1.0, -1.0, 0.0))
        d0 = self._deg(sender.stand_pose_target(0.0))
        self.assertTrue(all(v == 0.0 for v in d0.values()))

    def test_lean_targets_move_only_ffe_toe_down(self):
        t = sender.lean_sweep_targets(sender.STAND_TARGET, np.deg2rad(8), 4.0)
        end = sender.lean_sweep_targets(sender.STAND_TARGET, np.deg2rad(8), 100.0)
        for i, (a, b, s) in enumerate(zip(t, end, sender.STAND_TARGET)):
            if i in sender.ffe_indices():
                self.assertAlmostEqual(np.rad2deg(a - s), 4.0 * sender.LEAN_SWEEP_DPS)
                self.assertAlmostEqual(np.rad2deg(b - s), 8.0)
            else:
                self.assertEqual((a, b), (s, s))

    def test_balance_crossing(self):
        self.assertAlmostEqual(sender.lean_balance_deg([0, 1, 2, 3], [4, 2, -2, -4]), 1.5)
        self.assertIsNone(sender.lean_balance_deg([0, 1, 2], [5, 4, 3]))

    def test_flags_reach_run(self):
        argv = self._BASE + ["--stand-knee-deg", "10", "--lean-sweep-deg", "8"]
        out = io.StringIO()
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stdout", out):
            sender.main()
        kw = run.call_args.kwargs
        self.assertAlmostEqual(kw["lean_sweep_rad"], np.deg2rad(8))
        self.assertEqual(kw["stand_target"], sender.stand_pose_target(10.0))
        self.assertIn("LEAN SWEEP planned", out.getvalue())

    def test_policy_run_with_lean(self):
        argv = [a for a in self._BASE if a != "--stand-only"]
        argv = argv[:5] + ["20"] + argv[6:] + ["--stand-knee-deg", "10", "--stand-lean-deg", "3",
                                              "--policy-slew-dps", "60", "--duration", "5"]
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stdout", io.StringIO()):
            sender.main()
        self.assertEqual(run.call_args.kwargs["stand_target"], sender.stand_pose_target(10.0, 3.0))
        self.assertIsNone(run.call_args.kwargs["lean_sweep_rad"])

    def test_scope_and_ranges(self):
        no_floor = [a if a != "--floor-limits" else "--gravity-limits" for a in self._BASE]
        no_floor = no_floor[:5] + ["5"] + no_floor[6:]
        bad = [
            no_floor + ["--lean-sweep-deg", "8"],
            self._BASE + ["--lean-sweep-deg", "11"],
            self._BASE + ["--stand-knee-deg", "31"],
            self._BASE + ["--stand-lean-deg", "9"],
            self._BASE + ["--stand-lean-deg", "-6"],
            [a if a != "--floor-limits" else "--gravity-limits" for a in self._BASE][:5] + ["2"]
            + self._BASE[6:] + ["--haa-close-deg", "10", "--stand-knee-deg", "10"],
        ]
        for argv in bad:
            with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                    patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()

    def test_run_and_analyze_report_a_balance(self):
        import contextlib
        import csv as _csv
        first = SimpleNamespace(joint_target_h_order=(0.0,) * 10, action_raw=np.zeros(10))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "lean.csv"
            buffer = io.StringIO()
            with patch.object(sender, "HPolicy"), \
                    patch.object(sender, "DualBus", AllAxesSlewTests._fake_bus(None, [])), \
                    patch.object(sender, "RealT265", AllAxesSlewTests._FakeT265), \
                    patch.object(sender, "evaluate_cycle",
                                 side_effect=lambda *a, **k: (None, None, first, None)), \
                    patch.object(sender, "LEAN_SWEEP_DPS", 40.0), \
                    patch.object(sender, "LEAN_SWEEP_HOLD_S", 0.1), \
                    contextlib.redirect_stdout(buffer):
                sender.run(Path(directory), 0.0, path, 0.0, 0.0, 0.0, transmit=True,
                           ramp_seconds=0.05, motor_ids=sender.H_CAN_IDS,
                           current_limits=sender.floor_current_limits(), stand_seconds=0.1,
                           stand_only=True, stand_target=sender.stand_pose_target(20.0, -5.0),
                           lean_sweep_rad=np.deg2rad(10.0))
                with path.open(encoding="utf-8") as handle:
                    stages = {r["stage"] for r in _csv.DictReader(handle)}
                analyzed = io.StringIO()
                with contextlib.redirect_stdout(analyzed):
                    sender.analyze_csv(path)
        # The fake bus reports every axis at 0: FFE starts at -15 target (err -15), then the
        # lean brings the target to -5 (err -5): the mean error never crosses from + to -.
        self.assertIn("lean-sweep", stages)
        self.assertIn("LEAN SWEEP (D10-11)", buffer.getvalue())
        self.assertIn("BALANCE LEAN", analyzed.getvalue())
        self.assertIn("custom stand pose", analyzed.getvalue())


class AutoLeanWalkTests(unittest.TestCase):
    """D10-12 (2026-09-23): auto lean inside the stand hold, and the walk limits."""

    _POLICY = ["ver9_d8_sender.py", "--arm", "--all-axes", "--stand-seconds", "20",
               "--package", "x", "--ramp-seconds", "10", "--duration", "5",
               "--csv", "x.csv"]

    def test_auto_lean_step_follows_heels_and_is_bounded(self):
        dt = 0.02
        heels = [np.deg2rad(6.0), np.deg2rad(6.0)]
        toes = [np.deg2rad(-6.0), np.deg2rad(-6.0)]
        up = sender.auto_lean_step(0.0, heels, dt, np.deg2rad(8))
        self.assertAlmostEqual(np.rad2deg(up), sender.AUTO_LEAN_MAX_DPS * dt)  # rate-limited
        self.assertEqual(sender.auto_lean_step(0.0, toes, dt, np.deg2rad(8)), 0.0)  # never below 0
        self.assertAlmostEqual(sender.auto_lean_step(np.deg2rad(8), heels, dt, np.deg2rad(8)),
                               np.deg2rad(8))  # never above max
        small = sender.auto_lean_step(0.0, [np.deg2rad(1.0)] * 2, dt, np.deg2rad(8))
        self.assertAlmostEqual(np.rad2deg(small), sender.AUTO_LEAN_GAIN * 1.0 * dt)

    def test_walk_limits_table_and_flags(self):
        walk = sender.walk_current_limits()
        floor = sender.floor_current_limits()
        for mid in sender.H_CAN_IDS:
            if sender.joint_suffix(mid) in ("KFE", "FFE"):
                self.assertEqual(walk[mid], 10.0)
            else:
                self.assertEqual(walk[mid], floor[mid])
        argv = self._POLICY + ["--walk-limits", "--auto-lean-deg", "8",
                               "--policy-slew-dps", "150", "--vx", "0.2"]
        out = io.StringIO()
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stdout", out):
            sender.main()
        kw = run.call_args.kwargs
        self.assertEqual(kw["current_limits"], walk)
        self.assertAlmostEqual(kw["speed_abort_rad_s"], np.deg2rad(200.0))
        self.assertAlmostEqual(kw["auto_lean_rad"], np.deg2rad(8))
        self.assertAlmostEqual(kw["policy_slew_rad_s"], np.deg2rad(150.0))
        self.assertIn("WALK LIMITS", out.getvalue())

    def test_floor_policy_keeps_the_old_speed_and_slew(self):
        argv = self._POLICY + ["--floor-limits", "--auto-lean-deg", "8", "--policy-slew-dps", "60"]
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stdout", io.StringIO()):
            sender.main()
        self.assertIsNone(run.call_args.kwargs["speed_abort_rad_s"])
        self.assertEqual(run.call_args.kwargs["current_limits"], sender.floor_current_limits())

    def test_refusals(self):
        bad = [
            self._POLICY + ["--floor-limits", "--policy-slew-dps", "100"],        # >60 without walk
            self._POLICY + ["--walk-limits", "--floor-limits", "--policy-slew-dps", "60"],
            self._POLICY + ["--walk-limits"],                                      # no slew
            self._POLICY + ["--walk-limits", "--policy-slew-dps", "201"],
            self._POLICY + ["--gravity-limits", "--auto-lean-deg", "8", "--policy-slew-dps", "30"],
            self._POLICY[:4] + ["10"] + self._POLICY[5:] + ["--floor-limits", "--auto-lean-deg", "8",
                                                            "--policy-slew-dps", "30"],
            self._POLICY + ["--floor-limits", "--auto-lean-deg", "9", "--policy-slew-dps", "30"],
            self._POLICY + ["--floor-limits", "--auto-lean-deg", "8", "--stand-lean-deg", "3",
                            "--policy-slew-dps", "30"],
        ]
        for argv in bad:
            with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                    patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()

    def test_run_leans_from_the_heels_and_policy_starts_from_the_leaned_pose(self):
        import contextlib
        import csv as _csv
        sent = []
        base_bus = AllAxesSlewTests._fake_bus(None, sent)
        ffe_ids = {sender.H_CAN_IDS[i] for i in sender.ffe_indices()}

        class HeelBus(base_bus):
            def feedback(self):
                # FFE sits 20 deg toe-up: the robot is on its heels, so the lean should grow.
                return {mid: sender.MotorFeedback(mid, 0.0, -0.35 if mid in ffe_ids else 0.0, 0.0)
                        for mid in sender.H_CAN_IDS}

        first = SimpleNamespace(joint_target_h_order=sender.STAND_TARGET, action_raw=np.zeros(10))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "auto.csv"
            buffer = io.StringIO()
            with patch.object(sender, "HPolicy"), patch.object(sender, "DualBus", HeelBus), \
                    patch.object(sender, "RealT265", AllAxesSlewTests._FakeT265), \
                    patch.object(sender, "evaluate_cycle",
                                 side_effect=lambda *a, **k: (None, None, first, None)), \
                    patch.object(sender, "AUTO_LEAN_START_S", 0.0), \
                    patch.object(sender, "AUTO_LEAN_MAX_DPS", 100.0), \
                    patch.object(sender, "AUTO_LEAN_GAIN", 50.0), \
                    patch.object(sender, "STAND_MAX_SECONDS", 0.05), \
                    contextlib.redirect_stdout(buffer):
                sender.run(Path(directory), 0.1, path, 0.0, 0.0, 0.0, transmit=True,
                           ramp_seconds=0.05, motor_ids=sender.H_CAN_IDS,
                           current_limits=sender.walk_current_limits(),
                           policy_slew_rad_s=np.deg2rad(150.0), stand_seconds=0.4,
                           auto_lean_rad=np.deg2rad(8.0),
                           speed_abort_rad_s=np.deg2rad(200.0))
            with path.open(encoding="utf-8") as handle:
                rows = list(_csv.DictReader(handle))
        text = buffer.getvalue()
        self.assertIn("AUTO LEAN RESULT: final lean +8.0deg", text)
        ffe_labels = {f"0x{m:02X}" for m in ffe_ids}
        hold = [r for r in rows if r["stage"] == "stand-hold" and r["sent_motor_id"] in ffe_labels]
        self.assertAlmostEqual(np.rad2deg(float(hold[-1]["requested_target_rad"])),
                               np.rad2deg(sender.STAND_TARGET[sender.ffe_indices()[0]]) + 8.0, places=3)
        policy = [r for r in rows if r["stage"] == "policy" and r["sent_motor_id"] in ffe_labels]
        first_policy = policy[0]
        # The first policy tick slews from the LEANED pose toward the policy target (the sim default).
        self.assertGreater(np.rad2deg(float(first_policy["requested_target_rad"])),
                           np.rad2deg(sender.STAND_TARGET[sender.ffe_indices()[0]]) + 8.0 - 3.1)


class FloorWalkTests(unittest.TestCase):
    """D10-13 (2026-09-23): longer policy stage and a delayed velocity command."""

    _WALK = ["ver9_d8_sender.py", "--arm", "--all-axes", "--walk-limits", "--stand-seconds", "15",
             "--policy-slew-dps", "200", "--package", "x", "--ramp-seconds", "10",
             "--csv", "x.csv", "--vx", "0.2"]

    def _main(self, argv):
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
            sender.main()
        return run.call_args.kwargs

    def test_walk_allows_15_seconds_and_a_delay(self):
        kw = self._main(self._WALK + ["--duration", "15", "--command-delay-seconds", "2"])
        self.assertEqual(kw["command_delay_s"], 2.0)
        self.assertEqual(kw["current_limits"], sender.walk_current_limits())

    def test_refusals(self):
        floor = [a if a != "--walk-limits" else "--floor-limits" for a in self._WALK]
        floor[floor.index("200")] = "60"
        bad = [
            self._WALK + ["--duration", "16"],
            floor + ["--duration", "10"],
            floor + ["--duration", "5", "--command-delay-seconds", "2"],
            self._WALK + ["--duration", "10", "--command-delay-seconds", "6"],
            self._WALK + ["--duration", "2", "--command-delay-seconds", "2"],
        ]
        for argv in bad:
            with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                    patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()
        self.assertIsNone(self._main(floor + ["--duration", "5"])["command_delay_s"])

    def test_run_sends_zero_then_the_command(self):
        import contextlib
        seen = []
        first = SimpleNamespace(joint_target_h_order=sender.STAND_TARGET, action_raw=np.zeros(10))

        def fake_eval(policy, t265, feedback, command, last):
            seen.append((command.vx, command.vy, command.wz) if hasattr(command, "vx")
                        else tuple(getattr(command, f) for f in command._fields[1:4]))
            return None, None, first, None
        with tempfile.TemporaryDirectory() as directory:
            buffer = io.StringIO()
            with patch.object(sender, "HPolicy"), \
                    patch.object(sender, "DualBus", AllAxesSlewTests._fake_bus(None, [])), \
                    patch.object(sender, "RealT265", AllAxesSlewTests._FakeT265), \
                    patch.object(sender, "evaluate_cycle", side_effect=fake_eval), \
                    contextlib.redirect_stdout(buffer):
                sender.run(Path(directory), 0.3, Path(directory) / "w.csv", 0.2, 0.0, 0.0,
                           transmit=True, ramp_seconds=0.05, motor_ids=sender.H_CAN_IDS,
                           current_limits=sender.walk_current_limits(),
                           policy_slew_rad_s=np.deg2rad(200.0), stand_seconds=0.05,
                           speed_abort_rad_s=np.deg2rad(200.0), command_delay_s=0.15)
        policy_calls = seen[1:]           # the first call is the pre-arm evaluation
        self.assertTrue(policy_calls)
        self.assertEqual(policy_calls[0][0], 0.0)
        self.assertEqual(policy_calls[-1][0], 0.2)
        self.assertIn("COMMAND DELAY", buffer.getvalue())
        self.assertIn("COMMAND: vx=+0.20", buffer.getvalue())


class SoftStopTests(unittest.TestCase):
    """D10-13B (2026-09-23): raised walk speed abort and a soft stop after an abort."""

    _WALK = FloorWalkTests._WALK + ["--duration", "5"]

    def _main(self, argv):
        out = io.StringIO()
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stderr", io.StringIO()), patch("sys.stdout", out):
            sender.main()
        return run.call_args.kwargs, out.getvalue()

    def test_defaults_are_unchanged(self):
        kw, text = self._main(self._WALK)
        self.assertAlmostEqual(kw["speed_abort_rad_s"], np.deg2rad(200.0))
        self.assertIsNone(kw["soft_stop_s"])
        self.assertNotIn("SOFT STOP armed", text)

    def test_flags_reach_run(self):
        kw, text = self._main(self._WALK + ["--walk-speed-abort-dps", "400", "--soft-stop-seconds", "3"])
        self.assertAlmostEqual(kw["speed_abort_rad_s"], np.deg2rad(400.0))
        self.assertEqual(kw["soft_stop_s"], 3.0)
        self.assertIn("speed abort 400deg/s", text)
        self.assertIn("SOFT STOP armed", text)

    def test_refusals(self):
        floor = [a if a != "--walk-limits" else "--floor-limits" for a in self._WALK]
        floor[floor.index("200")] = "60"
        bad = [
            self._WALK + ["--walk-speed-abort-dps", "401"],
            self._WALK + ["--walk-speed-abort-dps", "199"],
            self._WALK + ["--soft-stop-seconds", "0"],
            self._WALK + ["--soft-stop-seconds", "5.1"],
            floor + ["--walk-speed-abort-dps", "300"],
            floor + ["--soft-stop-seconds", "3"],
        ]
        for argv in bad:
            with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                    patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()

    def test_which_aborts_get_a_soft_stop(self):
        self.assertTrue(sender.is_soft_stop_abort(RuntimeError("motion/current abort 0x1A ch=0: x")))
        self.assertTrue(sender.is_soft_stop_abort(RuntimeError("origin/pre-arm pose abort 0x2A ch=1: x")))
        self.assertFalse(sender.is_soft_stop_abort(RuntimeError("stale feedback 0x1A ch=0")))
        self.assertFalse(sender.is_soft_stop_abort(RuntimeError("motor error 0x1A ch=0: 3")))

    def _abort_run(self, soft_stop_s, bus_cls=None):
        import contextlib
        import csv as _csv
        calls = []
        first = SimpleNamespace(joint_target_h_order=sender.STAND_TARGET, action_raw=np.zeros(10))

        def fake_eval(*_a, **_k):
            calls.append(1)
            if len(calls) >= 3:
                raise RuntimeError("motion/current abort 0x1A ch=0: cur=+0.1A, speed=+250.0deg/s")
            return None, None, first, None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "s.csv"
            buffer = io.StringIO()
            with patch.object(sender, "HPolicy"), \
                    patch.object(sender, "DualBus", bus_cls or AllAxesSlewTests._fake_bus(None, [])), \
                    patch.object(sender, "RealT265", AllAxesSlewTests._FakeT265), \
                    patch.object(sender, "evaluate_cycle", side_effect=fake_eval), \
                    contextlib.redirect_stdout(buffer):
                with self.assertRaises(RuntimeError):
                    sender.run(Path(directory), 1.0, path, 0.0, 0.0, 0.0,
                               transmit=True, ramp_seconds=0.05, motor_ids=sender.H_CAN_IDS,
                               current_limits=sender.walk_current_limits(),
                               policy_slew_rad_s=np.deg2rad(200.0), stand_seconds=0.05,
                               speed_abort_rad_s=np.deg2rad(400.0), soft_stop_s=soft_stop_s)
            with path.open(encoding="utf-8") as handle:
                rows = list(_csv.DictReader(handle))
            abort = sender.abort_txt_path(path).read_text(encoding="utf-8")
        return rows, abort, buffer.getvalue()

    def test_abort_is_recorded_and_soft_stop_holds_then_zero(self):
        rows, abort, text = self._abort_run(0.1)
        self.assertIn("motion/current abort", abort)
        soft = [r for r in rows if r["stage"] == "soft-stop"]
        self.assertTrue(soft)
        self.assertEqual(len(soft) % len(sender.H_CAN_IDS), 0)
        self.assertIn("SOFT STOP complete", text)

    def test_without_the_flag_there_is_no_soft_stop_but_the_abort_is_recorded(self):
        rows, abort, text = self._abort_run(None)
        self.assertIn("motion/current abort", abort)
        self.assertFalse([r for r in rows if r["stage"] == "soft-stop"])
        self.assertNotIn("SOFT STOP", text)

    def test_soft_stop_ends_on_high_current(self):
        base = AllAxesSlewTests._fake_bus(None, [])

        class HotBus(base):
            def state(self, _mid):
                return SimpleNamespace(pos=0.0, spd=0.0, cur=20.0, err=0,
                                       t=__import__("time").time())
        rows, _abort, text = self._abort_run(0.5, HotBus)
        self.assertIn("SOFT STOP ended early", text)
        self.assertFalse([r for r in rows if r["stage"] == "soft-stop"])


class StartGateTests(unittest.TestCase):
    """D10-13C (2026-09-23): start the policy only upright, loop timing, CSV names."""

    _WALK = FloorWalkTests._WALK + ["--duration", "5"]

    class _TiltT265:
        gravity = (0.0, 0.0, -1.0)

        def __init__(self, _offset):
            pass

        def start(self):
            return None

        def latest(self):
            return SimpleNamespace(projected_gravity=self.gravity, confidence=3,
                                   acquired_monotonic_s=__import__("time").monotonic())

        def close(self):
            return None

    def _main(self, argv):
        with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
            sender.main()
        return run.call_args.kwargs

    def test_flags_reach_run_and_defaults_are_off(self):
        kw = self._main(self._WALK)
        self.assertIsNone(kw["start_gate_rad"])
        self.assertFalse(kw["fine_timer"])
        kw = self._main(self._WALK + ["--start-gate-deg", "5", "--fine-timer"])
        self.assertAlmostEqual(kw["start_gate_rad"], np.deg2rad(5.0))
        self.assertTrue(kw["fine_timer"])

    def test_refusals(self):
        floor = [a if a != "--walk-limits" else "--floor-limits" for a in self._WALK]
        floor[floor.index("200")] = "60"
        for argv in (self._WALK + ["--start-gate-deg", "1"], self._WALK + ["--start-gate-deg", "11"],
                     floor + ["--start-gate-deg", "5"]):
            with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                    patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
                with self.assertRaises(SystemExit):
                    sender.main()
            run.assert_not_called()

    def test_tilt_sign_forward_is_positive(self):
        pitch, roll = sender.tilt_deg(SimpleNamespace(projected_gravity=(0.1, 0.0, -0.995)))
        self.assertGreater(pitch, 5.0)
        self.assertAlmostEqual(roll, 0.0)
        self.assertEqual(sender.tilt_deg(object()), (None, None))

    def test_unique_csv_path_never_reuses_a_name(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "r.csv"
            self.assertEqual(sender.unique_csv_path(path), path)
            sender.obs_csv_path(path).write_text("x")          # only the sidecar survived
            self.assertEqual(sender.unique_csv_path(path).name, "r_2.csv")
            (Path(directory) / "r_2.csv").write_text("x")
            self.assertEqual(sender.unique_csv_path(path).name, "r_3.csv")

    def test_main_renames_a_used_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            used = Path(directory) / "w.csv"
            used.write_text("x")
            argv = [a if a != "x.csv" else str(used) for a in self._WALK]
            kw_args = None
            with patch.object(sys, "argv", argv), patch.object(sender, "run") as run, \
                    patch("sys.stdout", io.StringIO()):
                sender.main()
                kw_args = run.call_args.args
            self.assertEqual(Path(kw_args[2]).name, "w_2.csv")

    def test_fine_timer_is_undone(self):
        import time as _time
        original = _time.monotonic
        with patch("sys.stdout", io.StringIO()):
            state = sender.enable_fine_timer()
            self.assertIs(_time.monotonic, _time.perf_counter)
            sender.disable_fine_timer(state)
        self.assertIs(_time.monotonic, original)

    def _gate_run(self, gravity, timeout=None):
        import contextlib
        import csv as _csv
        first = SimpleNamespace(joint_target_h_order=sender.STAND_TARGET, action_raw=np.zeros(10))
        t265 = type("T", (self._TiltT265,), {"gravity": gravity})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "g.csv"
            buffer = io.StringIO()
            patches = [patch.object(sender, "HPolicy"),
                       patch.object(sender, "DualBus", AllAxesSlewTests._fake_bus(None, [])),
                       patch.object(sender, "RealT265", t265),
                       patch.object(sender, "evaluate_cycle",
                                    side_effect=lambda *a, **k: (None, None, first, None)),
                       patch.object(sender, "START_GATE_HOLD_S", 0.1)]
            if timeout is not None:
                patches.append(patch.object(sender, "START_GATE_TIMEOUT_S", timeout))
            with contextlib.ExitStack() as stack:
                for item in patches:
                    stack.enter_context(item)
                stack.enter_context(contextlib.redirect_stdout(buffer))
                sender.run(Path(directory), 0.2, path, 0.0, 0.0, 0.0,
                           transmit=True, ramp_seconds=0.05, motor_ids=sender.H_CAN_IDS,
                           current_limits=sender.walk_current_limits(),
                           policy_slew_rad_s=np.deg2rad(200.0), stand_seconds=0.05,
                           speed_abort_rad_s=np.deg2rad(400.0),
                           start_gate_rad=np.deg2rad(5.0), fine_timer=True)
            with path.open(encoding="utf-8") as handle:
                rows = list(_csv.DictReader(handle))
            with sender.timing_csv_path(path).open(encoding="utf-8") as handle:
                timing = list(_csv.DictReader(handle))
            abort = sender.abort_txt_path(path)
            abort_text = abort.read_text(encoding="utf-8") if abort.exists() else ""
        return rows, timing, abort_text, buffer.getvalue()

    def test_upright_opens_the_gate(self):
        rows, timing, abort, text = self._gate_run((0.0, 0.0, -1.0))
        self.assertIn("START GATE open", text)
        self.assertTrue([r for r in rows if r["stage"] == "policy"])
        self.assertTrue([r for r in timing if r["stage"] == "stand-gate"])
        self.assertTrue([r for r in timing if r["stage"] == "policy"])
        self.assertIn("LOOP TIMING (policy)", text)
        self.assertEqual(abort, "")

    def test_leaning_back_times_out_without_a_policy(self):
        rows, _timing, abort, text = self._gate_run((-0.4, 0.0, -0.92), timeout=0.3)
        self.assertIn("START GATE timeout", text)
        self.assertIn("BACK", text)
        self.assertFalse([r for r in rows if r["stage"] == "policy"])
        self.assertIn("START GATE timeout", abort)


class ZeroOffsetTests(unittest.TestCase):
    """D10-13D (2026-09-23): D7 origin (straight legs) -> sim joint zero (CAD pose)."""

    def setUp(self):
        self.addCleanup(sender.set_zero_offset, None)

    def test_default_is_zero(self):
        sender.set_zero_offset(None)
        self.assertTrue(np.all(sender.ZERO_OFFSET_RAD == 0.0))

    def test_targets_and_feedback_cross_the_offset(self):
        sender.set_zero_offset("cad_fk")
        kfe = sender.H_CAN_IDS[sender.OBS_JOINT_NAMES.index("LL_KFE")] if hasattr(sender, "OBS_JOINT_NAMES") else 0x1A
        self.assertEqual(kfe, 0x1A)
        _kp, _kd, wire, _v, _t = sender.wire_command(0x1A, np.deg2rad(20.0))
        # LL_KFE sign +1: sim 20 deg = 32.3 deg from the straight D7 origin.
        self.assertAlmostEqual(np.rad2deg(wire), 32.3, delta=0.1)
        state = SimpleNamespace(pos=32.3, spd=0.0, cur=0.0)
        position, _v, _c = sender.h_feedback(0x1A, state)
        self.assertAlmostEqual(np.rad2deg(position), 20.0, places=3)
        frame_targets = list(sender.STAND_TARGET)
        frames = sender.frames(frame_targets, (0x1A,))
        self.assertEqual(len(frames), 1)

    def test_right_leg_sign_is_respected(self):
        sender.set_zero_offset("cad_fk")
        # LR_KFE (0x12) has joint sign -1: sim 20 deg = 27.2 deg straight-origin = motor -27.2.
        _kp, _kd, wire, _v, _t = sender.wire_command(0x12, np.deg2rad(20.0))
        self.assertAlmostEqual(np.rad2deg(wire), -27.2, delta=0.1)
        position, _v, _c = sender.h_feedback(0x12, SimpleNamespace(pos=-27.2, spd=0.0, cur=0.0))
        self.assertAlmostEqual(np.rad2deg(position), 20.0, places=3)

    def test_flag(self):
        argv = FloorWalkTests._WALK + ["--duration", "5", "--zero-offset", "cad_fk"]
        out = io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            argv = [a if a != "x.csv" else str(Path(directory) / "z.csv") for a in argv]
            with patch.object(sys, "argv", argv), patch.object(sender, "run"), patch("sys.stdout", out):
                sender.main()
            meta = (Path(directory) / "z_meta.txt").read_text(encoding="utf-8")
        self.assertIn("ZERO OFFSET cad_fk", out.getvalue())
        self.assertIn("LL_KFE=-12.3", meta)
        bad = [a for a in argv if a != "--all-axes"]
        with patch.object(sys, "argv", bad), patch.object(sender, "run") as run, \
                patch("sys.stderr", io.StringIO()), patch("sys.stdout", io.StringIO()):
            with self.assertRaises(SystemExit):
                sender.main()
        run.assert_not_called()

    def test_stand_ramp_allows_45_deg_but_policy_gate_keeps_30(self):
        target = list(sender.STAND_TARGET)
        start = list(target)
        start[sender.H_CAN_IDS.index(0x1A)] = target[sender.H_CAN_IDS.index(0x1A)] - np.deg2rad(40)
        self.assertTrue(sender.all_axes_prearm_violations(target, start))
        self.assertFalse(sender.all_axes_prearm_violations(target, start, sender.STAND_RAMP_MAX_DELTA_DEG))

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("ver9_integration.py")
SPEC = importlib.util.spec_from_file_location("ver9_integration", MODULE_PATH)
integration = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = integration
SPEC.loader.exec_module(integration)


class FakePolicy:
    def evaluate(self, snapshot):
        action = np.arange(10, dtype=np.float32)
        return integration.PolicyOutput(action, integration.target_from_action(action))


class FakeT265:
    def __init__(self, sample):
        self.sample = sample
        self.started = self.closed = False

    def start(self):
        self.started = True

    def latest(self):
        return self.sample

    def close(self):
        self.closed = True


class FakeCan:
    tx_count = 0

    def __init__(self, motors):
        self.motors = motors
        self.started = self.closed = False

    def start(self):
        self.started = True

    def latest(self, ids):
        return {can_id: self.motors[can_id] for can_id in ids}

    def close(self):
        self.closed = True


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        now = 10.0
        self.t265 = integration.T265Sample(now, (1, 2, 3), (4, 5, 6), (0, 0, -1), 3, False)
        self.motors = {
            can_id: integration.MotorFeedback(can_id, now, float(index), float(index + 10))
            for index, can_id in enumerate(integration.H_CAN_IDS)
        }
        self.command = integration.VelocityCommand(now, 0.1, -0.2, 0.3, "fixed", "fixed")

    def test_exact_observation_layout_and_d4_can_order(self):
        _snapshot, observation, output, plan = integration.evaluate_cycle(
            FakePolicy(), self.t265, self.motors, self.command, np.zeros(10),
        )
        np.testing.assert_array_equal(observation[:12], np.array([1, 2, 3, 4, 5, 6, 0, 0, -1, .1, -.2, .3], dtype=np.float32))
        np.testing.assert_array_equal(observation[12:22], np.arange(10, dtype=np.float32))
        np.testing.assert_array_equal(observation[22:32], np.arange(10, 20, dtype=np.float32))
        self.assertEqual(tuple(target.can_id for target in plan), integration.H_CAN_IDS)
        self.assertEqual(tuple(target.model for target in plan), integration.H_MODELS)
        np.testing.assert_allclose([target.position_rad for target in plan], output.joint_target_h_order, rtol=0, atol=0)

    def test_user_verified_left_right_can_mapping(self):
        self.assertEqual(integration.H_CAN_IDS, (0x1C, 0x13, 0x11, 0x1B, 0x21, 0x2A, 0x1A, 0x12, 0x2B, 0x22))
        self.assertEqual(integration.LEFT_CAN_IDS, (0x1C, 0x11, 0x21, 0x1A, 0x2B))
        self.assertEqual(integration.RIGHT_CAN_IDS, (0x13, 0x1B, 0x2A, 0x12, 0x22))

    def test_missing_feedback_is_rejected(self):
        self.motors.pop(integration.H_CAN_IDS[3])
        with self.assertRaisesRegex(ValueError, "0x1B"):
            integration.policy_snapshot_from_inputs(self.t265, self.motors, self.command, np.zeros(10))

    def test_t265_offset_transform_is_applied(self):
        rotation = np.eye(3)
        velocity, angular, gravity = integration.transform_t265_world_to_base(
            rotation, np.zeros(3), np.array([0.0, -1.0, 0.0]), np.array([0.2, 0.0, 0.0]),
        )
        np.testing.assert_allclose(angular, np.array([0.0, 0.0, -1.0]))
        np.testing.assert_allclose(velocity, np.array([0.0, 0.2, 0.0]))
        np.testing.assert_allclose(gravity, np.array([0.0, 0.0, -1.0]))

    def test_live_dry_combines_both_channels_without_transmit(self):
        left = FakeCan(self.motors)
        right = FakeCan(self.motors)
        t265 = FakeT265(self.t265)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "live_dry.csv"
            summary = integration.run_live_dry(
                FakePolicy(), t265, left, right,
                integration.FixedCommandSource(0.0, 0.0, 0.0), 0.05, output,
            )
            self.assertGreaterEqual(summary["ticks"], 3)
            self.assertEqual(summary["can_tx_count"], 0)
            self.assertTrue(t265.started and t265.closed)
            self.assertTrue(left.started and left.closed)
            self.assertTrue(right.started and right.closed)
            with output.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), summary["ticks"])
            self.assertTrue(all(row["can_id_0"] == "0x1C" for row in rows))
            self.assertTrue(all(row["can_id_9"] == "0x22" for row in rows))

    def test_live_dry_source_has_no_can_transmit_call(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn(".send(", source)
        self.assertNotIn("stop_all(", source)


if __name__ == "__main__":
    unittest.main()

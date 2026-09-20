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

    def test_missing_feedback_is_rejected(self):
        self.motors.pop(integration.H_CAN_IDS[3])
        with self.assertRaisesRegex(ValueError, "0x11"):
            integration.policy_snapshot_from_inputs(self.t265, self.motors, self.command, np.zeros(10))

    def test_t265_offset_transform_is_applied(self):
        rotation = np.eye(3)
        velocity, angular, gravity = integration.transform_t265_world_to_base(
            rotation, np.zeros(3), np.array([0.0, -1.0, 0.0]), np.array([0.2, 0.0, 0.0]),
        )
        np.testing.assert_allclose(angular, np.array([0.0, 0.0, -1.0]))
        np.testing.assert_allclose(velocity, np.array([0.0, 0.2, 0.0]))
        np.testing.assert_allclose(gravity, np.array([0.0, 0.0, -1.0]))


if __name__ == "__main__":
    unittest.main()

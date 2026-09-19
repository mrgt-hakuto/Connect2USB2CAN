import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("policy_integration.py")
SPEC = importlib.util.spec_from_file_location("policy_integration", MODULE_PATH)
policy_integration = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = policy_integration
SPEC.loader.exec_module(policy_integration)


class PolicyIntegrationTests(unittest.TestCase):
    def test_observation_contract_layout(self):
        values = np.arange(42, dtype=np.float32)
        snapshot = policy_integration.snapshot_from_observation(values)
        np.testing.assert_array_equal(policy_integration.build_observation(snapshot), values)

    def test_target_uses_contract_default_and_scale(self):
        action = np.ones(10, dtype=np.float32)
        target = policy_integration.target_from_action(action)
        np.testing.assert_allclose(target, policy_integration.DEFAULT_JOINT_POS + 0.5, rtol=0, atol=0)

    def test_joint_observation_rejects_non_h_order_size(self):
        with self.assertRaisesRegex(ValueError, "joint position"):
            policy_integration.JointObservation(np.zeros(9), np.zeros(10))

    def test_velocity_command_rejects_non_finite_value(self):
        base = policy_integration.BaseObservation(np.zeros(3), np.zeros(3), np.array([0, 0, -1]))
        joints = policy_integration.JointObservation(np.zeros(10), np.zeros(10))
        with self.assertRaisesRegex(ValueError, "velocity_command contains"):
            policy_integration.PolicySnapshot(base, joints, np.array([np.nan, 0, 0]), np.zeros(10))


if __name__ == "__main__":
    unittest.main()

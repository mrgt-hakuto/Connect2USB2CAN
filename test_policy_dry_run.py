import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("policy_dry_run.py")
SPEC = importlib.util.spec_from_file_location("policy_dry_run", MODULE_PATH)
policy_dry_run = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(policy_dry_run)


class PolicyDryRunTests(unittest.TestCase):
    def test_manifest_requires_all_six_expected_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            package = Path(temporary)
            (package / "SHA256SUMS.txt").write_text("0" * 64 + "  policy.onnx\n", encoding="utf-8")
            with self.assertRaisesRegex(policy_dry_run.DryRunError, "six required files"):
                policy_dry_run.verify_manifest(package)

    def test_golden_rejects_wrong_observation_dtype(self):
        with tempfile.TemporaryDirectory() as temporary:
            package = Path(temporary)
            np.savez(
                package / "golden.npz",
                obs_flat=np.zeros((500, 42), dtype=np.float64),
                action_raw=np.zeros((500, 10), dtype=np.float32),
                joint_target=np.tile(policy_dry_run.DEFAULT_JOINT_POS, (500, 1)),
                default_joint_pos=policy_dry_run.DEFAULT_JOINT_POS,
                joint_names=np.array(policy_dry_run.JOINT_NAMES),
                action_scale=np.float64(0.5),
            )
            with self.assertRaisesRegex(policy_dry_run.DryRunError, "obs_flat"):
                policy_dry_run.load_golden(package)

    def test_target_formula_uses_deployment_contract(self):
        actions = np.zeros((500, 10), dtype=np.float32)
        actions[7, 4] = np.float32(0.2)
        expected = np.tile(policy_dry_run.DEFAULT_JOINT_POS, (500, 1)) + 0.5 * actions
        error, step = policy_dry_run.max_error(expected, expected)
        self.assertEqual((error, step), (0.0, 0))
        self.assertAlmostEqual(expected[7, 4], -0.0745, places=6)


if __name__ == "__main__":
    unittest.main()

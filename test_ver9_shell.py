import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("ver9_shell.py")
SPEC = importlib.util.spec_from_file_location("ver9_shell", MODULE_PATH)
ver9_shell = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = ver9_shell
SPEC.loader.exec_module(ver9_shell)


class Ver9ShellTests(unittest.TestCase):
    def test_parse_can_ids_rejects_non_ten_ids(self):
        with self.assertRaises(ver9_shell.argparse.ArgumentTypeError):
            ver9_shell._parse_can_ids("1,2")

    def test_synthetic_loop_logs_timestamps_and_zero_targets(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "d2.csv"
            shell = ver9_shell.Shell(
                ver9_shell.SyntheticT265(),
                ver9_shell.SyntheticCan(tuple(range(1, 11))),
                ver9_shell.FixedCommandSource(0.0, 0.0, 0.0),
                tuple(range(1, 11)),
            )
            summary = shell.run(0.08, path)
            self.assertGreaterEqual(summary["ticks"], 4)
            self.assertEqual(summary["can_tx_count"], 0)
            with path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), summary["ticks"])
            self.assertTrue(all(row["t265_acquired_monotonic_s"] for row in rows))
            self.assertTrue(all(row["missing_can_ids"] == "" for row in rows))
            self.assertTrue(all(float(row[f"observation_placeholder_{index}"]) == 0.0 for row in rows for index in range(42)))
            self.assertTrue(all(float(row[f"planned_target_{index}"]) == 0.0 for row in rows for index in range(10)))

    def test_fixed_command_rejects_out_of_range_value(self):
        with self.assertRaisesRegex(ValueError, r"\[-1, 1\]"):
            ver9_shell.FixedCommandSource(1.1, 0.0, 0.0)

    def test_confirmed_d3_offset_is_the_default_transform_input(self):
        np.testing.assert_array_equal(
            ver9_shell.T265_R_OFFSET_M,
            np.array([0.06345, 0.08900, 0.04275]),
        )
        velocity, angular, _gravity = ver9_shell.transform_t265_world_to_base(
            np.eye(3),
            np.zeros(3),
            np.array([0.0, -1.0, 0.0]),
            np.asarray(ver9_shell.T265_R_OFFSET_M),
        )
        np.testing.assert_allclose(angular, np.array([0.0, 0.0, -1.0]))
        np.testing.assert_allclose(velocity, np.array([-0.08900, 0.06345, 0.0]))

    def test_no_can_transmit_symbol_in_d2_source(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn(".send(", source)
        self.assertNotIn("stop_all(", source)


if __name__ == "__main__":
    unittest.main()

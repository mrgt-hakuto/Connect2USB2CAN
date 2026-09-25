"""Unit tests for D5's pure command-source boundary; no hardware required."""

import unittest

from command_source import controller_command, fixed_command


class CommandSourceTest(unittest.TestCase):
    def test_fixed_command_preserves_values(self) -> None:
        got = fixed_command(timestamp_s=10.0, vx=0.25, vy=-0.5)
        self.assertEqual((got.vx, got.vy, got.wz, got.state), (0.25, -0.5, 0.0, "fixed"))

    def test_zero_controller_input_is_zero_when_mapping_unconfirmed(self) -> None:
        got = controller_command(timestamp_s=10.0, x_norm=0.0, y_norm=0.0, mapping_confirmed=False)
        self.assertEqual((got.vx, got.vy, got.wz, got.state), (0.0, 0.0, 0.0, "mapping_unconfirmed"))

    def test_nonzero_controller_input_stays_zero_when_mapping_unconfirmed(self) -> None:
        got = controller_command(timestamp_s=10.0, x_norm=0.8, y_norm=-0.6, mapping_confirmed=False)
        self.assertEqual((got.vx, got.vy, got.wz, got.state), (0.0, 0.0, 0.0, "mapping_unconfirmed"))

    def test_out_of_range_fixed_value_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            fixed_command(timestamp_s=10.0, vx=1.01, vy=0.0)

    def test_nonzero_wz_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            fixed_command(timestamp_s=10.0, vx=0.0, vy=0.0, wz=0.1)


if __name__ == "__main__":
    unittest.main()

import csv
import importlib.util
import sys
import tempfile
import unittest
from unittest.mock import patch
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

    def test_servo_feedback_units_convert_to_h_radians(self):
        position, velocity = ver9_shell.servo_feedback_to_h_units(180.0, 31.5, 0x1C)
        self.assertAlmostEqual(position, np.pi)
        self.assertAlmostEqual(velocity, np.pi / 180.0)

    def test_servo_feedback_applies_the_joint_sign(self):
        # 0x2A LR_HFE is sign -1 (D10-6): motor +180 deg is H -pi.
        position, velocity = ver9_shell.servo_feedback_to_h_units(180.0, 31.5, 0x2A)
        self.assertAlmostEqual(position, -np.pi)
        self.assertAlmostEqual(velocity, -np.pi / 180.0)

    def test_real_can_latest_reports_h_frame(self):
        can = ver9_shell.RealCan(0, 1000000)
        states = {0x2A: type("S", (), {"pos": 20.0, "spd": 0.0})(),
                  0x21: type("S", (), {"pos": 20.0, "spd": 0.0})()}
        can._bus = type("B", (), {"rx_error": None, "tx_count": 0,
                                  "state": lambda self, mid: states.get(mid)})()
        out = can.latest((0x2A, 0x21))
        self.assertAlmostEqual(np.rad2deg(out[0x2A].position), -20.0)
        self.assertAlmostEqual(np.rad2deg(out[0x21].position), 20.0)

    def test_servo_feedback_refuses_without_a_motor_id(self):
        with self.assertRaises(TypeError):
            ver9_shell.servo_feedback_to_h_units(180.0, 31.5)

    def test_no_can_transmit_symbol_in_d2_source(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn(".send(", source)
        self.assertNotIn("stop_all(", source)

    def test_real_t265_releases_enumeration_context_before_pipeline_start(self):
        lifecycle = {"context_released": False}

        class Device:
            def get_info(self, _info):
                return "15322110478"

        class Context:
            def query_devices(self):
                return [Device()]

            def __del__(self):
                lifecycle["context_released"] = True

        class Config:
            def enable_device(self, _serial):
                return None

            def enable_stream(self, _stream):
                return None

        class Pipeline:
            def start(self, _config):
                if not lifecycle["context_released"]:
                    raise RuntimeError("No device connected")

            def wait_for_frames(self, _timeout_ms):
                raise RuntimeError("test receiver exit")

            def stop(self):
                return None

        class FakeRs:
            camera_info = type("CameraInfo", (), {"serial_number": object()})
            stream = type("Stream", (), {"pose": object()})
            context = Context
            pipeline = Pipeline
            config = Config

        with patch.dict(sys.modules, {"pyrealsense2": FakeRs}):
            source = ver9_shell.RealT265((0.0, 0.0, 0.0))
            source.start()
            source.close()
        self.assertTrue(lifecycle["context_released"])

    def test_real_t265_receiver_keeps_waiting_after_frame_timeout(self):
        source = ver9_shell.RealT265((0.0, 0.0, 0.0))

        class TimeoutPipe:
            def wait_for_frames(self, _timeout_ms):
                source._stop.set()
                raise RuntimeError("Frame didn't arrive within 100")

        source._pipe = TimeoutPipe()
        source._receive_loop()
        self.assertIsNone(source._error)


if __name__ == "__main__":
    unittest.main()

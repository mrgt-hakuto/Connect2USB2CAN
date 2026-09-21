import importlib.util
import sys
import unittest
from pathlib import Path

path = Path(__file__).with_name("ver9_jetson_deploy.py")
spec = importlib.util.spec_from_file_location("ver9_jetson_deploy", path)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = module
spec.loader.exec_module(module)

class JetsonMappingTests(unittest.TestCase):
    def test_stick_and_turn_mapping(self):
        self.assertEqual(module.map_command(0.0, -1.0, False, False, .3, .2, .3), (.3, 0.0, 0.0))
        self.assertEqual(module.map_command(1.0, 0.0, True, False, .3, .2, .3), (0.0, -.2, .3))
        self.assertEqual(module.map_command(0.0, 0.0, True, True, .3, .2, .3), (0.0, 0.0, 0.0))
    def test_normalize_deadzone(self):
        self.assertEqual(module.normalize(0, -32768, 32767, .05), 0.0)
        self.assertGreater(module.normalize(32767, -32768, 32767, .05), .99)

if __name__ == "__main__": unittest.main()

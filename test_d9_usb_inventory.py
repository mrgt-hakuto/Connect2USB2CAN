import unittest
from pathlib import Path


class UsbInventorySafetyTests(unittest.TestCase):
    def test_inventory_does_not_start_or_transmit_can(self):
        source = Path(__file__).with_name("d9_usb_inventory.py").read_text(encoding="utf-8")
        self.assertNotIn("can.Bus", source)
        self.assertNotIn(".start(", source)
        self.assertNotIn(".send(", source)


if __name__ == "__main__":
    unittest.main()

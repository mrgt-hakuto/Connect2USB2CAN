import unittest

import robot_joint_map as mapping
import ver9_integration as integration
import d7_origin_console as d7


class RobotJointMapTests(unittest.TestCase):
    def test_user_verified_channels_and_joint_identity(self):
        self.assertEqual(mapping.LEFT_CAN_IDS, (0x1C, 0x11, 0x21, 0x1A, 0x2B))
        self.assertEqual(mapping.RIGHT_CAN_IDS, (0x13, 0x1B, 0x2A, 0x12, 0x22))
        self.assertEqual(mapping.BY_ID[0x2A].name, "LR_HFE")

    def test_d10_6_joint_signs(self):
        # D10-6 (2026-09-23): hand-moved, receive only, compared with the URDF.
        minus = {name for name, joint in mapping.BY_NAME.items() if joint.sign == -1}
        self.assertEqual(minus, {"LR_HFE", "LR_KFE", "LL_FFE", "LL_HAA", "LR_HR"})
        # Every left/right pair has exactly one flipped side.
        for suffix in ("HR", "HAA", "HFE", "KFE", "FFE"):
            pair = mapping.BY_NAME["LL_" + suffix].sign * mapping.BY_NAME["LR_" + suffix].sign
            self.assertEqual(pair, -1, suffix)

    def test_sign_round_trip(self):
        for mid in mapping.H_CAN_IDS:
            self.assertAlmostEqual(mapping.motor_to_h(mid, mapping.h_to_motor(mid, 0.3)), 0.3)
        self.assertAlmostEqual(mapping.h_to_motor(0x2A, 0.3), -0.3)
        self.assertAlmostEqual(mapping.h_to_motor(0x21, 0.3), 0.3)

    def test_ver9_uses_the_shared_mapping(self):
        self.assertIs(integration.H_CAN_IDS, mapping.H_CAN_IDS)
        self.assertIs(integration.LEFT_CAN_IDS, mapping.LEFT_CAN_IDS)
        self.assertIs(integration.RIGHT_CAN_IDS, mapping.RIGHT_CAN_IDS)

    def test_d7_console_uses_the_shared_mapping(self):
        self.assertIs(d7.JOINTS, mapping.JOINTS)
        self.assertIs(d7.BY_ID, mapping.BY_ID)


if __name__ == "__main__":
    unittest.main()

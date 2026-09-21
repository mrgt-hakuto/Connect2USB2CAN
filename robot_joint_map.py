"""Single source of truth for the user-verified 10-joint CAN mapping."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class JointBinding:
    name: str
    motor_id: int
    channel: int
    model: str


# Verified by the user on 2026-09-21.  The older left/right mapping was
# completely reversed and must not be reused.
JOINTS = (
    JointBinding("LL_HR",  0x1C, 0, "AK10-9"),
    JointBinding("LL_HAA", 0x11, 0, "AK10-9"),
    JointBinding("LL_HFE", 0x21, 0, "AK80-9"),
    JointBinding("LL_KFE", 0x1A, 0, "AK10-9"),
    JointBinding("LL_FFE", 0x2B, 0, "AK80-9"),
    JointBinding("LR_HR",  0x13, 1, "AK10-9"),
    JointBinding("LR_HAA", 0x1B, 1, "AK10-9"),
    JointBinding("LR_HFE", 0x2A, 1, "AK80-9"),
    JointBinding("LR_KFE", 0x12, 1, "AK10-9"),
    JointBinding("LR_FFE", 0x22, 1, "AK80-9"),
)

BY_ID = {joint.motor_id: joint for joint in JOINTS}
BY_NAME = {joint.name: joint for joint in JOINTS}
H_JOINT_NAMES = (
    "LL_HR", "LR_HR", "LL_HAA", "LR_HAA", "LL_HFE", "LR_HFE",
    "LL_KFE", "LR_KFE", "LL_FFE", "LR_FFE",
)
H_CAN_IDS = tuple(BY_NAME[name].motor_id for name in H_JOINT_NAMES)
H_MODELS = tuple(BY_NAME[name].model for name in H_JOINT_NAMES)
LEFT_CAN_IDS = tuple(joint.motor_id for joint in JOINTS if joint.channel == 0)
RIGHT_CAN_IDS = tuple(joint.motor_id for joint in JOINTS if joint.channel == 1)

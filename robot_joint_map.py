"""Single source of truth for the user-verified 10-joint CAN mapping."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class JointBinding:
    name: str
    motor_id: int
    model: str


# Verified by the user on 2026-09-21.  The older left/right mapping was
# completely reversed and must not be reused.
JOINTS = (
    # gs_usb can swap its Python channel 0/1 enumeration after a reset.
    # Physical channel routing is therefore discovered afresh by reception;
    # this map deliberately contains only stable joint identity.
    JointBinding("LL_HR",  0x1C, "AK10-9"),
    JointBinding("LL_HAA", 0x11, "AK10-9"),
    JointBinding("LL_HFE", 0x21, "AK80-9"),
    JointBinding("LL_KFE", 0x1A, "AK10-9"),
    JointBinding("LL_FFE", 0x2B, "AK80-9"),
    JointBinding("LR_HR",  0x13, "AK10-9"),
    JointBinding("LR_HAA", 0x1B, "AK10-9"),
    JointBinding("LR_HFE", 0x2A, "AK80-9"),
    JointBinding("LR_KFE", 0x12, "AK10-9"),
    JointBinding("LR_FFE", 0x22, "AK80-9"),
)

BY_ID = {joint.motor_id: joint for joint in JOINTS}
BY_NAME = {joint.name: joint for joint in JOINTS}
H_JOINT_NAMES = (
    "LL_HR", "LR_HR", "LL_HAA", "LR_HAA", "LL_HFE", "LR_HFE",
    "LL_KFE", "LR_KFE", "LL_FFE", "LR_FFE",
)
H_CAN_IDS = tuple(BY_NAME[name].motor_id for name in H_JOINT_NAMES)
H_MODELS = tuple(BY_NAME[name].model for name in H_JOINT_NAMES)
# These retain their robot-side meaning; they must not be derived from USB
# channel numbers, which are a separate physical routing concern.
LEFT_CAN_IDS = tuple(BY_NAME[name].motor_id for name in H_JOINT_NAMES if name.startswith("LL_"))
RIGHT_CAN_IDS = tuple(BY_NAME[name].motor_id for name in H_JOINT_NAMES if name.startswith("LR_"))

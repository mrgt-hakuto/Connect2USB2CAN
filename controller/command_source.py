"""Dry-run command sources for ver9's (vx, vy, wz) policy input.

This module deliberately has no CAN, motor, ONNX, T265, or controller-device
imports.  A device reader may pass normalized left-stick values here later.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Literal


CommandState = Literal["fixed", "mapping_unconfirmed", "mapped"]
CommandSource = Literal["fixed", "controller"]


@dataclass(frozen=True)
class CommandRecord:
    """One command sample; values use the policy's unit-unconfirmed [-1, 1] range."""

    source: CommandSource
    timestamp_s: float
    x_norm: float | None
    y_norm: float | None
    vx: float
    vy: float
    wz: float
    state: CommandState
    unit_unconfirmed: bool = True

    def csv_row(self) -> dict[str, object]:
        return asdict(self)


def _require_policy_range(name: str, value: float) -> None:
    if not -1.0 <= value <= 1.0:
        raise ValueError(f"{name} must be in [-1.0, 1.0], got {value}")


def fixed_command(*, timestamp_s: float, vx: float, vy: float, wz: float = 0.0) -> CommandRecord:
    """Return a fixed command without silently clipping values.

    D5 deliberately keeps yaw unavailable, so nonzero ``wz`` is rejected.
    """
    _require_policy_range("vx", vx)
    _require_policy_range("vy", vy)
    _require_policy_range("wz", wz)
    if wz != 0.0:
        raise ValueError("D5 fixes wz at 0.0")
    return CommandRecord("fixed", timestamp_s, None, None, vx, vy, 0.0, "fixed")


def controller_command(
    *,
    timestamp_s: float,
    x_norm: float,
    y_norm: float,
    mapping_confirmed: bool,
    x_to_vy_sign: int = 1,
    y_to_vx_sign: int = 1,
) -> CommandRecord:
    """Map normalized left-stick input only after base-frame mapping is confirmed.

    ``x_to_vy_sign`` and ``y_to_vx_sign`` are intentionally explicit because
    the stick-to-base convention is not yet confirmed.  Until confirmation,
    this function always returns a zero command.
    """
    _require_policy_range("x_norm", x_norm)
    _require_policy_range("y_norm", y_norm)
    if x_to_vy_sign not in (-1, 1) or y_to_vx_sign not in (-1, 1):
        raise ValueError("mapping signs must each be -1 or +1")
    if not mapping_confirmed:
        return CommandRecord(
            "controller", timestamp_s, x_norm, y_norm, 0.0, 0.0, 0.0, "mapping_unconfirmed"
        )
    return CommandRecord(
        "controller",
        timestamp_s,
        x_norm,
        y_norm,
        y_to_vx_sign * y_norm,
        x_to_vy_sign * x_norm,
        0.0,
        "mapped",
    )

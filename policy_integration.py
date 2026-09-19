"""Hardware-free H policy integration.

This module owns the H observation layout and converts an ONNX action into a
target vector in H joint order.  It deliberately has no CAN, T265, controller,
or ``mit_sim`` import: the D3 and D4 adapters must supply verified values at
this boundary before a later, separately safety-reviewed sender can use them.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from policy_dry_run import (
    ACTION_SCALE,
    DEFAULT_JOINT_POS,
    ERROR_THRESHOLD,
    GoldenData,
    JOINT_NAMES,
    DryRunError,
    load_golden,
    validate_session,
    verify_manifest,
)


OBSERVATION_SIZE = 42
ACTION_SIZE = 10


def _vector(value: Sequence[float] | np.ndarray, size: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32)
    if array.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} contains a non-finite value")
    return array


@dataclass(frozen=True)
class BaseObservation:
    """D3 output, already expressed in the H base frame."""

    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    projected_gravity: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "linear_velocity", _vector(self.linear_velocity, 3, "linear_velocity"))
        object.__setattr__(self, "angular_velocity", _vector(self.angular_velocity, 3, "angular_velocity"))
        object.__setattr__(self, "projected_gravity", _vector(self.projected_gravity, 3, "projected_gravity"))


@dataclass(frozen=True)
class JointObservation:
    """D4 output: positions and velocities, both in ``JOINT_NAMES`` order."""

    position: np.ndarray
    velocity: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _vector(self.position, ACTION_SIZE, "joint position"))
        object.__setattr__(self, "velocity", _vector(self.velocity, ACTION_SIZE, "joint velocity"))


@dataclass(frozen=True)
class PolicySnapshot:
    base: BaseObservation
    joints: JointObservation
    velocity_command: np.ndarray
    last_action: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "velocity_command", _vector(self.velocity_command, 3, "velocity_command"))
        object.__setattr__(self, "last_action", _vector(self.last_action, ACTION_SIZE, "last_action"))


@dataclass(frozen=True)
class PolicyOutput:
    """No CAN IDs are included: this is strictly an H-order target vector."""

    action_raw: np.ndarray
    joint_target_h_order: np.ndarray


def build_observation(snapshot: PolicySnapshot) -> np.ndarray:
    """Build H's exact 42-element policy observation in contract order."""
    position = snapshot.joints.position
    result = np.concatenate(
        (
            snapshot.base.linear_velocity,       # [0:3] base_lin_vel
            snapshot.base.angular_velocity,      # [3:6] base_ang_vel
            snapshot.base.projected_gravity,     # [6:9] projected_gravity
            snapshot.velocity_command,           # [9:12] velocity_commands
            position[0:2],                       # [12:14] hip_pos (HR)
            position[2:8],                       # [14:20] kfe_pos (HAA/HFE/KFE)
            position[8:10],                      # [20:22] ffe_pos
            snapshot.joints.velocity,            # [22:32] joint_vel
            snapshot.last_action,                # [32:42] actions
        ),
        dtype=np.float32,
    )
    if result.shape != (OBSERVATION_SIZE,):  # Defensive guard against accidental contract edits.
        raise AssertionError(f"internal observation size is {result.shape}, expected ({OBSERVATION_SIZE},)")
    return result


def target_from_action(action_raw: Sequence[float] | np.ndarray) -> np.ndarray:
    action = _vector(action_raw, ACTION_SIZE, "action_raw")
    return DEFAULT_JOINT_POS + np.float32(ACTION_SCALE) * action


class HPolicy:
    """A verified deployment package evaluated without any hardware access."""

    def __init__(self, package: Path):
        self.package = package.resolve()
        verify_manifest(self.package)
        self._session, self._input_name, self._output_name = validate_session(self.package / "policy.onnx")

    def evaluate(self, snapshot: PolicySnapshot) -> PolicyOutput:
        observation = build_observation(snapshot)
        output = self._session.run(
            [self._output_name],
            {self._input_name: observation.reshape(1, OBSERVATION_SIZE)},
        )[0]
        if output.shape != (1, ACTION_SIZE) or output.dtype != np.float32:
            raise DryRunError(
                f"ONNX returned shape={output.shape}, dtype={output.dtype}; "
                "expected (1, 10), float32"
            )
        action = output[0]
        return PolicyOutput(action_raw=action, joint_target_h_order=target_from_action(action))


def snapshot_from_observation(observation: Sequence[float] | np.ndarray) -> PolicySnapshot:
    """Decode an H observation for golden replay; not an adapter for real sensors."""
    obs = _vector(observation, OBSERVATION_SIZE, "observation")
    return PolicySnapshot(
        base=BaseObservation(obs[0:3], obs[3:6], obs[6:9]),
        joints=JointObservation(
            np.concatenate((obs[12:14], obs[14:20], obs[20:22])),
            obs[22:32],
        ),
        velocity_command=obs[9:12],
        last_action=obs[32:42],
    )


def replay_golden(package: Path) -> tuple[float, int, float, int]:
    """Check observation construction, action, and H-order targets for all 500 rows."""
    package = package.resolve()
    golden: GoldenData = load_golden(package)
    policy = HPolicy(package)
    actions = np.empty_like(golden.action_raw)
    targets = np.empty_like(golden.joint_target)
    for step, expected_observation in enumerate(golden.obs_flat):
        snapshot = snapshot_from_observation(expected_observation)
        actual_observation = build_observation(snapshot)
        if not np.array_equal(actual_observation, expected_observation):
            raise DryRunError(f"observation assembly differs from golden at step {step}")
        result = policy.evaluate(snapshot)
        actions[step] = result.action_raw
        targets[step] = result.joint_target_h_order
    action_errors = np.max(np.abs(actions - golden.action_raw), axis=1)
    target_errors = np.max(np.abs(targets - golden.joint_target), axis=1)
    action_step = int(np.argmax(action_errors))
    target_step = int(np.argmax(target_errors))
    return float(action_errors[action_step]), action_step, float(target_errors[target_step]), target_step


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Verify H observation-to-target integration without hardware.")
    parser.add_argument("--package", required=True, type=Path)
    args = parser.parse_args()
    try:
        action_error, action_step, target_error, target_step = replay_golden(args.package)
    except (DryRunError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 1
    print(f"joint_order={','.join(JOINT_NAMES)}")
    print(f"action max_abs_error={action_error:.9g} worst_step={action_step}")
    print(f"joint_target_h_order max_abs_error={target_error:.9g} worst_step={target_step}")
    if action_error > ERROR_THRESHOLD or target_error > ERROR_THRESHOLD:
        print(f"FAIL: threshold is {ERROR_THRESHOLD:g}")
        return 1
    print(f"PASS: 42 observation, ONNX action, and H-order targets match golden")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

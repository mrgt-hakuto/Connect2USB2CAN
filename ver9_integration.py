#!/usr/bin/env python3
"""D8's send-free 50 Hz integration of H into ver9.

The module consumes D3 base-frame values and D4/D7 feedback in H order,
builds H's 42-element observation, evaluates the verified ONNX policy, and
creates an ordered ten-joint CAN plan.  ``--live-dry`` can read T265 and both
CAN channels, but this module has no CAN-transmit path.  D9 owns the
separately reviewed, suspended-robot CAN connection.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Protocol, Sequence

import numpy as np

from policy_integration import (
    ACTION_SIZE,
    BaseObservation,
    HPolicy,
    JointObservation,
    PolicyOutput,
    PolicySnapshot,
    build_observation,
    replay_golden,
    target_from_action,
)
from ver9_shell import (
    FixedCommandSource,
    MotorFeedback,
    RealCan,
    RealT265,
    T265_R_OFFSET_M,
    T265Sample,
    VelocityCommand,
    transform_t265_world_to_base,
)
from robot_joint_map import H_CAN_IDS, H_MODELS, LEFT_CAN_IDS, RIGHT_CAN_IDS


HZ = 50.0
PERIOD_S = 1.0 / HZ
@dataclass(frozen=True)
class CanTarget:
    """One D4/D7-converted target.  Values remain in radians and H sign."""

    h_index: int
    can_id: int
    model: str
    position_rad: float


class PolicyEvaluator(Protocol):
    def evaluate(self, snapshot: PolicySnapshot) -> PolicyOutput: ...


def h_targets_to_can_plan(target_h_order: Sequence[float] | np.ndarray) -> tuple[CanTarget, ...]:
    """Apply D4/D7's all-+1 mapping from H order to the physical CAN IDs."""
    targets = np.asarray(target_h_order, dtype=np.float32)
    if targets.shape != (ACTION_SIZE,):
        raise ValueError(f"H target must have shape ({ACTION_SIZE},), got {targets.shape}")
    if not np.isfinite(targets).all():
        raise ValueError("H target contains a non-finite value")
    return tuple(
        CanTarget(index, can_id, model, float(targets[index]))
        for index, (can_id, model) in enumerate(zip(H_CAN_IDS, H_MODELS))
    )


def policy_snapshot_from_inputs(
    t265: T265Sample,
    feedback_by_can_id: Mapping[int, MotorFeedback],
    command: VelocityCommand,
    last_action: Sequence[float] | np.ndarray,
) -> PolicySnapshot:
    """Create the exact H snapshot, rejecting a missing D4/D7 joint reading."""
    missing = tuple(can_id for can_id in H_CAN_IDS if can_id not in feedback_by_can_id)
    if missing:
        rendered = ", ".join(f"0x{can_id:02X}" for can_id in missing)
        raise ValueError(f"missing D4/D7 motor feedback for {rendered}")
    return PolicySnapshot(
        base=BaseObservation(t265.linear_velocity, t265.angular_velocity, t265.projected_gravity),
        joints=JointObservation(
            np.array([feedback_by_can_id[can_id].position for can_id in H_CAN_IDS], dtype=np.float32),
            np.array([feedback_by_can_id[can_id].velocity for can_id in H_CAN_IDS], dtype=np.float32),
        ),
        velocity_command=np.array((command.vx, command.vy, command.wz), dtype=np.float32),
        last_action=np.asarray(last_action, dtype=np.float32),
    )


def evaluate_cycle(
    evaluator: PolicyEvaluator,
    t265: T265Sample,
    feedback_by_can_id: Mapping[int, MotorFeedback],
    command: VelocityCommand,
    last_action: Sequence[float] | np.ndarray,
) -> tuple[PolicySnapshot, np.ndarray, PolicyOutput, tuple[CanTarget, ...]]:
    """One side-effect-free D8 cycle, ending at the D9 CAN-plan boundary."""
    snapshot = policy_snapshot_from_inputs(t265, feedback_by_can_id, command, last_action)
    observation = build_observation(snapshot)
    output = evaluator.evaluate(snapshot)
    return snapshot, observation, output, h_targets_to_can_plan(output.joint_target_h_order)


def _synthetic_inputs(now: float) -> tuple[T265Sample, dict[int, MotorFeedback], VelocityCommand]:
    t265 = T265Sample(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 3, offset_pending=False)
    motors = {can_id: MotorFeedback(can_id, now, 0.0, 0.0) for can_id in H_CAN_IDS}
    command = VelocityCommand(now, 0.0, 0.0, 0.0, source="fixed", state="fixed")
    return t265, motors, command


def _wait_until(deadline: float) -> None:
    """Sleep efficiently, then use a short spin to avoid Windows' coarse wakeup."""
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0.001:
            break
        time.sleep(remaining - 0.001)
    while time.perf_counter() < deadline:
        pass


def _set_windows_timer_resolution_1ms(enabled: bool) -> bool:
    """Use the resolution already required by the motor console for 50 Hz."""
    if sys.platform != "win32":
        return False
    try:
        import ctypes
        result = ctypes.windll.winmm.timeBeginPeriod(1) if enabled else ctypes.windll.winmm.timeEndPeriod(1)
        return result == 0
    except Exception:
        return False


def run_synthetic(policy: HPolicy, duration_s: float, csv_path: Path) -> dict[str, float | int]:
    """Run the entire D8 pipeline at 50 Hz with no device access."""
    if duration_s <= 0:
        raise ValueError("duration must be positive")
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["sequence", "tick_monotonic_s", "period_s", "overrun"]
    fields += [f"observation_{index}" for index in range(42)]
    fields += [f"action_{index}" for index in range(10)]
    fields += [f"target_h_{index}" for index in range(10)]
    fields += [f"can_id_{index}" for index in range(10)]
    fields += [f"can_target_{index}" for index in range(10)]
    previous_tick: float | None = None
    # On Windows, monotonic() is commonly backed by a ~15.6 ms tick.  The
    # scheduler needs perf_counter()'s high-resolution clock to assess 20 ms.
    next_tick = time.perf_counter()
    end = next_tick + duration_s
    last_action = np.zeros(ACTION_SIZE, dtype=np.float32)
    sequence = overrun_count = 0
    timer_resolution_changed = _set_windows_timer_resolution_1ms(True)
    try:
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while time.perf_counter() < end:
                _wait_until(next_tick)
                tick = time.perf_counter()
                t265, motors, command = _synthetic_inputs(tick)
                _snapshot, observation, output, plan = evaluate_cycle(policy, t265, motors, command, last_action)
                overrun = tick > next_tick + 0.001
                overrun_count += int(overrun)
                row: dict[str, float | int | str] = {
                    "sequence": sequence,
                    "tick_monotonic_s": f"{tick:.9f}",
                    "period_s": "" if previous_tick is None else f"{tick - previous_tick:.9f}",
                    "overrun": int(overrun),
                }
                row.update({f"observation_{index}": float(value) for index, value in enumerate(observation)})
                row.update({f"action_{index}": float(value) for index, value in enumerate(output.action_raw)})
                row.update({f"target_h_{index}": float(value) for index, value in enumerate(output.joint_target_h_order)})
                row.update({f"can_id_{index}": f"0x{target.can_id:02X}" for index, target in enumerate(plan)})
                row.update({f"can_target_{index}": target.position_rad for index, target in enumerate(plan)})
                writer.writerow(row)
                previous_tick = tick
                last_action = output.action_raw
                sequence += 1
                next_tick += PERIOD_S
                # Never emit a catch-up cycle immediately after a missed deadline:
                # that would produce a false near-zero control period.
                if next_tick <= tick:
                    next_tick = tick + PERIOD_S
    finally:
        if timer_resolution_changed:
            _set_windows_timer_resolution_1ms(False)
    return {"ticks": sequence, "overruns": overrun_count}


def run_live_dry(
    policy: PolicyEvaluator,
    t265: RealT265,
    left_can: RealCan,
    right_can: RealCan,
    command: FixedCommandSource,
    duration_s: float,
    csv_path: Path,
) -> dict[str, float | int]:
    """Read real D3/D4 inputs through the D8 policy path without sending CAN.

    The caller supplies two receive-only ``RealCan`` sources: ch=0 for the
    five left-leg IDs and ch=1 for the five right-leg IDs.  No source offered
    to this function has a transmit method.
    """
    if duration_s <= 0:
        raise ValueError("duration must be positive")
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["sequence", "tick_perf_counter_s", "period_s", "overrun", "t265_age_s"]
    fields += [f"observation_{index}" for index in range(42)]
    fields += [f"action_{index}" for index in range(10)]
    fields += [f"can_id_{index}" for index in range(10)]
    fields += [f"can_target_{index}" for index in range(10)]
    previous_tick: float | None = None
    last_action = np.zeros(ACTION_SIZE, dtype=np.float32)
    sequence = overrun_count = 0
    started: list[object] = []
    timer_resolution_changed = _set_windows_timer_resolution_1ms(True)
    try:
        for source in (t265, left_can, right_can):
            source.start()
            started.append(source)
        warmup_deadline = time.monotonic() + 5.0
        while t265.latest() is None:
            if time.monotonic() >= warmup_deadline:
                raise RuntimeError("T265 produced no pose sample within 5 seconds")
            time.sleep(0.01)
        next_tick = time.perf_counter()
        end = next_tick + duration_s
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while time.perf_counter() < end:
                _wait_until(next_tick)
                tick = time.perf_counter()
                t265_sample = t265.latest()
                if t265_sample is None:
                    raise RuntimeError("T265 has not produced a pose sample")
                feedback = left_can.latest(LEFT_CAN_IDS)
                feedback.update(right_can.latest(RIGHT_CAN_IDS))
                snapshot, observation, output, plan = evaluate_cycle(
                    policy, t265_sample, feedback, command.sample(tick), last_action,
                )
                overrun = tick > next_tick + 0.001
                overrun_count += int(overrun)
                row: dict[str, float | int | str] = {
                    "sequence": sequence,
                    "tick_perf_counter_s": f"{tick:.9f}",
                    "period_s": "" if previous_tick is None else f"{tick - previous_tick:.9f}",
                    "overrun": int(overrun),
                    "t265_age_s": f"{time.monotonic() - t265_sample.acquired_monotonic_s:.9f}",
                }
                row.update({f"observation_{index}": float(value) for index, value in enumerate(observation)})
                row.update({f"action_{index}": float(value) for index, value in enumerate(output.action_raw)})
                row.update({f"can_id_{index}": f"0x{target.can_id:02X}" for index, target in enumerate(plan)})
                row.update({f"can_target_{index}": target.position_rad for index, target in enumerate(plan)})
                writer.writerow(row)
                previous_tick, last_action = tick, output.action_raw
                sequence += 1
                next_tick += PERIOD_S
                if next_tick <= tick:
                    next_tick = tick + PERIOD_S
    finally:
        for source in reversed(started):
            source.close()
        if timer_resolution_changed:
            _set_windows_timer_resolution_1ms(False)
    if left_can.tx_count or right_can.tx_count:
        raise RuntimeError("live dry run observed CAN transmission")
    return {"ticks": sequence, "overruns": overrun_count, "can_tx_count": 0}


def main() -> int:
    parser = argparse.ArgumentParser(description="D8 H/ver9 integration check (never sends CAN).")
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--live-dry", action="store_true", help="read T265 and both CAN channels, but never transmit")
    parser.add_argument("--left-can-channel", type=int, default=0)
    parser.add_argument("--right-can-channel", type=int, default=1)
    parser.add_argument("--can-bitrate", type=int, default=1_000_000)
    parser.add_argument("--fixed-vx", type=float, default=0.0)
    parser.add_argument("--fixed-vy", type=float, default=0.0)
    parser.add_argument("--fixed-wz", type=float, default=0.0)
    args = parser.parse_args()
    try:
        action_error, action_step, target_error, target_step = replay_golden(args.package)
        policy = HPolicy(args.package)
        if args.live_dry:
            summary = run_live_dry(
                policy,
                RealT265(T265_R_OFFSET_M),
                RealCan(args.left_can_channel, args.can_bitrate),
                RealCan(args.right_can_channel, args.can_bitrate),
                FixedCommandSource(args.fixed_vx, args.fixed_vy, args.fixed_wz),
                args.duration,
                args.csv,
            )
        else:
            summary = run_synthetic(policy, args.duration, args.csv)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}")
        return 1
    print(f"golden_action_max_abs_error={action_error:.9g} worst_step={action_step}")
    print(f"golden_target_max_abs_error={target_error:.9g} worst_step={target_step}")
    print(f"ticks={summary['ticks']} overruns={summary['overruns']} csv={args.csv}")
    mode = "live receive-only" if args.live_dry else "synthetic"
    print(f"PASS: D8 {mode} loop made 42 observations, ONNX outputs, and 10 D4/D7 CAN plans; CAN was never sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""D2: receive-only, policy-independent 50 Hz ver9 shell.

This program gathers timestamped T265, CAN-feedback, and velocity-command
samples into one CSV row per scheduler tick.  It intentionally contains no
ONNX execution, no H-order conversion, and no CAN transmit path.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Protocol, Sequence

import numpy as np


HZ = 50.0
PERIOD_S = 1.0 / HZ
R_CB = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]], dtype=np.float64)
G_WORLD = np.array([0.0, -1.0, 0.0], dtype=np.float64)


def _triple(values: Sequence[float], name: str) -> tuple[float, float, float]:
    vector = tuple(float(value) for value in values)
    if len(vector) != 3 or not all(math.isfinite(value) for value in vector):
        raise ValueError(f"{name} must be three finite values")
    return vector  # type: ignore[return-value]


@dataclass(frozen=True)
class T265Sample:
    acquired_monotonic_s: float
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]
    projected_gravity: tuple[float, float, float]
    confidence: int
    offset_pending: bool = True


@dataclass(frozen=True)
class MotorFeedback:
    can_id: int
    acquired_monotonic_s: float
    position: float
    velocity: float


@dataclass(frozen=True)
class VelocityCommand:
    acquired_monotonic_s: float
    vx: float
    vy: float
    wz: float
    source: str
    state: str


@dataclass(frozen=True)
class CycleSnapshot:
    sequence: int
    tick_monotonic_s: float
    wall_time_s: float
    period_s: Optional[float]
    overrun: bool
    t265: Optional[T265Sample]
    motors: Dict[int, MotorFeedback]
    missing_can_ids: tuple[int, ...]
    command: VelocityCommand
    observation_placeholder: tuple[float, ...]
    planned_target: tuple[float, ...]


class T265Source(Protocol):
    def start(self) -> None: ...
    def latest(self) -> Optional[T265Sample]: ...
    def close(self) -> None: ...


class CanSource(Protocol):
    tx_count: int
    def start(self) -> None: ...
    def latest(self, expected_ids: Iterable[int]) -> Dict[int, MotorFeedback]: ...
    def close(self) -> None: ...


class CommandSource(Protocol):
    def sample(self, timestamp_s: float) -> VelocityCommand: ...


class FixedCommandSource:
    """D5's fixed-command boundary, with no device or controller import."""

    def __init__(self, vx: float, vy: float, wz: float) -> None:
        if not all(-1.0 <= value <= 1.0 for value in (vx, vy, wz)):
            raise ValueError("fixed command values must each be in [-1, 1]")
        self._values = (vx, vy, wz)

    def sample(self, timestamp_s: float) -> VelocityCommand:
        return VelocityCommand(timestamp_s, *self._values, source="fixed", state="fixed")


class SyntheticT265:
    """Hardware-free deterministic source for the D2 scheduler test."""

    def start(self) -> None:
        return None

    def latest(self) -> Optional[T265Sample]:
        now = time.monotonic()
        return T265Sample(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 3)

    def close(self) -> None:
        return None


class SyntheticCan:
    tx_count = 0

    def __init__(self, can_ids: Sequence[int]) -> None:
        self._can_ids = tuple(can_ids)

    def start(self) -> None:
        return None

    def latest(self, expected_ids: Iterable[int]) -> Dict[int, MotorFeedback]:
        now = time.monotonic()
        return {can_id: MotorFeedback(can_id, now, 0.0, 0.0) for can_id in expected_ids}

    def close(self) -> None:
        return None


def _quat_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("T265 returned a zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_t265_world_to_base(
    rotation_world_from_camera: np.ndarray,
    velocity_world: np.ndarray,
    angular_velocity_world: np.ndarray,
    base_to_tracking_center_m: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Apply D3's documented T265-to-base transform.

    ``base_to_tracking_center_m`` is deliberately an argument rather than a
    guessed module constant: it must be the CAD-recorded vector from the
    policy base origin to the T265 tracking center.
    """
    rotation = np.asarray(rotation_world_from_camera, dtype=np.float64)
    velocity = np.asarray(velocity_world, dtype=np.float64)
    angular_velocity = np.asarray(angular_velocity_world, dtype=np.float64)
    offset = np.asarray(base_to_tracking_center_m, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("T265 rotation must have shape (3, 3)")
    if any(vector.shape != (3,) for vector in (velocity, angular_velocity, offset)):
        raise ValueError("T265 velocity, angular velocity, and offset must have shape (3,)")
    if not all(np.isfinite(vector).all() for vector in (rotation, velocity, angular_velocity, offset)):
        raise ValueError("T265 transform inputs must be finite")
    velocity_camera = rotation.T @ velocity
    angular_camera = rotation.T @ angular_velocity
    angular_base = R_CB.T @ angular_camera
    linear_base = R_CB.T @ velocity_camera - np.cross(angular_base, offset)
    gravity_base = R_CB.T @ (rotation.T @ G_WORLD)
    return linear_base, angular_base, gravity_base


class RealT265:
    """Background T265 reader with the explicitly supplied D3 offset."""

    def __init__(self, base_to_tracking_center_m: Sequence[float]) -> None:
        self._offset = np.asarray(_triple(base_to_tracking_center_m, "T265 base-to-tracking-center offset"), dtype=np.float64)
        self._lock = threading.Lock()
        self._latest: Optional[T265Sample] = None
        self._error: Optional[str] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pipe = None

    def start(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as error:
            raise RuntimeError("pyrealsense2 is unavailable; use the Python 3.10 .venv") from error
        ctx = rs.context()
        devices = list(ctx.query_devices())
        if not devices:
            raise RuntimeError("T265 was not found")
        serial = devices[0].get_info(rs.camera_info.serial_number)
        pipe = None
        last_error: Optional[Exception] = None
        for _ in range(8):
            try:
                candidate = rs.pipeline()
                config = rs.config()
                config.enable_device(serial)
                config.enable_stream(rs.stream.pose)
                candidate.start(config)
                pipe = candidate
                break
            except RuntimeError as error:
                last_error = error
                time.sleep(2.0)
        if pipe is None:
            raise RuntimeError(f"T265 start failed after 8 attempts: {last_error}")
        self._pipe = pipe
        self._stop.clear()
        self._thread = threading.Thread(target=self._receive_loop, name="ver9-t265", daemon=True)
        self._thread.start()

    def _receive_loop(self) -> None:
        assert self._pipe is not None
        try:
            while not self._stop.is_set():
                frames = self._pipe.wait_for_frames(100)
                pose_frame = frames.get_pose_frame()
                if not pose_frame:
                    continue
                pose = pose_frame.get_pose_data()
                rotation = _quat_to_rotation(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)
                velocity_world = np.array([pose.velocity.x, pose.velocity.y, pose.velocity.z])
                angular_world = np.array([pose.angular_velocity.x, pose.angular_velocity.y, pose.angular_velocity.z])
                linear_base, angular_base, gravity_base = transform_t265_world_to_base(
                    rotation, velocity_world, angular_world, self._offset,
                )
                sample = T265Sample(
                    time.monotonic(),
                    _triple(linear_base, "linear velocity"),
                    _triple(angular_base, "angular velocity"),
                    _triple(gravity_base, "projected gravity"),
                    int(pose.tracker_confidence),
                    offset_pending=False,
                )
                with self._lock:
                    self._latest = sample
        except Exception as error:
            with self._lock:
                self._error = f"{type(error).__name__}: {error}"

    def latest(self) -> Optional[T265Sample]:
        with self._lock:
            if self._error:
                raise RuntimeError(f"T265 receiver stopped: {self._error}")
            return self._latest

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._pipe:
            self._pipe.stop()
            self._pipe = None


class RealCan:
    """Read Cubemars status frames only.  This class has no transmit method."""

    tx_count = 0

    def __init__(self, channel: int, bitrate: int) -> None:
        self._channel = channel
        self._bitrate = bitrate
        self._bus = None

    def start(self) -> None:
        import cubemars

        self._bus = cubemars.MotorBus(channel=self._channel, bitrate=self._bitrate)
        self._bus.open()  # MotorBus.open starts its receive thread; it does not transmit.

    def latest(self, expected_ids: Iterable[int]) -> Dict[int, MotorFeedback]:
        if self._bus is None:
            raise RuntimeError("CAN receiver is not open")
        now = time.monotonic()
        if self._bus.rx_error:
            raise RuntimeError(f"CAN receiver stopped: {self._bus.rx_error}")
        result: Dict[int, MotorFeedback] = {}
        for can_id in expected_ids:
            state = self._bus.state(can_id)
            if state is not None:
                result[can_id] = MotorFeedback(can_id, now, float(state.pos), float(state.spd))
        self.tx_count = int(self._bus.tx_count)
        return result

    def close(self) -> None:
        if self._bus is not None:
            # MotorBus.close defaults to stop_motors=True, which transmits.  D2 must not.
            self._bus.close(stop_motors=False)
            self._bus = None


class CsvLog:
    def __init__(self, path: Path, can_ids: Sequence[int]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("w", newline="", encoding="utf-8")
        self._can_ids = tuple(can_ids)
        fields = [
            "sequence", "tick_monotonic_s", "wall_time_s", "period_s", "overrun",
            "t265_acquired_monotonic_s", "t265_age_s", "t265_confidence", "t265_offset_pending",
            "linear_velocity_x", "linear_velocity_y", "linear_velocity_z",
            "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
            "projected_gravity_x", "projected_gravity_y", "projected_gravity_z",
            "command_acquired_monotonic_s", "command_age_s", "vx", "vy", "wz", "command_source", "command_state",
            "missing_can_ids",
        ]
        for can_id in self._can_ids:
            fields.extend((f"can_{can_id}_acquired_monotonic_s", f"can_{can_id}_age_s", f"can_{can_id}_position", f"can_{can_id}_velocity"))
        fields.extend(f"observation_placeholder_{index}" for index in range(42))
        fields.extend(f"planned_target_{index}" for index in range(10))
        self._writer = csv.DictWriter(self._file, fieldnames=fields)
        self._writer.writeheader()

    def write(self, snapshot: CycleSnapshot) -> None:
        row: Dict[str, object] = {
            "sequence": snapshot.sequence,
            "tick_monotonic_s": f"{snapshot.tick_monotonic_s:.9f}",
            "wall_time_s": f"{snapshot.wall_time_s:.6f}",
            "period_s": "" if snapshot.period_s is None else f"{snapshot.period_s:.9f}",
            "overrun": int(snapshot.overrun),
            "command_acquired_monotonic_s": f"{snapshot.command.acquired_monotonic_s:.9f}",
            "command_age_s": f"{snapshot.tick_monotonic_s - snapshot.command.acquired_monotonic_s:.9f}",
            "vx": snapshot.command.vx, "vy": snapshot.command.vy, "wz": snapshot.command.wz,
            "command_source": snapshot.command.source, "command_state": snapshot.command.state,
            "missing_can_ids": ";".join(str(value) for value in snapshot.missing_can_ids),
        }
        if snapshot.t265 is not None:
            sample = snapshot.t265
            row.update({
                "t265_acquired_monotonic_s": f"{sample.acquired_monotonic_s:.9f}",
                "t265_age_s": f"{snapshot.tick_monotonic_s - sample.acquired_monotonic_s:.9f}",
                "t265_confidence": sample.confidence, "t265_offset_pending": int(sample.offset_pending),
            })
            for prefix, values in (("linear_velocity", sample.linear_velocity), ("angular_velocity", sample.angular_velocity), ("projected_gravity", sample.projected_gravity)):
                row.update({f"{prefix}_{axis}": value for axis, value in zip(("x", "y", "z"), values)})
        for can_id in self._can_ids:
            sample = snapshot.motors.get(can_id)
            if sample is not None:
                row.update({
                    f"can_{can_id}_acquired_monotonic_s": f"{sample.acquired_monotonic_s:.9f}",
                    f"can_{can_id}_age_s": f"{snapshot.tick_monotonic_s - sample.acquired_monotonic_s:.9f}",
                    f"can_{can_id}_position": sample.position,
                    f"can_{can_id}_velocity": sample.velocity,
                })
        if len(snapshot.observation_placeholder) != 42:
            raise ValueError("D2 observation placeholder must have 42 elements")
        if len(snapshot.planned_target) != 10:
            raise ValueError("D2 planned target must have 10 elements")
        row.update({f"observation_placeholder_{index}": value for index, value in enumerate(snapshot.observation_placeholder)})
        row.update({f"planned_target_{index}": value for index, value in enumerate(snapshot.planned_target)})
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        self._file.close()


class Shell:
    def __init__(self, t265: Optional[T265Source], can: Optional[CanSource], command: CommandSource, can_ids: Sequence[int]) -> None:
        if len(set(can_ids)) != len(can_ids):
            raise ValueError("CAN IDs must not repeat")
        self._t265, self._can, self._command = t265, can, command
        self._can_ids = tuple(can_ids)

    def run(self, duration_s: float, output: Path) -> dict[str, float | int]:
        if duration_s <= 0:
            raise ValueError("duration must be positive")
        logger = CsvLog(output, self._can_ids)
        started: list[object] = []
        previous_tick: Optional[float] = None
        overrun_count = 0
        sequence = 0
        next_tick = time.monotonic()
        try:
            for source in (self._t265, self._can):
                if source is not None:
                    source.start()
                    started.append(source)
            end = time.monotonic() + duration_s
            while time.monotonic() < end:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                tick = time.monotonic()
                overrun = tick > next_tick + 0.001
                if overrun:
                    overrun_count += 1
                period = None if previous_tick is None else tick - previous_tick
                previous_tick = tick
                t265 = self._t265.latest() if self._t265 is not None else None
                motors = self._can.latest(self._can_ids) if self._can is not None else {}
                missing = tuple(can_id for can_id in self._can_ids if can_id not in motors)
                command = self._command.sample(tick)
                logger.write(CycleSnapshot(sequence, tick, time.time(), period, overrun, t265, motors, missing, command, (0.0,) * 42, (0.0,) * 10))
                sequence += 1
                next_tick += PERIOD_S
                if next_tick < tick - PERIOD_S:
                    next_tick = tick + PERIOD_S
        finally:
            logger.close()
            for source in reversed(started):
                source.close()
        tx_count = 0 if self._can is None else self._can.tx_count
        return {"ticks": sequence, "overruns": overrun_count, "can_tx_count": tx_count}


def _parse_can_ids(text: str) -> tuple[int, ...]:
    if not text:
        return ()
    values = tuple(int(part.strip(), 0) for part in text.split(","))
    if len(values) != 10:
        raise argparse.ArgumentTypeError("--can-ids must list exactly 10 IDs for D2")
    if any(value < 1 or value > 127 for value in values):
        raise argparse.ArgumentTypeError("CAN IDs must be in [1, 127]")
    return values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D2 receive-only 50 Hz ver9 shell (no CAN transmit, no ONNX).")
    parser.add_argument("--synthetic", action="store_true", help="hardware-free T265/CAN test sources")
    parser.add_argument("--t265", action="store_true", help="read T265 in a background thread")
    parser.add_argument("--t265-r-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="D3 CAD vector [m]: base origin to T265 tracking center")
    parser.add_argument("--can-listen", action="store_true", help="read Cubemars CAN feedback; never sends")
    parser.add_argument("--can-ids", type=_parse_can_ids, default=(), help="exactly 10 CAN IDs, comma-separated")
    parser.add_argument("--can-channel", type=int, default=1)
    parser.add_argument("--can-bitrate", type=int, default=1_000_000)
    parser.add_argument("--fixed-vx", type=float, default=0.0)
    parser.add_argument("--fixed-vy", type=float, default=0.0)
    parser.add_argument("--fixed-wz", type=float, default=0.0)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    args = parser.parse_args()
    if args.synthetic and (args.t265 or args.can_listen):
        parser.error("--synthetic cannot be combined with --t265 or --can-listen")
    if args.can_listen and not args.can_ids:
        parser.error("--can-listen requires the 10 raw CAN IDs via --can-ids")
    if args.can_ids and not args.can_listen and not args.synthetic:
        parser.error("--can-ids is only meaningful with --can-listen")
    if args.t265 and args.t265_r_offset is None:
        parser.error("--t265 requires --t265-r-offset X Y Z; do not guess the D3 CAD vector")
    return args


def main() -> int:
    args = parse_args()
    can_ids = tuple(range(1, 11)) if args.synthetic else args.can_ids
    t265: Optional[T265Source] = SyntheticT265() if args.synthetic else (RealT265(args.t265_r_offset) if args.t265 else None)
    can: Optional[CanSource] = SyntheticCan(can_ids) if args.synthetic else (RealCan(args.can_channel, args.can_bitrate) if args.can_listen else None)
    try:
        summary = Shell(t265, can, FixedCommandSource(args.fixed_vx, args.fixed_vy, args.fixed_wz), can_ids).run(args.duration, args.csv)
    except (RuntimeError, ValueError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"ticks={summary['ticks']} overruns={summary['overruns']} can_tx_count={summary['can_tx_count']} csv={args.csv}")
    if summary["can_tx_count"] != 0:
        print("ERROR: D2 invariant violated: CAN transmission was observed", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

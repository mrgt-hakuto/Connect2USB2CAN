#!/usr/bin/env python3
"""Jetson-only operator deployment: Switch 2 Pro + T265 + two USB2CAN buses.

The process refuses to arm without an NVIDIA ONNX Runtime provider.  It sends
MIT frames only with --arm; every exit after CAN opens sends three zero frames.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import select
import time
from pathlib import Path

import numpy as np

from policy_integration import ACTION_SIZE, HPolicy
from ver9_d8_sender import DualBus, frames, preview
from ver9_integration import evaluate_cycle
from ver9_shell import RealT265, T265_R_OFFSET_M, VelocityCommand

HZ = 50.0
PERIOD_S = 1.0 / HZ
T265_STALE_S = 0.200


def normalize(value: int, minimum: int, maximum: int, deadzone: float) -> float:
    center = (minimum + maximum) / 2.0
    half = max((maximum - minimum) / 2.0, 1.0)
    result = max(-1.0, min(1.0, (value - center) / half))
    return 0.0 if abs(result) <= deadzone else (abs(result) - deadzone) / (1.0 - deadzone) * (1.0 if result > 0 else -1.0)


def map_command(x_norm: float, y_norm: float, turn_right: bool, turn_left: bool, max_vx: float, max_vy: float, max_wz: float) -> tuple[float, float, float]:
    """D5-confirmed stick signs; A=right yaw and Y=left yaw."""
    if not all(0.0 <= value <= 1.0 for value in (max_vx, max_vy, max_wz)):
        raise ValueError("command maxima must be in [0, 1]")
    wz = max_wz if turn_right and not turn_left else (-max_wz if turn_left and not turn_right else 0.0)
    return (-y_norm * max_vx, -x_norm * max_vy, wz)


class ProController:
    def __init__(self, device_path: str, deadzone: float, max_vx: float, max_vy: float, max_wz: float) -> None:
        try:
            from evdev import InputDevice, ecodes
        except ImportError as error:
            raise RuntimeError("evdev is unavailable on Jetson") from error
        if not os.path.exists(device_path):
            raise RuntimeError(f"controller device not found: {device_path}")
        self._ecodes, self._device = ecodes, InputDevice(device_path)
        axes = {code: info for code, info in self._device.capabilities(absinfo=True).get(ecodes.EV_ABS, [])}
        if ecodes.ABS_X not in axes or ecodes.ABS_Y not in axes:
            self._device.close(); raise RuntimeError("controller lacks ABS_X/ABS_Y")
        self._x_info, self._y_info = axes[ecodes.ABS_X], axes[ecodes.ABS_Y]
        self._x = self._device.absinfo(ecodes.ABS_X).value
        self._y = self._device.absinfo(ecodes.ABS_Y).value
        keys = set(self._device.active_keys())
        self._right, self._left = ecodes.BTN_SOUTH in keys, ecodes.BTN_WEST in keys
        self._deadzone, self._maxima = deadzone, (max_vx, max_vy, max_wz)

    def close(self) -> None: self._device.close()

    def sample(self, now: float) -> VelocityCommand:
        if not os.path.exists(self._device.path):
            raise RuntimeError("controller device disappeared")
        readable, _, _ = select.select([self._device.fd], [], [], 0.0)
        if readable:
            try:
                for event in self._device.read():
                    if event.type == self._ecodes.EV_ABS and event.code == self._ecodes.ABS_X: self._x = event.value
                    elif event.type == self._ecodes.EV_ABS and event.code == self._ecodes.ABS_Y: self._y = event.value
                    elif event.type == self._ecodes.EV_KEY and event.code == self._ecodes.BTN_SOUTH: self._right = bool(event.value)
                    elif event.type == self._ecodes.EV_KEY and event.code == self._ecodes.BTN_WEST: self._left = bool(event.value)
            except OSError as error:
                raise RuntimeError(f"controller input failed: {error}") from error
        x = normalize(self._x, self._x_info.min, self._x_info.max, self._deadzone)
        y = normalize(self._y, self._y_info.min, self._y_info.max, self._deadzone)
        return VelocityCommand(now, *map_command(x, y, self._right, self._left, *self._maxima), source="switch2-pro", state="active")

    def require_neutral(self) -> None:
        command = self.sample(time.monotonic())
        if (command.vx, command.vy, command.wz) != (0.0, 0.0, 0.0):
            raise RuntimeError("controller must be neutral before arming")


def run(args: argparse.Namespace) -> None:
    controller = ProController(args.device, args.deadzone, args.max_vx, args.max_vy, args.max_wz)
    policy = HPolicy(args.package, providers=["TensorrtExecutionProvider", "CUDAExecutionProvider"])
    if not any(provider in policy.providers for provider in ("TensorrtExecutionProvider", "CUDAExecutionProvider")):
        controller.close(); raise RuntimeError(f"Jetson GPU provider unavailable: {policy.providers}")
    bus, t265, opened, t265_started = DualBus(args.left_can_channel, args.right_can_channel), RealT265(T265_R_OFFSET_M), False, False
    last_action = np.zeros(ACTION_SIZE, dtype=np.float32)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    try:
        controller.require_neutral(); bus.open(); opened = True; t265.start(); t265_started = True
        deadline = time.monotonic() + 5.0
        while t265.latest() is None:
            if time.monotonic() > deadline: raise RuntimeError("T265 warmup timeout")
            time.sleep(0.01)
        next_tick, end = time.monotonic(), time.monotonic() + args.duration
        with args.csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle); writer.writerow(("tick", "vx", "vy", "wz", *[f"target_h_{i}" for i in range(10)]))
            while time.monotonic() < end:
                time.sleep(max(0.0, next_tick - time.monotonic())); tick = time.monotonic()
                command, pose = controller.sample(tick), t265.latest()
                if pose is None or tick - pose.acquired_monotonic_s > T265_STALE_S: raise RuntimeError("T265 stale")
                _s, _o, output, _plan = evaluate_cycle(policy, pose, bus.feedback(), command, last_action)
                for frame in frames(output.joint_target_h_order): bus.send(frame)
                writer.writerow((f"{tick:.9f}", command.vx, command.vy, command.wz, *output.joint_target_h_order)); handle.flush()
                last_action, next_tick = output.action_raw, next_tick + PERIOD_S
                if next_tick <= tick: next_tick = tick + PERIOD_S
    finally:
        if opened: bus.zero(); bus.close()
        if t265_started: t265.close()
        controller.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Jetson full deployment; --arm is required for CAN transmit.")
    parser.add_argument("--preview", action="store_true"); parser.add_argument("--arm", action="store_true")
    parser.add_argument("--package", type=Path); parser.add_argument("--device"); parser.add_argument("--csv", type=Path); parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--left-can-channel", type=int, default=0); parser.add_argument("--right-can-channel", type=int, default=1)
    parser.add_argument("--deadzone", type=float, default=0.05); parser.add_argument("--max-vx", type=float, default=0.30); parser.add_argument("--max-vy", type=float, default=0.20); parser.add_argument("--max-wz", type=float, default=0.30)
    args = parser.parse_args()
    if args.preview: preview(); print("Jetson mapping: left stick=VX/VY, A=+WZ, Y=-WZ; GPU provider required"); return 0
    if not args.arm: parser.error("--arm is required; --preview never opens hardware")
    if not args.package or not args.device or not args.csv or not 0.0 < args.duration <= 60.0: parser.error("--package, --device, --csv and 0<--duration<=60 are required")
    if not 0.0 <= args.deadzone < 1.0: parser.error("--deadzone must be in [0,1)")
    try: run(args)
    except (OSError, RuntimeError, ValueError) as error: print(f"ERROR: {error}"); return 1
    return 0


if __name__ == "__main__": raise SystemExit(main())

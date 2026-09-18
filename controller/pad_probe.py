#!/usr/bin/env python3
"""Read a Switch 2 Pro controller at 50 Hz without CAN, ONNX, or motor access.

This is a dry-run input probe.  It never imports any motor/CAN/policy module and
it never sends data outside stdout and its CSV log.
"""

from __future__ import annotations

import argparse
import csv
import os
import select
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

DEFAULT_DEVICE = "/dev/input/by-id/usb-045e_XBOX_360_For_Windows_000000000001-event-joystick"
HZ = 50.0
STALE_S = 0.200


@dataclass
class Stick:
    x: int = 0
    y: int = 0
    last_event_s: Optional[float] = None


def normalize(value: int, minimum: int, maximum: int, deadzone: float) -> float:
    """Return a symmetric, dead-zoned axis value in [-1, 1]."""
    center = (minimum + maximum) / 2.0
    half_range = max((maximum - minimum) / 2.0, 1.0)
    result = max(-1.0, min(1.0, (value - center) / half_range))
    if abs(result) <= deadzone:
        return 0.0
    return (abs(result) - deadzone) / (1.0 - deadzone) * (1.0 if result > 0 else -1.0)


def command(x_norm: float, y_norm: float, mapping_confirmed: bool,
            x_sign: int, y_sign: int) -> tuple[float, float, float]:
    """Map the stick to a bounded base-frame command only after confirmation."""
    if not mapping_confirmed:
        return (0.0, 0.0, 0.0)
    return (0.3 * x_sign * y_norm, 0.2 * y_sign * x_norm, 0.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="50 Hz controller dry-run probe; no CAN/ONNX/motor control.")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="evdev device path")
    parser.add_argument("--csv", type=Path, default=None, help="CSV output path (default: controller/logs)")
    parser.add_argument("--deadzone", type=float, default=0.05)
    parser.add_argument("--confirm-base-mapping", action="store_true",
                        help="allow nonzero displayed command after checking obs_contract.md")
    parser.add_argument("--x-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--y-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--duration", type=float, default=None, help="stop after this many seconds")
    parser.add_argument("--self-test", action="store_true", help="test pure mapping logic; needs no controller")
    args = parser.parse_args()
    if not 0.0 <= args.deadzone < 1.0:
        parser.error("--deadzone must be in [0, 1)")
    return args


def self_test() -> None:
    assert normalize(0, -32768, 32767, 0.05) == 0.0
    assert normalize(32767, -32768, 32767, 0.05) > 0.99
    assert command(1.0, 1.0, False, 1, 1) == (0.0, 0.0, 0.0)
    assert command(1.0, 1.0, True, 1, 1) == (0.3, 0.2, 0.0)
    print("self-test: OK")


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    try:
        from evdev import InputDevice, ecodes
    except ImportError:
        print("evdev is not installed in this Python environment; do not install it automatically.", file=sys.stderr)
        return 2

    if not os.path.exists(args.device):
        print(f"controller device not found: {args.device}", file=sys.stderr)
        return 2
    device = InputDevice(args.device)
    caps = device.capabilities(absinfo=True).get(ecodes.EV_ABS, [])
    axes = {code: info for code, info in caps}
    if ecodes.ABS_X not in axes or ecodes.ABS_Y not in axes:
        print("ABS_X/ABS_Y are unavailable on this device; refusing to continue.", file=sys.stderr)
        return 2

    x_info, y_info = axes[ecodes.ABS_X], axes[ecodes.ABS_Y]
    stick = Stick()
    started = time.monotonic()
    next_tick = started
    csv_path = args.csv or (Path(__file__).resolve().parent / "logs" / f"pad_probe_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"device={device.path} name={device.name!r} csv={csv_path}")
    if not args.confirm_base_mapping:
        print("base mapping is UNCONFIRMED: command columns intentionally remain zero.")

    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("host_monotonic_s", "raw_x", "raw_y", "x_norm", "y_norm", "vx", "vy", "wz", "state"))
        try:
            while args.duration is None or time.monotonic() - started < args.duration:
                now = time.monotonic()
                timeout = max(0.0, next_tick - now)
                readable, _, _ = select.select([device.fd], [], [], timeout)
                if readable:
                    for event in device.read():
                        if event.type == ecodes.EV_ABS and event.code == ecodes.ABS_X:
                            stick.x, stick.last_event_s = event.value, time.monotonic()
                        elif event.type == ecodes.EV_ABS and event.code == ecodes.ABS_Y:
                            stick.y, stick.last_event_s = event.value, time.monotonic()
                now = time.monotonic()
                if now < next_tick:
                    continue
                next_tick += 1.0 / HZ
                if next_tick < now - 1.0 / HZ:
                    next_tick = now + 1.0 / HZ
                x_norm = normalize(stick.x, x_info.min, x_info.max, args.deadzone)
                y_norm = normalize(stick.y, y_info.min, y_info.max, args.deadzone)
                stale = stick.last_event_s is None or now - stick.last_event_s > STALE_S
                state = "timeout" if stale else ("mapped" if args.confirm_base_mapping else "mapping_unconfirmed")
                vx, vy, wz = (0.0, 0.0, 0.0) if stale else command(x_norm, y_norm, args.confirm_base_mapping, args.x_sign, args.y_sign)
                row = (f"{now:.6f}", stick.x, stick.y, f"{x_norm:.4f}", f"{y_norm:.4f}", f"{vx:.4f}", f"{vy:.4f}", f"{wz:.4f}", state)
                writer.writerow(row)
                file.flush()
                print(" ".join(map(str, row)))
        except KeyboardInterrupt:
            print("stopped by user")
        except OSError as error:
            print(f"input disconnected or unreadable: {error}", file=sys.stderr)
            return 3
        finally:
            device.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

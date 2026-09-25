#!/usr/bin/env python3
"""Print and CSV-log D5 command-source samples without hardware access."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

from command_source import CommandRecord, controller_command, fixed_command


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D5 dry-run command probe; no device, CAN, ONNX, T265, or motor access.")
    parser.add_argument("--source", choices=("fixed", "controller"), default="fixed")
    parser.add_argument("--vx", type=float, default=0.0, help="fixed vx in policy range [-1, 1]")
    parser.add_argument("--vy", type=float, default=0.0, help="fixed vy in policy range [-1, 1]")
    parser.add_argument("--wz", type=float, default=0.0, help="must remain zero for D5")
    parser.add_argument("--x-norm", type=float, default=0.0, help="normalized ABS_X value in [-1, 1]")
    parser.add_argument("--y-norm", type=float, default=0.0, help="normalized ABS_Y value in [-1, 1]")
    parser.add_argument("--mapping-confirmed", action="store_true", help="only use after the base-frame mapping is user-confirmed")
    parser.add_argument("--x-to-vy-sign", choices=(-1, 1), type=int, default=1)
    parser.add_argument("--y-to-vx-sign", choices=(-1, 1), type=int, default=1)
    parser.add_argument("--ticks", type=int, default=1, help="number of dry-run samples")
    parser.add_argument("--csv", type=Path, required=True, help="explicit CSV destination; do not place logs under Git")
    args = parser.parse_args()
    if args.ticks < 1:
        parser.error("--ticks must be at least 1")
    return args


def sample(args: argparse.Namespace, timestamp_s: float) -> CommandRecord:
    if args.source == "fixed":
        return fixed_command(timestamp_s=timestamp_s, vx=args.vx, vy=args.vy, wz=args.wz)
    return controller_command(
        timestamp_s=timestamp_s,
        x_norm=args.x_norm,
        y_norm=args.y_norm,
        mapping_confirmed=args.mapping_confirmed,
        x_to_vy_sign=args.x_to_vy_sign,
        y_to_vx_sign=args.y_to_vx_sign,
    )


def main() -> int:
    args = parse_args()
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(CommandRecord.__dataclass_fields__)
    try:
        with args.csv.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=fieldnames)
            writer.writeheader()
            for _ in range(args.ticks):
                record = sample(args, time.time())
                writer.writerow(record.csv_row())
                output.flush()
                print(record.csv_row())
    except ValueError as error:
        print(f"refused: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Read-only preflight for ver9_jetson_deploy.py; never opens CAN or sends frames."""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Jetson deployment preflight; no CAN transmit.")
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--device", required=True, help="current Bluetooth /dev/input/eventN")
    args = parser.parse_args()
    try:
        import can  # noqa: F401
        import evdev  # noqa: F401
        import numpy  # noqa: F401
        import onnxruntime as ort
        import pyrealsense2  # noqa: F401
        from policy_integration import HPolicy
    except ImportError as error:
        print(f"ERROR: missing Jetson runtime dependency: {error}", file=sys.stderr)
        return 1
    if not os.path.exists(args.device):
        print(f"ERROR: Bluetooth controller event device not found: {args.device}", file=sys.stderr)
        return 1
    available = tuple(ort.get_available_providers())
    if not any(provider in available for provider in ("TensorrtExecutionProvider", "CUDAExecutionProvider")):
        print(f"ERROR: GPU ONNX Runtime provider is unavailable: {available}", file=sys.stderr)
        return 1
    try:
        policy = HPolicy(args.package, providers=["TensorrtExecutionProvider", "CUDAExecutionProvider"])
    except Exception as error:
        print(f"ERROR: GPU policy initialization failed: {error}", file=sys.stderr)
        return 1
    if not any(provider in policy.providers for provider in ("TensorrtExecutionProvider", "CUDAExecutionProvider")):
        print(f"ERROR: ONNX session fell back from GPU: {policy.providers}", file=sys.stderr)
        return 1
    print(f"OK: device={args.device} providers={policy.providers} package={args.package.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

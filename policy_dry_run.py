"""Verify a deployment package without opening any hardware device.

This program only reads a package containing policy.onnx and golden.npz.  It
does not import CAN, USB, T265, or controller libraries.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import onnxruntime as ort


EXPECTED_FILES = frozenset(
    {
        "policy.onnx",
        "obs_contract.md",
        "golden.npz",
        "verify_onnx.md",
        "actuator_table.md",
        "DEPLOY_README.md",
    }
)
JOINT_NAMES = (
    "LL_HR", "LR_HR", "LL_HAA", "LR_HAA", "LL_HFE", "LR_HFE",
    "LL_KFE", "LR_KFE", "LL_FFE", "LR_FFE",
)
DEFAULT_JOINT_POS = np.array(
    [0.0, 0.0, 0.0, 0.0, -0.1745, -0.1745, 0.3491, 0.3491, -0.1745, -0.1745],
    dtype=np.float32,
)
ACTION_SCALE = 0.5
ERROR_THRESHOLD = 1e-4
STEP_COUNT = 500
CONTROL_PERIOD_S = 0.020


class DryRunError(RuntimeError):
    """A package does not meet the dry-run contract."""


@dataclass(frozen=True)
class GoldenData:
    obs_flat: np.ndarray
    action_raw: np.ndarray
    joint_target: np.ndarray
    default_joint_pos: np.ndarray


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_manifest(package: Path) -> None:
    """Verify exactly the six deployment files listed in SHA256SUMS.txt."""
    manifest = package / "SHA256SUMS.txt"
    if not manifest.is_file():
        raise DryRunError(f"manifest not found: {manifest}")

    entries: dict[str, str] = {}
    for line_number, raw_line in enumerate(manifest.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 2 or len(fields[0]) != 64:
            raise DryRunError(f"invalid manifest line {line_number}: {raw_line!r}")
        checksum, name = fields
        if any(char not in "0123456789abcdefABCDEF" for char in checksum):
            raise DryRunError(f"invalid SHA256 on manifest line {line_number}")
        if name in entries:
            raise DryRunError(f"duplicate manifest entry: {name}")
        entries[name] = checksum.lower()

    if set(entries) != EXPECTED_FILES:
        missing = sorted(EXPECTED_FILES - set(entries))
        extra = sorted(set(entries) - EXPECTED_FILES)
        raise DryRunError(f"manifest must contain the six required files (missing={missing}, extra={extra})")

    for name, expected in entries.items():
        path = package / name
        if not path.is_file():
            raise DryRunError(f"required package file not found: {path}")
        actual = sha256(path)
        if actual != expected:
            raise DryRunError(f"SHA256 mismatch: {name} (expected {expected}, got {actual})")


def require_array(data: Any, key: str, shape: tuple[int, ...], dtype: np.dtype[Any]) -> np.ndarray:
    if key not in data:
        raise DryRunError(f"golden.npz key missing: {key}")
    value = data[key]
    if value.shape != shape or value.dtype != dtype:
        raise DryRunError(
            f"golden.npz {key} must be shape={shape}, dtype={dtype}; "
            f"got shape={value.shape}, dtype={value.dtype}"
        )
    if not np.isfinite(value).all():
        raise DryRunError(f"golden.npz {key} contains non-finite values")
    return value


def load_golden(package: Path) -> GoldenData:
    golden_path = package / "golden.npz"
    try:
        with np.load(golden_path, allow_pickle=False) as data:
            obs_flat = require_array(data, "obs_flat", (STEP_COUNT, 42), np.dtype(np.float32))
            action_raw = require_array(data, "action_raw", (STEP_COUNT, 10), np.dtype(np.float32))
            joint_target = require_array(data, "joint_target", (STEP_COUNT, 10), np.dtype(np.float32))
            default_joint_pos = require_array(data, "default_joint_pos", (10,), np.dtype(np.float32))
            joint_names = data["joint_names"] if "joint_names" in data else None
            action_scale = data["action_scale"] if "action_scale" in data else None
    except (OSError, ValueError) as exc:
        raise DryRunError(f"could not read golden.npz: {exc}") from exc

    if joint_names is None or joint_names.shape != (10,) or joint_names.dtype.kind not in "SU":
        raise DryRunError("golden.npz joint_names must be a 10-element string array")
    if tuple(joint_names.tolist()) != JOINT_NAMES:
        raise DryRunError(f"golden.npz joint_names order differs from contract: {joint_names.tolist()}")
    if action_scale is None or action_scale.shape != () or action_scale.dtype.kind != "f":
        raise DryRunError("golden.npz action_scale must be a floating scalar")
    if not np.isclose(float(action_scale), ACTION_SCALE, rtol=0.0, atol=0.0):
        raise DryRunError(f"golden.npz action_scale must be {ACTION_SCALE}, got {action_scale}")
    if not np.allclose(default_joint_pos, DEFAULT_JOINT_POS, rtol=0.0, atol=1e-6):
        raise DryRunError("golden.npz default_joint_pos differs from the deployment contract")
    expected_target = default_joint_pos + np.float32(ACTION_SCALE) * action_raw
    if not np.allclose(joint_target, expected_target, rtol=0.0, atol=1e-6):
        raise DryRunError("golden.npz joint_target does not equal default_joint_pos + 0.5 * action_raw")
    return GoldenData(obs_flat, action_raw, joint_target, default_joint_pos)


def validate_session(model_path: Path) -> tuple[ort.InferenceSession, str, str]:
    try:
        session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    except Exception as exc:  # onnxruntime's exception hierarchy differs by version.
        raise DryRunError(f"could not load ONNX model: {exc}") from exc
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if len(inputs) != 1 or len(outputs) != 1:
        raise DryRunError(f"ONNX must have one input and one output; got {len(inputs)} and {len(outputs)}")
    model_input, model_output = inputs[0], outputs[0]
    if model_input.type != "tensor(float)" or model_output.type != "tensor(float)":
        raise DryRunError(f"ONNX tensors must be float32; got {model_input.type} -> {model_output.type}")
    if len(model_input.shape) != 2 or len(model_output.shape) != 2:
        raise DryRunError(f"ONNX tensors must be rank 2; got {model_input.shape} -> {model_output.shape}")
    if model_input.shape[-1] != 42 or model_output.shape[-1] != 10:
        raise DryRunError(f"ONNX must be 42 -> 10; got {model_input.shape} -> {model_output.shape}")
    return session, model_input.name, model_output.name


def max_error(actual: np.ndarray, expected: np.ndarray) -> tuple[float, int]:
    errors = np.max(np.abs(actual - expected), axis=1)
    step = int(np.argmax(errors))
    return float(errors[step]), step


def run(package: Path, realtime: bool, csv_path: Path | None) -> tuple[float, int, float, int]:
    verify_manifest(package)
    golden = load_golden(package)
    session, input_name, output_name = validate_session(package / "policy.onnx")
    action_actual = np.empty_like(golden.action_raw)
    timing_rows: list[dict[str, float | int]] = []
    start = time.perf_counter()
    previous_start: float | None = None
    for step, observation in enumerate(golden.obs_flat):
        scheduled = start + step * CONTROL_PERIOD_S
        if realtime:
            wait = scheduled - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
        actual_start = time.perf_counter()
        output = session.run([output_name], {input_name: observation.reshape(1, 42).astype(np.float32, copy=False)})[0]
        inference_end = time.perf_counter()
        if output.shape != (1, 10) or output.dtype != np.float32:
            raise DryRunError(f"ONNX returned shape={output.shape}, dtype={output.dtype}; expected (1, 10), float32")
        action_actual[step] = output[0]
        if realtime:
            timing_rows.append(
                {
                    "step": step,
                    "scheduled_s": step * CONTROL_PERIOD_S,
                    "actual_start_s": actual_start - start,
                    "deadline_delta_ms": (actual_start - scheduled) * 1000.0,
                    "inference_ms": (inference_end - actual_start) * 1000.0,
                    "period_ms": float("nan") if previous_start is None else (actual_start - previous_start) * 1000.0,
                }
            )
        previous_start = actual_start

    target_actual = golden.default_joint_pos + np.float32(ACTION_SCALE) * action_actual
    action_error, action_step = max_error(action_actual, golden.action_raw)
    target_error, target_step = max_error(target_actual, golden.joint_target)
    if realtime and csv_path is not None:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="", encoding="utf-8") as destination:
            writer = csv.DictWriter(destination, fieldnames=list(timing_rows[0]))
            writer.writeheader()
            writer.writerows(timing_rows)
    return action_error, action_step, target_error, target_step


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run ONNX golden verification without hardware access.")
    parser.add_argument("--package", required=True, type=Path, help="read-only deployment package directory")
    parser.add_argument("--realtime", action="store_true", help="replay the 500 observations on a 20 ms schedule")
    parser.add_argument("--csv", type=Path, help="timing CSV path; requires --realtime")
    args = parser.parse_args()
    if args.csv is not None and not args.realtime:
        parser.error("--csv requires --realtime")
    return args


def main() -> int:
    args = parse_args()
    package = args.package.resolve()
    if not package.is_dir():
        print(f"ERROR: package directory not found: {package}", file=sys.stderr)
        return 2
    csv_path = args.csv
    if args.realtime and csv_path is None:
        csv_path = Path.cwd() / "logs" / f"policy_dry_run_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        action_error, action_step, target_error, target_step = run(package, args.realtime, csv_path)
    except DryRunError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"action max_abs_error={action_error:.9g} worst_step={action_step}")
    print(f"joint_target max_abs_error={target_error:.9g} worst_step={target_step}")
    if csv_path is not None:
        print(f"timing_csv={csv_path.resolve()}")
    if action_error > ERROR_THRESHOLD or target_error > ERROR_THRESHOLD:
        print(f"FAIL: threshold is {ERROR_THRESHOLD:g}", file=sys.stderr)
        return 1
    print(f"PASS: both errors are <= {ERROR_THRESHOLD:g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

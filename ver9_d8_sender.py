#!/usr/bin/env python3
"""D8: 10-axis, dual-CAN MIT sender.  Hardware transmission requires --arm."""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import NamedTuple, Optional

import numpy as np

import cubemars as cm
from motor_console_ver8_2 import f_mit, quantized_cmd
from policy_integration import ACTION_SIZE, HPolicy
from robot_joint_map import BY_ID as H_BINDING_BY_ID
from ver9_integration import H_CAN_IDS, H_MODELS, evaluate_cycle
from ver9_shell import FixedCommandSource, MotorFeedback, RealT265, T265_R_OFFSET_M, VelocityCommand, servo_feedback_to_h_units

HZ = 50.0
PERIOD = 1.0 / HZ
BUILD_ID = "D10_3_AXISLOG_20260923_1200"
STALE_S = 0.30
# gs_usb resets its USB interface when a Bus is started.  The second adapter
# needs this full pause after the first one; otherwise python-can may emit a
# GsUsbBus finalizer warning while its own constructor is unwinding.
OPEN_SETTLE_S = 2.0
OPEN_RETRY_S = 1.5
CURRENT_ABORT_A = 1.0
SPEED_ABORT_RAD_S = np.deg2rad(100.0)
# 2026-09-23, from the D10-2 post mortem (reports/2026-09-23_d10-2_result.md).
# CURRENT_ABORT_A = 1.0 A was chosen for a ONE-AXIS probe on a machine where
# nothing carried load, and it is BELOW this robot's own static gravity load,
# so it cannot be used unchanged for a run that drives several axes at once:
#   * at the URDF zero pose, hanging, each HAA axis needs 2.1 N*m = 1.7 A just
#     to hold its own leg -- 1.7x the abort while the robot does nothing;
#   * over the reachable set, HFE needs up to 3.85 N*m = 7.4 A.
# All five D10-2 runs died inside the ramp for exactly this reason (every CSV
# holds stage "ramp" only, 1.4-6.6 s of a 15 s ramp; policy never entered).
# The table below is the worst-case static gravity current per axis, computed
# from onshape_export/myrobot_dummy/robot_sim.urdf with the measured c_p,
# times a 1.5 margin, rounded up.  It stays far below what the trained policy
# is itself allowed to use (AK10-9 53 N*m = 42.1 A, AK80-9 13.5 N*m = 25.8 A),
# so it is still a guard and not a licence.
# It is OPT-IN: --gravity-limits has to be typed, it refuses without
# --all-axes, and every one-axis path keeps CURRENT_ABORT_A untouched.
GRAVITY_CURRENT_ABORT_A_BY_JOINT = {
    "HR":   3.0,   # static worst case 2.45 N*m = 1.95 A (AK10-9)
    "HAA":  6.0,   # static worst case 4.75 N*m = 3.78 A (AK10-9)
    "HFE": 11.0,   # static worst case 3.85 N*m = 7.36 A (AK80-9)
    "KFE":  3.0,   # static worst case 1.17 N*m = 0.93 A (AK10-9)
    "FFE":  2.0,   # static worst case 0.10 N*m = 0.20 A (AK80-9)
}
# A D7 `o 0` is temporary across a power cycle.  Do not arm a policy when
# feedback is plainly not in that freshly zeroed reference frame.
ORIGIN_ABORT_RAD = np.deg2rad(45.0)
# A completed one-axis run is not evidence of position control if feedback
# never departs its D7 origin by even one servo-feedback display increment.
MIN_TRACKING_DEG = 0.1
MIN_TRACKING_RAD = np.deg2rad(MIN_TRACKING_DEG)
# A current above this small, observed-noise-safe level with no position
# feedback change distinguishes a loaded/stiction stall from a missing MIT
# torque response.  It is diagnostic only; it never adds torque.
STALL_CURRENT_A = 0.10
# A static probe separates breakaway/stiction from the policy/T265 path.  Its
# angle is not a tuning knob: with |I| = Kp * error, the angle is simply how
# much breakaway current the probe can reach, and CURRENT_ABORT_A caps that.
# 2026-09-22 (user approved): raised from 2.5 to 7.0 deg because 0x1C stayed
# still at 0.66 A, which 2.5 deg (0.35 A) could never exceed.  The per-axis
# limit below is the binding one; this is only the absolute ceiling.
STATIC_PROBE_MAX_DEG = 7.0
# Measured scatter of the reported |I| about Kp*error while the commanded
# error is constant (D9-3 hold phase, 2026-09-22: command 0.87 A, samples
# 0.79-0.93 A).  It is feedback noise, not extra torque, and it has two
# consequences that belong here rather than in the operator's head.  A probe
# must command far enough below CURRENT_ABORT_A that a noise peak cannot trip
# the abort, and no physical bound may be read off a single peak sample.
FEEDBACK_CURRENT_NOISE_A = 0.08
# The servo position feedback is quantized to 0.1 deg, so one or two changed
# increments are quantization dither, elastic wind-up or backlash take-up --
# not rotation.  D9-3 measured exactly that: 0.200 deg of non-monotone dither
# over 10 s, ending 0.2 deg from its start while 0.87 A was held.  A 0.1 deg
# floor cannot tell that from breakaway, so breakaway now means five feedback
# increments of *net, sustained* displacement toward the target.
# NOTE: this moves the D9-3 table's B/C boundary after the run.  It is a
# deliberate, documented change (reports/2026-09-22_d9-3_result.md) because
# the old boundary sat on the measurement's own resolution, and it moves the
# 0.200 deg result toward the more cautious action, not the convenient one.
FEEDBACK_POSITION_LSB_DEG = 0.1
BREAKAWAY_MIN_DEG = 0.5
BREAKAWAY_MIN_RAD = np.deg2rad(BREAKAWAY_MIN_DEG)
# Seeing one encoder increment is insufficient for a fixed-target probe.  It
# must cover a meaningful portion of the requested relative displacement.
STATIC_PROBE_MIN_TRACKING_FRACTION = 0.50
# Where the axis ENDED UP during the hold, not its average over the hold.  A
# breakaway in the last second leaves the median sitting on the stationary
# part before it.  D9-6 scored a real 3.5 deg move as 0.2 deg that way.
HOLD_TAIL_FRACTION = 0.25
# Samples to look back over for the current the axis was holding just before
# it let go.  0.2 s at 50 Hz.
BREAKAWAY_LOOKBACK_SAMPLES = 10
# The same criterion applies to the frozen first policy target: a one-axis
# policy ramp has not succeeded when it moves only one encoder increment.
POLICY_RAMP_MIN_TRACKING_FRACTION = 0.50
# 2026-09-22, measured from the two saved 0x1C runs (see
# reports/2026-09-22_d9-0x1c-current-vs-error.md): the MIT Kp field acts as
# "amps of phase current per radian of position error", with no model factor.
# A least squares fit of |I| against |error| gave 7.918 and 7.553 A/rad for a
# commanded Kp of 7.937, intercept |b| <= 0.02 A.  Physical torque is that
# current times Kt, and Kt is still unmeasured for the AK10-9, so every
# decision below is expressed in current so that it does not depend on Kt.
CURRENT_PER_KP_A_PER_RAD = 1.0
# Slope of |I| over error, divided by the commanded Kp.  Inside this band the
# MIT torque path is doing its job and a motionless axis is a mechanical fact;
# near zero the command is not reaching the motor at all.
TORQUE_PATH_SLOPE_TOLERANCE = 0.25
TORQUE_PATH_ABSENT_RATIO = 0.25
TORQUE_PATH_MIN_SAMPLES = 20
TORQUE_PATH_MIN_ERROR_RAD = np.deg2rad(0.5)
# A position loop can only move an axis where Kp * error exceeds its breakaway
# current.  A fixed-target run therefore probes breakaway only up to
# Kp * |target - position|.  A run whose ceiling is at or below a current that
# already left this axis stationary cannot produce new information, whatever
# its target angle, so it is refused before a single MIT frame.  Raising Kp,
# the target or the torque field to clear this guard is not permitted; the
# mechanical load or the diagnostic itself has to change.
# Sustained current each axis has already held without breaking away.
# 2026-09-22 D9-3 raised 0x1C from 0.66 to 0.86 A (logs/d9_3_breakaway_0x1c.csv,
# sustained over the hold stage).  With Kp 7.937 the largest angle the current
# abort allows is 6.64 deg, i.e. 0.92 A commanded, so this method has almost no
# headroom left and a repeat of the 6.5 deg run is now refused before any send.
# 2026-09-22 D9-4/D9-6: a stall is evidence about one axis *in one direction*,
# so the record is keyed that way.  It is now EMPTY: D9-6 broke 0x1C away in
# the positive direction at 0.90 A (logs/d9_6a_hr_pos_supported_0x1c.csv, net
# +3.50 deg), so "0x1C stays stationary at 0.86 A" is no longer true and must
# not refuse further runs.  The earlier C verdicts were a scoring bug, not a
# stalled axis: the axis let go in the last second of the hold and the median
# over the whole hold sat on the stationary part before it.
# Add an entry only for an axis that genuinely held a current without moving,
# and delete it when the mechanical load changes; do not raise Kp, the angle,
# the torque field or the abort to get past this guard.
DEMONSTRATED_STALL_CURRENT_A = {}
STALL_GUARD_MARGIN_A = 0.05
# H deployment stiffness/damping, converted with the measured c_p/c_d.
STIFFNESS = np.array((10,10,15,15,15,15,15,15,10,10), dtype=float)
DAMPING = np.full(10, 1.5, dtype=float)
CP = {"AK80-9": .523, "AK10-9": 1.258}
CD = {"AK80-9": .523, "AK10-9": 1.216}

def gains():
    return tuple((float(STIFFNESS[i]/CP[m]), float(DAMPING[i]/CD[m])) for i,m in enumerate(H_MODELS))


def joint_suffix(motor_id):
    """HR / HAA / HFE / KFE / FFE for one registered CAN id."""
    return H_BINDING_BY_ID[motor_id].name.split("_", 1)[1]


def default_current_limits():
    """The one-axis probe limit, unchanged, applied to every registered axis."""
    return {mid: CURRENT_ABORT_A for mid in H_CAN_IDS}


def gravity_current_limits():
    """Per-axis limits that clear this robot's own static gravity load."""
    return {mid: GRAVITY_CURRENT_ABORT_A_BY_JOINT[joint_suffix(mid)]
            for mid in H_CAN_IDS}


def rows_for_axis(rows, motor_id):
    """The CSV rows belonging to one axis of a multi-axis run."""
    label = f"0x{motor_id:02X}"
    return [row for row in rows if row[2] == label]


def axis_rows(tick, stage, desired, requested, bus, motor_ids):
    """One CSV row per DRIVEN axis, not just the first one.

    D10-2 drove ten axes and logged one: 0x1C, which happens to be both the
    first entry of H_CAN_IDS and the heaviest axis measured so far, and whose
    first policy target sat inside its own 6.5 deg deadband.  The question the
    judging table actually asks -- which axis failed to reach its target --
    therefore had no evidence at all.  This costs ten rows per tick and
    answers it.
    """
    out = []
    for mid in motor_ids:
        index = H_CAN_IDS.index(mid)
        state = bus.state(mid)
        position, velocity = servo_feedback_to_h_units(state.pos, state.spd)
        _kp, _kd, wire_target, _vel, _tau = wire_command(mid, requested[index])
        out.append((tick, stage, f"0x{mid:02X}", float(desired[index]),
                    float(requested[index]), wire_target,
                    position, velocity, state.cur))
    return out

def frames(targets, motor_ids=H_CAN_IDS):
    if len(targets) != ACTION_SIZE: raise ValueError("need ten targets")
    index_by_id = {motor_id: index for index, motor_id in enumerate(H_CAN_IDS)}
    if not motor_ids or any(motor_id not in index_by_id for motor_id in motor_ids):
        raise ValueError("motor_ids must be registered H CAN IDs")
    return tuple(
        f_mit(motor_id, *gains()[index_by_id[motor_id]],
              float(targets[index_by_id[motor_id]]), 0., 0.,
              H_MODELS[index_by_id[motor_id]])
        for motor_id in motor_ids
    )

def zero_frames(motor_ids=H_CAN_IDS):
    model_by_id = dict(zip(H_CAN_IDS, H_MODELS))
    return tuple(f_mit(mid, 0., 0., 0., 0., 0., model_by_id[mid]) for mid in motor_ids)


def wire_command(motor_id, target_rad):
    """Return the quantized MIT values, including the position on the wire."""
    index = H_CAN_IDS.index(motor_id)
    kp, kd = gains()[index]
    return quantized_cmd((kp, kd, float(target_rad), 0.0, 0.0), H_MODELS[index])


def ramp_targets(start, target, elapsed_s, ramp_seconds):
    """Linearly approach one frozen policy target without a first-frame step."""
    if ramp_seconds <= 0:
        raise ValueError("ramp_seconds must be positive")
    start = np.asarray(start, dtype=float)
    target = np.asarray(target, dtype=float)
    if start.shape != (ACTION_SIZE,) or target.shape != (ACTION_SIZE,):
        raise ValueError("ramp needs ten start and target angles")
    alpha = min(1.0, max(0.0, elapsed_s / ramp_seconds))
    return tuple(start + alpha * (target - start))


def slew_target(previous, desired, max_rate_rad_s, elapsed_s):
    """Rate-limit one commanded joint target without changing policy output."""
    if max_rate_rad_s < 0 or elapsed_s < 0:
        raise ValueError("slew rate and elapsed time must be nonnegative")
    max_step = max_rate_rad_s * elapsed_s
    return float(np.clip(desired, previous - max_step, previous + max_step))


def tracking_summary(rows, initial_position_rad):
    """Summarise one selected-axis run without inferring a torque direction."""
    if not rows:
        return 0.0, 0.0, "no selected-axis feedback samples"
    max_movement = max(abs(row[6] - initial_position_rad) for row in rows)
    max_current = max(abs(row[8]) for row in rows)
    if max_movement < MIN_TRACKING_RAD:
        if max_current >= STALL_CURRENT_A:
            verdict = "position stalled under load (current present; static friction/mechanical load suspected)"
        else:
            verdict = "no position response and no meaningful current (MIT torque response unproven)"
    else:
        verdict = "position tracking observed"
    return max_movement, max_current, verdict


class ProbeResult(NamedTuple):
    """One static probe, scored.  Not moving is a result, never an error."""

    net_rad: float
    excursion_rad: float
    sustained_current_a: float
    peak_current_a: float
    required_rad: float
    breakaway_current_a: Optional[float]
    letter: str
    verdict: str


def held_rows(rows):
    """The samples taken at the commanded target: hold stage, else last half."""
    hold = [row for row in rows if row[1].endswith("hold")]
    if hold:
        return hold
    return rows[len(rows) // 2:] or list(rows)


def median(values):
    ordered = sorted(values)
    if not ordered:
        return 0.0
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return float(ordered[middle])
    return float((ordered[middle - 1] + ordered[middle]) / 2.0)


def sustained_current_a(rows):
    """|I| the axis actually held, robust to a single noisy feedback sample."""
    return median([abs(row[8]) for row in held_rows(rows)])


def hold_tail_rows(rows):
    """The last part of the hold: where the axis ended up, not its average.

    2026-09-22 D9-6: three runs broke away in the final second of the hold and
    then ran toward the target while the error, and so the current, collapsed.
    A median over the whole hold is dominated by the stationary part before
    that, so it scored a 3.5 deg breakaway as 0.2 deg of dither.  Where the
    axis finished is the measurement; how long it took to let go is not.
    """
    hold = held_rows(rows)
    tail = max(1, int(round(len(hold) * HOLD_TAIL_FRACTION)))
    return hold[-tail:]


def breakaway_current_a(rows, initial_position_rad, target_delta_rad):
    """|I| the axis was holding immediately BEFORE it let go, if it did.

    Not the current at the first moved sample: by then the axis is already
    running and Kp*error, hence the current, has collapsed.  D9-6 read 0.02 A
    off 0x13 that way while it had been holding about 0.3 A one sample earlier.
    """
    sign = 1.0 if target_delta_rad >= 0 else -1.0
    for index, row in enumerate(rows):
        if sign * (row[6] - initial_position_rad) >= BREAKAWAY_MIN_RAD:
            window = rows[max(0, index - BREAKAWAY_LOOKBACK_SAMPLES):index]
            if not window:
                return abs(row[8])
            return max(abs(sample[8]) for sample in window)
    return None


def probe_result(rows, initial_position_rad, target_delta_rad):
    """Score one static probe against the D9-3 table, without raising.

    Displacement is the *net, sustained* motion toward the requested target,
    not the largest excursion: a feedback increment flickering up and down is
    not rotation, and the probe exists to measure breakaway, so an axis that
    stays put is a measurement rather than a failure of the run.
    """
    if not rows:
        return ProbeResult(0.0, 0.0, 0.0, 0.0, 0.0, None, "D",
                           "no selected-axis feedback samples")
    sign = 1.0 if target_delta_rad >= 0 else -1.0
    net = sign * (median([row[6] for row in hold_tail_rows(rows)]) - initial_position_rad)
    excursion = max(abs(row[6] - initial_position_rad) for row in rows)
    sustained = sustained_current_a(rows)
    peak = max(abs(row[8]) for row in rows)
    required = max(BREAKAWAY_MIN_RAD,
                   abs(target_delta_rad) * STATIC_PROBE_MIN_TRACKING_FRACTION)
    breakaway = breakaway_current_a(rows, initial_position_rad, target_delta_rad)
    if net >= required:
        letter = "A"
        verdict = "broke away and covered the requested displacement"
    elif net >= BREAKAWAY_MIN_RAD:
        letter = "B"
        verdict = "broke away but stopped short of the requested displacement"
    elif sustained >= STALL_CURRENT_A:
        letter = "C"
        verdict = ("no breakaway: net motion stayed inside the quantization and "
                   "wind-up band while the commanded current was held")
    else:
        letter = "D"
        verdict = "no breakaway and no meaningful current (MIT torque response unproven)"
    return ProbeResult(net, excursion, sustained, peak, required, breakaway,
                       letter, verdict)


def policy_ramp_summary(rows, initial_position_rad, initial_target_rad):
    """Require meaningful tracking of the frozen first policy-ramp target."""
    movement, current, verdict = tracking_summary(rows, initial_position_rad)
    required = max(
        MIN_TRACKING_RAD,
        abs(initial_target_rad - initial_position_rad) * POLICY_RAMP_MIN_TRACKING_FRACTION,
    )
    if movement < required:
        if current >= STALL_CURRENT_A:
            verdict = "position stalled during initial policy ramp (current present; static friction/mechanical load suspected)"
        else:
            verdict = "no meaningful policy-ramp response (MIT torque response unproven)"
    return movement, current, required, verdict


def probe_ceiling_current_a(kp_cmd, requested_delta_rad):
    """Largest |I| a fixed-target run can reach while the axis stays put."""
    reachable = abs(kp_cmd) * CURRENT_PER_KP_A_PER_RAD * abs(requested_delta_rad)
    return min(reachable, CURRENT_ABORT_A)


def max_probe_angle_deg(kp_cmd):
    """Largest probe angle this axis can ask for without tripping the abort.

    The abort watches the reported |I|, which scatters above the commanded
    Kp*error, so the commanded ceiling leaves a full noise peak of room.
    """
    if kp_cmd <= 0:
        raise ValueError("kp must be positive")
    reachable = ((CURRENT_ABORT_A - FEEDBACK_CURRENT_NOISE_A)
                 / (kp_cmd * CURRENT_PER_KP_A_PER_RAD))
    return min(STATIC_PROBE_MAX_DEG, float(np.rad2deg(reachable)))


def probe_direction(requested_delta_rad):
    """+1 or -1: which way this probe pushes.  Stall evidence is per direction."""
    return 1 if requested_delta_rad >= 0 else -1


def direction_label(direction):
    return "positive" if direction >= 0 else "negative"


def demonstrated_stall_current_a(motor_id, requested_delta_rad):
    """|I| this axis has already held *on this side* without breaking away."""
    return DEMONSTRATED_STALL_CURRENT_A.get(
        (motor_id, probe_direction(requested_delta_rad))
    )


def stall_guard(motor_id, kp_cmd, requested_delta_rad):
    """Refuse, before any MIT frame, a run that cannot move a healthy axis."""
    ceiling = probe_ceiling_current_a(kp_cmd, requested_delta_rad)
    direction = probe_direction(requested_delta_rad)
    detail = (
        f"probe ceiling |I| <= {ceiling:.2f}A for {np.rad2deg(requested_delta_rad):+.2f}deg "
        f"at Kp={kp_cmd:.3f}; this run can only move 0x{motor_id:02X} if its breakaway "
        f"current is below that, i.e. if its deadband is under "
        f"{np.rad2deg(abs(requested_delta_rad)):.2f}deg. That ceiling is the "
        f"commanded current; reported samples scatter about it by roughly "
        f"+/-{FEEDBACK_CURRENT_NOISE_A:.2f}A."
    )
    known = demonstrated_stall_current_a(motor_id, requested_delta_rad)
    if known is not None and ceiling <= known + STALL_GUARD_MARGIN_A:
        opposite = DEMONSTRATED_STALL_CURRENT_A.get((motor_id, -direction))
        hint = (
            ""
            if opposite is not None
            else (
                f" The {direction_label(-direction)} direction has not been probed; "
                "if the mechanism can only be blocked on one side, that run is new "
                "information rather than a repeat."
            )
        )
        raise RuntimeError(
            f"repeat-probe abort 0x{motor_id:02X}: {detail} That axis already stayed "
            f"stationary at {known:.2f}A in the {direction_label(direction)} "
            f"direction, so this run repeats a known result. Do not raise Kp, the "
            "target angle or the torque field to get past this; change the mechanical "
            f"load, or run a separately reviewed diagnostic.{hint}"
        )
    return ceiling, detail


def torque_path_summary(rows, kp_cmd):
    """Judge the MIT torque path itself, from |I| against commanded error.

    This is independent of whether the axis moved.  While the axis is
    stationary the commanded position error is known exactly, so the slope of
    |I| over that error says whether Kp reached the motor at all.  It needs no
    Kt, adds no torque and sends nothing.
    """
    samples = [(abs(row[5] - row[6]), abs(row[8])) for row in rows]
    samples = [pair for pair in samples if pair[0] >= TORQUE_PATH_MIN_ERROR_RAD]
    if kp_cmd <= 0 or len(samples) < TORQUE_PATH_MIN_SAMPLES:
        return None, None, "torque path undetermined (too few loaded samples)"
    count = len(samples)
    sum_e = sum(error for error, _ in samples)
    sum_i = sum(current for _, current in samples)
    sum_ee = sum(error * error for error, _ in samples)
    sum_ei = sum(error * current for error, current in samples)
    denominator = count * sum_ee - sum_e * sum_e
    if denominator <= 0:
        return None, None, "torque path undetermined (no spread in position error)"
    slope = (count * sum_ei - sum_e * sum_i) / denominator
    ratio = slope / (kp_cmd * CURRENT_PER_KP_A_PER_RAD)
    if abs(ratio - 1.0) <= TORQUE_PATH_SLOPE_TOLERANCE:
        verdict = "MIT torque path verified (|I| follows Kp*error)"
    elif ratio <= TORQUE_PATH_ABSENT_RATIO:
        verdict = "MIT torque absent (|I| does not follow Kp*error; route/mode/gain suspect)"
    else:
        verdict = "MIT torque path anomalous (|I| slope disagrees with the commanded Kp)"
    return slope, ratio, verdict


def deadband_lower_bound_rad(max_current_a, kp_cmd):
    """Breakaway deadband implied by a stationary axis, without using Kt."""
    if kp_cmd <= 0:
        return None
    return abs(max_current_a) / (kp_cmd * CURRENT_PER_KP_A_PER_RAD)


def report_torque_path(motor_id, rows, kp_cmd):
    """Print the Kt-free reading of the MIT torque path itself."""
    slope, ratio, verdict = torque_path_summary(rows, kp_cmd)
    if slope is None:
        print(f"TORQUE PATH: 0x{motor_id:02X} {verdict}")
        return verdict
    print(
        f"TORQUE PATH: 0x{motor_id:02X} |I|/error={slope:.2f}A/rad "
        f"(Kp={kp_cmd:.3f}, ratio={ratio:.2f}); {verdict}"
    )
    return verdict


def report_deadband(motor_id, kp_cmd, sustained_a, breakaway_a):
    """State the deadband from the sustained current, never from a peak.

    A peak sample is feedback noise; reading a physical bound off it
    overstates the deadband, which is what the D9-3 screen log did.
    """
    if kp_cmd <= 0:
        return
    if breakaway_a is not None:
        bound = deadband_lower_bound_rad(breakaway_a, kp_cmd)
        print(
            f"BREAKAWAY: 0x{motor_id:02X} started moving at |I|={breakaway_a:.2f}A, "
            f"so its position deadband at this Kp is {np.rad2deg(bound):.2f}deg."
        )
        return
    bound = deadband_lower_bound_rad(sustained_a, kp_cmd)
    print(
        f"DEADBAND: 0x{motor_id:02X} held a sustained {sustained_a:.2f}A without "
        f"breaking away, so its position deadband at this Kp is at least "
        f"{np.rad2deg(bound):.2f}deg (sustained current, not a peak sample)."
    )


# What each D9-3 letter means for the next action.  These are the pre-agreed
# steps from handoffs/2026-09-22_d9-3_ブレークアウェイ判定.md.
D9_3_NEXT_STEP = {
    "A": ("ordinary stiction; 0x1C is healthy. Record the breakaway current and "
          "deadband, then judge whether the policy survives that deadband before "
          "any single-leg step."),
    "B": ("the axis did break away, so it is not a mechanical fault. Record the "
          "breakaway current and deadband. Do not repeat this run."),
    "C": ("main power OFF. It did not break away at this current, so treat it as "
          "mechanical: interference, fasteners, cable, support. This says nothing "
          "about the other direction -- a hard stop blocks one side only, so check "
          "both by hand with the power off before calling the axis faulty. Do not "
          "raise Kp, the target angle or the current abort."),
    "D": ("main power OFF. |I| = Kp*error does not hold here, so the premise of "
          "this test is gone. Go back to the electrical and communication side "
          "and leave the mechanism alone."),
}
# Exit codes, so a verdict is legible to a script without a traceback.
D9_3_EXIT_CODE = {"A": 0, "B": 0, "C": 2, "D": 3}


def d9_3_letter(result, torque_verdict):
    """A verified torque path is the premise of A/B/C; without it the answer is D."""
    if "verified" not in torque_verdict:
        return "D"
    return result.letter


def report_probe(motor_id, rows, kp_cmd, result, requested_delta_rad=None):
    """Print the four D9-3 lines and return the scored letter."""
    increments = int(round(BREAKAWAY_MIN_DEG / FEEDBACK_POSITION_LSB_DEG))
    if requested_delta_rad is not None:
        direction = probe_direction(requested_delta_rad)
        print(
            f"PROBE DIRECTION: 0x{motor_id:02X} "
            f"{np.rad2deg(requested_delta_rad):+.2f}deg "
            f"({direction_label(direction)}); this run judges that side only."
        )
    print(
        f"STATIC PROBE: 0x{motor_id:02X} net sustained movement="
        f"{np.rad2deg(result.net_rad):+.3f}deg toward the target "
        f"(>= {np.rad2deg(result.required_rad):.3f}deg for A, "
        f">= {BREAKAWAY_MIN_DEG:.1f}deg for B; that floor is {increments} feedback "
        f"increments of {FEEDBACK_POSITION_LSB_DEG:.1f}deg); max excursion="
        f"{np.rad2deg(result.excursion_rad):.3f}deg; sustained |current|="
        f"{result.sustained_current_a:.2f}A (peak sample "
        f"{result.peak_current_a:.2f}A); {result.verdict}"
    )
    torque_verdict = report_torque_path(motor_id, rows, kp_cmd)
    report_deadband(motor_id, kp_cmd, result.sustained_current_a,
                    result.breakaway_current_a)
    letter = d9_3_letter(result, torque_verdict)
    reason = torque_verdict if letter == "D" else result.verdict
    print(f"VERDICT: D9-3 {letter} -- {reason}; next: {D9_3_NEXT_STEP[letter]}")
    return letter


class DualBus:
    """Both gs_usb channels plus a per-process route learned from feedback."""
    def __init__(self, current_limits=None):
        self.bus_by_channel = {
            channel: cm.MotorBus(channel=channel) for channel in (0, 1)
        }
        # Per-axis current abort.  The default is the unchanged one-axis limit
        # on every axis; only --gravity-limits replaces it, and only for a run
        # that has to carry the machine's own weight.
        self.current_limit_a = dict(current_limits or default_current_limits())
        self.route_by_motor_id = {}
        # This is deliberately separate from a route: a failed receive-only
        # preflight must close the adapter without injecting MIT frames.
        self.mit_frames_sent = False
        self.mit_motor_ids = set()

    def open(self):
        # See d7_origin_console.OPEN_ATTEMPTS: libusb0's reset fails on the
        # first open on this PC nearly every time, so two attempts left no
        # spare.  A failed open has sent no CAN frame and releases both
        # interfaces before the next try.
        for attempt in (1, 2, 3):
            try:
                for channel in (0, 1):
                    self.bus_by_channel[channel].open()
                    time.sleep(OPEN_SETTLE_S)
                return
            except Exception:
                # An open/reset failure has not armed MIT and must not emit a
                # CAN frame.  Release both interfaces before the one retry.
                self.close()
                if attempt == 3:
                    raise
                print(f"USB open failed; retrying once after {OPEN_RETRY_S:.1f}s (no CAN sent)")
                time.sleep(OPEN_RETRY_S)
    def close(self):
        for bus in self.bus_by_channel.values():
            bus.close(stop_motors=False)

    def discover_routes(self, timeout_s=5.0):
        """Require one fresh feedback source for every motor before any MIT send."""
        deadline = time.monotonic() + timeout_s
        while True:
            route = {}
            duplicate = set()
            for mid in H_CAN_IDS:
                sources = [
                    bus for bus in self.bus_by_channel.values()
                    if (state := bus.state(mid)) is not None and state.age() <= STALE_S
                ]
                if len(sources) == 1:
                    route[mid] = sources[0]
                elif len(sources) > 1:
                    duplicate.add(mid)
            missing = set(H_CAN_IDS) - set(route)
            if not missing and not duplicate:
                self.route_by_motor_id = route
                rendered = ", ".join(
                    f"0x{mid:02X}->ch{next(channel for channel, bus in self.bus_by_channel.items() if bus is route[mid])}"
                    for mid in H_CAN_IDS
                )
                print("CAN routes confirmed: " + rendered)
                return
            if time.monotonic() >= deadline:
                self.route_by_motor_id.clear()
                detail = ", ".join(f"0x{mid:02X}" for mid in sorted(missing | duplicate))
                raise RuntimeError(f"CAN route discovery failed: {detail}")
            time.sleep(0.01)

    def state(self, mid):
        try:
            return self.route_by_motor_id[mid].state(mid)
        except KeyError as error:
            raise RuntimeError(f"CAN route is not confirmed for 0x{mid:02X}") from error

    def send(self, frame):
        mid = frame.arbitration_id & 255
        try:
            sent = self.route_by_motor_id[mid].send(frame)
        except KeyError as error:
            raise RuntimeError(f"refusing MIT send before route confirmation for 0x{mid:02X}") from error
        if not sent:
            raise RuntimeError(f"MIT send failed 0x{mid:02X}")
        self.mit_frames_sent = True
        self.mit_motor_ids.add(mid)

    def channel_for(self, mid):
        try:
            return next(channel for channel, bus in self.bus_by_channel.items()
                        if bus is self.route_by_motor_id[mid])
        except (KeyError, StopIteration) as error:
            raise RuntimeError(f"CAN route is not confirmed for 0x{mid:02X}") from error

    @property
    def tx_count(self):
        """Total frames accepted by the two adapters in this process."""
        return sum(getattr(bus, "tx_count", 0) for bus in self.bus_by_channel.values())

    def feedback(self):
        now=time.monotonic(); out={}
        for mid in H_CAN_IDS:
            s=self.state(mid)
            channel = self.channel_for(mid)
            if s is None or time.time()-s.t > STALE_S:
                raise RuntimeError(f"stale feedback 0x{mid:02X} ch={channel}")
            if s.err: raise RuntimeError(f"motor error 0x{mid:02X} ch={channel}: {s.err}")
            p,v=servo_feedback_to_h_units(s.pos,s.spd)
            if abs(p) > ORIGIN_ABORT_RAD:
                raise RuntimeError(f"origin/pre-arm pose abort 0x{mid:02X} ch={channel}: {np.rad2deg(p):+.1f}deg; D7 o 0 is required")
            limit = self.current_limit_a.get(mid, CURRENT_ABORT_A)
            if abs(s.cur)>limit or abs(v)>SPEED_ABORT_RAD_S:
                raise RuntimeError(
                    f"motion/current abort 0x{mid:02X} ch={channel}: "
                    f"cur={s.cur:+.2f}A (limit ±{limit:.2f}A), "
                    f"speed={np.rad2deg(v):+.1f}deg/s (limit ±{np.rad2deg(SPEED_ABORT_RAD_S):.1f}deg/s), "
                    f"pos={np.rad2deg(p):+.1f}deg"
                )
            out[mid]=MotorFeedback(mid,now,p,v)
        return out
    def snapshot(self):
        """Every axis's last received state, for the line after an abort.

        A ten-axis run that stops names the axis that tripped and nothing
        else.  D10-2 lost exactly that: one axis in the CSV, one number in the
        abort line, and no way to see what the other nine were doing.  This
        raises nothing and sends nothing.
        """
        out = []
        for mid in H_CAN_IDS:
            try:
                state = self.route_by_motor_id[mid].state(mid)
                channel = self.channel_for(mid)
            except (KeyError, RuntimeError, StopIteration):
                out.append((mid, None, None, None, None, None, None))
                continue
            if state is None:
                out.append((mid, channel, None, None, None, None, None))
                continue
            position, velocity = servo_feedback_to_h_units(state.pos, state.spd)
            out.append((mid, channel, position, velocity, state.cur, state.err,
                        max(0.0, time.time() - state.t)))
        return out

    def zero(self):
        if not self.route_by_motor_id or not self.mit_frames_sent:
            return
        for _ in range(3):
            for fr in zero_frames(tuple(sorted(self.mit_motor_ids))):
                self.send(fr)
            time.sleep(.01)

def report_snapshot(snapshot, current_limits=None):
    """Print the ten-axis state a run stopped in.  Sends nothing, raises nothing."""
    limits = current_limits or default_current_limits()
    print("AXIS SNAPSHOT (all ten axes at the moment the run stopped):")
    for mid, channel, position, velocity, current, err, age in snapshot:
        name = H_BINDING_BY_ID[mid].name
        channel_text = "?" if channel is None else str(channel)
        if position is None:
            print(f"  0x{mid:02X} {name:7s} ch={channel_text} no state received")
            continue
        print(
            f"  0x{mid:02X} {name:7s} ch={channel_text} "
            f"pos={np.rad2deg(position):+8.2f}deg "
            f"vel={np.rad2deg(velocity):+8.1f}deg/s "
            f"cur={current:+6.2f}A (limit {limits.get(mid, CURRENT_ABORT_A):.2f}A) "
            f"err={err} age={age:.3f}s"
        )


def report_all_axes(rows, initial_positions, initial_targets, motor_ids):
    """Per-axis table: what each axis was asked for, and what it did.

    This replaces the single-axis tracking verdict for a whole-body run.  A
    ten-axis run judged on H_CAN_IDS[0] alone is judged on 0x1C, whose
    deadband is the largest measured on this machine, so a pass/fail keyed to
    it says nothing about the other nine.
    """
    print("PER-AXIS RESULT (commanded delta against feedback movement):")
    verdicts = {}
    for mid in motor_ids:
        index = H_CAN_IDS.index(mid)
        axis = rows_for_axis(rows, mid)
        kp_cmd = wire_command(mid, 0.0)[0]
        requested = initial_targets[index] - initial_positions[index]
        movement, current, required, verdict = policy_ramp_summary(
            axis, initial_positions[index], initial_targets[index]
        )
        deadband = deadband_lower_bound_rad(current, kp_cmd)
        deadband_deg = float("nan") if deadband is None else np.rad2deg(deadband)
        verdicts[mid] = verdict
        print(
            f"  0x{mid:02X} {H_BINDING_BY_ID[mid].name:7s} "
            f"asked={np.rad2deg(requested):+7.2f}deg "
            f"moved={np.rad2deg(movement):6.2f}deg "
            f"(need >= {np.rad2deg(required):5.2f}deg) "
            f"max|I|={current:5.2f}A "
            f"deadband >= {deadband_deg:5.2f}deg; {verdict}"
        )
    return verdicts


def preview():
    for i,(mid,model) in enumerate(zip(H_CAN_IDS,H_MODELS)):
        cmd=(gains()[i][0],gains()[i][1],0.,0.,0.)
        got=quantized_cmd(cmd,model)
        print(f"0x{mid:02X} {model} Kp={got[0]:.3f} Kd={got[1]:.3f} data={f_mit(mid,*cmd,model).data.hex()}")


def run_static_probe(csv_path, motor_id, target_delta_deg, ramp_seconds, duration):
    """One-axis, fixed-target MIT check with no policy package or T265 input."""
    bus = DualBus()
    rows = []
    bus_opened = False
    selected_index = H_CAN_IDS.index(motor_id)
    try:
        bus.open()
        bus_opened = True
        bus.discover_routes()
        feedback = bus.feedback()
        start = tuple(feedback[mid].position for mid in H_CAN_IDS)
        target = list(start)
        target[selected_index] += np.deg2rad(target_delta_deg)
        target = tuple(target)
        wire_kp, wire_kd, wire_target, _wire_vel, _wire_tau = wire_command(
            motor_id, target[selected_index]
        )
        print(
            f"STATIC PROBE active: motor=0x{motor_id:02X}; relative target="
            f"{target_delta_deg:+.2f}deg; ramp={ramp_seconds:g}s; hold={duration:g}s"
        )
        print(
            f"MIT wire check: 0x{motor_id:02X} Kp={wire_kp:.3f} Kd={wire_kd:.3f} "
            f"target={np.rad2deg(wire_target):+.3f}deg; no policy/T265 input."
        )
        _ceiling, ceiling_detail = stall_guard(
            motor_id, wire_kp, np.deg2rad(target_delta_deg)
        )
        print("STALL GUARD: " + ceiling_detail)
        start_time = time.monotonic()
        next_tick = start_time
        end_ramp = start_time + ramp_seconds
        end_hold = end_ramp + duration
        while True:
            now = time.monotonic()
            if now >= end_hold:
                break
            time.sleep(max(0.0, next_tick - now))
            tick = time.monotonic()
            requested = ramp_targets(start, target, tick - start_time, ramp_seconds)
            stage = "probe-ramp" if tick < end_ramp else "probe-hold"
            # Keep every existing stale/error/current/speed gate active.
            bus.feedback()
            for frame in frames(requested, (motor_id,)):
                bus.send(frame)
            state = bus.state(motor_id)
            feedback_pos, feedback_vel = servo_feedback_to_h_units(state.pos, state.spd)
            _kp, _kd, wire_position, _vel, _tau = wire_command(motor_id, requested[selected_index])
            rows.append((tick, stage, f"0x{motor_id:02X}", target[selected_index],
                         requested[selected_index], wire_position, feedback_pos,
                         feedback_vel, state.cur))
            next_tick += PERIOD
        # An axis that stays put is the measurement this probe exists to make,
        # so it is scored and reported.  Only a real abort (current, speed,
        # stale feedback, origin, stall guard) raises out of this run.
        result = probe_result(rows, start[selected_index],
                              np.deg2rad(target_delta_deg))
        return report_probe(motor_id, rows, wire_kp, result,
                            np.deg2rad(target_delta_deg))
    finally:
        if bus_opened:
            try:
                bus.zero()
            except Exception as error:
                print(f"WARNING: zero MIT cleanup failed: {error}")
            try:
                bus.close()
            except Exception as error:
                print(f"WARNING: CAN cleanup failed: {error}")
        try:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            with csv_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow((
                    "tick", "stage", "sent_motor_id", "desired_target_rad",
                    "requested_target_rad", "wire_target_rad", "feedback_position_rad",
                    "feedback_velocity_rad_s", "feedback_current_a",
                ))
                writer.writerows(rows)
        except Exception as error:
            print(f"WARNING: CSV cleanup failed: {error}")

def report_hold_pose(rows, start, motor_ids, current_limits):
    """What each axis needed to hold itself, and how far it sagged doing it.

    Under MIT position control |I| = Kp * error, so an axis commanded to stay
    where it already is sags until Kp*error balances its gravity load.  The
    steady state therefore gives two numbers per axis that nothing else in
    this program can give: the current that axis needs to hold itself, and the
    droop that current buys.  Both are read from the tail of the hold, not its
    mean, for the reason recorded in hold_tail_rows().
    """
    print("HOLD POSE RESULT (current each axis needs to hold itself):")
    worst = []
    for mid in motor_ids:
        index = H_CAN_IDS.index(mid)
        axis = rows_for_axis(rows, mid)
        name = H_BINDING_BY_ID[mid].name
        limit = current_limits.get(mid, CURRENT_ABORT_A)
        if not axis:
            print(f"  0x{mid:02X} {name:7s} no samples")
            continue
        tail = hold_tail_rows(axis)
        held = median([abs(row[8]) for row in tail])
        peak = max(abs(row[8]) for row in axis)
        droop = median([row[6] for row in tail]) - start[index]
        torque = held * CP[H_MODELS[index]]
        headroom = limit - peak
        worst.append((headroom, mid))
        print(
            f"  0x{mid:02X} {name:7s} hold|I|={held:5.2f}A "
            f"(peak {peak:5.2f}A, limit {limit:5.2f}A, headroom {headroom:+5.2f}A) "
            f"= {torque:5.2f}N*m; droop={np.rad2deg(droop):+7.2f}deg"
        )
    if worst:
        headroom, mid = min(worst)
        print(
            f"TIGHTEST AXIS: 0x{mid:02X} {H_BINDING_BY_ID[mid].name} with "
            f"{headroom:+.2f}A of headroom. Set the next run's limits from this "
            "table, not from an estimate."
        )
    return worst


def run_hold_pose(csv_path, duration, motor_ids=H_CAN_IDS, current_limits=None):
    """Hold every driven axis at the position it is ALREADY in, and measure.

    No policy, no T265, no package, no ramp toward a new pose: each target is
    the position that axis reports at t=0 and it never changes.  What the
    machine then does is the measurement D10-2 was missing.  It cannot be
    obtained from a walking run, because there a sagging axis and a policy
    command are the same number.

    Every existing gate stays active -- stale feedback, motor error, origin,
    speed -- and the current gate is per axis (see current_limits).
    """
    current_limits = dict(current_limits or default_current_limits())
    bus = DualBus(current_limits)
    rows = []
    bus_opened = False
    try:
        bus.open()
        bus_opened = True
        bus.discover_routes()
        feedback = bus.feedback()
        start = tuple(feedback[mid].position for mid in H_CAN_IDS)
        print(
            f"HOLD POSE active: {len(motor_ids)} axes; hold={duration:g}s; "
            "target = the position each axis is already in; no policy, no T265."
        )
        print("Hold targets: " + ", ".join(
            f"0x{mid:02X}={np.rad2deg(start[H_CAN_IDS.index(mid)]):+.1f}deg"
            for mid in motor_ids
        ))
        print("Current aborts: " + ", ".join(
            f"0x{mid:02X}={current_limits.get(mid, CURRENT_ABORT_A):.1f}A"
            for mid in motor_ids
        ))
        start_time = time.monotonic()
        next_tick = start_time
        end_hold = start_time + duration
        while True:
            now = time.monotonic()
            if now >= end_hold:
                break
            time.sleep(max(0.0, next_tick - now))
            tick = time.monotonic()
            bus.feedback()
            for frame in frames(start, motor_ids):
                bus.send(frame)
            rows.extend(axis_rows(tick, "hold-pose", start, start, bus, motor_ids))
            next_tick += PERIOD
        print(f"HOLD POSE complete: CAN tx={bus.tx_count}.")
        report_hold_pose(rows, start, motor_ids, current_limits)
        return 0
    finally:
        if bus_opened:
            try:
                report_snapshot(bus.snapshot(), current_limits)
            except Exception as error:
                print(f"WARNING: axis snapshot failed: {error}")
            try:
                bus.zero()
            except Exception as error:
                print(f"WARNING: zero MIT cleanup failed: {error}")
            try:
                bus.close()
            except Exception as error:
                print(f"WARNING: CAN cleanup failed: {error}")
        try:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            with csv_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow((
                    "tick", "stage", "sent_motor_id", "desired_target_rad",
                    "requested_target_rad", "wire_target_rad", "feedback_position_rad",
                    "feedback_velocity_rad_s", "feedback_current_a",
                ))
                writer.writerows(rows)
        except Exception as error:
            print(f"WARNING: CSV cleanup failed: {error}")


def run(package,duration,csv_path,vx,vy,wz,transmit=True,ramp_seconds=None,motor_ids=H_CAN_IDS,current_limits=None):
    current_limits = dict(current_limits or default_current_limits())
    policy=HPolicy(package); bus=DualBus(current_limits); t265=RealT265(T265_R_OFFSET_M); cmd=FixedCommandSource(vx,vy,wz); last=np.zeros(10,np.float32)
    rows=[]; bus_opened=False; t265_start_attempted=False
    try:
        bus.open(); bus_opened=True; t265_start_attempted=True; t265.start(); deadline=time.monotonic()+5
        while t265.latest() is None:
            if time.monotonic()>deadline: raise RuntimeError("T265 warmup timeout")
            time.sleep(.01)
        bus.discover_routes()
        # Read safety-relevant state once before any transmit.  In --preflight
        # mode this is the complete hardware interaction: no MIT cleanup frame
        # is sent because DualBus.mit_frames_sent remains false.
        feedback = bus.feedback()
        _s, _o, initial_out, _plan = evaluate_cycle(
            policy, t265.latest(), feedback, cmd.sample(time.monotonic()), last
        )
        initial_targets = initial_out.joint_target_h_order
        # MotorFeedback is the shell boundary object; its values are already
        # radians, under the public `position` / `velocity` names.
        initial_positions = tuple(feedback[mid].position for mid in H_CAN_IDS)
        print("Initial policy targets (no MIT sent yet): " + ", ".join(
            f"0x{mid:02X} target={np.rad2deg(target):+.1f}deg "
            f"delta={np.rad2deg(target-position):+.1f}deg"
            for mid, target, position in zip(H_CAN_IDS, initial_targets, initial_positions)
        ))
        if not transmit:
            print("PREFLIGHT complete: CAN transmit count is zero.")
            return

        if ramp_seconds is None:
            raise RuntimeError("--arm requires an explicit --ramp-seconds value")
        if ramp_seconds <= 0:
            raise ValueError("ramp_seconds must be positive")
        selected_id = motor_ids[0]
        selected_index = H_CAN_IDS.index(selected_id)
        wire_kp, wire_kd, initial_wire_target, _wire_vel, _wire_tau = wire_command(
            selected_id, initial_targets[selected_index]
        )
        required_ramp_tracking = max(
            MIN_TRACKING_RAD,
            abs(initial_targets[selected_index] - initial_positions[selected_index])
            * POLICY_RAMP_MIN_TRACKING_FRACTION,
        )
        # The same maximum slope used for 0 -> first policy target also
        # guards the transition from the frozen initial target into live
        # policy output.  This prevents a first live-policy frame from
        # undoing the ramp with an opposite-sign step.
        ramp_rate = abs(initial_targets[selected_index] - initial_positions[selected_index]) / ramp_seconds
        print(
            f"ARM active: motor=0x{selected_id:02X}; "
            f"ramp={ramp_seconds:g}s; then policy={duration:g}s"
        )
        print(
            f"MIT wire check: 0x{selected_id:02X} Kp={wire_kp:.3f} Kd={wire_kd:.3f} "
            f"target={np.rad2deg(initial_wire_target):+.3f}deg "
            f"(policy={np.rad2deg(initial_targets[selected_index]):+.3f}deg); "
            f"requires >= {np.rad2deg(required_ramp_tracking):.3f}deg feedback movement."
        )
        _ceiling, ceiling_detail = stall_guard(
            selected_id, wire_kp,
            initial_targets[selected_index] - initial_positions[selected_index],
        )
        print("STALL GUARD: " + ceiling_detail)

        # Do not apply the first policy target as a step.  The target is
        # frozen for this transition; after the ramp, the ordinary policy
        # loop resumes with its first action recorded as `last`.
        ramp_start = time.monotonic()
        ramp_next = ramp_start
        last_ramp_progress = -1
        while True:
            elapsed = time.monotonic() - ramp_start
            if elapsed >= ramp_seconds:
                break
            time.sleep(max(0, ramp_next - time.monotonic()))
            tick = time.monotonic()
            ramped = ramp_targets(initial_positions, initial_targets,
                                  tick - ramp_start, ramp_seconds)
            # Re-read feedback before every frame batch.  The established
            # stale/error/current/speed gates remain active during the ramp.
            bus.feedback()
            for fr in frames(ramped, motor_ids):
                bus.send(fr)
            rows.extend(axis_rows(tick, "ramp", initial_targets, ramped,
                                  bus, motor_ids))
            progress = min(int(tick - ramp_start), int(ramp_seconds))
            if progress != last_ramp_progress:
                print(
                    f"RAMP progress: {progress}/{ramp_seconds:g}s; "
                    f"0x{selected_id:02X} target={np.rad2deg(ramped[selected_index]):+.2f}deg; "
                    f"CAN tx={bus.tx_count}"
                )
                last_ramp_progress = progress
            ramp_next += PERIOD

        last = initial_out.action_raw
        print(f"RAMP complete: CAN tx={bus.tx_count}; entering policy hold.")
        end=time.monotonic()+duration; nxt=time.monotonic()
        commanded_target = initial_targets[selected_index]
        previous_policy_tick = time.monotonic()
        last_policy_progress = -1
        while time.monotonic()<end:
            time.sleep(max(0,nxt-time.monotonic())); tick=time.monotonic()
            _s,_o,out,plan=evaluate_cycle(policy,t265.latest(),bus.feedback(),cmd.sample(tick),last)
            desired_target = out.joint_target_h_order[selected_index]
            commanded_target = slew_target(
                commanded_target, desired_target, ramp_rate, tick - previous_policy_tick
            )
            targets = list(out.joint_target_h_order)
            targets[selected_index] = commanded_target
            for fr in frames(targets, motor_ids): bus.send(fr)
            rows.extend(axis_rows(tick, "policy", out.joint_target_h_order,
                                  targets, bus, motor_ids))
            progress = min(int(tick - (end - duration)), int(duration))
            if progress != last_policy_progress:
                print(
                    f"POLICY progress: {progress}/{duration:g}s; "
                    f"0x{selected_id:02X} desired={np.rad2deg(desired_target):+.2f}deg; "
                    f"sent={np.rad2deg(commanded_target):+.2f}deg; "
                    f"CAN tx={bus.tx_count}"
                )
                last_policy_progress = progress
            last=out.action_raw; previous_policy_tick=tick; nxt+=PERIOD
        selected_rows = rows_for_axis(rows, selected_id)
        max_tracking_rad, max_current_a, required_tracking_rad, tracking_verdict = policy_ramp_summary(
            selected_rows, initial_positions[selected_index], initial_targets[selected_index]
        )
        print(
            f"TRACKING: 0x{selected_id:02X} max feedback movement="
            f"{np.rad2deg(max_tracking_rad):.3f}deg "
            f"(required >= {np.rad2deg(required_tracking_rad):.3f}deg); "
            f"max |current|={max_current_a:.2f}A; {tracking_verdict}"
        )
        torque_verdict = report_torque_path(selected_id, selected_rows, wire_kp)
        if len(motor_ids) > 1:
            # A whole-body run is not pass/fail on one axis.  Report all ten
            # and let the judging table read the table, not a single verdict
            # keyed to the axis with the largest measured deadband.
            report_all_axes(rows, initial_positions, initial_targets, motor_ids)
        elif max_tracking_rad < required_tracking_rad:
            report_deadband(selected_id, wire_kp, sustained_current_a(selected_rows), None)
            tracking_verdict = f"{tracking_verdict}; {torque_verdict}"
            raise RuntimeError(
                f"tracking abort 0x{selected_id:02X}: feedback moved only "
                f"{np.rad2deg(max_tracking_rad):.3f}deg; required >= "
                f"{np.rad2deg(required_tracking_rad):.3f}deg. "
                f"max |current|={max_current_a:.2f}A; {tracking_verdict}."
            )
        print(f"POLICY complete: CAN tx={bus.tx_count}; sending selected-axis zero MIT cleanup.")
    finally:
        # Cleanup must never be skipped, including a stale-feedback or USB-open
        # failure.  Attempt all shutdown steps even if one of them fails.
        if bus_opened:
            try:
                report_snapshot(bus.snapshot(), current_limits)
            except Exception as error:
                print(f"WARNING: axis snapshot failed: {error}")
            try:
                bus.zero()
            except Exception as error:
                print(f"WARNING: zero MIT cleanup failed: {error}")
        if t265_start_attempted:
            try:
                t265.close()
            except Exception as error:
                print(f"WARNING: T265 cleanup failed: {error}")
        if bus_opened:
            try:
                bus.close()
            except Exception as error:
                print(f"WARNING: CAN cleanup failed: {error}")
        # Preserve evidence from a partial run without masking its primary error.
        try:
            csv_path.parent.mkdir(parents=True,exist_ok=True)
            with csv_path.open('w',newline='',encoding='utf-8') as f:
                w=csv.writer(f)
                w.writerow((
                    'tick', 'stage', 'sent_motor_id', 'desired_target_rad',
                    'requested_target_rad', 'wire_target_rad', 'feedback_position_rad',
                    'feedback_velocity_rad_s', 'feedback_current_a',
                ))
                w.writerows(rows)
        except Exception as error:
            print(f"WARNING: CSV cleanup failed: {error}")

def analyze_csv(csv_path):
    """Re-judge a saved one-axis CSV.  Opens no bus and sends no CAN frame."""
    with csv_path.open(newline="", encoding="utf-8") as file:
        records = list(csv.DictReader(file))
    if not records:
        raise RuntimeError(f"no rows in {csv_path}")
    motor_id = int(records[0]["sent_motor_id"], 16)
    if motor_id not in H_CAN_IDS:
        raise RuntimeError(f"{csv_path} is not a registered H axis")
    rows = [(
        float(record["tick"]), record["stage"], record["sent_motor_id"],
        float(record["desired_target_rad"]), float(record["requested_target_rad"]),
        float(record["wire_target_rad"]), float(record["feedback_position_rad"]),
        float(record["feedback_velocity_rad_s"]), float(record["feedback_current_a"]),
    ) for record in records]
    kp_cmd = wire_command(motor_id, rows[0][5])[0]
    initial_position = rows[0][6]
    initial_target = rows[0][3]
    stages = ", ".join(
        f"{stage}={sum(1 for row in rows if row[1] == stage)}"
        for stage in dict.fromkeys(row[1] for row in rows)
    )
    logged_ids = [int(label, 16) for label in dict.fromkeys(row[2] for row in rows)]
    print(f"ANALYZE {csv_path}: {len(logged_ids)} axis/axes "
          f"({', '.join(f'0x{mid:02X}' for mid in logged_ids)}); "
          f"{len(rows)} rows ({stages})")
    ticks = sorted(set(row[0] for row in rows))
    if ticks:
        print(f"SPAN: {ticks[-1] - ticks[0]:.2f}s of wall time over "
              f"{len(ticks)} ticks. Stages present: {stages}.")
        if not any(row[1] in ("policy", "probe-hold", "hold-pose") for row in rows):
            print(
                "INCOMPLETE: this CSV holds ramp samples only. The run did not "
                "reach its hold/policy stage, so it stopped early -- the CSV is "
                "written from a finally block, so partial rows mean the run "
                "raised or was interrupted. Read the screen log's last line "
                "before judging anything else."
            )
    if len(logged_ids) > 1:
        start = {}
        target = {}
        for mid in logged_ids:
            axis = rows_for_axis(rows, mid)
            start[mid] = axis[0][6]
            target[mid] = axis[-1][3]
        positions = tuple(start.get(mid, 0.0) for mid in H_CAN_IDS)
        targets = tuple(target.get(mid, start.get(mid, 0.0)) for mid in H_CAN_IDS)
        if rows[0][1] == "hold-pose":
            report_hold_pose(rows, positions, logged_ids, default_current_limits())
        else:
            report_all_axes(rows, positions, targets, logged_ids)
        return 0
    if rows[0][1].startswith("probe"):
        result = probe_result(rows, initial_position,
                              initial_target - initial_position)
        requested_delta = initial_target - initial_position
        letter = report_probe(motor_id, rows, kp_cmd, result, requested_delta)
        ceiling = probe_ceiling_current_a(kp_cmd, requested_delta)
        print(
            f"PROBE CEILING: this run could only prove motion for a breakaway "
            f"current below {ceiling:.2f}A (commanded)."
        )
        known = demonstrated_stall_current_a(motor_id, requested_delta)
        if letter == "C" and known is not None and ceiling <= known + STALL_GUARD_MARGIN_A:
            print(
                f"CAVEAT: that ceiling is at or below {known:.2f}A, which this axis "
                f"has already held without moving in the "
                f"{direction_label(probe_direction(requested_delta))} direction, so "
                "this run's C repeats a known result instead of adding one."
            )
        return D9_3_EXIT_CODE[letter]
    movement, max_current, required, verdict = policy_ramp_summary(
        rows, initial_position, initial_target
    )
    print(
        f"TRACKING: 0x{motor_id:02X} max feedback movement="
        f"{np.rad2deg(movement):.3f}deg (required >= {np.rad2deg(required):.3f}deg); "
        f"max |current|={max_current:.2f}A; {verdict}"
    )
    report_torque_path(motor_id, rows, kp_cmd)
    report_deadband(motor_id, kp_cmd, sustained_current_a(rows), None)
    return 0


def main():
    p=argparse.ArgumentParser(description='D8 10-axis MIT sender; --arm is required for any CAN transmit.')
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--arm',action='store_true')
    mode.add_argument('--preflight',action='store_true', help='open/receive/evaluate once and print initial targets; sends zero CAN frames')
    p.add_argument('--static-probe', action='store_true', help='with --arm: one-axis fixed relative target; no policy/T265')
    p.add_argument('--probe-target-deg', type=float, help=f'fixed relative target for --static-probe; abs <= {STATIC_PROBE_MAX_DEG:g} deg and within the per-axis current-abort limit')
    p.add_argument('--all-axes', action='store_true', help='with --arm: drive all ten registered axes instead of one. Suspended robot only; every existing abort stays active')
    p.add_argument('--hold-pose', action='store_true', help='with --arm --all-axes: freeze every target at the position that axis is already in and measure the current it needs to hold itself. No policy, no T265, no --package')
    p.add_argument('--gravity-limits', action='store_true', help=f'per-axis current abort sized to this robot static gravity load instead of the flat {CURRENT_ABORT_A:.1f}A one-axis limit. Requires --all-axes. Needs explicit user approval: it RAISES the abort on load-bearing axes')
    p.add_argument('--analyze', type=Path, help='re-judge a saved one-axis CSV offline; opens no CAN bus');    p.add_argument('--preview',action='store_true'); p.add_argument('--package',type=Path); p.add_argument('--duration',type=float,default=0.); p.add_argument('--csv',type=Path); p.add_argument('--vx',type=float,default=0.); p.add_argument('--vy',type=float,default=0.); p.add_argument('--wz',type=float,default=0.); p.add_argument('--ramp-seconds',type=float, help='required with --arm; initial policy target is reached linearly over this time'); p.add_argument('--motor-id', type=lambda value: int(value, 0), action='append', help='required once with --arm; only this registered motor receives MIT frames')
    a=p.parse_args()
    current_mode = 'hold-pose' if a.hold_pose else 'static-probe' if a.static_probe else 'arm' if a.arm else 'preflight' if a.preflight else 'none'
    print(f"ver9_d8_sender build={BUILD_ID}; mode={current_mode}")
    if a.analyze: return analyze_csv(a.analyze)
    if a.preview: preview(); return 0
    if not (a.arm or a.preflight): p.error('--arm is required for transmission; use --preflight for a receive-only live check')
    if a.gravity_limits and not a.all_axes:
        p.error('--gravity-limits is only for a whole-body run; pass --all-axes, '
                'or leave the one-axis limit alone')
    if a.gravity_limits and a.static_probe:
        p.error('--gravity-limits must not be combined with --static-probe; a one-axis '
                'probe carries no load and its limit is not the thing under test')
    limits = gravity_current_limits() if a.gravity_limits else default_current_limits()
    if a.gravity_limits:
        print("GRAVITY LIMITS: per-axis current abort raised to " + ", ".join(
            f"0x{mid:02X} {H_BINDING_BY_ID[mid].name}={limits[mid]:.1f}A"
            for mid in H_CAN_IDS
        ))
        print(f"GRAVITY LIMITS: the flat {CURRENT_ABORT_A:.1f}A limit is below this "
              "robot's own static gravity load (HAA needs 1.7A hanging at the zero "
              "pose), which is why every D10-2 run died inside the ramp. These "
              "values are still far under the trained policy's own effort limit "
              "(AK10-9 42.1A, AK80-9 25.8A). Speed abort, stale feedback, motor "
              "error and origin aborts are unchanged.")
    if a.hold_pose:
        if not a.arm or a.preflight:
            p.error('--hold-pose requires --arm and cannot be combined with --preflight')
        if not a.all_axes:
            p.error('--hold-pose is the whole-body hold measurement; pass --all-axes')
        if a.static_probe:
            p.error('--hold-pose and --static-probe are different runs; pass one')
        if a.motor_id:
            p.error('--hold-pose drives every registered axis; do not also pass --motor-id')
        if a.package:
            p.error('--hold-pose runs no policy; do not pass --package')
        if a.vx or a.vy or a.wz:
            p.error('--hold-pose runs no policy, so a velocity command means nothing; '
                    'leave --vx/--vy/--wz at zero')
        if not a.csv or not 0 < a.duration <= 5:
            p.error('--hold-pose requires --csv and 0<--duration<=5')
        print(f"HOLD POSE: holding all {len(H_CAN_IDS)} registered axes at their "
              "present position (suspended robot only). Every axis is logged.")
        return run_hold_pose(a.csv, a.duration, H_CAN_IDS, limits)
    if a.static_probe:
        if not a.arm or a.preflight: p.error('--static-probe requires --arm and cannot be combined with --preflight')
        if not a.csv or not 0 < a.duration <= 5: p.error('--static-probe requires --csv and 0<--duration<=5')
        if a.ramp_seconds is None or a.ramp_seconds <= 0: p.error('--static-probe requires a positive --ramp-seconds')
        if not a.motor_id or len(a.motor_id) != 1 or a.motor_id[0] not in H_CAN_IDS:
            p.error('--static-probe requires exactly one registered --motor-id')
        if a.probe_target_deg is None or abs(a.probe_target_deg) <= 0:
            p.error('--static-probe requires a nonzero --probe-target-deg')
        probe_kp = wire_command(a.motor_id[0], 0.0)[0]
        probe_limit = max_probe_angle_deg(probe_kp)
        if abs(a.probe_target_deg) > probe_limit:
            p.error(
                f'--probe-target-deg for 0x{a.motor_id[0]:02X} must satisfy '
                f'abs(value)<={probe_limit:.2f} (Kp={probe_kp:.3f}; beyond that the '
                f'{CURRENT_ABORT_A:.1f}A current abort trips before the target). '
                'Do not raise Kp or the abort to get a larger angle.'
            )
        # Refuse a known-useless run before the operator powers anything up,
        # not after the bus is open.  The same guard still runs inside the
        # probe, immediately before the first MIT frame.
        try:
            stall_guard(a.motor_id[0], probe_kp, np.deg2rad(a.probe_target_deg))
        except RuntimeError as error:
            p.error(str(error))
        letter = run_static_probe(a.csv, a.motor_id[0], a.probe_target_deg,
                                  a.ramp_seconds, a.duration)
        if letter in ("C", "D"):
            print("Main power OFF before touching the machine. Do not re-run this "
                  "probe, and do not raise Kp, the target angle or the current abort.")
        return D9_3_EXIT_CODE[letter]
    if not a.package: p.error('--package is required')
    if a.preflight:
        # A path is still supplied so evidence is written consistently, but
        # no policy frame or cleanup frame is put on CAN.
        run(a.package, 0., a.csv or Path('logs/d9_preflight.csv'), a.vx, a.vy, a.wz, transmit=False)
        return 0
    if not a.csv or not 0<a.duration<=5: p.error('--csv and 0<--duration<=5 are required with --arm')
    if a.ramp_seconds is None or a.ramp_seconds <= 0: p.error('--arm requires a positive --ramp-seconds value')
    if a.all_axes:
        # Every axis at once is the whole-body step.  It is deliberate and
        # explicit: --motor-id must not be given, so nobody reaches ten axes
        # by accident while thinking they selected one.  The current, speed,
        # stale-feedback and origin aborts all stay active and all of them
        # still watch every axis, not just the logged one.
        if a.motor_id:
            p.error('--all-axes drives every registered axis; do not also pass --motor-id')
        motor_ids = H_CAN_IDS
        print(f"ALL AXES: driving all {len(H_CAN_IDS)} registered axes "
              "(suspended robot only). Every abort stays active. "
              "All ten axes are logged to the CSV.")
    else:
        if not a.motor_id or len(a.motor_id) != 1 or a.motor_id[0] not in H_CAN_IDS:
            p.error('--arm requires exactly one registered --motor-id (for example, 0x1C), '
                    'or --all-axes for the whole-body step')
        motor_ids = tuple(a.motor_id)
    run(a.package, a.duration, a.csv, a.vx, a.vy, a.wz,
        transmit=True, ramp_seconds=a.ramp_seconds, motor_ids=motor_ids,
        current_limits=limits)
if __name__=='__main__': sys.exit(main() or 0)

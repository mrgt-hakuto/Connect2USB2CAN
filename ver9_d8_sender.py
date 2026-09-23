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
from policy_integration import ACTION_SIZE, DEFAULT_JOINT_POS, HPolicy
from robot_joint_map import BY_ID as H_BINDING_BY_ID, h_to_motor, motor_to_h
from ver9_integration import H_CAN_IDS, H_MODELS, evaluate_cycle
from ver9_shell import FixedCommandSource, MotorFeedback, RealT265, T265_R_OFFSET_M, VelocityCommand, servo_feedback_to_h_units

HZ = 50.0
PERIOD = 1.0 / HZ
BUILD_ID = "D10_13D_ZEROOFFSET_20260923_2300"
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
# D10-3 R3/R4 (2026-09-23): after the ramp only the selected axis (0x1C) was
# slew-limited.  The other nine jumped straight from the frozen ramp target to
# the live policy output -- up to 45 deg in one 20 ms tick (0x1A +35.5 deg,
# 0x22 -19.7 deg) -- and the next tick tripped the current abort (0x22 4.30 A,
# 0x1A 4.35 A at 94 deg/s).  A whole-body policy run therefore rate-limits
# EVERY commanded target with --policy-slew-dps.  This is the permitted
# range; the value itself is typed on the command line.
POLICY_SLEW_MAX_DPS = 60.0
# Pre-arm gate for a whole-body run.  The frozen first policy target is
# ramped to open-loop, so it must stay clear of the 45 deg origin abort and
# no single ramp may exceed ALL_AXES_MAX_DELTA_DEG.  A 2026-09-23 run ramped
# 0x2B toward -63.8 deg and died on the origin abort at -45.2 deg.
ALL_AXES_MAX_TARGET_DEG = 40.0
ALL_AXES_MAX_DELTA_DEG = 30.0
# D10-4 (2026-09-23): starting the policy from the all-zero pose (knees
# straight) and ramping to its first output put the policy outside what it
# saw in training -- the sim resets near DEFAULT_JOINT_POS (x0.5-1.5) -- and
# it asked for KFE up to +178 deg.  --stand-seconds ramps to the sim default
# pose instead, holds it, and only then hands over to the slew-limited policy.
STAND_MAX_SECONDS = 5.0
# D10-10 (2026-09-23): the first run with the feet on the floor.  The robot
# stays on its hoist rope; the ramp to the stand pose is done in the air and
# the operator lowers the hoist during the stand hold until the feet carry the
# weight.  That needs a longer hold than STAND_MAX_SECONDS, so --floor-limits
# (and only it) allows up to FLOOR_STAND_MAX_SECONDS.
FLOOR_STAND_MAX_SECONDS = 30.0
# Standing on the floor loads KFE and FFE far above anything the hanging runs
# saw.  From robot_sim.urdf (10.1 kg), double support, feet under the ankles
# with the centre of pressure 0 or +5 cm forward, at the sim default pose and
# at the golden mean walking pose, the static need is
#   HR <=0.1 A, HAA <=3.5 A, HFE <=9.1 A, KFE <=3.9 A, FFE <=4.7 A
# (AK10-9 c_p 1.258, AK80-9 c_p 0.523).  --gravity-limits caps KFE at 3.0 A
# and FFE at 2.0 A, below that.  This table raises ONLY those two, to about
# 1.5x the estimate; everything else is the --gravity-limits table.  It is a
# starting point for the FIRST loaded stand: the D10-10 R1 hold currents are
# the measured values the next table must come from.  Still far under the
# trained policy's effort (AK10-9 42.1 A, AK80-9 25.8 A).
FLOOR_CURRENT_ABORT_A_BY_JOINT = {
    "HR":   3.0,
    "HAA":  6.0,
    "HFE": 11.0,
    "KFE":  6.0,   # static stance estimate <= 3.9 A
    "FFE":  6.0,   # static stance estimate <= 4.7 A (CoP 5 cm ahead of the ankle)
}
# D10-11 (2026-09-23): on the floor at the sim default pose the robot tipped
# BACKWARD in every run (R1 stand, R2 x2 with the policy), although
# robot_sim.urdf puts the whole-body CoM 1.5 cm AHEAD of the ankles there.  The
# URDF masses were back-calculated from density, so the real CoM is not known.
# Two knobs, both keeping the sole parallel to the body when hanging:
#   --stand-knee-deg K : HFE = -K/2, KFE = +K, FFE = -K/2 (sim default is K=20)
#   --stand-lean-deg L : add L to both FFE targets.  FFE + is toe DOWN; with the
#                        sole flat on the floor that tips the shank, and the
#                        whole robot, FORWARD by L.
# --lean-sweep-deg adds the lean slowly on the floor and logs where the ankles
# stop carrying a backward-tipping torque: FFE feedback lags its target on the
# toe-UP side while the robot leans back on its heels, and on the toe-DOWN side
# once it leans forward.  The zero crossing of that error is the lean at which
# the CoM is over the ankles.  The error, not the current, is used: the logged
# AK80-9 current sign does not follow the error consistently (D10-10 logs).
STAND_KNEE_MAX_DEG = 30.0
STAND_LEAN_MIN_DEG = -5.0
STAND_LEAN_MAX_DEG = 8.0
LEAN_SWEEP_DPS = 0.5
LEAN_SWEEP_MAX_DEG = 10.0
LEAN_SWEEP_HOLD_S = 3.0
# D10-12 (2026-09-23): no time to run D10-11 first.  --auto-lean-deg finds
# the balance lean inside the stand hold of the SAME run that then starts the
# policy: from AUTO_LEAN_START_S into the hold (the hoist is lowered before
# that), the lean on both FFE follows the mean FFE error (target - feedback,
# + = on the heels) at AUTO_LEAN_GAIN deg/s per deg of error, rate-limited
# to AUTO_LEAN_MAX_DPS and clamped to [0, M].  The policy then starts from the
# leaned pose.  Same sign convention and error signal as --lean-sweep-deg.
AUTO_LEAN_START_S = 12.0
AUTO_LEAN_GAIN = 0.2
AUTO_LEAN_MAX_DPS = 0.5
# D10-12: walking needs more than the hanging guards allow.  The sim policy
# moves KFE at ~250 deg/s (golden, std), the D10-10 policy stage died on FFE
# ~6 A while catching a backward fall, and a 60 deg/s slew held KFE up to
# 209 deg behind the policy.  --walk-limits (explicit approval, hoist rope
# attached) is --floor-limits with KFE/FFE 10 A, speed abort 200 deg/s and a
# slew allowed up to 200 deg/s.  Still under the policy's own effort
# (AK10-9 42.1 A, AK80-9 25.8 A).  Nothing else changes.
WALK_CURRENT_ABORT_A_BY_JOINT = {
    "HR":   3.0,
    "HAA":  6.0,
    "HFE": 11.0,
    "KFE": 10.0,
    "FFE": 10.0,
}
WALK_SPEED_ABORT_RAD_S = float(np.deg2rad(200.0))
WALK_POLICY_SLEW_MAX_DPS = 200.0
# D10-13 (2026-09-23): the walking run itself.  With --walk-limits the policy
# stage may last up to WALK_DURATION_MAX_S, and --command-delay-seconds keeps
# the velocity command at zero for the first seconds of the policy stage so
# the policy first catches its balance on the floor, then walks.
WALK_DURATION_MAX_S = 15.0
COMMAND_DELAY_MAX_S = 5.0
# D10-13B (2026-09-23): every D10-13 policy stage on the floor ended within
# 0.2-4.3 s on the 200 deg/s speed abort (last logged 186-211 deg/s), and the
# zero-MIT cleanup that follows every abort dropped the robot: "the knees
# collapsed" was the cleanup, not the policy.  The sim policy moves KFE at
# ~250 deg/s (std), so 200 deg/s is below its own normal speed.
#   --walk-speed-abort-dps S (with --walk-limits, explicit approval): raise the
#     speed abort up to WALK_SPEED_ABORT_MAX_DPS.  The default stays 200.
#   --soft-stop-seconds T (with --walk-limits): after a speed/current/origin
#     abort in the policy stage, hold every axis where it stopped with the
#     stand-hold gains for T s, then send the usual zero MIT.  Stale feedback,
#     a motor error or a current above the table during the soft stop sends
#     zero MIT at once.  Without the flag nothing changes.
WALK_SPEED_ABORT_MAX_DPS = 400.0
SOFT_STOP_MAX_S = 5.0
SOFT_STOP_ABORT_PREFIXES = ("motion/current abort", "origin/pre-arm pose abort")
# D10-13C (2026-09-23): the only long policy stage so far (7.4 s of stepping
# in place, D10-13B 19:14) started with the body upright (pitch -1 deg).
# Every run that started leaning back 14-26 deg ended within 0.7 s.  The sim
# never starts an episode leaning back, so --start-gate-deg G keeps holding
# the stand pose after --stand-seconds until the T265 reads |pitch| and |roll|
# <= G for START_GATE_HOLD_S, then starts the policy; no policy after
# START_GATE_TIMEOUT_S.
START_GATE_MIN_DEG = 2.0
START_GATE_MAX_DEG = 10.0
START_GATE_HOLD_S = 0.5
START_GATE_TIMEOUT_S = 20.0
START_GATE_PRINT_S = 0.5
# D10-13C: Windows time.monotonic() (Python 3.10, GetTickCount64) and the
# default 15.6 ms timer made the 50 Hz loop run at 0/16/31/47 ms steps.
# --fine-timer switches the loop clock to perf_counter and asks Windows for a
# 1 ms timer for the duration of the run.
TIMING_LATE_S = 0.025
# D10-13D (2026-09-23): the sim's joint zero is the CAD assembly pose at URDF
# export, and forward kinematics of onshape_export/myrobot_dummy/robot_sim.urdf
# says that pose is NOT a straight leg: thigh +2.9 / -0.9 deg forward, knee
# bent 12.3 (left) / 7.2 (right) deg, shank 9.4 / 8.1 deg back.  The D7
# origin is set with the legs held straight by eye.  If both are true, the
# robot's H angle and the sim's joint angle differ by a constant per joint:
#     sim_angle = H_angle_from_the_D7_origin + offset
# (assumes the D7 pose is pivots collinear and sole parallel to the body,
# and the CAD pose also has the sole parallel to the body).  With the offset
# the hold pose is the sim default for real (CoM 1.5-2.3 cm in front of the
# ankles in the URDF) instead of CoM 0.1-0.2 cm in front.  Opt-in only.
ZERO_OFFSET_PRESETS_DEG = {
    "cad_fk": {"LL_HFE": 2.9, "LR_HFE": -0.9, "LL_KFE": -12.3, "LR_KFE": -7.2,
               "LL_FFE": 9.4, "LR_FFE": 8.1},
}
ZERO_OFFSET_RAD = np.zeros(10)   # H order; sim_angle = D7-origin angle + this
# D10-13D: a whole-body run that ramps to the fixed stand pose over >= 10 s may
# start up to this far from it (the policy-target gate stays at 30 deg).
STAND_RAMP_MAX_DELTA_DEG = 45.0
# Printed every this many seconds during a long (floor) stand hold, so the
# operator lowering the hoist can see the load arrive on the legs.
STAND_PROGRESS_S = 2.0
STAND_TARGET = tuple(float(value) for value in DEFAULT_JOINT_POS)
# D10-7 (2026-09-23): after the D10-6 joint-sign fix, the stand pose alone
# cannot show a wrong HR/HAA sign -- both sit at 0 there.  The sign pose adds
# the same small outward angle to HR (toes out) and HAA (legs open) on BOTH
# sides.  In H coordinates the sim is left/right mirror symmetric, so a
# correct build looks mirror symmetric; a leg that turns or swings INWARD
# instead has a wrong sign on that axis.  Feedback cannot show this: the
# position loop is closed in whatever frame it is given, so H feedback always
# matches the H target.  Only the eye (or an external reference) can.
SIGN_POSE_EXTRA_DEG = 8.0
_SIGN_POSE_EXTRA = {"HR": SIGN_POSE_EXTRA_DEG, "HAA": SIGN_POSE_EXTRA_DEG}
SIGN_POSE_TARGET = tuple(
    STAND_TARGET[index] + float(np.deg2rad(_SIGN_POSE_EXTRA.get(name.split("_", 1)[1], 0.0)))
    for index, name in enumerate(("LL_HR", "LR_HR", "LL_HAA", "LR_HAA", "LL_HFE", "LR_HFE",
                                  "LL_KFE", "LR_KFE", "LL_FFE", "LR_FFE"))
)
# What a correct sign looks like for each positive (or negative) H angle.
LOOK_BY_JOINT = {
    "HR":  "toe turned OUTWARD (+)",
    "HAA": "leg swung OUTWARD, away from the other leg (+)",
    "HFE": "thigh FORWARD (-)",
    "KFE": "knee bent, foot BACKWARD (+)",
    "FFE": "toe UP (-)",
}
# D10-9 (2026-09-23): in D10-8 the policy closed the legs and the feet hit
# each other (R1 at HAA feedback -4 / -9 deg, R2 around LR_HAA -15 deg).  The
# sim walks with BOTH HAA at -13.5..-16.4 deg (golden.npz, vx 0.5), where the
# URDF puts the ankle joints ~18 cm apart, with self-collision enabled and no
# contact.  --haa-close-deg answers, from the log, the one question that
# decides whether the trained gait fits this machine: at what HAA angle do the
# real feet touch?  After the stand hold (sim default pose), both HAA targets
# move inward together at HAA_SWEEP_DPS.  Gravity pulls a hanging leg INWARD
# (D10-8 droop -2.2..-5.1 deg), so a free leg sits at or inside its target; a
# leg that falls BEHIND its target by HAA_BLOCK_DEG for HAA_BLOCK_TICKS ticks
# is being stopped by something (the other foot).  Then both HAA targets are
# frozen where the legs actually are, so the feet are not pressed together.
HAA_SWEEP_DPS = 2.0
HAA_CLOSE_MAX_DEG = 16.0
HAA_BLOCK_DEG = 4.0
HAA_BLOCK_RAD = float(np.deg2rad(HAA_BLOCK_DEG))
HAA_BLOCK_TICKS = 10
HAA_SWEEP_HOLD_S = 2.0


def haa_indices():
    """H-order indices of LL_HAA and LR_HAA."""
    return tuple(H_CAN_IDS.index(mid) for mid in H_CAN_IDS
                 if H_BINDING_BY_ID[mid].name.endswith("_HAA"))


def haa_sweep_targets(stand_target, close_rad, elapsed_s, rate_rad_s=None):
    """The stand pose with both HAA moved inward (negative H) by the swept amount."""
    rate = float(np.deg2rad(HAA_SWEEP_DPS)) if rate_rad_s is None else rate_rad_s
    delta = min(abs(close_rad), rate * max(elapsed_s, 0.0))
    out = list(stand_target)
    for index in haa_indices():
        out[index] = stand_target[index] - delta
    return tuple(out)


def haa_block_tick(errors_rad, threshold_rad=None, ticks=None):
    """First index at which feedback-minus-target stayed > threshold for `ticks` ticks.

    errors_rad[i] = feedback - target in H.  Inward is negative for both HAA,
    so a POSITIVE error means the leg is less far in than it was told to be.
    Returns the index where the run of blocked ticks STARTED, or None.
    """
    threshold = HAA_BLOCK_RAD if threshold_rad is None else threshold_rad
    need = HAA_BLOCK_TICKS if ticks is None else ticks
    run = 0
    for index, error in enumerate(errors_rad):
        run = run + 1 if error > threshold else 0
        if run >= need:
            return index - need + 1
    return None


def report_haa_sweep(rows):
    """What the HAA sweep found, from CSV rows (live and --analyze)."""
    sweep = [row for row in rows if row[1] in ("haa-sweep", "haa-hold")]
    if not sweep:
        return None
    print(f"HAA SWEEP (D10-9): both HAA inward at {HAA_SWEEP_DPS:g}deg/s from the stand pose; "
          f"blocked = feedback {HAA_BLOCK_DEG:g}deg short of target for {HAA_BLOCK_TICKS} ticks.")
    found = {}
    for index in haa_indices():
        mid = H_CAN_IDS[index]
        name = H_BINDING_BY_ID[mid].name
        axis = rows_for_axis(sweep, mid)
        if not axis:
            print(f"  0x{mid:02X} {name:7s} no sweep samples")
            continue
        errors = [row[6] - row[4] for row in axis]
        at = haa_block_tick(errors)
        inmost = min(row[6] for row in axis)
        peak = max(abs(row[8]) for row in axis)
        last_target = min(row[4] for row in axis)
        if at is None:
            print(f"  0x{mid:02X} {name:7s} NOT blocked: target reached {np.rad2deg(last_target):+.1f}deg, "
                  f"feedback most inward {np.rad2deg(inmost):+.1f}deg, max|I|={peak:.2f}A")
        else:
            row = axis[at]
            found[name] = row[6]
            print(f"  0x{mid:02X} {name:7s} BLOCKED at feedback {np.rad2deg(row[6]):+.1f}deg "
                  f"(target {np.rad2deg(row[4]):+.1f}deg, t={row[0] - axis[0][0]:.2f}s into the sweep); "
                  f"most inward {np.rad2deg(inmost):+.1f}deg, max|I|={peak:.2f}A")
    if found:
        print("HAA SWEEP RESULT: the legs were stopped before the sweep end. Sim gait holds both "
              "HAA at -13.5..-16.4deg; compare the BLOCKED angles above with that.")
    else:
        print("HAA SWEEP RESULT: no leg was blocked over the whole sweep.")
    return found


def joint_sign_line():
    """One line naming every joint sign this build applies."""
    return "JOINT SIGNS (motor = sign * H): " + ", ".join(
        f"0x{mid:02X} {H_BINDING_BY_ID[mid].name}={H_BINDING_BY_ID[mid].sign:+d}"
        for mid in H_CAN_IDS)


def print_look_check(target):
    """What the operator must SEE on both legs; the log cannot tell."""
    print("LOOK CHECK (both legs must match, mirror images of each other):")
    for suffix in ("HR", "HAA", "HFE", "KFE", "FFE"):
        index = H_CAN_IDS.index(next(mid for mid in H_CAN_IDS
                                     if H_BINDING_BY_ID[mid].name == "LL_" + suffix))
        angle = np.rad2deg(target[index])
        if abs(angle) < 0.5:
            print(f"  {suffix:3s} target {angle:+5.1f}deg: straight (not checkable in this pose)")
        else:
            print(f"  {suffix:3s} target {angle:+5.1f}deg: {LOOK_BY_JOINT[suffix]}")
    print("  A leg doing the OPPOSITE on one axis = wrong sign on that axis: "
          "main power OFF, report which leg and which axis.")
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


def h_feedback(motor_id, state):
    """(position rad, velocity rad/s, current A) of one servo state, in the H frame.

    Every feedback quantity crosses the joint sign here (D10-6, 2026-09-23).
    The current keeps its magnitude; only its sign follows the joint.
    """
    position, velocity = servo_feedback_to_h_units(state.pos, state.spd, motor_id)
    return position + zero_offset_rad(motor_id), velocity, motor_to_h(motor_id, state.cur)


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


def stand_pose_target(knee_deg=None, lean_deg=0.0):
    """The stand pose: sim default, or a flat-foot crouch of knee_deg, plus a lean on FFE."""
    out = list(STAND_TARGET)
    for index, mid in enumerate(H_CAN_IDS):
        suffix = joint_suffix(mid)
        if knee_deg is not None:
            if suffix == "KFE":
                out[index] = float(np.deg2rad(knee_deg))
            elif suffix in ("HFE", "FFE"):
                out[index] = float(np.deg2rad(-knee_deg / 2.0))
        if suffix == "FFE":
            out[index] += float(np.deg2rad(lean_deg))
    return tuple(out)


def ffe_indices():
    """H-order indices of LL_FFE and LR_FFE."""
    return tuple(H_CAN_IDS.index(mid) for mid in H_CAN_IDS if joint_suffix(mid) == "FFE")


def lean_sweep_targets(stand_target, max_rad, elapsed_s, rate_rad_s=None):
    """The stand pose with both FFE moved toe-down (positive H) by the swept lean."""
    rate = float(np.deg2rad(LEAN_SWEEP_DPS)) if rate_rad_s is None else rate_rad_s
    lean = min(abs(max_rad), rate * max(elapsed_s, 0.0))
    out = list(stand_target)
    for index in ffe_indices():
        out[index] = stand_target[index] + lean
    return tuple(out)


def lean_balance_deg(leans_deg, errors_deg):
    """Interpolated lean where the mean FFE error (target - feedback) first falls through 0."""
    for i in range(1, len(leans_deg)):
        a, b = errors_deg[i - 1], errors_deg[i]
        if a > 0 >= b:
            if a == b:
                return leans_deg[i]
            return leans_deg[i - 1] + (leans_deg[i] - leans_deg[i - 1]) * a / (a - b)
    return None


def report_lean_sweep(rows):
    """What the lean sweep found, per 1-degree bin of added lean (live and --analyze)."""
    sweep = [row for row in rows if row[1] in ("lean-sweep", "lean-hold")]
    stand = [row for row in rows if row[1] == "stand-hold"]
    if not sweep or not stand:
        return None
    ffe = ffe_indices()
    base = {}
    for index in ffe:
        label = f"0x{H_CAN_IDS[index]:02X}"
        base[index] = next(row[4] for row in stand if row[2] == label)
    ticks = sorted(set(row[0] for row in sweep))
    by_tick = {}
    for row in sweep:
        by_tick.setdefault(row[0], {})[row[2]] = row
    bins = {}
    for tick in ticks:
        entry = by_tick[tick]
        leans, errors = [], []
        for index in ffe:
            row = entry.get(f"0x{H_CAN_IDS[index]:02X}")
            if row is None:
                break
            leans.append(np.rad2deg(row[4] - base[index]))
            errors.append(np.rad2deg(row[4] - row[6]))
        else:
            key = int(np.floor(np.mean(leans) + 1e-6))
            currents = {suffix: max(abs(r[8]) for mid_label, r in entry.items()
                                    if joint_suffix(int(mid_label, 16)) == suffix)
                        for suffix in ("HFE", "KFE", "FFE")}
            bins.setdefault(key, []).append((np.mean(leans), errors[0], errors[1], currents))
    print(f"LEAN SWEEP (D10-11): both FFE toe-down at {LEAN_SWEEP_DPS:g}deg/s on the floor. "
          "err = target - feedback; + = pushed toe-UP (robot on its heels, tipping back), "
          "- = pushed toe-DOWN (tipping forward).")
    xs, ys = [], []
    for key in sorted(bins):
        values = bins[key]
        lean = float(np.mean([v[0] for v in values]))
        ll = float(np.mean([v[1] for v in values]))
        lr = float(np.mean([v[2] for v in values]))
        cur = {suffix: max(v[3][suffix] for v in values) for suffix in ("HFE", "KFE", "FFE")}
        xs.append(lean)
        ys.append((ll + lr) / 2.0)
        print(f"  lean {lean:+5.1f}deg: FFE err L/R {ll:+5.1f}/{lr:+5.1f}deg (mean {ys[-1]:+5.1f}); "
              f"max|I| HFE {cur['HFE']:.2f}A KFE {cur['KFE']:.2f}A FFE {cur['FFE']:.2f}A")
    balance = lean_balance_deg(xs, ys)
    if balance is None:
        print(f"BALANCE LEAN: not crossed; mean FFE err {ys[0]:+.1f}deg at lean {xs[0]:+.1f}, "
              f"{ys[-1]:+.1f}deg at lean {xs[-1]:+.1f}.")
    else:
        print(f"BALANCE LEAN: {balance:+.1f}deg (mean FFE error crosses 0 here). "
              f"Use --stand-lean-deg {int(round(balance))} for the next stand.")
    return balance


def run_lean_sweep(bus, rows, stand_target, max_rad, motor_ids):
    """D10-11: lean the standing robot forward through its ankles, slowly."""
    rate = float(np.deg2rad(LEAN_SWEEP_DPS))
    print(f"LEAN SWEEP: both FFE toe-down by up to {np.rad2deg(abs(max_rad)):.1f}deg at "
          f"{LEAN_SWEEP_DPS:g}deg/s. Support only from the SIDES; do not hold it front/back.")
    start = time.monotonic()
    nxt = start
    hold_end = None
    last_second = -1
    first = ffe_indices()[0]
    while True:
        time.sleep(max(0, nxt - time.monotonic()))
        tick = time.monotonic()
        feedback = bus.feedback()
        targets = lean_sweep_targets(stand_target, max_rad, tick - start, rate)
        sweeping = tick - start < abs(max_rad) / rate
        if not sweeping and hold_end is None:
            print(f"LEAN SWEEP reached +{np.rad2deg(abs(max_rad)):.1f}deg; holding {LEAN_SWEEP_HOLD_S:g}s.")
            hold_end = tick + LEAN_SWEEP_HOLD_S
        for fr in frames(targets, motor_ids):
            bus.send(fr)
        rows.extend(axis_rows(tick, "lean-sweep" if sweeping else "lean-hold",
                              targets, targets, bus, motor_ids))
        second = int(tick - start)
        if sweeping and second != last_second:
            print(f"LEAN SWEEP progress: {second}s; lean=+{np.rad2deg(targets[first] - stand_target[first]):.1f}deg; "
                  + ", ".join(f"{H_BINDING_BY_ID[H_CAN_IDS[i]].name} err="
                              f"{np.rad2deg(targets[i] - feedback[H_CAN_IDS[i]].position):+.1f}deg"
                              for i in ffe_indices()))
            last_second = second
        if hold_end is not None and tick >= hold_end:
            break
        nxt += PERIOD
    return report_lean_sweep(rows)


def walk_current_limits():
    """D10-12: --floor-limits with KFE/FFE at 10 A for a policy run on the floor."""
    return {mid: WALK_CURRENT_ABORT_A_BY_JOINT[joint_suffix(mid)] for mid in H_CAN_IDS}


def auto_lean_step(lean_rad, ffe_errors_rad, dt_s, max_rad):
    """One update of the auto lean: follow the mean FFE error, rate-limited, clamped to [0, max]."""
    rate = float(np.deg2rad(AUTO_LEAN_MAX_DPS))
    mean_err_deg = float(np.rad2deg(np.mean(ffe_errors_rad)))
    step_rad = float(np.deg2rad(AUTO_LEAN_GAIN * mean_err_deg)) * dt_s
    step_rad = max(-rate * dt_s, min(rate * dt_s, step_rad))
    return max(0.0, min(abs(max_rad), lean_rad + step_rad))


def floor_current_limits():
    """D10-10: --gravity-limits with KFE and FFE raised for a loaded stand."""
    return {mid: FLOOR_CURRENT_ABORT_A_BY_JOINT[joint_suffix(mid)]
            for mid in H_CAN_IDS}


def stand_progress_line(elapsed_s, total_s, rows):
    """Largest |current| per joint pair over the last progress window."""
    parts = []
    for suffix in ("HAA", "HFE", "KFE", "FFE"):
        values = []
        for side in ("LL", "LR"):
            mid = next(m for m in H_CAN_IDS if H_BINDING_BY_ID[m].name == f"{side}_{suffix}")
            label = f"0x{mid:02X}"
            recent = [abs(row[8]) for row in rows if row[2] == label]
            values.append(max(recent) if recent else 0.0)
        parts.append(f"{suffix} {values[0]:.2f}/{values[1]:.2f}A")
    return f"STAND HOLD {elapsed_s:4.0f}/{total_s:g}s: max|I| L/R " + ", ".join(parts)


def set_zero_offset(preset):
    """Activate one ZERO_OFFSET_PRESETS_DEG entry (None = all zero).  Returns the table."""
    global ZERO_OFFSET_RAD
    table = ZERO_OFFSET_PRESETS_DEG[preset] if preset else {}
    names = [H_BINDING_BY_ID[mid].name for mid in H_CAN_IDS]
    unknown = set(table) - set(names)
    if unknown:
        raise ValueError(f"unknown joints in zero offset: {sorted(unknown)}")
    ZERO_OFFSET_RAD = np.array([np.deg2rad(table.get(name, 0.0)) for name in names])
    return table


def zero_offset_rad(motor_id):
    return float(ZERO_OFFSET_RAD[H_CAN_IDS.index(motor_id)])


def all_axes_prearm_violations(initial_targets, initial_positions, max_delta_deg=None):
    """Axes whose frozen first policy target a whole-body ramp must not chase."""
    max_delta_deg = ALL_AXES_MAX_DELTA_DEG if max_delta_deg is None else max_delta_deg
    out = []
    for mid, target, position in zip(H_CAN_IDS, initial_targets, initial_positions):
        target_deg = float(np.rad2deg(target))
        delta_deg = float(np.rad2deg(target - position))
        if abs(target_deg) > ALL_AXES_MAX_TARGET_DEG or abs(delta_deg) > max_delta_deg:
            out.append(f"0x{mid:02X} {H_BINDING_BY_ID[mid].name} "
                       f"target={target_deg:+.1f}deg delta={delta_deg:+.1f}deg")
    return out


def slew_all(previous, desired, max_rate_rad_s, elapsed_s):
    """slew_target applied to every one of the ten commanded targets."""
    if len(previous) != ACTION_SIZE or len(desired) != ACTION_SIZE:
        raise ValueError("slew_all needs ten previous and ten desired targets")
    return tuple(slew_target(float(p), float(d), max_rate_rad_s, elapsed_s)
                 for p, d in zip(previous, desired))


def report_slew_gap(rows, motor_ids):
    """How far the slew held each axis back from the live policy target.

    A large, persistent gap means the policy asks for faster motion than the
    run allows: the leg lags by design, not because the axis is weak.
    """
    policy_rows = [row for row in rows if row[1] == "policy"]
    ticks = len(set(row[0] for row in policy_rows))
    print(f"POLICY STAGE: {ticks} tick(s) logged "
          f"({ticks * PERIOD:.2f}s at {1 / PERIOD:.0f} Hz).")
    if not policy_rows:
        return {}
    print("SLEW GAP (live policy target minus the slew-limited target that was sent):")
    gaps = {}
    for mid in motor_ids:
        axis = rows_for_axis(policy_rows, mid)
        if not axis:
            continue
        gap = [abs(row[3] - row[4]) for row in axis]
        held = sum(1 for value in gap if value > np.deg2rad(0.5)) / len(gap)
        gaps[mid] = max(gap)
        print(f"  0x{mid:02X} {H_BINDING_BY_ID[mid].name:7s} "
              f"max gap={np.rad2deg(max(gap)):6.2f}deg "
              f"held back on {held * 100:5.1f}% of ticks; "
              f"max|I|={max(abs(row[8]) for row in axis):5.2f}A")
    return gaps


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
        position, velocity, current = h_feedback(mid, state)
        _kp, _kd, wire_target, _vel, _tau = wire_command(mid, requested[index])
        out.append((tick, stage, f"0x{mid:02X}", float(desired[index]),
                    float(requested[index]), wire_target,
                    position, velocity, current))
    return out

def frames(targets, motor_ids=H_CAN_IDS):
    if len(targets) != ACTION_SIZE: raise ValueError("need ten targets")
    index_by_id = {motor_id: index for index, motor_id in enumerate(H_CAN_IDS)}
    if not motor_ids or any(motor_id not in index_by_id for motor_id in motor_ids):
        raise ValueError("motor_ids must be registered H CAN IDs")
    return tuple(
        f_mit(motor_id, *gains()[index_by_id[motor_id]],
              h_to_motor(motor_id, targets[index_by_id[motor_id]] - zero_offset_rad(motor_id)), 0., 0.,
              H_MODELS[index_by_id[motor_id]])
        for motor_id in motor_ids
    )

def zero_frames(motor_ids=H_CAN_IDS):
    model_by_id = dict(zip(H_CAN_IDS, H_MODELS))
    return tuple(f_mit(mid, 0., 0., 0., 0., 0., model_by_id[mid]) for mid in motor_ids)


def wire_command(motor_id, target_rad):
    """Return the quantized MIT values, including the position on the wire.

    target_rad is in the H frame; the returned position is in the MOTOR frame
    (joint sign applied), exactly as frames() puts it on the bus.
    """
    index = H_CAN_IDS.index(motor_id)
    kp, kd = gains()[index]
    return quantized_cmd((kp, kd, h_to_motor(motor_id, target_rad - zero_offset_rad(motor_id)), 0.0, 0.0),
                         H_MODELS[index])


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
    # Keyed in the MOTOR frame (a stall is a fact about the motor); the
    # probe's delta arrives in the H frame, so cross the joint sign here.
    return DEMONSTRATED_STALL_CURRENT_A.get(
        (motor_id, probe_direction(h_to_motor(motor_id, requested_delta_rad)))
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
        motor_direction = probe_direction(h_to_motor(motor_id, requested_delta_rad))
        opposite = DEMONSTRATED_STALL_CURRENT_A.get((motor_id, -motor_direction))
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
            p,v=servo_feedback_to_h_units(s.pos,s.spd,mid)
            if abs(p) > ORIGIN_ABORT_RAD:
                raise RuntimeError(f"origin/pre-arm pose abort 0x{mid:02X} ch={channel}: {np.rad2deg(p):+.1f}deg "
                                   f"from the D7 origin (limit {np.rad2deg(ORIGIN_ABORT_RAD):.0f}deg). "
                                   "If the leg really is there, no new D7 is needed; if it is not, redo D7 o 0")
            p = p + zero_offset_rad(mid)
            limit = self.current_limit_a.get(mid, CURRENT_ABORT_A)
            speed_limit = getattr(self, "speed_abort_rad_s", SPEED_ABORT_RAD_S)
            if abs(s.cur)>limit or abs(v)>speed_limit:
                raise RuntimeError(
                    f"motion/current abort 0x{mid:02X} ch={channel}: "
                    f"cur={s.cur:+.2f}A (limit ±{limit:.2f}A), "
                    f"speed={np.rad2deg(v):+.1f}deg/s (limit ±{np.rad2deg(speed_limit):.1f}deg/s), "
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
            position, velocity, current = h_feedback(mid, state)
            out.append((mid, channel, position, velocity, current, state.err,
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
            feedback_pos, feedback_vel, feedback_cur = h_feedback(motor_id, state)
            _kp, _kd, wire_position, _vel, _tau = wire_command(motor_id, requested[selected_index])
            rows.append((tick, stage, f"0x{motor_id:02X}", target[selected_index],
                         requested[selected_index], wire_position, feedback_pos,
                         feedback_vel, feedback_cur))
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
                    "requested_target_rad", "wire_target_motor_rad", "feedback_position_rad",
                    "feedback_velocity_rad_s", "feedback_current_h_a",
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
                    "requested_target_rad", "wire_target_motor_rad", "feedback_position_rad",
                    "feedback_velocity_rad_s", "feedback_current_h_a",
                ))
                writer.writerows(rows)
        except Exception as error:
            print(f"WARNING: CSV cleanup failed: {error}")


def run_haa_sweep(bus, rows, stand_target, close_rad, motor_ids):
    """D10-9: close both HAA from the stand pose until the sweep end or a block.

    The block check stays on through the end hold, so a leg that is stopped
    only at the very end of the sweep is still caught and released.
    """
    indices = haa_indices()
    rate = float(np.deg2rad(HAA_SWEEP_DPS))
    print(f"HAA SWEEP: both HAA inward to {-np.rad2deg(abs(close_rad)):+.1f}deg at "
          f"{HAA_SWEEP_DPS:g}deg/s. Hands off. Say out loud when the feet touch.")
    start = time.monotonic()
    nxt = start
    errors = {index: [] for index in indices}
    targets = tuple(stand_target)
    sweeping = True
    blocked_found = False
    hold_end = None
    pending_freeze = None
    last_second = -1
    while True:
        time.sleep(max(0, nxt - time.monotonic()))
        tick = time.monotonic()
        feedback = bus.feedback()
        if not blocked_found:
            if sweeping:
                targets = haa_sweep_targets(stand_target, close_rad, tick - start, rate)
            for index in indices:
                errors[index].append(feedback[H_CAN_IDS[index]].position - targets[index])
            blocked = [index for index in indices if haa_block_tick(errors[index]) is not None]
            if blocked:
                print("HAA CONTACT: " + ", ".join(
                    f"{H_BINDING_BY_ID[H_CAN_IDS[index]].name} feedback="
                    f"{np.rad2deg(feedback[H_CAN_IDS[index]].position):+.1f}deg "
                    f"target={np.rad2deg(targets[index]):+.1f}deg" for index in indices)
                    + f"; blocked: {', '.join(H_BINDING_BY_ID[H_CAN_IDS[i]].name for i in blocked)}. "
                    "Both HAA frozen where they are.")
                # This tick is still sent and logged with the target that was
                # checked (so --analyze finds the same block tick); the frozen
                # target takes over from the next tick, 20 ms later.
                frozen = list(targets)
                for index in indices:
                    frozen[index] = feedback[H_CAN_IDS[index]].position
                pending_freeze = tuple(frozen)
                blocked_found = True
                sweeping = False
                hold_end = tick + HAA_SWEEP_HOLD_S
            elif sweeping and tick - start >= abs(close_rad) / rate:
                print(f"HAA SWEEP reached {np.rad2deg(targets[indices[0]]):+.1f}deg; "
                      f"holding {HAA_SWEEP_HOLD_S:g}s (block check still on).")
                sweeping = False
                hold_end = tick + HAA_SWEEP_HOLD_S
        stage = "haa-sweep" if sweeping else "haa-hold"
        for fr in frames(targets, motor_ids):
            bus.send(fr)
        rows.extend(axis_rows(tick, stage, targets, targets, bus, motor_ids))
        if pending_freeze is not None:
            targets = pending_freeze
            pending_freeze = None
        second = int(tick - start)
        if sweeping and second != last_second:
            print(f"HAA SWEEP progress: {second}s; target={np.rad2deg(targets[indices[0]]):+.1f}deg; "
                  + ", ".join(f"{H_BINDING_BY_ID[H_CAN_IDS[i]].name}="
                              f"{np.rad2deg(feedback[H_CAN_IDS[i]].position):+.1f}deg" for i in indices))
            last_second = second
        if hold_end is not None and tick >= hold_end:
            break
        nxt += PERIOD
    report_haa_sweep(rows)


def run(package,duration,csv_path,vx,vy,wz,transmit=True,ramp_seconds=None,motor_ids=H_CAN_IDS,current_limits=None,policy_slew_rad_s=None,stand_seconds=None,stand_only=False,stand_target=STAND_TARGET,haa_close_rad=None,lean_sweep_rad=None,auto_lean_rad=None,speed_abort_rad_s=None,command_delay_s=None,soft_stop_s=None,start_gate_rad=None,fine_timer=False):
    timer_state = enable_fine_timer() if fine_timer else None
    timing_rows = []
    current_limits = dict(current_limits or default_current_limits())
    policy=HPolicy(package); bus=DualBus(current_limits)
    if speed_abort_rad_s is not None:
        bus.speed_abort_rad_s = speed_abort_rad_s
    t265=RealT265(T265_R_OFFSET_M); cmd=FixedCommandSource(vx,vy,wz); last=np.zeros(10,np.float32)
    rows=[]; bus_opened=False; t265_start_attempted=False
    abort_message=None
    # D10-8: every policy evaluation's full 42-dim observation and raw action,
    # so what the policy SAW can be judged from the saved log.
    obs_rows=[]
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
        _s, initial_obs, initial_out, _plan = evaluate_cycle(
            policy, t265.latest(), feedback, cmd.sample(time.monotonic()), last
        )
        if initial_obs is not None:
            obs_rows.append((time.monotonic(), "initial", *map(float, initial_obs),
                             *map(float, initial_out.action_raw)))
            g = np.asarray(initial_obs[6:9], dtype=float)
            tilt = float(np.rad2deg(np.arccos(np.clip(-g[2] / max(np.linalg.norm(g), 1e-9), -1, 1))))
            print(f"INITIAL OBS: gravity=({g[0]:+.3f},{g[1]:+.3f},{g[2]:+.3f}) tilt={tilt:.1f}deg; "
                  "pos_rel(deg)=" + ", ".join(
                      f"{name}={np.rad2deg(value):+.1f}"
                      for name, value in zip(OBS_JOINT_NAMES, initial_obs[12:22])))
        initial_targets = initial_out.joint_target_h_order
        # MotorFeedback is the shell boundary object; its values are already
        # radians, under the public `position` / `velocity` names.
        initial_positions = tuple(feedback[mid].position for mid in H_CAN_IDS)
        print("Initial policy targets (no MIT sent yet): " + ", ".join(
            f"0x{mid:02X} target={np.rad2deg(target):+.1f}deg "
            f"delta={np.rad2deg(target-position):+.1f}deg"
            for mid, target, position in zip(H_CAN_IDS, initial_targets, initial_positions)
        ))
        # With a stand stage the open-loop ramp goes to the sim default pose,
        # not to the policy's first output, so that is what the gate checks.
        ramp_goal = tuple(stand_target) if stand_seconds else tuple(initial_targets)
        if stand_seconds:
            print("STAND TARGET (sim default pose, no MIT sent yet): " + ", ".join(
                f"0x{mid:02X} target={np.rad2deg(target):+.1f}deg "
                f"delta={np.rad2deg(target-position):+.1f}deg"
                for mid, target, position in zip(H_CAN_IDS, stand_target, initial_positions)
            ))
        delta_limit = STAND_RAMP_MAX_DELTA_DEG if stand_seconds else ALL_AXES_MAX_DELTA_DEG
        violations = all_axes_prearm_violations(ramp_goal, initial_positions, delta_limit)
        if violations:
            print(f"ALL-AXES PRE-ARM GATE: would REFUSE a whole-body run "
                  f"(|target|>{ALL_AXES_MAX_TARGET_DEG:g}deg or |delta|>"
                  f"{delta_limit:g}deg): " + "; ".join(violations))
        else:
            print(f"ALL-AXES PRE-ARM GATE: ok (every |target|<={ALL_AXES_MAX_TARGET_DEG:g}deg "
                  f"and |delta|<={delta_limit:g}deg).")
        if not transmit:
            print("PREFLIGHT complete: CAN transmit count is zero.")
            return
        if len(motor_ids) > 1:
            if policy_slew_rad_s is None and not stand_only:
                raise RuntimeError("a whole-body policy run requires --policy-slew-dps; "
                                   "no MIT frame was sent")
            if violations:
                raise RuntimeError("ALL-AXES PRE-ARM GATE refused before any MIT frame: "
                                   + "; ".join(violations)
                                   + ". Hold the legs near the stand pose (straight down) and start again; "
                                   "no new D7 `oa` is needed unless scan shows the origin moved.")

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
            ramped = ramp_targets(initial_positions, ramp_goal,
                                  tick - ramp_start, ramp_seconds)
            # Re-read feedback before every frame batch.  The established
            # stale/error/current/speed gates remain active during the ramp.
            bus.feedback()
            for fr in frames(ramped, motor_ids):
                bus.send(fr)
            rows.extend(axis_rows(tick, "stand-ramp" if stand_seconds else "ramp",
                                  ramp_goal, ramped, bus, motor_ids))
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
        print(f"RAMP complete: CAN tx={bus.tx_count}; entering "
              f"{'stand hold' if stand_seconds else 'policy hold'}.")
        if stand_seconds:
            print(f"STAND HOLD: holding the sim default pose for {stand_seconds:g}s. "
                  "Hands off the legs now.")
            stand_start = time.monotonic()
            stand_end = stand_start + stand_seconds
            stand_next = stand_start
            window = []
            next_progress = stand_start + STAND_PROGRESS_S
            hold_target = tuple(stand_target)
            lean = 0.0
            ffe = ffe_indices()
            if auto_lean_rad:
                print(f"AUTO LEAN: from {AUTO_LEAN_START_S:g}s into the hold, both FFE lean toe-down "
                      f"to balance (0..{np.rad2deg(abs(auto_lean_rad)):.1f}deg). Lower the hoist before that.")
            while time.monotonic() < stand_end:
                time.sleep(max(0, stand_next - time.monotonic()))
                tick = time.monotonic()
                feedback = bus.feedback()
                if auto_lean_rad and tick - stand_start >= AUTO_LEAN_START_S:
                    errors = [hold_target[i] - feedback[H_CAN_IDS[i]].position for i in ffe]
                    lean = auto_lean_step(lean, errors, PERIOD, auto_lean_rad)
                    shifted = list(stand_target)
                    for i in ffe:
                        shifted[i] = stand_target[i] + lean
                    hold_target = tuple(shifted)
                for fr in frames(hold_target, motor_ids):
                    bus.send(fr)
                new_rows = axis_rows(tick, "stand-hold", hold_target, hold_target,
                                     bus, motor_ids)
                rows.extend(new_rows)
                timing_rows.append(timing_row(tick, "stand-hold",
                                              timing_rows[-1][0] if timing_rows else None,
                                              t265.latest()))
                if stand_seconds > STAND_MAX_SECONDS:
                    window.extend(new_rows)
                    if tick >= next_progress:
                        line = stand_progress_line(tick - stand_start, stand_seconds, window)
                        if auto_lean_rad:
                            line += (f"; lean=+{np.rad2deg(lean):.1f}deg FFE err "
                                     + "/".join(f"{np.rad2deg(hold_target[i] - feedback[H_CAN_IDS[i]].position):+.1f}"
                                                for i in ffe))
                        print(line)
                        window = []
                        next_progress += STAND_PROGRESS_S
                stand_next += PERIOD
            if auto_lean_rad:
                print(f"AUTO LEAN RESULT: final lean +{np.rad2deg(lean):.1f}deg "
                      f"(limit {np.rad2deg(abs(auto_lean_rad)):.1f}); the policy starts from this pose."
                      + (" LIMIT REACHED: still on the heels." if lean >= abs(auto_lean_rad) - 1e-6 else ""))
            stand_target = hold_target
            print(f"STAND HOLD complete: CAN tx={bus.tx_count}.")
            print_look_check(stand_target)
            report_hold_pose([row for row in rows if row[1] == "stand-hold"],
                             stand_target, motor_ids, current_limits)
            if haa_close_rad:
                run_haa_sweep(bus, rows, stand_target, haa_close_rad, motor_ids)
            if lean_sweep_rad:
                run_lean_sweep(bus, rows, stand_target, lean_sweep_rad, motor_ids)
            if stand_only:
                print("STAND ONLY: no policy stage. Sending zero MIT cleanup.")
                return
            if start_gate_rad:
                gate_deg = float(np.rad2deg(start_gate_rad))
                print(f"START GATE: the policy starts once |pitch| and |roll| <= {gate_deg:g}deg "
                      f"for {START_GATE_HOLD_S:g}s (timeout {START_GATE_TIMEOUT_S:g}s). "
                      "Hold the body upright by hand or rope, then let go.")
                gate_start = time.monotonic()
                gate_next = gate_start
                next_print = gate_start
                upright_since = None
                opened = False
                while True:
                    time.sleep(max(0, gate_next - time.monotonic()))
                    tick = time.monotonic()
                    bus.feedback()
                    for fr in frames(stand_target, motor_ids):
                        bus.send(fr)
                    rows.extend(axis_rows(tick, "stand-gate", stand_target, stand_target,
                                          bus, motor_ids))
                    sample = t265.latest()
                    timing_rows.append(timing_row(tick, "stand-gate",
                                                  timing_rows[-1][0] if timing_rows else None, sample))
                    pitch, roll = tilt_deg(sample)
                    upright = (pitch is not None and abs(pitch) <= gate_deg and abs(roll) <= gate_deg)
                    upright_since = (upright_since if upright_since is not None else tick) if upright else None
                    if upright_since is not None and tick - upright_since >= START_GATE_HOLD_S:
                        opened = True
                        break
                    if tick >= next_print:
                        if pitch is None:
                            print("START GATE: no T265 tilt yet")
                        else:
                            print(f"START GATE: pitch {pitch:+.1f}deg ({'forward' if pitch >= 0 else 'BACK'}) "
                                  f"roll {roll:+.1f}deg -> {'upright' if upright else 'not upright'}")
                        next_print += START_GATE_PRINT_S
                    if tick - gate_start >= START_GATE_TIMEOUT_S:
                        break
                    gate_next += PERIOD
                if not opened:
                    abort_message = "START GATE timeout: never upright; no policy stage"
                    print(f"START GATE timeout after {START_GATE_TIMEOUT_S:g}s: the body never stayed "
                          "upright. No policy stage; sending zero MIT cleanup.")
                    return
                print(f"START GATE open after {tick - gate_start:.1f}s: pitch {pitch:+.1f}deg, "
                      f"roll {roll:+.1f}deg. Policy starts now.")
            # The sim starts every episode with a zero previous action; the
            # action computed at the closed-leg start pose is stale by now.
            last = np.zeros(ACTION_SIZE, np.float32)
        end=time.monotonic()+duration; nxt=time.monotonic()
        commanded_target = initial_targets[selected_index]
        commanded_all = tuple(stand_target) if stand_seconds else tuple(ramp_goal)
        if policy_slew_rad_s is not None:
            print(f"POLICY SLEW: every axis limited to {np.rad2deg(policy_slew_rad_s):.1f}deg/s "
                  f"from the {'stand pose' if stand_seconds else 'frozen ramp target'} toward the live policy output.")
        previous_policy_tick = time.monotonic()
        last_policy_progress = -1
        policy_start = time.monotonic()
        zero_cmd = FixedCommandSource(0.0, 0.0, 0.0)
        command_started = False
        if command_delay_s is not None:
            print(f"COMMAND DELAY: zero command for the first {command_delay_s:g}s of the policy stage.")
        try:
            while time.monotonic()<end:
                time.sleep(max(0,nxt-time.monotonic())); tick=time.monotonic()
                use_cmd = cmd if (command_delay_s is None or tick - policy_start >= command_delay_s) else zero_cmd
                if command_delay_s is not None and not command_started and use_cmd is cmd:
                    print(f"COMMAND: vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f} from now "
                          f"({tick - policy_start:.2f}s into the policy stage).")
                    command_started = True
                t265_sample = t265.latest()
                timing_rows.append(timing_row(tick, "policy",
                                              timing_rows[-1][0] if timing_rows and timing_rows[-1][1] == "policy" else None,
                                              t265_sample))
                _s,_o,out,plan=evaluate_cycle(policy,t265_sample,bus.feedback(),use_cmd.sample(tick),last)
                if _o is not None:
                    obs_rows.append((tick, "policy", *map(float, _o), *map(float, out.action_raw)))
                desired_target = out.joint_target_h_order[selected_index]
                if policy_slew_rad_s is not None:
                    commanded_all = slew_all(commanded_all, out.joint_target_h_order,
                                             policy_slew_rad_s, tick - previous_policy_tick)
                    targets = list(commanded_all)
                    commanded_target = targets[selected_index]
                else:
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
        except RuntimeError as error:
            if soft_stop_s and is_soft_stop_abort(error):
                print(f"ABORT in the policy stage: {error}")
                abort_message = f"{type(error).__name__}: {error}"
                abort_message += " | " + soft_stop(bus, rows, motor_ids, current_limits, soft_stop_s)
            raise
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
            report_all_axes(rows, initial_positions, ramp_goal, motor_ids)
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
    except BaseException as error:
        # D10-13B: the abort reason used to exist only on the screen.
        if abort_message is None:
            abort_message = f"{type(error).__name__}: {error}"
        raise
    finally:
        # Cleanup must never be skipped, including a stale-feedback or USB-open
        # failure.  Attempt all shutdown steps even if one of them fails.
        if abort_message is not None and csv_path is not None:
            try:
                path = abort_txt_path(csv_path)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(time.strftime("%Y-%m-%d %H:%M:%S") + " " + abort_message + "\n",
                                encoding="utf-8")
            except Exception as error:
                print(f"WARNING: abort record failed: {error}")
        try:
            report_timing(timing_rows, "policy")
        except Exception as error:
            print(f"WARNING: loop timing report failed: {error}")
        try:
            if timing_rows and csv_path is not None:
                write_timing_csv(timing_csv_path(csv_path), timing_rows)
        except Exception as error:
            print(f"WARNING: timing CSV failed: {error}")
        disable_fine_timer(timer_state)
        if policy_slew_rad_s is not None and any(row[1] == "policy" for row in rows):
            try:
                report_slew_gap(rows, motor_ids)
            except Exception as error:
                print(f"WARNING: slew gap report failed: {error}")
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
            if obs_rows:
                write_obs_csv(obs_csv_path(csv_path), obs_rows)
        except Exception as error:
            print(f"WARNING: observation CSV cleanup failed: {error}")
        try:
            csv_path.parent.mkdir(parents=True,exist_ok=True)
            with csv_path.open('w',newline='',encoding='utf-8') as f:
                w=csv.writer(f)
                w.writerow((
                    'tick', 'stage', 'sent_motor_id', 'desired_target_rad',
                    'requested_target_rad', 'wire_target_motor_rad', 'feedback_position_rad',
                    'feedback_velocity_rad_s', 'feedback_current_h_a',
                ))
                w.writerows(rows)
        except Exception as error:
            print(f"WARNING: CSV cleanup failed: {error}")

OBS_TERMS = (
    ("lin_vel", 0, 3), ("ang_vel", 3, 6), ("gravity", 6, 9), ("command", 9, 12),
    ("pos_rel", 12, 22), ("joint_vel", 22, 32), ("last_action", 32, 42),
)
OBS_JOINT_NAMES = ("LL_HR", "LR_HR", "LL_HAA", "LR_HAA", "LL_HFE", "LR_HFE",
                   "LL_KFE", "LR_KFE", "LL_FFE", "LR_FFE")


def timing_csv_path(csv_path):
    """Loop timing and T265 tilt per tick (D10-13C)."""
    csv_path = Path(csv_path)
    return csv_path.with_name(csv_path.stem + "_timing.csv")


TIMING_HEADER = ("tick", "stage", "loop_dt_s", "t265_age_s", "t265_confidence",
                 "gravity_x", "gravity_y", "gravity_z", "pitch_fwd_deg", "roll_deg")


def tilt_deg(sample):
    """(pitch forward +, roll) in degrees from a T265 sample, or (None, None)."""
    gravity = getattr(sample, "projected_gravity", None)
    if gravity is None:
        return None, None
    gx, gy, gz = (float(v) for v in gravity)
    return float(np.rad2deg(np.arctan2(gx, -gz))), float(np.rad2deg(np.arctan2(gy, -gz)))


def timing_row(tick, stage, previous_tick, sample):
    gravity = getattr(sample, "projected_gravity", None)
    acquired = getattr(sample, "acquired_monotonic_s", None)
    pitch, roll = tilt_deg(sample)
    nan = float("nan")
    gx, gy, gz = (float(v) for v in gravity) if gravity is not None else (nan, nan, nan)
    return (tick, stage, nan if previous_tick is None else tick - previous_tick,
            nan if acquired is None else tick - float(acquired),
            getattr(sample, "confidence", ""), gx, gy, gz,
            nan if pitch is None else pitch, nan if roll is None else roll)


def write_timing_csv(path, timing_rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(TIMING_HEADER)
        writer.writerows(timing_rows)


def report_timing(timing_rows, stage="policy"):
    """Loop period and T265 sample age of one stage.  Sends nothing."""
    dts = np.array([float(r[2]) for r in timing_rows if r[1] == stage and np.isfinite(float(r[2]))])
    ages = np.array([float(r[3]) for r in timing_rows if r[1] == stage and np.isfinite(float(r[3]))])
    if not len(dts):
        return None
    late = int(np.sum(dts > TIMING_LATE_S))
    early = int(np.sum(dts < 0.010))
    text = (f"LOOP TIMING ({stage}): {len(dts) + 1} ticks, dt median {np.median(dts) * 1e3:.1f}ms "
            f"p95 {np.percentile(dts, 95) * 1e3:.1f}ms max {dts.max() * 1e3:.1f}ms; "
            f">{TIMING_LATE_S * 1e3:.0f}ms: {late}, <10ms: {early}")
    if len(ages):
        text += f"; T265 age median {np.median(ages) * 1e3:.1f}ms max {ages.max() * 1e3:.1f}ms"
    print(text)
    return float(np.median(dts))


def report_timing_csv(path):
    with path.open(newline="", encoding="utf-8") as file:
        records = list(csv.reader(file))[1:]
    stages = list(dict.fromkeys(r[1] for r in records))
    for stage in stages:
        report_timing(records, stage)
    gate = [r for r in records if r[1] in ("stand-hold", "stand-gate")]
    if gate:
        last = gate[-1]
        print(f"TILT at the policy switch: pitch {float(last[8]):+.1f}deg (+ = forward), "
              f"roll {float(last[9]):+.1f}deg")


def enable_fine_timer():
    """perf_counter as the loop clock and a 1 ms Windows timer.  Returns the undo state."""
    state = {"monotonic": time.monotonic, "winmm": False}
    time.monotonic = time.perf_counter
    try:
        import ctypes
        state["winmm"] = ctypes.windll.winmm.timeBeginPeriod(1) == 0
    except Exception:
        state["winmm"] = False
    print("FINE TIMER: loop clock = perf_counter; Windows 1 ms timer "
          + ("on." if state["winmm"] else "not available (not Windows?)."))
    return state


def disable_fine_timer(state):
    if not state:
        return
    time.monotonic = state["monotonic"]
    if state["winmm"]:
        try:
            import ctypes
            ctypes.windll.winmm.timeEndPeriod(1)
        except Exception:
            pass


def unique_csv_path(path):
    """Never overwrite an earlier run: return path, or path_2, path_3, ... (D10-13C)."""
    path = Path(path)
    def taken(candidate):
        return any(x.exists() for x in (candidate, obs_csv_path(candidate), abort_txt_path(candidate),
                                         timing_csv_path(candidate), meta_txt_path(candidate)))
    if not taken(path):
        return path
    for number in range(2, 1000):
        candidate = path.with_name(f"{path.stem}_{number}{path.suffix}")
        if not taken(candidate):
            return candidate
    raise RuntimeError(f"no free CSV name next to {path}")


def meta_txt_path(csv_path):
    """Build, command line and zero offset of one run (D10-13D)."""
    csv_path = Path(csv_path)
    return csv_path.with_name(csv_path.stem + "_meta.txt")


def abort_txt_path(csv_path):
    """The one-line abort record that belongs to one run CSV (D10-13B)."""
    csv_path = Path(csv_path)
    return csv_path.with_name(csv_path.stem + "_abort.txt")


def is_soft_stop_abort(error):
    """True for the aborts a soft stop may follow: speed, current, origin.

    Stale feedback, a motor error or a lost route mean the feedback itself
    cannot be trusted, so those always go straight to zero MIT.
    """
    return isinstance(error, RuntimeError) and str(error).startswith(SOFT_STOP_ABORT_PREFIXES)


def soft_stop(bus, rows, motor_ids, current_limits, seconds):
    """Hold every driven axis where it stopped, with the stand-hold gains.

    Returns the one-line outcome (complete, skipped or ended early and why);
    the caller appends it to the abort record.  Either way the caller's
    cleanup sends zero MIT afterwards.
    """
    targets = list(STAND_TARGET)
    for mid in motor_ids:
        state = bus.state(mid)
        if state is None or time.time() - state.t > STALE_S or state.err:
            reason = f"SOFT STOP skipped: 0x{mid:02X} has no fresh error-free feedback; zero MIT now."
            print(reason)
            return reason
        position, _velocity, _current = h_feedback(mid, state)
        targets[H_CAN_IDS.index(mid)] = position
    print(f"SOFT STOP: holding every axis where it stopped (stand-hold gains) for {seconds:g}s, "
          "then zero MIT. Catch the body before it ends.")
    end = time.monotonic() + seconds
    next_tick = time.monotonic()
    while time.monotonic() < end:
        time.sleep(max(0.0, next_tick - time.monotonic()))
        tick = time.monotonic()
        for mid in motor_ids:
            state = bus.state(mid)
            if state is None or time.time() - state.t > STALE_S or state.err:
                reason = (f"SOFT STOP ended early after {tick - end + seconds:.2f}s: 0x{mid:02X} "
                          "stale feedback or motor error; zero MIT now.")
                print(reason)
                return reason
            limit = current_limits.get(mid, CURRENT_ABORT_A)
            if abs(state.cur) > limit:
                reason = (f"SOFT STOP ended early after {tick - end + seconds:.2f}s: 0x{mid:02X} "
                          f"cur={state.cur:+.2f}A above {limit:.2f}A; zero MIT now.")
                print(reason)
                return reason
        for frame in frames(targets, motor_ids):
            bus.send(frame)
        rows.extend(axis_rows(tick, "soft-stop", targets, targets, bus, motor_ids))
        next_tick += PERIOD
    reason = f"SOFT STOP complete after {seconds:g}s."
    print(reason)
    return reason


def obs_csv_path(csv_path):
    """The observation sidecar that belongs to one run CSV."""
    csv_path = Path(csv_path)
    return csv_path.with_name(csv_path.stem + "_obs.csv")


def obs_header():
    names = []
    for term, lo, hi in OBS_TERMS:
        if term in ("pos_rel", "joint_vel", "last_action"):
            names += [f"obs_{term}_{joint}" for joint in OBS_JOINT_NAMES]
        else:
            names += [f"obs_{term}_{axis}" for axis in "xyz"[: hi - lo]]
    return ("tick", "stage", *names, *[f"action_raw_{joint}" for joint in OBS_JOINT_NAMES])


def write_obs_csv(path, obs_rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(obs_header())
        writer.writerows(obs_rows)


def report_observation(path):
    """What the policy saw, from the saved sidecar: body tilt, rotation, joints.

    Hanging on a rope, the body should read level (gravity ~ (0,0,-1)), still
    and with joint_pos_rel near zero at the stand pose.  Anything else is
    something the policy reacts to that the eye does not see.
    """
    with path.open(newline="", encoding="utf-8") as file:
        records = list(csv.DictReader(file))
    policy = [r for r in records if r["stage"] == "policy"]
    initial = [r for r in records if r["stage"] == "initial"]
    print(f"OBSERVATION ({path.name}): {len(policy)} policy tick(s).")
    if initial:
        rel = [float(initial[0][f"obs_pos_rel_{joint}"]) for joint in OBS_JOINT_NAMES]
        print("  initial pos_rel at program start, before the ramp (deg): " + ", ".join(
            f"{joint}={np.rad2deg(value):+.1f}" for joint, value in zip(OBS_JOINT_NAMES, rel)))
    if not policy:
        return None
    def column(name):
        return np.array([float(r[name]) for r in policy])
    gravity = np.stack([column(f"obs_gravity_{axis}") for axis in "xyz"], axis=1)
    mean_g = gravity.mean(axis=0)
    tilt = np.rad2deg(np.arccos(np.clip(-mean_g[2] / max(np.linalg.norm(mean_g), 1e-9), -1, 1)))
    roll = np.rad2deg(np.arctan2(mean_g[1], -mean_g[2]))
    pitch = np.rad2deg(np.arctan2(-mean_g[0], -mean_g[2]))
    print(f"  gravity mean=({mean_g[0]:+.3f},{mean_g[1]:+.3f},{mean_g[2]:+.3f}) "
          f"tilt={tilt:.1f}deg (roll~{roll:+.1f}, pitch~{pitch:+.1f}; sim golden walks at ~2deg)")
    for term in ("lin_vel", "ang_vel"):
        values = np.stack([column(f"obs_{term}_{axis}") for axis in "xyz"], axis=1)
        print(f"  {term} mean=({', '.join(f'{v:+.2f}' for v in values.mean(axis=0))}) "
              f"max|.|=({', '.join(f'{v:.2f}' for v in np.abs(values).max(axis=0))})")
    command = [column(f"obs_command_{axis}").mean() for axis in "xyz"]
    print(f"  command mean=({command[0]:+.2f},{command[1]:+.2f},{command[2]:+.2f})")
    print("  pos_rel mean (deg): " + ", ".join(
        f"{joint}={np.rad2deg(column(f'obs_pos_rel_{joint}').mean()):+.1f}" for joint in OBS_JOINT_NAMES))
    print("  action_raw mean: " + ", ".join(
        f"{joint}={column(f'action_raw_{joint}').mean():+.2f}" for joint in OBS_JOINT_NAMES))
    return tilt


def analyze_csv(csv_path):
    """Re-judge a saved one-axis CSV.  Opens no bus and sends no CAN frame."""
    with csv_path.open(newline="", encoding="utf-8") as file:
        records = list(csv.DictReader(file))
    if not records:
        raise RuntimeError(f"no rows in {csv_path}")
    motor_id = int(records[0]["sent_motor_id"], 16)
    if motor_id not in H_CAN_IDS:
        raise RuntimeError(f"{csv_path} is not a registered H axis")
    # CSVs written before D10-6 (build D10_5 and older) used the old column
    # names and were recorded with every joint sign assumed +1, i.e. their
    # H-frame columns are really motor-frame for the five sign -1 joints.
    legacy = "wire_target_motor_rad" not in records[0]
    if legacy:
        print("NOTE: pre-D10-6 CSV: all joint signs were +1 when it was recorded.")
    wire_key = "wire_target_rad" if legacy else "wire_target_motor_rad"
    current_key = "feedback_current_a" if legacy else "feedback_current_h_a"
    rows = [(
        float(record["tick"]), record["stage"], record["sent_motor_id"],
        float(record["desired_target_rad"]), float(record["requested_target_rad"]),
        float(record[wire_key]), float(record["feedback_position_rad"]),
        float(record["feedback_velocity_rad_s"]), float(record[current_key]),
    ) for record in records]
    kp_cmd = wire_command(motor_id, rows[0][5])[0]
    initial_position = rows[0][6]
    initial_target = rows[0][3]
    meta_path = meta_txt_path(csv_path)
    if meta_path.exists():
        print("RUN META: " + " | ".join(meta_path.read_text(encoding="utf-8").strip().splitlines()))
    timing_path = timing_csv_path(csv_path)
    if timing_path.exists():
        report_timing_csv(timing_path)
    abort_path = abort_txt_path(csv_path)
    if abort_path.exists():
        print(f"ABORT RECORD ({abort_path.name}): {abort_path.read_text(encoding='utf-8').strip()}")
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
        if not any(row[1] in ("policy", "probe-hold", "hold-pose", "stand-hold") for row in rows):
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
            # The frozen ramp target, not the last live policy output: after
            # the handover `desired` is whatever the policy said on the tick
            # the run stopped, which is not what the ramp asked for.
            ramp = [row for row in axis if row[1] in ("ramp", "stand-ramp")]
            target[mid] = ramp[0][3] if ramp else axis[-1][3]
        positions = tuple(start.get(mid, 0.0) for mid in H_CAN_IDS)
        targets = tuple(target.get(mid, start.get(mid, 0.0)) for mid in H_CAN_IDS)
        if rows[0][1] == "hold-pose":
            report_hold_pose(rows, positions, logged_ids, default_current_limits())
        else:
            stand = [row for row in rows if row[1] == "stand-hold"]
            if stand:
                # The stand target is read from the CSV itself: --sign-pose
                # (D10-7) holds a different pose than the sim default.
                stand_target = list(STAND_TARGET)
                for mid in logged_ids:
                    axis_stand = rows_for_axis(stand, mid)
                    if axis_stand:
                        stand_target[H_CAN_IDS.index(mid)] = axis_stand[0][3]
                stand_target = tuple(stand_target)
                label = ("sim default pose" if np.allclose(stand_target, STAND_TARGET)
                         else "sign pose" if np.allclose(stand_target, SIGN_POSE_TARGET)
                         else "custom stand pose (--stand-knee-deg / --stand-lean-deg)")
                print(f"STAND HOLD ({label}):")
                print_look_check(stand_target)
                report_hold_pose(stand, stand_target, logged_ids, default_current_limits())
            report_all_axes(rows, positions, targets, logged_ids)
            report_slew_gap(rows, logged_ids)
            report_haa_sweep(rows)
            report_lean_sweep(rows)
        sidecar = obs_csv_path(csv_path)
        if sidecar.exists():
            report_observation(sidecar)
        elif any(row[1] == "policy" for row in rows):
            print("NOTE: no observation sidecar (runs before D10-8 did not save one).")
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
    p.add_argument('--floor-limits', action='store_true', help=f'D10-10, feet on the floor with the hoist rope still attached: the --gravity-limits table with KFE and FFE raised to {FLOOR_CURRENT_ABORT_A_BY_JOINT["KFE"]:g} / {FLOOR_CURRENT_ABORT_A_BY_JOINT["FFE"]:g} A, and --stand-seconds allowed up to {FLOOR_STAND_MAX_SECONDS:g} s so the hoist can be lowered during the hold. Requires --all-axes and --stand-seconds; not with --gravity-limits. Needs explicit user approval')
    p.add_argument('--policy-slew-dps', type=float, help=f'with --arm --all-axes (required there): rate-limit EVERY axis target in the policy stage to this many deg/s, 0 < value <= {POLICY_SLEW_MAX_DPS:g}. Removes the ramp->policy step that tripped D10-3 R3/R4')
    p.add_argument('--stand-seconds', type=float, help=f'with --arm --all-axes: ramp to the sim default pose (HAA 0, HFE -10, KFE +20, FFE -10 deg) over --ramp-seconds, hold it this long (0 < value <= {STAND_MAX_SECONDS:g}), then start the slew-limited policy from there')
    p.add_argument('--sign-pose', action='store_true', help=f'with --stand-only (D10-7): hold the sim default pose plus {SIGN_POSE_EXTRA_DEG:g} deg outward on HR and HAA of both legs, so every joint sign can be checked by eye')
    p.add_argument('--haa-close-deg', type=float, help=f'with --stand-only (D10-9): after the stand hold, move BOTH HAA inward together at {HAA_SWEEP_DPS:g} deg/s up to this many deg (0 < value <= {HAA_CLOSE_MAX_DEG:g}) and log the angle at which the feet stop each other; both HAA freeze there')
    p.add_argument('--stand-knee-deg', type=float, help=f'D10-11: stand pose HFE=-K/2, KFE=+K, FFE=-K/2 (sole parallel to the body) instead of the sim default (K=20); 0 <= K <= {STAND_KNEE_MAX_DEG:g}')
    p.add_argument('--stand-lean-deg', type=float, help=f'D10-11: add this many deg to both FFE stand targets (+ = toe down = robot leans FORWARD once the sole is flat on the floor); {STAND_LEAN_MIN_DEG:g} <= L <= {STAND_LEAN_MAX_DEG:g}')
    p.add_argument('--lean-sweep-deg', type=float, help=f'D10-11, with --floor-limits --stand-only: after the stand hold, add toe-down lean to both FFE at {LEAN_SWEEP_DPS:g} deg/s up to this many deg (0 < value <= {LEAN_SWEEP_MAX_DEG:g}) and log where the robot balances over its ankles')
    p.add_argument('--auto-lean-deg', type=float, help=f'D10-12, with --floor-limits or --walk-limits and --stand-seconds >= {AUTO_LEAN_START_S + 4:g}: from {AUTO_LEAN_START_S:g}s into the stand hold, lean both FFE toe-down until the ankles stop being pushed toe-up (0 < max <= {STAND_LEAN_MAX_DEG:g} deg); the policy starts from the leaned pose')
    p.add_argument('--walk-limits', action='store_true', help=f'D10-12, policy on the floor with the hoist rope attached: --floor-limits with KFE/FFE {WALK_CURRENT_ABORT_A_BY_JOINT["KFE"]:g} A, speed abort {np.rad2deg(WALK_SPEED_ABORT_RAD_S):.0f} deg/s, --policy-slew-dps allowed up to {WALK_POLICY_SLEW_MAX_DPS:g}. Needs explicit user approval')
    p.add_argument('--walk-speed-abort-dps', type=float, help=f'D10-13B, with --walk-limits: speed abort in deg/s ({np.rad2deg(WALK_SPEED_ABORT_RAD_S):.0f} <= value <= {WALK_SPEED_ABORT_MAX_DPS:g}; default {np.rad2deg(WALK_SPEED_ABORT_RAD_S):.0f}). Needs explicit user approval')
    p.add_argument('--soft-stop-seconds', type=float, help=f'D10-13B, with --walk-limits: after a speed/current/origin abort in the policy stage, hold every axis where it stopped for this long (0 < value <= {SOFT_STOP_MAX_S:g}) before zero MIT')
    p.add_argument('--start-gate-deg', type=float, help=f'D10-13C, with --walk-limits: after --stand-seconds keep holding until the T265 reads |pitch| and |roll| <= G deg for {START_GATE_HOLD_S:g}s, then start the policy ({START_GATE_MIN_DEG:g} <= G <= {START_GATE_MAX_DEG:g}; no policy after {START_GATE_TIMEOUT_S:g}s)')
    p.add_argument('--zero-offset', choices=sorted(ZERO_OFFSET_PRESETS_DEG), help='D10-13D, with --all-axes: map the D7 origin (legs straight by eye) to the sim joint zero (CAD pose) with a fixed per-joint offset; see ZERO_OFFSET_PRESETS_DEG')
    p.add_argument('--fine-timer', action='store_true', help='D10-13C: perf_counter loop clock and a 1 ms Windows timer (Python 3.10 time.monotonic steps 15.6 ms)')
    p.add_argument('--command-delay-seconds', type=float, help=f'D10-13, with --walk-limits: keep the velocity command at zero for this long after the policy starts (0 < value <= {COMMAND_DELAY_MAX_S:g}), then use --vx/--vy/--wz')
    p.add_argument('--stand-only', action='store_true', help='with --stand-seconds: stop after the stand hold; no policy stage, no --duration, no --policy-slew-dps')
    p.add_argument('--analyze', type=Path, help='re-judge a saved one-axis CSV offline; opens no CAN bus');    p.add_argument('--preview',action='store_true'); p.add_argument('--package',type=Path); p.add_argument('--duration',type=float,default=0.); p.add_argument('--csv',type=Path); p.add_argument('--vx',type=float,default=0.); p.add_argument('--vy',type=float,default=0.); p.add_argument('--wz',type=float,default=0.); p.add_argument('--ramp-seconds',type=float, help='required with --arm; initial policy target is reached linearly over this time'); p.add_argument('--motor-id', type=lambda value: int(value, 0), action='append', help='required once with --arm; only this registered motor receives MIT frames')
    a=p.parse_args()
    current_mode = 'hold-pose' if a.hold_pose else 'static-probe' if a.static_probe else 'arm' if a.arm else 'preflight' if a.preflight else 'none'
    print(f"ver9_d8_sender build={BUILD_ID}; mode={current_mode}")
    print(joint_sign_line())
    if a.analyze: return analyze_csv(a.analyze)
    if a.preview: preview(); return 0
    if not (a.arm or a.preflight): p.error('--arm is required for transmission; use --preflight for a receive-only live check')
    if a.gravity_limits and not a.all_axes:
        p.error('--gravity-limits is only for a whole-body run; pass --all-axes, '
                'or leave the one-axis limit alone')
    if a.gravity_limits and a.static_probe:
        p.error('--gravity-limits must not be combined with --static-probe; a one-axis '
                'probe carries no load and its limit is not the thing under test')
    if a.walk_limits:
        if a.gravity_limits or a.floor_limits:
            p.error('--walk-limits already contains the floor table; do not also pass --floor-limits/--gravity-limits')
        if not a.all_axes or a.stand_seconds is None or a.stand_only or a.policy_slew_dps is None:
            p.error('--walk-limits is for a policy run on the floor: --all-axes, --stand-seconds, '
                    '--policy-slew-dps, no --stand-only')
        a.floor_limits = True
    if a.floor_limits:
        if a.gravity_limits:
            p.error('--floor-limits already contains the --gravity-limits table; pass one of them')
        if not a.all_axes or a.hold_pose or a.static_probe or a.stand_seconds is None:
            p.error('--floor-limits is for a whole-body stand on the floor: --all-axes and '
                    '--stand-seconds, without --hold-pose or --static-probe')
        if a.haa_close_deg is not None or a.sign_pose:
            p.error('--floor-limits is not for the hanging checks (--haa-close-deg, --sign-pose)')
    for flag, value in (("--walk-speed-abort-dps", a.walk_speed_abort_dps),
                        ("--soft-stop-seconds", a.soft_stop_seconds)):
        if value is not None and not a.walk_limits:
            p.error(f'{flag} is for the floor walking run: pass --walk-limits')
    walk_speed_dps = float(np.rad2deg(WALK_SPEED_ABORT_RAD_S))
    if a.walk_speed_abort_dps is not None:
        if not walk_speed_dps <= a.walk_speed_abort_dps <= WALK_SPEED_ABORT_MAX_DPS:
            p.error(f'--walk-speed-abort-dps must satisfy {walk_speed_dps:.0f} <= value <= {WALK_SPEED_ABORT_MAX_DPS:g}')
        walk_speed_dps = float(a.walk_speed_abort_dps)
    if a.start_gate_deg is not None:
        if not a.walk_limits:
            p.error('--start-gate-deg is for the floor walking run: pass --walk-limits')
        if not START_GATE_MIN_DEG <= a.start_gate_deg <= START_GATE_MAX_DEG:
            p.error(f'--start-gate-deg must satisfy {START_GATE_MIN_DEG:g} <= value <= {START_GATE_MAX_DEG:g}')
    if a.soft_stop_seconds is not None and not 0 < a.soft_stop_seconds <= SOFT_STOP_MAX_S:
        p.error(f'--soft-stop-seconds must satisfy 0 < value <= {SOFT_STOP_MAX_S:g}')
    limits = (walk_current_limits() if a.walk_limits
              else floor_current_limits() if a.floor_limits
              else gravity_current_limits() if a.gravity_limits else default_current_limits())
    if a.walk_limits:
        print("WALK LIMITS: per-axis current abort " + ", ".join(
            f"0x{mid:02X} {H_BINDING_BY_ID[mid].name}={limits[mid]:.1f}A" for mid in H_CAN_IDS)
            + f"; speed abort {walk_speed_dps:.0f}deg/s; slew allowed up to "
            f"{WALK_POLICY_SLEW_MAX_DPS:g}deg/s. Keep the hoist rope attached.")
        if a.soft_stop_seconds is not None:
            print(f"SOFT STOP armed: after a speed/current/origin abort in the policy stage, "
                  f"hold where it stopped for {a.soft_stop_seconds:g}s, then zero MIT.")
    elif a.floor_limits:
        print("FLOOR LIMITS: per-axis current abort " + ", ".join(
            f"0x{mid:02X} {H_BINDING_BY_ID[mid].name}={limits[mid]:.1f}A" for mid in H_CAN_IDS))
        print(f"FLOOR LIMITS: --gravity-limits with KFE/FFE raised for a loaded stand "
              f"(URDF static stance estimate KFE <=3.9A, FFE <=4.7A). Speed abort "
              f"{np.rad2deg(SPEED_ABORT_RAD_S):.0f}deg/s, stale feedback, motor error and origin "
              "aborts are unchanged. Keep the hoist rope attached.")
    if a.gravity_limits:
        print("GRAVITY LIMITS: per-axis current abort raised to " + ", ".join(
            f"0x{mid:02X} {H_BINDING_BY_ID[mid].name}={limits[mid]:.1f}A"
            for mid in H_CAN_IDS
        ))
        print(f"GRAVITY LIMITS: D10-3 measured the suspended static hold at <=0.10A "
              f"per axis, but moving the legs to the policy pose needed up to 2.0A "
              f"(LL_HAA), above the flat {CURRENT_ABORT_A:.1f}A limit. These "
              "values are still far under the trained policy's own effort limit "
              "(AK10-9 42.1A, AK80-9 25.8A). Speed abort, stale feedback, motor "
              "error and origin aborts are unchanged.")
    slew_max = WALK_POLICY_SLEW_MAX_DPS if a.walk_limits else POLICY_SLEW_MAX_DPS
    if a.command_delay_seconds is not None:
        if not a.walk_limits:
            p.error('--command-delay-seconds is for the floor walking run: pass --walk-limits')
        if not 0 < a.command_delay_seconds <= COMMAND_DELAY_MAX_S:
            p.error(f'--command-delay-seconds must satisfy 0 < value <= {COMMAND_DELAY_MAX_S:g}')
        if a.command_delay_seconds >= a.duration:
            p.error('--command-delay-seconds must be shorter than --duration')
    if a.policy_slew_dps is not None:
        if not a.arm or not a.all_axes or a.hold_pose or a.static_probe:
            p.error('--policy-slew-dps is only for a whole-body policy run: --arm --all-axes, '
                    'without --hold-pose or --static-probe')
        if not 0 < a.policy_slew_dps <= slew_max:
            p.error(f'--policy-slew-dps must satisfy 0 < value <= {slew_max:g}'
                    + ('' if a.walk_limits else f' ({WALK_POLICY_SLEW_MAX_DPS:g} only with --walk-limits)'))
    if a.stand_seconds is not None or a.stand_only:
        if not a.all_axes or a.hold_pose or a.static_probe:
            p.error('--stand-seconds/--stand-only are for a whole-body run: --all-axes, '
                    'without --hold-pose or --static-probe')
        stand_max = FLOOR_STAND_MAX_SECONDS if a.floor_limits else STAND_MAX_SECONDS
        if a.stand_seconds is None or not 0 < a.stand_seconds <= stand_max:
            p.error(f'--stand-seconds must satisfy 0 < value <= {stand_max:g}'
                    + ('' if a.floor_limits else
                       f' (up to {FLOOR_STAND_MAX_SECONDS:g} only with --floor-limits)'))
        if a.stand_only and (a.duration or a.policy_slew_dps is not None
                             or a.vx or a.vy or a.wz):
            p.error('--stand-only runs no policy: leave --duration, --policy-slew-dps '
                    'and --vx/--vy/--wz unset')
    if a.sign_pose and not a.stand_only:
        p.error('--sign-pose is a look-only check: it requires --stand-only (no policy stage)')
    if a.haa_close_deg is not None:
        if not a.stand_only or a.sign_pose:
            p.error('--haa-close-deg is a no-policy check: it requires --stand-only and '
                    'cannot be combined with --sign-pose')
        if not 0 < a.haa_close_deg <= HAA_CLOSE_MAX_DEG:
            p.error(f'--haa-close-deg must satisfy 0 < value <= {HAA_CLOSE_MAX_DEG:g}')
        print(f"HAA SWEEP planned: after the stand hold, both HAA inward to "
              f"{-a.haa_close_deg:+.1f}deg at {HAA_SWEEP_DPS:g}deg/s "
              f"({a.haa_close_deg / HAA_SWEEP_DPS:.1f}s), stop and freeze at the first block.")
    haa_close_rad = None if a.haa_close_deg is None else float(np.deg2rad(a.haa_close_deg))
    stand_target = SIGN_POSE_TARGET if a.sign_pose else STAND_TARGET
    auto_lean_rad = None
    if a.auto_lean_deg is not None:
        if not a.floor_limits or a.stand_seconds is None or a.stand_seconds < AUTO_LEAN_START_S + 4:
            p.error(f'--auto-lean-deg needs --floor-limits or --walk-limits and '
                    f'--stand-seconds >= {AUTO_LEAN_START_S + 4:g}')
        if a.lean_sweep_deg is not None or a.stand_lean_deg is not None:
            p.error('--auto-lean-deg replaces --lean-sweep-deg/--stand-lean-deg; pass one')
        if not 0 < a.auto_lean_deg <= STAND_LEAN_MAX_DEG:
            p.error(f'--auto-lean-deg must satisfy 0 < value <= {STAND_LEAN_MAX_DEG:g}')
        auto_lean_rad = float(np.deg2rad(a.auto_lean_deg))
        print(f"AUTO LEAN planned: up to +{a.auto_lean_deg:g}deg on both FFE from "
              f"{AUTO_LEAN_START_S:g}s into the {a.stand_seconds:g}s stand hold.")
    lean_sweep_rad = None
    if a.stand_knee_deg is not None or a.stand_lean_deg is not None or a.lean_sweep_deg is not None:
        if a.stand_seconds is None or a.sign_pose or a.haa_close_deg is not None:
            p.error('--stand-knee-deg/--stand-lean-deg/--lean-sweep-deg shape the stand pose: they need '
                    '--stand-seconds and cannot be combined with --sign-pose or --haa-close-deg')
        if a.stand_knee_deg is not None and not 0 <= a.stand_knee_deg <= STAND_KNEE_MAX_DEG:
            p.error(f'--stand-knee-deg must satisfy 0 <= value <= {STAND_KNEE_MAX_DEG:g}')
        if a.stand_lean_deg is not None and not STAND_LEAN_MIN_DEG <= a.stand_lean_deg <= STAND_LEAN_MAX_DEG:
            p.error(f'--stand-lean-deg must satisfy {STAND_LEAN_MIN_DEG:g} <= value <= {STAND_LEAN_MAX_DEG:g}')
        if a.lean_sweep_deg is not None:
            if not (a.floor_limits and a.stand_only):
                p.error('--lean-sweep-deg is a floor balance check: it needs --floor-limits and --stand-only')
            if not 0 < a.lean_sweep_deg <= LEAN_SWEEP_MAX_DEG:
                p.error(f'--lean-sweep-deg must satisfy 0 < value <= {LEAN_SWEEP_MAX_DEG:g}')
            lean_sweep_rad = float(np.deg2rad(a.lean_sweep_deg))
        stand_target = stand_pose_target(a.stand_knee_deg, a.stand_lean_deg or 0.0)
        print("STAND POSE (D10-11): " + ", ".join(
            f"{H_BINDING_BY_ID[mid].name}={np.rad2deg(t):+.1f}deg"
            for mid, t in zip(H_CAN_IDS, stand_target)
            if joint_suffix(mid) in ("HFE", "KFE", "FFE")))
        if lean_sweep_rad:
            print(f"LEAN SWEEP planned: after the stand hold, both FFE toe-down by up to "
                  f"{a.lean_sweep_deg:g}deg at {LEAN_SWEEP_DPS:g}deg/s "
                  f"({a.lean_sweep_deg / LEAN_SWEEP_DPS:.0f}s), then {LEAN_SWEEP_HOLD_S:g}s hold.")
    if a.sign_pose:
        print(f"SIGN POSE: sim default pose + {SIGN_POSE_EXTRA_DEG:g}deg outward on HR and HAA, "
              "both legs. Watch both legs; they must be mirror images.")
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
    if a.zero_offset:
        if not a.all_axes:
            p.error('--zero-offset is for a whole-body run: pass --all-axes')
        table = set_zero_offset(a.zero_offset)
        print(f"ZERO OFFSET {a.zero_offset}: sim angle = D7-origin angle + offset: "
              + ", ".join(f"{name}={value:+.1f}deg" for name, value in table.items()))
    if a.csv and (a.arm or a.preflight) and not a.analyze:
        fresh = unique_csv_path(a.csv)
        if fresh != Path(a.csv):
            print(f"CSV NAME: {a.csv} is already used; this run writes {fresh} instead.")
        a.csv = fresh
        try:
            meta = meta_txt_path(a.csv)
            meta.parent.mkdir(parents=True, exist_ok=True)
            meta.write_text(f"{time.strftime('%Y-%m-%d %H:%M:%S')} build={BUILD_ID}\n"
                            f"argv={' '.join(sys.argv[1:])}\n"
                            f"zero_offset={a.zero_offset or 'none'} "
                            + " ".join(f"{H_BINDING_BY_ID[mid].name}={np.rad2deg(zero_offset_rad(mid)):+.1f}"
                                       for mid in H_CAN_IDS) + "\n", encoding="utf-8")
        except Exception as error:
            print(f"WARNING: meta record failed: {error}")
    if a.preflight:
        # A path is still supplied so evidence is written consistently, but
        # no policy frame or cleanup frame is put on CAN.
        run(a.package, 0., a.csv or Path('logs/d9_preflight.csv'), a.vx, a.vy, a.wz, transmit=False,
            stand_seconds=a.stand_seconds, stand_target=stand_target)
        return 0
    if a.stand_only:
        if not a.csv: p.error('--csv is required with --arm')
    else:
        duration_max = WALK_DURATION_MAX_S if a.walk_limits else 5.0
        if not a.csv or not 0 < a.duration <= duration_max:
            p.error(f'--csv and 0<--duration<={duration_max:g} are required with --arm'
                    + ('' if a.walk_limits else f' ({WALK_DURATION_MAX_S:g} only with --walk-limits)'))
    if a.ramp_seconds is None or a.ramp_seconds <= 0: p.error('--arm requires a positive --ramp-seconds value')
    if a.all_axes:
        # Every axis at once is the whole-body step.  It is deliberate and
        # explicit: --motor-id must not be given, so nobody reaches ten axes
        # by accident while thinking they selected one.  The current, speed,
        # stale-feedback and origin aborts all stay active and all of them
        # still watch every axis, not just the logged one.
        if a.motor_id:
            p.error('--all-axes drives every registered axis; do not also pass --motor-id')
        if a.policy_slew_dps is None and not a.stand_only:
            p.error('--all-axes with a policy requires --policy-slew-dps (D10-3: without it '
                    'nine axes step to the live policy output in one tick)')
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
        current_limits=limits,
        policy_slew_rad_s=None if a.policy_slew_dps is None else float(np.deg2rad(a.policy_slew_dps)),
        stand_seconds=a.stand_seconds, stand_only=a.stand_only, stand_target=stand_target,
        haa_close_rad=haa_close_rad, lean_sweep_rad=lean_sweep_rad,
        auto_lean_rad=auto_lean_rad,
        speed_abort_rad_s=float(np.deg2rad(walk_speed_dps)) if a.walk_limits else None,
        command_delay_s=a.command_delay_seconds,
        soft_stop_s=a.soft_stop_seconds,
        start_gate_rad=None if a.start_gate_deg is None else float(np.deg2rad(a.start_gate_deg)),
        fine_timer=a.fine_timer)
if __name__=='__main__': sys.exit(main() or 0)

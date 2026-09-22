#!/usr/bin/env python3
"""D8: 10-axis, dual-CAN MIT sender.  Hardware transmission requires --arm."""
from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np

import cubemars as cm
from motor_console_ver8_2 import f_mit, quantized_cmd
from policy_integration import ACTION_SIZE, HPolicy
from ver9_integration import H_CAN_IDS, H_MODELS, evaluate_cycle
from ver9_shell import FixedCommandSource, MotorFeedback, RealT265, T265_R_OFFSET_M, VelocityCommand, servo_feedback_to_h_units

HZ = 50.0
PERIOD = 1.0 / HZ
BUILD_ID = "D9_RAMP_20260922_1745"
STALE_S = 0.30
# gs_usb resets its USB interface when a Bus is started.  The second adapter
# needs this full pause after the first one; otherwise python-can may emit a
# GsUsbBus finalizer warning while its own constructor is unwinding.
OPEN_SETTLE_S = 2.0
OPEN_RETRY_S = 1.5
CURRENT_ABORT_A = 1.0
SPEED_ABORT_RAD_S = np.deg2rad(100.0)
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
# A static probe deliberately stays close to a D7 origin.  Its purpose is to
# separate breakaway/stiction from the policy/T265 path, not to tune a joint.
STATIC_PROBE_MAX_DEG = 2.5
# Seeing one encoder increment is insufficient for a fixed-target probe.  It
# must cover a meaningful portion of the requested relative displacement.
STATIC_PROBE_MIN_TRACKING_FRACTION = 0.50
# The same criterion applies to the frozen first policy target: a one-axis
# policy ramp has not succeeded when it moves only one encoder increment.
POLICY_RAMP_MIN_TRACKING_FRACTION = 0.50
# H deployment stiffness/damping, converted with the measured c_p/c_d.
STIFFNESS = np.array((10,10,15,15,15,15,15,15,10,10), dtype=float)
DAMPING = np.full(10, 1.5, dtype=float)
CP = {"AK80-9": .523, "AK10-9": 1.258}
CD = {"AK80-9": .523, "AK10-9": 1.216}

def gains():
    return tuple((float(STIFFNESS[i]/CP[m]), float(DAMPING[i]/CD[m])) for i,m in enumerate(H_MODELS))

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


def static_probe_summary(rows, initial_position_rad, target_delta_rad):
    """Require a static probe to cover half of its requested displacement."""
    movement, current, verdict = tracking_summary(rows, initial_position_rad)
    required = max(MIN_TRACKING_RAD, abs(target_delta_rad) * STATIC_PROBE_MIN_TRACKING_FRACTION)
    if movement < required:
        if current >= STALL_CURRENT_A:
            verdict = "position stalled before target (current present; static friction/mechanical load suspected)"
        else:
            verdict = "no meaningful position response before target (MIT torque response unproven)"
    return movement, current, required, verdict


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


class DualBus:
    """Both gs_usb channels plus a per-process route learned from feedback."""
    def __init__(self):
        self.bus_by_channel = {
            channel: cm.MotorBus(channel=channel) for channel in (0, 1)
        }
        self.route_by_motor_id = {}
        # This is deliberately separate from a route: a failed receive-only
        # preflight must close the adapter without injecting MIT frames.
        self.mit_frames_sent = False
        self.mit_motor_ids = set()

    def open(self):
        for attempt in (1, 2):
            try:
                for channel in (0, 1):
                    self.bus_by_channel[channel].open()
                    time.sleep(OPEN_SETTLE_S)
                return
            except Exception:
                # An open/reset failure has not armed MIT and must not emit a
                # CAN frame.  Release both interfaces before the one retry.
                self.close()
                if attempt == 2:
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
            if abs(s.cur)>CURRENT_ABORT_A or abs(v)>SPEED_ABORT_RAD_S:
                raise RuntimeError(
                    f"motion/current abort 0x{mid:02X} ch={channel}: "
                    f"cur={s.cur:+.2f}A (limit ±{CURRENT_ABORT_A:.2f}A), "
                    f"speed={np.rad2deg(v):+.1f}deg/s (limit ±{np.rad2deg(SPEED_ABORT_RAD_S):.1f}deg/s), "
                    f"pos={np.rad2deg(p):+.1f}deg"
                )
            out[mid]=MotorFeedback(mid,now,p,v)
        return out
    def zero(self):
        if not self.route_by_motor_id or not self.mit_frames_sent:
            return
        for _ in range(3):
            for fr in zero_frames(tuple(sorted(self.mit_motor_ids))):
                self.send(fr)
            time.sleep(.01)

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
        max_tracking_rad, max_current_a, required_tracking_rad, verdict = static_probe_summary(
            rows, start[selected_index], np.deg2rad(target_delta_deg)
        )
        print(
            f"STATIC PROBE: 0x{motor_id:02X} max feedback movement="
            f"{np.rad2deg(max_tracking_rad):.3f}deg "
            f"(required >= {np.rad2deg(required_tracking_rad):.3f}deg); "
            f"max |current|={max_current_a:.2f}A; {verdict}"
        )
        if max_tracking_rad < required_tracking_rad:
            raise RuntimeError(
                f"static probe abort 0x{motor_id:02X}: feedback moved only "
                f"{np.rad2deg(max_tracking_rad):.3f}deg; required >= "
                f"{np.rad2deg(required_tracking_rad):.3f}deg; "
                f"max |current|={max_current_a:.2f}A; {verdict}."
            )
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

def run(package,duration,csv_path,vx,vy,wz,transmit=True,ramp_seconds=None,motor_ids=H_CAN_IDS):
    policy=HPolicy(package); bus=DualBus(); t265=RealT265(T265_R_OFFSET_M); cmd=FixedCommandSource(vx,vy,wz); last=np.zeros(10,np.float32)
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
            state = bus.state(selected_id)
            feedback_pos, feedback_vel = servo_feedback_to_h_units(state.pos, state.spd)
            _kp, _kd, wire_target, _vel, _tau = wire_command(selected_id, ramped[selected_index])
            rows.append((tick, "ramp", f"0x{selected_id:02X}",
                         initial_targets[selected_index], ramped[selected_index], wire_target,
                         feedback_pos, feedback_vel, state.cur))
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
            state = bus.state(selected_id)
            feedback_pos, feedback_vel = servo_feedback_to_h_units(state.pos, state.spd)
            _kp, _kd, wire_target, _vel, _tau = wire_command(selected_id, commanded_target)
            rows.append((tick, "policy", f"0x{selected_id:02X}", desired_target,
                         commanded_target, wire_target, feedback_pos, feedback_vel, state.cur))
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
        max_tracking_rad, max_current_a, required_tracking_rad, tracking_verdict = policy_ramp_summary(
            rows, initial_positions[selected_index], initial_targets[selected_index]
        )
        print(
            f"TRACKING: 0x{selected_id:02X} max feedback movement="
            f"{np.rad2deg(max_tracking_rad):.3f}deg "
            f"(required >= {np.rad2deg(required_tracking_rad):.3f}deg); "
            f"max |current|={max_current_a:.2f}A; {tracking_verdict}"
        )
        if max_tracking_rad < required_tracking_rad:
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

def main():
    p=argparse.ArgumentParser(description='D8 10-axis MIT sender; --arm is required for any CAN transmit.')
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--arm',action='store_true')
    mode.add_argument('--preflight',action='store_true', help='open/receive/evaluate once and print initial targets; sends zero CAN frames')
    p.add_argument('--static-probe', action='store_true', help='with --arm: one-axis fixed relative target; no policy/T265')
    p.add_argument('--probe-target-deg', type=float, help=f'fixed relative target for --static-probe; abs <= {STATIC_PROBE_MAX_DEG:g} deg')
    p.add_argument('--preview',action='store_true'); p.add_argument('--package',type=Path); p.add_argument('--duration',type=float,default=0.); p.add_argument('--csv',type=Path); p.add_argument('--vx',type=float,default=0.); p.add_argument('--vy',type=float,default=0.); p.add_argument('--wz',type=float,default=0.); p.add_argument('--ramp-seconds',type=float, help='required with --arm; initial policy target is reached linearly over this time'); p.add_argument('--motor-id', type=lambda value: int(value, 0), action='append', help='required once with --arm; only this registered motor receives MIT frames')
    a=p.parse_args()
    current_mode = 'static-probe' if a.static_probe else 'arm' if a.arm else 'preflight' if a.preflight else 'none'
    print(f"ver9_d8_sender build={BUILD_ID}; mode={current_mode}")
    if a.preview: preview(); return 0
    if not (a.arm or a.preflight): p.error('--arm is required for transmission; use --preflight for a receive-only live check')
    if a.static_probe:
        if not a.arm or a.preflight: p.error('--static-probe requires --arm and cannot be combined with --preflight')
        if not a.csv or not 0 < a.duration <= 5: p.error('--static-probe requires --csv and 0<--duration<=5')
        if a.ramp_seconds is None or a.ramp_seconds <= 0: p.error('--static-probe requires a positive --ramp-seconds')
        if not a.motor_id or len(a.motor_id) != 1 or a.motor_id[0] not in H_CAN_IDS:
            p.error('--static-probe requires exactly one registered --motor-id')
        if a.probe_target_deg is None or not 0 < abs(a.probe_target_deg) <= STATIC_PROBE_MAX_DEG:
            p.error(f'--static-probe requires 0<abs(--probe-target-deg)<={STATIC_PROBE_MAX_DEG:g}')
        run_static_probe(a.csv, a.motor_id[0], a.probe_target_deg, a.ramp_seconds, a.duration)
        return
    if not a.package: p.error('--package is required')
    if a.preflight:
        # A path is still supplied so evidence is written consistently, but
        # no policy frame or cleanup frame is put on CAN.
        run(a.package, 0., a.csv or Path('logs/d9_preflight.csv'), a.vx, a.vy, a.wz, transmit=False)
        return 0
    if not a.csv or not 0<a.duration<=5: p.error('--csv and 0<--duration<=5 are required with --arm')
    if a.ramp_seconds is None or a.ramp_seconds <= 0: p.error('--arm requires a positive --ramp-seconds value')
    if not a.motor_id or len(a.motor_id) != 1 or a.motor_id[0] not in H_CAN_IDS:
        p.error('--arm requires exactly one registered --motor-id (for example, 0x1C)')
    run(a.package, a.duration, a.csv, a.vx, a.vy, a.wz,
        transmit=True, ramp_seconds=a.ramp_seconds, motor_ids=tuple(a.motor_id))
if __name__=='__main__': main()

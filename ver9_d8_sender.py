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
from ver9_integration import H_CAN_IDS, H_MODELS, LEFT_CAN_IDS, RIGHT_CAN_IDS, evaluate_cycle
from ver9_shell import FixedCommandSource, MotorFeedback, RealT265, T265_R_OFFSET_M, VelocityCommand, servo_feedback_to_h_units

HZ = 50.0
PERIOD = 1.0 / HZ
STALE_S = 0.30
CURRENT_ABORT_A = 1.0
SPEED_ABORT_RAD_S = np.deg2rad(100.0)
# H deployment stiffness/damping, converted with the measured c_p/c_d.
STIFFNESS = np.array((10,10,15,15,15,15,15,15,10,10), dtype=float)
DAMPING = np.full(10, 1.5, dtype=float)
CP = {"AK80-9": .523, "AK10-9": 1.258}
CD = {"AK80-9": .523, "AK10-9": 1.216}

def gains():
    return tuple((float(STIFFNESS[i]/CP[m]), float(DAMPING[i]/CD[m])) for i,m in enumerate(H_MODELS))

def frames(targets):
    if len(targets) != ACTION_SIZE: raise ValueError("need ten targets")
    return tuple(f_mit(mid, *gains()[i], float(targets[i]), 0., 0., model)
                 for i,(mid,model) in enumerate(zip(H_CAN_IDS,H_MODELS)))

def zero_frames():
    return tuple(f_mit(mid, 0., 0., 0., 0., 0., model) for mid,model in zip(H_CAN_IDS,H_MODELS))

class DualBus:
    def __init__(self): self.left=cm.MotorBus(channel=0); self.right=cm.MotorBus(channel=1)
    def open(self): self.left.open(); self.right.open()
    def close(self): self.left.close(stop_motors=False); self.right.close(stop_motors=False)
    def state(self, mid): return (self.left if mid in LEFT_CAN_IDS else self.right).state(mid)
    def send(self, frame): (self.left if (frame.arbitration_id & 255) in LEFT_CAN_IDS else self.right).send(frame)
    def feedback(self):
        now=time.monotonic(); out={}
        for mid in H_CAN_IDS:
            s=self.state(mid)
            if s is None or time.time()-s.t > STALE_S: raise RuntimeError(f"stale feedback 0x{mid:02X}")
            if s.err: raise RuntimeError(f"motor error 0x{mid:02X}: {s.err}")
            p,v=servo_feedback_to_h_units(s.pos,s.spd)
            if abs(s.cur)>CURRENT_ABORT_A or abs(v)>SPEED_ABORT_RAD_S: raise RuntimeError(f"motion/current abort 0x{mid:02X}")
            out[mid]=MotorFeedback(mid,now,p,v)
        return out
    def zero(self):
        for _ in range(3):
            for fr in zero_frames(): self.send(fr)
            time.sleep(.01)

def preview():
    for i,(mid,model) in enumerate(zip(H_CAN_IDS,H_MODELS)):
        cmd=(gains()[i][0],gains()[i][1],0.,0.,0.)
        got=quantized_cmd(cmd,model)
        print(f"0x{mid:02X} {model} Kp={got[0]:.3f} Kd={got[1]:.3f} data={f_mit(mid,*cmd,model).data.hex()}")

def run(package,duration,csv_path,vx,vy,wz):
    policy=HPolicy(package); bus=DualBus(); t265=RealT265(T265_R_OFFSET_M); cmd=FixedCommandSource(vx,vy,wz); last=np.zeros(10,np.float32)
    rows=[]; bus.open(); t265.start(); deadline=time.monotonic()+5
    try:
        while t265.latest() is None:
            if time.monotonic()>deadline: raise RuntimeError("T265 warmup timeout")
            time.sleep(.01)
        end=time.monotonic()+duration; nxt=time.monotonic()
        while time.monotonic()<end:
            time.sleep(max(0,nxt-time.monotonic())); tick=time.monotonic()
            _s,_o,out,plan=evaluate_cycle(policy,t265.latest(),bus.feedback(),cmd.sample(tick),last)
            for fr in frames(out.joint_target_h_order): bus.send(fr)
            rows.append((tick,*out.joint_target_h_order)); last=out.action_raw; nxt+=PERIOD
    finally:
        bus.zero(); t265.close(); bus.close()
    csv_path.parent.mkdir(parents=True,exist_ok=True)
    with csv_path.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(('tick',*H_CAN_IDS)); w.writerows(rows)

def main():
    p=argparse.ArgumentParser(description='D8 10-axis MIT sender; --arm is required for any CAN transmit.')
    p.add_argument('--preview',action='store_true'); p.add_argument('--arm',action='store_true'); p.add_argument('--package',type=Path); p.add_argument('--duration',type=float,default=0.); p.add_argument('--csv',type=Path); p.add_argument('--vx',type=float,default=0.); p.add_argument('--vy',type=float,default=0.); p.add_argument('--wz',type=float,default=0.)
    a=p.parse_args()
    if a.preview: preview(); return 0
    if not a.arm: p.error('--arm is required; use --preview for a no-hardware check')
    if not a.package or not a.csv or not 0<a.duration<=5: p.error('--package, --csv and 0<--duration<=5 are required')
    run(a.package,a.duration,a.csv,a.vx,a.vy,a.wz)
if __name__=='__main__': main()

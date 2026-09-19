"""二台（0x22/0x12）を同一50 Hz周期でMIT保持する、ver8派生の安全確認用ツール。

用途は「二台が同時に受信でき、50 Hz周期と停止処理が成立するか」の段階的な確認です。
ID 0x22 (34, AK80-9) と 0x12 (18, AK10-9) に、同じ周期内で別々に量子化した
MITフレームを送ります。CAN上の二フレームは物理的には直列なので完全同時ではなく、
各周期の送信時刻差（skew）を記録します。

最初は必ず --dry-run でフレームと上限を確認すること。実機送信には --live と
--confirm-safe-setup の両方が必要です。非常停止は電源OFFです。

例（両軸を原点姿勢で o 0 済み、まず保持だけを2秒）：
  py -3.13 motor_console_ver8_2.py --live --confirm-safe-setup \
    --pos22 0 --pos12 0 --seconds 2
"""

import argparse
import csv
import os
import sys
import time

import cubemars as cm
from motor_console_ver8 import f_mit


CHANNEL = 1
BITRATE = 1_000_000
SEND_HZ = 50
MOTORS = (
    {"key": "22", "id": 0x22, "model": "AK80-9"},
    {"key": "12", "id": 0x12, "model": "AK10-9"},
)

# この試験は「二台通信」の確認であり、関節を動かすためのゲイン探索ではない。
MAX_SECONDS = 3.0
MAX_ABS_TARGET_RAD = 0.20       # o 0 を打った基準姿勢から約11.5 degまで
MAX_KP = 1.0
MAX_KD = 0.50
MAX_CURRENT_A = 1.0
MAX_SPEED_DEG_S = 100.0
MAX_MOVE_DEG = 8.0
MAX_AGE = 0.30
STATIC_ERPM = 50.0
ZERO_CYCLES = 10


class SafetyAbort(RuntimeError):
    pass


def make_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--live", action="store_true", help="CANへ実際に送信する（単独では不足）")
    p.add_argument("--confirm-safe-setup", action="store_true",
                   help="吊り・可動域・電源OFF非常停止を確認済みであることを明示")
    p.add_argument("--seconds", type=float, default=2.0, help="送信時間。最大3秒")
    p.add_argument("--pos22", type=float, default=None, help="ID 0x22 のMIT目標位置 [rad]")
    p.add_argument("--pos12", type=float, default=None, help="ID 0x12 のMIT目標位置 [rad]")
    p.add_argument("--kp22", type=float, default=1.0, help="ID 0x22 の指令Kp。最大1.0")
    p.add_argument("--kp12", type=float, default=1.0, help="ID 0x12 の指令Kp。最大1.0")
    p.add_argument("--kd22", type=float, default=0.50, help="ID 0x22 の指令Kd。最大0.50")
    p.add_argument("--kd12", type=float, default=0.50, help="ID 0x12 の指令Kd。最大0.50")
    p.add_argument("--dry-run", action="store_true", help="CANへ接続せず送るフレームだけ表示")
    return p


def command_for(motor, args):
    key = motor["key"]
    pos = getattr(args, f"pos{key}")
    kp = getattr(args, f"kp{key}")
    kd = getattr(args, f"kd{key}")
    return kp, kd, pos, 0.0, 0.0


def validate(args):
    if not 0 < args.seconds <= MAX_SECONDS:
        raise ValueError(f"--seconds は 0 より大きく {MAX_SECONDS:g} 以下にしてください")
    for motor in MOTORS:
        key = motor["key"]
        pos = getattr(args, f"pos{key}")
        kp = getattr(args, f"kp{key}")
        kd = getattr(args, f"kd{key}")
        if pos is None:
            raise ValueError(f"--pos{key} が必要です。原点確認済みのMIT目標位置を明示してください")
        if abs(pos) > MAX_ABS_TARGET_RAD:
            raise ValueError(f"--pos{key} は ±{MAX_ABS_TARGET_RAD:g} rad の範囲だけ許可しています")
        if not 0 <= kp <= MAX_KP:
            raise ValueError(f"--kp{key} は 0..{MAX_KP:g} の範囲だけ許可しています")
        if kp > 0 and not 0 < kd <= MAX_KD:
            raise ValueError(f"--kp{key} > 0 では --kd{key} を 0より大きく {MAX_KD:g} 以下にしてください")
        if kp == 0 and kd != 0:
            raise ValueError(f"--kp{key}=0 のとき --kd{key}=0 にしてください")
    if args.live and not args.confirm_safe_setup:
        raise ValueError("実機送信には --confirm-safe-setup も必要です")


def print_plan(args):
    mode = "実機CAN送信" if args.live else "dry-run（CAN未接続）"
    print(f"{mode}: {SEND_HZ} Hz / {args.seconds:.1f} s / 送信は 0x822 → 0x812")
    for motor in MOTORS:
        cmd = command_for(motor, args)
        frame = f_mit(motor["id"], *cmd, motor["model"])
        print(f"  ID=0x{motor['id']:02X} {motor['model']:7s}  Kp={cmd[0]:.3f} Kd={cmd[1]:.3f} "
              f"pos={cmd[2]:+.4f} rad  frame={frame.data.hex(' ')}")
    print(f"  中断: age>{MAX_AGE:.2f}s / |I|>{MAX_CURRENT_A:.1f}A / "
          f"|speed|>{MAX_SPEED_DEG_S:.0f}deg/s / move>{MAX_MOVE_DEG:.0f}deg / err!=0")


def wait_for_feedback(bus):
    deadline = time.time() + 5.0
    while time.time() < deadline:
        states = {m["id"]: bus.state(m["id"]) for m in MOTORS}
        if all(s is not None and s.alive(MAX_AGE) for s in states.values()):
            if all(abs(s.spd) <= STATIC_ERPM and s.err == 0 for s in states.values()):
                return states
        time.sleep(0.02)
    detail = ", ".join(
        f"0x{m['id']:02X}={'none' if bus.state(m['id']) is None else 'not-ready'}" for m in MOTORS)
    raise SafetyAbort(f"フィードバック待機失敗: {detail}。送信しません")


def check_states(bus, start_pos):
    rows = []
    for motor in MOTORS:
        mid = motor["id"]
        s = bus.state(mid)
        if s is None or not s.alive(MAX_AGE):
            raise SafetyAbort(f"ID=0x{mid:02X} のフィードバックが {MAX_AGE:.2f}s 以上途絶")
        if s.err:
            raise SafetyAbort(f"ID=0x{mid:02X} err={s.err}")
        if abs(s.cur) > MAX_CURRENT_A:
            raise SafetyAbort(f"ID=0x{mid:02X} |I|={abs(s.cur):.2f}A")
        speed_deg_s = s.spd / 31.5
        if abs(speed_deg_s) > MAX_SPEED_DEG_S:
            raise SafetyAbort(f"ID=0x{mid:02X} speed={speed_deg_s:.1f}deg/s")
        if abs(s.pos - start_pos[mid]) > MAX_MOVE_DEG:
            raise SafetyAbort(f"ID=0x{mid:02X} move={s.pos - start_pos[mid]:+.1f}deg")
        rows.append((mid, s.pos, speed_deg_s, abs(s.cur), s.temp, s.err, s.age()))
    return rows


def send_zero(bus):
    for _ in range(ZERO_CYCLES):
        for motor in MOTORS:
            bus.send(f_mit(motor["id"], 0.0, 0.0, 0.0, 0.0, 0.0, motor["model"]), quiet=True)
        time.sleep(1.0 / SEND_HZ)


def run_live(args):
    bus = cm.MotorBus(channel=CHANNEL, bitrate=BITRATE, model="AK80-9")
    log_dir = os.path.join("logs", "dual_v8_2_" + time.strftime("%Y%m%d"))
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(log_dir, "dual_" + time.strftime("%H%M%S") + ".csv")
    try:
        bus.open()
        time.sleep(0.5)
        initial = wait_for_feedback(bus)
        start_pos = {mid: s.pos for mid, s in initial.items()}
        print("フィードバック確認済み: " + ", ".join(f"0x{mid:02X}={pos:+.1f}deg" for mid, pos in start_pos.items()))
        with open(path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["elapsed_s", "tick", "skew_ms", "id", "pos_deg", "speed_deg_s", "abs_current_a", "temp_c", "err", "age_ms"])
            period, t0, next_send, tick = 1.0 / SEND_HZ, time.perf_counter(), time.perf_counter(), 0
            max_skew = 0.0
            while time.perf_counter() - t0 < args.seconds:
                now = time.perf_counter()
                if now < next_send:
                    time.sleep(min(0.002, next_send - now))
                    continue
                tx_times = []
                for motor in MOTORS:
                    tx_times.append(time.perf_counter())
                    bus.send(f_mit(motor["id"], *command_for(motor, args), motor["model"]), quiet=True)
                skew = (tx_times[1] - tx_times[0]) * 1000.0
                max_skew = max(max_skew, skew)
                for row in check_states(bus, start_pos):
                    writer.writerow([f"{time.perf_counter()-t0:.6f}", tick, f"{skew:.3f}", *row])
                tick += 1
                next_send = t0 + tick * period
            print(f"完了: {tick} 周期、最大送信skew={max_skew:.3f} ms、記録={path}")
    finally:
        try:
            send_zero(bus)
            print(f"零MIT指令を両方へ {ZERO_CYCLES} 周期送信しました")
        finally:
            bus.close(stop_motors=False)


def main():
    args = make_parser().parse_args()
    try:
        validate(args)
        print_plan(args)
        if args.live:
            run_live(args)
    except (ValueError, SafetyAbort) as exc:
        print(f"中止: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\nCtrl+C: finallyで零MIT指令を送ります", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

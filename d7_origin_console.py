"""D7専用: ch=0 / ch=1 の全10軸を受信確認し、原点だけを設定する。

このプログラムは位置・速度・トルク・MIT指令を送らない。原点設定を確定した
ときだけ、CubeMars サーボモード5 (`SET_ORIGIN`, kind=0) を1フレーム送る。
終了処理も送信しない。非常停止は主電源OFFである。
"""

from __future__ import annotations

import pathlib
import sys
import threading
import time
from typing import Dict, Optional

import cubemars as cm
from robot_joint_map import BY_ID, JOINTS, JointBinding


BITRATE = 1_000_000
FRESH_S = 0.3
OPEN_SETTLE_S = 0.75


Joint = JointBinding


def parse_hex_motor_id(text: str) -> Optional[int]:
    """`0x13` 形式だけを受け入れ、10進数の取り違えを防ぐ。"""
    if not text.lower().startswith("0x"):
        return None
    try:
        value = int(text, 16)
    except ValueError:
        return None
    return value if value in BY_ID else None


class D7Console:
    def __init__(self) -> None:
        self.buses: Dict[int, cm.MotorBus] = {
            channel: cm.MotorBus(channel=channel, bitrate=BITRATE)
            for channel in (0, 1)
        }
        log_dir = pathlib.Path("logs") / "d7_origin"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f"session_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        self._record("D7 dual-channel origin session started; no origin command sent yet.")

    def _record(self, text: str) -> None:
        with self.log_path.open("a", encoding="utf-8") as log:
            log.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {text}\n")

    def start(self) -> None:
        for channel, bus in self.buses.items():
            try:
                bus.open()
            except Exception as error:
                self._record(f"open FAILED ch={channel}: {type(error).__name__}: {error}")
                raise
            print(f"接続: ch={channel} {BITRATE} bps（受信開始、まだ送信なし）")
            self._record(f"open OK ch={channel}; USB reset settle={OPEN_SETTLE_S:.2f}s")
            # gs_usb resets its USB interface at Bus() construction.  Let each
            # interface settle before opening the next one; this sends no CAN.
            time.sleep(OPEN_SETTLE_S)
        print(f"記録: {self.log_path}")

    def close(self) -> None:
        # stop_motors=False は重要: q/例外でもCANフレームを送らない。
        for bus in self.buses.values():
            bus.close(stop_motors=False)

    def state_line(self, joint: Joint) -> tuple[bool, str]:
        state = self.buses[joint.channel].state(joint.motor_id)
        prefix = f"{joint.name:6s} 0x{joint.motor_id:02X} ch={joint.channel} {joint.model:7s}"
        if state is None:
            return False, f"{prefix}  状態なし"
        age = state.age()
        fresh = age <= FRESH_S
        ok = fresh and state.err == 0 and state.src == "servo"
        return ok, (f"{prefix}  pos={state.pos:+.1f}deg temp={state.temp}C "
                    f"err={state.err} age={age:.3f}s src={state.src}"
                    + ("" if ok else "  <-- 原点設定不可"))

    def show(self, motor_id: Optional[int] = None) -> None:
        joints = (BY_ID[motor_id],) if motor_id is not None else JOINTS
        for joint in joints:
            _ok, line = self.state_line(joint)
            print(line)

    def scan(self, seconds: float) -> None:
        if not 0 < seconds <= 10:
            print("秒数は 0 より大きく10以下にしてください。")
            return
        print(f"{seconds:g}秒間、ch=0/ch=1を同時に受信します（送信しません）。")
        rows: Dict[int, list] = {}
        threads = [
            threading.Thread(
                target=lambda ch=channel: rows.setdefault(ch, self.buses[ch].feedback_hz(seconds=seconds))
            )
            for channel in (0, 1)
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        found = set()
        for channel in (0, 1):
            print(f"ch={channel}:")
            for _arb, motor_id, count, hz, _dlc, _ext, _last in rows.get(channel, []):
                found.add(motor_id)
                joint = BY_ID.get(motor_id)
                label = joint.name if joint else "未登録ID"
                ok, detail = self.state_line(joint) if joint else (False, "")
                print(f"  0x{motor_id:02X} {label:7s} {count:3d}件 {hz:5.1f}Hz  {detail.split('  ', 1)[-1] if detail else ''}")
                if not ok:
                    print("    <-- D7を続けない")
            self._record("scan ch=" + str(channel) + " found=" + ",".join(
                f"0x{motor_id:02X}" for _arb, motor_id, *_rest in rows.get(channel, [])
            ))
        missing = sorted(set(BY_ID) - found)
        unexpected = sorted(found - set(BY_ID))
        if missing:
            print("不足:", ", ".join(f"0x{motor_id:02X}" for motor_id in missing))
        if unexpected:
            print("未登録:", ", ".join(f"0x{motor_id:02X}" for motor_id in unexpected))
        if not missing and not unexpected:
            print("OK: 登録済み10軸が全て見えています。")
        self._record("scan found=" + ",".join(f"0x{motor_id:02X}" for motor_id in sorted(found)))

    def request_origin(self, motor_id: int) -> None:
        joint = BY_ID[motor_id]
        ok, line = self.state_line(joint)
        print(line)
        if not ok:
            print("中止: 新鮮なサーボ形式フィードバックと err=0 が必要です。")
            return
        confirmation = input(
            f"原点設定を送る対象は {joint.name} / 0x{joint.motor_id:02X} / ch={joint.channel} です。"
            f"確定するなら YES 0x{joint.motor_id:02X} と入力: "
        ).strip()
        if confirmation != f"YES 0x{joint.motor_id:02X}":
            print("送信しませんでした。")
            return
        # SET_ORIGIN kind=0: 現在角を一時原点にする。軸を動かす命令ではない。
        sent = self.buses[joint.channel].send(cm.f_set_origin(joint.motor_id, 0))
        if not sent:
            print("送信失敗。主電源をOFFにして確認してください。")
            self._record(f"origin FAILED {joint.name} 0x{joint.motor_id:02X} ch={joint.channel}")
            return
        time.sleep(0.35)
        _ok, after = self.state_line(joint)
        print("原点設定を送信しました（軸は動かない）。")
        print(after)
        self._record(f"origin sent {joint.name} 0x{joint.motor_id:02X} ch={joint.channel}; {after}")

    def loop(self) -> None:
        print("入力: scan 3 / s / s 0x13 / o 0x13 / q")
        print("IDは必ず16進数（例: 0x13）。qは送信せずに終了します。")
        while True:
            try:
                parts = input("[D7 ch0+ch1]> ").strip().split()
            except (EOFError, KeyboardInterrupt):
                print("\n終了します（送信なし）。")
                return
            if not parts:
                continue
            command = parts[0].lower()
            if command in {"q", "quit", "exit"}:
                return
            if command == "scan":
                try:
                    self.scan(float(parts[1]) if len(parts) == 2 else 3.0)
                except ValueError:
                    print("使い方: scan 3")
                continue
            if command == "s":
                if len(parts) == 1:
                    self.show()
                elif len(parts) == 2 and (motor_id := parse_hex_motor_id(parts[1])) is not None:
                    self.show(motor_id)
                else:
                    print("使い方: s または s 0x13")
                continue
            if command == "o" and len(parts) == 2:
                motor_id = parse_hex_motor_id(parts[1])
                if motor_id is None:
                    print("使い方: o 0x13（登録済みの16進数IDだけ指定できます）")
                else:
                    self.request_origin(motor_id)
                continue
            print("使い方: scan 3 / s / s 0x13 / o 0x13 / q")


def main() -> int:
    console = D7Console()
    try:
        console.start()
        console.loop()
    except Exception as error:
        print(f"エラー: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    finally:
        console.close()
        print("ch=0/ch=1を閉じました（終了時の送信なし）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

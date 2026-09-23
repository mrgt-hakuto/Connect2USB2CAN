"""D7専用: ch=0 / ch=1 の全10軸を受信確認し、原点だけを設定する。

このプログラムは位置・速度・トルク・MIT指令を送らない。原点設定を確定した
ときだけ、CubeMars サーボモード5（通常はkind=0）を1フレーム送る。
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
ZERO_WARN_DEG = 45.0
# Each gs_usb Bus() start may reset its USB interface.  With two adapters the
# second start needs a little longer than the old 0.75 s pause on this PC;
# otherwise python-can can abandon a half-initialised GsUsbBus and print its
# misleading "not properly shut down" warning before our no-send retry.
OPEN_SETTLE_S = 2.0
# 2026-09-23: libusb0's device reset fails on the FIRST open on this PC almost
# every time ("could not reset device, win error 31"), so attempt 1 was being
# spent as a matter of course and attempt 2 was the only one left.  That is no
# margin at all: one extra flaky enumeration and D7 does not start.  Three
# attempts keeps the same behaviour and restores a spare.  This path still
# transmits nothing: a failed open closes both interfaces before retrying.
OPEN_ATTEMPTS = 3
OPEN_RETRY_S = 1.5


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
        # gs_usb may swap the two Python channel indices after reset.  This
        # is populated only by a complete, current scan; never guessed.
        self.route_by_motor_id: Dict[int, int] = {}
        log_dir = pathlib.Path("logs") / "d7_origin"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f"session_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        self._record("D7 dual-channel origin session started; no origin command sent yet.")

    def _record(self, text: str) -> None:
        with self.log_path.open("a", encoding="utf-8") as log:
            log.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {text}\n")

    def start(self) -> None:
        for attempt in range(1, OPEN_ATTEMPTS + 1):
            try:
                for channel, bus in self.buses.items():
                    bus.open()
                    print(f"接続: ch={channel} {BITRATE} bps（受信開始、まだ送信なし）")
                    self._record(
                        f"open OK attempt={attempt} ch={channel}; "
                        f"USB reset settle={OPEN_SETTLE_S:.2f}s"
                    )
                    # gs_usb resets its USB interface at Bus() construction.
                    # Let each interface settle before opening the next one;
                    # this path never transmits CAN.
                    time.sleep(OPEN_SETTLE_S)
                return
            except Exception as error:
                self._record(
                    f"open FAILED attempt={attempt}: {type(error).__name__}: {error}"
                )
                for bus in self.buses.values():
                    try:
                        bus.close(stop_motors=False)
                    except Exception as close_error:
                        self._record(f"close after failed open: {type(close_error).__name__}: {close_error}")
                if attempt == OPEN_ATTEMPTS:
                    raise
                print(f"USB open失敗。送信せず{OPEN_RETRY_S:.1f}秒待って1回だけ再試行します。")
                self._record(f"retrying open after {OPEN_RETRY_S:.1f}s (no CAN sent)")
                time.sleep(OPEN_RETRY_S)
        print(f"記録: {self.log_path}")

    def close(self) -> None:
        # stop_motors=False は重要: q/例外でもCANフレームを送らない。
        for bus in self.buses.values():
            try:
                bus.close(stop_motors=False)
            except Exception as error:
                self._record(f"close failed: {type(error).__name__}: {error}")

    def state_line(self, joint: Joint) -> tuple[bool, str]:
        channel = self.route_by_motor_id.get(joint.motor_id)
        if channel is None:
            return False, f"{joint.name:6s} 0x{joint.motor_id:02X} {joint.model:7s}  経路未確定（scan 3 が必要）"
        state = self.buses[channel].state(joint.motor_id)
        prefix = f"{joint.name:6s} 0x{joint.motor_id:02X} ch={channel} {joint.model:7s}"
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
        route: Dict[int, int] = {}
        duplicate_ids = set()
        unexpected = set()
        for channel in (0, 1):
            for _arb, motor_id, *_rest in rows.get(channel, []):
                if motor_id not in BY_ID:
                    unexpected.add(motor_id)
                elif motor_id in route:
                    duplicate_ids.add(motor_id)
                else:
                    route[motor_id] = channel
        missing = sorted(set(BY_ID) - set(route))
        if missing or unexpected or duplicate_ids:
            self.route_by_motor_id.clear()
        else:
            self.route_by_motor_id = route
        for channel in (0, 1):
            print(f"ch={channel}:")
            for _arb, motor_id, count, hz, _dlc, _ext, _last in rows.get(channel, []):
                found.add(motor_id)
                joint = BY_ID.get(motor_id)
                label = joint.name if joint else "未登録ID"
                ok, detail = self.state_line(joint) if joint else (False, "")
                h_text = ""
                if joint:
                    h_state = self.buses[channel].state(motor_id)
                    if h_state is not None:
                        h_text = f"  H={joint.sign * h_state.pos:+.1f}deg"
                print(f"  0x{motor_id:02X} {label:7s} {count:3d}件 {hz:5.1f}Hz  {detail.split('  ', 1)[-1] if detail else ''}{h_text}")
                if not ok:
                    print("    <-- D7を続けない")
            self._record("scan ch=" + str(channel) + " found=" + ",".join(
                f"0x{motor_id:02X}" for _arb, motor_id, *_rest in rows.get(channel, [])
            ))
        if missing:
            print("不足:", ", ".join(f"0x{motor_id:02X}" for motor_id in missing))
        if unexpected:
            print("未登録:", ", ".join(f"0x{motor_id:02X}" for motor_id in unexpected))
        if duplicate_ids:
            print("重複受信:", ", ".join(f"0x{motor_id:02X}" for motor_id in sorted(duplicate_ids)))
        if not missing and not unexpected and not duplicate_ids:
            layout = "; ".join(
                "ch=" + str(channel) + "=" + ",".join(
                    f"0x{motor_id:02X}" for motor_id in sorted(
                        motor_id for motor_id, routed_channel in route.items() if routed_channel == channel
                    )
                ) for channel in (0, 1)
            )
            print("OK: 登録済み10軸を受信し、このプロセスの送信経路を確定しました。")
            self._record("route confirmed " + layout)
            # D10-7 (2026-09-23): D10-6 lost five of ten hand-moved readings
            # because only |pos| > 45 deg was written.  Every axis is now
            # recorded, in the motor's own frame and in the H (sim) frame.
            positions = []
            for joint in JOINTS:
                state = self.buses[route[joint.motor_id]].state(joint.motor_id)
                if state is not None:
                    positions.append(f"0x{joint.motor_id:02X} {joint.name} motor={state.pos:+.1f}deg "
                                     f"H={joint.sign * state.pos:+.1f}deg")
            self._record("scan pos " + "; ".join(positions))
            off_zero = []
            for joint in JOINTS:
                state = self.buses[route[joint.motor_id]].state(joint.motor_id)
                if state is not None and abs(state.pos) > ZERO_WARN_DEG:
                    off_zero.append(f"0x{joint.motor_id:02X}={state.pos:+.1f}deg")
            if off_zero:
                warning = ", ".join(off_zero)
                print("警告: 通信OKは原点OKを意味しません。基準姿勢から外れた座標: " + warning)
                self._record("origin WARNING " + warning)
        else:
            print("不合格: 10軸を一意に受信できません。原点設定・MIT送信へ進まないでください。")
        self._record("scan found=" + ",".join(f"0x{motor_id:02X}" for motor_id in sorted(found)))

    def request_origin(self, motor_id: int, permanent: bool = False) -> None:
        joint = BY_ID[motor_id]
        ok, line = self.state_line(joint)
        print(line)
        if not ok:
            print("中止: 新鮮なサーボ形式フィードバックと err=0 が必要です。")
            return
        if permanent and joint.model == "AK80-9":
            print("拒否: AK80-9 V3.0はシングルエンコーダです。公式仕様でkind=1恒久原点はデュアルエンコーダ機専用です。")
            return
        kind = 1 if permanent else 0
        label = "kind=1原点（デュアルエンコーダ確認用）" if permanent else "起動中の基準原点"
        confirmation_word = "DUALENCODER" if permanent else "YES"
        confirmation = input(
            f"{label}を送る対象は {joint.name} / 0x{joint.motor_id:02X} / ch={self.route_by_motor_id[joint.motor_id]} です。"
            f"軸は動きません。確定するなら {confirmation_word} 0x{joint.motor_id:02X} と入力: "
        ).strip()
        if confirmation != f"{confirmation_word} 0x{joint.motor_id:02X}":
            print("送信しませんでした。")
            return
        # SET_ORIGIN: kind=0 is temporary; kind=1 writes the motor's
        # permanent-origin setting.  Neither command moves the axis.
        channel = self.route_by_motor_id[joint.motor_id]
        sent = self.buses[channel].send(cm.f_set_origin(joint.motor_id, kind))
        if not sent:
            print("送信失敗。主電源をOFFにして確認してください。")
            self._record(f"origin FAILED kind={kind} {joint.name} 0x{joint.motor_id:02X} ch={channel}")
            return
        time.sleep(0.35)
        _ok, after = self.state_line(joint)
        print(f"{label}を送信しました（軸は動かない）。")
        print(after)
        self._record(f"origin sent kind={kind} {joint.name} 0x{joint.motor_id:02X} ch={channel}; {after}")

    def request_all_origins(self) -> None:
        """Set kind=0 origin for all ten axes after one explicit confirmation."""
        if set(self.route_by_motor_id) != set(BY_ID):
            print("中止: 先に scan 3 を完了し、10軸の送信経路を確定してください。")
            return

        print("一括原点設定の事前確認（送信前）:")
        all_ok = True
        for joint in JOINTS:
            ok, line = self.state_line(joint)
            print(line)
            all_ok = all_ok and ok
        if not all_ok:
            print("中止: 10軸すべてが新鮮なサーボ形式フィードバックかつ err=0 である必要があります。")
            return

        confirmation = input(
            "現在の機械ゼロ姿勢を固定済みで、10軸すべてにkind=0原点を送るなら "
            "YES ALL 10 と入力: "
        ).strip()
        if confirmation != "YES ALL 10":
            print("送信しませんでした。")
            return

        self._record("origin-all begin kind=0")
        for joint in JOINTS:
            ok, line = self.state_line(joint)
            if not ok:
                print(f"中止: {joint.name} のフィードバックが古くなったため、残りは送信しません。")
                self._record(f"origin-all ABORT before {joint.name}: {line}")
                return
            channel = self.route_by_motor_id[joint.motor_id]
            sent = self.buses[channel].send(cm.f_set_origin(joint.motor_id, 0))
            if not sent:
                print(f"送信失敗: {joint.name} / 0x{joint.motor_id:02X}。主電源をOFFにしてください。")
                self._record(f"origin-all FAILED {joint.name} 0x{joint.motor_id:02X} ch={channel}")
                return
            self._record(f"origin-all sent kind=0 {joint.name} 0x{joint.motor_id:02X} ch={channel}")
            time.sleep(0.05)

        print("10軸すべてへkind=0原点を送信しました。")
        time.sleep(0.35)
        self.show()
        self._record("origin-all complete kind=0")

    def loop(self) -> None:
        print("入力: scan 3 / s / s 0x13 / o 0x13 / oa / op 0x13 / q")
        print("o=起動中の基準原点。op=デュアルエンコーダ機だけのkind=1確認用。IDは16進数。qは送信なし。")
        print("oa=10軸すべてへkind=0原点（YES ALL 10 の確認が必要）。")
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
            if command == "oa" and len(parts) == 1:
                self.request_all_origins()
                continue
            if command in {"o", "op"} and len(parts) == 2:
                motor_id = parse_hex_motor_id(parts[1])
                if motor_id is None:
                    print("使い方: o 0x13（起動中）/ op 0x13（デュアルエンコーダ確認用）。登録済み16進数IDだけ指定できます")
                else:
                    self.request_origin(motor_id, permanent=(command == "op"))
                continue
            print("使い方: scan 3 / s / s 0x13 / o 0x13 / oa / op 0x13 / q")


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

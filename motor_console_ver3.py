"""
motor_console.py -- CubeMars サーボモード 対話コンソール（1モーター用）v3

v2 からの変更:
  ・ERPM ⇄ 出力軸 deg/s の換算を表示に追加（実測で検証済みの式を使用）
  ・相対移動コマンド r を追加（今の位置から ±deg。関節可動域テスト用）
  ・電流指令 c に安全ガード（既定0.3秒・上限値・警告）。
    電流モードは位置/速度ループを通らないため、無負荷だと際限なく加速する。
  ・位置指令の後は保持を続けるか脱力するかを選べるようにした（hold_after）

⚠️ 安全
  ・v / p / r / c はモーターが実際に回ります。脚は外す/吊る/固定してから。
  ・c（電流＝トルク指令）は無負荷で暴走的に加速します。実機の脚では p を使うこと。
  ・指令は指定秒で自動的に止まります。Ctrl+C でも速度0を送ってから閉じます。

コマンド:
  s              現在の状態を1回表示
  m [秒]         指令せず状態だけ監視（既定3秒）
  o              今の位置を原点(0)に定義し直す ※モーターは動きません
  p <deg> [秒]   絶対位置へ動かす（既定2秒）例: p 90 2
  r <deg> [秒]   今の位置から相対移動（既定2秒）例: r -30 2
  v <ERPM> [秒]  速度指令（既定2秒）例: v 5000 3
  c <A> [秒]     電流(トルク)指令（既定0.3秒）⚠無負荷で暴走。例: c 0.8 0.3
  x              停止（速度0）
  q              終了
"""

import can
import sys
import time
import struct
import threading

# ------------------------------------------------------------
# 設定（実測で確定した値）
# ------------------------------------------------------------
CHANNEL = 1              # 実測: ch=0 は接続できるが通信できない
BITRATE = 1000000
MOTOR_ID = 0x2B          # 実測: フィードバック ID=0x292B の下位8bit = 43

SEND_HZ = 50             # 指令の送信周期

# --- ERPM ⇄ 出力軸角速度の換算（実測で検証済み） ---
#   減速比 9:1、極対数 21 → ERPM = 出力軸rpm * 189
#   出力軸[deg/s] = ERPM / 31.5
#   検証: v 5000 で実測 158.65 deg/s → 5000 / 158.65 = 31.52  ✓
GEAR_RATIO = 9
POLE_PAIRS = 21
ERPM_PER_DEG_S = GEAR_RATIO * POLE_PAIRS / 6.0     # = 31.5

# --- 安全上限 ---
MAX_CURRENT_A = 3.0      # これを超える電流指令は拒否する
MAX_ERPM = 20000         # これを超える速度指令は拒否する

# サーボモードの制御パケット種別（CAN ID の上位8bit）
PKT_CURRENT     = 1      # データ = int32 (A * 1000)
PKT_VELOCITY    = 3      # データ = int32 ERPM
PKT_POSITION    = 4      # データ = int32 (deg * 10000)  ※出力軸の角度
PKT_SET_ORIGIN  = 5      # データ = 1byte (0=一時 / 1=恒久 / 2=初期化)
PKT_STATUS      = 0x29   # フィードバック（実測 0x29 = 41）

stop_event = threading.Event()
motor_state = {}
state_lock = threading.Lock()


# ------------------------------------------------------------
# フィードバック解析（係数はすべて実測で確定済み）
#   位置: int16/10 → 出力軸 deg（多回転積算。±3276.7deg で飽和）
#         検証: p 90 → 90.0 / p 180 → 179.9 / p 360 → 359.9
#   速度: int16*10 → ERPM
#         検証: v 5000 で実測 158.65deg/s = 4997 ERPM
#   電流: int16/100 → A
#         検証: c 0.8 で cur 0.78〜0.95 を表示
# ------------------------------------------------------------
def parse_feedback(msg):
    if msg.data is None or len(msg.data) < 8:
        return None
    if (msg.arbitration_id >> 8) != PKT_STATUS:
        return None

    motor_id = msg.arbitration_id & 0xFF
    pos  = struct.unpack('>h', msg.data[0:2])[0] / 10.0
    spd  = struct.unpack('>h', msg.data[2:4])[0] * 10.0
    cur  = struct.unpack('>h', msg.data[4:6])[0] / 100.0
    temp = msg.data[6]
    err  = msg.data[7]
    return motor_id, pos, spd, cur, temp, err


def get_state(motor_id=MOTOR_ID):
    with state_lock:
        s = motor_state.get(motor_id)
        return dict(s) if s is not None else None


def receive_background(bus):
    """受信スレッド。保存するだけ。print しない（入力プロンプトを邪魔しない）"""
    while not stop_event.is_set():
        try:
            message = bus.recv(timeout=0.1)
            if message is None:
                continue
            result = parse_feedback(message)
            if result is None:
                continue
            motor_id, pos, spd, cur, temp, err = result
            with state_lock:
                motor_state[motor_id] = {
                    "pos": pos, "spd": spd, "cur": cur,
                    "temp": temp, "err": err, "t": time.time(),
                }
        except can.CanError:
            break
        except Exception:
            break


# ------------------------------------------------------------
# 送信
# ------------------------------------------------------------
def send_packet(bus, packet_type, data_bytes, quiet=False):
    arb = (packet_type << 8) | MOTOR_ID
    try:
        bus.send(can.Message(arbitration_id=arb,
                             data=data_bytes, is_extended_id=True))
        if not quiet:
            print(f"  送信 ID=0x{arb:03X} DATA={' '.join(f'{b:02X}' for b in data_bytes)}")
    except can.CanError as e:
        print(f"  送信エラー: {e}")


def send_velocity(bus, erpm, quiet=False):
    send_packet(bus, PKT_VELOCITY, list(struct.pack('>i', int(erpm))), quiet)


def send_position(bus, deg, quiet=False):
    send_packet(bus, PKT_POSITION, list(struct.pack('>i', int(deg * 10000))), quiet)


def send_current(bus, amp, quiet=False):
    send_packet(bus, PKT_CURRENT, list(struct.pack('>i', int(amp * 1000))), quiet)


# ------------------------------------------------------------
# 表示
# ------------------------------------------------------------
def fmt_state(s):
    if s is None:
        return "  状態なし（フィードバック未受信）"
    age = (time.time() - s["t"]) * 1000
    degs = s["spd"] / ERPM_PER_DEG_S
    return (f"  pos={s['pos']:8.1f} deg  spd={s['spd']:8.0f} ERPM ({degs:7.1f} deg/s)  "
            f"cur={s['cur']:6.2f} A  temp={s['temp']:3d} C  err={s['err']}  "
            f"({age:.0f}ms前)")


def show_state():
    print(fmt_state(get_state()))


def monitor(seconds):
    t0 = time.time()
    next_print = t0
    while time.time() - t0 < seconds:
        if time.time() >= next_print:
            show_state()
            next_print += 0.2
        time.sleep(0.005)


# ------------------------------------------------------------
# 指令を「送り続ける」中核。制御ループの原型
#   hold_after: 終了後も指令を送り続けるか（位置保持したいとき True）
# ------------------------------------------------------------
def hold(bus, send_fn, value, seconds, label, stop_with_velocity_zero=True):
    s0 = get_state()
    pos_start = s0["pos"] if s0 else None
    print(f"  {label} を {SEND_HZ}Hz で {seconds:.1f} 秒間 送り続けます")

    t0 = time.time()
    next_send = t0
    next_print = t0
    spd_peak = 0.0
    cur_peak = 0.0

    try:
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= next_send:
                send_fn(bus, value, quiet=True)
                next_send += 1.0 / SEND_HZ
            s = get_state()
            if s:
                spd_peak = max(spd_peak, abs(s["spd"]))
                cur_peak = max(cur_peak, abs(s["cur"]))
            if now >= next_print:
                print(fmt_state(s))
                next_print += 0.2
            time.sleep(0.002)
    except KeyboardInterrupt:
        print("  中断")

    if stop_with_velocity_zero:
        for _ in range(5):
            send_velocity(bus, 0, quiet=True)
            time.sleep(0.01)
    time.sleep(0.2)

    s1 = get_state()
    pos_end = s1["pos"] if s1 else None
    print("  --- 結果 ---")
    if pos_start is not None and pos_end is not None:
        print(f"  位置: {pos_start:.1f} → {pos_end:.1f} deg  "
              f"(変化 {pos_end - pos_start:+.1f} deg)")
    print(f"  速度の最大: {spd_peak:.0f} ERPM ({spd_peak/ERPM_PER_DEG_S:.1f} deg/s) / "
          f"電流の最大: {cur_peak:.2f} A")
    if spd_peak < 20:
        print("  ※ 軸がほぼ動いていない。電流指令なら、摩擦に負けてトルク不足の可能性。")


# ------------------------------------------------------------
def main():
    try:
        bus = can.Bus(interface='gs_usb', channel=CHANNEL, bitrate=BITRATE)
    except Exception as e:
        print(f"CAN接続に失敗: {e}")
        sys.exit(1)
    print(f"CANバスに接続しました (ch={CHANNEL}, {BITRATE} bps, motor_id={MOTOR_ID})")
    print(f"換算: 1 deg/s = {ERPM_PER_DEG_S:.1f} ERPM （減速比{GEAR_RATIO}:1 × 極対数{POLE_PAIRS}）")

    rx = threading.Thread(target=receive_background, args=(bus,), daemon=True)
    rx.start()
    time.sleep(0.3)
    show_state()
    print(__doc__.split("コマンド:")[1])

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            c = parts[0].lower()

            def num(i, default):
                try:
                    return float(parts[i])
                except (IndexError, ValueError):
                    return default

            if c == "q":
                break

            elif c == "s":
                show_state()

            elif c == "m":
                monitor(num(1, 3.0))

            elif c == "o":
                send_packet(bus, PKT_SET_ORIGIN, [0x00])
                time.sleep(0.3)
                show_state()
                print("  ※ o は『今いる位置を0と定義し直す』コマンドです。軸は動きません。")
                print("     物理的に原点へ戻したいときは p 0 を使ってください。")

            elif c == "p":
                deg = num(1, 0)
                hold(bus, send_position, deg, num(2, 2.0), f"位置 {deg:.1f} deg")

            elif c == "r":
                s = get_state()
                if s is None:
                    print("  状態が取れていないので相対移動できません")
                    continue
                target = s["pos"] + num(1, 0)
                hold(bus, send_position, target, num(2, 2.0),
                     f"相対 {num(1,0):+.1f} deg → 位置 {target:.1f} deg")

            elif c == "v":
                erpm = num(1, 0)
                if abs(erpm) > MAX_ERPM:
                    print(f"  拒否: {MAX_ERPM} ERPM を超える指令です")
                    continue
                hold(bus, send_velocity, erpm, num(2, 2.0),
                     f"速度 {erpm:.0f} ERPM ({erpm/ERPM_PER_DEG_S:.1f} deg/s)")

            elif c == "c":
                amp = num(1, 0)
                if abs(amp) > MAX_CURRENT_A:
                    print(f"  拒否: {MAX_CURRENT_A} A を超える電流指令です")
                    continue
                dur = num(2, 0.3)
                print("  ⚠ 電流(トルク)指令は位置/速度ループを通りません。")
                print("    無負荷だと摩擦とつり合うまで加速し続けます。短時間で試してください。")
                hold(bus, send_current, amp, dur, f"電流 {amp:.2f} A")

            elif c == "x":
                send_velocity(bus, 0)

            else:
                print("  不明なコマンド。s / m / o / p / r / v / c / x / q")

    except KeyboardInterrupt:
        print("\n中断")
    finally:
        print("停止指令(速度0)を送信します...")
        try:
            for _ in range(5):
                send_velocity(bus, 0, quiet=True)
                time.sleep(0.01)
            print("  速度0を送信しました")
        except Exception:
            pass
        stop_event.set()
        rx.join(timeout=1.0)
        time.sleep(0.1)
        bus.shutdown()
        print("CANバスを切断しました")


if __name__ == "__main__":
    main()
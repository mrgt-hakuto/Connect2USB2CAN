"""
motor_console.py -- CubeMars サーボモード 対話コンソール（1モーター用）v2

v1 からの変更:
  ・指令を「1発だけ送る」のをやめ、指定秒数のあいだ 50Hz で送り続ける方式にした。
    サーボモードは通信が途切れると保護で停止するため、1発送信だと一瞬動いて
    止まってしまい、s コマンドで見たときには spd=0 に戻っている。
  ・指令中の状態を 5Hz で表示し、終了時に「位置がいくつ動いたか」を要約する。
    これで位置・速度の係数を実測で検証できる。
  ・手で軸を回す前提の w は廃止（AK10-9 は 9:1 ギアで手回しできない）。
    代わりに p（位置指令）で pos が指令値に一致するかを見る。

⚠️ 安全
  ・v / p / c はモーターが実際に回ります。脚は外す/吊る/固定してから。
  ・電源をすぐ落とせる状態で。Ctrl+C でも必ず速度0を送ってから閉じます。
  ・指令は指定秒で自動的に止まります（既定2秒）。回りっぱなしになりません。

コマンド:
  s              現在の状態を1回表示
  m [秒]         指令せず状態だけ監視（既定3秒）
  o              現在位置を原点にする（テスト前に必ず。pos は ±3276.7deg で飽和する）
  v <ERPM> [秒]  速度指令を指定秒だけ送り続ける（既定2秒）例: v 500 3
  p <deg> [秒]   位置指令を指定秒だけ送り続ける（既定2秒）例: p 90 2
  c <A> [秒]     電流指令を指定秒だけ送り続ける（既定1秒）例: c 1.0 1
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

SEND_HZ = 50             # 指令の送信周期。制御ループもこのくらいで回す

# サーボモードの制御パケット種別（CAN ID の上位8bit）
PKT_CURRENT     = 1      # データ = int32 (A * 1000)
PKT_VELOCITY    = 3      # データ = int32 ERPM
PKT_POSITION    = 4      # データ = int32 (deg * 10000)
PKT_SET_ORIGIN  = 5      # データ = 1byte (0=一時 / 1=恒久 / 2=初期化)
PKT_STATUS      = 0x29   # フィードバック（実測 0x29 = 41）

stop_event = threading.Event()
motor_state = {}
state_lock = threading.Lock()


# ------------------------------------------------------------
# フィードバック解析
#   実測: ID=0x292B data=fa ce 00 00 00 00 21 00
#     位置 -133.0deg / 速度 0 / 電流 0.00A / 温度 33C / エラー 0
#   バイト並びは確定（o コマンドで pos が 0 になることで裏取り済み）。
#   速度・電流の係数は v コマンドの hold 中の表示で検証する。
# ------------------------------------------------------------
def parse_feedback(msg):
    if msg.data is None or len(msg.data) < 8:
        return None
    if (msg.arbitration_id >> 8) != PKT_STATUS:
        return None

    motor_id = msg.arbitration_id & 0xFF
    pos  = struct.unpack('>h', msg.data[0:2])[0] / 10.0     # 角度 [deg] 多回転積算
    spd  = struct.unpack('>h', msg.data[2:4])[0] * 10.0     # 速度 [ERPM] ←係数検証中
    cur  = struct.unpack('>h', msg.data[4:6])[0] / 100.0    # 電流 [A]   ←係数検証中
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
    return (f"  pos={s['pos']:8.1f} deg  spd={s['spd']:8.0f} ERPM  "
            f"cur={s['cur']:6.2f} A  temp={s['temp']:3d} C  err={s['err']}  "
            f"({age:.0f}ms前)")


def show_state():
    print(fmt_state(get_state()))


def monitor(seconds):
    """指令せず状態だけ見る"""
    t0 = time.time()
    next_print = t0
    while time.time() - t0 < seconds:
        if time.time() >= next_print:
            show_state()
            next_print += 0.2
        time.sleep(0.005)


# ------------------------------------------------------------
# 指令を「送り続ける」中核。制御ループの原型でもある
# ------------------------------------------------------------
def hold(bus, send_fn, value, seconds, label):
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

    # 必ず止める
    for _ in range(5):
        send_velocity(bus, 0, quiet=True)
        time.sleep(0.01)
    time.sleep(0.2)

    s1 = get_state()
    pos_end = s1["pos"] if s1 else None
    print("  --- 結果 ---")
    if pos_start is not None and pos_end is not None:
        print(f"  位置: {pos_start:.1f} deg → {pos_end:.1f} deg  "
              f"(変化 {pos_end - pos_start:+.1f} deg)")
    print(f"  観測した速度の最大: {spd_peak:.0f} ERPM / 電流の最大: {cur_peak:.2f} A")
    if spd_peak == 0.0:
        print("  ※ 速度が一度も0以外にならなかった。実際に軸が回ったか目視で確認を。")


# ------------------------------------------------------------
def main():
    try:
        bus = can.Bus(interface='gs_usb', channel=CHANNEL, bitrate=BITRATE)
    except Exception as e:
        print(f"CAN接続に失敗: {e}")
        sys.exit(1)
    print(f"CANバスに接続しました (ch={CHANNEL}, {BITRATE} bps, motor_id={MOTOR_ID})")

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
            elif c == "v":
                hold(bus, send_velocity, num(1, 0), num(2, 2.0),
                     f"速度 {num(1,0):.0f} ERPM")
            elif c == "p":
                hold(bus, send_position, num(1, 0), num(2, 2.0),
                     f"位置 {num(1,0):.1f} deg")
            elif c == "c":
                hold(bus, send_current, num(1, 0), num(2, 1.0),
                     f"電流 {num(1,0):.2f} A")
            elif c == "x":
                send_velocity(bus, 0)
            else:
                print("  不明なコマンド。s / m / o / v / p / c / x / q")

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
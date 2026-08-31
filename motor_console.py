"""
motor_console.py -- CubeMars サーボモード用 対話コンソール（1モーター用）

feedbackSample.py の問題を潰した版:
  ・受信スレッドは print しない（状態を保存するだけ）→ 入力プロンプトが流れない
  ・モーターIDを実測値(43 = 0x2B)に修正
  ・数字を4回手打ちする代わりに、短いコマンドで指令を出す
  ・Ctrl+C で必ず「速度0」を送ってから閉じる

⚠️ 安全
  ・v / p コマンドはモーターが実際に回ります。脚は外す/吊る/固定してから。
  ・電源をすぐ落とせる状態で実行すること。
  ・Ctrl+C か "x" で即停止（速度0送信）。

コマンド:
  s          現在の状態を1回表示
  w [秒]     状態を5Hzで表示し続ける（既定3秒）。手で軸を回して角度が動くか見る用
  v [ERPM]   速度指令（例: v 500）。モーターが回る
  p [deg]    位置指令（例: p 90）。モーターが回る
  c [A]      電流指令（例: c 1.0）。モーターが回る
  o          現在位置を原点にする（一時的）
  x          停止（速度0）
  q          終了
"""

import can
import sys
import time
import struct
import threading

# ------------------------------------------------------------
# 設定（実測で確定した値）
# ------------------------------------------------------------
CHANNEL = 1              # ★ ch=0 では繋がるだけで通信できなかった。実測で ch=1
BITRATE = 1000000
MOTOR_ID = 0x2B          # ★ 実測: フィードバック ID=0x292B の下位8bit = 43

# サーボモードの制御パケット種別（上位8bit に入る）
PKT_DUTY        = 0
PKT_CURRENT     = 1
PKT_CURRENT_BRK = 2
PKT_VELOCITY    = 3      # データ = int32 ERPM
PKT_POSITION    = 4      # データ = int32 (deg * 10000)
PKT_SET_ORIGIN  = 5      # データ = 1byte (0=一時, 1=恒久, 2=初期化)

# フィードバックのパケット種別（実測: 0x29 = 41）
PKT_STATUS      = 0x29

stop_event = threading.Event()
motor_state = {}
state_lock = threading.Lock()


# ------------------------------------------------------------
# フィードバック解析
#   実測フレーム: ID=0x292B data=fa ce 00 00 00 00 21 00
#     fa ce -> -1330 /10 = -133.0 deg
#     00 00 -> 0 ERPM
#     00 00 -> 0.00 A
#     21    -> 33 ℃
#     00    -> エラーなし
#   → 並びは想定どおり。係数は w コマンドで軸を手回しして検証すること。
# ------------------------------------------------------------
def parse_feedback(msg):
    if msg.data is None or len(msg.data) < 8:
        return None
    if (msg.arbitration_id >> 8) != PKT_STATUS:      # 状態フレーム以外は無視
        return None

    motor_id = msg.arbitration_id & 0xFF
    pos  = struct.unpack('>h', msg.data[0:2])[0] / 10.0     # 角度 [deg]
    spd  = struct.unpack('>h', msg.data[2:4])[0] * 10.0     # 速度 [ERPM]
    cur  = struct.unpack('>h', msg.data[4:6])[0] / 100.0    # 電流 [A]
    temp = msg.data[6]                                      # 温度 [℃]
    err  = msg.data[7]                                      # エラーコード
    return motor_id, pos, spd, cur, temp, err


def get_state(motor_id):
    with state_lock:
        s = motor_state.get(motor_id)
        return dict(s) if s is not None else None


# ------------------------------------------------------------
# 受信スレッド: 保存するだけ。print しない（入力プロンプトを邪魔しないため）
# ------------------------------------------------------------
def receive_background(bus):
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
def send_packet(bus, packet_type, data_bytes):
    arb = (packet_type << 8) | MOTOR_ID
    try:
        bus.send(can.Message(arbitration_id=arb,
                             data=data_bytes, is_extended_id=True))
        print(f"  送信 ID=0x{arb:03X} DATA={' '.join(f'{b:02X}' for b in data_bytes)}")
    except can.CanError as e:
        print(f"  送信エラー: {e}")


def cmd_velocity(bus, erpm):
    send_packet(bus, PKT_VELOCITY, list(struct.pack('>i', int(erpm))))


def cmd_position(bus, deg):
    send_packet(bus, PKT_POSITION, list(struct.pack('>i', int(deg * 10000))))


def cmd_current(bus, amp):
    send_packet(bus, PKT_CURRENT, list(struct.pack('>i', int(amp * 1000))))


def show_state():
    s = get_state(MOTOR_ID)
    if s is None:
        print("  状態なし（フィードバックを受信していない）")
        return
    age = time.time() - s["t"]
    print(f"  pos={s['pos']:8.1f} deg   spd={s['spd']:7.0f} ERPM   "
          f"cur={s['cur']:6.2f} A   temp={s['temp']:3d} C   err={s['err']}   "
          f"({age*1000:.0f}ms前)")


def watch(seconds):
    print("  軸を手で回して pos が動くか見てください。")
    t0 = time.time()
    while time.time() - t0 < seconds:
        show_state()
        time.sleep(0.2)


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
            arg = parts[1] if len(parts) > 1 else None

            if c == "q":
                break
            elif c == "s":
                show_state()
            elif c == "w":
                watch(float(arg) if arg else 3.0)
            elif c == "v":
                cmd_velocity(bus, float(arg) if arg else 0)
            elif c == "p":
                cmd_position(bus, float(arg) if arg else 0)
            elif c == "c":
                cmd_current(bus, float(arg) if arg else 0)
            elif c == "o":
                send_packet(bus, PKT_SET_ORIGIN, [0x00])
            elif c == "x":
                cmd_velocity(bus, 0)
            else:
                print("  不明なコマンド。s / w / v / p / c / o / x / q")

    except KeyboardInterrupt:
        print("\n中断")
    finally:
        print("停止指令(速度0)を送信します...")
        try:
            for _ in range(5):
                cmd_velocity(bus, 0)
                time.sleep(0.01)
        except Exception:
            pass
        stop_event.set()
        rx.join(timeout=1.0)
        time.sleep(0.1)
        bus.shutdown()
        print("CANバスを切断しました")


if __name__ == "__main__":
    main()
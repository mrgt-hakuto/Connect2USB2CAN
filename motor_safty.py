import can
import sys
import time
import struct
import threading

# ============================================================
#  ⑤安全機構 だけを搭載した版(単体で動く)
#  含む機能:
#    (1) 非常停止 estop_all … 全モーターを即座に無トルクにする
#    (2) enable_all / disable_all … モーターの有効化/無効化
#    (3) 受信タイムアウト・フェイルセーフ … 一定時間フレームが
#        来なければ自動で estop（暴走・断線対策）
#
#  ★注意: 受信タイムアウト監視は「最後に受けた時刻」を使う。
#    これは本来①(フィードバック解析)が持つ情報。ここでは
#    フル解析はせず「受信時刻だけ記録する最小の見張り」を内蔵する。
#    統合時はこの見張りを①の受信スレッドに合流させること。
#
#  【ライブラリ(python-can)】can.Message 組み立て / bus.send / bus.recv
#  【自作】停止フレームの中身、タイムアウト判定ロジック、監視スレッド
# ============================================================

# ------------------------------------------------------------
# 設定(実測で確定するまでの仮置き)
# ------------------------------------------------------------
# 制御方式。手順書では MIT が本命。実測で決まったら合わせる。
MODE = "mit"                 # "mit" か "servo"

# 監視・停止の対象モーターID。実機のID割り当てに合わせる。
# 例: V3 の 0x801+ 方式なら [0x801, 0x802, ...]、旧サーボなら [42, ...]。
MOTOR_IDS = [0x801, 0x802, 0x803, 0x804, 0x805,
             0x806, 0x807, 0x808, 0x809, 0x80A]

# MIT は通常 標準11bitフレーム(要実測)。旧サーボは拡張フレーム。
MIT_EXTENDED_ID   = False
SERVO_EXTENDED_ID = True

# 受信がこの秒数途切れたら異常とみなして estop する
RX_TIMEOUT = 0.2             # [s]  制御周期の数倍を目安に

# ------------------------------------------------------------
# 共有状態
# ------------------------------------------------------------
stop_event   = threading.Event()   # スレッド全体の終了合図
estop_active = threading.Event()   # 非常停止が発動済みかのフラグ

_last_rx_time = time.time()
_rx_lock = threading.Lock()


# ============================================================
#  (1) 非常停止 ―― 最重要
# ============================================================
# MIT: 全FF+FD が「モーター無効(トルク解除)」。まず全軸へ送る。
def _mit_disable_frame(bus, motor_id):
    data = bytes([0xFF]*7 + [0xFD])
    bus.send(can.Message(arbitration_id=motor_id, data=data,
                         is_extended_id=MIT_EXTENDED_ID))

# MIT: 位置0・速度0・kp0・kd0・トルク0 の「無トルク指令」。念押し用。
def _mit_zero_cmd(bus, motor_id):
    # kp=0,kd=0,torque=0 → どの値でもトルクを出さない
    data = bytes([0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF])
    bus.send(can.Message(arbitration_id=motor_id, data=data,
                         is_extended_id=MIT_EXTENDED_ID))

# サーボ: 電流0指令(制御タイプ1=Current)。トルクを止める。
def _servo_zero_current(bus, motor_id):
    arb  = (1 << 8) | (motor_id & 0xFF)             # 1 = SET_CURRENT
    data = struct.pack('>i', 0)                     # 0 A
    bus.send(can.Message(arbitration_id=arb, data=data,
                         is_extended_id=SERVO_EXTENDED_ID))


def estop_all(bus):
    """全モーターを即座に無トルクにする。何度呼んでも安全。"""
    estop_active.set()
    for mid in MOTOR_IDS:
        try:
            if MODE == "mit":
                _mit_zero_cmd(bus, mid)             # まず無トルク指令
                _mit_disable_frame(bus, mid)        # 続けて無効化
            else:
                _servo_zero_current(bus, mid)
        except can.CanError as e:
            # 1軸送信に失敗しても残りは止めにいく(握り潰さず記録)
            print(f"[ESTOP] ID={mid} 送信失敗: {e}")
    print("‼️  非常停止を発動：全モーター無トルク。")


# ============================================================
#  (2) enable / disable
# ============================================================
def _mit_enable_frame(bus, motor_id):
    data = bytes([0xFF]*7 + [0xFC])
    bus.send(can.Message(arbitration_id=motor_id, data=data,
                         is_extended_id=MIT_EXTENDED_ID))

def enable_all(bus):
    if MODE != "mit":
        print("[enable] サーボモードは enable フレーム不要。スキップ。")
        return
    estop_active.clear()
    for mid in MOTOR_IDS:
        _mit_enable_frame(bus, mid)
    print("全モーターを有効化しました。")

def disable_all(bus):
    for mid in MOTOR_IDS:
        if MODE == "mit":
            _mit_disable_frame(bus, mid)
        else:
            _servo_zero_current(bus, mid)
    print("全モーターを無効化しました。")


# ============================================================
#  (3) 受信タイムアウト・フェイルセーフ
# ============================================================
# 最小の受信見張り(パースはしない=①ではない)。受信時刻だけ更新。
def rx_timestamp_watch(bus):
    global _last_rx_time
    print("【受信見張り】フレーム受信時刻の記録を開始。")
    while not stop_event.is_set():
        try:
            msg = bus.recv(timeout=0.1)             # ライブラリ
            if msg is not None:
                with _rx_lock:
                    _last_rx_time = time.time()     # 時刻だけ更新
        except can.CanError as e:
            print(f"【受信見張り】CAN通信エラー: {e}")
            break
    print("【受信見張り】停止。")


# 監視スレッド:最後の受信から RX_TIMEOUT を超えたら estop
def failsafe_monitor(bus):
    print(f"【フェイルセーフ】受信途絶 {RX_TIMEOUT}s で自動停止。")
    while not stop_event.is_set():
        with _rx_lock:
            elapsed = time.time() - _last_rx_time
        if elapsed > RX_TIMEOUT and not estop_active.is_set():
            print(f"[フェイルセーフ] 受信が {elapsed:.2f}s 途絶 → 非常停止。")
            estop_all(bus)
        time.sleep(RX_TIMEOUT / 4.0)
    print("【フェイルセーフ】停止。")


# ------------------------------------------------------------
# 接続(元のまま)
# ------------------------------------------------------------
def connect2USB2CAN(channel):
    try:
        bus = can.Bus(interface='gs_usb', channel=channel, bitrate=1000000)
        print(f"CANバスに接続しました(CAN={channel})")
        return bus
    except can.CanError as e:
        print(f"CAN通信エラー: {e}")
        return None
    except Exception as e:
        print(f"予期せぬエラー: {e}")
        return None


# ------------------------------------------------------------
# 単体デモ:enable → しばらく監視 → 手動 estop で終わる
#   ※実機では脚を吊る/トルクを絞る/この estop を手元に置いてから通電。
# ------------------------------------------------------------
def main():
    bus = connect2USB2CAN(channel=0)
    if bus is None:
        print("接続に失敗したため中断します。")
        sys.exit()

    # 受信見張りとフェイルセーフを起動
    t_watch = threading.Thread(target=rx_timestamp_watch, args=(bus,), daemon=True)
    t_fail  = threading.Thread(target=failsafe_monitor,  args=(bus,), daemon=True)
    t_watch.start()
    t_fail.start()

    try:
        enable_all(bus)
        print("\nEnterで非常停止、Ctrl+Cでも安全に停止します。")
        input()                         # ここで待機(実際は制御ループが回る場所)
        estop_all(bus)

    except KeyboardInterrupt:
        print("\nCtrl+C を検知。")
        estop_all(bus)                  # 割り込みでも必ず止める

    finally:
        stop_event.set()
        estop_all(bus)                  # 二重の保険
        t_watch.join(timeout=1.0)
        t_fail.join(timeout=1.0)
        bus.shutdown()
        print("CANバスを切断しました。")


if __name__ == "__main__":
    main()
import can
import sys
import time
import struct
import threading

# ============================================================
#  ①フィードバック解析だけを追加した版
#  ・元プログラムの「手打ちで指令を送る」機能はそのまま
#  ・受信スレッドが返信を「捨てる」のをやめ、
#    現在角度/速度/電流/温度/エラー に翻訳して保存する
#
#  【ライブラリ(python-can)の担当】
#    bus.recv() が 1フレームを can.Message にして返す。
#    .arbitration_id(整数) と .data(生バイト列) までをくれる。
#  【自作の担当】
#    そのバイト列を struct.unpack で数値に戻し、係数を掛けて
#    物理量にする(=parse_feedback)。最新状態を Lock 付き辞書に保存。
# ============================================================

# ------------------------------------------------------------
# 設定
# ------------------------------------------------------------
# 最初は True にして「生ログ」で実際のフレーム並びと係数を確認する(手順1)。
# 並びとスケールが確認できたら False にして翻訳済みの値を使う。
RAW_LOG = True

# スレッドを安全に終了させるためのフラグ
stop_event = threading.Event()

# 最新のモーター状態を保存する共有辞書と、その排他ロック(自作)
#   {motor_id: {"pos":deg, "spd":ERPM, "cur":A, "temp":℃, "err":code}}
motor_state = {}
state_lock = threading.Lock()


# ------------------------------------------------------------
# ① フィードバック解析(自作コーデック)
# ------------------------------------------------------------
# msg.data の並び・係数は CubeMars サーボモードの典型値。
# ★ファームのバージョンで変わることがあるので、RAW_LOG=True の
#   生ログと必ず突き合わせてから信用すること。
def parse_feedback(msg):
    # データ長が足りない返信(ACK等)は解析対象外
    if msg.data is None or len(msg.data) < 8:
        return None

    motor_id = msg.arbitration_id & 0xFF            # 下位1バイト = モーターID

    # '>h' = ビッグエンディアンの符号付き16bit整数
    pos  = struct.unpack('>h', msg.data[0:2])[0] / 10.0    # 現在角度 [deg]  ←係数要確認
    spd  = struct.unpack('>h', msg.data[2:4])[0] * 10.0    # 速度 [ERPM]    ←係数要確認
    cur  = struct.unpack('>h', msg.data[4:6])[0] / 100.0   # 電流 [A]       ←係数要確認
    temp = msg.data[6]                                      # 温度 [℃]
    err  = msg.data[7]                                      # エラーコード

    return motor_id, pos, spd, cur, temp, err


# 他スレッド(将来の制御ループ等)から最新状態を安全に読む関数(自作)???????????
def get_state(motor_id):
    with state_lock:
        s = motor_state.get(motor_id)
        return dict(s) if s is not None else None   # コピーを返して競合を避ける    ??????
    


# ------------------------------------------------------------
# バックグラウンド受信スレッド(①を組み込み)
# ------------------------------------------------------------
def receive_background(bus):
    print("【受信スレッド】裏側で受信バッファの監視を開始しました。")

    while not stop_event.is_set():
        try:
            message = bus.recv(timeout=0.1)         # ← ここまで python-can
            if message is None:
                continue

            # 手順1:生ログ(実際のID・バイト並びを目で確認するため)
            if RAW_LOG:
                print(f"\n[生受信] ID=0x{message.arbitration_id:08X} "
                      f"DLC={message.dlc} data={message.data.hex(' ')}")

            # 手順2:翻訳して保存(ここから自作)
            result = parse_feedback(message)
            if result is not None:
                motor_id, pos, spd, cur, temp, err = result
                with state_lock:
                    motor_state[motor_id] = {
                        "pos": pos, "spd": spd, "cur": cur,
                        "temp": temp, "err": err,
                    }
                if not RAW_LOG:
                    print(f"\n[状態] ID={motor_id} "
                          f"pos={pos:.1f}deg spd={spd:.0f}ERPM "
                          f"cur={cur:.2f}A temp={temp}C err={err}")

        except can.CanError as e:
            print(f"【受信スレッド】CAN通信エラー: {e}")
            break
        except Exception as e:
            print(f"【受信スレッド】予期せぬエラー: {e}")
            break

    print("【受信スレッド】停止しました。")


# ------------------------------------------------------------
# USB2CANへ接続し、そのbusを返す関数(元のまま)
# ------------------------------------------------------------
def connect2USB2CAN(channel):
    bustype = 'gs_usb'
    bitrate = 1000000  # 1000 kbps

    try:
        bus = can.Bus(interface=bustype, channel=channel, bitrate=bitrate)
        print(f"CANバスに接続しました(CAN={channel})")
    except can.CanError as e:
        print(f"CAN通信エラーが発生しました: {e}")
        return None
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")
        return None

    return bus


# ------------------------------------------------------------
# モーターへデータを送信する関数(元のまま)
# ------------------------------------------------------------
def send2Morter(bus, arbitration_id, data):
    try:
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        bus.send(msg)
        print(f"メッセージを送信しました。(ID=0x{arbitration_id:X})")
    except can.CanError as e:
        print(f"CAN通信エラーが発生しました: {e}")
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")


# ------------------------------------------------------------
# メイン(元の手打ちループを維持。受信は裏スレッドが解析する)
# ------------------------------------------------------------
def main():
    bus = connect2USB2CAN(channel=1)
    if bus is None:
        print("CANバスの接続に失敗したため、処理を中断します。")
        sys.exit()

    rx_thread = threading.Thread(target=receive_background, args=(bus,))
    rx_thread.daemon = True
    rx_thread.start()

    try:
        arbitration_id = 0x032b
        data = [0x00, 0x00, 0x18, 0x88]

        while True:
            # ※元コードにあった main 内の bus.recv(...) は削除。
            #   受信は裏スレッドが一手に担当するため(奪い合い・誤breakを防ぐ)。
            print('arbitration_id')
            arbitration_id = int(input(), 16)

            print('data[0]')
            data[0] = int(input(), 16)
            print('data[1]')
            data[1] = int(input(), 16)
            print('data[2]')
            data[2] = int(input(), 16)
            print('data[3]')
            data[3] = int(input(), 16)

            send2Morter(bus, arbitration_id, data)
            time.sleep(1)

    finally:
        print("受信スレッドを停止しています...")
        stop_event.set()
        rx_thread.join(timeout=1.0)
        bus.shutdown()
        print("CANバスを切断(手動クローズ)しました")


if __name__ == "__main__":
    main()
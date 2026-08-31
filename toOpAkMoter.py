import can
import sys
import time
import threading

# マイナスの数値も正しく判定できる関数を追加
def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def main():
    # USB2CANのCAN0へ接続
    bus0 = connect2USB2CAN(channel = 0)
    if bus0 is None:
        print("CANバスの接続に失敗したため、処理を中断します。")
        sys.exit()

    rx_thread = threading.Thread(target=receive_background, args=(bus0,))
    rx_thread.daemon = True  # メイン処理が強制終了した際、道連れで終了させる設定
    rx_thread.start()

    # モーターへ命令を送信
    try:
        print("※終了したい場合はCtrl+Cなどで強制終了すること。\n※各質問に「h」と入力すると、説明が表示されます。")

        while True:

            controlMode, arbitration_id = inputId()
            data = inputData(controlMode)

            send2Motor(bus0, arbitration_id, data)
            time.sleep(1)

    finally:
        print("受信スレッドを停止しています...")
        stop_event.set()           # スレッド内の while ループを終わらせる合図を送る
        rx_thread.join(timeout=1.0)

        # 例外が発生しても、通信が終わったら必ずここを通ってクローズする
        bus0.shutdown()
        print("CANバスを切断（手動クローズ）しました")



# スレッドを安全に終了させるためのフラグ
stop_event = threading.Event()
# バックグラウンドで動き続ける受信専用関数
def receive_background(bus):
    print("【受信スレッド】裏側で受信バッファの監視を開始しました。")

    # stop_eventに「終了しろ」という合図が送られるまで無限ループ
    while not stop_event.is_set():
        try:
            # timeout=0.1 を設定し、0.1秒ごとにループを回して終了合図を確認できるようにする
            message = bus.recv(timeout=0.1)

            if message:
                # 受信したデータを処理（今回はバッファを空にするのが目的なので何もしない）
                # ※モーターからの返信を確認したい場合は、以下の # を外してください
                # print(f"\n[受信] ID=0x{message.arbitration_id:X}, Data={message.data}")
                pass

        except can.CanError as e:
            print(f"【受信スレッド】CAN通信エラー: {e}")
            break
        except Exception as e:
            print(f"【受信スレッド】予期せぬエラー: {e}")
            break

    print("【受信スレッド】停止しました。")

# USB2CANへ接続し、そのbusを返す関数
def connect2USB2CAN(channel):
    # USB-CANアダプタの設定
    bustype = 'gs_usb'
    bitrate = 1000000 # 1000 kbps

    # USB2CANへ接続
    try:
        bus = can.Bus(interface=bustype, channel=channel, bitrate=bitrate)
        print(f"CANバスに接続しました(CAN={channel})")

    # CAN関連のエラーをキャッチ
    except can.CanError as e:
        print(f"CAN通信エラーが発生しました: {e}")
        return None
    # その他予期せぬエラー
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")
        return None

    return bus

# モーターへデータを送信する関数。
def send2Motor(bus, arbitration_id, data):
    try:
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        bus.send(msg)
        print(f"メッセージを送信しました。(ID=0x{arbitration_id:X})")

    # CAN関連のエラーをキャッチ
    except can.CanError as e:
        print(f"CAN通信エラーが発生しました: {e}")
    # その他予期せぬエラー
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")

# モーターからデータを受信する関数
def receiveMotor(bus):
    try:
        print("メッセージを待機中...")
        message = bus.recv(timeout=5.0)
        if message:
            print(f"受信: ID={message.arbitration_id:X} Data={message.data}")
        else:
            print("タイムアウトしました。")

    # CAN関連のエラーをキャッチ
    except can.CanError as e:
        print(f"CAN通信エラーが発生しました: {e}")
    # その他予期せぬエラー
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")

# arbitration_idを読み取る関数
def inputId():
    # 使用する変数
    motorId = 0
    controlMode = 0

    # 操作するモーターの指定
    while True:
        ans = input("操作するモーターのID：")

        if is_int(ans):  # isdigit から変更
            motorId = int(ans)
            if 0 <= motorId < 256:
                break

        if ans == "h":
            print("各モーターに貼り付けてある値を入力します。")
        else:
            print("エラー：対応する数値以外が入力されました。")

    # モータの制御モードとデータの指定
    while True:
        ans = input("モーターの制御モード：")
        if is_int(ans):  # isdigit から変更
            controlMode = int(ans)
            break

        if ans == "h":
            print("0:デューティーサイクルモード\n1:電流ループモード\n2:電流ブレーキモード\n3:速度ループモード\n4:位置ループモード\n5:原点設定モード\n6:位置-速度ループモード\n8:力制御モード")
        else:
            print("エラー：対応する数値以外が入力されました。")

    arbitration_id = 0x100 * controlMode + motorId

    return controlMode, arbitration_id


# Dataを読み取る関数
def inputData(controlMode):
    # 使用する変数
    data = []

    # モータの制御モードとデータの指定
    match controlMode:
        case 0:
            ans = input("目標デューティー比×100000の値(int32)：")
            if is_int(ans):
                data = makeDataArray(int(ans), 4)
            else:
                print("エラー：対応する数値以外が入力されました。")
                data = [0x00, 0x00, 0x00, 0x00]

        case 1:
            ans = input("目標lq電流(mA)の値(int32)：")
            if is_int(ans):
                data = makeDataArray(int(ans), 4)
            else:
                print("エラー：対応する数値以外が入力されました。")
                data = [0x00, 0x00, 0x00, 0x00]

        case 2:
            ans = input("目標ブレーキ電流(mA)の値(int32)：")
            if is_int(ans):
                data = makeDataArray(int(ans), 4)
            else:
                print("エラー：対応する数値以外が入力されました。")
                data = [0x00, 0x00, 0x00, 0x00]

        case 3:
            ans = input("目標速度(ERPM)の値(int32)：")
            if is_int(ans):
                data = makeDataArray(int(ans), 4)
            else:
                print("エラー：対応する数値以外が入力されました。")
                data = [0x00, 0x00, 0x00, 0x00]

        case 4:
            ans = input("目標角度(度)×10000の値(int32)：")
            if is_int(ans):
                data = makeDataArray(int(ans), 4)
            else:
                print("エラー：対応する数値以外が入力されました。")
                data = [0x00, 0x00, 0x00, 0x00]

        case 5:
            ans = input("(0:一時的なゼロ点 1:恒久的なゼロ点)：")
            if is_int(ans):
                data = [0x00] if ans == "0" else [0x01]
            else:
                print("エラー：対応する数値以外が入力されました。")
                data = [0x02]

        case 6:
            ans0 = input("目標角度(度)×10000の値(int32)：")
            ans1 = input("設定速度(ERPM)/10の値(int16)：")
            ans2 = input("設定加速度(ERPM/s^2)/10の値(int16)：")
            if is_int(ans0) and is_int(ans1) and is_int(ans2):
                # マスク処理を追加し、上位データが破壊されないようにする
                val0 = int(ans0) & 0xFFFFFFFF
                val1 = int(ans1) & 0xFFFF
                val2 = int(ans2) & 0xFFFF

                # シフト演算で結合（掛け算でも良いですが、こちらの方が安全です）
                ans = (val0 << 32) | (val1 << 16) | val2
                data = makeDataArray(ans, 8)
            else:
                print("エラー：対応する数値以外が入力されました。")
                data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        case 8:
            data = inputMITData()

        case _:
            print("エラー：対応する数値以外が入力されました。")
            data = [0x00, 0x00, 0x00, 0x00]

    return data

# 数値から配列に作成するモード
def makeDataArray(num, size = 4):
    if size not in (4, 8):
        raise ValueError("要素数は4または8を指定してください")

    # 最上位バイトから順にシフトしてマスク処理を行う
    # 例(size=4の場合): 24ビット右シフト -> 16ビット -> 8ビット -> 0ビット
    return [(num >> (8 * (size - 1 - i))) & 0xFF for i in range(size)]

def inputMITData():
    data = []
    ans0 = input("Kp(比例ゲイン)(12bit)")
    ans1 = input("Kd(微分ゲイン)(12bit)")
    ans2 = input("目標位置(rad)(16bit)")
    ans3 = input("目標速度(rad/s)(12bit)")
    ans4 = input("目標トルク(N×m)(12bit)")

    if is_int(ans0) and is_int(ans1) and is_int(ans2) and is_int(ans3) and is_int(ans4):
        # 各ビット数に合わせてマスク処理(&)を行う
        val0 = int(ans0) & 0xFFF
        val1 = int(ans1) & 0xFFF
        val2 = int(ans2) & 0xFFFF
        val3 = int(ans3) & 0xFFF
        val4 = int(ans4) & 0xFFF

        # シフト演算で正しく結合する
        ans = (val0 << 52) | (val1 << 40) | (val2 << 24) | (val3 << 12) | val4
        data = makeDataArray(ans, 8)
    else:
        print("エラー：対応する数値以外が入力されました。")
        data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    return data
if __name__ == "__main__":
    main()

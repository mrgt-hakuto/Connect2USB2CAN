import can
import sys
import time
import threading

def main():
    # USB2CANのCAN0へ接続
    bus = connect2USB2CAN(channel = 0)
    if bus is None:
        print("CANバスの接続に失敗したため、処理を中断します。")
        sys.exit()

    rx_thread = threading.Thread(target=receive_background, args=(bus,))
    rx_thread.daemon = True  # メイン処理が強制終了した際、道連れで終了させる設定
    rx_thread.start()

    # モーターへ命令を送信
    try:
        arbitration_id = 0x032a
        data= [0x00, 0x00, 0x18, 0x88]

        while 1:
            print('arbitration_id')
            arbitration_id = int(input(), 16)

            msg = bus.recv(timeout=0.1)
            if msg is None:
                break

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
        stop_event.set()           # スレッド内の while ループを終わらせる合図を送る
        rx_thread.join(timeout=1.0)

        # 例外が発生しても、通信が終わったら必ずここを通ってクローズする
        bus.shutdown()
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
def send2Morter(bus, arbitration_id, data):
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
def receiveMorter(bus):
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


if __name__ == "__main__":
    main()

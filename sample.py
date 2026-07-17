import can
import sys

def main():
    # USB2CANのCAN0へ接続
    bus = connect2USB2CAN(channel = 0)
    if bus is None:
        print("CANバスの接続に失敗したため、処理を中断します。")
        sys.exit()

    # モーターへ命令を送信
    try:
        arbitration_id = 0x0368
        data = [0x00, 0x00, 0x13, 0x88]

        send2Morter(bus, arbitration_id, data)

    finally:
        # 例外が発生しても、通信が終わったら必ずここを通ってクローズする
        bus.shutdown()
        print("CANバスを切断（手動クローズ）しました")



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

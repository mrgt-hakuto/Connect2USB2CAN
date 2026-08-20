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
        while 1:
            print("回転モード：")
            control_mode_id = int(input())
            if control_mode_id == "fin":
                break

            print("モーターID：")
            drive_id = int(input(), 16)

            print("送信データ：")
            dataText = input()

            send2Motor(control_mode_id, drive_id, dataText, bus)

    finally:
        # 例外が発生しても、通信が終わったら必ずここを通ってクローズする
        bus.shutdown()
        print("CANバスを切断（手動クローズ）しました")



# USB2CANへ接続し、そのbusを返す関数
def connect2USB2CAN(channel):
    # USB-CANアダプタの設定
    bustype = 'gs_usb'
    bitrate = 1000000  # 1000 kbps

    try:
        bus = can.Bus(interface=bustype, channel=channel, bitrate=bitrate)
        print(f"CANバスに接続しました(CAN={channel})")
        return bus
    except can.CanError as e:
        print(f"CAN通信エラーが発生しました: {e}")
        return None
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")
        return None


# CANでデータを送信する関数
def sendByCan(bus, arbitration_id, data, is_extended_id):
    try:
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=is_extended_id)
        bus.send(msg)
        print(f"メッセージを送信しました。(ID=0x{arbitration_id:X})")
    except can.CanError as e:
        print(f"CAN通信エラーが発生しました: {e}")
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")


# CANでデータを受信する関数 (現在未使用)
def receiveCan(bus):
    try:
        print("メッセージを待機中...")
        message = bus.recv(timeout=5.0)
        if message:
            print(f"受信: ID={message.arbitration_id:X} Data={message.data}")
        else:
            print("タイムアウトしました。")
    except can.CanError as e:
        print(f"CAN通信エラーが発生しました: {e}")
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")


# 文字列をCAN通信用のリストに変換する関数
def string2CanList(dataText):
    result = [int(dataText[i:i+2], 16) for i in range(0, len(dataText), 2)]
    return result


# CAN IDを生成する関数
def makeCanID(control_mode_id, drive_id):
    canId = control_mode_id * 0x100 + drive_id
    return canId


# モーターへデータを送信する関数
def send2Motor(control_mode_id, drive_id, dataText, bus):
    arbitration_id = makeCanID(control_mode_id, drive_id)
    data = string2CanList(dataText)
    # ※モーターの仕様が標準ID(11bit)の場合は、以下の True を False に変更してください
    sendByCan(bus, arbitration_id, data, True)


if __name__ == "__main__":
    main()

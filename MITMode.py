import can
import sys
import time
import threading

MOTOR_TYPE = "AK10-9"
MIT_CONTROL_MODE = 8

PARAM_RANGES = {
    "AK10-9": {
        "P_MIN": -12.56,
        "P_MAX": 12.56,
        "V_MIN": -28.0,
        "V_MAX": 28.0,
        "T_MIN": -54.0,
        "T_MAX": 54.0,
        "KP_MIN": 0.0,
        "KP_MAX": 500.0,
        "KD_MIN": 0.0,
        "KD_MAX": 5.0,
    },
    "AK80-9": {
        "P_MIN": -12.56,
        "P_MAX": 12.56,
        "V_MIN": -65.0,
        "V_MAX": 65.0,
        "T_MIN": -18.0,
        "T_MAX": 18.0,
        "KP_MIN": 0.0,
        "KP_MAX": 500.0,
        "KD_MIN": 0.0,
        "KD_MAX": 5.0,
    },
}

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """浮動小数点数を指定ビット数の符号なし整数へ線形変換 (マニュアル45ページ)"""
    span = x_max - x_min
    if x < x_min:
        x = x_min
    elif x > x_max:
        x = x_max
    # マニュアルの式: (int)((x - x_min) * ((float)((1 << bits) / span)))
    # Pythonでの丸め誤差を防ぐため以下のように計算
    return int(((x - x_min) / span) * ((1 << bits) - 1))

def createMITData(
    kp: float,
    kd: float,
    position: float,
    velocity: float,
    torque: float,
    motor_type: str = MOTOR_TYPE,
):
    if motor_type not in PARAM_RANGES:
        raise ValueError(f"未対応のモータータイプです: {motor_type}")

    return packMITData(
        kp,
        kd,
        position,
        velocity,
        torque,
        PARAM_RANGES[motor_type],
    )

def packMITData(kp: float, kd: float, position: float, velocity: float, torque: float, params: dict):
    # 各パラメータの整数マッピング (44~45ページ)
    kp_int = float_to_uint(kp, params["KP_MIN"], params["KP_MAX"], 12)
    kd_int = float_to_uint(kd, params["KD_MIN"], params["KD_MAX"], 12)
    p_int = float_to_uint(position, params["P_MIN"], params["P_MAX"], 16)
    v_int = float_to_uint(velocity, params["V_MIN"], params["V_MAX"], 12)
    t_int = float_to_uint(torque, params["T_MIN"], params["T_MAX"], 12)

    # CANバッファへのパッキング (44~45ページ pack_cmd の仕様)
    data = [0] * 8
    data[0] = (kp_int >> 4) & 0xFF  # KP high 8 bits
    data[1] = ((kp_int & 0x0F) << 4) | (
        (kd_int >> 8) & 0x0F
    )  # KP Low 4 | Kd High 4
    data[2] = kd_int & 0xFF  # Kd low 8 bits
    data[3] = (p_int >> 8) & 0xFF  # Position high 8 bits
    data[4] = p_int & 0xFF  # Position low 8 bits
    data[5] = (v_int >> 4) & 0xFF  # Speed high 8 bits
    data[6] = ((v_int & 0x0F) << 4) | (
        (t_int >> 8) & 0x0F
    )  # Speed low 4 | Torque high 4
    data[7] = t_int & 0xFF  # Torque low 8 bits

    return data

def inputMITParameters(motor_type: str = MOTOR_TYPE):
    params = PARAM_RANGES.get(motor_type, PARAM_RANGES[MOTOR_TYPE])

    try:
        kp = float(
            input(f"Kp(比例ゲイン) ({params['KP_MIN']}~{params['KP_MAX']}): ")
        )
        kd = float(
            input(f"Kd(微分ゲイン) ({params['KD_MIN']}~{params['KD_MAX']}): ")
        )
        position = float(
            input(f"目標位置(rad) ({params['P_MIN']}~{params['P_MAX']}): ")
        )
        velocity = float(
            input(f"目標速度(rad/s) ({params['V_MIN']}~{params['V_MAX']}): ")
        )
        torque = float(
            input(f"目標トルク(N*m) ({params['T_MIN']}~{params['T_MAX']}): ")
        )
    except ValueError:
        print("エラー：有効な数値を入力してください。")
        return 0.0, 0.0, 0.0, 0.0, 0.0

    return kp, kd, position, velocity, torque

def sendMITCommand(
    bus,
    motor_id: int,
    kp: float,
    kd: float,
    position: float,
    velocity: float,
    torque: float,
    motor_type: str = MOTOR_TYPE,
):
    if not 0 <= motor_id < 0x100:
        raise ValueError("motor_idは0x00から0xFFの範囲で指定してください")

    arbitration_id = 0x100 * MIT_CONTROL_MODE + motor_id
    data = createMITData(kp, kd, position, velocity, torque, motor_type)
    send2Motor(bus, arbitration_id, data)

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
        print("※終了したい場合はCtrl+Cなどで強制終了すること。")

        while True:

            motor_id = inputId()
            kp, kd, position, velocity, torque = inputMITParameters()

            sendMITCommand(
                bus0,
                motor_id,
                kp,
                kd,
                position,
                velocity,
                torque,
            )
            time.sleep(1)

    finally:
        print("受信スレッドを停止しています...")
        stop_event.set()           # スレッド内の while ループを終わらせる合図を送る
        rx_thread.join(timeout=1.0)

        # 例外が発生しても、通信が終わったら必ずここを通ってクローズする
        bus0.shutdown()
        print("CANバスを切断（手動クローズ）しました")

# マイナスの数値も正しく判定できる関数を追加
def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False


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

# モーターのIDを読み取る関数
def inputId():
    # 使用する変数
    motorId = 0

    # 操作するモーターの指定
    while True:
        ans = input("操作するモーターのID：")

        if is_int(ans):  # isdigit から変更
            motorId = int(ans, 16)
            if 0 <= motorId < 256:
                break

        if ans == "h":
            print("各モーターに貼り付けてある値を入力します。")
        else:
            print("エラー：対応する数値以外が入力されました。")

    return motorId

if __name__ == "__main__":
    main()

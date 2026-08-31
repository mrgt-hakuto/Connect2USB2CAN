# -*- coding: utf-8 -*-
"""
非常停止用: Duty=0 と 電流=0 を送ってモーターを脱力させる。
使い方:
    python stop_motor.py        -> ID 1〜127 の全部へ送る(相手が分からない時)
    python stop_motor.py 42     -> ID 42 だけへ送る
※脱力なので、脚が吊られていないと自重で落ちます。手で支えるか吊ってから。
"""
import can
import sys
import time

CHANNEL = 0
BITRATE = 1000000
ZERO = [0x00, 0x00, 0x00, 0x00]

MODE_DUTY = 0
MODE_CURRENT = 1

def main():
    if len(sys.argv) >= 2:
        ids = [int(sys.argv[1])]
    else:
        ids = list(range(1, 128))

    try:
        bus = can.Bus(interface="gs_usb", channel=CHANNEL, bitrate=BITRATE)
    except Exception as e:
        print("CANバスを開けません:", repr(e))
        sys.exit(1)

    print("Duty=0 / 電流=0 を送ります。対象ID数:", len(ids))
    try:
        for _ in range(3):
            for mid in ids:
                for mode in (MODE_DUTY, MODE_CURRENT):
                    arb = (mode << 8) | mid
                    try:
                        bus.send(can.Message(arbitration_id=arb, data=ZERO,
                                             is_extended_id=True))
                    except can.CanError:
                        pass
            time.sleep(0.1)
        print("送信完了。止まらない場合は電源を落としてください。")
    finally:
        bus.shutdown()
        print("CANバスを閉じました。")

if __name__ == "__main__":
    main()

# -*- coding: utf-8 -*-
"""
段階1: 受信専用。CANへ1バイトも送信しない。
モーターの電源を入れ、CAN配線をつないだ状態で実行する。
モーターは回らない(送信しないため)。Ctrl+C で途中終了できる。
実行: python listen_only.py
"""
import can
import sys
import time

SECONDS = 15          # 監視する秒数
CHANNEL = 0
BITRATE = 1000000

def main():
    try:
        bus = can.Bus(interface="gs_usb", channel=CHANNEL, bitrate=BITRATE)
    except Exception as e:
        print("CANバスを開けません:", repr(e))
        print("-> 先に check_env.py を実行して原因を切り分けてください。")
        sys.exit(1)

    print("CANバスに接続しました。これから %d 秒間、受信だけします。" % SECONDS)
    print("(送信は一切しないので、モーターは動きません)")
    print("-" * 60)

    count = 0
    ids = {}
    t0 = time.time()
    try:
        while time.time() - t0 < SECONDS:
            msg = bus.recv(timeout=0.2)
            if msg is None:
                continue
            count += 1
            ids[msg.arbitration_id] = ids.get(msg.arbitration_id, 0) + 1
            if count <= 60:      # 出しすぎ防止。最初の60本だけ表示
                print("[%04d] ID=0x%08X ext=%s DLC=%d data=%s"
                      % (count, msg.arbitration_id, msg.is_extended_id,
                         msg.dlc, msg.data.hex(" ")))
    except KeyboardInterrupt:
        print("\nCtrl+C で中断しました。")
    finally:
        bus.shutdown()

    print("-" * 60)
    print("受信フレーム数:", count)
    if count == 0:
        print("1本も受信しませんでした。考えられる原因:")
        print("  1. モーターの電源が入っていない")
        print("  2. V3ファームの『Send status over CAN(定期フィードバック)』が無効")
        print("     -> 上位機ソフト(R-Link等)で有効化する")
        print("  3. CANH/CANL の結線ミス、終端抵抗120Ω不足")
        print("  4. ビットレート不一致(このスクリプトは 1 Mbps)")
    else:
        print("送信元IDの内訳(このIDが実機のモーターIDの手がかり):")
        for k in sorted(ids):
            print("  ID=0x%08X (下位1バイト=%d) : %d本" % (k, k & 0xFF, ids[k]))
        print()
        print(">>> ここでIDとバイト並びが取れれば、B10の『生ログで実測』が達成です。")

if __name__ == "__main__":
    main()

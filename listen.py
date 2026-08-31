"""
can_listen.py -- 受信専用。バス上に流れている全フレームをそのまま表示する。
モーターには何も送らないので、脚が動くことはない（安全）。

目的: モーターが定期フィードバックを出しているかを確認し、
      出ていれば「実際のCAN ID」と「実際のデータ並び」を実測する。
"""

import can
import time

CHANNEL = 0
BITRATE = 1000000      # 1 Mbps。上位機ソフト側の設定と一致していること
LISTEN_SEC = 10.0      # 待ち受け秒数


def main():
    try:
        bus = can.Bus(interface="gs_usb", channel=CHANNEL, bitrate=BITRATE)
    except Exception as e:
        print(f"CAN接続に失敗: {e}")
        return

    print(f"CAN接続 OK (ch={CHANNEL}, {BITRATE} bps)")
    print(f"{LISTEN_SEC:.0f} 秒間、バス上の全フレームを表示します。Ctrl+C で中断。")
    print("-" * 70)

    t0 = time.time()
    count = 0
    ids = {}

    try:
        while time.time() - t0 < LISTEN_SEC:
            msg = bus.recv(timeout=0.5)
            if msg is None:
                continue
            count += 1
            ids[msg.arbitration_id] = ids.get(msg.arbitration_id, 0) + 1
            kind = "拡張29bit" if msg.is_extended_id else "標準11bit"
            hexdata = " ".join(f"{b:02X}" for b in msg.data)
            print(f"[{count:4d}] ID=0x{msg.arbitration_id:08X} ({kind}) "
                  f"DLC={msg.dlc} DATA={hexdata}")
    except KeyboardInterrupt:
        print("\n中断しました")
    finally:
        bus.shutdown()

    print("-" * 70)
    print(f"受信フレーム数: {count}")
    if ids:
        print("見えた CAN ID 一覧:")
        for i, c in sorted(ids.items()):
            print(f"  0x{i:08X}  (下位8bit = {i & 0xFF} = 0x{i & 0xFF:02X})  {c} 回")
    else:
        print("→ 1フレームも来ていません。次を疑ってください:")
        print("   1) モーターの主電源が入っていない")
        print("   2) CAN_H / CAN_L の配線ミス・逆挿し、GND未接続")
        print("   3) 終端抵抗 120Ω が入っていない")
        print("   4) ビットレート不一致（モーター側が 500k などになっている）")
        print("   5) 定期フィードバックが無効（上位機ソフトで有効化する）")


if __name__ == "__main__":
    main()
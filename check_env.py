# -*- coding: utf-8 -*-
"""
段階0: 環境チェック。CANへ1バイトも送信しない。
モーターの電源は切ったままで実行してよい。
実行: python check_env.py
"""
import sys

print("=" * 60)
print("Python:", sys.version.split()[0], sys.executable)

# --- 1) python-can -------------------------------------------------
try:
    import can
    print("[OK] python-can", can.__version__)
except Exception as e:
    print("[NG] python-can が import できません:", e)
    sys.exit(1)

# --- 2) pyusb / libusb ---------------------------------------------
try:
    import usb.core
    import usb.backend.libusb1
    print("[OK] pyusb import 成功")
except Exception as e:
    print("[NG] pyusb が import できません:", e)
    print("     -> pip install pyusb")
    sys.exit(1)

backend = usb.backend.libusb1.get_backend()
if backend is None:
    print("[NG] libusb-1.0.dll が見つかりません")
    print("     -> README 2.1 の通り C:\\Windows\\System32 へコピー")
    sys.exit(1)
print("[OK] libusb バックエンド有効")

# --- 3) USBデバイス一覧 --------------------------------------------
print("-" * 60)
print("PCが認識しているUSBデバイス:")
candidates = []
for d in usb.core.find(find_all=True):
    try:
        name = d.product
    except Exception as e:
        name = "(名前取得不可: %s)" % type(e).__name__
    line = "  VID=0x%04X PID=0x%04X  %s" % (d.idVendor, d.idProduct, name)
    print(line)
    if d.idVendor in (0x1D50, 0x16D0, 0x1209, 0x0483):
        candidates.append(line)
print("-" * 60)
if candidates:
    print("gs_usb系(candleLight/USB2CAN)らしきデバイス:")
    for c in candidates:
        print(c)
else:
    print("gs_usb系らしきデバイスが見当たりません。")
    print(" -> アダプタが挿さっていない / Zadig でドライバを差し替えていない、のどちらか")

# --- 4) gs_usb ライブラリからの見え方 ------------------------------
print("-" * 60)
try:
    from gs_usb.gs_usb import GsUsb
    devs = GsUsb.scan()
    print("GsUsb.scan() の検出数:", len(devs))
    for i, dv in enumerate(devs):
        print("  index=%d: %r" % (i, dv))
    if len(devs) == 0:
        print(" -> ここが0なら python-can も繋がりません(ドライバ差し替えを疑う)")
except Exception as e:
    print("[NG] gs_usb スキャン失敗:", repr(e))

# --- 5) CANバスを開いて閉じるだけ(送信しない) ----------------------
print("-" * 60)
try:
    bus = can.Bus(interface="gs_usb", channel=0, bitrate=1000000)
    print("[OK] CANバスを開けました (channel=0, 1 Mbps)")
    bus.shutdown()
    print("[OK] CANバスを閉じました")
    print()
    print(">>> 段階0クリア。次は listen_only.py へ。")
except Exception as e:
    print("[NG] CANバスを開けません:", repr(e))
    print(" -> 上のGsUsb.scan()が0台なら原因はドライバ/接続。")
    print(" -> 1台以上見えているのにここで落ちるなら channel の番号を 0 以外も試す。")
print("=" * 60)

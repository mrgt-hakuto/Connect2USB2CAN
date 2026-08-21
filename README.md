# 1. python-canとWindows固有の依存関係をインストール
下記のコードをコンソールで実行する。

```
pip install -r requirement.txt
```

# 2. ドライバをインストール
## 2.1 libusb-1.0.dll
1. 「[https://github.com/libusb/libusb/releases](https://github.com/libusb/libusb/releases)」へアクセスし、「libusb-(任意の番号).7z」をダウンロード
2. 展開し、`./VS2019/MS64/dll/`の「libusb-1.0.dll」を、`C:\Windows\System32`へコピー

## 2.2 Zadig
1. 「[https://zadig.akeo.ie/](https://zadig.akeo.ie/)」へアクセスし、「Zadig 2.9」をダウンロード
2. ダウンロードしたexeファイルを実行し、メニューの「Options」>「List All Devices」にチェックを入れる。
3. ドロップダウンリストから「USB2CAN V3.3」と書かれているものを選択。
4. Driverの右側（書き換え後）を「WinUSB」に設定し、「Replace Driver」または「Install Driver」をクリックする。
5. Driverの右側（書き換え後）を「libusb-win32」・「libusbK」にそれぞれ設定し、再度4を実行。

# 3. 実行
## 3.1 サンプルプログラム
「sample.py」の`main`関数内を適切に書き換えたうえで、実行すればモーターが回転する。

## 3.2 コードの解説
1. `connect2USB2CAN`関数でUSB2CANへ接続する。引数は接続するCANの番号。
2. 1.の戻り値をbusとし、arbitration_id、dataを適切に設定の上、send2Morter関数を実行すると、モーターへ指定したデータを送信することができる。
3. 最後には必ずバスを閉じること。(例:`bus.shutdown()`)

# 4. 参考
- [WindowsでUSB2CANを使う方法(python-can)](https://zenn.dev/yama0804/articles/a96cf3f7aafc79)

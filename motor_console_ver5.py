"""
motor_console_ver5.py -- CubeMars 対話コンソール v5

cubemars.py を使う。全制御モードを物理単位で叩けて、指令は定周期で送られ、
必ず自動停止し、フィードバックは解釈された状態で表示される。
ver4 は原状のまま残してある（動くものを壊さないため）。

v4 → v5 の追加:
  ・mscheme … MIT の CAN ID 方式をその場で切り替える。cubemars.py を編集して
    再起動する必要がなくなった（std / ext / extid の3方式）
  ・mprobe  … 3方式を総当たりし、enable の前後で受信フレームを比べる。
    チェックリスト A8（MIT の CAN ID 方式）の判定材料を1コマンドで出す
  ・mrx     … MIT 応答の解釈を ON/OFF。v4 までは parse_mit_reply が受信ループから
    一度も呼ばれておらず、MIT が有効でも状態が更新されなかった
  ・hz      … 定期フィードバックのレートと帯域を実測（チェックリスト A1 / A11）
  ・x が MIT にも効くようになった。v4 まではサーボ用の速度0しか送っておらず、
    MIT モードのモーターには停止が効かなかった
  ・ヘルプを「実際に打てる数字入りの例」に書き換え（山括弧のプレースホルダを廃止）

⚠ 安全
  ・脚は外す/吊る/固定してから。電源をすぐ落とせる状態で。
  ・c（電流＝トルク指令）は位置/速度ループを通らないので無負荷で暴走します。
    実機の脚では ps（位置-速度）か p（位置）を使ってください。
  ・すべての指令は指定秒で自動停止します。Ctrl+C でも停止指令を出してから閉じます。
  ・x はソフト停止です。本当の非常停止は電源を切ること。
"""

import sys
import time

import cubemars as cm

# ------------------------------------------------------------
# 設定（実測で確定した値）
# ------------------------------------------------------------
CHANNEL = 1              # 実測: ch=0 は接続できるが通信できない
BITRATE = 1_000_000
DEFAULT_MOTOR_ID = 43    # 実測: フィードバック ID=0x292B の下位8bit
MODEL = "AK80-9"         # ID43 の実機は AK80-9。model コマンドで切り替えられる
                         # ERPM換算は両機種とも 31.5 で同じだが、MIT のトルクレンジが
                         # ±65（AK10-9）と ±18（AK80-9）で 3.6 倍違うので取り違えないこと


def set_model(name):
    """操作対象のモデルを実行時に切り替える"""
    global MODEL
    MODEL = name

SEND_HZ = 50

# 安全上限（超える指令は拒否する）
LIM_CURRENT_A = 3.0
LIM_ERPM      = 20000
LIM_DUTY      = 0.30
LIM_KP        = 50.0
LIM_KD        = 3.0
LIM_TORQUE    = 5.0

HELP = f"""
すべて「実際に打てる形」で書いてあります。そのままコピーして数字だけ変えてください。
角度は出力軸の deg、MIT だけ rad。時間の単位は秒。

状態を見る（軸は動かない）
  ?                     このヘルプをもう一度出す
  s                     今の状態を1行表示（位置・速度・電流・温度・エラー）
  m 3                   3秒間、指令せず状態だけ流し見る
  scan 2                バス上のモーターIDを2秒探す
  id 43                 操作対象のモーターIDを 43 にする
  model AK80-9          操作対象の機種を切り替え（MITのトルクレンジが変わる）
  hz 5                  定期フィードバックのレートを5秒測る（→ 制御周期の上限が分かる）
  raw on                受信フレームを生で表示（戻すのは raw off）
  log bench.csv         状態を bench.csv に記録（止めるのは log off）

サーボ指令（指定秒だけ {SEND_HZ}Hz で送り、終わると必ず停止する）
  p 90 2                90度へ動かして2秒保持（絶対位置。今の原点が基準）
  p 0 2                 0度へ戻す
  r -30 2               今の位置から -30度 動かして2秒
  ps 90 2000 5000 3     90度へ。最高2000ERPM・加速度5000ERPM/s で3秒
                        ★ 速度が制限されるので実機の脚ではこれを使う
  v 5000 2              5000ERPM（= 159 deg/s）で2秒回す
  vd 159 2              159 deg/s（= 5000ERPM）で2秒回す
  c 0.8 0.3             0.8A を0.3秒 ⚠位置/速度ループを通らないので無負荷では暴走する
                        （実測: 起動しきい値は約0.63A、c 0.8 を1秒で約3回転した）
  brake 0.5 0.5         ブレーキ電流 0.5A を0.5秒
  duty 0.05 0.3         デューティー比 0.05 を0.3秒
  o 0                   今いる位置を原点に定義し直す（0=一時 1=恒久）※軸は動かない

MITモード（CAN ID 方式は未確定。まず mprobe で当たりを付ける）
  mscheme               今の方式と、3方式それぞれの送信IDを表示（何も送らない）
  mscheme ext           方式を ext に切り替え（拡張29bit・ID=0x82B）
  mprobe 1              MITのCAN ID方式を総当たり（enable を送る。軸は動かない）
                        既定は std と extid のみ。ext はバスを落とすので除外
  mprobe 1 all          ext も含めて試す ⚠バスが落ちて再起動が要る
  mrx on                未知フレームをMIT応答として解釈（戻すのは mrx off）
  me                    enable（MIT で動かす前に必要）
  md                    disable
  mz                    今の位置をMITのゼロ点にする
  mit 1.57 0 20 1 0 2   1.57rad(=90度) へ Kp=20 Kd=1 トルク前置0 で2秒

その他
  send 32B 00 00 13 88  任意フレームを1回送る（この例はサーボ速度5000ERPM）
  x                     停止（速度0、MIT有効なら零トルク+disable も送る）
  q                     停止指令を出して終了

換算メモ
  1 deg/s = 31.5 ERPM     5000 ERPM = 159 deg/s = 26.5 rpm（出力軸）
  1.57 rad = 90 度        3.14 rad = 180 度
  位置は出力軸の積算角。±3276.7 deg で頭打ちするので、長く回す前に o で原点を戻す

安全上限（超える指令は拒否される）
  電流 {LIM_CURRENT_A} A / {LIM_ERPM} ERPM / デューティー {LIM_DUTY} /
  Kp {LIM_KP} / Kd {LIM_KD} / トルク {LIM_TORQUE} N·m
  ※ x はソフト停止です。本当の非常停止は電源を切ること。
"""


class Console:
    def __init__(self):
        self.bus = cm.MotorBus(channel=CHANNEL, bitrate=BITRATE, model=MODEL)
        self.target = DEFAULT_MOTOR_ID
        self.logfile = None
        self.logname = None
        self.current_cmd = ""

    # ---------- 表示 ----------
    def fmt(self, s):
        if s is None:
            return "  状態なし（フィードバック未受信）"
        warn = ""
        if getattr(self.bus, "rx_error", None):
            warn = f"  ⚠受信スレッド停止: {self.bus.rx_error}"
        if s.src == "mit":
            # ⚠ MIT応答は単位がサーボと違う。rad / rad/s / N*m。
            #   deg のつもりで読むと 1.57 を「1.6度しか動いていない」と
            #   誤読する（実際は90度）。
            return (f"  ID={s.id:3d}  pos={s.pos:8.4f} rad ({s.pos*57.2958:7.1f} deg)  "
                    f"spd={s.spd:7.2f} rad/s  "
                    f"T={s.cur:6.2f} N*m  temp={s.temp if s.temp is not None else -1:3d} C  "
                    f"err={s.err}  [MIT]  ({s.age()*1000:.0f}ms前){warn}")
        degs = cm.erpm_to_deg_s(s.spd, MODEL)
        return (f"  ID={s.id:3d}  pos={s.pos:8.1f} deg  "
                f"spd={s.spd:8.0f} ERPM ({degs:7.1f} deg/s)  "
                f"cur={s.cur:6.2f} A  temp={s.temp:3d} C  err={s.err}  [servo]  "
                f"({s.age()*1000:.0f}ms前){warn}")

    def show(self):
        print(self.fmt(self.bus.state(self.target)))

    def sample(self, elapsed):
        s = self.bus.state(self.target)
        print(self.fmt(s))
        self.write_log(s, elapsed)

    # ---------- CSV ----------
    def open_log(self, name):
        self.close_log()
        self.logfile = open(name, "w", encoding="utf-8")
        self.logfile.write("t,cmd,id,pos_deg,spd_erpm,spd_deg_s,cur_a,temp_c,err\n")
        self.logname = name
        print(f"  CSV記録を開始: {name}")

    def close_log(self):
        if self.logfile:
            self.logfile.close()
            print(f"  CSV記録を終了: {self.logname}")
        self.logfile = None
        self.logname = None

    def write_log(self, s, elapsed):
        if not self.logfile or s is None:
            return
        self.logfile.write(
            f"{elapsed:.4f},{self.current_cmd},{s.id},{s.pos:.1f},{s.spd:.0f},"
            f"{cm.erpm_to_deg_s(s.spd, MODEL):.2f},{s.cur:.2f},{s.temp},{s.err}\n")

    # ---------- 指令の実行 ----------
    def run(self, frame_fn, seconds, label, sample_hz=5):
        """frame_fn() が返すフレームを seconds 秒だけ送り続ける"""
        s0 = self.bus.state(self.target)
        pos0 = s0.pos if s0 else None
        self.current_cmd = label
        print(f"  {label} を {SEND_HZ}Hz で {seconds:.1f} 秒間 送ります")

        peak = {"spd": 0.0, "cur": 0.0}

        def on_sample(elapsed):
            self.sample(elapsed)

        def make(elapsed):
            s = self.bus.state(self.target)
            if s:
                peak["spd"] = max(peak["spd"], abs(s.spd))
                peak["cur"] = max(peak["cur"], abs(s.cur))
            return frame_fn()

        self.bus.hold(make, seconds, hz=SEND_HZ,
                      on_sample=on_sample,
                      sample_hz=(50 if self.logfile else sample_hz),
                      watchdog_ids=[self.target])

        time.sleep(0.2)
        s1 = self.bus.state(self.target)
        pos1 = s1.pos if s1 else None
        print("  --- 結果 ---")
        if pos0 is not None and pos1 is not None:
            print(f"  位置: {pos0:.1f} → {pos1:.1f} deg  (変化 {pos1 - pos0:+.1f} deg)")
        print(f"  速度の最大: {peak['spd']:.0f} ERPM "
              f"({cm.erpm_to_deg_s(peak['spd'], MODEL):.1f} deg/s) / "
              f"電流の最大: {peak['cur']:.2f} A")
        if peak["spd"] < 20:
            print("  ※ 軸がほぼ動いていない。電流指令なら摩擦に負けてトルク不足の可能性"
                  "（実測の起動しきい値は約 0.63 A）。")
        self.current_cmd = ""

    # ---------- メインループ ----------
    def loop(self):
        print(HELP)
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            p = line.split()
            c = p[0].lower()

            def num(i, default=0.0):
                try:
                    return float(p[i])
                except (IndexError, ValueError):
                    return default

            def integer(i, default=0):
                try:
                    return int(p[i])
                except (IndexError, ValueError):
                    return default

            mid = self.target

            try:
                if c == "q":
                    break

                elif c in ("?", "help", "h"):
                    print(HELP)

                elif c == "s":
                    self.show()

                elif c == "m":
                    t0 = time.time()
                    dur = num(1, 3.0)
                    while time.time() - t0 < dur:
                        self.show()
                        time.sleep(0.2)

                elif c == "scan":
                    ids = self.bus.scan(num(1, 2.0))
                    if ids:
                        print(f"  見つかったモーターID: {ids}")
                        for i in ids:
                            print(self.fmt(self.bus.state(i)))
                    else:
                        print("  1台も見つかりません。電源・配線・定期フィードバック設定を確認。")

                elif c == "id":
                    self.target = integer(1, self.target)
                    print(f"  操作対象を ID={self.target} にしました")

                elif c == "model":
                    if len(p) > 1:
                        name = p[1]
                        if name not in cm.MODELS:
                            print(f"  未知のモデル {name}。"
                                  f"選べるのは {list(cm.MODELS)}")
                            continue
                        set_model(name)
                        self.bus.model = name
                    print(f"  モデル: {MODEL}  "
                          f"(1 deg/s = {cm.erpm_per_deg_s(MODEL):.1f} ERPM, "
                          f"MITトルクレンジ ±{cm.MODELS[MODEL]['t_lim']} N·m)")

                elif c == "raw":
                    self.bus.raw_log = (len(p) > 1 and p[1].lower() == "on")
                    print(f"  生ログ: {'ON' if self.bus.raw_log else 'OFF'}")

                elif c == "log":
                    if len(p) > 1 and p[1].lower() != "off":
                        self.open_log(p[1])
                    else:
                        self.close_log()

                # ---- 位置系 ----
                elif c == "p":
                    d = num(1)
                    self.run(lambda: cm.f_position(mid, d), num(2, 2.0),
                             f"位置 {d:.1f} deg")

                elif c == "r":
                    s = self.bus.state(mid)
                    if s is None:
                        print("  状態が取れていないので相対移動できません")
                        continue
                    tgt = s.pos + num(1)
                    self.run(lambda: cm.f_position(mid, tgt), num(2, 2.0),
                             f"相対 {num(1):+.1f} deg → {tgt:.1f} deg")

                elif c == "ps":
                    d, e, a = num(1), num(2, 5000), num(3, 20000)
                    if abs(e) > LIM_ERPM:
                        print(f"  拒否: {LIM_ERPM} ERPM 超")
                        continue
                    self.run(lambda: cm.f_pos_spd(mid, d, e, a), num(4, 3.0),
                             f"位置-速度 {d:.1f} deg (最高{e:.0f} ERPM, 加速{a:.0f})")

                # ---- 速度系 ----
                elif c == "v":
                    e = num(1)
                    if abs(e) > LIM_ERPM:
                        print(f"  拒否: {LIM_ERPM} ERPM 超")
                        continue
                    self.run(lambda: cm.f_velocity(mid, e), num(2, 2.0),
                             f"速度 {e:.0f} ERPM ({cm.erpm_to_deg_s(e, MODEL):.1f} deg/s)")

                elif c == "vd":
                    ds = num(1)
                    e = cm.deg_s_to_erpm(ds, MODEL)
                    if abs(e) > LIM_ERPM:
                        print(f"  拒否: {LIM_ERPM} ERPM 超（{ds} deg/s = {e:.0f} ERPM）")
                        continue
                    self.run(lambda: cm.f_velocity(mid, e), num(2, 2.0),
                             f"速度 {ds:.1f} deg/s ({e:.0f} ERPM)")

                # ---- トルク系 ----
                elif c == "c":
                    a = num(1)
                    if abs(a) > LIM_CURRENT_A:
                        print(f"  拒否: {LIM_CURRENT_A} A 超")
                        continue
                    print("  ⚠ 電流(トルク)指令は位置/速度ループを通りません。無負荷では加速し続けます。")
                    self.run(lambda: cm.f_current(mid, a), num(2, 0.3), f"電流 {a:.3f} A")

                elif c == "brake":
                    a = num(1)
                    if abs(a) > LIM_CURRENT_A:
                        print(f"  拒否: {LIM_CURRENT_A} A 超")
                        continue
                    self.run(lambda: cm.f_current_brake(mid, a), num(2, 0.5),
                             f"ブレーキ電流 {a:.2f} A")

                elif c == "duty":
                    d = num(1)
                    if abs(d) > LIM_DUTY:
                        print(f"  拒否: デューティー比 {LIM_DUTY} 超")
                        continue
                    self.run(lambda: cm.f_duty(mid, d), num(2, 0.3), f"デューティー {d:.3f}")

                # ---- 原点 ----
                elif c == "o":
                    kind = integer(1, 0)
                    self.bus.send(cm.f_set_origin(mid, kind), quiet=False)
                    time.sleep(0.3)
                    self.show()
                    print("  ※ o は『今いる位置を0と定義し直す』コマンドです。軸は動きません。")
                    print("     物理的に原点へ戻すなら p 0 を使ってください。")

                # ---- MIT ----
                elif c == "mscheme":
                    if len(p) > 1:
                        try:
                            cm.set_mit_scheme(p[1].lower())
                        except ValueError as e:
                            print(f"  {e}")
                            continue
                        print(f"  MIT の CAN ID 方式を {cm.MIT_SCHEME} にしました: "
                              f"{cm.mit_scheme_desc()}")
                    else:
                        print(f"  現在: {cm.MIT_SCHEME} = {cm.mit_scheme_desc()}")
                    for k in cm.MIT_SCHEMES:
                        mark = "→" if k == cm.MIT_SCHEME else "  "
                        print(f"   {mark} {k:6s} 送信ID 0x{cm.mit_arbitration_id(mid, k):X}"
                              f"  {cm.MIT_SCHEMES[k][2]}")

                elif c == "mprobe":
                    sec = num(1, 1.0)
                    want_all = any(a.lower() == "all" for a in p[1:])
                    schemes = (list(cm.MIT_SCHEMES) if want_all
                               else list(cm.PROBE_SCHEMES_DEFAULT))
                    print(f"  MIT の CAN ID 方式を総当たりします"
                          f"（{len(schemes)}方式 × 約{sec * 2 + 0.4:.1f}秒）: "
                          f"{', '.join(schemes)}")
                    if want_all:
                        print("  ⚠ ext を含めています。送信するとCANバスが落ちて"
                              "再起動が必要になります。")
                    else:
                        print("  ※ ext は除外しています"
                              "（送信するとバスが落ちると実測で判明）。"
                              "含めるなら mprobe 1 all")
                    print("  ⚠ enable フレームを送ります。軸は動きませんが、"
                          "念のため脚は外すか固定した状態で実行してください。")
                    res = self.bus.probe_mit(mid, schemes=schemes, settle=sec)
                    print("\n  --- 結果 ---")
                    hit = False
                    for r in res:
                        print(f"  [{r['scheme']}] 送信 {r['frame']}")
                        print(f"      サーボ形式(0x29xx)の受信: "
                              f"{r['servo_before']} → {r['servo_after']} フレーム")
                        if r.get("echoes"):
                            print(f"      （自分の送信エコー {r['echoes']} 件を除外済み）")
                        if r["new_ids"]:
                            hit = True
                            print("      ★ 新しい CAN ID が出現: "
                                  + ", ".join(f"0x{a:08X}" for a in r["new_ids"]))
                            for a in r["new_ids"]:
                                e = r["after"][a]
                                print(f"         0x{a:08X} DLC={e['dlc']} "
                                      f"data={e['last'].hex(' ')}")
                        else:
                            print("      新しい CAN ID なし")
                        if r["servo_before"] and r["servo_after"] == 0:
                            if r["scheme"] == "ext":
                                print("      ▲ サーボ形式が止まったが、これは"
                                      "MIT有効化ではなくバスが落ちた結果"
                                      "（エコー多数＝ACKなしで再送）")
                            else:
                                hit = True
                                print("      ★ サーボ形式の定期フィードバックが"
                                      "止まった")
                    print("\n  判定の目安:")
                    print("   ・MIT に切り替わったなら、サーボ形式の受信が止まるか"
                          "新しい CAN ID が現れるはず")
                    print("   ・自分が送ったフレームのエコーは除外している。"
                          "2026-09-07 にこれで誤判定した実績があるため")
                    if hit:
                        print("   ・★ の付いた方式が本命。mscheme で選んでから me → mrx on "
                              "→ raw on で中身を確認してください")
                    else:
                        print("   ・どれも無反応。3方式とも効いていないので、上位機ソフト側で"
                              "MIT モードに設定する必要がある可能性が高いです")

                elif c == "mrx":
                    self.bus.mit_rx = (len(p) > 1 and p[1].lower() == "on")
                    print(f"  MIT応答の解釈: {'ON' if self.bus.mit_rx else 'OFF'}")
                    if self.bus.mit_rx:
                        print("  ※ サーボ形式でない8バイトフレームを MIT 応答とみなします。"
                              "誤解釈しうるので調査中だけ ON にしてください。")

                elif c == "me":
                    self.bus.mit_enable(mid, quiet=False)
                    print(f"  方式 {cm.MIT_SCHEME} ({cm.mit_scheme_desc()}) で送信しました")
                    print("  ※ 反応が無ければ mscheme で方式を変えるか mprobe を実行")
                elif c == "md":
                    self.bus.mit_disable(mid, quiet=False)
                elif c == "mz":
                    self.bus.send(cm.f_mit_set_zero(mid), quiet=False)
                elif c == "mit":
                    pos, vel, kp, kd, tau = num(1), num(2), num(3), num(4), num(5)
                    if kp > LIM_KP or kd > LIM_KD or abs(tau) > LIM_TORQUE:
                        print(f"  拒否: Kp≤{LIM_KP} Kd≤{LIM_KD} |トルク|≤{LIM_TORQUE} にしてください")
                        continue
                    self.run(lambda: cm.f_mit(mid, pos, vel, kp, kd, tau, MODEL),
                             num(6, 2.0),
                             f"MIT pos={pos:.3f}rad vel={vel:.2f} Kp={kp} Kd={kd} tau={tau}")

                # ---- 生フレーム ----
                elif c == "send":
                    if len(p) < 2:
                        print("  使い方: send 32B 00 00 13 88")
                        continue
                    arb = int(p[1], 16)
                    data = bytes(int(x, 16) for x in p[2:])
                    self.bus.send(cm.Frame(arb, data, True), quiet=False)

                elif c == "hz":
                    sec = num(1, 2.0)
                    print(f"  {sec:.1f} 秒間、受信フレームを数えます…")
                    rows = self.bus.feedback_hz(seconds=sec)
                    if not rows:
                        print("  1フレームも受信していません。"
                              "電源・配線・定期フィードバック設定を確認してください。")
                        continue
                    for arb, m_id, n, hz, dlc, ext, last in rows:
                        print(f"  ID=0x{arb:08X} (モーターID={m_id})  {n:5d}フレーム  "
                              f"{hz:6.1f} Hz  DLC={dlc}  "
                              f"{'拡張' if ext else '標準'}  最後={last.hex(' ')}")
                    total = sum(r[3] for r in rows)
                    print(f"  合計 {total:.1f} フレーム/秒")
                    print(f"  ※ 制御周期はこのレートを超えても意味が薄い。"
                          f"10モーターなら単純計算で {total * 10:.0f} フレーム/秒 "
                          f"（1Mbps・拡張ID8バイトで約 {total * 10 * 128 / 10000:.1f}% の帯域）")

                elif c == "x":
                    had_mit = sorted(self.bus.mit_enabled)
                    ids = self.bus.stop_all()
                    msg = f"  停止: {ids} へ速度0"
                    if had_mit:
                        msg += f" / {had_mit} へ零トルクMIT + disable"
                    print(msg)
                    print("  ※ これはソフト停止です。本当の非常停止は電源を切ること。")

                else:
                    print("  不明なコマンド。? でヘルプ")

            except Exception as e:
                print(f"  コマンド実行エラー: {e}")


def main():
    con = Console()
    try:
        con.bus.open()
    except Exception as e:
        print(f"CAN接続に失敗: {e}")
        sys.exit(1)

    print(f"CANバスに接続しました (ch={CHANNEL}, {BITRATE} bps, "
          f"操作対象 ID={con.target}, モデル={MODEL})")
    print(f"換算: 1 deg/s = {cm.erpm_per_deg_s(MODEL):.1f} ERPM")
    time.sleep(0.4)
    con.show()

    try:
        con.loop()
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        con.close_log()
        print("停止指令を送信して切断します...")
        con.bus.close()
        print("CANバスを切断しました")


if __name__ == "__main__":
    main()
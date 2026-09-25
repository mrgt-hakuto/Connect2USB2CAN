"""
motor_console_ver8.py -- MIT（モード8）でデプロイに要る数字を取りに行く実験コンソール

★ ver8 の変更（2026-09-16 夜、ID34 実機の生ログから）
  フィードバックの電流は「大きさ」は正しいが「符号」が信用できない。
    t +0.20 → +0.35 A（安定）/ t −0.20 → +0.35 A（符号が逆のまま安定）/ t −0.10 → +0.17 と −0.17 がサンプルごとに入れ替わる
  （logs/v7_20260916/193229_*, 193226_*）。ver7 の morg・kpscale・vscale は電流の符号で直線を当てていたので全部不成立だった。
  → ver8 は電流を |I| だけで扱う。原点は「V 字の底」を探す。倍率は ± を対にして大きさの比を取る。
  同じログから: 零指令付近で X＝サーボ角 のとき |I|≈0 ＝ この電源投入では MIT位置 ≈ サーボ角（offset≈0）の見込み。
  wdog: 送信を止めても 3 秒間トルクを出し続けた（2回）。lat: 最小 8 ms。

以下は ver7 の説明:

起動（実機）:  cd C:\\Users\\harut\\Connect2USB2CAN  →  python motor_console_ver8.py
起動（練習）:  python motor_console_ver8.py --sim     ← CANに触らない。にせモーター（mit_sim.py）

ver6 からの変更点
  1. 全コマンドの送受信を CSV に自動保存（logs/v7_日付/）。index.csv に1行ずつ索引。
  2. morg を作り直し。①Kp=1LSB の単点プローブで大まかな原点へ寄せる →
     ②Kp=0.24 の5点直線 → ③Kp=0.98 の5点直線 → ④保持で残差確認。
     電流の符号（+トルクで+電流か）を最初に実測してから寄せる。
     結果は mit_calib.json に保存（電源を入れ直すと無効かもしれないので「未検証」扱いで読む）。
  3. 新しい実験（すべて「指令トルク単位」の比較なので Kt が未決でも結論が出る）
       vscale   Kp=0。実効Kd と「ファームの速度レンジ」を分けて測る（原点不要）
       kpscale  実効Kp / 指令Kp を、同じ大きさのトルク指令との電流比で測る（原点必要）
       wdog     指令を止めたらモーターが何秒で脱力するか（A4）。ケーブルを抜かずに測る
       lat      指令→フィードバックの遅れ（D2）
       jit      このPCで 50Hz の送信周期がどれだけ揺れるか（D1）
       ocheck   サーボの原点設定（o 0）で MIT の原点も動くか
       ofit     何か所かで morg した結果から「ただのずれ」か「倍率つき」かを判定
       scan     バス上のモーターIDを全部列挙（B1）
  4. フィードバックの err（マニュアル 4.3.1: 1過温度 2過電流 3過電圧 4低電圧 5エンコーダ 6MOSFET過温度 7ロック）
     が 0 以外になったら即中断。
  5. Windows のタイマー分解能を 1ms に上げる（timeBeginPeriod）。既定の 15.6ms だと 50Hz が揺れる。

公式資料から分かったこと（2026-09-16 調査。詳細は pbl プロジェクトの motor_mit_official_notes.md）
  ・MIT = 制御モードID 8、拡張フレーム、Kp→Kd→位置→速度→トルク（V3.0.0 マニュアル 38頁の表）。
  ・AK 3.0 では「サーボと MIT の区別がなく、CAN を定期フィードバックか問い合わせ応答かに設定する」（公式FAQ）。
  ・定期フィードバックの周期は 1〜500Hz で設定可能（マニュアル）＝50Hz は工場設定であって上限ではない。
    上位機（CubeMars Tool / R-Link）の「アプリケーション機能」に CAN ID・通信レート・
    「CAN 通信途絶時の設定」がある＝A4 は測るだけでなく設定できる可能性が高い。
  ・AK80-9 V3.0 のエンコーダは内側（ロータ側）の磁気式 16bit 1個。公式FAQ「原点を電源断後も
    覚えさせるにはデュアルエンコーダ品が要る」。→ 恒久原点（A3）が3回とも失敗したのと整合。
    9:1 なので電源投入時の出力軸角は 360/9 = 40 度ごとにしか決まらない可能性がある。
  ・速度レンジは資料の版で違う: V1.0.15（AK80-9 ±50）/ V3.0.0（±30）/ ver6 が採用した値（±65）。
    MIT_RANGES が実機のファームと違うと、送った速度がそのぶん縮んで読まれる → vscale で判定する。

安全
  ・全コマンドは指定秒で自動終了し、終了時に零指令（Kp=Kd=0, トルク0）を送る。
  ・自動中断: 観測が 0.3 秒途切れる / 電流 3A 超 / 速度 250 deg/s 超 / 移動量 / err≠0 / バス異常。
  ・位置指令（Kp>0）は、このセッションで morg か overify を通した原点がある時だけ。
  ・x はソフト停止。本当の非常停止は電源を切ること。
  ・脚に付いたモーターで morg / vscale をしない（出力軸が数十度〜数百度回る）。ベンチか、脚を外して。
"""

import json
import math
import os
import sys
import time

SIM_MODE = "--sim" in sys.argv
if SIM_MODE:
    sys.argv.remove("--sim")

import cubemars as cm  # noqa: E402

CHANNEL = 1
BITRATE = 1_000_000
DEFAULT_MOTOR_ID = 34
DEFAULT_MODEL = "AK80-9"
SEND_HZ = 50
MODE_MIT = 8

RAD2DEG = 57.29577951308232
DEG2RAD = 0.017453292519943295
ERPM_PER_DEG_S = 31.5      # AK80-9: ERPM = 出力rpm × 9 × 21（極対数21は未検証 B3。vscale で位置の差分と照合）

KT_CMD = 0.59              # 指令トルク[N*m] ÷ 電流[A]。ファームの内部定数の疑い（物理Ktは未決）

# ---- 安全上限 ----
LIM_TAU = 1.0
LIM_KP = 20.0
LIM_KD = 5.0
LIM_SEC = 15.0
LIM_STEP_DEG = 90.0
LIM_KDV = 0.6              # Kd × |速度指令| の上限 [N*m]（止まっている軸に一気にかかるトルク）

# ---- 自動中断 ----
MAX_AGE = 0.3
GRACE = 0.4
CUR_ABORT = 3.0
SPD_ABORT_DEG_S = 250.0
MOVE_ABORT_DEG = 360.0
POS_SAT_DEG = 3200.0       # フィードバック位置は int16/10 で ±3276.7 deg に張り付く
ECHO_RATIO_ABORT = 3.0
STATIC_ERPM = 200.0        # これより遅いサンプルを「止まっている」とみなす（6.3 deg/s）

ERR_NAMES = {0: "なし", 1: "モーター過温度", 2: "過電流", 3: "過電圧", 4: "低電圧",
             5: "エンコーダ異常", 6: "MOSFET過温度", 7: "モーターロック"}

# ============================================================
# MIT のレンジ（送る側の解釈）。ファームと食い違うと値が縮んで読まれる
# ============================================================
MIT_RANGES = {
    "AK80-9": dict(P=12.56, V=65.0, T=18.0, KP=500.0, KD=5.0),
    "AK10-9": dict(P=12.56, V=28.0, T=54.0, KP=500.0, KD=5.0),
    "AK60-6": dict(P=12.56, V=60.0, T=12.0, KP=500.0, KD=5.0),
}
RANGE_SOURCES = {
    "AK80-9": "速度 ±65（ver6 採用）/ ±30（公式 V3.0.0）/ ±50（公式 V1.0.15）。位置 ±12.56（V3.0.0）/ ±12.5（V1.0.15）",
    "AK10-9": "速度 ±28・トルク ±54（V3.0.0）/ ±50・±65（V1.0.15）",
}

CALIB_FILE = "mit_calib.json"


def f2u(x, lo, hi, bits):
    return cm.float_to_uint(x, lo, hi, bits)


def u2f(v, lo, hi, bits):
    return cm.uint_to_float(v, lo, hi, bits)


def q12(x, hi, lo=None):
    """12bit に丸めた後、モーターが読む値（こちらのレンジ解釈で）"""
    lo = -hi if lo is None else lo
    return u2f(f2u(x, lo, hi, 12), lo, hi, 12)


_DITHER = [0]


def pack_mit(kp, kd, pos, vel, tau, model, dither=True):
    """
    ★ 速度0・トルク0 の欄は 12bit の真ん中（2047.5）に乗らず 2047 に切り捨てられ、半LSB 負に読まれる。
      AK80-9（±65）だと速度 −0.016 rad/s → Kd=1 で常に −0.016 N*m のトルクが乗る。
      これが morg の小さい Kp（0.24）では 3〜4 deg の原点ずれになる（シムで確認）。
      そこで 0 の欄だけ 2047 / 2048 を1フレームごとに交互に送り、平均を真ん中にする（dither）。
    """
    r = MIT_RANGES[model]
    kp_i = f2u(kp, 0.0, r["KP"], 12)
    kd_i = f2u(kd, 0.0, r["KD"], 12)
    p_i = f2u(pos, -r["P"], r["P"], 16)
    v_i = f2u(vel, -r["V"], r["V"], 12)
    t_i = f2u(tau, -r["T"], r["T"], 12)
    if dither:
        _DITHER[0] ^= 1
        if vel == 0 and v_i in (2047, 2048):
            v_i = 2047 + _DITHER[0]
        if tau == 0 and t_i in (2047, 2048):
            t_i = 2047 + _DITHER[0]
    return bytes([
        (kp_i >> 4) & 0xFF,
        ((kp_i & 0x0F) << 4) | ((kd_i >> 8) & 0x0F),
        kd_i & 0xFF,
        (p_i >> 8) & 0xFF,
        p_i & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F),
        t_i & 0xFF,
    ])


def unpack_mit(d, model):
    r = MIT_RANGES[model]
    kp_i = (d[0] << 4) | ((d[1] >> 4) & 0x0F)
    kd_i = ((d[1] & 0x0F) << 8) | d[2]
    p_i = (d[3] << 8) | d[4]
    v_i = (d[5] << 4) | ((d[6] >> 4) & 0x0F)
    t_i = ((d[6] & 0x0F) << 8) | d[7]
    return dict(kp=u2f(kp_i, 0.0, r["KP"], 12), kd=u2f(kd_i, 0.0, r["KD"], 12),
                pos=u2f(p_i, -r["P"], r["P"], 16), vel=u2f(v_i, -r["V"], r["V"], 12),
                tau=u2f(t_i, -r["T"], r["T"], 12))


def f_mit(mid, kp, kd, pos, vel, tau, model):
    return cm.Frame((MODE_MIT << 8) | (mid & 0xFF), pack_mit(kp, kd, pos, vel, tau, model), True)


def quantized_cmd(cmd, model):
    """(kp,kd,pos,vel,tau) → 実際にモーターへ届く値（こちらのレンジ解釈）"""
    b = unpack_mit(pack_mit(*cmd, model, dither=False), model)
    return (b["kp"], b["kd"], b["pos"], b["vel"], b["tau"])


# ============================================================
# 小道具
# ============================================================
def mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else float("nan")


def std(xs):
    xs = list(xs)
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return (sum((x - m) ** 2 for x in xs) / len(xs)) ** 0.5


def median(xs):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def linfit(xs, ys):
    """y = a x + b。戻り値 (a, b, R^2)。直線が引けなければ None"""
    n = len(xs)
    if n < 3:
        return None
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    my = sy / n
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return a, b, r2


def denoise(m, floor):
    """|I+雑音| の平均 m から |I| を推定する。floor は零指令時の |雑音| 平均（= σ√(2/π)）。
    単純に floor を引くと、信号が雑音より大きいとき 10〜20% 小さく出る（シムで確認）。"""
    sigma2 = (floor * floor) * math.pi / 2
    return math.sqrt(max(m * m - sigma2, 0.0))


def windows_timer_1ms():
    if sys.platform != "win32":
        return False
    try:
        import ctypes
        ctypes.windll.winmm.timeBeginPeriod(1)
        return True
    except Exception:
        return False


class Tee:
    def __init__(self, stream, fh):
        self.stream, self.fh = stream, fh

    def write(self, x):
        self.stream.write(x)
        try:
            self.fh.write(x)
            self.fh.flush()
        except Exception:
            pass
        return len(x)

    def flush(self):
        self.stream.flush()


class Aborted(RuntimeError):
    def __init__(self, msg, bus_trouble=False):
        super().__init__(msg)
        self.bus_trouble = bus_trouble


# ============================================================
# 1回ぶんの送受信の記録
# ============================================================
class Run:
    def __init__(self, label):
        self.label = label
        self.tx = []      # (t, kp, kd, pos, vel, tau)  ※量子化後の値
        self.rx = []      # (t, pos_deg, spd_erpm, |cur_a|, temp, err, cur_a 生値)  ※電流の符号は信用できない
        self.abort = None
        self.bus_trouble = False
        self.t_stop_send = None
        self.csv = None

    @property
    def ok(self):
        return self.abort is None

    def window(self, t0=0.0, t1=1e9, static=None):
        out = [r for r in self.rx if t0 <= r[0] < t1]
        if static is True:
            out = [r for r in out if abs(r[2]) < STATIC_ERPM]
        return out

    def moved_deg(self):
        if len(self.rx) < 2:
            return 0.0
        ps = [r[1] for r in self.rx]
        return max(ps) - min(ps)

    def jitter(self):
        ts = [r[0] for r in self.tx]
        if len(ts) < 5:
            return None
        d = [b - a for a, b in zip(ts, ts[1:])]
        return mean(d), std(d), max(d)


# ============================================================
# コンソール
# ============================================================
class Console:
    def __init__(self, bus):
        self.bus = bus
        self.target = DEFAULT_MOTOR_ID
        self.model = DEFAULT_MODEL
        self.csv_on = True
        self.log_dir = os.path.join("logs", "v7_" + time.strftime("%Y%m%d"))
        self.off = None            # MIT位置 = サーボ角[rad] + off
        self.off_verified = False
        self.i_sign = None         # +トルク指令で電流が + なら +1
        self.kp_ratio = None       # 電流/(指令Kp×ずれ) を 指令トルク単位に直した比（morg の傾きから）
        self.morg_hist = []        # (サーボ角rad, off, R2, 時刻)
        self.move_cap = None       # limit で設定。全実験の移動量中断をこの角度以下に抑える（脚付き用）
        self.calib = self._load_calib()
        self._pull_calib()

    # ---------- 保存 ----------
    def _load_calib(self):
        try:
            with open(CALIB_FILE, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_calib(self, **kw):
        key = str(self.target)
        e = self.calib.setdefault(key, {})
        e.update(kw)
        e["model"] = self.model
        e["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
        try:
            with open(CALIB_FILE, "w", encoding="utf-8") as f:
                json.dump(self.calib, f, ensure_ascii=False, indent=2)
        except Exception as ex:
            print(f"  ⚠ {CALIB_FILE} に保存できません: {ex}")

    def _pull_calib(self):
        e = self.calib.get(str(self.target), {})
        self.off = e.get("offset_rad")
        self.off_verified = False
        self.i_sign = e.get("i_sign")
        self.kp_ratio = e.get("kp_ratio_est")
        self.morg_hist = []
        if self.off is not None:
            print(f"  {CALIB_FILE} から ID={self.target} の原点 offset={self.off:+.4f} rad を読みました"
                  f"（{e.get('updated', '?')}）。")
            print("  ★ 未検証扱いです。電源を入れ直した・o を打った・手で回した後は外れている可能性があるので、"
                  "位置指令の前に overify を通してください。")

    # ---------- 状態 ----------
    def st(self):
        return self.bus.state(self.target)

    def theta(self, s=None):
        s = s or self.st()
        return None if s is None else s.pos * DEG2RAD

    def show(self, s=None):
        s = s or self.st()
        if s is None:
            print("  状態なし（フィードバック未受信）")
            return
        extra = ""
        if self.off is not None:
            extra = f"  MIT位置≈{(s.pos * DEG2RAD + self.off):+.3f} rad" + ("" if self.off_verified else "（未検証）")
        print(f"  ID={s.id:3d}  pos={s.pos:8.1f} deg  spd={s.spd:7.0f} ERPM ({s.spd / ERPM_PER_DEG_S:+6.1f} deg/s)  "
              f"cur={s.cur:+6.2f} A  temp={s.temp}C  err={s.err}({ERR_NAMES.get(s.err, '?')})  "
              f"[{s.src}] {s.age() * 1000:.0f}ms前{extra}")

    def need_feedback(self, wait_static=3.0):
        s = self.st()
        if s is not None and s.alive(0.5) and wait_static:
            # 軸がまだ回っている（前のコマンドの惰性）なら止まるまで待つ。回ったまま測ると全部ゴミになる
            t0, n, last = time.time(), 0, None
            while time.time() - t0 < wait_static:
                s2 = self.st()
                if s2 is not None and s2.t != last:
                    last = s2.t
                    n = n + 1 if abs(s2.spd) < 50 else 0
                    if n >= 8:
                        break
                time.sleep(0.005)
            else:
                print(f"  中止: 軸が {wait_static:.0f} 秒たっても止まりません（spd={self.st().spd:.0f} ERPM）。x を打ってから。")
                return None
            s = self.st()
        if s is None or not s.alive(0.5):
            print(f"  中止: ID={self.target} のフィードバックが来ていません（scan で ID を確認）。")
            return None
        if s.src != "servo":
            print(f"  中止: 応答形式が [{s.src}] です。ver7 はサーボ形式（deg/ERPM/A）の定期フィードバック前提です。")
            return None
        if abs(s.pos) > POS_SAT_DEG:
            print(f"  中止: 位置 {s.pos:.1f} deg が頭打ち（±3276.7）に近いです。o 0 で原点を戻してください。")
            return None
        if s.err:
            print(f"  中止: err={s.err}（{ERR_NAMES.get(s.err, '?')}）。電源を切って原因を確認してください。")
            return None
        return s

    def need_origin(self, verified=True):
        if self.off is None:
            print("  中止: MIT の原点がありません。先に morg を実行してください。")
            return False
        if verified and not self.off_verified:
            print("  中止: 原点が未検証です。overify を通してください（ダメなら morg）。")
            return False
        return True

    # ---------- 送受信の本体 ----------
    def run(self, label, make_cmd, seconds, move_abort=MOVE_ABORT_DEG, spd_abort=SPD_ABORT_DEG_S,
            cur_abort=CUR_ABORT, tick=0.0, quiet=False, kdv_limit=LIM_KDV, hz=SEND_HZ):
        """
        make_cmd(t) が (kp, kd, pos_rad_MIT, vel, tau) を返したら 50Hz で送る。None を返した周期は送らない
        （wdog の「送信停止後の観察」に使う）。受信は新しいフレームごとに全部記録する。
        """
        bus, mid, model = self.bus, self.target, self.model
        run = Run(label)
        if self.move_cap is not None:
            move_abort = min(move_abort, self.move_cap)
        period = 1.0 / hz
        if not quiet:
            print(f"  ▶ {label}（{seconds:.1f} 秒）")
        t0 = time.time()
        next_send = t0
        last_rx_t = None
        pos0 = None
        tx0, echo0 = bus.tx_count, bus.echo_count
        last_sent = None
        next_tick = 0.0
        try:
            while True:
                now = time.time()
                el = now - t0
                if el >= seconds:
                    break
                if now >= next_send:
                    cmd = make_cmd(el)
                    if cmd is not None:
                        kp, kd, pos, vel, tau = cmd
                        r = MIT_RANGES[model]
                        if kp > LIM_KP or kd > LIM_KD or abs(tau) > LIM_TAU or kd * abs(vel) > kdv_limit + 1e-9:
                            raise Aborted(f"指令が安全上限を超えました Kp={kp:.2f} Kd={kd:.2f} "
                                          f"tau={tau:+.2f} Kd*|v|={kd * abs(vel):.2f}")
                        if kp > 0 and abs(pos) > r["P"] - 0.05:
                            raise Aborted(f"MIT 目標位置 {pos:+.3f} rad がレンジ ±{r['P']} の端です（張り付くと過大トルク）")
                        bus.send(f_mit(mid, *cmd, model))
                        run.tx.append((el,) + quantized_cmd(cmd, model))
                        last_sent = now
                    elif run.t_stop_send is None and last_sent is not None:
                        run.t_stop_send = el
                    next_send += period
                    if next_send < now - period:
                        next_send = now + period
                s = bus.state(mid)
                if s is not None and s.t != last_rx_t:
                    last_rx_t = s.t
                    run.rx.append((s.t - t0, s.pos, s.spd, abs(s.cur), s.temp, s.err, s.cur))
                    if s.src != "servo":
                        raise Aborted(f"応答形式が [{s.src}] に変わりました（単位が違うので止めます）")
                    if pos0 is None:
                        pos0 = s.pos
                    if s.err:
                        raise Aborted(f"err={s.err}（{ERR_NAMES.get(s.err, '?')}）")
                    if abs(s.cur) > cur_abort:
                        raise Aborted(f"電流 {s.cur:+.2f} A（しきい値 {cur_abort} A）")
                    if abs(s.spd) / ERPM_PER_DEG_S > spd_abort:
                        raise Aborted(f"速度 {s.spd / ERPM_PER_DEG_S:+.0f} deg/s（しきい値 {spd_abort:.0f}）")
                    if abs(s.pos - pos0) > move_abort:
                        raise Aborted(f"開始位置から {s.pos - pos0:+.1f} deg 動きました（しきい値 {move_abort:.0f}）")
                    if abs(s.pos) > POS_SAT_DEG:
                        raise Aborted("位置が頭打ち付近です")
                    if tick and el >= next_tick:
                        print(f"    t={el:5.2f}  pos={s.pos:+8.2f} deg  spd={s.spd / ERPM_PER_DEG_S:+7.1f} deg/s  "
                              f"cur={s.cur:+6.2f} A")
                        next_tick = el + tick
                sending = last_sent is not None and now - last_sent < 0.1
                if sending and el > GRACE and (s is None or s.age() > MAX_AGE):
                    raise Aborted(f"フィードバックが {MAX_AGE} 秒以上途切れました", bus_trouble=True)
                sends = bus.tx_count - tx0
                if sends >= 10 and (bus.echo_count - echo0) / sends > ECHO_RATIO_ABORT:
                    raise Aborted("エコーが送信の3倍超（ACKなし再送＝bus-off の前兆）", bus_trouble=True)
                if bus.rx_error:
                    raise Aborted(f"受信スレッド停止: {bus.rx_error}", bus_trouble=True)
                time.sleep(0.001)
        except Aborted as e:
            run.abort = str(e)
            run.bus_trouble = e.bus_trouble
        except KeyboardInterrupt:
            run.abort = "Ctrl+C"
        finally:
            for _ in range(3):
                try:
                    bus.send(f_mit(mid, 0, 0, 0, 0, 0, model))
                except Exception:
                    break
                time.sleep(0.01)
        if run.abort and not quiet:
            print(f"  ■ 中断: {run.abort}")
            print("    零指令を送りました。" + ("電源OFF → USB2CAN 抜き差し → 電源ON が必要かもしれません。"
                                             if run.bus_trouble else "電源の入れ直しは不要です。"))
        elif run.abort:
            print(f"  ■ 中断: {run.abort}")
        j = run.jitter()
        if j and j[2] > 1.8 * period:
            print(f"  ⚠ 送信周期が揺れました: 平均 {j[0] * 1000:.1f} ms / 最大 {j[2] * 1000:.1f} ms")
        self._write_csv(run)
        return run

    def _write_csv(self, run):
        if not self.csv_on or (not run.tx and not run.rx):
            return
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            safe = "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in run.label)[:40]
            fn = os.path.join(self.log_dir, time.strftime("%H%M%S_") + f"id{self.target}_" + safe + ".csv")
            rows = [(r[0], "tx", "%.4f,%.4f,%.5f,%.4f,%.4f,,,,," % tuple(r[1:6])) for r in run.tx]
            rows += [(r[0], "rx", ",,,,,%.1f,%.0f,%.2f,%s,%s" % (r[1], r[2], r[6], r[4], r[5])) for r in run.rx]
            rows.sort(key=lambda x: x[0])
            with open(fn, "w", encoding="utf-8", newline="") as f:
                f.write("t,kind,kp,kd,pos_cmd_rad,vel_cmd,tau_cmd,pos_deg,spd_erpm,cur_a,temp,err\r\n")
                for t, k, body in rows:
                    f.write(f"{t:.4f},{k},{body}\r\n")
            new = not os.path.exists(os.path.join(self.log_dir, "index.csv"))
            with open(os.path.join(self.log_dir, "index.csv"), "a", encoding="utf-8", newline="") as f:
                if new:
                    f.write("time,file,id,model,label,offset_rad,abort\r\n")
                off = "" if self.off is None else f"{self.off:.5f}"
                f.write(f"{time.strftime('%H:%M:%S')},{os.path.basename(fn)},{self.target},{self.model},"
                        f"\"{run.label}\",{off},\"{run.abort or ''}\"\r\n")
            run.csv = fn
        except Exception as ex:
            print(f"  ⚠ CSV 保存失敗: {ex}")

    def zero(self):
        for _ in range(3):
            self.bus.send(f_mit(self.target, 0, 0, 0, 0, 0, self.model))
            time.sleep(0.01)

    # ============================================================
    # 見る
    # ============================================================
    def scan(self, sec=2.0):
        print(f"  {sec:.0f} 秒聞きます（何も送りません）")
        rows = self.bus.feedback_hz(seconds=sec)
        if not rows:
            print("  1フレームも受信していません。電源・CANH/L・終端・channel を確認。")
            return
        for arb, m_id, n, hz, dlc, ext, last in rows:
            s = self.bus.state(m_id)
            extra = f" pos={s.pos:+.1f}deg temp={s.temp}C err={s.err}" if s is not None else ""
            print(f"  CAN ID 0x{arb:X}  モーターID={m_id:3d}  {hz:5.1f} Hz  {'拡張' if ext else '標準'}{extra}")
        print("  ※ 機種（AK80-9 / AK10-9）はフィードバックから判別できません。銘板か配線で確認して model で指定。")

    def sniff(self, sec=3.0):
        """バス上の全フレームを CAN ID ごとに集計（何も送らない）。標準フレームも出す"""
        if not hasattr(self.bus, "capture"):
            print("  このバスでは使えません（--sim）")
            return
        print(f"  {sec:.0f} 秒聞きます（何も送りません）")
        cap = self.bus.capture(sec)
        if not cap:
            print("  1フレームも受信していません。")
            return
        for arb in sorted(cap):
            e = cap[arb]
            kind = "拡張" if e["ext"] else "標準"
            note = ""
            if e["ext"] and (arb >> 8) == 0x29:
                note = f"← ID{arb & 0xFF} のサーボ形式フィードバック"
            elif e["ext"]:
                note = f"← モード{arb >> 8} / ID{arb & 0xFF} 宛て？（誰かの送信）"
            else:
                note = "← 標準フレーム（旧 MIT 形式の応答なら data[0] がモーターID）"
            print(f"  0x{arb:08X} {kind} {e['n']:5d}件 {e['n'] / sec:6.1f}Hz DLC={e['dlc']} 最後={bytes(e['last']).hex(' ')}  {note}")

    def ping(self, ids):
        """
        零指令（Kp=Kd=0, 速度0, トルク0）を各IDへ 0.2 秒だけ送り、その間に返ってきたフレームを集計する。
        Kp/Kd が先頭の並びなので、零指令は位置・速度の欄に関係なく無害（ver6 で確認済み）。
        問い合わせ応答モードのモーターは、ここで初めて返事をするはず。
        """
        if not hasattr(self.bus, "capture"):
            print("  このバスでは使えません（--sim）")
            return
        import threading
        base = self.bus.capture(1.0)
        print(f"  送る前の1秒: {', '.join(f'0x{a:X}' for a in sorted(base)) or 'なし'}")
        for mid in ids:
            box = {}
            th = threading.Thread(target=lambda: box.update(cap=self.bus.capture(0.5)))
            th.start()
            time.sleep(0.05)
            for _ in range(10):
                self.bus.send(f_mit(mid, 0, 0, 0, 0, 0, self.model))
                time.sleep(0.02)
            th.join()
            cap = box.get("cap", {})
            newf = {a: e for a, e in cap.items() if a not in base}
            print(f"  ID{mid}（送信 0x{(MODE_MIT << 8) | mid:X} ×10）→ 新しく現れたフレーム: "
                  + (", ".join(f"0x{a:X}({'拡張' if e['ext'] else '標準'} {e['n']}件 {bytes(e['last']).hex(' ')})"
                               for a, e in sorted(newf.items())) or "なし"))
            time.sleep(0.2)
        print("  ※ 送る前から 0x29xx が流れている ID は「なし」が正常（定期フィードバックは既に見えている）。"
              "送る前にも後にも何も無い ID は、電源・配線・ID違いのどれか。")

    def jog(self, deg=10.0, erpm=400.0, accel=2000.0):
        """
        手で回せないモーターの「どれが動くか・どっち向きか」を確かめるための小さな相対移動。
        サーボの位置-速度モード（モード6。ver5 で安定動作を確認済み: 最大電流 1A 前後）で、
        今の角度から deg だけ、最高 erpm（400 ERPM ≒ 12.7 deg/s）でゆっくり動かす。原点は不要。
        終わったら電流0指令（モード1）と MIT 零指令を送って脱力させる。
        """
        if abs(deg) > 30 or erpm > 1000 or erpm <= 0:
            print("  拒否: |deg|≤30、ERPM は 1〜1000")
            return
        s = self.need_feedback()
        if not s:
            return
        mid = self.target
        p0 = s.pos
        tgt = p0 + deg
        sec = abs(deg) / (erpm / ERPM_PER_DEG_S) + 1.5
        print(f"  ★ ID{mid} を {p0:+.1f} → {tgt:+.1f} deg へゆっくり動かします（最高 {erpm / ERPM_PER_DEG_S:.0f} deg/s・{sec:.1f} 秒）。")
        t0 = time.time()
        nxt = t0
        last = None
        imax = 0.0
        abort = None
        try:
            while time.time() - t0 < sec:
                now = time.time()
                if now >= nxt:
                    self.bus.send(cm.f_pos_spd(mid, tgt, erpm, accel))
                    nxt += 1.0 / SEND_HZ
                st = self.st()
                if st is not None and st.t != last:
                    last = st.t
                    imax = max(imax, abs(st.cur))
                    if st.err:
                        abort = f"err={st.err}（{ERR_NAMES.get(st.err, '?')}）"
                        break
                    if abs(st.cur) > CUR_ABORT:
                        abort = f"電流 {st.cur:+.2f} A"
                        break
                    if abs(st.pos - p0) > abs(deg) + 10:
                        abort = f"{st.pos - p0:+.1f} deg 動きすぎ"
                        break
                if now - t0 > GRACE and (st is None or st.age() > MAX_AGE):
                    abort = "フィードバック途切れ"
                    break
                time.sleep(0.002)
        except KeyboardInterrupt:
            abort = "Ctrl+C"
        finally:
            for _ in range(3):
                self.bus.send(cm.f_current(mid, 0.0))
                time.sleep(0.01)
            self.zero()
        time.sleep(0.3)
        p1 = self.st().pos
        if abort:
            print(f"  ■ 中断: {abort}")
        print(f"  結果: {p0:+.1f} → {p1:+.1f} deg（変化 {p1 - p0:+.1f} / 指令 {deg:+.1f}）  最大電流 {imax:.2f} A")
        if abs(p1 - p0) < 1.0:
            print("  動いていません（最大電流が 0 付近なら指令が届いていない、大きければ引っかかっている）。")
        else:
            print("  ★ いま動いたモーターが ID{} です。指令＋で動いた向きをメモしてください。".format(mid))

    def jit(self, sec=5.0):
        """送らずに 50Hz の予定表だけ回して、このPCで出る周期の揺れを測る"""
        period = 1.0 / SEND_HZ
        ts = []
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < sec:
            now = time.time()
            if now >= nxt:
                ts.append(now)
                nxt += period
            time.sleep(0.001)
        d = [(b - a) * 1000 for a, b in zip(ts, ts[1:])]
        ds = sorted(d)
        sl = []
        for _ in range(200):
            a = time.perf_counter()
            time.sleep(0.001)
            sl.append((time.perf_counter() - a) * 1000)
        print(f"  50Hz 予定表: {len(d)} 周期  平均 {mean(d):.2f} ms  標準偏差 {std(d):.2f}  "
              f"p99 {ds[int(len(ds) * 0.99)]:.1f}  最大 {max(d):.1f} ms")
        print(f"  time.sleep(1ms) の実測: 中央値 {median(sl):.2f} ms / 最大 {max(sl):.2f} ms")
        print("  目安: 最大が 25 ms を超えるなら、10モーター＋推論を載せると 50Hz を割る危険。")

    # ============================================================
    # トルクだけ（Kp=0 なので原点不要）
    # ============================================================
    def cmd_t(self, tau, sec):
        if abs(tau) > LIM_TAU or sec > LIM_SEC:
            print(f"  拒否: |tau|≤{LIM_TAU} / 秒≤{LIM_SEC}")
            return
        if not self.need_feedback():
            return
        run = self.run(f"t {tau:+.2f}", lambda t: (0, 0, 0, 0, tau), sec, tick=0.2)
        w = run.window(min(0.3, sec / 2))
        if w:
            print(f"  電流 平均 {mean(r[3] for r in w):+.3f} A / 位置変化 {run.rx[-1][1] - run.rx[0][1]:+.1f} deg")

    def measure_isign(self, tau=0.12):
        """電流の大きさの比（指令トルク/|電流|）と、符号がどれだけ信用できないかを表示する"""
        out = {}
        for q in (tau, -tau):
            run = self.run(f"電流の大きさ t{q:+.2f}", lambda t, q=q: (0, 0, 0, 0, q), 0.6, move_abort=10, quiet=True)
            w = run.window(0.15, static=True)
            if not run.ok or len(w) < 5:
                print("  測れませんでした（動いた/中断）")
                return None
            raw = [r[6] for r in w]
            out[q] = (mean(r[3] for r in w), sum(1 for x in raw if x > 0) / len(raw))
            time.sleep(0.2)
        fl = self.noise_floor()
        for q, (i, pos_frac) in out.items():
            print(f"  {q:+.2f} N*m → |電流| {i:.3f} A（雑音補正後の比 {abs(q) / max(denoise(i, fl), 1e-3):.3f} N*m/A）"
                  f"  生値が＋だった割合 {pos_frac * 100:.0f}%")
        print("  ※ ＋指令で 100%、−指令で 0% なら符号は正しい。そうでなければ ver8 どおり大きさだけで扱う。")
        self.i_sign = 1
        return 1

    def noise_floor(self, sec=0.5):
        """零指令のときの |電流| の平均（ノイズの床）。大きさで比を取るときに引く"""
        run = self.run("床 零指令", lambda t: (0, 0, 0, 0, 0), sec, move_abort=5, quiet=True)
        w = run.window(0.15)
        self.i_floor = mean(r[3] for r in w) if w else 0.0
        return self.i_floor

    def brk(self, lo=0.20, hi=0.60, step=0.02, sec=1.0):
        if not self.need_feedback():
            return
        tau = lo
        still = None
        while tau <= hi + 1e-9 and tau <= LIM_TAU:
            run = self.run(f"brk {tau:.2f}", lambda t, q=tau: (0, 0, 0, 0, q), sec, move_abort=30, spd_abort=120,
                           quiet=True)
            moving = (not run.ok) or run.moved_deg() > 1.0
            w = run.window(0.3, static=True)
            i = f"{mean(r[3] for r in w):+.3f} A" if w else "-"
            print(f"    {tau:.2f} N*m → 電流 {i}  {'★動いた' if moving else '静止'}")
            if moving:
                print(f"  ★ 静止摩擦（指令トルク単位）は {still if still is not None else lo:.2f} 〜 {tau:.2f} N*m")
                self._save_calib(static_friction_cmd=[still, tau])
                return
            still = tau
            tau = round(tau + step, 4)
            time.sleep(0.3)
        print(f"  {hi:.2f} N*m まで動きませんでした")

    # ============================================================
    # vscale: 実効Kd と ファームの速度レンジ（原点不要）
    # ============================================================
    def vscale(self, v=2.5):
        """
        止まった軸に Kp=0・Kd・速度指令 V を送ると tau = c_d*Kd*(r_v*V - 0)。トルク指令と電流を比べると
        c_d*r_v が出る（静止テスト）。回した状態の定常速度 ω = r_v*V - F/Kd を Kd 3点で当てると r_v が出る。
          c_d = 実効Kd / 指令Kd、r_v = ファームが読む速度 / 送った速度（レンジが合っていれば 1）
        """
        if not self.need_feedback():
            return
        model = self.model
        V = MIT_RANGES[model]["V"]
        print("  ★ 出力軸が回ります（+方向に最大約300度 → −方向に戻る、を3回）。脚は外して、手を離して。")
        print(f"  いまの速度レンジ解釈: ±{V} rad/s（{RANGE_SOURCES.get(model, '')}）")
        # --- 1) 静止テスト（電流は大きさだけ。± を対にして平均）---
        fl = self.noise_floor()
        it, iv = [], []
        kd = 1.0
        for tau in (0.1, 0.2):   # 脚付きだと ±0.3 のトルク指令で動いた（2026-09-16）
            qt = abs(quantized_cmd((0, 0, 0, 0, tau), model)[4])
            vv = tau / kd
            for sg in (+1, -1):
                r = self.run(f"vscale 基準 t{sg * tau:+.2f}", lambda t, q=sg * tau: (0, 0, 0, 0, q), 0.6, move_abort=5,
                             quiet=True)
                w = r.window(0.25, static=True)
                if r.ok and len(w) >= 5 and r.moved_deg() < 1.0:
                    it.append((qt, denoise(mean(x[3] for x in w), fl)))
                qk, qv = quantized_cmd((0, kd, 0, sg * vv, 0), model)[1], quantized_cmd((0, kd, 0, sg * vv, 0), model)[3]
                r = self.run(f"vscale 静止 Kd{kd} v{sg * vv:+.2f}", lambda t, b=sg * vv: (0, kd, 0, b, 0), 0.6,
                             move_abort=5, quiet=True)
                w = r.window(0.25, static=True)
                if r.ok and len(w) >= 5 and r.moved_deg() < 1.0:
                    iv.append((abs(qk * qv), denoise(mean(x[3] for x in w), fl)))
                time.sleep(0.2)
        ratio_static = None
        if len(it) >= 3 and len(iv) >= 3:
            gt = sum(i for _, i in it) / sum(q for q, _ in it)
            gv = sum(i for _, i in iv) / sum(q for q, _ in iv)
            ratio_static = gv / gt
            print(f"  静止: |電流|/トルク指令 {gt:.3f} A/(N*m) / |電流|/(Kd*V) {gv:.3f}（雑音補正後。床 {fl:.3f} A）")
            print(f"  → c_d × r_v = {ratio_static:.3f}")
        else:
            print("  静止テストが成立しませんでした（動いた/点が足りない）。")
        # --- 2) 回転テスト ---
        if self.move_cap is not None:
            print(f"  limit {self.move_cap:g} が掛かっているので回転テスト（数百度回る）は飛ばします。")
            print("  ※ 静止テストの c_d×r_v だけ。c_d と r_v を分けるのは、モーターを脚から外したときに。")
            if ratio_static is not None:
                self._save_calib(vscale_static=dict(c_d_x_r_v=ratio_static, v_range_used=V))
            return
        # 速度指令は 0.6 秒かけて上げる（止まった軸に Kd*V が一気にかからないように）。
        # 軸が引っかかって回らなければ電流ガード（3A）で止まる。
        res = []
        ramp = 0.6
        for kd in (1.0, 2.0, 3.0):
            omegas = []
            for sgn in (+1, -1):
                r = self.run(f"vscale 回転 Kd{kd:g} {'+' if sgn > 0 else '-'}",
                             lambda t, kd=kd, sgn=sgn: (0, kd, 0, sgn * v * min(1.0, t / ramp), 0), 2.2,
                             move_abort=400, spd_abort=230, quiet=True, kdv_limit=kd * v)
                if not r.ok:
                    print("  回転テストを中止します。")
                    return
                w = r.window(1.0)
                if len(w) < 10:
                    continue
                # 位置の差分から速度（ERPM 換算に依存しない）
                wdot = (w[-1][1] - w[0][1]) / (w[-1][0] - w[0][0]) * DEG2RAD
                werpm = mean(x[2] for x in w) / ERPM_PER_DEG_S * DEG2RAD
                qv = quantized_cmd((0, kd, 0, sgn * v, 0), model)[3]
                omegas.append((abs(wdot), abs(werpm), abs(qv)))
                time.sleep(0.3)
            if len(omegas) == 2:
                wd, we, qv = mean(o[0] for o in omegas), mean(o[1] for o in omegas), mean(o[2] for o in omegas)
                res.append((kd, qv, wd, we))
                print(f"    Kd={kd:g} 指令 {qv:.3f} rad/s → 定常 {wd:.3f} rad/s（位置の差分）/ {we:.3f}（ERPM換算）")
                if wd < 0.05:
                    print("    ⚠ ほぼ回っていません（摩擦に負けた）。v を上げて vscale 3 を試してください。")
        if len(res) < 2:
            print("  回転テストの点が足りません。")
            return
        # ω/V = r_v - F/(Kd*V)  を x = 1/(Kd*V) に対して直線で当てる（V が Kd で違っても使える形）
        xs = [1.0 / (kd * qv) for kd, qv, _, _ in res]
        ys = [wd / qv for kd, qv, wd, _ in res]
        fit = linfit(xs, ys) if len(res) >= 3 else None
        if fit is None:
            (x1, y1), (x2, y2) = list(zip(xs, ys))[:2]
            a = (y2 - y1) / (x2 - x1)
            fit = (a, y1 - a * x1, float("nan"))
        r_v = fit[1]
        F = -fit[0]
        erpm_ratio = mean(we / wd for _, _, wd, we in res if wd > 0.05)
        print(f"  → r_v（ファームが読む速度/送った速度）= {r_v:.3f}   摩擦/c_d = {F:.3f} N*m   R²={fit[2]:.3f}")
        print(f"  → ERPM換算 / 位置の差分 = {erpm_ratio:.3f}（1.00 なら ERPM = deg/s × 31.5 と極対数21 が正しい）")
        print(f"  → ファームの速度レンジ推定 ±{V * r_v:.1f} rad/s（候補: 65 / 50 / 30 / 28）")
        if ratio_static is not None and r_v > 0.05:
            c_d = ratio_static / r_v
            print(f"  → c_d（実効Kd/指令Kd）= {c_d:.3f}")
            if abs(r_v - 1) > 0.1:
                print("  ⚠ r_v が 1 から外れています。ver6 の『実効Kd 0.48』に速度レンジのずれが混ざっていた可能性。")
            self._save_calib(vscale=dict(r_v=r_v, c_d=c_d, c_d_x_r_v=ratio_static, fric_over_cd=F,
                                         erpm_ratio=erpm_ratio, v_range_used=V))
        else:
            self._save_calib(vscale=dict(r_v=r_v, fric_over_cd=F, erpm_ratio=erpm_ratio, v_range_used=V))
        print("  レンジを直すなら range V 30 のように打ってから vscale をもう一度（r_v が 1.00 に近づけば確定）。")

    # ============================================================
    # wdog: 指令が途切れたとき（A4）
    # ============================================================
    def wdog(self, tau=0.15, hold=1.0, watch=3.0):
        if abs(tau) > 0.25:
            print("  拒否: 静止摩擦（0.3〜0.4）より十分小さいトルクで測ります。|tau|≤0.25")
            return
        if not self.need_feedback():
            return
        print(f"  トルク {tau:+.2f} N*m を {hold:.1f} 秒送り、そのあと送信だけ止めて {watch:.1f} 秒、電流を見ます。")
        print("  静止摩擦より小さいので軸は回らない想定。動いたら自動で零指令を送ります。")
        run = self.run("wdog", lambda t: (0, 0, 0, 0, tau) if t < hold else None, hold + watch, move_abort=10,
                       spd_abort=60, tick=0.25)
        if not run.ok and run.t_stop_send is None:
            return
        ts = run.t_stop_send if run.t_stop_send is not None else hold
        before = [r[3] for r in run.window(hold - 0.5, ts)]
        i_hold = mean(before)
        after = run.window(ts)
        print(f"  送信停止までの電流 {i_hold:+.3f} A")
        if abs(i_hold) < 0.1:
            print("  電流が小さすぎて判定できません。tau を 0.2 にして再実行。")
            return
        drop = None
        for k in range(len(after) - 2):
            if all(abs(after[k + j][3]) < 0.3 * abs(i_hold) for j in range(3)):
                drop = after[k][0] - ts
                break
        if drop is None:
            print(f"  ★ 送信を止めても {watch:.1f} 秒間ずっと電流が残りました ＝ 最後の指令を出し続ける。")
            print("    → 制御ループ側に受信/送信タイムアウトの零指令が必須。上位機の『CAN 通信途絶時の設定』も確認。")
        else:
            print(f"  ★ 送信停止から {drop * 1000:.0f} ms（±20ms）で電流が落ちました ＝ モーター側にタイムアウトがある。")
        self._save_calib(wdog=dict(tau=tau, i_hold=i_hold, drop_s=drop, watch_s=watch))

    # ============================================================
    # lat: 指令→フィードバックの遅れ（D2）
    # ============================================================
    def lat(self, tau=0.15, cycles=20, half=0.25):
        if not self.need_feedback():
            return
        per = 2 * half
        # 送信を 47Hz にして、フィードバック（50Hz）との位相を毎回ずらす（同じ 50Hz だと位相が固定されて最小値が出ない）
        run = self.run("lat", lambda t: (0, 0, 0, 0, tau if (t % per) >= half else 0.0), cycles * per,
                       move_abort=10, spd_abort=60, quiet=False, hz=47)
        if not run.ok:
            return
        hi = [r[3] for r in run.rx if (r[0] % per) > half + 0.12]
        i_hi = mean(hi)
        delays = []
        for c in range(1, cycles):
            te = next((x[0] for x in run.tx if x[0] >= c * per + half - 1e-3 and abs(x[5]) > 1e-3), None)
            if te is None:
                continue
            first = next((r[0] for r in run.rx if r[0] > te and abs(r[3]) > 0.5 * abs(i_hi)), None)
            if first is not None:
                delays.append((first - te) * 1000)
        if not delays:
            print("  遅れを測れませんでした（電流が小さい？）")
            return
        print(f"  指令 → 電流が半分に達したフィードバックの受信まで: 最小 {min(delays):.0f} / 中央値 {median(delays):.0f} / "
              f"最大 {max(delays):.0f} ms（{len(delays)} 回）")
        print("  ※ フィードバックが 20ms 間隔なので、最小値が「本当の遅れ」に近い。最大−最小 ≒ 20ms なら正常。")
        self._save_calib(latency_ms=dict(min=min(delays), median=median(delays), max=max(delays)))

    # ============================================================
    # morg: MIT の原点
    # ============================================================
    def _probe(self, off_guess, kp, kd, sec, label, move_abort=20):
        """X = θ + off_guess に Kp を掛けて |電流| を読む。戻り値 (|電流|平均, [(u, |I|)], run, X)"""
        th0 = self.theta()
        X = th0 + off_guess
        run = self.run(label, lambda t: (kp, kd, X, 0, 0), sec, move_abort=move_abort, spd_abort=120, quiet=True)
        samples = [((X - r[1] * DEG2RAD), r[3]) for r in run.window(0.2, static=True)]
        i = mean(c for _, c in samples) if samples else float("nan")
        return i, samples, run, X

    @staticmethod
    def _vfit(us, cs):
        """|I| = a*|u - off| + b を当てる（off を格子で探して最小二乗）。戻り値 (off, a, b, R2)"""
        lo, hi = min(us), max(us)
        best = None
        for rng, n in (((lo, hi), 200), (None, 100)):
            if rng is None:
                step = (hi - lo) / 200
                rng = (best[1] - 2 * step, best[1] + 2 * step)
            for k in range(n + 1):
                off = rng[0] + (rng[1] - rng[0]) * k / n
                xs = [abs(u - off) for u in us]
                f = linfit(xs, cs)
                if f is None:
                    continue
                a, b, r2 = f
                sse = sum((c - (a * x + b)) ** 2 for x, c in zip(xs, cs))
                if best is None or sse < best[0]:
                    best = (sse, off, a, b, r2)
        return best[1:] if best else None

    def morg(self, mode="full"):
        s = self.need_feedback()
        if not s:
            return
        model = self.model
        print("  MIT の原点を実測します（MIT位置 = サーボ角 + offset）。電流は大きさだけで V 字の底を探します。")
        fl = self.noise_floor()
        ratio = self.kp_ratio or 1.0
        guess = self.off if self.off is not None else 0.0
        # --- ① 単点プローブ（Kp=1LSB）: |電流| からずれの大きさ、左右を試して向きを決める ---
        if mode == "full" or self.off is None:
            kp = 0.13
            kpq = quantized_cmd((kp, 0, 0, 0, 0), model)[0]

            def probe(g, k):
                i, smp, run, X = self._probe(g, kp, 1.5, 0.4, f"morg 寄せ{k}")
                if len(smp) < 4:
                    allw = run.window(0.05)
                    i = mean(r[3] for r in allw) if allw else float("nan")
                return i, len(smp) >= 4
            i, still = probe(guess, 1)
            for k in range(6):
                if math.isnan(i):
                    print("  中止: 電流が読めません")
                    return
                e = denoise(i, fl) * KT_CMD / (kpq * ratio)
                print(f"    寄せ: offset候補 {guess:+.3f} rad → |電流| {i:.3f} A（床 {fl:.3f}）→ ずれの大きさ ≈ {e:.3f} rad"
                      f"{'（動いた）' if not still else ''}")
                if e < 0.12 and still:
                    break
                ip, sp = probe(guess + e, 2 * k + 2)
                im, sm = probe(guess - e, 2 * k + 3)
                print(f"      +側 {guess + e:+.3f}: {ip:.3f} A / −側 {guess - e:+.3f}: {im:.3f} A")
                if ip <= im:
                    guess, i, still = guess + e, ip, sp
                else:
                    guess, i, still = guess - e, im, sm
                if abs(guess) > MIT_RANGES[model]["P"] * 2:
                    print("  中止: offset 推定がレンジの2倍を超えました。")
                    return
        # --- ②③ V 字あてはめ ---
        results = []
        kp_ratio = None
        for kp, npts, sec in ((0.25, 9, 0.6), (1.0, 9, 0.6)):
            kpq = quantized_cmd((kp, 0, 0, 0, 0), model)[0]
            span = min(0.8, 0.20 / (kpq * min(ratio, 1.0)))
            span = min(span, 0.8)
            us, cs = [], []
            for j in range(npts):
                d = -span + 2 * span * j / (npts - 1)
                i, smp, run, X = self._probe(guess + d, kp, 1.0, sec, f"morg Kp{kp:g} {j + 1}/{npts}", move_abort=10)
                if not run.ok:
                    print("  中止しました。")
                    return
                for u, c in smp:
                    us.append(u)
                    cs.append(c)
                time.sleep(0.15)
            fit = self._vfit(us, cs) if len(us) > 20 else None
            if fit is None or fit[1] <= 1e-6:
                print(f"  ■ Kp={kp:g}: |電流| が目標位置に反応していません。原点は採用しません。")
                return
            off, a, b, r2 = fit
            kp_ratio = a * KT_CMD / kpq
            print(f"    Kp={kpq:.3f}: offset {off:+.4f} rad ({off * RAD2DEG:+.2f} deg)  R²={r2:.3f}  "
                  f"傾き {a:.3f} A/rad  底 {b:.3f} A → (実効Kp/指令Kp)×(0.59/Kt) ≈ {kp_ratio:.2f}  [{len(us)} サンプル]")
            # Kp=0.24 段は信号が小さく、|I| の雑音が ±0.05 A に増えると R² 0.75 前後になる（2026-09-16 実測、底の位置は ±1 deg で一致）。
            # 粗い段は 0.6、信号が4倍の Kp=0.98 段は 0.8 を要求し、最後に overify で左右差を見る。
            need = 0.6 if kp < 0.5 else 0.8
            if r2 < need:
                print(f"  ■ V 字の当てはまりが悪い（R²={r2:.3f} < {need}）ので採用しません。")
                return
            results.append(off)
            guess = off
            ratio = max(kp_ratio, 0.2)
        self.kp_ratio = kp_ratio
        if abs(results[1] - results[0]) > 3 * DEG2RAD:
            print(f"  ■ Kp=0.24 と 0.98 で {abs(results[1] - results[0]) * RAD2DEG:.1f} deg 違います。採用しません。")
            return
        self.off = results[1]
        self.off_verified = False
        if self.overify(quiet=True):
            th = self.theta()
            self.morg_hist.append((th, self.off, time.strftime("%H:%M:%S")))
            self._save_calib(offset_rad=self.off, theta_at_morg_deg=th * RAD2DEG, kp_ratio_est=kp_ratio)
            print(f"  ★ 原点を採用: offset = {self.off:+.4f} rad（サーボ角 {th * RAD2DEG:+.1f} deg で測定）。位置指令が使えます。")

    def overify(self, quiet=False):
        """
        offset の左右 ±δ に同じ Kp を掛け、|電流| が左右で揃うかを見る（V 字の底が offset にあるか）。
        ずれ d = δ × (I+ − I−)/(I+ + I−)。倍率・Kt・ノイズの床に鈍い。
        """
        if not self.need_feedback() or not self.need_origin(verified=False):
            return False
        kp, delta = 1.0, 0.15
        fl = self.noise_floor()
        vals = []
        for sg in (+1, -1):
            i, smp, run, X = self._probe(self.off + sg * delta, kp, 1.0, 0.7, f"overify {'+' if sg > 0 else '-'}", move_abort=5)
            if not run.ok or len(smp) < 5:
                print("  ■ 検証中に動いた/中断。原点は外れています。morg をやり直してください。")
                self.off_verified = False
                return False
            vals.append(denoise(i, fl))
            time.sleep(0.15)
        ip, im = vals
        tot = ip + im
        if tot < 0.03:
            print(f"  ■ |電流| が小さすぎて判定できません（+ {ip:.3f} / − {im:.3f} A）。")
            self.off_verified = False
            return False
        ratio = (ip - im) / tot
        d = delta * ratio
        # I+ が大きい ＝ +側のほうが底から遠い ＝ 真の底は offset より −側 ＝ offset を下げるべき
        print(f"    +{delta:.2f} rad: {ip:.3f} A / −{delta:.2f} rad: {im:.3f} A → 原点の残差 ≈ {d * RAD2DEG:+.2f} deg"
              f"（+ なら真の原点は offset より小さい側）")
        if abs(ratio) < 0.8 and abs(d) <= 2.0 * DEG2RAD:
            self.off_verified = True
            if not quiet:
                print("  ★ 原点は合っています（残差 2 deg 以内）。")
            return True
        self.off_verified = False
        if abs(ratio) >= 0.8:
            print(f"  ■ 左右の差が大きすぎます（底が ±{delta * RAD2DEG:.1f} deg の外）。原点は外れています。morg をやり直してください。")
        else:
            print(f"  ■ 残差 {d * RAD2DEG:+.1f} deg。原点は外れています。")
        return False

    def ocheck(self):
        """o 0（モード5の原点設定）で MIT の原点も動くか"""
        if not self.need_feedback() or not self.need_origin():
            return
        th1, off1 = self.theta(), self.off
        print(f"  いま: サーボ角 {th1 * RAD2DEG:+.2f} deg / offset {off1:+.4f} rad。o 0 を送ります（軸は動きません）。")
        self.bus.send(cm.f_set_origin(self.target, 0))
        self.morg_hist = []
        time.sleep(0.5)
        th2 = self.theta()
        print(f"  o 0 の後: サーボ角 {th2 * RAD2DEG:+.2f} deg")
        h0 = off1 + th1 - th2
        h1 = -th2
        print(f"  予想 A（MIT 原点は動かない）: offset {h0:+.4f} rad / 予想 B（MIT 原点も今の位置へ）: offset {h1:+.4f} rad")
        self.off = h0
        self.off_verified = False
        self.morg(mode="full")
        if self.off is None or not self.off_verified:
            self.off = None
            print("  morg が通りませんでした。morg full で測り直してください。")
            return
        da, db = abs(self.off - h0) * RAD2DEG, abs(self.off - h1) * RAD2DEG
        verdict = "A: o 0 は MIT の原点を動かさない" if da < db else "B: o 0 で MIT の原点も動く"
        print(f"  ★ 結果: {verdict}（A との差 {da:.1f} deg / B との差 {db:.1f} deg）")
        self._save_calib(ocheck=dict(result=verdict, diff_A_deg=da, diff_B_deg=db))

    def ofit(self):
        h = self.morg_hist
        if len(h) < 2:
            print("  morg の結果が2つ以上要ります。x → 手で軸を回す（30〜120 deg）→ morg、を繰り返してから。")
            return
        print("  サーボ角[deg]   offset[deg]")
        for th, off, tm in h:
            print(f"   {th * RAD2DEG:+9.1f}   {off * RAD2DEG:+9.2f}   {tm}")
        ths = [x[0] for x in h]
        offs = [x[1] for x in h]
        spread = (max(offs) - min(offs)) * RAD2DEG
        if (max(ths) - min(ths)) * RAD2DEG < 20:
            print("  角度が 20 deg 以上離れた場所で測ってください。")
            return
        fit = linfit(ths, offs) if len(h) >= 3 else None
        if fit is None:
            a = (offs[1] - offs[0]) / (ths[1] - ths[0])
        else:
            a = fit[0]
        print(f"  offset のばらつき {spread:.2f} deg / 傾き d(offset)/d(角度) = {a:+.4f}")
        print(f"  → MIT位置 ≈ {1 + a:.4f} × サーボ角 + 定数")
        if abs(a) < 0.01:
            print("  ★ ただのずれ（倍率 1）。offset 1個で扱える。")
        elif abs(1 + a + 1) < 0.02:
            print("  ★ 向きが逆（倍率 −1）。")
        else:
            print("  ★ 倍率つき。位置レンジの解釈（±12.56 / ±12.5）か単位の違いを疑う。range P で変えて測り直す。")
        self._save_calib(ofit=dict(scale=1 + a, spread_deg=spread, n=len(h)))

    # ============================================================
    # 位置系（原点必要）
    # ============================================================
    def hold(self, kp, kd, sec, quiet=False):
        if kp > LIM_KP or kd > LIM_KD or sec > LIM_SEC:
            print(f"  拒否: Kp≤{LIM_KP} Kd≤{LIM_KD} 秒≤{LIM_SEC}")
            return None
        if kp > 0 and kd < 0.3:
            print("  拒否: Kp>0 なら Kd≥0.3（減衰なしは発振する。277 deg/s の実績）")
            return None
        if not self.need_feedback() or not self.need_origin():
            return None
        th0 = self.theta()
        X = th0 + self.off
        run = self.run(f"hold Kp{kp:g} Kd{kd:g}", lambda t: (kp, kd, X, 0, 0), sec, move_abort=30,
                       tick=0 if quiet else 0.5, quiet=quiet)
        w = run.window(min(0.5, sec * 0.3))
        if not w:
            return None
        cur = [r[3] for r in w]
        pos = [r[1] for r in w]
        res = dict(kp=kp, kd=kd, ok=run.ok, i_mean=mean(cur), i_p2p=max(cur) - min(cur), i_std=std(cur),
                   pos_p2p=max(pos) - min(pos), drift=pos[-1] - th0 * RAD2DEG, freq=0.0)
        if not quiet:
            print(f"  位置の振れ幅 {res['pos_p2p']:.2f} deg / ズレ {res['drift']:+.2f} deg / |電流| 平均 {res['i_mean']:.3f} A "
                  f"振れ幅 {res['i_p2p']:.3f} 標準偏差 {res['i_std']:.3f}")
        return res

    @staticmethod
    def _osc_freq(w):
        if len(w) < 10:
            return 0.0
        m = mean(r[3] for r in w)
        z = sum(1 for a, b in zip(w, w[1:]) if (a[3] - m) * (b[3] - m) < 0)
        dur = w[-1][0] - w[0][0]
        return z / 2 / dur if dur > 0 else 0.0

    def kpscale(self, kps=(1.0, 2.0, 5.0), taus=(0.1, 0.2)):
        """
        同じ大きさのトルクを「Kp×(±δ)」と「トルク指令 ±τ」で出して |電流| を比べる。
        ±δ を対にすると原点の残差 ε は |δ+ε|+|δ−ε| = 2δ で打ち消える（|ε|<δ のとき）。比 = 実効Kp/指令Kp。
        """
        if not self.need_feedback() or not self.need_origin():
            return
        model = self.model
        print("  各段 0.6 秒。静止摩擦より小さいトルク（≤0.2）なので軸はほぼ動きません。")
        fl = self.noise_floor()
        ref = {}
        for tau in taus:
            vals = []
            for sg in (+1, -1):
                r = self.run(f"kpscale 基準 t{sg * tau:+.2f}", lambda t, q=sg * tau: (0, 0, 0, 0, q), 0.6, move_abort=5,
                             quiet=True)
                w = r.window(0.25, static=True)
                if r.ok and len(w) >= 5:
                    vals.append(denoise(mean(x[3] for x in w), fl))
            if len(vals) == 2:
                ref[tau] = mean(vals) / abs(quantized_cmd((0, 0, 0, 0, tau), model)[4])
        if not ref:
            print("  基準（トルク指令）が取れませんでした。")
            return
        g_ref = mean(ref.values())
        print(f"  基準: |電流|/トルク指令 = {g_ref:.3f} A/(N*m)（雑音補正後。床 {fl:.3f} A。τ別 "
              + ", ".join(f"{k:g}:{v:.3f}" for k, v in ref.items()) + "）")
        out = []
        for kp in kps:
            kpq = quantized_cmd((kp, 0, 0, 0, 0), model)[0]
            gs = []
            for tau in taus:
                delta = tau / kpq
                vals = []
                for sg in (+1, -1):
                    X = self.theta() + self.off + sg * delta
                    r = self.run(f"kpscale Kp{kp:g} d{sg * delta * RAD2DEG:+.1f}", lambda t, x=X: (kp, 1.0, x, 0, 0), 0.6,
                                 move_abort=5, quiet=True)
                    if not r.ok:
                        print("  中止しました。")
                        return
                    w = r.window(0.25, static=True)
                    if len(w) >= 5:
                        vals.append(denoise(mean(x[3] for x in w), fl))
                    time.sleep(0.15)
                if len(vals) == 2:
                    gs.append(mean(vals) / (kpq * delta))
                    print(f"    Kp={kpq:.3f} δ=±{delta * RAD2DEG:.2f} deg: |電流|（雑音補正後）+ {vals[0]:.3f} / − {vals[1]:.3f} A")
            if gs:
                c_p = mean(gs) / g_ref
                out.append((kp, c_p))
                print(f"    → Kp={kpq:.3f}: 実効Kp/指令Kp = {c_p:.3f}")
        if out:
            c = median([o[1] for o in out])
            print(f"  ★ c_p（実効Kp/指令Kp）≈ {c:.3f}")
            print(f"  → シム stiffness 15 を出すには 指令Kp ≈ {15 / c:.1f}")
            vs = self.calib.get(str(self.target), {})
            c_d = vs.get("vscale", {}).get("c_d")
            if c_d:
                print(f"  → シム damping 1.5 を出すには 指令Kd ≈ {1.5 / c_d:.2f}（vscale の c_d={c_d:.3f} から）")
            self._save_calib(kpscale=dict(c_p=c, per_kp={str(k): v for k, v in out}))

    def ramp(self, kd=1.0, sec=3.0, kps=(2.0, 5.0, 10.0, 15.0)):
        if not self.need_origin():
            return
        print(f"  Kp を {kps} と上げて各 {sec:g} 秒保持（Kd={kd:g}）。リミットサイクル（C6）を見ます。")
        for kp in kps:
            r = self.hold(kp, kd, sec, quiet=True)
            if r is None or not r["ok"]:
                return
            print(f"    Kp={kp:<4g} 位置振れ幅 {r['pos_p2p']:.2f} deg / |電流| 平均 {r['i_mean']:.3f} 振れ幅 {r['i_p2p']:.3f} "
                  f"標準偏差 {r['i_std']:.3f}（位置が止まっていて |電流| の振れ幅が小さければ振動なし）")
            time.sleep(0.3)
        print("  目安: サーボモードでは 90 deg 保持で 0.4↔1.0 A を約5Hz。振れ幅が小さいままなら C6 は MIT で解消。")

    def step(self, deg, kp, kd, sec):
        if abs(deg) > LIM_STEP_DEG or kp > LIM_KP or kd > LIM_KD or sec > LIM_SEC:
            print(f"  拒否: |deg|≤{LIM_STEP_DEG} Kp≤{LIM_KP} Kd≤{LIM_KD} 秒≤{LIM_SEC}")
            return
        if kp > 0 and kd < 0.3:
            print("  拒否: Kd≥0.3")
            return
        if not self.need_feedback() or not self.need_origin():
            return
        th0 = self.theta()
        X0 = th0 + self.off
        X1 = X0 + deg * DEG2RAD
        pre = 1.0
        print(f"  ★ 動きます: {th0 * RAD2DEG:+.1f} deg を1秒保持 → {deg:+.0f} deg へ（Kp={kp:g} Kd={kd:g}）")
        run = self.run(f"step {deg:+.0f} Kp{kp:g} Kd{kd:g}", lambda t: (kp, kd, X0 if t < pre else X1, 0, 0), pre + sec,
                       move_abort=abs(deg) + 30, tick=0.25)
        after = run.window(pre)
        if not after:
            return
        p0 = th0 * RAD2DEG
        frac = [((r[1] - p0) / deg, r[0] - pre) for r in after]
        t10 = next((t for f, t in frac if f >= 0.1), None)
        t90 = next((t for f, t in frac if f >= 0.9), None)
        peak = max(f for f, _ in frac)
        tail = after[-max(1, len(after) // 5):]
        final = mean(r[1] for r in tail)
        rise = f"{(t90 - t10) * 1000:.0f} ms" if t10 is not None and t90 is not None else "未到達"
        print(f"  立ち上がり(10-90%) {rise} / 行き過ぎ {(peak - 1) * 100:+.1f}% / 定常偏差 {p0 + deg - final:+.2f} deg / "
              f"最大電流 {max(abs(r[3]) for r in after):.2f} A")
        if run.csv:
            print(f"  CSV: {run.csv}")

    # ============================================================
    # 入力ループ
    # ============================================================
    def loop(self):
        print(HELP)
        while True:
            try:
                line = input(f"[ID{self.target} {self.model}{' 原点OK' if self.off_verified else ''}]> ").strip()
            except EOFError:
                break
            if not line:
                continue
            p = line.split()
            c = p[0].lower()

            def num(i, d):
                try:
                    return float(p[i])
                except (IndexError, ValueError):
                    return d
            try:
                if c == "q":
                    break
                elif c in ("?", "h", "help"):
                    print(HELP)
                elif c == "plan":
                    print(PLAN)
                elif c == "s":
                    self.show()
                elif c == "m":
                    t0 = time.time()
                    while time.time() - t0 < num(1, 3.0):
                        self.show()
                        time.sleep(0.25)
                elif c == "sniff":
                    self.sniff(num(1, 3.0))
                elif c == "ping":
                    ids = [int(x) for x in p[1:]] or [self.target]
                    self.ping(ids)
                elif c == "limit":
                    if len(p) > 1 and p[1] == "off":
                        self.move_cap = None
                    elif len(p) > 1:
                        self.move_cap = abs(float(p[1]))
                    print(f"  移動量の上限: {'なし（実験ごとの既定値）' if self.move_cap is None else f'{self.move_cap:g} deg（全実験）'}")
                elif c == "jog":
                    self.jog(num(1, 10.0), num(2, 400.0))
                elif c == "scan":
                    self.scan(num(1, 2.0))
                elif c == "id":
                    self.target = int(num(1, self.target))
                    print(f"  対象 ID={self.target}（MIT 送信ID 0x{(MODE_MIT << 8) | self.target:X}）")
                    self._pull_calib()
                elif c == "model":
                    if len(p) < 2 or p[1] not in MIT_RANGES:
                        print(f"  選べるのは {list(MIT_RANGES)}")
                        continue
                    self.model = p[1]
                    self.i_sign = None
                    print(f"  機種 {self.model}: {MIT_RANGES[self.model]}")
                    print("  ⚠ 機種の取り違えはトルクが3倍になります（AK80-9 ±18 / AK10-9 ±54）。")
                elif c == "range":
                    r = MIT_RANGES[self.model]
                    if len(p) >= 3 and p[1].upper() in ("P", "V"):
                        r[p[1].upper()] = float(p[2])
                        print(f"  {self.model} の {p[1].upper()} レンジを ±{float(p[2])} にしました（このセッションだけ）")
                        if p[1].upper() == "P":
                            self.off_verified = False
                            print("  位置の解釈が変わったので原点は未検証に戻しました。")
                    print(f"  {self.model}: {r}")
                    print(f"  出典: {RANGE_SOURCES.get(self.model, '-')}")
                elif c == "hz":
                    self.scan(num(1, 2.0))
                elif c == "health":
                    b = self.bus
                    print(f"  送信 {b.tx_count} / エコー {b.echo_count} / 受信 {b.rx_count} / 受信スレッド {b.rx_error or '正常'}")
                elif c == "raw":
                    self.bus.raw_log = len(p) > 1 and p[1] == "on"
                    print(f"  生ログ {'ON' if self.bus.raw_log else 'OFF'}")
                elif c == "csv":
                    self.csv_on = not (len(p) > 1 and p[1] == "off")
                    print(f"  CSV 自動保存 {'ON → ' + self.log_dir if self.csv_on else 'OFF'}")
                elif c == "log":
                    if len(p) > 1 and p[1] == "off":
                        if isinstance(sys.stdout, Tee):
                            fh = sys.stdout.fh
                            sys.stdout = sys.stdout.stream
                            fh.close()
                            print("  記録終了")
                    elif not isinstance(sys.stdout, Tee):
                        os.makedirs(self.log_dir, exist_ok=True)
                        fn = os.path.join(self.log_dir, "session_" + time.strftime("%H%M%S") + ".txt")
                        sys.stdout = Tee(sys.stdout, open(fn, "a", encoding="utf-8"))
                        print(f"  この画面を {fn} に記録します")
                elif c == "preview":
                    kp, kd, pos, vel, tau = (num(i, 0.0) for i in range(1, 6))
                    fr = cm.Frame((MODE_MIT << 8) | self.target, pack_mit(kp, kd, pos, vel, tau, self.model, dither=False), True)
                    b = unpack_mit(fr.data, self.model)
                    print(f"  ID=0x{fr.arbitration_id:X} DATA={fr.data.hex(' ')}")
                    print(f"  届く値: Kp={b['kp']:.4f} Kd={b['kd']:.5f} pos={b['pos']:+.5f} vel={b['vel']:+.4f} tau={b['tau']:+.4f}")
                elif c == "jit":
                    self.jit(num(1, 5.0))
                elif c == "t":
                    self.cmd_t(num(1, 0.1), num(2, 2.0))
                elif c == "isign":
                    self.measure_isign()
                elif c == "brk":
                    self.brk(num(1, 0.20), num(2, 0.60), num(3, 0.02))
                elif c == "vscale":
                    self.vscale(num(1, 2.5))
                elif c == "wdog":
                    self.wdog(num(1, 0.15), num(2, 1.0), num(3, 3.0))
                elif c == "lat":
                    self.lat(num(1, 0.15))
                elif c == "o":
                    kind = int(num(1, 0))
                    self.bus.send(cm.f_set_origin(self.target, kind))
                    print(f"  原点設定（モード5, kind={kind}）を送りました。軸は動きません。")
                    self.morg_hist = []
                    if self.off is not None:
                        self.off_verified = False
                        print("  MIT の原点が動いたか分からないので未検証に戻しました（ocheck で調べられます）。")
                    time.sleep(0.3)
                    self.show()
                elif c == "morg":
                    self.morg("fine" if (len(p) > 1 and p[1] == "fine") else "full")
                elif c == "overify":
                    self.overify()
                elif c == "ocheck":
                    self.ocheck()
                elif c == "ofit":
                    self.ofit()
                elif c == "oclear":
                    self.off, self.off_verified, self.morg_hist = None, False, []
                    print("  原点を消しました（ファイルはそのまま）")
                elif c == "hold":
                    self.hold(num(1, 2.0), num(2, 1.0), num(3, 3.0))
                elif c == "kpscale":
                    self.kpscale()
                elif c == "ramp":
                    self.ramp(num(1, 1.0), num(2, 3.0))
                elif c == "step":
                    self.step(num(1, 30.0), num(2, 5.0), num(3, 1.0), num(4, 2.0))
                elif c == "x":
                    for _ in range(3):
                        self.bus.send(f_mit(self.target, 0, 0, 0, 0, 0, self.model))
                        time.sleep(0.01)
                    print("  零指令を送りました（ソフト停止。非常停止は電源OFF）")
                else:
                    print("  不明なコマンド。? でヘルプ、plan で今日の順番")
            except Exception as e:
                print(f"  コマンド実行エラー: {type(e).__name__}: {e}")
                try:
                    self.zero()
                except Exception:
                    pass


HELP = f"""
motor_console_ver8 -- MIT（モード8）でデプロイ用の数字を取る（電流は大きさだけで判定）。plan で今日の順番。

見る       s / m 3 / scan / sniff 3 / ping 22 / jog 10 / limit 8 / id 34 / model AK80-9 / range / range V 30 / health / raw on / log on / csv off / jit
トルクだけ  t 0.1 2 / isign / brk / vscale / wdog / lat          ← Kp=0。原点不要
原点       morg / morg fine / overify / ocheck / ofit / o 0 / oclear
位置系     hold 2 1 3 / kpscale / ramp / step 30 5 1 2          ← 原点を morg か overify で通してから
その他     preview 2 1 0 0 0（Kp Kd 位置 速度 トルク の届く値）/ x / q

安全上限  トルク {LIM_TAU} N*m / Kp {LIM_KP} / Kd {LIM_KD} / Kd×速度 {LIM_KDV} N*m / {LIM_SEC} 秒 / ステップ {LIM_STEP_DEG} deg
自動中断  観測 {MAX_AGE}s 途切れ / 電流 {CUR_ABORT} A / 速度 {SPD_ABORT_DEG_S:.0f} deg/s / 移動量 / err≠0 / バス異常
記録      すべての指令の送受信を logs\\v7_日付\\ に CSV で保存。結果の数字は {CALIB_FILE} に ID ごとに保存。
★ Kp は 1LSB=0.1221（0.06未満は0）。Kp>0 なら Kd≥0.3。x はソフト停止、非常停止は電源OFF。
"""

PLAN = """
今日の順番（1モーター・ベンチ・脚は外す。各行の後に結果を見せてください）
  0. log on → s → scan → jit               状態・ID・このPCの周期の揺れ
  1. isign → brk                           電流の符号と静止摩擦（ver6 と一致するか）
  2. vscale                                 速度レンジのずれ（r_v）と実効Kd（c_d）。原点不要
  3. wdog → lat                             指令が途切れたら何秒で脱力するか（A4）・遅れ（D2）
  4. morg → overify                         MIT の原点（A13）。ここを通るまで 5 以降に進まない
  5. x → 手で軸を 60〜90 deg 回す → morg を2回繰り返す → ofit   ただのずれか倍率つきか
  6. ocheck                                 o 0 で MIT の原点も動くか
  7. kpscale                                実効Kp（E5b）。シムの stiffness/damping → 指令値
  8. ramp → step 30 5 1 2                   リミットサイクル（C6）とステップ応答
  9. q → 電源OFF → 10秒 → 電源ON → 起動 → overify   原点が電源断を越えて残るか（A3・40deg の取り違え）
"""


def main():
    windows_timer_1ms()
    if SIM_MODE:
        import mit_sim
        params = json.loads(os.environ.get("MIT_SIM", "{}"))   # 例: set MIT_SIM={"c_p":0.45,"wdog":null}
        bus = mit_sim.SimBus(**params)
        print("★ シミュレーター（mit_sim.py）で起動。CAN には何も送りません。")
    else:
        bus = cm.MotorBus(channel=CHANNEL, bitrate=BITRATE, model=DEFAULT_MODEL)
    try:
        bus.open()
    except Exception as e:
        print(f"CAN 接続に失敗: {e}")
        sys.exit(1)
    if hasattr(bus, "mit_rx"):
        bus.mit_rx = False       # 応答はサーボ形式だけ。MIT 形式の解釈は誤読の元なので切る
    time.sleep(0.5)
    con = Console(bus)
    print(f"接続: ch={CHANNEL} {BITRATE}bps / ID={con.target} / {con.model} / MIT 送信ID 0x{(MODE_MIT << 8) | con.target:X}")
    con.show()
    try:
        con.loop()
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        try:
            con.zero()
        except Exception:
            pass
        bus.close(stop_motors=False)
        print("零指令を送って切断しました")


if __name__ == "__main__":
    main()

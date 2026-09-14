"""
motor_console_ver6.py -- MIT を「サーボ拡張フレームの制御モード8」で送る検証用コンソール

v5 との違いは送信IDの1点だけ:
    v5 (マニュアル5.3)  標準11bit  ID = モーターID            → 無反応だった
    v6 (今回)           拡張29bit  ID = (8 << 8) | モーターID  → これを検証する
ペイロード（pos16/vel12/kp12/kd12/tau12）は v5 と同じものを cubemars.f_mit から
そのまま借りる。詰め方は test_mit_encode.py で検証済みなので作り直さない。

⚠ 重要: このIDは v5 の `mscheme ext`（0x800+ID）と同じ値。
   2026-09-07 に ext を送ってCANバスが bus-off に落ちた実績がある（ACKが返らず
   再送を繰り返した）。そのため本ファイルは送信のたびにエコー数と受信スレッドの
   生死を見て、異常なら即座に送信を止める。

⚠ cubemars.py / motor_console_ver5.py は一切変更していない。この1ファイルで完結する。

安全:
  ・全コマンドは指定秒で自動終了し、終了時に必ず零トルクのmode8フレームを送る
  ・トルク上限 1.0 N*m（静止摩擦の実測は 0.54 N*m）
  ・フィードバックが最初から無い状態では実行を拒否する（観測できない指令は打たない）
  ・x はソフト停止。本当の非常停止は電源を切ること
"""

import sys
import time

import cubemars as cm

CHANNEL = 1
BITRATE = 1_000_000
DEFAULT_MOTOR_ID = 34
MODEL = "AK80-9"
SEND_HZ = 50

RAD2DEG = 57.29577951308232
DEG2RAD = 0.017453292519943295

# 安全上限
LIM_TAU = 1.0        # N*m。静止摩擦 0.54 / 動摩擦 0.22 の実測に対して余裕を持たせた値
LIM_KP = 20.0
LIM_KD = 3.0
LIM_SEC = 10.0
LIM_STEP_DEG = 90.0
MORG_KP = 0.25       # 原点探索の Kp。★1LSB=0.1221 なので 0.06 未満は 0 に潰れる
MORG_KD = 0.5        # 原点探索の Kd。動き出したときの減衰用（静止中は効かない）
MORG_SPD_GATE = 200  # ERPM。これより速いサンプルは Kd 項が混ざるので捨てる
MORG_MIN_R2 = 0.70   # これ未満の当てはまりなら原点を採用しない
MORG_KP_MAX = 0.3    # 較正前に許す Kp の上限
KT_OUT = 0.59        # N*m/A。2026-09-14 実測（停止中4点）。回転中の0.78は未決

# バス健全性の判定: 正常時のエコーは送信1件につき1件。
# 2026-09-07 の bus-off 時は 1送信あたり12件だった（ACKなしの再送）。
ECHO_RATIO_ABORT = 3.0

# 観測と電流の中断しきい値（2026-09-14 の事故を受けて追加）
MAX_AGE = 0.3        # フィードバックがこれ以上古くなったら送信を止める
CUR_ABORT = 3.0      # A。Bus Current Limit 35A / Phase 60A に対して十分手前
TAU_ABORT = 5.0      # N*m。MIT応答のときは cur が電流でなくトルクなので別の値で見る

# ⚠ 2026-09-14: t 0.6 で軸が 42,900 ERPM まで回り、2秒で6回転以上した。
#   そのとき電流は ほぼ 0 A。回っている間は電流が出ないので、電流ガードでは
#   暴走を止められない。速度と移動量でも見る。
SPD_ABORT_ERPM = 8000     # ≒254 deg/s（出力軸）。サーボ応答のとき
SPD_ABORT_RADS = 5.0      # MIT応答のとき
MOVE_ABORT_DEG = 360.0    # 開始位置からこれだけ動いたら止める

MODE_ID = 8          # ★ 検証対象。mode コマンドで実行時に変えられる

# ============================================================
# ★ 送信ロック（2026-09-14 16:xx 追加）
# ============================================================
# モード8に「classic MIT の詰め方」で送ったところ、Kp=0 Kd=0 トルク0.3N*m の
# 指令で 48.7 A が流れた。つまり相手はこのバイト列を別の意味で読んでいる。
#
# 原因の見当: classic MIT はオフセットバイナリで、「位置0 rad」が 0x7FFF になる。
#   t 0.3 で実際に送っていたのは 7F FF 7F F0 00 00 08 21
#   相手が先頭を符号付き int16 で読むと 0x7FFF = +32767 ＝ 最大値の指令。
#   全部ゼロを狙った指令ですら 7F FF 7F F0 00 00 07 FF になるので、
#   「無害な値」が存在しない。だから総当たりで探ってはいけない。
#
# ペイロード仕様（バイト順・ビット幅・符号の有無・レンジ）が確定するまで
# モード8への送信を禁止する。arm コマンドで一時的に解除できるが、
# 解除しても過電流ガードは効いたままにする。
# 2026-09-14 解除: ペイロード仕様（マニュアル42/44-45頁）が判明し、
# pack_mit を正しい並びに直したため。過電流ガードはそのまま残す。
SEND_LOCKED = False
LOCK_NOTE = "モード8のペイロード仕様が未確定"


# ============================================================
# パラメータ範囲（マニュアル42頁）
# ============================================================
# ⚠ cubemars.py の MODELS とは値が違う。こちらが一次資料。
#   AK80-9 の速度は ±50 ではなく ±65、位置は ±12.5 ではなく ±12.56。
#   AK10-9 は速度 ±28 / トルク ±54（MODELS は 50 / 65 で誤り）。
#   レンジがズレると、送る値も読む値もスケールごと狂う。
MIT_RANGES = {
    "AK10-9": dict(P=(-12.56, 12.56), V=(-28.0, 28.0), T=(-54.0, 54.0),
                   KP=(0.0, 500.0), KD=(0.0, 5.0)),
    "AK80-9": dict(P=(-12.56, 12.56), V=(-65.0, 65.0), T=(-18.0, 18.0),
                   KP=(0.0, 500.0), KD=(0.0, 5.0)),
    "AK60-6": dict(P=(-12.56, 12.56), V=(-60.0, 60.0), T=(-12.0, 12.0),
                   KP=(0.0, 500.0), KD=(0.0, 5.0)),
}


def sync_ranges_into_cubemars():
    """
    受信側（cm.parse_mit_reply）も同じレンジで解くようにする。
    cubemars.py のファイルは書き換えず、読み込んだ辞書の中身だけ差し替える。
    """
    for name, r in MIT_RANGES.items():
        if name in cm.MODELS:
            cm.MODELS[name]["p_lim"] = r["P"][1]
            cm.MODELS[name]["v_lim"] = r["V"][1]
            cm.MODELS[name]["t_lim"] = r["T"][1]
            cm.MODELS[name]["kp_max"] = r["KP"][1]
            cm.MODELS[name]["kd_max"] = r["KD"][1]


# ============================================================
# フレーム
# ============================================================
def pack_mit(kp, kd, pos, vel, tau, model=MODEL):
    """
    マニュアル44-45頁の pack_cmd と同じ並び。

    ⚠ classic MIT（位置→速度→Kp→Kd→トルク）とは順序が違う。
      こちらは Kp → Kd → 位置 → 速度 → トルク。
      2026-09-14、classic の並びで送って 48.7A が流れた。理由は、
      「位置0rad」のオフセットバイナリ 0x7FFF が先頭2バイトに来て、
      それを相手が Kp として読み Kp=250 / Kd=4.84 になったため。
      ゲインが先頭にある並びなので、Kp=Kd=0 なら位置・速度は無視される＝
      零指令が本当に安全になる。
    """
    r = MIT_RANGES[model]
    kp_i = cm.float_to_uint(kp, *r["KP"], 12)
    kd_i = cm.float_to_uint(kd, *r["KD"], 12)
    p_i = cm.float_to_uint(pos, *r["P"], 16)
    v_i = cm.float_to_uint(vel, *r["V"], 12)
    t_i = cm.float_to_uint(tau, *r["T"], 12)
    return bytes([
        (kp_i >> 4) & 0xFF,                          # Kp 上位8
        ((kp_i & 0x0F) << 4) | ((kd_i >> 8) & 0x0F),  # Kp 下位4 | Kd 上位4
        kd_i & 0xFF,                                  # Kd 下位8
        (p_i >> 8) & 0xFF,                            # 位置 上位8
        p_i & 0xFF,                                   # 位置 下位8
        (v_i >> 4) & 0xFF,                            # 速度 上位8
        ((v_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F),    # 速度 下位4 | トルク上位4
        t_i & 0xFF,                                   # トルク 下位8
    ])


def unpack_mit(data, model=MODEL):
    """pack_mit の逆。送る前の確認と、事故ったバイト列の解読に使う"""
    r = MIT_RANGES[model]
    d = data
    kp_i = (d[0] << 4) | ((d[1] >> 4) & 0x0F)
    kd_i = ((d[1] & 0x0F) << 8) | d[2]
    p_i = (d[3] << 8) | d[4]
    v_i = (d[5] << 4) | ((d[6] >> 4) & 0x0F)
    t_i = ((d[6] & 0x0F) << 8) | d[7]
    return dict(kp=cm.uint_to_float(kp_i, *r["KP"], 12),
                kd=cm.uint_to_float(kd_i, *r["KD"], 12),
                pos=cm.uint_to_float(p_i, *r["P"], 16),
                vel=cm.uint_to_float(v_i, *r["V"], 12),
                tau=cm.uint_to_float(t_i, *r["T"], 12))


def quantized(val, lo, hi, bits):
    """実際に送られる値。12bit/16bit に丸めた後の値を返す"""
    return cm.uint_to_float(cm.float_to_uint(val, lo, hi, bits), lo, hi, bits)


def gain_note(kp, kd, model=MODEL):
    """Kp/Kd が量子化でどうなるかを1行で"""
    r = MIT_RANGES[model]
    qp = quantized(kp, *r["KP"], 12)
    qd = quantized(kd, *r["KD"], 12)
    msg = f"実際に送られる Kp={qp:.4f} / Kd={qd:.4f}"
    if kp > 0 and qp == 0:
        msg += f"  ⚠ Kp が 0 に潰れています（1LSB={r['KP'][1] / 4095:.4f}）"
    if kd > 0 and qd == 0:
        msg += f"  ⚠ Kd が 0 に潰れています（1LSB={r['KD'][1] / 4095:.5f}）"
    return msg


def f_mit_mode(motor_id, pos, vel, kp, kd, tau, model=MODEL, mode=None):
    """MITペイロードを (mode<<8)|ID の拡張フレームに載せる"""
    m = MODE_ID if mode is None else mode
    return cm.Frame((m << 8) | (motor_id & 0xFF),
                    pack_mit(kp, kd, pos, vel, tau, model), True)


def f_mode_raw(motor_id, data, mode=None):
    """同じIDに任意の8バイトを載せる（ペイロード仕様が違ったとき用）"""
    m = MODE_ID if mode is None else mode
    return cm.Frame((m << 8) | (motor_id & 0xFF), bytes(data), True)


def f_zero(motor_id, model=MODEL):
    """
    安全な後始末用。全部ゼロ＝何もするな。

    正しい並び（Kpが先頭）なら Kp=0 / Kd=0 になるので、位置と速度の欄が
    何であってもモーターは無視する。トルクだけが残り、それも0。
    ＝これが本当に無害な指令。
    ※ 半LSBのズレは残る（トルク -0.0044 N*m 程度）。動摩擦 0.22 N*m の
      2% なので軸は動かないが、電流が完全な0にならない理由はこれ。
    """
    return f_mit_mode(motor_id, 0.0, 0.0, 0.0, 0.0, 0.0, model)


# ============================================================
# 状態の読み取り（サーボ形式とMIT形式で単位が違う）
# ============================================================
def pos_rad(s):
    """状態の位置を rad に揃える。サーボ形式は deg、MIT形式は rad"""
    if s is None:
        return None
    return s.pos * DEG2RAD if s.src != "mit" else s.pos


def cur_a(s):
    """サーボ形式の cur は A。MIT形式は N*m なので、電流として使えるのはサーボ側だけ"""
    if s is None:
        return None
    return s.cur if s.src != "mit" else None


def fmt_state(s):
    if s is None:
        return "  状態なし（フィードバック未受信）"
    if s.src == "mit":
        return (f"  ID={s.id:3d}  pos={s.pos:8.4f} rad ({s.pos * RAD2DEG:7.1f} deg)  "
                f"spd={s.spd:7.2f} rad/s  T={s.cur:6.2f} N*m  "
                f"temp={s.temp if s.temp is not None else -1:3d}C  err={s.err}  "
                f"[MIT]  ({s.age() * 1000:.0f}ms前)")
    return (f"  ID={s.id:3d}  pos={s.pos:8.1f} deg  spd={s.spd:8.0f} ERPM  "
            f"cur={s.cur:6.3f} A  temp={s.temp:3d}C  err={s.err}  "
            f"[servo]  ({s.age() * 1000:.0f}ms前)")


# ============================================================
# 送信（バス健全性を見ながら）
# ============================================================
class Tee:
    """画面とファイルの両方に出す。今日しか取れない出力を取りこぼさないため"""

    def __init__(self, stream, fh):
        self.stream = stream
        self.fh = fh

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


class BusTrouble(RuntimeError):
    """バスやアダプタの異常。電源/USBの入れ直しが要る"""


class SafetyAbort(RuntimeError):
    """
    指令は届いているが安全しきい値に当たって止めた場合。
    ⚠ こちらは電源を入れ直す必要がない。同じ文言を出すと、要らない
      電源断を毎回させることになる（実際そうなっていた）。
    """


def burst(bus, motor_id, make_frame, seconds, hz=SEND_HZ,
          on_sample=None, sample_hz=50, max_age=MAX_AGE, cur_abort=CUR_ABORT,
          spd_abort=SPD_ABORT_ERPM):
    """
    make_frame(t) を hz で送り続けながら状態を記録する。
    戻り値: rows = [(t, pos_rad, spd, cur_or_torque, src, 受信時刻)]

    ⚠ 中断する条件が4つある。2026-09-14 の事故を受けて追加:
      ① フィードバックが max_age 秒途切れた
         → 観測できない状態で送り続けると暴走に気づけない。実際に
           「電流0.00A・位置不変」と表示されたまま実機は回っていた。
      ② 電流が cur_abort を超えた（原点ズレで 16.5A / 39.7A が出た）
      ②' 速度 or 開始位置からの移動量が大きすぎる
         → 回っている間は電流が出ない。t 0.6 で 42,900 ERPM・6回転したとき
           電流はほぼ0で、電流ガードは働かなかった。
      ③ エコー数/送信数 が ECHO_RATIO_ABORT 超（ACKなし再送＝bus-off前兆）
      ④ 受信スレッドが例外で停止した
    """
    rows = []
    grace = 0.4          # 起動直後、状態が来るまでの猶予
    period = 1.0 / hz
    sample_period = (1.0 / sample_hz) if sample_hz else None
    t0 = time.time()
    next_send = t0
    next_sample = t0
    tx0 = bus.tx_count
    echo0 = bus.echo_count
    n = 0
    trouble = None
    safety = False       # True なら安全中断（電源入れ直し不要）
    pos_start = None

    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if elapsed >= seconds:
                break

            if now >= next_send:
                bus.send(make_frame(elapsed))
                n += 1
                next_send += period

                # --- ① 観測と過電流（最優先）---
                # ⚠ 過電流の判定に猶予を与えてはいけない。過大トルクは送り始めた
                #   その瞬間に立つので、猶予0.4秒の間に一番危ない山が過ぎてしまう。
                #   猶予が要るのは「まだ状態が届いていない」場合だけ。
                sw = bus.state(motor_id)
                fresh = sw is not None and sw.age() <= max_age
                if fresh:
                    # --- 速度と移動量（電流ガードをすり抜ける暴走への備え）---
                    if pos_start is None:
                        pos_start = pos_rad(sw)
                    moved = abs(pos_rad(sw) - pos_start) * RAD2DEG
                    if moved > MOVE_ABORT_DEG:
                        trouble = (f"開始位置から {moved:.0f} deg 動きました"
                                   f"（中断しきい値 {MOVE_ABORT_DEG:.0f} deg）")
                        safety = True
                        break
                    if sw.src == "mit":
                        if abs(sw.spd) > SPD_ABORT_RADS:
                            trouble = (f"速度が {sw.spd:+.1f} rad/s に達しました"
                                       f"（中断しきい値 {SPD_ABORT_RADS} rad/s）")
                            safety = True
                            break
                    elif abs(sw.spd) > spd_abort:
                        trouble = (f"速度が {sw.spd:+.0f} ERPM "
                                   f"({sw.spd / 31.5:+.0f} deg/s) に達しました"
                                   f"（中断しきい値 {spd_abort} ERPM）")
                        safety = True
                        break
                    # cur の意味が応答形式で違う。servo なら A、MIT なら N*m。
                    if sw.src == "mit":
                        if abs(sw.cur) > TAU_ABORT:
                            trouble = (f"トルクが {sw.cur:+.1f} N*m に達しました"
                                       f"（中断しきい値 {TAU_ABORT} N*m）")
                            safety = True
                            break
                    elif abs(sw.cur) > cur_abort:
                        trouble = (f"電流が {sw.cur:+.1f} A に達しました"
                                   f"（中断しきい値 {cur_abort} A）")
                        safety = True
                        break
                elif elapsed > grace:
                    age = sw.age() if sw else float("inf")
                    trouble = (f"フィードバックが {age:.2f} 秒途切れました。"
                               "観測できない状態で指令を送り続けません")
                    break

                # --- ③ バス健全性 ---
                if n >= 10:
                    sends = bus.tx_count - tx0
                    echoes = bus.echo_count - echo0
                    if sends > 0 and echoes / sends > ECHO_RATIO_ABORT:
                        trouble = (f"送信 {sends} 件に対してエコー {echoes} 件。"
                                   f"ACKが返らず再送を繰り返しています（bus-off の前兆）")
                        break
                if bus.rx_error:
                    trouble = f"受信スレッドが停止: {bus.rx_error}"
                    break

            if sample_period and now >= next_sample:
                s = bus.state(motor_id)
                if s is not None:
                    # 受信時刻も残す。これが変わらない＝新しい状態が来ていない＝
                    # 表示の数字は指令前の残骸、という判定に使う
                    rows.append((elapsed, pos_rad(s), s.spd, s.cur, s.src, s.t))
                if on_sample:
                    on_sample(elapsed, s)
                next_sample += sample_period

            time.sleep(0.002)
    except KeyboardInterrupt:
        print("  中断")
    finally:
        # 後始末は必ず零トルク。ここを省くと最後の指令が残る
        for _ in range(3):
            try:
                bus.send(f_zero(motor_id))
            except Exception:
                break
            time.sleep(0.01)

    if trouble:
        raise (SafetyAbort if safety else BusTrouble)(trouble)
    return rows


def stats(rows, t_from=0.0):
    sel = [r for r in rows if r[0] >= t_from]
    if not sel:
        return None
    pos = [r[1] for r in sel]
    cur = [r[3] for r in sel]
    n = len(sel)
    mean_c = sum(cur) / n
    # 受信時刻の種類数。1 なら「1度も更新されていない＝観測できていない」
    fresh = len({r[5] for r in sel}) if len(sel[0]) > 5 else len(sel)
    return dict(
        n=n, fresh=fresh,
        pos_first=pos[0], pos_last=pos[-1],
        pos_p2p=max(pos) - min(pos),
        cur_mean=mean_c,
        cur_max=max(cur, key=abs),
        cur_p2p=max(cur) - min(cur),
        cur_std=(sum((c - mean_c) ** 2 for c in cur) / n) ** 0.5,
        spd_max=max((abs(r[2]) for r in sel)),
        src=sel[-1][4],
    )


HELP = f"""
motor_console_ver6 -- MITを「制御モード8の拡張フレーム」で送る検証用

送信ID = (モード番号 << 8) | モーターID の拡張29bit。既定はモード8。
  ID=34 なら 0x822。これは v5 の mscheme ext と同じ値なので、
  バスが落ちないか監視しながら送っています。

見る
  s                 状態を1行
  m 3               3秒 流し見（指令しない）
  id 34             対象モーターID
  model AK80-9      機種（MITのトルクレンジが変わる）
  hz 5              定期フィードバックのレートを5秒測る
  o 0               今いる位置を原点に定義し直す（軸は動かない）
                    ※位置は±3276.7degで頭打ち。張り付くと移動量ガードが効かない
  raw on            生フレーム表示（raw off で戻す）
  mrx on            MIT形式の応答を解釈する（既定ON。off にすると見えなくなる）
  mzero             FF..FE を送る前に、それがどう読まれるかを表示（既定では送らない）
  health            バス健全性（送信/エコー比・受信スレッド）
  log on            この画面の内容をファイルに残す（log off で終了）
  mode 8            使う制御モード番号を変える（既定8）
  preview 0 0 0 0 0.3   その指令の8バイトと、解き直した値を表示（送信しない）
  explain 7f ff 7f f0 00 00 08 21   そのバイト列をモーターがどう読むか
  arm off           送信ロックをかける（安全側に倒したいとき）

MIT指令（すべて指定秒で自動終了。終了時に零トルクを送る）
  t 0.3 2           トルクだけ 0.3 N*m を2秒（pos/vel/kp/kd は0）
  m8 0 0 5 0.5 0 2  pos[rad] vel[rad/s] Kp Kd トルク 秒 をそのまま指定
  raw8 7f ff 7f f0 00 00 07 ff    任意の8バイトをモード8で1回だけ送る

実験
  sweep             トルクを 0.1→0.5 N*m と上げて各1.5秒、電流を測る（Kt推定）
  brk               0.28→0.44 N*m を0.02刻みで、動き出すトルクを挟む（Kt非依存）
  morg              MITの位置原点を実測（Kp=0.25）。位置指令の前に必ず1回
  morg 0.5 2        Kp と振り幅を指定（当てはまりが悪いと言われたらKpを倍に）
  hold 5 0.5 3      今の角度を Kp=5 Kd=0.5 で3秒保持（Kd=0 は禁止）
  ramp 3            Kp を 2→5→10→15 と上げて各3秒保持（リミットサイクル探し）
  step 90 5 0.5 3   今の角度から +90度 へ Kp=5 Kd=0.5、3秒記録

その他
  x                 停止（零トルク＋サーボ速度0）
  q                 終了

安全上限  トルク {LIM_TAU} N*m / Kp {LIM_KP} / Kd {LIM_KD} / 1コマンド {LIM_SEC} 秒 / ステップ {LIM_STEP_DEG} deg
自動中断  観測 {MAX_AGE} 秒途切れ / 電流 {CUR_ABORT} A 超 / 速度 {SPD_ABORT_ERPM} ERPM 超 / 移動 {MOVE_ABORT_DEG} deg 超 / バス異常
★ ペイロードはマニュアル44-45頁の並び（Kp → Kd → 位置 → 速度 → トルク）。
   classic MIT（位置が先頭）とは違います。この違いで 2026-09-14 に 48.7A が出ました。
   Kp と Kd が先頭にあるので、Kp=Kd=0 なら位置・速度は無視されます＝零指令が安全。
★ 位置指令（Kp>0 の hold/ramp/step/m8）は、morg で原点を実測するまで実行しません。
   サーボの角度はMITと原点が別なので、そのまま目標にすると過大トルクになります
   （2026-09-14 実測: Kp=2 で 16.5A、Kp=5 のステップで 39.7A）。
   MIT専用の応答フレームは存在せず、サーボ形式の50Hz定期が流れ続けるので、
   旧条件「s に [MIT] と出るまで待つ」は永久に満たされませんでした。
   トルクだけの t / sweep は Kp=0 なので影響を受けません。
実測の参考  静止摩擦 0.30〜0.32 N*m / 動摩擦 0.22 N*m / Kt 0.59 N*m/A（2026-09-14 実測）
★ ゲインの量子化: Kp は 1LSB=0.1221（0.06未満は0）/ Kd は 1LSB=0.00122。
   Kp=0.05 は 0 に潰れます。2026-09-14、これで morg が空振りしました。
※ x はソフト停止です。本当の非常停止は電源を切ること。
"""


class Console:
    def __init__(self):
        self.bus = cm.MotorBus(channel=CHANNEL, bitrate=BITRATE, model=MODEL)
        # ⚠ 既定でON。OFFだとMIT形式の応答が黙って捨てられ、状態が更新されないまま
        #   「電流0.00A」と表示され続ける。2026-09-14 はこれで実機の暴走を見逃した。
        self.bus.mit_rx = True
        self.target = DEFAULT_MOTOR_ID
        self.model = MODEL
        # MIT座標系の位置 = サーボ角[rad] + mit_offset。morg で実測する
        self.mit_offset = None

    # ---- 表示 ----
    def show(self):
        s = self.bus.state(self.target)
        print(fmt_state(s))
        if s is not None and s.src != "mit" and abs(s.pos) > 3100:
            print(f"  ⚠ 位置 {s.pos:.1f} deg は頭打ち（±3276.7 deg）に近い/張り付いています。")
            print("    位置が読めないと移動量ガードも効きません。o 0 で原点を戻してください。")

    def tick(self, t, s):
        if s is None:
            return
        p = pos_rad(s)
        c = s.cur
        unit = "A" if s.src != "mit" else "N*m"
        print(f"  t={t:5.2f}s  pos={p * RAD2DEG:+8.2f} deg  "
              f"spd={s.spd:+9.1f}  cur={c:+7.3f} {unit}  [{s.src}]")

    def check_lock(self):
        if not SEND_LOCKED:
            return True
        print(f"  送信ロック中: {LOCK_NOTE}")
        print("  送るバイト列は preview で確認できます（送信しません）。")
        print("  仕様が分かったら arm で一時解除できますが、解除しても")
        print(f"  電流 {CUR_ABORT}A / 観測途切れ {MAX_AGE}秒 のガードは効いたままです。")
        return False

    def require_feedback(self):
        """観測できない指令は打たない。フィードバックが無ければ実行を拒否する"""
        s = self.bus.state(self.target)
        if s is None or not s.alive(0.5):
            print(f"  中止: ID={self.target} のフィードバックが来ていません。")
            print("  指令しても効いたかどうか観測できないので実行しません。")
            print("  id を確認し、s で状態が出ることを確かめてください。")
            print("  ※ モード8を送るとサーボの定期フィードバックが止まる場合があります。")
            print("    mrx on でMIT形式の応答を解釈できるか試してください。")
            return False
        return True

    def mit_target(self, s):
        """サーボ角[rad] を MIT座標系の目標位置に直す"""
        return pos_rad(s) + (self.mit_offset or 0.0)

    def require_mit_position(self, kp=None):
        """
        位置指令（Kp>0）を許す条件。

        ⚠ 2026-09-14 の事故: サーボの角度[deg]を rad に直して MIT の目標位置へ
          そのまま入れていた。MITの位置原点はサーボの原点と別物なので、
          159deg = 2.775rad がまるごと偏差になり、Kp=2 で 16.5A、
          Kp=5 のステップで 39.7A が流れた（Bus Current Limit は 35A）。

        ⚠ 当初この条件を「s に [MIT] と出たら許可」にしていたが、これは誤り。
          MIT専用の応答フレームは存在せず、MITで制御していても
          サーボ形式の50Hz定期フィードバックが流れ続ける（2026-09-14 実測）。
          つまり旧条件は永久に満たされず、位置指令が一生使えなかった。
          正しい条件は「morg でMITの位置原点を実測してあること」。
        """
        s = self.bus.state(self.target)
        if s is not None and s.src == "mit":
            return True
        if self.mit_offset is not None:
            return True
        print("  中止: 位置指令（Kp>0）は、MITの位置原点が分かるまで実行しません。")
        print("  サーボの角度とMITの原点は別物で、その差がまるごと偏差になります")
        print("  （実測: Kp=2 で 16.5A、Kp=5 のステップで 39.7A）。")
        print("  先に morg を実行してください。Kp=0.05 で原点だけを実測します。")
        print("  トルクだけの指令（t 0.3 2 / sweep / brk）は Kp=0 なので今でも使えます。")
        return False

    def run_cmd(self, label, make_frame, seconds, tick_hz=5.0,
                spd_abort=SPD_ABORT_ERPM):
        """1コマンドぶんの送信。戻り値 rows"""
        print(f"  {label} を {SEND_HZ}Hz で {seconds:.1f} 秒 送ります")
        state = {"next": 0.0}
        step = 1.0 / tick_hz if tick_hz else None

        def on_sample(t, s):
            if step and t >= state["next"]:
                self.tick(t, s)
                state["next"] = t + step

        try:
            return burst(self.bus, self.target, make_frame, seconds,
                         on_sample=on_sample, spd_abort=spd_abort)
        except SafetyAbort as e:
            print(f"  ■ 安全しきい値で停止: {e}")
            print("  零トルクを送って止めています。電源の入れ直しは要りません。")
            return None
        except BusTrouble as e:
            print(f"  ⚠ バス異常で中断: {e}")
            print("  電源とUSBを入れ直してから再接続してください。")
            return None

    # ---- メインループ ----
    def loop(self):
        global MODE_ID, SEND_LOCKED     # mode / arm で実行時に変えるため
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

            def num(i, d=0.0):
                try:
                    return float(p[i])
                except (IndexError, ValueError):
                    return d

            def integer(i, d=0):
                try:
                    return int(p[i])
                except (IndexError, ValueError):
                    return d

            mid = self.target

            try:
                if c == "q":
                    break
                elif c in ("?", "h", "help"):
                    print(HELP)
                elif c == "s":
                    self.show()
                elif c == "m":
                    t0 = time.time()
                    dur = num(1, 3.0)
                    while time.time() - t0 < dur:
                        self.show()
                        time.sleep(0.2)
                elif c == "id":
                    self.target = integer(1, self.target)
                    print(f"  対象を ID={self.target} にしました "
                          f"（モード{MODE_ID}の送信IDは 0x{(MODE_ID << 8) | self.target:X}）")
                elif c == "model":
                    name = p[1] if len(p) > 1 else ""
                    if name not in MIT_RANGES:
                        print(f"  未知のモデル。選べるのは {list(MIT_RANGES)}")
                        continue
                    self.model = name
                    if name in cm.MODELS:
                        self.bus.model = name
                    r = MIT_RANGES[name]
                    print(f"  モデル: {name}  位置 ±{r['P'][1]} rad / "
                          f"速度 ±{r['V'][1]} rad/s / トルク ±{r['T'][1]} N*m / "
                          f"Kp 0〜{r['KP'][1]} / Kd 0〜{r['KD'][1]}")
                elif c == "mode":
                    MODE_ID = integer(1, MODE_ID)
                    print(f"  制御モード番号を {MODE_ID} にしました "
                          f"（送信ID 0x{(MODE_ID << 8) | mid:X}）")
                elif c == "raw":
                    self.bus.raw_log = (len(p) > 1 and p[1].lower() == "on")
                    print(f"  生ログ: {'ON' if self.bus.raw_log else 'OFF'}")
                elif c == "mrx":
                    self.bus.mit_rx = not (len(p) > 1 and p[1].lower() == "off")
                    print(f"  MIT形式の応答の解釈: "
                          f"{'ON' if self.bus.mit_rx else 'OFF'}")
                    print("  ※ OFFにすると、MIT応答が来ていても状態が更新されません。"
                          "調査目的以外でOFFにしないでください。")
                elif c == "mzero":
                    if not self.check_lock():
                        continue
                    data = bytes([0xFF] * 7 + [0xFE])
                    b = unpack_mit(data, self.model)
                    print(f"  ⚠ {data.hex(' ')} は classic MIT の『ゼロ点設定』ですが、")
                    print(f"    モード{MODE_ID}のペイロードとして読むとこうなります:")
                    print(f"      Kp={b['kp']:.0f}  Kd={b['kd']:.2f}  "
                          f"位置={b['pos']:+.2f} rad  速度={b['vel']:+.0f} rad/s  "
                          f"トルク={b['tau']:+.2f} N*m")
                    print("    モード8が特殊フレームとして解釈してくれる保証はありません。")
                    print("    データとして読まれたら全力で回ります（48.7Aの再現）。")
                    if not (len(p) > 1 and p[1].lower() == "yes"):
                        print("  送信しません。理解した上で送るなら mzero yes")
                        continue
                    print("  送信します。")
                    self.bus.send(f_mode_raw(mid, data), quiet=False)
                    time.sleep(0.3)
                    self.show()
                elif c == "preview":
                    pos, vel, kp, kd, tau = num(1), num(2), num(3), num(4), num(5)
                    fr = f_mit_mode(mid, pos, vel, kp, kd, tau, self.model)
                    back = unpack_mit(fr.data, self.model)
                    print(f"  送信するとしたら: {fr}")
                    print(f"    ID = 0x{fr.arbitration_id:X}（拡張29bit）  "
                          f"データ = {fr.data.hex(' ')}")
                    print( "    詰めて解き直した値（モーターが読む値）:")
                    print(f"      Kp={back['kp']:.2f}  Kd={back['kd']:.3f}  "
                          f"位置={back['pos']:+.4f} rad ({back['pos'] * RAD2DEG:+.1f} deg)  "
                          f"速度={back['vel']:+.2f} rad/s  トルク={back['tau']:+.3f} N*m")
                    if back["kp"] > 0.5 or back["kd"] > 0.05:
                        print("      ⚠ ゲインが乗っています。位置・速度の欄が効きます。")
                    else:
                        print("      ※ Kp=Kd=0 なので位置と速度は無視されます。"
                              "トルクだけが効きます。")
                elif c == "explain":
                    try:
                        d = bytes(int(x, 16) for x in p[1:])
                    except ValueError:
                        d = b""
                    if len(d) != 8:
                        print("  使い方: explain 7f ff 7f f0 00 00 08 21  （8バイト）")
                        continue
                    b = unpack_mit(d, self.model)
                    print(f"  {d.hex(' ')} をモーターはこう読みます（{self.model}）:")
                    print(f"    Kp={b['kp']:.2f}  Kd={b['kd']:.3f}  "
                          f"位置={b['pos']:+.4f} rad ({b['pos'] * RAD2DEG:+.1f} deg)  "
                          f"速度={b['vel']:+.2f} rad/s  トルク={b['tau']:+.3f} N*m")
                elif c == "arm":
                    global SEND_LOCKED
                    if len(p) > 1 and p[1].lower() == "off":
                        SEND_LOCKED = True
                        print("  送信ロック: ON（安全側）")
                    else:
                        print(f"  ⚠ {LOCK_NOTE}")
                        print("  本当に解除するなら arm yes と打ってください。")
                        if len(p) > 1 and p[1].lower() == "yes":
                            SEND_LOCKED = False
                            print("  送信ロック: OFF。電流ガードは効いたままです。")
                elif c == "o":
                    # サーボの SET_ORIGIN（制御モード5）。今いる物理位置を 0 と
                    # 定義し直すだけで軸は動かない。
                    # ⚠ 位置は int16/10 なので ±3276.7 deg で頭打ちする。
                    #   張り付くと位置が読めず、移動量ガードも効かなくなる。
                    kind = integer(1, 0)
                    fr = cm.f_set_origin(mid, kind)
                    print(f"  送信 {fr}  （{'恒久' if kind == 1 else '一時'}原点）")
                    print("  ※ 今いる位置を0と定義し直すだけです。軸は動きません。")
                    self.bus.send(fr, quiet=False)
                    time.sleep(0.3)
                    self.show()
                elif c == "log":
                    if p[1:2] and p[1] == "off":
                        if isinstance(sys.stdout, Tee):
                            fh = sys.stdout.fh
                            sys.stdout = sys.stdout.stream
                            fh.close()
                            print("  記録を終了しました")
                        else:
                            print("  記録していません")
                    else:
                        if isinstance(sys.stdout, Tee):
                            print("  すでに記録中です（log off で終了）")
                        else:
                            fn = (p[1] if len(p) > 1 and p[1] != "on"
                                  else "bench_"
                                  + time.strftime("%Y%m%d_%H%M%S") + ".txt")
                            sys.stdout = Tee(sys.stdout,
                                             open(fn, "a", encoding="utf-8"))
                            print(f"  この画面の内容を {fn} に記録します")
                elif c == "health":
                    sends = self.bus.tx_count
                    echoes = self.bus.echo_count
                    r = (echoes / sends) if sends else 0.0
                    print(f"  送信 {sends} 件 / 除外したエコー {echoes} 件 "
                          f"（比 {r:.2f}、正常は1前後）")
                    print(f"  受信 {self.bus.rx_count} 件 / "
                          f"受信スレッド: {self.bus.rx_error or '正常'}")
                elif c == "hz":
                    sec = num(1, 2.0)
                    rows = self.bus.feedback_hz(seconds=sec)
                    if not rows:
                        print("  1フレームも受信していません。")
                        continue
                    for arb, m_id, nn, hz, dlc, ext, last in rows:
                        print(f"  ID=0x{arb:08X} (ID={m_id})  {nn:5d}フレーム  "
                              f"{hz:6.1f} Hz  DLC={dlc}  "
                              f"{'拡張' if ext else '標準'}  最後={last.hex(' ')}")

                # ---------- MIT 指令 ----------
                elif c == "t":
                    tau, sec = num(1, 0.1), num(2, 2.0)
                    if abs(tau) > LIM_TAU or sec > LIM_SEC:
                        print(f"  拒否: |トルク|≤{LIM_TAU} / 秒数≤{LIM_SEC}")
                        continue
                    if not self.check_lock():
                        continue
                    if not self.require_feedback():
                        continue
                    if abs(tau) > 0.54:
                        print(f"  ⚠ {tau:.2f} N*m は静止摩擦の実測 0.54 N*m を超えます。"
                              "軸が回り出します。")
                    rows = self.run_cmd(
                        f"モード{MODE_ID} トルク {tau:+.2f} N*m",
                        lambda t: f_mit_mode(mid, 0, 0, 0, 0, tau, self.model), sec)
                    self.report(rows, expect=f"トルク {tau:+.2f} N*m")

                elif c == "m8":
                    pos, vel, kp, kd, tau = num(1), num(2), num(3), num(4), num(5)
                    sec = num(6, 2.0)
                    if (kp > LIM_KP or kd > LIM_KD or abs(tau) > LIM_TAU
                            or sec > LIM_SEC):
                        print(f"  拒否: Kp≤{LIM_KP} Kd≤{LIM_KD} "
                              f"|トルク|≤{LIM_TAU} 秒≤{LIM_SEC}")
                        continue
                    if not self.check_lock():
                        continue
                    if not self.require_feedback():
                        continue
                    if kp > 0 and not self.require_mit_position():
                        continue
                    s0 = self.bus.state(mid)
                    if kp > 0:
                        jump = abs(pos - pos_rad(s0)) * RAD2DEG
                        if jump > LIM_STEP_DEG:
                            print(f"  拒否: 今の角度から {jump:.0f} deg 離れた目標です"
                                  f"（上限 {LIM_STEP_DEG:.0f}）。step を使ってください")
                            continue
                    rows = self.run_cmd(
                        f"モード{MODE_ID} pos={pos:.3f}rad Kp={kp:g} Kd={kd:g} "
                        f"tau={tau:+.2f}",
                        lambda t: f_mit_mode(mid, pos, vel, kp, kd, tau, self.model),
                        sec)
                    self.report(rows)

                elif c == "raw8":
                    try:
                        data = bytes(int(x, 16) for x in p[1:])
                    except ValueError:
                        print("  使い方: raw8 7f ff 7f f0 00 00 07 ff")
                        continue
                    if len(data) != 8:
                        print(f"  拒否: 8バイト必要です（今 {len(data)} バイト）")
                        continue
                    fr = f_mode_raw(mid, data)
                    if not self.check_lock():
                        continue
                    b = unpack_mit(data, self.model)
                    print(f"  このバイト列はこう読まれます: Kp={b['kp']:.1f} "
                          f"Kd={b['kd']:.2f} 位置={b['pos']:+.3f} rad "
                          f"速度={b['vel']:+.1f} rad/s トルク={b['tau']:+.3f} N*m")
                    if b["kp"] > LIM_KP or b["kd"] > LIM_KD:
                        print(f"  拒否: 解釈後の Kp/Kd が上限（{LIM_KP}/{LIM_KD}）を超えます。")
                        print("  そのまま送ると過大トルクになります。")
                        continue
                    print(f"  送信 {fr}")
                    self.bus.send(fr, quiet=False)

                # ---------- 実験 ----------
                elif c == "morg":
                    self.morg(mid, num(1, MORG_KP), num(2, 1.5))
                elif c == "sweep":
                    self.sweep(mid)
                elif c == "brk":
                    self.brk(mid, num(1, 0.28), num(2, 0.44), num(3, 0.02))
                elif c == "hold":
                    self.hold(mid, num(1, 5.0), num(2, 0.5), num(3, 3.0))
                elif c == "ramp":
                    self.ramp(mid, num(1, 3.0), num(2, 0.5))
                elif c == "step":
                    self.step(mid, num(1, 90.0), num(2, 5.0), num(3, 0.5),
                              num(4, 3.0), p[5] if len(p) > 5 else None)

                elif c == "x":
                    for _ in range(3):
                        self.bus.send(f_zero(mid))
                        self.bus.send(cm.f_velocity(mid, 0))
                        time.sleep(0.01)
                    print("  停止: 零トルク(mode8) + サーボ速度0 を送りました")
                    print("  ※ ソフト停止です。本当の非常停止は電源を切ること。")
                else:
                    print("  不明なコマンド。? でヘルプ")
            except Exception as e:
                print(f"  コマンド実行エラー: {type(e).__name__}: {e}")

    # ---- 結果表示 ----
    def report(self, rows, expect=None):
        if not rows:
            return
        # 過渡を外したいが、短いコマンドで固定値にすると全サンプルが捨てられて
        # 何も表示されなくなる。長さに応じて決める。
        st = stats(rows, t_from=min(0.3, rows[-1][0] * 0.3))
        if st is None:
            print("  サンプルが取れませんでした")
            return
        print("  --- 結果 ---")
        print(f"  位置 {st['pos_first'] * RAD2DEG:+.2f} → "
              f"{st['pos_last'] * RAD2DEG:+.2f} deg "
              f"(変化 {(st['pos_last'] - st['pos_first']) * RAD2DEG:+.2f})")
        unit = "A" if st["src"] != "mit" else "N*m"
        print(f"  電流 平均 {st['cur_mean']:+.3f} {unit} / "
              f"最大 {st['cur_max']:+.3f} / 振れ幅 {st['cur_p2p']:.3f} / "
              f"標準偏差 {st['cur_std']:.3f}")
        print(f"  応答の形式: [{st['src']}]  "
              f"（{st['n']} サンプル中 {st['fresh']} 回だけ状態が更新された）")
        if st["fresh"] <= 1:
            print("  ⚠ 送信中ずっと状態が更新されていません。上の数字は指令前の残骸で、")
            print("    実機が何をしていたかは分かりません。『電流0＝効いていない』とは")
            print("    読まないでください（見えないまま軸が回っていた実績があります）。")
        elif (expect and "+0.00 N*m" not in expect
                and abs(st["cur_mean"]) < 0.03):
            print(f"  ※ {expect} を送ったのに電流がノイズ範囲（±0.05A）のままです。"
                  "＝指令が受け付けられていません。")

    # ---- 実験1: トルク掃引（Ktの実測） ----
    def sweep(self, mid, taus=(0.1, 0.2, 0.3, 0.4, 0.5), sec=1.5):
        if not self.check_lock():
            return
        if not self.require_feedback():
            return
        print(f"  トルクを {', '.join(f'{t:g}' for t in taus)} N*m と上げて各 {sec:.1f} 秒。")
        print("  ⚠ 0.54 N*m（静止摩擦の実測）を超えると回り出します。")
        out = []
        for tau in taus:
            rows = self.run_cmd(f"トルク {tau:.2f} N*m",
                                lambda t, q=tau: f_mit_mode(mid, 0, 0, 0, 0, q,
                                                            self.model),
                                sec, tick_hz=0)
            if rows is None:
                return
            st = stats(rows, t_from=0.3)
            if st is None:
                continue
            moved = (st["pos_last"] - st["pos_first"]) * RAD2DEG
            out.append((tau, st))
            # ⚠ 位置は ±3276.7 deg で頭打ちする。張り付いていると「動いていない」
            #   ように見えるので、動いたかどうかは速度で判定する。
            spinning = st["spd_max"] > (200 if st["src"] != "mit" else 0.2)
            unit = "ERPM" if st["src"] != "mit" else "rad/s"
            print(f"    {tau:.2f} N*m → 電流 {st['cur_mean']:+.3f} A / "
                  f"最大速度 {st['spd_max']:.0f} {unit} / 位置変化 {moved:+.2f} deg")
            # ⚠ 動き出したらそこで打ち切る。静止摩擦を超えると回り続けるので、
            #   それ以上トルクを上げても「止まっている状態の電流」は測れない。
            #   動き出した点そのものが、Kt に依存しない静止摩擦の実測値になる。
            if spinning or abs(moved) > 2.0:
                prev = out[-2][0] if len(out) > 1 else 0.0
                print(f"  ★ ここで動き出しました。静止摩擦は "
                      f"{prev:.2f} 〜 {tau:.2f} N*m の間です。")
                print("    （この値は電流を経由しないので Kt が未確定でも信用できます）")
                break
            time.sleep(0.3)
        print("  --- トルク指令 vs 実電流 ---")
        print("    指令[N*m]   実電流[A]   Kt_out[N*m/A]")
        for tau, st in out:
            kt = tau / st["cur_mean"] if abs(st["cur_mean"]) > 0.01 else float("nan")
            print(f"    {tau:8.2f}   {st['cur_mean']:9.3f}   {kt:12.3f}")
        print("  ※ 比例していれば、この Kt_out が分銅なしで得られた実測値になります")
        print("    （bench の kt は分銅未装着で3回とも無効だった項目）")

    # ---- 実験1b: 静止摩擦を細かく挟む ----
    def brk(self, mid, lo=0.28, hi=0.44, step=0.02, sec=1.2):
        """
        動き出すトルクを細かく探す。

        この値は電流を経由しないので Kt が未確定でも信用できる。
        従来の 0.54 N*m は「起動電流0.63A × データシートのKt」で逆算した値で、
        Kt が違えば連動して狂う。こちらが一次データになる。
        """
        if not self.check_lock() or not self.require_feedback():
            return
        s0 = self.bus.state(mid)
        if s0.src != "mit" and abs(s0.pos) > 3100:
            print("  中止: 位置が頭打ちです。先に o 0 を打ってください。")
            return
        n = int(round((hi - lo) / step)) + 1
        print(f"  {lo:.2f} → {hi:.2f} N*m を {step:.2f} 刻みで各 {sec:.1f} 秒。"
              f"動き出したら止めます（最大{n}段）")
        last_still = None
        for i in range(n):
            tau = lo + step * i
            if tau > LIM_TAU:
                break
            rows = self.run_cmd(f"トルク {tau:.2f} N*m",
                                lambda t, q=tau: f_mit_mode(mid, 0, 0, 0, 0, q,
                                                            self.model),
                                sec, tick_hz=0, spd_abort=1500)
            if rows is None:
                print(f"  {tau:.2f} N*m で動き出しました。")
                print(f"  ★ 静止摩擦は {last_still if last_still else lo:.2f} 〜 "
                      f"{tau:.2f} N*m の間")
                return
            st = stats(rows, t_from=min(0.3, sec * 0.3))
            if st is None:
                continue
            spinning = st["spd_max"] > (200 if st["src"] != "mit" else 0.2)
            print(f"    {tau:.2f} N*m → 電流 {st['cur_mean']:+.3f} A / "
                  f"最大速度 {st['spd_max']:.0f} / "
                  f"{'★動いた' if spinning else '静止'}")
            if spinning:
                print(f"  ★ 静止摩擦は {last_still if last_still else lo:.2f} 〜 "
                      f"{tau:.2f} N*m の間")
                return
            last_still = tau
            time.sleep(0.4)
        print(f"  {hi:.2f} N*m まで動きませんでした。hi を上げてください。")

    # ---- 実験0: MITの位置原点を実測する ----
    def morg(self, mid, kp=MORG_KP, span=1.5, npts=5, sec=0.8, kd=MORG_KD):
        """
        MIT座標系の位置と、サーボが返す角度との差（offset）を実測する。

        原理:
          MIT の発生トルクは   tau = Kp_eff * (X - theta_mit)
          フィードバック電流は  I   = tau / Kt   なので
              I = (Kp_eff / Kt) * ((X - theta_servo) - offset)
          目標 X を数点振って I を直線で当て、ゼロ交差を外挿する。
          ゼロ交差の位置は傾き（Kp_eff/Kt）に依存しないので、
          ★ Kt も Kp のスケールも未知のままで offset が求まる。
          軸が多少動いても、サンプルごとに theta_servo を読んで差を取るので効かない。

        既定の Kp=0.05 なら、偏差が 3.5 rad あってもトルクは静止摩擦（0.30 N*m）
        程度にしかならず、軸はほとんど動かない。電流も 1 A 未満に収まる。
        """
        if kp > MORG_KP_MAX:
            print(f"  拒否: morg の Kp は {MORG_KP_MAX} 以下にしてください")
            return
        if not self.check_lock():
            return
        if not self.require_feedback():
            return
        s = self.bus.state(mid)
        if s is None:
            return
        print(f"  MITの位置原点を実測します。Kp={kp:g} Kd={kd:g} / 目標を {npts} 点振ります")
        print("  " + gain_note(kp, kd, self.model))
        ctr = self.mit_offset
        print(f"  （現在のサーボ角 {pos_rad(s) * RAD2DEG:+.1f} deg / "
              f"中心 offset={0.0 if ctr is None else ctr:+.3f} rad の周り ±{span:g} rad）")
        print("  ⚠ 軸に手を触れないでください。わずかに動くことがあります。")
        samples = []
        for i in range(npts):
            d = -span + 2.0 * span * i / (npts - 1)
            s_now = self.bus.state(mid)
            if s_now is None:
                print("  中止: 状態が読めません")
                return
            # 2回目以降は前回の推定値を中心に振る＝外挿でなく内挿になり精度が上がる
            X = self.mit_target(s_now) + d
            rows = self.run_cmd(f"原点探索 {i + 1}/{npts} (d={d:+.2f} rad)",
                                lambda t, X=X: f_mit_mode(mid, X, 0, kp, kd, 0,
                                                          self.model),
                                sec, tick_hz=0)
            if rows is None:
                print("  中止しました")
                return
            got = 0
            for t, pr, sp, cu, src_, _ts in rows:
                if t >= 0.3 and src_ != "mit" and abs(sp) < MORG_SPD_GATE:
                    samples.append((X - pr, cu))
                    got += 1
            print(f"    {i + 1}/{npts}  d={d:+.2f} rad  "
                  f"電流 {sum(c for _u, c in samples[-got:]) / max(got, 1):+.3f} A "
                  f"({got} サンプル)")
            time.sleep(0.25)
        if len(samples) < 20:
            print("  中止: サンプルが足りません")
            return
        n = len(samples)
        su = sum(u for u, _c in samples)
        sc = sum(c for _u, c in samples)
        suu = sum(u * u for u, _c in samples)
        suc = sum(u * c for u, c in samples)
        den = n * suu - su * su
        if abs(den) < 1e-12:
            print("  中止: 目標がばらつかず直線が引けません")
            return
        a = (n * suc - su * sc) / den
        b = (sc - a * su) / n
        if abs(a) < 1e-6:
            print("  中止: 電流が目標位置に反応していません（Kp が小さすぎる可能性）")
            return
        off = -b / a
        mean_u = su / n
        mean_c = sc / n
        ss_tot = sum((c - mean_c) ** 2 for _u, c in samples)
        ss_res = sum((c - (a * u + b)) ** 2 for u, c in samples)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
        kp_eff = a * KT_OUT
        print("  --- 結果 ---")
        print(f"  offset = {off:+.4f} rad ({off * RAD2DEG:+.2f} deg)   "
              f"[MIT位置 = サーボ角 + offset]")
        print(f"  傾き {a:+.4f} A/rad / 直線の当てはまり R^2={r2:.4f} "
              f"({n} サンプル)")
        print(f"  → 実効Kp ≈ {kp_eff:.3f} N*m/rad（Kt={KT_OUT} 前提）"
              f" ＝ 指令Kp の {kp_eff / kp:.2f} 倍")
        if r2 < MORG_MIN_R2 or abs(a) < 0.02:
            print(f"  ■ 採用しません: 当てはまり R^2={r2:.3f} / 傾き {a:+.4f} A/rad")
            print("    電流が目標位置に反応していません。考えられる原因:")
            print(f"      ・Kp={kp:g} が小さすぎて量子化で潰れている"
                  f"（{gain_note(kp, kd, self.model)}）")
            print("      ・偏差が小さく、静止摩擦の中に埋もれている")
            print(f"    対処: morg {kp * 2:g} {span:g} で Kp を倍にして再実行。")
            print("    位置指令はロックしたままにします（ゴミの原点で動かすと暴走します）。")
            return
        self.mit_offset = off
        print("  位置指令（hold/ramp/step）が使えるようになりました。")
        if ctr is None:
            print("  ★ 1回目は外挿なので数degの誤差が残ります。もう一度 morg を実行して")
            print("     ください。2回目は今の推定値の周りを振るので内挿になり精度が上がります。")
        else:
            print(f"  前回からの変化 {(off - ctr) * RAD2DEG:+.2f} deg"
                  "（0.5deg以内に収まっていれば収束）")
        print("  確認: hold 1 0 2 で電流がほぼ0なら原点は合っています。")

    # ---- 実験2: 今の角度を保持 ----
    def hold(self, mid, kp, kd, sec, quiet=False):
        if kp > LIM_KP or kd > LIM_KD or sec > LIM_SEC:
            print(f"  拒否: Kp≤{LIM_KP} Kd≤{LIM_KD} 秒≤{LIM_SEC}")
            return None
        if not self.check_lock():
            return None
        if not self.require_feedback():
            return None
        if kp > 0 and kd <= 0:
            print("  拒否: Kp>0 で Kd=0 は禁止です。減衰が無いと行き過ぎて発振し、\n"
                  "        そのまま暴走します（2026-09-14 実測: Kp=1 Kd=0 で 277 deg/s）。\n"
                  "        Kd は 0.3 以上を入れてください。例: hold 1 0.5 2")
            return None
        if kp > 0 and not self.require_mit_position():
            return None
        s0 = self.bus.state(mid)
        tgt_servo = pos_rad(s0)
        tgt = self.mit_target(s0)
        if not quiet:
            print(f"  今の角度 {tgt_servo * RAD2DEG:+.2f} deg を目標に "
                  f"Kp={kp:g} Kd={kd:g} で {sec:.1f} 秒保持します")
            print("  " + gain_note(kp, kd, self.model))
            print("  ⚠ 目標が今の角度なので原則その場から動きませんが、"
                  "MITの原点がサーボの原点とズレていると引っ張られます。手を離してください。")
        rows = self.run_cmd(f"保持 Kp={kp:g} Kd={kd:g}",
                            lambda t: f_mit_mode(mid, tgt, 0, kp, kd, 0, self.model),
                            sec, tick_hz=(0 if quiet else 5))
        if rows is None:
            return None
        st = stats(rows, t_from=min(0.5, sec * 0.3))
        if st is None:
            return None
        drift = (st["pos_last"] - tgt_servo) * RAD2DEG
        if not quiet:
            print(f"  位置の振れ幅 {st['pos_p2p'] * RAD2DEG:.3f} deg / "
                  f"目標からのズレ {drift:+.2f} deg")
            print(f"  電流 平均 {st['cur_mean']:+.3f} A / "
                  f"振れ幅 {st['cur_p2p']:.3f} / 標準偏差 {st['cur_std']:.3f}")
            if abs(drift) > 2.0:
                print("  ※ ズレが大きい。MITの位置原点がサーボの角度とズレている可能性")
        return dict(kp=kp, kd=kd, st=st, drift=drift)

    # ---- 実験3: Kpランプ（リミットサイクル探し） ----
    def ramp(self, mid, sec=3.0, kd=0.5, kps=(2.0, 5.0, 10.0, 15.0)):
        if not self.require_feedback():
            return
        if not self.require_mit_position():
            return
        print(f"  Kp を {', '.join(f'{k:g}' for k in kps)} と上げて各 {sec:.1f} 秒保持。"
              f"Kd={kd:g} 固定")
        print("  ⚠ Kp を上げるほど保持が固くなります。軸から手を離してください。")
        out = []
        for kp in kps:
            r = self.hold(mid, kp, kd, sec, quiet=True)
            if r is None:
                return
            out.append(r)
            print(f"    Kp={kp:<5g} 位置振れ幅 {r['st']['pos_p2p'] * RAD2DEG:7.3f} deg / "
                  f"電流 平均 {r['st']['cur_mean']:+.3f} A 振れ幅 "
                  f"{r['st']['cur_p2p']:.3f} 標準偏差 {r['st']['cur_std']:.3f}")
            time.sleep(0.3)
        print("  --- 判定の目安 ---")
        print("   ・サーボモードでは90度保持で電流が 0.4↔1.0A を約5Hzで振動し続けた")
        print("   ・どのKpでも電流の振れ幅が小さいままなら、MITでリミットサイクルは出て")
        print("     いない＝C6（未解決事項）が解消したことになる")
        print("   ・Kpを上げた途端に振れ幅が跳ねるなら、そのKpが実機の上限")

    # ---- 実験4: ステップ応答 ----
    def step(self, mid, delta_deg, kp, kd, sec, csv=None):
        if (kp > LIM_KP or kd > LIM_KD or sec > LIM_SEC
                or abs(delta_deg) > LIM_STEP_DEG):
            print(f"  拒否: Kp≤{LIM_KP} Kd≤{LIM_KD} 秒≤{LIM_SEC} "
                  f"|ステップ|≤{LIM_STEP_DEG}deg")
            return
        if not self.check_lock():
            return
        if not self.require_feedback():
            return
        if not self.require_mit_position():
            return
        s0 = self.bus.state(mid)
        p0 = pos_rad(s0)
        p0m = self.mit_target(s0)
        tgt = p0 + delta_deg * DEG2RAD
        tgtm = p0m + delta_deg * DEG2RAD
        pre = 1.0
        print(f"  ★ 実機が動きます。今の角度 {p0 * RAD2DEG:+.1f} deg を1秒保持してから "
              f"{delta_deg:+.0f} deg 動かし、{sec:.1f} 秒記録します (Kp={kp:g} Kd={kd:g})")
        print("  脚・配線・手の位置を確認してください。")
        rows = self.run_cmd(
            f"ステップ {delta_deg:+.0f}deg Kp={kp:g} Kd={kd:g}",
            lambda t: f_mit_mode(mid, p0m if t < pre else tgtm, 0, kp, kd, 0,
                                 self.model),
            pre + sec, tick_hz=5)
        if rows is None:
            return
        after = [r for r in rows if r[0] >= pre]
        if not after:
            return
        d = tgt - p0
        t_start = after[0][0]
        t10 = t90 = None
        for r in after:
            f = (r[1] - p0) / d if d else 0.0
            if t10 is None and f >= 0.1:
                t10 = r[0] - t_start
            if t90 is None and f >= 0.9:
                t90 = r[0] - t_start
                break
        peak = max(after, key=lambda r: (r[1] - p0) / d if d else 0.0)
        tail = after[-max(1, len(after) // 5):]
        final = sum(r[1] for r in tail) / len(tail)
        print("  --- ステップ応答 ---")
        print(f"  {p0 * RAD2DEG:+.1f} → {tgt * RAD2DEG:+.1f} deg "
              f"（{delta_deg:+.0f} deg）")
        rise = f"{(t90 - t10) * 1000:.0f} ms" if (t10 is not None and t90 is not None) else "未到達"
        print(f"  立ち上がり(10-90%) {rise} / "
              f"行き過ぎ {(((peak[1] - p0) / d) - 1) * 100 if d else 0:+.1f} %")
        print(f"  定常偏差 {(tgt - final) * RAD2DEG:+.2f} deg / "
              f"最大電流 {max(abs(r[3]) for r in after):.3f} A")
        if csv:
            with open(csv, "w", encoding="utf-8", newline="") as f:
                f.write("t,target_deg,pos_deg,spd,cur,src\r\n")
                for t, pr, sp, cu, src, _ts in rows:
                    tg = (p0 if t < pre else tgt) * RAD2DEG
                    f.write(f"{t:.4f},{tg:.3f},{pr * RAD2DEG:.3f},"
                            f"{sp:.3f},{cu:.4f},{src}\r\n")
            print(f"  CSV: {csv}")


def main():
    con = Console()
    try:
        con.bus.open()
    except Exception as e:
        print(f"CAN接続に失敗: {e}")
        sys.exit(1)
    print(f"CANバスに接続 (ch={CHANNEL}, {BITRATE} bps, ID={con.target}, "
          f"モデル={MODEL})")
    print(f"MIT送信ID = (モード{MODE_ID} << 8) | {con.target} = "
          f"0x{(MODE_ID << 8) | con.target:X} の拡張フレーム")
    sync_ranges_into_cubemars()
    r = MIT_RANGES[MODEL]
    print(f"レンジ({MODEL}): 位置±{r['P'][1]} rad / 速度±{r['V'][1]} rad/s / "
          f"トルク±{r['T'][1]} N*m / Kp 0〜{r['KP'][1]} / Kd 0〜{r['KD'][1]}")
    print(f"MIT応答の解釈: {'ON' if con.bus.mit_rx else 'OFF'} / "
          f"自動中断: 観測{MAX_AGE}秒途切れ・電流{CUR_ABORT}A超")
    if SEND_LOCKED:
        print(f"★ 送信ロック中: {LOCK_NOTE}")
        print("  preview でバイト列を確認できます。解除は arm yes。")
    time.sleep(0.5)
    con.show()
    try:
        con.loop()
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        print("零トルクを送って切断します...")
        try:
            for _ in range(3):
                con.bus.send(f_zero(con.target))
                time.sleep(0.01)
        except Exception:
            pass
        con.bus.close(stop_motors=False)
        print("切断しました")


if __name__ == "__main__":
    main()

"""
cubemars.py -- CubeMars AK シリーズ CAN 制御ライブラリ

このファイルは「部品」だけを置く。対話UIは motor_console.py 側にある。
将来 motor_control/ に置いて、学習方策を動かす制御ループから import する想定。

含むもの:
  ・サーボモード 全制御モード(0〜6)のフレーム生成  … 物理単位で渡せる
  ・MITモードのフレーム生成                        … float→uint 線形マッピング付き
  ・フィードバックのデコード                        … 実測で係数確定済み
  ・MotorBus                                        … 受信スレッド + 状態保持 + 安全停止

【確定事項（実測）】
  ・フィードバック CAN ID = (0x29 << 8) | motor_id、拡張29bit、DLC=8
  ・pos = int16/10 [出力軸deg] / spd = int16*10 [ERPM] / cur = int16/100 [A]
  ・ERPM = 出力軸[deg/s] * (減速比 * 極対数 / 6)  … AK10-9/AK80-9 とも 9*21/6 = 31.5
【未確定（要実測）】
  ・MITモードの CAN ID 方式（MIT_SCHEMES / set_mit_scheme / probe_mit を参照）
  ・MIT の各レンジ（P/V/T）はモデル依存。下の MODELS は資料値であって実測値ではない
"""

import struct
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum

import can


# ============================================================
# モデル諸元
#   gear/pole_pairs は実測で裏取り済み（v 5000 の追従から 31.5 が出た）
#   p_lim/v_lim/t_lim/kp_max/kd_max は資料値。MIT を使う前に実測で確認すること。
# ============================================================
MODELS = {
    "AK10-9": dict(gear=9, pole_pairs=21,
                   p_lim=12.5, v_lim=50.0, t_lim=65.0, kp_max=500.0, kd_max=5.0),
    "AK80-9": dict(gear=9, pole_pairs=21,
                   p_lim=12.5, v_lim=50.0, t_lim=18.0, kp_max=500.0, kd_max=5.0),
}
DEFAULT_MODEL = "AK10-9"


def erpm_per_deg_s(model=DEFAULT_MODEL):
    """出力軸 1 deg/s が何 ERPM か。AK10-9/AK80-9 とも 31.5"""
    m = MODELS[model]
    return m["gear"] * m["pole_pairs"] / 6.0


def erpm_to_deg_s(erpm, model=DEFAULT_MODEL):
    return erpm / erpm_per_deg_s(model)


def deg_s_to_erpm(deg_s, model=DEFAULT_MODEL):
    return deg_s * erpm_per_deg_s(model)


# ============================================================
# サーボモード
# ============================================================
class ServoMode(IntEnum):
    DUTY          = 0   # データ int32 = duty * 100000
    CURRENT       = 1   # データ int32 = A * 1000
    CURRENT_BRAKE = 2   # データ int32 = A * 1000
    VELOCITY      = 3   # データ int32 = ERPM
    POSITION      = 4   # データ int32 = deg * 10000
    SET_ORIGIN    = 5   # データ 1byte = 0:一時 / 1:恒久 / 2:初期化
    POS_SPD       = 6   # データ int32 pos + int16 spd/10 + int16 accel/10


@dataclass(frozen=True)
class Frame:
    """送信する1フレーム。python-can の Message に落とす前の素の形"""
    arbitration_id: int
    data: bytes
    is_extended_id: bool = True

    def to_message(self):
        return can.Message(arbitration_id=self.arbitration_id,
                           data=self.data,
                           is_extended_id=self.is_extended_id)

    def __str__(self):
        return (f"ID=0x{self.arbitration_id:X} "
                f"DATA={' '.join(f'{b:02X}' for b in self.data)}")


def _servo(motor_id, mode, data):
    return Frame((int(mode) << 8) | (motor_id & 0xFF), bytes(data), True)


def f_duty(motor_id, duty):
    """デューティー比 (-1.0 〜 1.0)"""
    return _servo(motor_id, ServoMode.DUTY, struct.pack('>i', int(duty * 100000)))


def f_current(motor_id, amp):
    """iq 電流 [A]。位置/速度ループを通らない純トルク指令"""
    return _servo(motor_id, ServoMode.CURRENT, struct.pack('>i', int(amp * 1000)))


def f_current_brake(motor_id, amp):
    """ブレーキ電流 [A]"""
    return _servo(motor_id, ServoMode.CURRENT_BRAKE, struct.pack('>i', int(amp * 1000)))


def f_velocity(motor_id, erpm):
    """速度 [ERPM]。出力軸 deg/s から作るなら deg_s_to_erpm() を通す"""
    return _servo(motor_id, ServoMode.VELOCITY, struct.pack('>i', int(erpm)))


def f_position(motor_id, deg):
    """絶対位置 [出力軸 deg]"""
    return _servo(motor_id, ServoMode.POSITION, struct.pack('>i', int(deg * 10000)))


def f_set_origin(motor_id, kind=0):
    """今いる位置を原点に定義し直す。軸は動かない。0:一時 1:恒久 2:初期化"""
    return _servo(motor_id, ServoMode.SET_ORIGIN, bytes([kind & 0xFF]))


def f_pos_spd(motor_id, deg, erpm, accel_erpm_s2):
    """位置-速度ループ。最高速度と加速度を指定して位置へ動かす（位置モードより安全）"""
    data = struct.pack('>ihh',
                       int(deg * 10000),
                       int(erpm) // 10,
                       int(accel_erpm_s2) // 10)
    return _servo(motor_id, ServoMode.POS_SPD, data)


# ============================================================
# MIT モード
#   ⚠ 未検証。CAN ID 方式が資料によって割れている:
#     (a) 標準11bit、ID = motor_id                （MIT/TMotor 系の実装）
#     (b) 拡張29bit、ID = 0x800 + motor_id 相当   （V3 の報告例 0x801+）
#   set_mit_scheme() で実行時に切り替えられる（ファイル編集・再起動は不要）。
#   MotorBus.probe_mit() で3方式を総当たりして反応を比べられる。
# ============================================================
MIT_SCHEMES = {
    #  名前   : (IDベース, 拡張29bitか, 説明)
    "std":   (0x000, False, "標準11bit, ID = モーターID"),
    "ext":   (0x800, True,  "拡張29bit, ID = 0x800 + モーターID ⚠バスが落ちる"),
    "extid": (0x000, True,  "拡張29bit, ID = モーターID"),
}

# ⚠ ext は 2026-09-07 の実測で「送信するとCANバスが落ちる」と判明した。
#   送信エコーが12件（他方式は1件）＝ ACK が返らず再送を繰り返している。
#   その結果 error-passive/bus-off に落ち、送受信とも止まる。
#   「サーボの定期フィードバックが 50→0 になる」のはMITに切り替わったのではなく
#   こちらのコントローラが死んでいるだけ。既定の総当たりからは外す。
PROBE_SCHEMES_DEFAULT = ["std", "extid"]
MIT_SCHEME = "std"           # 現在の方式。未確定なので既定は従来どおり std


def set_mit_scheme(name):
    """MIT の CAN ID 方式を切り替える。再起動不要。"""
    global MIT_SCHEME
    if name not in MIT_SCHEMES:
        raise ValueError("未知の方式 %s。選べるのは %s" % (name, list(MIT_SCHEMES)))
    MIT_SCHEME = name
    return mit_scheme_desc(name)


def mit_scheme_desc(name=None):
    name = name or MIT_SCHEME
    base, ext, desc = MIT_SCHEMES[name]
    return "%s（%s）" % (desc, "拡張29bit" if ext else "標準11bit")


def mit_arbitration_id(motor_id, name=None):
    """その方式で実際に送信される CAN ID"""
    base, ext, _ = MIT_SCHEMES[name or MIT_SCHEME]
    return base + (motor_id & 0xFF)

MIT_ENABLE   = bytes([0xFF] * 7 + [0xFC])
MIT_DISABLE  = bytes([0xFF] * 7 + [0xFD])
MIT_SET_ZERO = bytes([0xFF] * 7 + [0xFE])


def float_to_uint(x, x_min, x_max, bits):
    """物理量を指定ビット幅の符号なし整数へ線形マッピング"""
    span = x_max - x_min
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(v, x_min, x_max, bits):
    span = x_max - x_min
    return v * span / ((1 << bits) - 1) + x_min


def _mit_frame(motor_id, data, scheme=None):
    base, ext, _ = MIT_SCHEMES[scheme or MIT_SCHEME]
    return Frame(base + (motor_id & 0xFF), data, ext)


def f_mit_enable(motor_id, scheme=None):
    return _mit_frame(motor_id, MIT_ENABLE, scheme)


def f_mit_disable(motor_id, scheme=None):
    return _mit_frame(motor_id, MIT_DISABLE, scheme)


def f_mit_set_zero(motor_id, scheme=None):
    return _mit_frame(motor_id, MIT_SET_ZERO, scheme)


def f_mit(motor_id, pos_rad, vel_rad_s, kp, kd, torque_nm,
          model=DEFAULT_MODEL, scheme=None):
    """
    MIT 制御フレーム。目標位置・速度・Kp・Kd・トルク前置を1フレームに詰める。
    ビット配置（MIT Cheetah 由来の標準並び）:
      pos 16bit / vel 12bit / kp 12bit / kd 12bit / torque 12bit = 64bit
    Isaac Lab の actuators.stiffness/damping が kp/kd に直接対応する。
    """
    m = MODELS[model]
    p = float_to_uint(pos_rad,   -m["p_lim"], m["p_lim"], 16)
    v = float_to_uint(vel_rad_s, -m["v_lim"], m["v_lim"], 12)
    kp_i = float_to_uint(kp, 0.0, m["kp_max"], 12)
    kd_i = float_to_uint(kd, 0.0, m["kd_max"], 12)
    t = float_to_uint(torque_nm, -m["t_lim"], m["t_lim"], 12)

    data = bytes([
        (p >> 8) & 0xFF,
        p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0x0F) << 4) | ((kp_i >> 8) & 0x0F),
        kp_i & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i & 0x0F) << 4) | ((t >> 8) & 0x0F),
        t & 0xFF,
    ])
    return _mit_frame(motor_id, data, scheme)


# ============================================================
# フィードバックのデコード
# ============================================================
STATUS_PACKET = 0x29        # 実測: フィードバック ID の上位8bit


def parse_status(msg):
    """
    サーボモードの定期フィードバック。係数はすべて実測で確定済み。
    戻り値 dict / 対象外フレームなら None
    """
    if msg.data is None or len(msg.data) < 8:
        return None
    if (msg.arbitration_id >> 8) != STATUS_PACKET:
        return None
    return {
        "id":   msg.arbitration_id & 0xFF,
        "pos":  struct.unpack('>h', msg.data[0:2])[0] / 10.0,     # 出力軸 deg
        "spd":  struct.unpack('>h', msg.data[2:4])[0] * 10.0,     # ERPM
        "cur":  struct.unpack('>h', msg.data[4:6])[0] / 100.0,    # A
        "temp": msg.data[6],                                      # ℃
        "err":  msg.data[7],
        "src":  "servo",
    }


def parse_mit_reply(msg, model=DEFAULT_MODEL):
    """
    MIT モードの応答（未検証）。data[0] にモーターID、以降 pos16/vel12/cur12。
    MIT を実際に有効化したら生ログと突き合わせて確認すること。
    """
    if msg.data is None or len(msg.data) < 6:
        return None
    m = MODELS[model]
    d = msg.data
    p_int = (d[1] << 8) | d[2]
    v_int = (d[3] << 4) | (d[4] >> 4)
    i_int = ((d[4] & 0x0F) << 8) | d[5]
    return {
        "id":   d[0],
        "pos":  uint_to_float(p_int, -m["p_lim"], m["p_lim"], 16),   # rad
        "spd":  uint_to_float(v_int, -m["v_lim"], m["v_lim"], 12),   # rad/s
        "cur":  uint_to_float(i_int, -m["t_lim"], m["t_lim"], 12),
        "temp": d[6] if len(d) > 6 else None,
        "err":  d[7] if len(d) > 7 else None,
        "src":  "mit",
    }


# ============================================================
# モーター1個の状態
# ============================================================
@dataclass
class MotorState:
    id: int
    pos: float = 0.0
    spd: float = 0.0
    cur: float = 0.0
    temp: int = 0
    err: int = 0
    t: float = 0.0
    src: str = ""

    def age(self):
        return time.time() - self.t if self.t else float("inf")

    def alive(self, max_age=0.3):
        return self.age() < max_age


# ============================================================
# バス管理
# ============================================================
class MotorBus:
    """
    CANバス + 受信スレッド + 全モーターの最新状態。
    受信スレッドは print しない（対話プロンプトを邪魔しないため）。
    生ログが見たいときは raw_log=True にするか on_raw コールバックを渡す。
    """

    def __init__(self, channel=1, bitrate=1_000_000, interface='gs_usb',
                 model=DEFAULT_MODEL, raw_log=False):
        self.channel = channel
        self.bitrate = bitrate
        self.interface = interface
        self.model = model
        self.raw_log = raw_log
        self.on_raw = None

        self.bus = None
        self._rx_thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._states = {}
        self._seen_ids = set()
        self.tx_count = 0
        self.rx_count = 0
        self.mit_rx = False        # True で未知フレームを MIT 応答として解釈する
        self.echo_window = 0.08    # 送信からこの秒数以内の同一フレームはエコーとみなす
        self.echo_count = 0        # 除外したエコーの数
        self.show_echo = False     # True なら raw ログにエコーも [TX-echo] として出す
        self._tx_recent = []       # [(時刻, ID, データ)] 自分が送ったフレーム
        self.mit_enabled = set()   # MIT enable を送った相手（停止時に disable する）
        self.mit_reply_ids = set() # MIT応答が実際に届いた CAN ID（A10の実測用）
        self.rx_error = None       # 受信スレッドが止まった理由。None なら正常
        self._cap = None           # capture() 実行中だけ dict になる

    # ---- 接続 ----
    def open(self):
        self.bus = can.Bus(interface=self.interface,
                           channel=self.channel,
                           bitrate=self.bitrate)
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        return self

    def close(self, stop_motors=True):
        """必ず停止指令を出してから閉じる"""
        if stop_motors:
            try:
                self.stop_all()
            except Exception:
                pass
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        time.sleep(0.1)
        if self.bus:
            self.bus.shutdown()

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ---- 受信 ----
    def _is_echo(self, msg):
        """
        自分が送ったフレームが受信側に返ってきたものかを判定する。

        ⚠ これが要る理由（2026-09-07 実測）:
        gs_usb アダプタは送信フレームを受信ストリームにも流す。これを
        モーターからの応答と取り違えると「新しい CAN ID が現れた＝MITが
        有効になった」という誤判定になる。実際に probe_mit が
        `0x082B DATA=FF..FC`（自分が送った enable そのもの）を新規IDとして
        報告し、ext 方式が本命だと誤って結論づけた。
        """
        if getattr(msg, "is_rx", True) is False:
            return True
        now = time.time()
        data = bytes(msg.data)
        with self._lock:
            self._tx_recent = [e for e in self._tx_recent
                               if now - e[0] < self.echo_window]
            for _t, arb, d in self._tx_recent:
                if arb == msg.arbitration_id and d == data:
                    return True
        return False

    def _looks_like_mit(self, msg):
        """
        MIT応答らしいフレームだけを parse_mit_reply に通す。

        ⚠ これが要る理由: parse_mit_reply は「8バイト未満でなければ何でも」
        解釈してしまう。フィルタが無いと、バス上の無関係なフレームが
        data[0] を勝手にモーターIDとして状態表に書き込む。エラーは出ない。
        10モーターになったら幽霊IDが増えるだけで気づけない。
        """
        d = msg.data
        if d is None or len(d) < 6:
            return False
        if (msg.arbitration_id >> 8) == STATUS_PACKET:
            return False          # サーボ定期フィードバックの短いやつ
        mid = d[0]
        if self.mit_enabled:
            return mid in self.mit_enabled
        return 1 <= mid <= 127

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                if self._is_echo(msg):
                    self.echo_count += 1
                    if self.show_echo and (self.raw_log or self.on_raw):
                        line = (f"[TX-echo] ID=0x{msg.arbitration_id:08X} "
                                f"DLC={msg.dlc} DATA={msg.data.hex(' ')}")
                        if self.on_raw:
                            self.on_raw(line, msg)
                        elif self.raw_log:
                            print(line)
                    continue

                self.rx_count += 1
                if self._cap is not None:
                    with self._lock:
                        if self._cap is not None:
                            e = self._cap.setdefault(
                                msg.arbitration_id,
                                {"n": 0, "ext": msg.is_extended_id,
                                 "dlc": msg.dlc, "last": b""})
                            e["n"] += 1
                            e["last"] = bytes(msg.data)
                if self.raw_log or self.on_raw:
                    line = (f"[RX] ID=0x{msg.arbitration_id:08X} DLC={msg.dlc} "
                            f"DATA={msg.data.hex(' ')}")
                    if self.on_raw:
                        self.on_raw(line, msg)
                    elif self.raw_log:
                        print(line)

                st = parse_status(msg)
                if st is None and self.mit_rx and self._looks_like_mit(msg):
                    st = parse_mit_reply(msg, self.model)
                    if st is not None:
                        self.mit_reply_ids.add(msg.arbitration_id)
                if st is None:
                    continue
                with self._lock:
                    self._seen_ids.add(st["id"])
                    s = self._states.setdefault(st["id"], MotorState(id=st["id"]))
                    s.pos, s.spd, s.cur = st["pos"], st["spd"], st["cur"]
                    s.temp, s.err, s.src = st["temp"], st["err"], st["src"]
                    s.t = time.time()
            except can.CanError as e:
                self.rx_error = "CanError: %s" % e
                break
            except Exception as e:
                # ここで黙って break すると「応答は来ているのに状態が更新
                # されない」という原因不明の症状になる。理由を残す。
                self.rx_error = "%s: %s" % (type(e).__name__, e)
                break

    # ---- 状態 ----
    def state(self, motor_id):
        with self._lock:
            s = self._states.get(motor_id)
            if s is None:
                return None
            return MotorState(**vars(s))

    def seen_ids(self):
        with self._lock:
            return sorted(self._seen_ids)

    def scan(self, seconds=2.0):
        """定期フィードバックを聞いて、バス上にいるモーターIDを集める"""
        with self._lock:
            self._seen_ids.clear()
        t0 = time.time()
        while time.time() - t0 < seconds:
            time.sleep(0.05)
        return self.seen_ids()

    # ---- 送信 ----
    def send(self, frame, quiet=True):
        try:
            self.bus.send(frame.to_message())
            self.tx_count += 1
            with self._lock:
                self._tx_recent.append(
                    (time.time(), frame.arbitration_id, bytes(frame.data)))
                if len(self._tx_recent) > 200:
                    del self._tx_recent[:100]
            if not quiet:
                print(f"  送信 {frame}")
            return True
        except can.CanError as e:
            print(f"  送信エラー: {e}")
            return False

    def stop_all(self, motor_ids=None):
        """
        停止。既知の全モーターへ速度0を送り、MIT を有効にした相手には
        零トルクの MIT フレーム + disable も送る。
        （サーボ用の速度0フレームは MIT モードのモーターには効かないため）

        ⚠ これはソフトウェア上の停止であって非常停止ではない。
           本当の非常停止は電源を切ること。
        """
        ids = motor_ids if motor_ids is not None else (self.seen_ids() or [])
        if motor_ids is None:
            mit_ids = sorted(self.mit_enabled)
        else:
            mit_ids = sorted(self.mit_enabled & set(motor_ids))
        for _ in range(3):
            for mid in ids:
                self.send(f_velocity(mid, 0))
            for mid in mit_ids:
                self.send(f_mit(mid, 0.0, 0.0, 0.0, 0.0, 0.0, self.model))
            time.sleep(0.01)
        for mid in mit_ids:
            self.mit_disable(mid)
        return ids

    # ---- 生フレームの集計（プロトコル調査用） ----
    def capture(self, seconds=1.0):
        """
        指定秒だけ受信フレームを CAN ID ごとに集計する。
        戻り値: {arbitration_id: {"n": 回数, "ext": 拡張IDか, "dlc": 長さ, "last": 最後のデータ}}
        """
        with self._lock:
            self._cap = {}
        time.sleep(seconds)
        with self._lock:
            cap = self._cap or {}
            self._cap = None
        return cap

    def feedback_hz(self, motor_id=None, seconds=2.0):
        """
        定期フィードバックのレート [Hz] を実測する（チェックリスト A1）。
        戻り値: [(arbitration_id, motor_id, 回数, Hz, dlc, ext, last)]
        """
        cap = self.capture(seconds)
        rows = []
        for arb, e in sorted(cap.items()):
            mid = arb & 0xFF
            if motor_id is not None and mid != motor_id:
                continue
            rows.append((arb, mid, e["n"], e["n"] / seconds,
                         e["dlc"], e["ext"], e["last"]))
        return rows

    # ---- MIT ----
    def mit_enable(self, motor_id, scheme=None, quiet=True):
        """MIT enable。停止時に disable できるよう送った相手を覚えておく。"""
        ok = self.send(f_mit_enable(motor_id, scheme), quiet=quiet)
        if ok:
            self.mit_enabled.add(motor_id)
        return ok

    def mit_disable(self, motor_id, scheme=None, quiet=True):
        ok = self.send(f_mit_disable(motor_id, scheme), quiet=quiet)
        self.mit_enabled.discard(motor_id)
        return ok

    def probe_mit(self, motor_id, schemes=None, settle=1.0):
        """
        MIT の CAN ID 方式を総当たりで試す（チェックリスト A8）。

        方式ごとに
          ① enable 前の受信フレームを settle 秒集計（ベースライン）
          ② enable を送る
          ③ もう一度 settle 秒集計
        して差を見る。MIT に切り替わったなら、サーボ形式（0x29xx）の定期
        フィードバックが止まるか、別の CAN ID のフレームが現れるはず。

        判定材料を返すだけで結論は出さない（推測で確定しないため）。
        戻り値: [{"scheme","frame","before","after","new_ids",
                  "servo_before","servo_after"}]
        """
        def servo_count(cap):
            return sum(e["n"] for a, e in cap.items()
                       if (a >> 8) == STATUS_PACKET)

        names = list(schemes) if schemes else list(PROBE_SCHEMES_DEFAULT)
        original = MIT_SCHEME
        results = []
        try:
            for name in names:
                set_mit_scheme(name)
                frame = f_mit_enable(motor_id)
                before = self.capture(settle)
                e0 = self.echo_count
                self.send(frame)
                time.sleep(0.2)
                after = self.capture(settle)
                echoes = self.echo_count - e0
                self.send(f_mit_disable(motor_id))
                time.sleep(0.2)
                results.append({
                    "scheme": name,
                    "frame": str(frame),
                    "before": before,
                    "after": after,
                    "new_ids": sorted(set(after) - set(before)),
                    "servo_before": servo_count(before),
                    "servo_after": servo_count(after),
                    "echoes": echoes,
                })
        finally:
            set_mit_scheme(original)
            self.mit_enabled.discard(motor_id)
        return results

    # ---- 定周期送信（制御ループの原型） ----
    def hold(self, make_frames, seconds, hz=50, on_sample=None, sample_hz=5,
             stop_after=True, watchdog_ids=None, watchdog_age=0.5):
        """
        make_frames(t) -> Frame または Frame のリスト を hz で送り続ける。
        watchdog_ids に指定したモーターのフィードバックが watchdog_age 秒
        途切れたら即座に停止する（フェイルセーフ）。

        戻り値: 送信回数
        """
        t0 = time.time()
        next_send = t0
        next_sample = t0
        n = 0
        period = 1.0 / hz
        sample_period = 1.0 / sample_hz if sample_hz else None

        try:
            while True:
                now = time.time()
                elapsed = now - t0
                if elapsed >= seconds:
                    break

                if now >= next_send:
                    frames = make_frames(elapsed)
                    if isinstance(frames, Frame):
                        frames = [frames]
                    for fr in frames:
                        self.send(fr)
                    n += 1
                    next_send += period

                if watchdog_ids:
                    for mid in watchdog_ids:
                        s = self.state(mid)
                        if s is None or not s.alive(watchdog_age):
                            print(f"  ⚠ フェイルセーフ: ID={mid} のフィードバックが"
                                  f"{watchdog_age}秒途切れました。停止します。")
                            self.stop_all(watchdog_ids)
                            return n

                if sample_period and on_sample and now >= next_sample:
                    on_sample(elapsed)
                    next_sample += sample_period

                time.sleep(0.002)
        except KeyboardInterrupt:
            print("  中断")
        finally:
            if stop_after:
                self.stop_all(watchdog_ids)
        return n
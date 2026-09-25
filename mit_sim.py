"""
mit_sim.py -- motor_console_ver7/ver8 の練習・検算用の「にせモーター」

実機の代わりに `python motor_console_ver7/ver8.py --sim` で使う。CAN にもモーターにも触らない。
中身は AK80-9 を1個だけ模した簡易モデル（無負荷の出力軸）。

  ・送られた MIT フレーム（モード8）を「ファーム側のレンジ」で解読して、
      tau = c_p*Kp*(P - θ_mit) + c_d*Kd*(V - ω) + T
    を出す。c_p / c_d / ファームの速度レンジは SIM で好きに変えられる
    （＝ver7 の実験が、隠した値を当てられるかの検算に使う）。
  ・MIT の位置原点はサーボ原点と別（mit_zero と servo_origin が別の変数）。
  ・静止摩擦 / 動摩擦 / 粘性、50Hz の定期フィードバック（サーボ形式・deg/ERPM/A）、
    指令が途切れたら wdog 秒で零トルク、指令の反映遅れ latency。

⚠ ここでの数字は「実験の手順とコードを検算する」ための仮置き。実機の値ではない。
"""

import math
import random
import threading
import time

RAD2DEG = 57.29577951308232

SIM = dict(
    off_true=2.30,        # [rad] MIT位置 − サーボ角（隠し値）
    c_p=0.80,             # 実効Kp / 指令Kp（隠し値）
    c_d=1.00,             # 実効Kd / 指令Kd（隠し値）
    fw_v=30.0,            # ファームが速度欄を読むレンジ ±[rad/s]（隠し値。ver7 は既定 65 で詰める）
    fw_p=12.56,           # ファームが位置欄を読むレンジ
    fw_t=18.0,
    kt_cmd=0.59,          # 指令トルク → 電流 の内部定数
    fs=0.35, fd=0.22, b=0.013,
    J=9.77e-3,
    wdog=0.50,            # [s] 指令が途切れてから零トルクになるまで。None なら保持し続ける
    latency=0.006,        # [s]
    noise=0.02,           # [A]
    o_resets_mit=False,   # モード5（原点設定）で MIT の原点も動くか
    sign_bad=True,        # 負の電流の符号をランダムにする（実機の再現）
    motor_id=34,
)


def _u2f(v, lo, hi, bits):
    return v * (hi - lo) / ((1 << bits) - 1) + lo


class _State:
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def age(self):
        return time.time() - self.t

    def alive(self, max_age=0.3):
        return self.t > 0 and self.age() <= max_age


class SimBus:
    def __init__(self, **params):
        self.p = dict(SIM)
        self.p.update(params)
        self.model = "AK80-9"
        self.raw_log = False
        self.mit_rx = False
        self.tx_count = 0
        self.echo_count = 0
        self.rx_count = 0
        self.rx_error = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.theta = 1.0          # 真の出力軸角
        self.omega = 0.0
        self.servo_origin = 0.0
        self.mit_zero = self.servo_origin - self.p["off_true"]
        self.cmd = None           # (kp, kd, P, V, T, 受信時刻)
        self.pending = []
        self.servo_vel_hold = False
        self._st = None
        self.last_tau = 0.0

    # ---- 接続 ----
    def open(self):
        threading.Thread(target=self._physics, daemon=True).start()
        threading.Thread(target=self._feedback, daemon=True).start()
        return self

    def close(self, stop_motors=False):
        self._stop.set()

    # ---- 送信 ----
    def send(self, frame, quiet=True):
        now = time.time()
        with self._lock:
            self.tx_count += 1
            self.echo_count += 1
            self.pending.append((now + self.p["latency"], frame.arbitration_id, bytes(frame.data)))
        if not quiet:
            print(f"  送信 {frame}")
        return True

    def _apply(self, arb, d):
        mode, mid = arb >> 8, arb & 0xFF
        if mid != self.p["motor_id"]:
            return
        if mode == 8 and len(d) == 8:
            kp_i = (d[0] << 4) | (d[1] >> 4)
            kd_i = ((d[1] & 0x0F) << 8) | d[2]
            p_i = (d[3] << 8) | d[4]
            v_i = (d[5] << 4) | (d[6] >> 4)
            t_i = ((d[6] & 0x0F) << 8) | d[7]
            P = self.p
            self.cmd = (_u2f(kp_i, 0, 500, 12), _u2f(kd_i, 0, 5, 12),
                        _u2f(p_i, -P["fw_p"], P["fw_p"], 16),
                        _u2f(v_i, -P["fw_v"], P["fw_v"], 12),
                        _u2f(t_i, -P["fw_t"], P["fw_t"], 12), time.time())
            self.servo_vel_hold = False
        elif mode == 5:
            self.servo_origin = self.theta
            if self.p["o_resets_mit"]:
                self.mit_zero = self.theta
        elif mode == 3:
            self.cmd = None
            self.servo_vel_hold = True

    # ---- 物理 ----
    def _physics(self):
        P = self.p
        dt = 0.0005
        last = time.time()
        while not self._stop.is_set():
            now = time.time()
            steps = int((now - last) / dt)
            if steps <= 0:
                time.sleep(dt)
                continue
            steps = min(steps, 200)
            last += steps * dt
            with self._lock:
                due = [e for e in self.pending if e[0] <= now]
                self.pending = [e for e in self.pending if e[0] > now]
            for _t, arb, d in due:
                self._apply(arb, d)
            for _ in range(steps):
                tau = 0.0
                c = self.cmd
                if c is not None and (P["wdog"] is None or now - c[5] < P["wdog"]):
                    kp, kd, Pp, V, T, _ = c
                    th_mit = self.theta - self.mit_zero
                    tau = P["c_p"] * kp * (Pp - th_mit) + P["c_d"] * kd * (V - self.omega) + T
                    tau = max(-P["fw_t"], min(P["fw_t"], tau))
                elif self.servo_vel_hold:
                    tau = -2.0 * self.omega
                self.last_tau = tau
                w = self.omega
                if w == 0.0:
                    if abs(tau) <= P["fs"]:
                        continue
                    net = tau - math.copysign(P["fd"], tau)
                else:
                    net = tau - math.copysign(P["fd"], w) - P["b"] * w
                w2 = w + net / P["J"] * dt
                if w != 0.0 and (w2 * w < 0) and abs(tau) <= P["fs"]:
                    w2 = 0.0
                self.omega = w2
                self.theta += w2 * dt

    def _feedback(self):
        P = self.p
        period = 0.02
        nxt = time.time()
        while not self._stop.is_set():
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))
            pos = (self.theta - self.servo_origin) * RAD2DEG
            pos = max(-3276.7, min(3276.7, round(pos, 1)))
            spd = round(self.omega * RAD2DEG * 31.5 / 10.0) * 10.0
            i_true = self.last_tau / P["kt_cmd"] + random.gauss(0, P["noise"])
            if P.get("sign_bad", True) and i_true < 0:
                i_true = abs(i_true) * random.choice((-1, 1))   # 実機: 負の電流は符号が当てにならない（2026-09-16）
            cur = round(i_true, 2)
            with self._lock:
                self.rx_count += 1
                self._st = _State(id=P["motor_id"], pos=pos, spd=spd, cur=cur,
                                  temp=35, err=0, src="servo", t=time.time())

    # ---- 状態 ----
    def state(self, motor_id):
        with self._lock:
            s = self._st
            if s is None or motor_id != self.p["motor_id"]:
                return None
            return _State(**vars(s))

    def seen_ids(self):
        return [self.p["motor_id"]] if self._st else []

    def feedback_hz(self, seconds=2.0):
        n0 = self.rx_count
        time.sleep(seconds)
        n = self.rx_count - n0
        mid = self.p["motor_id"]
        return [(0x2900 | mid, mid, n, n / seconds, 8, True, b"")]

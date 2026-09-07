# -*- coding: utf-8 -*-
"""
bench_inertia_friction.py  ---- モーター1個の物理パラメータをベンチで実測する

ここで出る3つの値は、そのまま Isaac Lab の機体定義に入る:

    Kt (トルク定数)  ->  電流とトルクの換算基準。effort_limit の裏どりにも使う
    J  (出力軸慣性)  ->  armature にそのまま入れる値（出力軸で測るので x81 は不要）
    摩擦 (クーロン+粘性) -> friction に入れる値（Isaac Sim 5.x では N.m）

■ 安全（毎回守る）
    - 脚やアームを付けない（kt モードだけは校正用アームを付ける）
    - モーターを台にしっかり固定する
    - 電源をすぐ切れる状態にしておく
    - Ctrl+C でいつでも止まる（finally で全モーター停止指令を出す）

■ 使い方（Windows のコマンドプロンプトで、このファイルのある場所で実行）
    python bench_inertia_friction.py fric  --id 43 --model AK80-9
    python bench_inertia_friction.py coast --id 43 --model AK80-9
    python bench_inertia_friction.py kt    --id 43 --model AK80-9 --mass 0.5 --arm 0.15
    python bench_inertia_friction.py analyze --id 43 --model AK80-9

    まず fric（無負荷で回すだけ）、次に coast（空転させるだけ）。
    kt は校正用アームと分銅が用意できたときだけ。
"""

import argparse
import csv
import math
import os
import sys
import time

import cubemars as cm

G = 9.80665

# データシート値（2026-09-03 CubeMars 公式製品ページ）
DATASHEET = {
    "AK10-9": dict(kt_motor=0.16,  gear=9, rated_nm=18.0, peak_nm=53.0,
                   rotor_inertia=1.002e-4,  no_load_rpm=320),
    "AK80-9": dict(kt_motor=0.095, gear=9, rated_nm=9.0,  peak_nm=22.0,
                   rotor_inertia=1.1183e-4, no_load_rpm=570),
}


# ---------------------------------------------------------------- 小道具
def kt_out_default(model):
    d = DATASHEET[model]
    return d["kt_motor"] * d["gear"]


def deg_s_to_rad_s(x):
    return x * math.pi / 180.0


def linfit(xs, ys):
    """最小二乗で y = a + b*x を解く。numpy を使わない"""
    n = len(xs)
    if n < 2:
        return None, None
    sx = sum(xs); sy = sum(ys)
    sxx = sum(x * x for x in xs); sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if abs(den) < 1e-15:
        return None, None
    b = (n * sxy - sx * sy) / den
    a = (sy - b * sx) / n
    return a, b


def median(v):
    if not v:
        return float("nan")
    s = sorted(v)
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


def confirm(msg):
    print("\n" + "=" * 62)
    print(msg)
    print("=" * 62)
    ans = input("この内容で実行してよければ y を入力して Enter: ").strip().lower()
    if ans != "y":
        print("中止しました。")
        sys.exit(0)


class Sampler:
    """hold() の on_sample から呼ばれ、フィードバックが更新された分だけ記録する"""

    def __init__(self, bus, mid):
        self.bus = bus
        self.mid = mid
        self.rows = []
        self._last_t = None
        self.t0 = time.time()

    def __call__(self, elapsed):
        s = self.bus.state(self.mid)
        if s is None or s.t == self._last_t:
            return
        self._last_t = s.t
        self.rows.append(dict(t=s.t - self.t0, pos_deg=s.pos, spd_erpm=s.spd,
                              cur_a=s.cur, temp_c=s.temp, err=s.err))


def save_csv(path, rows, extra_cols=()):
    if not rows:
        print(f"  (記録なし: {path} は作りません)")
        return
    cols = list(rows[0].keys()) + list(extra_cols)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    print(f"  記録しました: {path}  ({len(rows)} 行)")


# ---------------------------------------------------------------- fric
def run_friction(bus, mid, model, speeds, dwell, out):
    """一定速度で回し、定常電流から摩擦トルクを測る"""
    all_rows = []
    points = []
    for v in speeds:
        erpm = cm.deg_s_to_erpm(v, model)
        print(f"  {v:6.1f} deg/s ({erpm:7.0f} ERPM) で {dwell:.1f} 秒...")
        smp = Sampler(bus, mid)
        bus.hold(lambda t, e=erpm: cm.f_velocity(mid, e),
                 seconds=dwell, hz=50, on_sample=smp, sample_hz=200,
                 stop_after=False, watchdog_ids=[mid], watchdog_age=0.5)
        # 後半だけを定常とみなす
        tail = [r for r in smp.rows if r["t"] > dwell * 0.5]
        if not tail:
            print("    フィードバックが取れませんでした。スキップします。")
            continue
        cur = sum(r["cur_a"] for r in tail) / len(tail)
        spd = sum(cm.erpm_to_deg_s(r["spd_erpm"], model) for r in tail) / len(tail)
        points.append((deg_s_to_rad_s(spd), cur))
        for r in tail:
            r["target_deg_s"] = v
        all_rows.extend(tail)
        print(f"    実速度 {spd:7.1f} deg/s / 定常電流 {cur:6.3f} A")
    bus.stop_all([mid])
    save_csv(out, all_rows, extra_cols=())
    return points


def report_friction(points, kt_out):
    print("\n--- 摩擦の推定 ---")
    if len(points) < 2:
        print("  点が足りません。")
        return None, None
    xs = [w for w, _ in points]
    ys = [kt_out * i for _, i in points]
    tau_c, b = linfit(xs, ys)
    print(f"  クーロン摩擦 tau_c = {tau_c:.4f} N.m   （速度に依らない一定の抵抗）")
    print(f"  粘性摩擦     b     = {b:.5f} N.m/(rad/s)")
    for w, i in points:
        print(f"    omega={w:7.3f} rad/s  I={i:6.3f} A  tau={kt_out*i:6.3f} N.m"
              f"  (fit {tau_c + b*w:6.3f})")
    return tau_c, b


# ---------------------------------------------------------------- coast
def run_coast(bus, mid, model, v0, spin, coast, mode, out):
    """一定速度まで回してからトルクを切り、空転減速を記録する"""
    erpm = cm.deg_s_to_erpm(v0, model)
    print(f"  {v0:.1f} deg/s まで回します（{spin:.1f} 秒）...")
    bus.hold(lambda t: cm.f_velocity(mid, erpm), seconds=spin, hz=50,
             stop_after=False, watchdog_ids=[mid], watchdog_age=0.5)

    print(f"  トルクを切って {coast:.1f} 秒ぶん空転を記録します（mode={mode}）...")
    if mode == "duty":
        make = lambda t: cm.f_duty(mid, 0.0)
    else:
        make = lambda t: cm.f_current(mid, 0.0)
    smp = Sampler(bus, mid)
    bus.hold(make, seconds=coast, hz=50, on_sample=smp, sample_hz=200,
             stop_after=True, watchdog_ids=[mid], watchdog_age=0.5)
    save_csv(out, smp.rows)
    return smp.rows


def report_coast(rows, model, tau_c, b, kt_out):
    print("\n--- 出力軸慣性の推定 ---")
    if len(rows) < 10:
        print("  データが足りません。")
        return None
    if tau_c is None:
        print("  先に fric を実行して摩擦を求めてください（J は摩擦が分からないと出せません）。")
        return None
    ws = [(r["t"], deg_s_to_rad_s(cm.erpm_to_deg_s(r["spd_erpm"], model))) for r in rows]
    ws = [(t, w) for t, w in ws if abs(w) > 0.05]      # 停止後は使わない
    if len(ws) < 10:
        print("  減速区間が短すぎます。--v0 を上げるか --coast を延ばしてください。")
        return None
    js = []
    step = 3
    for k in range(len(ws) - step):
        t1, w1 = ws[k]
        t2, w2 = ws[k + step]
        dt = t2 - t1
        if dt <= 0:
            continue
        dwdt = (w2 - w1) / dt
        if dwdt >= -1e-3:                              # 減速していない点は捨てる
            continue
        wm = 0.5 * (w1 + w2)
        tau_f = tau_c + b * abs(wm)
        js.append(tau_f / (-dwdt))
    if not js:
        print("  減速が検出できませんでした。トルクが本当に切れているか確認してください。")
        return None
    j = median(js)
    print(f"  出力軸まわりの慣性 J = {j:.5f} kg.m^2  （{len(js)} 点の中央値）")
    d = DATASHEET[model]
    ref = d["rotor_inertia"] * d["gear"] ** 2
    print(f"  データシート由来の参考値（ロータ慣性 x 81） = {ref:.5f} kg.m^2")
    if ref > 0:
        print(f"  比 = {j / ref:.2f} 倍  （減速機ぶんがあるので 1.0〜2.0 なら妥当）")
    return j


# ---------------------------------------------------------------- kt
def run_kt(bus, mid, model, mass, arm, dwell, out):
    """既知トルクを静止保持させ、必要電流から Kt を逆算する"""
    s = bus.state(mid)
    if s is None:
        print("  フィードバックが来ていません。配線と ID を確認してください。")
        return None
    hold_deg = s.pos
    tau_ref = mass * G * arm
    print(f"  現在角 {hold_deg:.1f} deg を保持します。基準トルク = {tau_ref:.4f} N.m")
    smp = Sampler(bus, mid)
    bus.hold(lambda t: cm.f_pos_spd(mid, hold_deg, cm.deg_s_to_erpm(30.0, model), 5000),
             seconds=dwell, hz=50, on_sample=smp, sample_hz=200,
             stop_after=True, watchdog_ids=[mid], watchdog_age=0.5)
    tail = [r for r in smp.rows if r["t"] > dwell * 0.5]
    if not tail:
        print("  記録が取れませんでした。")
        return None
    cur = sum(r["cur_a"] for r in tail) / len(tail)
    save_csv(out, tail)
    if abs(cur) < 1e-3:
        print("  電流がほぼ0です。アームが水平になっているか確認してください。")
        return None
    kt_out = tau_ref / abs(cur)
    d = DATASHEET[model]
    print(f"\n--- トルク定数の推定 ---")
    print(f"  保持電流 = {cur:.3f} A")
    print(f"  出力軸換算 Kt_out = {kt_out:.4f} N.m/A")
    print(f"  モーター側 Kt     = {kt_out / d['gear']:.4f} N.m/A"
          f"  （データシート {d['kt_motor']:.3f}）")
    print(f"  定格電流での出力トルク見積り = {kt_out * d['kt_motor']:.1f} 〜 "
          f"データシート定格 {d['rated_nm']:.1f} N.m と比べること")
    return kt_out


# ---------------------------------------------------------------- まとめ
def print_paste(model, joint_hint, armature, friction):
    print("\n" + "=" * 62)
    print("Isaac Lab の my_robot_code/skyentific_poclegs.py に入れる値")
    print("=" * 62)
    if armature is not None:
        print(f"  {joint_hint}_ARMATURE = {armature:.5e}   # {model} 実測")
    if friction is not None:
        print(f"  {joint_hint}_FRICTION = {friction:.4f}      # {model} 実測（静止摩擦トルク N.m）")
    print("  ※ friction は Isaac Sim 5.0 以降ではトルク [N.m]。4.5 までの係数とは意味が違う。")


def main():
    p = argparse.ArgumentParser(description="CubeMars モーターの Kt / 慣性 / 摩擦を実測する")
    p.add_argument("mode", choices=["fric", "coast", "kt", "analyze"])
    p.add_argument("--id", type=int, required=True, help="モーターID（scan で確認した値）")
    p.add_argument("--model", default="AK80-9", choices=list(DATASHEET.keys()))
    p.add_argument("--channel", type=int, default=1)
    p.add_argument("--speeds", default="20,40,60,80,100,120",
                   help="fric で回す速度 [deg/s] のリスト")
    p.add_argument("--dwell", type=float, default=3.0, help="各速度での保持時間 [s]")
    p.add_argument("--v0", type=float, default=120.0, help="coast の初速 [deg/s]")
    p.add_argument("--spin", type=float, default=2.5, help="coast の助走時間 [s]")
    p.add_argument("--coast", type=float, default=4.0, help="coast の記録時間 [s]")
    p.add_argument("--coast-mode", default="current", choices=["current", "duty"])
    p.add_argument("--mass", type=float, default=None, help="kt: 分銅の質量 [kg]")
    p.add_argument("--arm", type=float, default=None, help="kt: 腕の長さ [m]")
    p.add_argument("--kt-out", type=float, default=None,
                   help="出力軸トルク定数 [N.m/A]。省略時はデータシート値 Kt x 9")
    a = p.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    f_fric = os.path.join(here, f"bench_fric_{a.id}.csv")
    f_coast = os.path.join(here, f"bench_coast_{a.id}.csv")
    f_kt = os.path.join(here, f"bench_kt_{a.id}.csv")

    kt_out = a.kt_out if a.kt_out else kt_out_default(a.model)
    speeds = [float(x) for x in a.speeds.split(",") if x.strip()]
    hint = "AK10" if a.model == "AK10-9" else "AK80"

    # --- 計算だけ（モーターを触らない） ---
    if a.mode == "analyze":
        pts = []
        if os.path.exists(f_fric):
            rows = list(csv.DictReader(open(f_fric, encoding="utf-8")))
            groups = {}
            for r in rows:
                groups.setdefault(r.get("target_deg_s", "0"), []).append(r)
            for _, g in sorted(groups.items(), key=lambda kv: float(kv[0])):
                w = sum(deg_s_to_rad_s(cm.erpm_to_deg_s(float(r["spd_erpm"]), a.model))
                        for r in g) / len(g)
                i = sum(float(r["cur_a"]) for r in g) / len(g)
                pts.append((w, i))
        tau_c, b = report_friction(pts, kt_out)
        j = None
        if os.path.exists(f_coast):
            rows = [dict(t=float(r["t"]), spd_erpm=float(r["spd_erpm"]))
                    for r in csv.DictReader(open(f_coast, encoding="utf-8"))]
            j = report_coast(rows, a.model, tau_c, b, kt_out)
        print_paste(a.model, hint, j, tau_c)
        return

    # --- 実機を動かす ---
    if a.mode == "kt" and (a.mass is None or a.arm is None):
        print("kt モードには --mass と --arm が必要です。")
        sys.exit(1)

    warn = {
        "fric": f"ID={a.id} ({a.model}) を無負荷で {speeds} deg/s まで回します。\n"
                "脚やアームを外し、モーターを台に固定してください。",
        "coast": f"ID={a.id} ({a.model}) を {a.v0} deg/s まで回してから空転させます。\n"
                 "脚やアームを外し、モーターを台に固定してください。",
        "kt": f"ID={a.id} ({a.model}) に校正アームを付けた状態で、現在角を保持します。\n"
              f"分銅 {a.mass} kg / 腕 {a.arm} m。アームが水平になっていることを確認してください。",
    }[a.mode]
    confirm(warn + "\n（Ctrl+C でいつでも止まります。終了時に停止指令を出します。）")

    bus = cm.MotorBus(channel=a.channel, model=a.model)
    tau_c = b = j = None
    try:
        with bus:
            time.sleep(0.5)
            if bus.state(a.id) is None:
                print(f"  ⚠ ID={a.id} からのフィードバックがありません。")
                print("    scan で ID を確認し、CAN チャンネルが合っているか見てください。")
                return
            if a.mode == "fric":
                pts = run_friction(bus, a.id, a.model, speeds, a.dwell, f_fric)
                tau_c, b = report_friction(pts, kt_out)
                print_paste(a.model, hint, None, tau_c)
            elif a.mode == "coast":
                rows = run_coast(bus, a.id, a.model, a.v0, a.spin, a.coast,
                                 a.coast_mode, f_coast)
                if os.path.exists(f_fric):
                    pts = []
                    src = list(csv.DictReader(open(f_fric, encoding="utf-8")))
                    groups = {}
                    for r in src:
                        groups.setdefault(r.get("target_deg_s", "0"), []).append(r)
                    for _, g in sorted(groups.items(), key=lambda kv: float(kv[0])):
                        w = sum(deg_s_to_rad_s(cm.erpm_to_deg_s(float(r["spd_erpm"]), a.model))
                                for r in g) / len(g)
                        i = sum(float(r["cur_a"]) for r in g) / len(g)
                        pts.append((w, i))
                    tau_c, b = report_friction(pts, kt_out)
                j = report_coast([dict(t=r["t"], spd_erpm=r["spd_erpm"]) for r in rows],
                                 a.model, tau_c, b, kt_out)
                print_paste(a.model, hint, j, tau_c)
            elif a.mode == "kt":
                run_kt(bus, a.id, a.model, a.mass, a.arm, a.dwell, f_kt)
    except KeyboardInterrupt:
        print("\n中断しました。")
    finally:
        try:
            bus.stop_all([a.id])
        except Exception:
            pass
        print("停止指令を送りました。")


if __name__ == "__main__":
    main()

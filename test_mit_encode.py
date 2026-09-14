"""
test_mit_encode.py -- MIT フレームの符号化・復号の自己検証（実機不要）

指示書 claude/mit_implementation_briefing.md ステップ1:
  「既知の値を詰めて、逆変換したら元に戻るか」を実機に送る前に確認する。

実行:  python test_mit_encode.py      （python-can が無くても動く）

⚠ ここで検証できるのは「自分のエンコーダとデコーダが整合しているか」だけ。
   MODELS のレンジ（p_lim/v_lim/t_lim/kp_max/kd_max）が実機と一致しているかは
   ここでは分からない。それは A9 の実測でしか確定しない。
"""

import sys
import types

# python-can が無い環境でも cubemars.py を import できるようにする
if "can" not in sys.modules:
    try:
        import can  # noqa: F401
    except ImportError:
        stub = types.ModuleType("can")
        stub.Message = object
        stub.CanError = Exception
        stub.Bus = object
        sys.modules["can"] = stub

import cubemars as cm


def unpack_mit_cmd(data, model):
    """f_mit が作った8バイトを、仕様どおりに独立して解きほぐす"""
    m = cm.MODELS[model]
    p_int = (data[0] << 8) | data[1]
    v_int = (data[2] << 4) | (data[3] >> 4)
    kp_int = ((data[3] & 0x0F) << 8) | data[4]
    kd_int = (data[5] << 4) | (data[6] >> 4)
    t_int = ((data[6] & 0x0F) << 8) | data[7]
    return dict(
        pos=cm.uint_to_float(p_int, -m["p_lim"], m["p_lim"], 16),
        vel=cm.uint_to_float(v_int, -m["v_lim"], m["v_lim"], 12),
        kp=cm.uint_to_float(kp_int, 0.0, m["kp_max"], 12),
        kd=cm.uint_to_float(kd_int, 0.0, m["kd_max"], 12),
        torque=cm.uint_to_float(t_int, -m["t_lim"], m["t_lim"], 12),
    )


def lsb(x_min, x_max, bits):
    return (x_max - x_min) / ((1 << bits) - 1)


fails = []


def check(name, got, want, tol):
    ok = abs(got - want) <= tol
    if not ok:
        fails.append(f"{name}: 期待 {want:+.6f} / 実際 {got:+.6f} (許容 {tol:.6f})")
    return ok


print("=" * 64)
print("1. 量子化ステップ（1LSB = この刻みでしか送れない）")
print("=" * 64)
for model in ("AK10-9", "AK80-9"):
    m = cm.MODELS[model]
    print(f"[{model}]")
    print(f"  位置   16bit  +-{m['p_lim']:>5.1f} rad     1LSB = {lsb(-m['p_lim'], m['p_lim'], 16)*1000:.4f} mrad "
          f"({lsb(-m['p_lim'], m['p_lim'], 16)*57.2958:.5f} deg)")
    print(f"  速度   12bit  +-{m['v_lim']:>5.1f} rad/s   1LSB = {lsb(-m['v_lim'], m['v_lim'], 12):.5f} rad/s")
    print(f"  Kp     12bit   0..{m['kp_max']:<5.0f}       1LSB = {lsb(0, m['kp_max'], 12):.5f}")
    print(f"  Kd     12bit   0..{m['kd_max']:<5.1f}       1LSB = {lsb(0, m['kd_max'], 12):.6f}")
    print(f"  トルク 12bit  +-{m['t_lim']:>5.1f} N*m     1LSB = {lsb(-m['t_lim'], m['t_lim'], 12):.5f} N*m")

print()
print("=" * 64)
print("2. 往復テスト（詰めて → 解いて → 元に戻るか）")
print("=" * 64)
CASES = [
    # pos[rad], vel[rad/s], kp, kd, torque[N*m]
    (0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 2.0, 0.1, 0.0),      # ステップ4の初期値
    (0.0, 0.0, 15.0, 1.5, 0.0),     # シムの stiffness/damping
    (1.5708, 0.0, 5.0, 0.5, 0.0),   # 90度
    (-1.5708, 0.0, 5.0, 0.5, 0.0),
    (0.5, 10.0, 50.0, 2.0, 3.0),
    (-0.5, -10.0, 50.0, 2.0, -3.0),
    (12.5, 50.0, 500.0, 5.0, 18.0),  # 上限
    (-12.5, -50.0, 0.0, 0.0, -18.0),  # 下限
]
for model in ("AK10-9", "AK80-9"):
    m = cm.MODELS[model]
    tol_p = lsb(-m["p_lim"], m["p_lim"], 16)
    tol_v = lsb(-m["v_lim"], m["v_lim"], 12)
    tol_kp = lsb(0, m["kp_max"], 12)
    tol_kd = lsb(0, m["kd_max"], 12)
    tol_t = lsb(-m["t_lim"], m["t_lim"], 12)
    print(f"[{model}]")
    for pos, vel, kp, kd, tq in CASES:
        fr = cm.f_mit(43, pos, vel, kp, kd, tq, model=model)
        got = unpack_mit_cmd(fr.data, model)
        # 送った値がレンジ外ならクリップされた値が正解
        w_pos = max(-m["p_lim"], min(m["p_lim"], pos))
        w_vel = max(-m["v_lim"], min(m["v_lim"], vel))
        w_kp = max(0.0, min(m["kp_max"], kp))
        w_kd = max(0.0, min(m["kd_max"], kd))
        w_tq = max(-m["t_lim"], min(m["t_lim"], tq))
        tag = f"pos={pos:+.4f} vel={vel:+.1f} kp={kp:.1f} kd={kd:.2f} T={tq:+.1f}"
        ok = True
        ok &= check(f"{model} {tag} pos", got["pos"], w_pos, tol_p)
        ok &= check(f"{model} {tag} vel", got["vel"], w_vel, tol_v)
        ok &= check(f"{model} {tag} kp", got["kp"], w_kp, tol_kp)
        ok &= check(f"{model} {tag} kd", got["kd"], w_kd, tol_kd)
        ok &= check(f"{model} {tag} torque", got["torque"], w_tq, tol_t)
        print(f"  {'OK ' if ok else 'NG '} {tag}   -> {fr.data.hex(' ')}")

print()
print("=" * 64)
print("3. クリップ（レンジ外を投げても8バイトに収まるか）")
print("=" * 64)
for pos, vel, kp, kd, tq in [(999, 999, 9999, 99, 999), (-999, -999, -50, -1, -999)]:
    fr = cm.f_mit(43, pos, vel, kp, kd, tq, model="AK80-9")
    got = unpack_mit_cmd(fr.data, "AK80-9")
    assert len(fr.data) == 8, "8バイトでない"
    print(f"  入力 pos={pos} vel={vel} kp={kp} kd={kd} T={tq}")
    print(f"    -> {fr.data.hex(' ')}  = pos {got['pos']:+.3f} vel {got['vel']:+.2f} "
          f"kp {got['kp']:.2f} kd {got['kd']:.3f} T {got['torque']:+.3f}")

print()
print("=" * 64)
print("4. 機種取り違えの影響（同じトルク値が別のバイトになる）")
print("=" * 64)
a10 = cm.f_mit(43, 0, 0, 0, 0, 5.0, model="AK10-9")
a80 = cm.f_mit(43, 0, 0, 0, 0, 5.0, model="AK80-9")
print(f"  T=+5.0 N*m  AK10-9 -> {a10.data.hex(' ')}")
print(f"  T=+5.0 N*m  AK80-9 -> {a80.data.hex(' ')}")
mis = unpack_mit_cmd(a80.data, "AK10-9")
print(f"  AK80-9のつもりで作った 5.0 N*m を AK10-9 のモーターが読むと {mis['torque']:+.2f} N*m "
      f"（{mis['torque']/5.0:.2f}倍）")

print()
print("=" * 64)
print("5. 特殊フレーム")
print("=" * 64)
for name, fn in (("enable", cm.f_mit_enable), ("disable", cm.f_mit_disable), ("set_zero", cm.f_mit_set_zero)):
    fr = fn(43)
    print(f"  {name:9s} ID=0x{fr.arbitration_id:03X} ext={fr.is_extended_id} DATA={fr.data.hex(' ')}")
print(f"  現在の ID 方式: {cm.mit_scheme_desc()}")

print()
print("=" * 64)
print("6. 応答デコード（parse_mit_reply の往復）")
print("=" * 64)


class FakeMsg:
    def __init__(self, arb, data):
        self.arbitration_id = arb
        self.data = bytes(data)
        self.dlc = len(data)
        self.is_extended_id = False
        self.is_rx = True


def build_mit_reply(motor_id, pos, vel, cur, model):
    m = cm.MODELS[model]
    p = cm.float_to_uint(pos, -m["p_lim"], m["p_lim"], 16)
    v = cm.float_to_uint(vel, -m["v_lim"], m["v_lim"], 12)
    i = cm.float_to_uint(cur, -m["t_lim"], m["t_lim"], 12)
    return bytes([motor_id, (p >> 8) & 0xFF, p & 0xFF,
                  (v >> 4) & 0xFF, ((v & 0x0F) << 4) | ((i >> 8) & 0x0F), i & 0xFF])


for pos, vel, cur in [(0.0, 0.0, 0.0), (1.5708, 3.0, 2.5), (-1.5708, -3.0, -2.5)]:
    data = build_mit_reply(43, pos, vel, cur, "AK80-9")
    got = cm.parse_mit_reply(FakeMsg(0x000, data), "AK80-9")
    m = cm.MODELS["AK80-9"]
    check(f"reply pos {pos}", got["pos"], pos, lsb(-m["p_lim"], m["p_lim"], 16))
    check(f"reply spd {vel}", got["spd"], vel, lsb(-m["v_lim"], m["v_lim"], 12))
    check(f"reply cur {cur}", got["cur"], cur, lsb(-m["t_lim"], m["t_lim"], 12))
    print(f"  id={got['id']} pos {got['pos']:+.5f} rad / spd {got['spd']:+.4f} rad/s / "
          f"cur {got['cur']:+.4f}   ({data.hex(' ')})")

print()
print("=" * 64)
if fails:
    print(f"NG が {len(fails)} 件")
    for f in fails:
        print("  " + f)
    sys.exit(1)
print("すべて一致。エンコーダとデコーダは整合している。")
print("※ レンジそのものが実機と合っているかは A9 の実測で確認すること。")
print("=" * 64)

"""
mitmode_test_ver1.py -- MIT疎通の最小確認 & A9/A10 の実測（ステップ1）

【このファイルで何をするか】
  1. MITモードに本当に入ったかを、コードから判定する
  2. A10: MIT応答フレームの形式（どのCAN IDで、どんなバイト並びで返るか）を実測
  3. A9: 位置レンジ p_lim を、手回しで実測

【安全】
  ★このファイルが送るMITフレームは すべて kp=0 / kd=0 / torque=0。
    つまりモーターは一切トルクを出さない。軸は自分からは動かない。
    （range モードでは人間が手で回す。モーターは自由回転する＝脚は付けない）
  ★止め方: Ctrl+C。本当の非常停止は電源を切ること。

【使い方】 cd C:\\Users\\harut\\Connect2USB2CAN
  python mitmode_test_ver1.py             ← 疎通確認（軸は動かない）
  python mitmode_test_ver1.py range       ← 手回しで位置レンジを実測
  python mitmode_test_ver1.py --id 43 --model AK80-9

【ver の区切り】
  ver1 = ここ（疎通・A10応答形式・A9位置レンジ）
  ver2 = 零トルク保持とバックドライブ確認
  ver3 = 低ゲイン位置保持 → Kp段階上げ → リミットサイクル（C6）がMITで消えるか
  ver4 = 90度ステップ応答と ps（サーボ）との比較
"""

import argparse
import sys
import time

import cubemars as cm

RESULT_FILE = "mit_probe_ver1.txt"
_lines = []


def say(s=""):
    print(s)
    _lines.append(s)


def dump_capture(cap, title):
    say(f"  --- {title} ---")
    if not cap:
        say("    （1フレームも受信していない）")
        return
    for arb, e in sorted(cap.items()):
        kind = "サーボFB" if (arb >> 8) == cm.STATUS_PACKET else "?"
        say(f"    ID=0x{arb:08X} {'ext' if e['ext'] else 'std'} DLC={e['dlc']} "
            f"{e['n']:4d}回  最後={e['last'].hex(' ')}  [{kind}]")


def probe(bus, mid, model):
    say("=" * 62)
    say("ステップ0  MITに本当に入ったかの判定")
    say("=" * 62)

    say("[1] サーボ定期フィードバックが来ているか（MIT前のベースライン）")
    before = bus.capture(2.0)
    dump_capture(before, "MIT enable 前の2秒")
    servo_before = sum(e["n"] for a, e in before.items()
                       if (a >> 8) == cm.STATUS_PACKET)
    say(f"  サーボFB {servo_before} 回 / 2秒 = {servo_before/2.0:.1f} Hz")
    if servo_before == 0:
        say("  → サーボFBが来ていない。MITに切り替わった可能性と、単に")
        say("     バスが落ちている・電源が入っていない可能性の両方がある。")
    say()

    say("[2] MIT enable を送る（トルクは出ない。軸は動かない）")
    say(f"  ID方式: {cm.mit_scheme_desc()}  送信ID=0x{cm.mit_arbitration_id(mid):03X}")
    bus.mit_rx = True
    bus.mit_enable(mid, quiet=False)
    time.sleep(0.3)

    say()
    say("[3] 零トルク・零ゲインのMITフレームを 50Hz で3秒送る")
    say("    （pos=0 vel=0 kp=0 kd=0 T=0 → モーターは何もしない）")
    bus.capture(0.01)
    sent = bus.hold(lambda t: cm.f_mit(mid, 0.0, 0.0, 0.0, 0.0, 0.0, model),
                    seconds=3.0, hz=50, sample_hz=0, stop_after=False)
    after = bus.capture(2.0)
    say(f"  送信 {sent} 回 / エコー除外 {bus.echo_count} 件")
    dump_capture(after, "MIT指令中の2秒")

    bus.mit_disable(mid, quiet=False)

    say()
    say("=" * 62)
    say("判定")
    say("=" * 62)
    servo_after = sum(e["n"] for a, e in after.items()
                      if (a >> 8) == cm.STATUS_PACKET)
    new_ids = sorted(set(after) - set(before))
    say(f"  サーボFB: {servo_before/2.0:.1f} Hz → {servo_after/2.0:.1f} Hz")
    say(f"  新しく現れたCAN ID: {[hex(i) for i in new_ids] if new_ids else 'なし'}")
    say(f"  MIT応答として解釈できたID: "
        f"{[hex(i) for i in sorted(bus.mit_reply_ids)] if bus.mit_reply_ids else 'なし'}")
    if bus.rx_error:
        say(f"  ⚠受信スレッドが停止: {bus.rx_error}")

    st = bus.state(mid)
    if st is not None:
        say(f"  最後の状態: src={st.src}  pos={st.pos}  spd={st.spd}  cur={st.cur}")

    say()
    if bus.mit_reply_ids:
        say("  ★MITに入っている可能性が高い。")
        say("    → 上の『最後={バイト列}』を A10（応答フレーム形式）として記録する。")
        say("    → 続けて `python mitmode_test_ver1.py range` で位置レンジを実測する。")
    elif servo_after > 0:
        say("  ✕ サーボ形式のフィードバックが流れ続けている＝MITに入っていない。")
        say("    → CubeMarsTool の Mode Switch をやり直す。")
        say("      それでも駄目なら CubeMarsTool V1.32 が V3.0 に未対応の疑い。")
        say("      最新版『AK Series V3.23 Upper Computer』への入れ替えを検討する。")
    else:
        say("  ▲ 何も返ってきていない。次の3つのどれか:")
        say("    (a) フィードバックが『応答式』設定になっている（上位機で『定期』にする）")
        say("    (b) MITの送受信IDが違う（`mscheme` / `mprobe` で当たりを取る）")
        say("    (c) バスが落ちている（電源を入れ直す。⚠`ext`方式は絶対に使わない）")


def measure_range(bus, mid, model):
    say("=" * 62)
    say("A9  位置レンジ p_lim の実測（手回し）")
    say("=" * 62)
    say("やること: kp=0 / kd=0 で送り続けるのでモーターは自由回転する。")
    say("          その状態で、出力軸を手でゆっくり回す。")
    say("          MITが返す pos と、実際に回した角度を突き合わせる。")
    say()
    say(f"  今の MODELS[{model}] の仮定: p_lim = ±{cm.MODELS[model]['p_lim']} rad "
        f"(= ±{cm.MODELS[model]['p_lim']*57.2958:.0f} deg)")
    say("  ★この仮定が違えば、表示される角度が実際と比例ズレする。")
    say("    例: 手で90度回したのに 45deg と出たら p_lim は仮定の2倍。")
    say()
    say("  ⚠ 脚・負荷は外しておくこと。自由回転する。")
    input("  準備ができたら Enter を押してください（止めるときは Ctrl+C）> ")

    bus.mit_rx = True
    bus.mit_enable(mid)
    time.sleep(0.3)

    pos0 = [None]

    def sample(elapsed):
        s = bus.state(mid)
        if s is None:
            print(f"  {elapsed:5.1f}s  状態なし")
            return
        if s.src == "mit":
            if pos0[0] is None:
                pos0[0] = s.pos
            d = s.pos - pos0[0]
            print(f"  {elapsed:5.1f}s  [MIT] pos={s.pos:+8.4f} rad "
                  f"({s.pos*57.2958:+7.1f} deg)  開始から {d*57.2958:+7.1f} deg  "
                  f"spd={s.spd:+6.2f} rad/s")
        else:
            print(f"  {elapsed:5.1f}s  [servo] pos={s.pos:+7.1f} deg  "
                  f"（MIT応答ではない）")

    try:
        bus.hold(lambda t: cm.f_mit(mid, 0.0, 0.0, 0.0, 0.0, 0.0, model),
                 seconds=30.0, hz=50, on_sample=sample, sample_hz=4,
                 stop_after=False)
    except KeyboardInterrupt:
        say("  中断しました")
    finally:
        bus.mit_disable(mid)

    say()
    say("  記録するもの:")
    say("   ・手で回した実角度（分度器やマーキングで測る） vs 表示された deg")
    say("   ・比が 1.0 でなければ p_lim を その比 倍して MODELS を直す")
    say("   ・結果は claude/motor_can_findings.md の A9 に書き戻す")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", nargs="?", default="probe", choices=["probe", "range"])
    ap.add_argument("--id", type=int, default=43)
    ap.add_argument("--model", default="AK80-9", choices=list(cm.MODELS))
    ap.add_argument("--channel", type=int, default=1)
    a = ap.parse_args()

    say(f"モーターID={a.id}  モデル={a.model}  channel={a.channel}")
    say("送るMITフレームは すべて kp=0 / kd=0 / torque=0（トルクを出さない）")
    say(f"日時: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    say()

    bus = cm.MotorBus(channel=a.channel, model=a.model)
    bus.open()
    try:
        if a.mode == "probe":
            probe(bus, a.id, a.model)
        else:
            measure_range(bus, a.id, a.model)
    except KeyboardInterrupt:
        say("\n  Ctrl+C で中断")
    finally:
        bus.close(stop_motors=True)

    with open(RESULT_FILE, "w", encoding="utf-8") as f:
        f.write("\n".join(_lines) + "\n")
    print(f"\n結果を {RESULT_FILE} に保存しました。")


if __name__ == "__main__":
    main()

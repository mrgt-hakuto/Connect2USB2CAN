# -*- coding: utf-8 -*-
"""
T265 の pose を読み、方策の観測 3 項目（base_lin_vel / base_ang_vel / projected_gravity）を
胴体座標（+X 前・+Y 左・+Z 上）で表示・CSV 保存する確認用スクリプト（M6）。モーターは使わない。

  cd C:\\Users\\harut\\Connect2USB2CAN\\t265
  python t265_check.py              # 10 Hz で表示、Ctrl+C で終了。CSV は t265\\logs\\ に保存
  python t265_check.py --raw        # T265 の生の値（T265 座標）も並べて出す

前提（realsense_t265.md）:
  - pyrealsense2 は 2.53.1 以下（2.54 以降は T265 非対応）。Windows の wheel は Python 3.9 / 3.10 まで。
  - T265 座標: X 右・Y 上・Z 後ろ。world は起動時の姿勢が原点で Y が重力と逆。
  - velocity / angular_velocity が world 表現か本体表現かは資料で断定しない → VEL_IN_WORLD を手で動かして決める。
  - 取り付け向き R_CB と取り付け位置 R_OFFSET は仮置き。符号確認のあとで直す。
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("pyrealsense2 が入っていません。README の手順で Python 3.10 の venv に 2.53.1.4623 を入れてください。")
    sys.exit(1)

# ---- 取り付けの仮定（ここを実測で直す）----------------------------------------
# v_c = R_CB @ v_b（胴体座標のベクトルを T265 座標で表す）。列 = 胴体の x, y, z 軸を T265 座標で書いたもの。
# 前向き・水平取り付け: x_b = -z_c, y_b = -x_c, z_b = +y_c
R_CB = np.array([[0.0, -1.0, 0.0],
                 [0.0, 0.0, 1.0],
                 [-1.0, 0.0, 0.0]])
# 胴体原点 → T265 のベクトル [m]（胴体座標）。CAD から取るまで 0。base 原点は股の中点から横に 2 cm（robot_model_conventions）。
R_OFFSET = np.array([0.0, 0.0, 0.0])
# pose.velocity / angular_velocity を world 表現とみなすか（False なら T265 本体表現）
VEL_IN_WORLD = True
# 途絶判定
STALE_S = 0.05        # 最後のフレームからこれ以上空いたら途絶（200 Hz なので 10 フレーム）
MIN_CONFIDENCE = 2    # 0 失敗 / 1 低 / 2 中 / 3 高
G_WORLD = np.array([0.0, -1.0, 0.0])  # T265 world の重力方向（単位ベクトル）
# ---------------------------------------------------------------------------------


def quat_to_R(x, y, z, w):
    """本体(T265)→world の回転行列"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def to_base(pose):
    q = pose.rotation
    R_wc = quat_to_R(q.x, q.y, q.z, q.w)
    v = np.array([pose.velocity.x, pose.velocity.y, pose.velocity.z])
    w = np.array([pose.angular_velocity.x, pose.angular_velocity.y, pose.angular_velocity.z])
    if VEL_IN_WORLD:
        v_c, w_c = R_wc.T @ v, R_wc.T @ w
    else:
        v_c, w_c = v, w
    w_b = R_CB.T @ w_c
    v_cam_b = R_CB.T @ v_c
    v_b = v_cam_b - np.cross(w_b, R_OFFSET)          # v_base = v_cam − ω × r
    g_b = R_CB.T @ (R_wc.T @ G_WORLD)                # projected_gravity
    return v_b, w_b, g_b, v, w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", action="store_true", help="T265 座標の生の速度も表示")
    ap.add_argument("--hz", type=float, default=10.0, help="表示の周期")
    ap.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()

    print(f"pyrealsense2 {getattr(rs, '__version__', '(版の属性なし)')}")
    # 実績（2026-09-17）: pipeline を毎回新しく作る＋serial 指定の版は 5 回目で start できた。
    # 同じ context で query_devices し直す版は "Unable to create USB device" で全滅した → 前者に戻す。
    # 失敗後にネイティブ側で黙って落ちることがあるので、起動し直しは親（supervise）が担当。
    ctx = rs.context()
    devs = list(ctx.query_devices())
    if not devs:
        print("T265 が見つかりません。")
        sys.exit(3)
    d = devs[0]
    serial = d.get_info(rs.camera_info.serial_number)
    print(f"  デバイス: {d.get_info(rs.camera_info.name)} {serial} FW {d.get_info(rs.camera_info.firmware_version)}")
    del devs, d, ctx
    pipe = None
    for k in range(1, 9):
        try:
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.pose)
            pipe.start(cfg)
            print(f"  start 成功（{k} 回目）")
            break
        except RuntimeError as e:
            print(f"  start 失敗（{k} 回目）: {e} → 2 秒待つ")
            pipe = None
            time.sleep(2.0)
    if pipe is None:
        print("T265 を開始できませんでした。")
        sys.exit(3)

    os.makedirs(os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"), exist_ok=True)
    fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs",
                      "t265_" + time.strftime("%Y%m%d_%H%M%S") + ".csv")
    fh = open(fn, "w", newline="")
    wr = csv.writer(fh)
    wr.writerow(["t_host", "frame", "conf", "vx_b", "vy_b", "vz_b", "wx_b", "wy_b", "wz_b", "gx_b", "gy_b", "gz_b",
                 "vx_raw", "vy_raw", "vz_raw", "wx_raw", "wy_raw", "wz_raw", "px", "py", "pz"])
    print(f"CSV: {fn}")
    print("静止・水平で g_b ≒ (0, 0, -1) か確認 → 頭を下げると g_b.x が +、左を下げると g_b.y が +（Isaac Lab と同じ向きか）")

    t0 = time.perf_counter()
    last_frame_t = None
    last_print = 0.0
    n_frames, n_stale, max_gap = 0, 0, 0.0
    stale = False
    try:
        while True:
            try:
                frames = pipe.wait_for_frames(100)
            except RuntimeError:
                frames = None
            now = time.perf_counter() - t0
            if frames:
                pf = frames.get_pose_frame()
                if pf:
                    if last_frame_t is not None:
                        max_gap = max(max_gap, now - last_frame_t)
                    last_frame_t = now
                    n_frames += 1
                    p = pf.get_pose_data()
                    v_b, w_b, g_b, v_raw, w_raw = to_base(p)
                    wr.writerow([f"{now:.4f}", pf.frame_number, p.tracker_confidence, *np.round(v_b, 4),
                                 *np.round(w_b, 4), *np.round(g_b, 4), *np.round(v_raw, 4), *np.round(w_raw, 4),
                                 round(p.translation.x, 4), round(p.translation.y, 4), round(p.translation.z, 4)])
                    bad = p.tracker_confidence < MIN_CONFIDENCE
                    if now - last_print >= 1.0 / args.hz:
                        last_print = now
                        s = (f"{now:7.2f}s conf{p.tracker_confidence}{'!' if bad else ' '} "
                             f"v_b({v_b[0]:+.2f},{v_b[1]:+.2f},{v_b[2]:+.2f}) "
                             f"w_b({w_b[0]:+.2f},{w_b[1]:+.2f},{w_b[2]:+.2f}) "
                             f"g_b({g_b[0]:+.2f},{g_b[1]:+.2f},{g_b[2]:+.2f})")
                        if args.raw:
                            s += f" | raw v({v_raw[0]:+.2f},{v_raw[1]:+.2f},{v_raw[2]:+.2f}) w({w_raw[0]:+.2f},{w_raw[1]:+.2f},{w_raw[2]:+.2f})"
                        print(s)
            if last_frame_t is not None and now - last_frame_t > STALE_S:
                if not stale:
                    n_stale += 1
                    print(f"  ⚠ 途絶 {now - last_frame_t:.3f}s")
                stale = True
            else:
                stale = False
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        fh.close()
        el = time.perf_counter() - t0
        print(f"フレーム {n_frames}（{n_frames / max(el, 1e-6):.1f} Hz）、途絶 {n_stale} 回、最大間隔 {max_gap * 1000:.1f} ms")
        print(f"CSV: {fn}")


def supervise():
    """子プロセスで main を回す。ネイティブ側の異常終了（トレースバックなしで消える）でも起動し直す。"""
    import subprocess
    cmd = [sys.executable, os.path.abspath(__file__), "--child"] + sys.argv[1:]
    for n in range(1, 11):
        try:
            rc = subprocess.call(cmd)
        except KeyboardInterrupt:
            return
        if rc == 0:
            return
        print(f"--- 子プロセスが終了コード {rc} で終わりました（{n} 回目）→ 3 秒後に起動し直します。止めるなら Ctrl+C")
        try:
            time.sleep(3.0)
        except KeyboardInterrupt:
            return


if __name__ == "__main__":
    if "--child" in sys.argv:
        main()
    else:
        supervise()

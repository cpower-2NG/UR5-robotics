#!/usr/bin/env python3
"""巡检位**看得见哪一片地面**：把画面四角按 K 与 base←camera 打到 z=0 上，
再判断题里那个拾取区是不是**整个**落在画面里（带像素边距）。

为什么值得单独一个脚本
----------------------
相机不会因为「配置的 pick_zone 写了多大」就看多大。2026-10-25 实测：巡检位
（tool0 z=364 mm，视线竖直向下）的地面足迹只有

    x ∈ [254, 736] mm   y ∈ [225, 498] mm

而板上的 `pick_zone` 是 x∈[0.40,0.60]、y∈[0.00,0.45] —— **近端 y<0.225 整段
在画面外**。那里的块永远检不出来，计数会悄悄少一块，而现场看不出来。
所以拾取区的四角必须逐帧验：不是「按理说看得见」，是**算出来在里面**。

只读：只读一个帧目录里的 `tf.json` / `meta.json`，不连板子、不动臂。

两种给位姿的方式（都不动臂）：

- 默认：用帧里的 `tf.json` 整条链 —— **前提是这帧就是在巡检位拍的**。
  没加 `--go` 抓的帧不在巡检位，算出来是当前位姿的足迹，别拿它下结论。
- `--q "a,b,c,d,e,f" --urdf R`：用 urkin 正解算 base←tool0，再叠 tf.json
  里的静态 tool0→相机。这条能**在动臂之前**就验计划好的巡检位看不看得见，
  正是计划里「离线第 0 步」要的那一步。

用法：
  python3 survey_footprint.py --frame F --zone 0.38,0.62,0.25,0.45
  python3 survey_footprint.py --frame F --zone ... --q "$Q" --urdf robot.urdf
  python3 survey_footprint.py --frame F --zone ... --json out.json

退出码：0 四角都在画面内（含边距）；2 有角出画面或射线打不到地面。
"""
import argparse
import json
import os
import sys

import numpy as np

# base_link → 相机光心：与 precision_capture.CHAIN_KEYS 同一条链
CHAIN = (
    ("base_link", "base_link_inertia"), ("base_link_inertia", "shoulder_link"),
    ("shoulder_link", "upper_arm_link"), ("upper_arm_link", "forearm_link"),
    ("forearm_link", "wrist_1_link"), ("wrist_1_link", "wrist_2_link"),
    ("wrist_2_link", "wrist_3_link"), ("wrist_3_link", "flange"),
    ("flange", "tool0"), ("tool0", "camera_link"),
    ("camera_link", "camera_color_frame"),
    ("camera_color_frame", "camera_color_optical_frame"),
)


def _quat_to_R(q):
    x, y, z, w = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _tf_matrix(e):
    M = np.eye(4)
    M[:3, :3] = _quat_to_R([e["rotation"][k] for k in "xyzw"])
    M[:3, 3] = [e["translation"][k] for k in "xyz"]
    return M


def cam_pose(frame_dir, q=None, urdf=None):
    """base_link ← camera_color_optical_frame，返回 (R, t)。缺任何一帧就报出来。

    q 给定就用 urkin 正解（在动臂之前验计划位姿）；不给就整条链取自 tf.json。
    """
    tf = json.load(open(os.path.join(frame_dir, "tf.json")))["ok"]
    if q is not None:
        import urkin
        T = urkin.Chain(urdf or os.path.join(frame_dir, "..", "..", "robot.urdf")).fk(
            np.array(q, float))
    else:
        T = np.eye(4)
        for a, b in CHAIN[:-3]:
            key = "%s:%s" % (a, b)
            if key not in tf:
                raise SystemExit("tf.json 缺 %s —— 这条链不完整，地面足迹算不出来" % key)
            T = T @ _tf_matrix(tf[key])
    for a, b in CHAIN[-3:]:            # tool0 → 相机：全是静态帧，与臂位姿无关
        key = "%s:%s" % (a, b)
        if key not in tf:
            raise SystemExit("tf.json 缺 %s —— 这条链不完整，地面足迹算不出来" % key)
        T = T @ _tf_matrix(tf[key])
    return T[:3, :3], T[:3, 3]


def camera_k(frame_dir):
    meta = json.load(open(os.path.join(frame_dir, "meta.json")))
    return (np.array(meta["camera_info"]["K"], float).reshape(3, 3),
            int(meta["color"]["width"]), int(meta["color"]["height"]))


def ground_hit(R, o, K, u, v, ground_z=0.0):
    """像素 (u,v) 的射线与 z=ground_z 平面的交点（米）；射线朝上则返回 None。"""
    d = R @ np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
    if abs(d[2]) < 1e-9:
        return None
    s = (ground_z - o[2]) / d[2]
    return None if s <= 0 else o + s * d


def project(R, o, K, P):
    """地面点 (x,y,0) → 像素；在相机后面返回 None。"""
    pc = R.T @ (np.asarray(P, float) - o)
    if pc[2] <= 0:
        return None
    p = K @ pc
    return float(p[0] / p[2]), float(p[1] / p[2])


def footprint(frame_dir, q=None, urdf=None):
    R, o = cam_pose(frame_dir, q, urdf)
    K, W, H = camera_k(frame_dir)
    pts = [ground_hit(R, o, K, u, v) for u, v in ((0, 0), (W, 0), (W, H), (0, H))]
    return {"R": R, "o": o, "K": K, "W": W, "H": H,
            "corners": [p for p in pts if p is not None],
            "n_off_ground": sum(p is None for p in pts)}


def zone_in_view(frame_dir, zone, margin_px=40, ground_z=0.0, q=None, urdf=None):
    """拾取区四角是否都在画面内（含边距）。返回 dict，`ok` 是总判据。"""
    fp = footprint(frame_dir, q, urdf)
    R, o, K = fp["R"], fp["o"], fp["K"]
    W, H = fp["W"], fp["H"]
    x0, x1, y0, y1 = zone
    rows = []
    for name, (px, py) in (("近左", (x0, y0)), ("近右", (x1, y0)),
                           ("远右", (x1, y1)), ("远左", (x0, y1))):
        uv = project(R, o, K, (px, py, ground_z))
        inside = (uv is not None
                  and margin_px <= uv[0] <= W - margin_px
                  and margin_px <= uv[1] <= H - margin_px)
        rows.append({"corner": name, "xy_mm": [round(px * 1000, 1), round(py * 1000, 1)],
                     "uv": [round(uv[0], 1), round(uv[1], 1)] if uv else None,
                     "inside": bool(inside)})
    return {"ok": all(r["inside"] for r in rows) and fp["n_off_ground"] == 0,
            "margin_px": margin_px, "image": [W, H], "corners": rows,
            "ground_footprint_mm": {
                "x": [round(min(p[0] for p in fp["corners"]) * 1000, 1),
                      round(max(p[0] for p in fp["corners"]) * 1000, 1)],
                "y": [round(min(p[1] for p in fp["corners"]) * 1000, 1),
                      round(max(p[1] for p in fp["corners"]) * 1000, 1)],
            } if fp["corners"] else None,
            "camera_base_mm": [round(v * 1000, 1) for v in fp["o"]]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", required=True)
    ap.add_argument("--zone", required=True, help="x_min,x_max,y_min,y_max（米）")
    ap.add_argument("--margin-px", type=int, default=40)
    ap.add_argument("--q", default=None, help="6 个关节角，逗号分隔（在动臂前验计划位姿）")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()
    zone = [float(v) for v in args.zone.split(",")]
    q = [float(v) for v in args.q.split(",")] if args.q else None
    res = zone_in_view(args.frame, zone, args.margin_px, q=q, urdf=args.urdf)
    res["pose_source"] = "urkin" if q else "tf.json"
    f = res["ground_footprint_mm"]
    print(("位姿来源 %s；相机 base_link (%7.1f, %7.1f, %7.1f) mm"
           % ((res["pose_source"],) + tuple(res["camera_base_mm"]))))
    if f:
        print("地面足迹 x ∈ [%.1f, %.1f]  y ∈ [%.1f, %.1f] mm"
              % (f["x"][0], f["x"][1], f["y"][0], f["y"][1]))
    for r in res["corners"]:
        print("  %s %s mm → 像素 %s  %s"
              % (r["corner"], r["xy_mm"], r["uv"],
                 "✓" if r["inside"] else "✗ **出画面**"))
    print("⇒ %s" % ("拾取区整个在画面内" if res["ok"] else "**拾取区有角出画面**"))
    if args.json:
        json.dump(res, open(args.json, "w"), ensure_ascii=False, indent=1)
    return 0 if res["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())

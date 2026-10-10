#!/usr/bin/env python3
"""解腕部相机内参：读 `wrist_intrinsics_capture.py` 收下的会话目录。

输出四样东西
------------
1. **K / D / 重投影 RMS**，以及每帧的残差，把最差的几帧点名出来。
2. **和出厂内参的对比**。这一条不只是看"标得对不对"——台账第五十八轮留下一个
   **悬案**：那条链上有 **1.2%** 的尺度偏差，而"深度尺度"和"焦距错"在**平面靶
   上数学不可分**。内参标定**不用深度**，所以：
   - 标出来的 fx 和出厂 fx 差 ≈ 1.2% ⇒ 偏差在**内参**；
   - 两者几乎一样 ⇒ 偏差在**深度尺度**。
   ⇒ 这一次顺便把那个悬案判了。
3. **交叉验证**：按奇偶把视图劈成两半，一半标、另一半评（只看没参与标定的那些帧
   的残差）。这比"重投影 RMS 好看"有说服力得多 —— 后者是**自己评自己**。
4. 一份 ROS `camera_calibration` 格式的 YAML，可直接喂给驱动/其他节点。

用法
----
    python3 scripts/wrist_intrinsics_solve.py --session <板上工作区>/calib/<会话名>
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np


def _sharp(view: dict) -> float:
    """清晰度。老会话只有整幅的 `laplacian_var`，新会话记的是**板 ROI** 的。"""
    return float(view.get("board_laplacian_var", view.get("laplacian_var", float("nan"))))


def _board_wh(view: dict) -> tuple[float, float]:
    if "board_width_px" in view:
        return float(view["board_width_px"]), float(view["board_height_px"])
    pts = np.array(view["corners"], dtype=np.float64).reshape(-1, 2)
    return float(pts[:, 0].max() - pts[:, 0].min()), float(pts[:, 1].max() - pts[:, 1].min())


def _load_session(session_dir: str):
    with open(os.path.join(session_dir, "session.json"), encoding="utf-8") as f:
        meta = json.load(f)
    views = []
    for path in sorted(glob.glob(os.path.join(session_dir, "view_*.json"))):
        with open(path, encoding="utf-8") as f:
            views.append(json.load(f))
    return meta, views


def _points_for(view: dict, square_mm: float):
    cols = int(view["pattern_cols"])
    rows = int(view["pattern_rows"])
    sq = square_mm / 1000.0
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * sq
    imgp = np.array(view["corners"], dtype=np.float32).reshape(-1, 1, 2)
    return objp.reshape(-1, 1, 3), imgp


def _calibrate(views, square_mm: float, size: tuple[int, int]):
    objpoints, imgpoints = [], []
    for v in views:
        o, i = _points_for(v, square_mm)
        objpoints.append(o)
        imgpoints.append(i)
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)
    return rms, K, D, objpoints, imgpoints


def _per_view_rms(objpoints, imgpoints, K, D) -> list[float]:
    out = []
    for o, i in zip(objpoints, imgpoints):
        ok, rvec, tvec = cv2.solvePnP(o, i, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            out.append(float("nan"))
            continue
        proj, _ = cv2.projectPoints(o, rvec, tvec, K, D)
        diff = proj.reshape(-1, 2) - i.reshape(-1, 2)
        out.append(float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "--session",
        required=True,
        nargs="+",
        help="会话目录，可以给多个（会合并；板/格/分辨率必须一致）",
    )
    ap.add_argument("--square-mm", type=float, default=None, help="不给就用会话里记的")
    ap.add_argument("--out-yaml", default=None, help="不给我就写在会话目录里")
    args = ap.parse_args()

    metas, per_session = [], []
    for d in args.session:
        m, v = _load_session(d)
        metas.append(m)
        per_session.append((d, v))
    meta = metas[0]
    # 合并多个会话前先确认它们说的是同一块板、同一个分辨率 —— 不然会解出一个
    # 看着正常、其实两边对不上的结果。
    for d, m in zip(args.session[1:], metas[1:]):
        for key in ("square_mm", "cols", "rows", "image_width", "image_height"):
            if m.get(key) != meta.get(key):
                print(f"❌ {d} 的 {key}={m.get(key)} 与首个会话的 {meta.get(key)} 不一致，不能合并。")
                return 2
    views = [v for _, vs in per_session for v in vs]
    if len(views) < 8:
        print(f"❌ 只有 {len(views)} 帧，太少（内参至少要 8–10 帧，且姿态要散）。先多收一些。")
        return 2

    square_mm = args.square_mm if args.square_mm is not None else float(meta["square_mm"])
    size = (int(meta["image_width"]), int(meta["image_height"]))

    print(f"会话：{args.session}")
    for d, vs in per_session:
        print(f"  {d}  →  {len(vs)} 帧")
    print(f"  合计 {len(views)} 帧   图像 {size[0]}×{size[1]}   格 {square_mm} mm")
    dists = np.array([v["distance_mm"] for v in views])
    print(f"  距离范围 {dists.min():.0f} – {dists.max():.0f} mm（中位 {np.median(dists):.0f}）")
    rot90 = sum(1 for v in views if v.get("rot90"))
    print(f"  其中转 90° 才认出来的 {rot90} 帧（意料之中，腕部侧装）")
    print()

    rms, K, D, objpoints, imgpoints = _calibrate(views, square_mm, size)
    per_view = _per_view_rms(objpoints, imgpoints, K, D)

    print("=== 标定结果 ===")
    print(f"  重投影 RMS = {rms:.4f} px")
    print(f"  fx={K[0,0]:.3f}  fy={K[1,1]:.3f}  cx={K[0,2]:.3f}  cy={K[1,2]:.3f}")
    print(f"  畸变 [k1 k2 p1 p2 k3] = {np.array2string(D.ravel(), precision=6)}")
    print()

    fk = meta.get("factory_K")
    if fk:
        fk = np.array(fk, dtype=np.float64).reshape(3, 3)
        print("=== 和出厂内参对比（台账第五十八轮那 1.2% 悬案的判据）===")
        for name, i, j in (("fx", 0, 0), ("fy", 1, 1), ("cx", 0, 2), ("cy", 1, 2)):
            f, c = fk[i, j], K[i, j]
            print(f"  {name}: 出厂 {f:9.3f} → 标定 {c:9.3f}   比值 {c / f:.5f}  ({(c / f - 1) * 100:+.3f}%)")
        ratio = (K[0, 0] / fk[0, 0] + K[1, 1] / fk[1, 1]) / 2.0
        print()
        if abs(ratio - 1.0) < 0.003:
            print("  ⇒ 焦距和出厂值一致 ⇒ 那 1.2% 的偏差**不在内参**，指向**深度尺度**。")
        elif abs(ratio - 1.0) > 0.008:
            print("  ⇒ 焦距和出厂值**明显不同** ⇒ 那 1.2% 很可能就是**内参**。")
        else:
            print("  ⇒ 差异在 0.3%–0.8% 之间，**判不了**，只能说两边都有一点。")
        print()

    bad = sorted(range(len(per_view)), key=lambda i: -per_view[i])[:5]
    print("=== 最差的 5 帧（若明显高于中位，多半是那一帧动糊了）===")
    med = float(np.nanmedian(per_view))
    print(f"  中位 {med:.3f} px")
    for i in bad:
        print(
            f"  [{i:03d}] {per_view[i]:.3f} px   距离 {views[i]['distance_mm']:.0f} mm"
            f"  锐 {_sharp(views[i]):.0f}  板宽 {_board_wh(views[i])[0]:.0f} px"
        )
    print()

    # 这一节才是**内参标得好不好**的真正判据。
    # 重投影 RMS 再小，只要 88 个角点全都挤在画面正中一小块，
    # 畸变系数和主点就基本没有约束 —— 那是"自己评自己"评出来的漂亮数字。
    print("=== 角点在画面里铺开得够不够（决定内参可信度的真指标）===")
    w, h = size
    ux0 = uy0 = float("inf")
    ux1 = uy1 = float("-inf")
    widths = []
    for v in views:
        pts = np.array(v["corners"], dtype=np.float64).reshape(-1, 2)
        ux0, uy0 = min(ux0, pts[:, 0].min()), min(uy0, pts[:, 1].min())
        ux1, uy1 = max(ux1, pts[:, 0].max()), max(uy1, pts[:, 1].max())
        widths.append(pts[:, 0].max() - pts[:, 0].min())
    print(f"  所有帧的角点合起来覆盖：x {ux0:.0f}–{ux1:.0f} / {w}，y {uy0:.0f}–{uy1:.0f} / {h}")
    cov_x, cov_y = (ux1 - ux0) / w, (uy1 - uy0) / h
    print(f"  覆盖率：横 {100 * cov_x:.0f}%  纵 {100 * cov_y:.0f}%")
    print(f"  单帧板宽：最小 {min(widths):.0f} px、中位 {np.median(widths):.0f} px、最大 {max(widths):.0f} px")
    # **倾斜角**是内参标定里最要紧的一项：平面靶上焦距和距离简并，只有倾斜能拆开。
    tl = np.array([v["tilt_deg"] for v in views if "tilt_deg" in v])
    if len(tl):
        print(
            f"  倾斜角：最小 {tl.min():.1f}°  中位 {np.median(tl):.1f}°  最大 {tl.max():.1f}°"
            f"，>20° 的有 {int((tl > 20).sum())} / {len(tl)} 帧"
        )
    # 画面四角各 1/8 的区域里有没有落过角点 —— 畸变主要靠这里约束。
    quads = {"左上": 0, "右上": 0, "左下": 0, "右下": 0}
    for v in views:
        pts = np.array(v["corners"], dtype=np.float64).reshape(-1, 2)
        for tag, (mx, my) in (
            ("左上", (pts[:, 0] < w / 2, pts[:, 1] < h / 2)),
            ("右上", (pts[:, 0] > w / 2, pts[:, 1] < h / 2)),
            ("左下", (pts[:, 0] < w / 2, pts[:, 1] > h / 2)),
            ("右下", (pts[:, 0] > w / 2, pts[:, 1] > h / 2)),
        ):
            if np.any(mx & my):
                quads[tag] += 1
    print(f"  四个象限各有几帧落了角点：{quads}")
    tilt_ok = len(tl) == 0 or int((tl > 20).sum()) >= 6
    verdict = cov_x > 0.65 and cov_y > 0.65 and min(quads.values()) >= 3 and tilt_ok
    print(
        f"  ⇒ {'铺得开、也歪得够，内参可信' if verdict else '**数据不够散**（角点没铺满 / 大倾斜帧不够）—— 焦距或畸变约束偏弱'}"
    )
    print()

    # 交叉验证：奇偶劈半，互相评。只看"没参与标定的那一半"的残差。
    print("=== 交叉验证（一半标、另一半评，两边互评）===")
    idx_a = [i for i in range(len(views)) if i % 2 == 0]
    idx_b = [i for i in range(len(views)) if i % 2 == 1]
    for tag, train, test in (("奇数组标 → 偶数组评", idx_a, idx_b), ("偶数组标 → 奇数组评", idx_b, idx_a)):
        if len(train) < 5 or len(test) < 3:
            print(f"  {tag}: 帧太少，跳过")
            continue
        _, Kt, Dt, _, _ = _calibrate([views[i] for i in train], square_mm, size)
        held = _per_view_rms(
            [_points_for(views[i], square_mm)[0] for i in test],
            [_points_for(views[i], square_mm)[1] for i in test],
            Kt,
            Dt,
        )
        print(f"  {tag}: 留出集 RMS 中位 {np.nanmedian(held):.4f} px，最大 {np.nanmax(held):.4f} px")
        print(f"     该半解出的 fx={Kt[0,0]:.3f}（与全量差 {(Kt[0,0] / K[0,0] - 1) * 100:+.3f}%）")
    print()

    out_yaml = args.out_yaml or os.path.join(args.session[0], "wrist_intrinsics.yaml")
    with open(out_yaml, "w", encoding="utf-8") as f:
        f.write("# 由 scripts/wrist_intrinsics_solve.py 生成，ROS camera_calibration 格式\n")
        f.write("# 会话：" + "、".join(args.session) + "\n")
        f.write(f"# 帧数：{len(views)}   重投影 RMS：{rms:.4f} px\n")
        f.write(f"image_width: {size[0]}\nimage_height: {size[1]}\n")
        f.write("camera_name: wrist_camera\n")
        f.write("camera_matrix:\n  rows: 3\n  cols: 3\n  data:\n")
        for row in K:
            for val in row:
                f.write(f"  - {val:.10f}\n")
        f.write("distortion_model: plumb_bob\n")
        f.write("distortion_coefficients:\n  rows: 1\n  cols: 5\n  data:\n")
        for val in D.ravel():
            f.write(f"  - {val:.10f}\n")
    print(f"已写出 {out_yaml}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

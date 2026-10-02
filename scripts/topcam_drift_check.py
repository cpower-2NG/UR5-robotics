#!/usr/bin/env python3
"""俯视相机**漂了没有** —— 拿两张相隔很久的帧比一比，判定外参还作不作数。

为什么需要它
============
B 路外参是 `相机←靶`（①，拍一张）∘ `base←靶`（②，臂碰四个角）拼出来的。
**① 一旦过期，整个外参就废了** —— 而它会以两种完全不同的方式过期：

  · **靶纸被碰动** —— 台面没动、相机没动，只有那张纸挪了；
  · **相机被碰动（或自己转）** —— 台面、纸、积木**全都**在像里一起挪。

这两种在原文档里只有一条判据（「垂距/夹角不变、`|t|` 变了 ⇒ 是靶动了」），
**而它分不开这两种** —— 见下。

为什么原来的判据不够
====================
2026-10-02 实测：相隔一天的两次 ①（同一台相机、同一张纸）

    垂距   0.6442 → 0.6437 m     （差 0.5 mm，噪声级）
    与法线 35.87° → 35.86°       （差 0.01°）
    |t|    0.8797 → 0.8780 m     （差 1.7 mm）

三项全都「没变」⇒ 按原判据会判成「**什么都没动**」。**但相机实际转了 0.95°。**

原因是那个判据的几何前提不成立：相机**绕靶面法线附近**转时，
「垂距」与「与法线夹角」都**只到二阶才变**（一阶为零），而靶原点几乎
**整个**垂直于转轴（它离相机 0.88 m，离转轴 0.868 m），所以旋转对 `|t|`
的贡献也是一阶为零。三项都是白的 ⇒ **判据静默放行**。

真正分开这两种的，是**纸以外的那些东西动不动**：

    · 纸上和纸外**一起挪同样的量** ⇒ 相机动了（整幅图被同一个单应解释）
    · 只有纸上挪、纸外纹丝不动 ⇒ 靶纸动了

所以本脚本量的是**全幅光流**，不是棋盘格。棋盘格只是其中一个特例。

判据（按顺序）
==============
1. 位移 > 2 px 的点**太少**（不到 2%）⇒ **都没动**，外参还有效。
2. 否则，**只用那些动了的点**解一个单应 H（它们若是同一个刚体，H 就解释得了），
   再把 H **用到那些没动的点上**：
     · 没动的点**也符合 H**（它们本该跟着动，而且确实动了）⇒ **整幅一起动 ⇒ 相机动了**
     · 没动的点**不符合 H**（它们本该动，却纹丝不动）⇒ **局部动 ⇒ 靶纸/物体被碰**
3. 静态点太少、判不了时，退回「**动了的点在画面上铺得开吗**」：
   铺得开（横竖各 ≥60%、至少踩到 3 个象限）⇒ 相机；挤在一处 ⇒ 局部。

⚠️ **不能只看「中位位移」**（第一版就是这么写的，被自检抓出来了）：
   靶纸只占画面 2.4% 的面积，它平移 35 mm 时**全幅中位数仍然是 0**，
   会被判成「都没动」—— 而那正是这个工具存在的理由。所以第 1 条看的是**占比**。
⚠️ **不能只看「内点占比」**：一小块**整体平移**也是单应，它自己的点 100% 内点。
   所以第 2 条必须拿 H 去撞**没动的那些点**，而不是看内点比例。
⚠️ 第 2 条判的是「相机**相对场景**变了」，分不出「相机往左挪」与
   「整个工作台往右挪」—— 而对本仓库来说这**不需要分**：
   两种情况外参都作废，都得重跑 ⓪①。台子不会自己走，所以读作「相机动了」。

分不开的情形（已知，别当成故障）
================================
· 一个**占满整个视场**的刚体整体平移（把静态点全遮住）⇒ 像素级与相机移动
  **不可分**。本场景不适用：靶纸只有 ~150×145 px，画面 1280×720。
· 位移 **< 2 px** 的漂移量不出来（噪声就在这个量级）。2 px ≈ 0.12°；
  0.12° 在 0.9 m 处是 1.9 mm —— 想要更细就得靠 ①（PnP）那条路。

用法
====
    python3 scripts/topcam_drift_check.py \\
        --old /tmp/topcam/frame.png --new /tmp/topcam_now/frame.png \\
        --info /tmp/topcam/info.json \\
        --old-json /tmp/topcam/cam_from_board.json \\
        --new-json /tmp/topcam_now/cam_from_board_now.json

`--info` 与两个 `--*-json` 都是可选的：没有它们就只做像素级的判定；
有 `--info` 会把单应分解成**旋转角**，有那两个 json 会额外做一次
**PnP 对拍**（两条独立路线解同一个「相机转了多少」，差得多就说明哪条坏了）。

退出码：0 = 没检测到变化（外参还有效）；1 = 检测到变化（**外参作废**，
要重跑 ⓪①并重生成编号图）；2 = 环境不满足（图读不到 / 特征点太少 / 尺寸不一致）。
"""

import argparse
import json
import math
import sys

import numpy as np

try:
    import cv2
except ImportError:                                       # pragma: no cover
    print("✗ 需要 cv2（python3-opencv）")
    sys.exit(2)

# ── 判据阈值（都写在名字上，改的时候一眼看得见影响哪个判断）─────────────────
MOVING_PX = 2.0          # 单个点位移超过它才算「动了」（噪声就在 0.2~0.5 px）
STATIC_FRAC_MAX = 0.02   # 动了的点占比低于它 ⇒ 判「都没动」
STAYER_OK_PX = 2.0       # H 用到静态点上，残差低于它才算「H 解释了它们」
GLOBAL_SPAN_MIN = 0.60    # 「铺得开」要求的包围盒占画面比例
GLOBAL_QUADRANTS_MIN = 3  # 「铺得开」至少踩到几个象限
MIN_TRACKED = 30          # 追踪到的点少于它就没法判，退出码 2
MIN_MOVERS = 10           # 动了的点少于它，不足以撑起任何判定
MIN_STAYERS = 8           # 静态点少于它，就用不了「拿 H 去撞静态点」那条判据


def track(a, b):
    """Shi-Tomasi + LK 光流 + 前后向一致性过滤 → `(p0, p1, 位移场)`。

    前后向过滤是必须的：LK 在**白台面**上（本场景里占画面一多半）会给出
    自信但随机的匹配，而这些点恰好能凑出假的「整幅一起动」。
    """
    pts = cv2.goodFeaturesToTrack(a, maxCorners=800, qualityLevel=0.01,
                                  minDistance=12, blockSize=7)
    if pts is None or len(pts) < MIN_TRACKED:
        return None, None, None
    p1, st, _ = cv2.calcOpticalFlowPyrLK(a, b, pts, None,
                                         winSize=(31, 31), maxLevel=4)
    fb, _, _ = cv2.calcOpticalFlowPyrLK(b, a, p1, None,
                                        winSize=(31, 31), maxLevel=4)
    p0 = pts.reshape(-1, 2)
    p1 = p1.reshape(-1, 2)
    fb = fb.reshape(-1, 2)
    ok = (st.ravel() == 1) & (np.linalg.norm(fb - p0, axis=1) < 1.0)
    p0, p1 = p0[ok], p1[ok]
    return p0, p1, p1 - p0


def spans_image(p, shape):
    """内点是不是**铺满**画面（而不是挤在一小块里）。"""
    h, w = shape
    if len(p) < 8:
        return False, "内点太少"
    bw = p[:, 0].max() - p[:, 0].min()
    bh = p[:, 1].max() - p[:, 1].min()
    quad = {(int(x > w / 2), int(y > h / 2)) for x, y in p}
    ok = (bw >= GLOBAL_SPAN_MIN * w and bh >= GLOBAL_SPAN_MIN * h
          and len(quad) >= GLOBAL_QUADRANTS_MIN)
    return ok, "包围盒 %dx%d px（画面 %dx%d），踩到 %d 个象限" % (bw, bh, w, h, len(quad))


def largest_blob(points, shape, cell=32):
    """把外点按粗网格做连通域，返回最大那块（bbox + 点数）。纯 numpy/栈实现。

    网格取 **32 px** 而不是更细：特征点在移动物体上的分布是**稀疏的**
    （真实场景里更稀），16 px 网格会把一整块拆成互不相邻的碎片，
    报出来的框只有真实范围的一小角 —— 那是**会误导人去搬错东西**的。
    实测（自检的 C 节，320×200 的块）：16 px → 报 96×64，
    32 px → 报 352×224，真值 320×200。
    """
    h, w = shape
    gx, gy = -(-w // cell), -(-h // cell)
    grid = np.zeros((gy, gx), bool)
    for x, y in points:
        grid[min(gy - 1, max(0, int(y) // cell)),
             min(gx - 1, max(0, int(x) // cell))] = True
    seen = np.zeros_like(grid)
    best = []
    for j in range(gy):
        for i in range(gx):
            if not grid[j, i] or seen[j, i]:
                continue
            stack, comp = [(j, i)], []
            seen[j, i] = True
            while stack:
                cy, cx = stack.pop()
                comp.append((cy, cx))
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1),
                               (1, 1), (1, -1), (-1, 1), (-1, -1)):
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < gy and 0 <= nx < gx and grid[ny, nx] \
                            and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((ny, nx))
            if len(comp) > len(best):
                best = comp
    if not best:
        return None
    ys = [c[0] for c in best]
    xs = [c[1] for c in best]
    x0, x1 = min(xs) * cell, (max(xs) + 1) * cell
    y0, y1 = min(ys) * cell, (max(ys) + 1) * cell
    pts = [(x, y) for x, y in points
           if x0 <= x < x1 and y0 <= y < y1]
    return {"x0": x0, "x1": x1, "y0": y0, "y1": y1,
            "cells": len(best), "points": len(pts)}


def rotation_of(H, K):
    """把单应拆出「相机转了多少」：`R ≈ K⁻¹HK` 的正交化，附奇异值当体检。

    奇异值三个都 ≈1 ⇒ 与**纯转动**一致（转动与深度无关，所以整幅图
    才会被同一个单应解释）。偏离大 ⇒ 里面有平移，此时把 K⁻¹HK 硬掰成
    旋转矩阵只是**近似**，数只能当参考。
    """
    M = np.linalg.inv(K) @ H @ K
    M = M / M[2, 2]
    U, S, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = -R
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(ax)
    return ang, (ax / n if n > 1e-12 else np.zeros(3)), S


def pnp_angle(pa, pb):
    """两次 PnP 之间相机转了多少（相对同一张靶 ⇒ 就是相机自己的转动）。"""
    Ra, Rb = np.array(pa["R"]), np.array(pb["R"])
    R = Rb @ Ra.T
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(ax)
    return ang, (ax / n if n > 1e-12 else np.zeros(3))


def plane_terms(p):
    """靶面几何三件套（原判据用的量）：垂距 / 与法线夹角 / |t|。"""
    R, t = np.array(p["R"]), np.array(p["t"])
    n = R[:, 2]                                    # 靶面法线在相机系
    return (abs(float(n @ t)),
            math.degrees(math.acos(max(-1.0, min(1.0, abs(float(R[2, 2])))))),
            float(np.linalg.norm(t)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--old", required=True, help="旧帧（外参就是据它算的）")
    ap.add_argument("--new", required=True, help="新帧（刚才拍的）")
    ap.add_argument("--info", help="内参 json（capture 产出的 info.json）")
    ap.add_argument("--old-json", help="旧的 cam_from_board.json")
    ap.add_argument("--new-json", help="新的 cam_from_board.json")
    args = ap.parse_args()

    a = cv2.imread(args.old, cv2.IMREAD_GRAYSCALE)
    b = cv2.imread(args.new, cv2.IMREAD_GRAYSCALE)
    if a is None or b is None:
        print("✗ 读不到图：%s / %s" % (args.old, args.new))
        return 2
    if a.shape != b.shape:
        print("✗ 两帧尺寸不同 %s vs %s —— 不是同一台相机的同一档配置"
              % (a.shape, b.shape))
        return 2
    print("两帧 %dx%d" % (a.shape[1], a.shape[0]))

    p0, p1, d = track(a, b)
    if p0 is None or len(p0) < MIN_TRACKED:
        print("✗ 追踪到的特征点太少（%s）—— 画面太白/太糊就没法判"
              % (0 if p0 is None else len(p0)))
        return 2
    mag = np.linalg.norm(d, axis=1)
    moving = mag > MOVING_PX
    n_m, n_s = int(moving.sum()), int((~moving).sum())
    print("光流：%d 个点（前后向过滤后）  动了 %d 个 / 没动 %d 个  "
          "中位 |d| %.2f px  p90 %.2f px  最大 %.2f px"
          % (len(p0), n_m, n_s, np.median(mag), np.percentile(mag, 90), mag.max()))

    H, stay_explained, verdict, code = None, None, None, None

    if n_m < MIN_MOVERS or n_m < STATIC_FRAC_MAX * len(p0):
        verdict, code = "都没动", 0
        print("\n判定：**动了的点只占 %.1f%%（%d 个）⇒ 两帧之间什么都没动。**\n"
              "      ⇒ 外参还有效，① 不用重跑。"
              % (100.0 * n_m / len(p0), n_m))
    else:
        # 只用「动了的点」解单应 —— 它们若是同一个刚体，一个 H 就够
        H, _ = cv2.findHomography(p0[moving], p1[moving], cv2.RANSAC,
                                  MOVING_PX, maxIters=5000, confidence=0.999)
        span_ok, span_why = spans_image(p0[moving], a.shape)
        print("\n动了的点解出的单应，用到**没动的那些点**上：")
        if n_s >= MIN_STAYERS:
            pr = cv2.perspectiveTransform(p0[~moving].reshape(-1, 1, 2),
                                          H).reshape(-1, 2)
            r = np.linalg.norm(pr - p1[~moving], axis=1)
            stay_explained = float(np.median(r)) < STAYER_OK_PX
            print("  %d 个没动的点，被这个单应解释的残差 中位 %.2f px  p90 %.2f px"
                  % (n_s, np.median(r), np.percentile(r, 90)))
            print("  ⇒ 它们%s这个单应 ⇒ %s"
                  % ("**符合**" if stay_explained else "**不符合**",
                     "整幅一起动" if stay_explained else "只有一部分动"))
        else:
            print("  静态点只有 %d 个（<%d），这条判据用不了，退回「铺得开」检查"
                  % (n_s, MIN_STAYERS))
        print("  动了的点分布：%s ⇒ %s" % (span_why, "铺得开" if span_ok else "**挤在一块**"))

        if stay_explained is False:
            verdict, code = "局部动（靶纸/物体）", 1
        elif stay_explained is True:
            verdict, code = "整幅一起动（相机）", 1
        else:
            verdict, code = ("整幅一起动（相机）" if span_ok
                             else "局部动（靶纸/物体）"), 1

        if code == 1 and verdict.startswith("整幅"):
            print("\n判定：**纸上和纸外一起挪了同一个量 ⇒ 相机动了，靶纸没动。**"
                  "\n      ⇒ ① 作废，标定前必须重跑 ⓪① 并**重生成编号图**。")
        else:
            print("\n判定：**只有一部分挪了 ⇒ 相机没动，是那块东西被碰了。**")
            blob = largest_blob(p0[moving], a.shape)
            if blob:
                sel = ((p0[:, 0] >= blob["x0"]) & (p0[:, 0] < blob["x1"])
                       & (p0[:, 1] >= blob["y0"]) & (p0[:, 1] < blob["y1"])
                       & moving)
                dm = d[sel].mean(0) if sel.any() else np.zeros(2)
                print("      动的那块：x[%d,%d] y[%d,%d]（%d 个格，%d 个点）"
                      % (blob["x0"], blob["x1"], blob["y0"], blob["y1"],
                         blob["cells"], blob["points"]))
                print("      该块平均位移 (%+.1f, %+.1f) px" % (dm[0], dm[1]))
            print("      看上面那个位置是不是靶纸。是靶纸 ⇒ ① 作废、编号图作废。")

    # ── 附加：把单应拆成旋转角（要内参）──────────────────────────────
    if args.info:
        K = np.array(json.load(open(args.info))["K"]).reshape(3, 3)
        if H is not None:
            ang, ax, S = rotation_of(H, K)
            print("\n单应分解（用内参）：旋转 **%.3f°**  轴 %s" % (ang, np.round(ax, 3)))
            print("  K⁻¹HK 奇异值 %s  ⇒ %s"
                  % (np.round(S, 5),
                     "三个都≈1，与**纯转动**一致（整幅图才会被一个单应解释）"
                     if max(abs(S - 1)) < 0.02 else
                     "偏离 1 较多，里面有平移成分，角度只能当参考"))
            print("  参考量级：f·θ = %.1f px（画面里该看到多大的整体位移）"
                  % (K[0, 0] * math.radians(ang)))

    # ── 附加：PnP 对拍 + 原判据那三件套 ──────────────────────────────
    if args.old_json and args.new_json:
        pa, pb = json.load(open(args.old_json)), json.load(open(args.new_json))
        ta, tb = np.array(pa["t"]), np.array(pb["t"])
        ang, ax = pnp_angle(pa, pb)
        print("\nPnP 对拍（只用棋盘格角点，与上面的全幅光流**互相独立**）")
        print("  两次之间：平移差 %.1f mm   旋转 **%.3f°**  轴 %s"
              % (np.linalg.norm(tb - ta) * 1000, ang, np.round(ax, 3)))
        print("  原判据那三件套：")
        for nm, q in (("旧", pa), ("新", pb)):
            dep, ang_n, tt = plane_terms(q)
            print("    %s  垂距 %.4f m | 与法线 %.2f° | |t| %.4f m" % (nm, dep, ang_n, tt))
        da = np.array([plane_terms(pa)[i] - plane_terms(pb)[i] for i in range(3)])
        print("    差  %+.4f m | %+.2f° | %+.4f m" % tuple(da))
        print("  ⚠️ **这三件套全是白的也不能判「没动」** —— 相机绕靶面法线转时，"
              "垂距与夹角只到二阶才变，`|t|` 也几乎不变（靶原点几乎整个垂直于转轴）。"
              "\n     2026-10-02 就是这样：三项全白，而相机实际转了 0.95°。"
              "\n     能判的是上面那条**纸外动不动**。")
        if abs(ang) > 1.0 and code == 0:
            print("  🚨 **矛盾**：PnP 说相机转了 %.2f°，光流说没动 —— 别信任何一条，查。" % ang)
            code = 2

    print("\n判据：%s" % verdict)
    print("退出码 %d = %s" % (code, {0: "外参还有效", 1: "外参作废，重跑 ⓪①",
                                     2: "环境不满足"}[code]))
    return code


if __name__ == "__main__":
    sys.exit(main())

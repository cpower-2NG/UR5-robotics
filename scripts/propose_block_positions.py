#!/usr/bin/env python3
"""从一张俯视图**提案**出台面上各方块的位置 → `blocks.json`，并出一张复核图。

为什么需要它
============
`auto_label_grid.py` 要的是**方块在台面系里的坐标**（然后它把 28.5 mm 的立方体
精确投影成 YOLO 框）。计划 W4 的原话是「方块摆在工作台**已知网格**上，位置实测」。

但实测下来「已知网格」这条路有个不好办的地方：**人手摆的误差本身就有 5~10 mm**
（在 0.9 m 外、fx≈917 时约 5~10 px），而框本身才 32 px —— 摆位误差直接就是标签
误差，跟「标定不准」是同一量级。所以「摆到网格上」并不自动等于「位置已知」。

本脚本换个做法：**颜色只用来找「这里有个东西」，位置由形状解出来，再由形状验回去。**

    1. 高饱和连通域 → 候选。**它只给一个起点，不给标签。**
    2. 以起点为源，在 (x,y,size) 上**最小化倒角距离**：把立方体的投影轮廓
       采样成点，量每个点到「最近的图像边缘」的距离，让整条轮廓滑到方块的实际
       棱边上。这一步**完全不看颜色**（这是它能干活的关键，见下）。
    3. **体检**：位置定住后扫一遍方块尺寸（20~38 mm）。真方块应当明显落在
       28.5 mm 附近；「最像立方体」的尺寸跑到别处去的，多半根本不是立方体。

为什么第 2 步必须由形状来做，而不是直接信颜色掩膜
--------------------------------------------------
颜色掩膜会把方块**旁边的阴影**一起吞进去 —— 实测同一个方块，掩膜给出 50×54 px，
而立方体在那一处只该有 35×45 px。于是「固定位置解尺寸」报出 1.18 倍，一块真方块
被当成异类剔掉。阴影是平滑渐变，Canny 几乎不产生边缘，所以倒角场对它**基本免疫**：
放开位置让它自己找，轮廓会滑回方块本身的棱。低饱和度的方块（土黄/棕）同理受益 ——
倒角一步不看颜色，掩膜抓到多少都无所谓。

⚠️ 为什么不干脆用倒角距离做**全局搜索**（试过，不行）
------------------------------------------------------
倒角距离是个好**精修器**，但是个坏**搜索器**：只要轮廓恰好压在**任意一条**图像
边缘上，分数就是 0。而方块成堆的地方到处是边缘，于是几乎每个错误位置都能拿满分 ——
实测全局扫描产出 81 个「0.00 px」的峰。它必须有一个**大致正确的起点**才有意义，
这就是第 1 步仍然存在的理由。

⚠️ 仍然**只当提案**：复核图必须人工过一遍（计划的要求）。四类实测到的盲区：
   (a) 被画面边缘切掉的方块，解出的位置会朝画面外偏；
   (b) 两个紧贴的方块可能并成一个连通域，进而并成一个提案；
   (c) **低饱和度的方块会被漏掉** —— 白纸上那块紫色的，饱和度不到阈值，
       连候选都不是；
   (d) **急停按钮的红色蘑菇头会假阳性**（圆头在那个距离上确实和立方体轮廓
       重合，实测 0.00 px / 28.5 mm，三道闸门全过）。
   它们都要靠 `--draw` 的复核图来看，脚本不给保证。降低 `--sat-min` 能救 (c)，
   但同时会把阴影并进掩膜 —— 现在靠倒角精修能吸收一部分，但不是全部。

用法
----
    python3 scripts/propose_block_positions.py \\
        --image frame.png --info info.json \\
        --extrinsics labeler_extrinsics.json \\
        --out blocks.json --draw check.png --block-mm 28.5

退出码：0 = 有提案（哪怕部分被剔）；2 = 一个候选都没有。
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import auto_label_grid as L                                            # noqa: E402

CANNY_LO, CANNY_HI = 40, 110


# ---------------------------------------------------------------- 几何（正向）

def _corner_offsets(size):
    """立方体 8 个角点相对**中心**的偏移，沿用 labeler 自己的 `box_corners` 约定。

    直接调 `L.box_corners(0,0,0,size)` 取偏移，而不是在这里另写一套 ——
    投影约定必须与 `auto_label_grid.py` 逐字一致，否则提案和标注会差一个镜像。
    """
    return np.asarray(L.box_corners(0.0, 0.0, 0.0, size), float)


def project_cubes(proj, xy, zc, offsets):
    """(N,2) 台面中心 → ``(uv, ok)``，``uv`` 是 (N,8,2) 像素角点。

    ``ok`` 为 False 表示该位置有点落在相机背后（投影无意义）。
    """
    N = xy.shape[0]
    P = np.empty((N, offsets.shape[0], 3))
    P[:, :, 0] = xy[:, 0:1] + offsets[None, :, 0]
    P[:, :, 1] = xy[:, 1:2] + offsets[None, :, 1]
    P[:, :, 2] = zc + offsets[None, :, 2]
    # 靶系 → 相机系：to_camera(p) = Rᵀ(p − t)；堆叠后即 (P − t) @ R
    C = (P - proj.t[None, None, :]) @ proj.R
    Z = C[:, :, 2]
    ok = np.all(Z > 1e-6, axis=1)
    Zs = np.where(Z > 1e-6, Z, 1.0)
    u = proj.fx * (C[:, :, 0] / Zs) + proj.cx
    v = proj.fy * (C[:, :, 1] / Zs) + proj.cy
    return np.stack([u, v], axis=2), ok


def pixel_bbox(proj, center, size):
    """立方体 8 角点 → 像素外接矩形 ``(u0, v0, u1, v1)``；有点在背后则 None。

    不走 `L.bbox_from_pixels`：那个返回**归一化**坐标并且会**裁剪到画面内**，
    而复核图要画的是未经裁剪的原始框 —— 被画面切掉的方块靠「原始框超出
    画面」这一点才能被认出来。
    """
    uv, ok = project_cubes(proj, np.array([center[:2]]), center[2],
                           _corner_offsets(size))
    if not ok[0]:
        return None
    p = uv[0]
    return (float(p[:, 0].min()), float(p[:, 1].min()),
            float(p[:, 0].max()), float(p[:, 1].max()))


def _rough_seed(proj, obs, zc):
    """观测框中心 → 台面上的粗起点：反投一条射线，取与 z=zc 平面的交点。

    只用它当**起点**，精度无所谓；真值由 `refine_chamfer` 解出来。
    """
    u = (obs[0] + obs[2]) / 2.0
    v = (obs[1] + obs[3]) / 2.0
    d_cam = np.array([(u - proj.cx) / proj.fx, (v - proj.cy) / proj.fy, 1.0])
    # ⚠️ 正向是 p_target = R·p_cam + t（见 TfProjector.__init__），所以
    #    「相机系方向 → 靶系方向」要用 **R**，不是 R.T。写成 R.T 不会报错 ——
    #    它仍然给出一根像样的射线，只是每次都偏离真值 ~80 mm（实测），
    #    在旧版里被后面的最小二乘顺手救回来了，于是这个错一直没露头。
    d = proj.R @ d_cam
    if abs(d[2]) < 1e-9:
        return np.array([0.0, 0.0])
    lam = (zc - proj.t[2]) / d[2]
    return (proj.t + lam * d)[:2]


# ---------------------------------------------------------------- 形状（倒角）

def chamfer_field(img):
    """图像边缘 → 每个像素到最近边缘的欧氏距离（px）。"""
    import cv2

    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.dilate(cv2.Canny(g, CANNY_LO, CANNY_HI), np.ones((3, 3), np.uint8))
    return cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)


def _densify(poly, step):
    """多边形边界按弧长均匀取点（向量化）。"""
    a = poly
    b = np.roll(poly, -1, axis=0)
    seg = np.linalg.norm(b - a, axis=1)
    n = np.maximum(2, (seg / step).astype(int))
    idx = np.repeat(np.arange(len(a)), n)
    starts = np.repeat(np.cumsum(n) - n, n)
    t = (np.arange(int(n.sum())) - starts) / np.repeat(n, n)
    return a[idx] + (b[idx] - a[idx]) * t[:, None]


def chamfer_score(dt, uv8, W, H, step=1.5, pct=75.0):
    """一个位置的形状分：投影轮廓上采样点到最近图像边缘的距离的 **p75**。

    为什么用 p75 而不是最大值：轮廓总有几处被遮挡或被画面切掉，用最大值会让
    好方块因为一个角就出局。p75 要求「**大部分**轮廓贴着边」，同时容许一两处例外。
    实测分档：真方块 0.0~2.3 px ｜ 空桌面 7.6~59 px ｜ 绿罐盖 4.8 px ｜
    急停按钮的**黄色底座** 11.0 px。⚠️ 但急停按钮的**红色蘑菇头**单独成一个
    连通域，实测拿到 0.00 px / 28.5 mm —— 它是本脚本唯一已知的假阳性，
    圆头在那个距离上确实和立方体轮廓重合。见文件头的「已知盲区 (c)」。
    """
    import cv2

    hull = cv2.convexHull(np.ascontiguousarray(uv8, dtype=np.float32)).reshape(-1, 2)
    if len(hull) < 3:
        return None
    pts = _densify(hull, step)
    u = np.clip(pts[:, 0], 0, W - 1).astype(np.int32)
    v = np.clip(pts[:, 1], 0, H - 1).astype(np.int32)
    return float(np.percentile(dt[v, u], pct))


def score_at(proj, dt, x, y, zc, size, W, H, offsets=None):
    """在给定台面位置，算该立方体投影轮廓的倒角 p75。"""
    uv, ok = project_cubes(proj, np.array([[x, y]]), zc,
                           _corner_offsets(size) if offsets is None else offsets)
    if not ok[0]:
        return None
    return chamfer_score(dt, uv[0], W, H)


def refine_chamfer(proj, dt, W, H, x0, y0, size0, zsign,
                   span_m=0.012, step_m=0.002,
                   size_lo_mm=20.0, size_hi_mm=38.0, size_step_mm=0.5,
                   fine_span_m=0.003, fine_step_m=0.0005):
    """以 (x0,y0,size0) 为起点，在 (x,y) 上**最小化倒角 p75**，并顺带体检尺寸。

    为什么需要它：颜色掩膜会把方块**旁边的阴影**一起吞进去，于是观测框偏大
    （实测 50×54 vs 应有的 35×45）。阴影是平滑渐变，Canny 几乎不产生边缘，
    所以倒角场对它**基本免疫** —— 放开位置让它自己找，轮廓会滑回方块本身的棱。

    顺序是 (x,y) 粗修 → 尺寸体检 → (x,y) 精修，而不是一次三维搜索：倒角对
    「位置」远比「尺寸」敏感，先定位置再定尺寸再回收位置，两三步就收敛，
    而且避免了在错误的尺寸上把位置也带偏。

    最后一步精修**回到名义尺寸 size0**，这样交付的坐标与标注器将要投影的那个
    立方体是同一套几何；体检出的 best_size 只用于判断。

    返回 ``(x, y, best_size_m, score)``；``zsign`` 是靶系 z 指向（±1），
    因为方块中心 z = zsign·size/2 随尺寸一起变。
    """
    def s_of(x, y, s):
        return score_at(proj, dt, x, y, zsign * s / 2.0, s, W, H)

    def sweep(x, y, s, span, step):
        best = (s_of(x, y, s), x, y)
        if best[0] is None:
            return None
        n = int(span / step)
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                if i == 0 and j == 0:
                    continue
                v = s_of(x + i * step, y + j * step, s)
                if v is not None and v < best[0]:
                    best = (v, x + i * step, y + j * step)
        return best

    cur = sweep(x0, y0, size0, span_m, step_m)
    if cur is None:
        return None
    _, x, y = cur

    # 位置定住后扫尺寸 —— **纯体检**：形状不对的东西（圆盖子）会在这里
    # 报出一个明显不是 28.5 的「最像立方体」尺寸。
    best_size, best_v = size0, None
    for mm in np.arange(size_lo_mm, size_hi_mm + 1e-9, size_step_mm):
        v = s_of(x, y, mm / 1000.0)
        if v is not None and (best_v is None or v < best_v):
            best_v, best_size = v, mm / 1000.0

    cur = sweep(x, y, size0, fine_span_m, fine_step_m)
    if cur is None:
        return None
    v, x, y = cur
    return float(x), float(y), float(best_size), float(v)


# ---------------------------------------------------------------- 候选

def candidates(img, sat_min, val_min, area_min, area_max, aspect_lo, aspect_hi):
    """高饱和连通域 → 候选像素外接矩形。**只给起点，不给标签。**"""
    import cv2

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = ((hsv[:, :, 1] > sat_min) & (hsv[:, :, 2] > val_min)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(m)
    out = []
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if not (area_min <= a <= area_max and w <= 140 and h <= 140):
            continue
        if not (aspect_lo <= w / float(h) <= aspect_hi):
            continue
        out.append((float(x), float(y), float(x + w), float(y + h), int(a)))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='从俯视图提案方块位置（颜色取种子 + 倒角精修 + 尺寸体检）',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', required=True)
    ap.add_argument('--info', required=True)
    ap.add_argument('--extrinsics', required=True, help='④ labeler 的产物')
    ap.add_argument('--out', default='blocks.json')
    ap.add_argument('--draw', default=None, help='复核图输出路径')
    ap.add_argument('--block-mm', type=float, default=28.5)
    ap.add_argument('--sat-min', type=int, nargs='+', default=[110, 50],
                    help='可给多个：**并集**取候选。不同阈值抓到不同的方块 —— '
                         '实测高饱和的橙/黄在 110 稳，米色的 S≈108 要 100 以下，'
                         '阴影里的紫方块 S≈66 更低。并集之后由倒角/尺寸闸门统一筛。')
    ap.add_argument('--val-min', type=int, default=80)
    ap.add_argument('--area-min', type=int, default=450)
    ap.add_argument('--area-max', type=int, default=6500)
    ap.add_argument('--aspect-lo', type=float, default=0.55)
    ap.add_argument('--aspect-hi', type=float, default=1.8)
    ap.add_argument('--max-chamfer-px', type=float, default=3.0,
                    help='倒角 p75 超过它就剔除（空桌面 7.6 px 起，已留一倍余量）')
    ap.add_argument('--max-size-dev-mm', type=float, default=4.0,
                    help='体检出的方块尺寸偏离 --block-mm 超过它就剔除')
    ap.add_argument('--nms-mm', type=float, default=0.020,
                    help='两个提案靠得比这近就只留分高的那个（台面米）')
    a = ap.parse_args(argv)

    import cv2

    img = cv2.imread(a.image)
    if img is None:
        raise SystemExit('读不到图：%s' % a.image)
    with open(a.info) as fh:
        info = json.load(fh)
    with open(a.extrinsics) as fh:
        extr = json.load(fh)
    for k in ('fx', 'fy', 'cx', 'cy'):
        if k not in info:
            raise SystemExit('内参文件里没有 %r —— ②capture 现在两种形状都写；'
                             '这份多半是旧的，重抓一次' % k)
    proj = L.TfProjector(info, extr)

    H, W = img.shape[:2]
    size = a.block_mm / 1000.0
    # 靶系 z 指向哪边是不定的（见 ④ labeler 的打印）。方块摆在台面上，
    # 中心在「背离相机」的那一侧。
    zsign = 1.0 if proj.t[2] > 0 else -1.0
    zc = zsign * size / 2.0

    print('靶系 %s → 像素  %dx%d' % (extr.get('target_frame'), W, H))
    print('相机光心在靶系 = %s  ⇒ 方块中心 z = %+.5f'
          % (np.round(proj.t, 4).tolist(), zc))

    dt = chamfer_field(img)

    # 多个饱和度阈值取**并集**。为什么不是挑一个最好的：方块的颜色跨度比
    # 台面杂物大得多（实测同一台上 S 从 27 到 217），任何一个全局阈值都会
    # 漏掉一端。低阈值会带来多余的候选和**粘连**，但那些由后面的倒角/尺寸
    # 闸门和最后那道非极大抑制统一收掉 —— 让每一层干它擅长的事。
    cands, seen, per = [], [], []
    for s in a.sat_min:
        got = candidates(img, s, a.val_min, a.area_min, a.area_max,
                         a.aspect_lo, a.aspect_hi)
        per.append((s, len(got)))
        for c in got:
            cx, cy = (c[0] + c[2]) / 2.0, (c[1] + c[3]) / 2.0
            if any((cx - k[0]) ** 2 + (cy - k[1]) ** 2 < 6.0 ** 2 for k in seen):
                continue
            seen.append((cx, cy))
            cands.append(c)
    print('\n候选连通域 %d 个（亮度 >%d、面积 %d~%d、长宽比 %.2f~%.2f）'
          % (len(cands), a.val_min, a.area_min, a.area_max,
             a.aspect_lo, a.aspect_hi))
    print('  饱和度并集：%s（去重前）'
          % '  '.join('>%d: %d 个' % (s, n) for s, n in per))

    kept, dropped = [], []
    for (u0, v0, u1, v1, area) in cands:
        seed = _rough_seed(proj, np.array([u0, v0, u1, v1]), zc)
        res = refine_chamfer(proj, dt, W, H, seed[0], seed[1], size, zsign)
        if res is None:
            dropped.append(dict(u0=u0, v0=v0, u1=u1, v1=v1, area=area,
                                x=float(seed[0]), y=float(seed[1]),
                                chamfer_px=None, size_mm=None,
                                why='精修时方块落到相机背后了'))
            continue
        x, y, best_size, ch = res
        rec = dict(u0=u0, v0=v0, u1=u1, v1=v1, area=area, x=x, y=y,
                   chamfer_px=ch, size_mm=best_size * 1000.0)
        dev = abs(best_size * 1000.0 - a.block_mm)
        if ch > a.max_chamfer_px:
            rec['why'] = '倒角 p75 %.2f px > %.1f' % (ch, a.max_chamfer_px)
        elif dev > a.max_size_dev_mm:
            rec['why'] = '体检尺寸 %.1f mm，偏离 %.1f mm' % (rec['size_mm'], dev)
        elif not (-0.95 < x < 0.95 and -0.95 < y < 0.95):
            rec['why'] = '解到靶系外 (%.2f, %.2f) m' % (x, y)
        else:
            kept.append(rec)
            continue
        dropped.append(rec)

    # 非极大抑制：靠近的提案只留分高的。**放在最后**，因为倒角精修会把同一个
    # 方块附近的若干候选都拉到同一个点上 —— 不抑制就得到一串重复框。
    kept.sort(key=lambda r: r['chamfer_px'])
    dedup = []
    for r in kept:
        if all((r['x'] - q['x']) ** 2 + (r['y'] - q['y']) ** 2 > a.nms_mm ** 2
               for q in dedup):
            dedup.append(r)
    if len(dedup) != len(kept):
        print('（%d 个提案被非极大抑制合并）' % (len(kept) - len(dedup)))
    kept = dedup

    print('\n✅ 采纳 %d 个：' % len(kept))
    for r in sorted(kept, key=lambda d: -d['area']):
        print('   (%+.3f, %+.3f)  掩膜框 %3.0fx%3.0f  倒角 %.2f px  体检尺寸 %.1f mm'
              % (r['x'], r['y'], r['u1'] - r['u0'], r['v1'] - r['v0'],
                 r['chamfer_px'], r['size_mm']))
    if dropped:
        print('\n❌ 剔除 %d 个：' % len(dropped))
        for r in dropped:
            print('   (%+.3f, %+.3f)  掩膜框 %3.0fx%3.0f  —— %s'
                  % (r['x'], r['y'], r['u1'] - r['u0'], r['v1'] - r['v0'], r['why']))

    blocks = [{'x': round(r['x'], 5), 'y': round(r['y'], 5),
               'z': round(zc, 5), 'size': size} for r in kept]
    with open(a.out, 'w') as fh:
        json.dump(blocks, fh, indent=2)
    print('\n→ %s（%d 块）' % (a.out, len(blocks)))

    if a.draw:
        out = img.copy()
        for r in kept:
            bb = pixel_bbox(proj, (r['x'], r['y'], zc), size)
            if bb:
                cv2.rectangle(out, (int(bb[0]), int(bb[1])),
                              (int(bb[2]), int(bb[3])), (0, 255, 0), 2)
                cv2.putText(out, '%.1f/%.1f' % (r['chamfer_px'], r['size_mm']),
                            (int(bb[0]), int(bb[1]) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)
        for r in dropped:
            bb = pixel_bbox(proj, (r['x'], r['y'], zc), size)
            if bb:
                cv2.rectangle(out, (int(bb[0]), int(bb[1])),
                              (int(bb[2]), int(bb[3])), (0, 0, 255), 2)
        cv2.imwrite(a.draw, out)
        print('→ %s  （绿=采纳，红=剔除；框上标的是「倒角 px / 体检尺寸 mm」，'
              '框应套住方块）' % a.draw)
    return 0 if blocks else 2


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""SAM 感知层：``find_blocks_sam`` —— ``graspnet_candidates.find_blocks`` 的
SAM 替身（同 schema 行输出，台账第三百四十五轮立项、调研 §3.2 设计注记）。

分层（每层坏了有兜底）：
  几何簇（``gc.find_blocks(split=False, return_masks=True)``）负责「哪里可能有
  块」——它给提示点与簇顶面像素集；SAM 掩码负责「实例归属」（把簇顶面像素按
  掩码切分）；深度负责「顶面」（簇的 3 mm 带，不变）；色相只负责「身份」
  （掩码内饱和度加权投票）。

与网格链的三点不同：
  1. 拆分不再依赖色相投票（red/orange、orange/tan 与 S2 同色对同一路径）；
  2. 提示 = base 系长轴端点经相机投影（第 346 轮实验；像素系直接算会偏 ~90°）；
  3. 钳制与色相无关（两个真实块的中心距不可能 <0.95×块边 ⇒ 带内即贴合对）。

失败模式与兜底：
  · SAM 没盖住簇（无掩码/覆盖 <70%）⇒ 该簇回退网格行（记 ``sam_fallback``）；
  · 单片但票面混合 ⇒ ``mixed_pair_unsplit``（build_truth 丢弃，同网格链 ④）；
  · 块数与布局表不符 ⇒ 编排器既有 fail-closed（6≠6 停轮）。

权重不入库：缺省 ``~/robotics/third_party/weights/mobile_sam.pt``（``SAM_WEIGHTS``
可覆盖）；预测器可注入（测试用假预测器，不依赖 torch/ultralytics）。
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import graspnet_candidates as gc                      # noqa: E402
import block_classes as bc                            # noqa: E402

WEIGHTS_DEFAULT = os.path.expanduser(
    "~/robotics/third_party/weights/mobile_sam.pt")

#: 单片最小顶面像素（真实顶面实测 ≥600；镜面/边界碎片 378–498，第 345 轮）
PIECE_MIN_PX = 300
#: 重叠去重阈值（顶面像素 IoU）
PIECE_IOU_MAX = 0.3
#: 身份份额下限（掩码内色相投票）
PIECE_SHARE_MIN = 0.6
#: SAM 覆盖率下限（低于则该簇回退网格行）
COVER_MIN = 0.7
#: 贴合对带（块边比例；与 stage4_layouts 的贴合带同族口径）
PAIR_LO, PAIR_HI = 0.55, 0.95


# ---------------------------------------------------------------- 模型加载
class UltralyticsSam:
    """ultralytics SAM 预测器包装：``masks_for(points) -> list[np.bool HxW]``。

    ultralytics 对同一张图缓存嵌入（热调用 ~0.09 s，第 346 轮实测），逐簇调用
    不重复编码。
    """

    def __init__(self, weights=None):
        from ultralytics import SAM
        self._m = SAM(weights or WEIGHTS_DEFAULT)
        self._image_path = None

    def set_image_path(self, image_path):
        self._image_path = image_path

    def masks_for(self, points, image_path=None):
        path = image_path or self._image_path
        r = self._m(path, points=[list(p) for p in points])[0]
        if r.masks is None:
            return []
        return (r.masks.data > 0.5).cpu().numpy()


def load_predictor(weights=None):
    return UltralyticsSam(weights)


# ---------------------------------------------------------------- 纯逻辑
def base_to_uv(bx, by, top_m, cam):
    """base 点（m）→ 像素（与 find_blocks 的 ``uv`` 同一投影）。"""
    K, R, t = cam["K"], cam["R_bc"], cam["t_bc"]
    uv = K @ (R.T @ (np.array([bx, by, top_m]) - t))
    return [round(float(uv[0] / uv[2]), 1), round(float(uv[1] / uv[2]), 1)]


def discover_prompts(row, cam):
    """base 系长轴端点（±长边/4，块顶高度）→ 相机投影 → 两个提示点。"""
    cx, cy = row["cx"] / 1000.0, row["cy"] / 1000.0
    top = float(row.get("top", row.get("h", 29.5))) / 1000.0
    long_mm = float(row.get("ext_rect", [29.5, 29.5])[0])
    yaw = math.radians(float(row.get("yaw_rect", 0.0)))
    ax, ay = math.cos(yaw), math.sin(yaw)
    half = long_mm / 4.0 / 1000.0
    pts = []
    for sign in (1.0, -1.0):
        p = np.array([cx + sign * ax * half, cy + sign * ay * half, top])
        uv = cam["K"] @ (cam["R_bc"].T @ (p - cam["t_bc"]))
        pts.append((float(uv[0] / uv[2]), float(uv[1] / uv[2])))
    return pts


def piece_stats(idx, Pb, pix, color):
    """一片顶面像素（``Pb`` 下标）→ 身份/票 + (质心 base xy, PCA yaw, ext, rect)。

    身份 = 掩码内色相投票（饱和度加权），**外加 orange/tan 二义的 2D 消除**
    （``bc.disambiguate_orange_tan``：低饱和米色的 tan 在光照/白平衡变化下
    色相漂进 orange 带——2026-11-17 s1-3-att2 实证，纯色相投票无解；饱和度
    中位反而把两类分得更开）。返回 ``cls``/``share`` 已按二义结果修正。
    """
    F = Pb[idx]
    c = F[:, :2].mean(0)
    d = F[:, :2] - c
    _, evecs = np.linalg.eigh(d.T @ d / max(1, len(d)))
    yaw = float(np.degrees(np.arctan2(evecs[1, -1], evecs[0, -1])))
    ext = (d @ evecs).max(0) - (d @ evecs).min(0)
    rect = gc.min_area_rect_2d(F[:, :2])
    px = color[pix[idx, 1], pix[idx, 0]].astype(float)
    votes, wsum = {}, 0.0
    hue_samples, sat_samples = [], []
    for p in px[::max(1, len(px) // 400)]:
        h, s, v = gc.rgb_to_hsv255(p)
        if v < 40:
            continue
        hue_samples.append(h)
        sat_samples.append(s)
        if s < 60:
            continue
        name, _ = bc.nearest_class_by_hue(h)
        votes[name] = votes.get(name, 0.0) + s
        wsum += s
    hue_med = float(np.median(hue_samples)) if hue_samples else None
    rel_sat_med = float(np.median(sat_samples)) if sat_samples else None
    cls = max(votes, key=votes.get) if wsum > 0 else "unknown"
    if wsum > 0 and cls in ("orange", "tan"):
        fixed = bc.disambiguate_orange_tan(hue_med, rel_sat_med)
        if fixed is not None and fixed != cls:
            # 票面质量随二义结果重新归属（该片的饱和像素本属 tan）
            votes[fixed] = votes.get(fixed, 0.0) + votes.pop(cls)
            cls = fixed
    share = (votes.get(cls, 0.0) / wsum) if wsum > 0 else 0.0
    return {"c": c, "yaw": yaw, "ext": ext, "rect": rect,
            "votes": votes, "wsum": wsum, "px": px,
            "cls": cls, "share": round(share, 3),
            "hue_med": None if hue_med is None else round(hue_med, 1),
            "rel_sat_med": None if rel_sat_med is None
            else round(rel_sat_med, 1)}


def mixed_votes(votes, wsum):
    """票面双类都 ≥25%（同网格链 ④ 的口径）⇒ 未拆混合。"""
    top2 = sorted(votes.values(), reverse=True)[:2]
    return len(top2) == 2 and wsum > 0 and top2[1] / wsum >= 0.25


def clamp_pairs(rows, cam, block_size=0.0295):
    """贴合对钳制（与色相无关；至多一对取最小距，钳前快照上判）。

    ``cx/cy`` 沿连线对称推开到块边；``uv`` 重投影保持一致——build_truth 的
    射线∩平面以 uv 为准，不同步会把钳制丢掉。
    """
    best = None
    live = [r for r in rows if not r.get("low")]
    for i in range(len(live)):
        for j in range(i + 1, len(live)):
            a, b = live[i], live[j]
            d = math.hypot(a["cx"] - b["cx"], a["cy"] - b["cy"])
            lo = PAIR_LO * block_size * 1000
            hi = PAIR_HI * block_size * 1000
            if lo <= d <= hi and (best is None or d < best[0]):
                best = (d, a, b)
    if best is None:
        return None
    d, a, b = best
    ux = (b["cx"] - a["cx"]) / d
    uy = (b["cy"] - a["cy"]) / d
    push = (block_size * 1000.0 - d) / 2.0
    for r, sign in ((a, -1.0), (b, 1.0)):
        r["cx"] = round(r["cx"] + sign * ux * push, 4)
        r["cy"] = round(r["cy"] + sign * uy * push, 4)
        r["uv"] = base_to_uv(r["cx"] / 1000.0, r["cy"] / 1000.0,
                             float(r["top"]) / 1000.0, cam)
        r["pair_clamp_from_mm"] = round(d, 2)
    return d


def _iou_idx(ia, ib):
    sa, sb = set(ia.tolist()), set(ib.tolist())
    inter = len(sa & sb)
    return inter / max(1, len(sa | sb))


# ---------------------------------------------------------------- 主入口
def find_blocks_sam(frame, predictor, hue_table=None, return_masks=False,
                    max_h_mm=70.0):
    """SAM 版 ``find_blocks``：同 schema 行输出（``return_masks`` 带掩码像素）。"""
    depth, color, K = frame["depth"], frame["color"], frame["K"]
    H, W = depth.shape
    grid_rows, _ = gc.find_blocks(frame, split=False, return_masks=True,
                                  hue_table=hue_table,
                                  max_h_mm=max_h_mm)

    P, pix = gc.backproject(depth, K)
    Pb = P @ frame["R_bc"].T + frame["t_bc"]
    table_z = gc.table_plane_z(Pb)
    pix_map = np.full((H, W), -1, dtype=np.int64)
    pix_map[pix[:, 1], pix[:, 0]] = np.arange(len(pix))
    image_path = frame.get("image_path") or _frame_image(frame)

    out = []
    for row in grid_rows:
        if row.get("low"):
            out.append(row)
            continue
        muv = row.get("mask_uv")
        if muv is None or len(muv) < 2 * PIECE_MIN_PX:
            row["sam_fallback"] = "no_mask_uv_or_too_small"
            out.append(row)
            continue

        prompts = discover_prompts(row, frame)
        masks = predictor.masks_for(prompts, image_path=image_path)
        muv = row["mask_uv"]
        if masks is None or len(masks) == 0:
            row["sam_fallback"] = "no_masks"
            out.append(row)
            continue

        # ── 掩码层级处理（SAM 常给「部分-整体」层级：小掩码贴单块、大掩码
        #    盖整簇）。按面积**升序**认领：小掩码先认（贴块、精确），大掩码
        #    只认剩余像素——剩余恰好是「被小掩码盖住的其它块」。每个最终片段
        #    在最终像素集上重算质心/身份（大掩码扣除后剩余会变纯）。
        order = sorted(masks, key=lambda mk: int(mk[muv[:, 1],
                                                   muv[:, 0]].sum()))
        claimed = np.zeros(len(muv), dtype=bool)
        finals = []
        for mk in order:
            owner = mk[muv[:, 1], muv[:, 0]]
            un = owner & ~claimed
            if un.sum() < PIECE_MIN_PX:
                continue                     # 全在已认领区 ⇒ 同块重复
            claimed |= un
            piece_px = muv[un]
            idx = pix_map[piece_px[:, 1], piece_px[:, 0]]
            idx = idx[idx >= 0]
            if len(idx) < PIECE_MIN_PX:
                continue
            st = piece_stats(idx, Pb, pix, color)
            finals.append(dict(st, idx=idx, px=piece_px, n=int(len(idx))))

        # ── 同块分裂合并：中心距 <0.55×块边的片段并入较大者（像素并集后
        #    重投票）。真块中心距不可能小于它。
        merged = []
        for pc in sorted(finals, key=lambda p: p["n"]):
            hit = None
            for q in merged:
                if math.hypot(pc["c"][0] - q["c"][0],
                              pc["c"][1] - q["c"][1]) < 0.55 * 0.0295:
                    hit = q
                    break
            if hit is None:
                merged.append(pc)
                continue
            union = np.concatenate([hit["idx"], pc["idx"]])
            rest = piece_stats(union, Pb, pix, color)
            hit.update({k: rest[k] for k in
                        ("c", "yaw", "ext", "rect", "votes", "wsum", "px",
                         "cls", "share", "hue_med", "rel_sat_med")})
            hit["n"] = int(len(union))

        picked = []
        for pc in merged:
            cls = pc.get("cls", "unknown")
            share = float(pc.get("share", 0.0))
            if cls == "unknown" or share < PIECE_SHARE_MIN:
                continue                     # 剩余像素身份不纯 ⇒ 丢（可见）
            picked.append(dict(pc, cls=cls, share=share))

        covered = claimed.sum()
        if not picked or covered < COVER_MIN * len(muv):
            row["sam_fallback"] = "cover=%.2f pieces=%d" % (
                covered / max(1, len(muv)), len(picked))
            out.append(row)
            continue

        for pi, pc in enumerate(picked):
            wsum = pc["wsum"]
            cls = pc["cls"]
            share = pc["share"]
            med_rgb = np.median(color[pix[pc["idx"], 1], pix[pc["idx"], 0]],
                                axis=0)
            rec = {
                "cx": round(float(pc["c"][0]) * 1000, 4),
                "cy": round(float(pc["c"][1]) * 1000, 4),
                "top": row["top"], "h": row["h"],
                "uv": base_to_uv(pc["c"][0], pc["c"][1],
                                 float(row["top"]) / 1000.0, frame),
                "ext": [round(float(v), 1) for v in pc["ext"]],
                "yaw": round(float(pc["yaw"]), 2), "n": pc["n"],
                "yaw_rect": round(float(pc["rect"]["yaw_deg"]), 2),
                "ext_rect": [round(float(v), 1)
                             for v in pc["rect"]["ext_mm"]],
                "rect_fill": round(float(pc["rect"]["fill"]), 3),
                "yaw_trusted": bool(float(pc["rect"]["fill"]) >= 0.85),
                "rgb": [round(float(v)) for v in med_rgb],
                "hsv": [round(float(v), 1) for v in gc.rgb_to_hsv255(med_rgb)],
                "px_area_mm2": row.get("px_area_mm2"),
                "class": cls, "class_share": round(share, 3),
                "votes": {k: int(v) for k, v in pc["votes"].items()},
                "cells": pc["n"],
                "low": False,
                "split_n": len(picked), "split_i": pi,
                "split_src": "sam",
                "parent_ext_rect": list(row.get("ext_rect", [0.0, 0.0])),
                "pair_clamp_from_mm": None,
                "mixed_pair_unsplit": (len(picked) == 1
                                       and mixed_votes(pc["votes"],
                                                       pc["wsum"])),
            }
            if return_masks:
                rec["mask_uv"] = pc["px"].astype(np.int32)
            out.append(rec)

    clamp_pairs(out, frame)
    out.sort(key=lambda r: (r["cx"], r["cy"]))
    for i, r in enumerate(out):
        r["idx"] = i
    return out, table_z


def _frame_image(frame):
    d = frame.get("dir")
    return os.path.join(d, "color_000.png") if d else None

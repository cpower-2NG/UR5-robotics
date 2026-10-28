#!/usr/bin/env python3
"""阶段 2 的**颜色判定双源**：顶面 HSV 投票 + 七类 YOLO。

为什么不是「YOLO 说了算」
────────────────────────
2026-10-27 在 A3 存档腕部帧上实测（`block_best_7cls.pt`，640/960/1280 ×
conf 0.1/0.25/0.5）：

  · 只有 **imgsz=640** 能检出；960/1280 直接 0 个框（模型是 640 训的，
    放大反而失效）。
  · 640 下每帧只检出 **4 个**：红 / 橙 / 红 / 青；紫、黄、棕、黄绿**一个都没有**。
  · 检出的 4 个与顶面 HSV 判定**全部一致**（红/橙/红/青），没有误标。

所以模型不是「全色分类器」，它只对红/橙/青有召回；其余颜色必须靠 HSV 顶面投票。
把 YOLO 当唯一来源会让紫/黄块全部变成 unknown，进而整轮跳块。

判定策略（fail-closed）
──────────────────────
· 两源都有且一致          → ``class``，``source=both``
· 只有 HSV（多数类）        → ``class``，``source=hsv``
· 只有 YOLO（HSV 低票/unknown）→ ``class``，``source=yolo``
· 两源**冲突**             → ``class=None``，``status=conflict``（默认不猜）
· 两源都没有               → ``class=None``，``status=unknown``

冲突默认不猜，是因为「入错格」比「跳一块」更难在判分里解释；主链拿到 None
应当跳过该块并记一条原因，而不是按某个来源硬判。需要临时放宽时显式传
``conflict_policy="hsv"`` 或 ``"yolo"``，并在报告里注明。

本模块只做**纯判定**，不读图、不加载模型；检测/投票的输入来自
``graspnet_candidates.find_blocks`` 的块记录与 ``precision_verify.yolo_detect``
的检测列表。这样测试不需要 ultralytics。
"""

import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import block_classes                                              # noqa: E402

DEFAULT_CLASSES = list(block_classes.DEFAULT_CLASSES)
HSV_MIN_SHARE = 0.60
YOLO_MIN_CONF = 0.35
YOLO_MATCH_PX = 60.0
CONFLICT_POLICIES = ("unknown", "hsv", "yolo")


def _dist_px(a, b):
    return float(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5)


def match_yolo(block, detections, max_uv_px=YOLO_MATCH_PX):
    """给一个块找最近的 YOLO 检测框（按块顶面中心的投影像素）。

    返回 ``(det, dist_px)``；没有在阈值内的返回 ``(None, None)``。
    一个检测框**可以被多块匹配** —— 这里不做一对一分配，因为调用方通常在
    单帧上逐块判色；若需要一对一，见 ``assign_yolo``。
    """
    if not detections or 'uv' not in block:
        return None, None
    best, best_d = None, None
    for d in detections:
        if 'uv' not in d:
            continue
        dd = _dist_px(block['uv'], d['uv'])
        if best_d is None or dd < best_d:
            best, best_d = d, dd
    if best_d is not None and best_d <= max_uv_px:
        return best, best_d
    return None, None


def assign_yolo(blocks, detections, max_uv_px=YOLO_MATCH_PX):
    """一对一分配：每个检测框只给最近的块，块按距离升序认领。

    为什么需要：同一帧里两个块可能挨得近，一个检测框的中心落在两者之间；
    如果逐块都取「最近检测」，同一个框会被两块共用，等于把一块的颜色抄给另一块。
    """
    pairs = []
    for bi, b in enumerate(blocks):
        for di, d in enumerate(detections):
            if 'uv' not in b or 'uv' not in d:
                continue
            pairs.append((_dist_px(b['uv'], d['uv']), bi, di))
    pairs.sort()
    used_b, used_d, out = set(), set(), {}
    for dist, bi, di in pairs:
        if dist > max_uv_px:
            break
        if bi in used_b or di in used_d:
            continue
        used_b.add(bi)
        used_d.add(di)
        out[bi] = (detections[di], dist)
    return out


def classify_block_dual(block, det=None, det_dist_px=None,
                        hsv_min_share=HSV_MIN_SHARE,
                        yolo_min_conf=YOLO_MIN_CONF,
                        conflict_policy="unknown"):
    """单个块的最终颜色判定。``det`` 可为 None（没有 YOLO 匹配）。

    返回 dict：
      class / status(ok|conflict|unknown) / source(both|hsv|yolo|none)
      hsv_class / hsv_share / yolo_class / yolo_conf / agree / reason
    """
    if conflict_policy not in CONFLICT_POLICIES:
        raise ValueError('conflict_policy 只能是 %s' % (CONFLICT_POLICIES,))

    hsv_class = block.get('class', 'unknown')
    hsv_share = float(block.get('class_share', 0.0))
    hsv_ok = (hsv_class in DEFAULT_CLASSES and hsv_share >= hsv_min_share)

    yolo_class = None
    yolo_conf = None
    yolo_ok = False
    if det is not None:
        yolo_class = det.get('class')
        yolo_conf = float(det.get('conf', 0.0))
        yolo_ok = (yolo_class in DEFAULT_CLASSES and yolo_conf >= yolo_min_conf)

    rec = {
        'hsv_class': hsv_class, 'hsv_share': round(hsv_share, 3),
        'yolo_class': yolo_class,
        'yolo_conf': round(yolo_conf, 3) if yolo_conf is not None else None,
        'yolo_dist_px': round(float(det_dist_px), 1) if det_dist_px is not None else None,
        'class': None, 'status': 'unknown', 'source': 'none',
        'agree': None, 'reason': '',
    }

    if hsv_ok and yolo_ok:
        rec['agree'] = (hsv_class == yolo_class)
        if hsv_class == yolo_class:
            rec.update(class_=hsv_class, status='ok', source='both',
                       reason='两源一致')
        else:
            rec['reason'] = '两源冲突：HSV=%s(%.2f) YOLO=%s(%.2f)' % (
                hsv_class, hsv_share, yolo_class, yolo_conf)
            if conflict_policy == 'hsv':
                rec.update(class_=hsv_class, status='ok', source='hsv',
                           reason=rec['reason'] + '；按 hsv 策略取 HSV')
            elif conflict_policy == 'yolo':
                rec.update(class_=yolo_class, status='ok', source='yolo',
                           reason=rec['reason'] + '；按 yolo 策略取 YOLO')
            else:
                rec.update(status='conflict')
    elif hsv_ok:
        rec.update(class_=hsv_class, status='ok', source='hsv',
                   reason='YOLO 无匹配/低置信，HSV 票 %.2f' % hsv_share)
    elif yolo_ok:
        rec.update(class_=yolo_class, status='ok', source='yolo',
                   reason='HSV 票不足（%s %.2f），YOLO %.2f' %
                          (hsv_class, hsv_share, yolo_conf))
    else:
        rec['reason'] = '两源都不足：HSV=%s(%.2f) YOLO=%s(%s)' % (
            hsv_class, hsv_share, yolo_class,
            '%.2f' % yolo_conf if yolo_conf is not None else '无')

    # Python 关键字：内部用 class_，对外仍叫 class。
    if 'class_' in rec:
        rec['class'] = rec.pop('class_')
    return rec


def classify_blocks(blocks, detections, **kw):
    """对一帧的所有块做双源判定；YOLO 检测按一对一分配。

    ``max_uv_px`` 只属于分配阶段；必须摘掉再转给 ``classify_block_dual``，
    否则 TypeError（该函数不接受这个参数）。
    """
    kw = dict(kw)
    max_uv_px = kw.pop('max_uv_px', YOLO_MATCH_PX)
    assigned = assign_yolo(blocks, detections, max_uv_px=max_uv_px)
    out = []
    for i, b in enumerate(blocks):
        det, dist = assigned.get(i, (None, None))
        rec = classify_block_dual(b, det, dist, **kw)
        # 配对按**位置**（``assign_yolo`` 返回的是 enumerate 下标），但对外报的 id
        # 必须沿用块自带的 ``idx``（有就用）—— ``find_blocks`` 的 idx 会跳号
        # （被丢掉的簇不占号），拿位置当真 id 会让调用方按 idx 查表时**查空**：
        # 2026-10-27 B2 终态测量就是这么把 4 块真积木全判成 unknown 的。
        own = b.get('idx')
        rec['block_idx'] = own if isinstance(own, int) else i
        rec['block_xy'] = [b.get('cx'), b.get('cy')]
        out.append(rec)
    return out

# ---------------------------------------------------------------- 跨帧/跨姿态投票
# 「两强票权差小于它就别猜，标 conflict 交人看」—— 2026-10-28 用户拍板。
CLASS_VOTE_MARGIN = 0.2


def vote_class(records):
    """按票权投票选颜色：一条记录一票，权重是它自己的 HSV 票份额。

    返回 ``(class, status, share, reason)``。两强票权差 < ``CLASS_VOTE_MARGIN``
    ⇒ ``conflict``（交人看，不猜）；任何一条自己就是两源冲突的，也带上来。
    缺 ``class_share`` 的旧记录按 1.0 计（一票就是一票，别当成 0 票丢掉）。
    """
    votes, statuses = {}, []
    for b in records:
        cls = b.get("class")
        status = b.get("class_status")
        # 自相矛盾的记录不投票：状态说「认不出」却带着类名，说明那份类名不该
        # 被当证据（否则一条 unknown 记录能把整块推成 ok）。
        if status == "unknown":
            cls = None
        if cls not in (None, "", "unknown"):
            share = b.get("class_share")
            votes[cls] = votes.get(cls, 0.0) + (1.0 if share is None
                                                else float(share))
        if status:
            statuses.append(status)
    if not votes:
        return None, "unknown", 0.0, ""
    ranked = sorted(votes.items(), key=lambda kv: (-kv[1], kv[0]))
    cls, w = ranked[0]
    runner = ranked[1][1] if len(ranked) > 1 else 0.0
    if (w - runner) < CLASS_VOTE_MARGIN:
        return cls, "conflict", w, "票权接近：%s（差 %.2f < %.2f）" % (
            " / ".join("%s %.2f" % kv for kv in ranked), w - runner,
            CLASS_VOTE_MARGIN)
    if "conflict" in statuses:
        # 某一帧/某一姿态内部两源就打起来了：它不投票，但这份不确定必须
        # 带上来，否则会被别的干净票盖成 ok（安静地出错）。
        return cls, "conflict", w, "其中一帧/姿态内部两源冲突"
    return cls, "ok", w, ""

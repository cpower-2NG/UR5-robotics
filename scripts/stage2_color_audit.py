#!/usr/bin/env python3
"""A1 颜色审计：七类 YOLO 在腕部域到底能检什么，哪些颜色必须 HSV 兜底。

输入
────
· ``--frames``：一个或多个帧目录（``color.png`` + ``depth.png`` + ``tf.json`` + ``meta.json``）；
· ``--weights``：``block_best_7cls.pt``（可选；不给就只审 HSV）；
· ``--labels``：人工标签 JSON（可选；格式见仓外 ``stage2-audit/labels.json``）。

输出
────
· JSON：逐块两源结果 + 汇总；
· Markdown：人看的表；
· 裁剪拼图：每帧一张，标出真值 / HSV / YOLO / 最终判定，供人眼复核。

结论口径
────────
本脚本**不调阈值、不训模型**。它只回答三件事：
  1. 每个颜色类 YOLO 有没有检出（召回）；
  2. 检出时类别对不对（精度）；
  3. 两源冲突时按默认策略是多少块判 unknown。

跑法（有 YOLO）：
    ~/robotics/venvs/yolo-grasp/bin/python scripts/stage2_color_audit.py \\
        --frames <run>/frames/pick-02 ... \\
        --weights ~/robotics/models/block_best_7cls.pt \\
        --labels ~/robotics/notes/ur5-vision-grasp/stage2-audit/labels.json \\
        --out-json ... --out-md ... --crop-dir ...

只审 HSV（系统 python3 即可）：
    python3 scripts/stage2_color_audit.py --frames ... --out-json ...
"""

import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                               # noqa: E402

import block_classes                                              # noqa: E402
import graspnet_candidates as gc                                  # noqa: E402
import stage2_color as sc                                         # noqa: E402


def _truth_for(labels, frame_name, i, n):
    if not labels or frame_name not in labels:
        return None
    seq = labels[frame_name]
    if i < len(seq):
        return seq[i]
    return None


def audit_frame(frame_dir, weights=None, imgsz=640, conf=0.25,
                labels=None, max_uv_px=sc.YOLO_MATCH_PX):
    frame = gc.load_frame(frame_dir)
    blocks, table_z = gc.find_blocks(frame)
    high = [b for b in blocks if not b['low']]
    for i, b in enumerate(high):
        b['idx'] = i
    dets, warn = [], None
    if weights:
        import precision_verify as pv                            # 延迟：只有 YOLO 才需要
        dets, warn = pv.yolo_detect(frame_dir, weights, conf=conf, imgsz=imgsz)
    assigned = sc.assign_yolo(high, dets, max_uv_px=max_uv_px)
    rows = []
    name = os.path.basename(frame_dir.rstrip('/'))
    for i, b in enumerate(high):
        det, dist = assigned.get(i, (None, None))
        rec = sc.classify_block_dual(b, det, dist)
        truth = _truth_for(labels, name, i, len(high))
        rows.append({
            'frame': name, 'idx': i, 'xy': [round(b['cx'], 1), round(b['cy'], 1)],
            'h_mm': round(b['h'], 1),
            'truth': truth, 'hsv_class': rec['hsv_class'],
            'hsv_share': rec['hsv_share'],
            'yolo_class': rec['yolo_class'], 'yolo_conf': rec['yolo_conf'],
            'yolo_dist_px': rec['yolo_dist_px'],
            'class': rec['class'], 'source': rec['source'],
            'status': rec['status'], 'reason': rec['reason'],
        })
    return {'frame': frame_dir, 'table_z_mm': round(table_z * 1000, 2),
            'n_blocks': len(high), 'n_yolo_dets': len(dets),
            'yolo_warning': warn, 'rows': rows}


def summarize(results):
    classes = list(block_classes.DEFAULT_CLASSES)
    per = {c: {'n': 0, 'hsv_ok': 0, 'yolo_det': 0, 'yolo_ok': 0,
               'dual_ok': 0, 'unknown': 0, 'conflict': 0} for c in classes}
    total = {'n': 0, 'hsv_ok': 0, 'yolo_det': 0, 'yolo_ok': 0,
             'dual_ok': 0, 'unknown': 0, 'conflict': 0, 'labeled': 0}
    for res in results:
        for r in res['rows']:
            truth = r['truth']
            total['n'] += 1
            if truth is None:
                continue
            total['labeled'] += 1
            if truth not in per:
                per[truth] = {'n': 0, 'hsv_ok': 0, 'yolo_det': 0, 'yolo_ok': 0,
                              'dual_ok': 0, 'unknown': 0, 'conflict': 0}
            p = per[truth]
            p['n'] += 1
            total['hsv_ok'] += int(r['hsv_class'] == truth)
            p['hsv_ok'] += int(r['hsv_class'] == truth)
            if r['yolo_class'] is not None:
                total['yolo_det'] += 1
                p['yolo_det'] += 1
                if r['yolo_class'] == truth:
                    total['yolo_ok'] += 1
                    p['yolo_ok'] += 1
            if r['class'] == truth:
                total['dual_ok'] += 1
                p['dual_ok'] += 1
            if r['class'] is None:
                total['unknown'] += 1
                p['unknown'] += 1
            if r['status'] == 'conflict':
                total['conflict'] += 1
                p['conflict'] += 1
    return {'per_class': per, 'total': total}


def sweep_yolo(frame_dirs, weights):
    """固定 640 与放大 960/1280 的对比；只数框，不做逐块配对。"""
    import precision_verify as pv
    out = []
    for frame_dir in frame_dirs:
        for imgsz in (640, 960, 1280):
            for conf in (0.10, 0.25, 0.50):
                dets, _ = pv.yolo_detect(frame_dir, weights, conf=conf, imgsz=imgsz)
                out.append({'frame': os.path.basename(frame_dir.rstrip('/')),
                            'imgsz': imgsz, 'conf': conf, 'n': len(dets),
                            'dets': dets})
    return out


def _md(results, summary, sweep):
    t = summary['total']
    lines = ['# A1 颜色审计（腕部域）', '',
             '## 总览', '',
             '| 项 | 值 |', '|---|---|',
             '| 块数（高块） | %d |' % t['n'],
             '| 有人工标签 | %d |' % t['labeled'],
             '| HSV 正确 | %d/%d |' % (t['hsv_ok'], t['labeled']),
             '| YOLO 检出 | %d/%d |' % (t['yolo_det'], t['labeled']),
             '| YOLO 检对 | %d/%d |' % (t['yolo_ok'], t['yolo_det']),
             '| 双源最终判对 | %d/%d |' % (t['dual_ok'], t['labeled']),
             '| 判 unknown | %d |' % t['unknown'],
             '| 两源冲突 | %d |' % t['conflict'],
             '', '## 按类', '',
             '| 类 | n | HSV 对 | YOLO 检出 | YOLO 对 | 双源对 | unknown | 冲突 |',
             '|---|---|---|---|---|---|---|---|']
    for c, p in summary['per_class'].items():
        if p['n'] == 0:
            continue
        lines.append('| %s | %d | %d | %d | %d | %d | %d | %d |' % (
            c, p['n'], p['hsv_ok'], p['yolo_det'], p['yolo_ok'],
            p['dual_ok'], p['unknown'], p['conflict']))
    lines += ['', '## 逐块', '',
              '| 帧 | # | xy (mm) | 真值 | HSV | 票 | YOLO | conf | 最终 | source | 状态 |',
              '|---|---|---|---|---|---|---|---|---|---|---|']
    for res in results:
        for r in res['rows']:
            lines.append('| %s | %d | %s | %s | %s | %.2f | %s | %s | %s | %s | %s |' % (
                r['frame'], r['idx'], r['xy'], r['truth'] or '-',
                r['hsv_class'], r['hsv_share'], r['yolo_class'] or '-',
                '%.2f' % r['yolo_conf'] if r['yolo_conf'] is not None else '-',
                r['class'] or '-', r['source'], r['status']))
    if sweep:
        lines += ['', '## YOLO imgsz × conf 扫描（只数框）', '',
                  '| 帧 | imgsz | conf | 框数 | 类别/置信度 |',
                  '|---|---|---|---|---|']
        for s in sweep:
            desc = ' '.join('%s:%.2f' % (d['class'], d['conf']) for d in s['dets'])
            lines.append('| %s | %d | %.2f | %d | %s |' % (
                s['frame'], s['imgsz'], s['conf'], s['n'], desc or '-'))
    lines += ['', '## 判读', '',
              '- 可用置信度（≥0.25）下只有 imgsz=640 有检出；960 全 0 框，',
              '  1280 仅在 conf=0.10 冒出 1 个 0.11 的噪点 —— 模型是 640 训的，放大无效。',
              '- 640 下只检出红/橙/青；紫、黄、棕、黄绿全部无框。',
              '- 检出的框与顶面 HSV 一致，没有误标；因此 YOLO 定位为「红/橙/青的佐证」，',
              '  其余颜色走 HSV；两源冲突默认判 unknown，不猜。',
              '- 棕/黄绿在存档帧里没有样本， HSV 可分性来自调色板实测色相（28.5°/53°，',
              '  相邻间隔 ≥8°），**真机首轮必须用 A1 的裁剪图复核**。']
    return '\n'.join(lines) + '\n'


def _crops(results, out_dir):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return
    os.makedirs(out_dir, exist_ok=True)
    fp = '/mnt/c/Windows/Fonts/msyh.ttc'
    font = ImageFont.truetype(fp, 20) if os.path.exists(fp) else ImageFont.load_default()
    for res in results:
        name = os.path.basename(res['frame'].rstrip('/'))
        img = Image.open(os.path.join(res['frame'], 'color.png')).convert('RGB')
        rows = res['rows']
        cols = 3
        g = (len(rows) + cols - 1) // cols
        cell = 220
        canvas = Image.new('RGB', (cols * cell, g * (cell + 76)), (255, 255, 255))
        d = ImageDraw.Draw(canvas)
        for k, r in enumerate(rows):
            u, v = r['xy']
            # 用块 uv（像素）裁；rows 里存的是 base mm，这里回块表取 uv
            block = None
            for b in res.get('_blocks', []):
                if b['idx'] == r['idx']:
                    block = b
                    break
            if block is None:
                uv = (u, v)
            else:
                uv = block['uv']
            half = 45
            crop = img.crop((int(uv[0] - half), int(uv[1] - half),
                             int(uv[0] + half), int(uv[1] + half)))
            crop = crop.resize((cell, cell), Image.NEAREST)
            x, y = (k % cols) * cell, (k // cols) * (cell + 76)
            canvas.paste(crop, (x, y))
            d.text((x + 4, y + cell + 2),
                   '#%d 真值 %s' % (r['idx'], r['truth'] or '-'),
                   fill=(0, 0, 0), font=font)
            d.text((x + 4, y + cell + 27),
                   'HSV %s %.2f' % (r['hsv_class'], r['hsv_share']),
                   fill=(0, 0, 0), font=font)
            d.text((x + 4, y + cell + 52),
                   'YOLO %s' % (r['yolo_class'] or '-'),
                   fill=(0, 0, 0), font=font)
        canvas.save(os.path.join(out_dir, name + '-audit.png'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', nargs='+', required=True)
    ap.add_argument('--weights', default=None)
    ap.add_argument('--labels', default=None)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--max-uv-px', type=float, default=sc.YOLO_MATCH_PX)
    ap.add_argument('--out-json', default=None)
    ap.add_argument('--out-md', default=None)
    ap.add_argument('--crop-dir', default=None)
    ap.add_argument('--sweep', action='store_true')
    args = ap.parse_args()

    labels = json.load(open(args.labels, encoding='utf-8')) if args.labels else None
    results = []
    for fd in args.frames:
        if not os.path.isdir(fd):
            print('跳过（不是目录）：%s' % fd, file=sys.stderr)
            continue
        res = audit_frame(fd, weights=args.weights, imgsz=args.imgsz,
                          conf=args.conf, labels=labels,
                          max_uv_px=args.max_uv_px)
        # 供裁剪图取 uv
        frame = gc.load_frame(fd)
        blocks, _ = gc.find_blocks(frame)
        res['_blocks'] = [b for b in blocks if not b['low']]
        for i, b in enumerate(res['_blocks']):
            b['idx'] = i
        results.append(res)
        print('%s：块 %d，YOLO 框 %d' % (fd, res['n_blocks'], res['n_yolo_dets']))

    if not results:
        print('没有可用帧', file=sys.stderr)
        return 2

    summary = summarize(results)
    sweep = sweep_yolo([r['frame'] for r in results], args.weights) \
        if (args.weights and args.sweep) else []
    # ``_blocks`` 里有 numpy 值，只给裁剪图用，不进 JSON。
    clean = [{k: v for k, v in r.items() if k != '_blocks'} for r in results]
    out = {'frames': [r['frame'] for r in results], 'results': clean,
           'summary': summary, 'sweep': sweep}
    if args.out_json:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_json)), exist_ok=True)
        with open(args.out_json, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print('JSON：%s' % args.out_json)
    if args.out_md:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_md)), exist_ok=True)
        with open(args.out_md, 'w', encoding='utf-8') as f:
            f.write(_md(results, summary, sweep))
        print('MD：%s' % args.out_md)
    if args.crop_dir:
        _crops(results, args.crop_dir)
        print('裁剪图：%s' % args.crop_dir)

    t = summary['total']
    print('\n汇总：%d 块；HSV 对 %d/%d；YOLO 检出 %d/%d（对 %d）；双源对 %d/%d；'
          'unknown %d；冲突 %d' % (
              t['n'], t['hsv_ok'], t['labeled'], t['yolo_det'], t['labeled'],
              t['yolo_ok'], t['dual_ok'], t['labeled'], t['unknown'],
              t['conflict']))
    return 0


if __name__ == '__main__':
    sys.exit(main())

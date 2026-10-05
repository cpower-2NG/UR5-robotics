#!/usr/bin/env python3
"""分拣任务**配置层**的自检：现场 yaml 里的槽位/类别表是不是我们要的那张。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。
不需要 ROS、不需要 MoveIt、不需要 colcon build。）

为什么值得钉
============
A2 的验收判据里有一句「**类别错分必须能被测试发现**」。这句在**代码**那层
（``slot_routing.py``）已经由 ``test_slot_routing.py`` 的变异钉住了；但错分还有
另一个入口 —— **配置本身写错**：把 ``place_slot_classes`` 里两个名字对调、
少写一格、或者把某一格挪进抓取区。这三件事规划器**一件都不会报错**：

  · 名字对调 → 每一格都有效、间距也合法，只是放错颜色；
  · 少写一格 → 长度检查会拦（这条有）；
  · 挪进抓取区 → ``_validate_zones`` 会拦（这条也有）。

所以真正没被钉住的只有第一条：**同列同色这张表本身**。本文件就是钉它的。
判据不能是「表等于某张写死的表」（那是抄一遍自己，改错了跟着一起改），
而要从用户那句需求里取**结构不变量**：

    同列同色  ⇒  同一列（同 y）的两个槽位必须是同一个类；
                不同列必须是不同的类（4 列 ↔ 4 类，一一对应）。

这条对调两个名字就会破 —— 而它破不破，正是本文件末尾的变异要证明的。
"""
import os
import sys

import yaml

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SITE_YAML = os.path.join(REPO, 'docs', '现场yaml草稿-真机.yaml')

sys.path.insert(0, HERE)
import block_classes                     # noqa: E402  类别表的唯一定义处

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


# ----------------------------------------------------------------------
# 纯判据：喂 (slots, classes, pick_zone, place_z) 进去，出来一串结论。
# 之所以做成纯函数：变异要能在**不碰磁盘**的情况下重跑同一批判据。

def in_box(zone, x, y, z):
    """点是否在 3D 盒子里。

    ⚠️ 这里**重写了一遍**，没有 import ``planner_node.in_zone`` —— 那个模块
    一 import 就拖进 rclpy / MoveIt。判据只有三行，抄过来比拖依赖划算；
    代价是它可能与 planner_node 漂移，所以下面第 6 节拿一个**已知在盒内/
    盒外的点**去验它自己的方向没写反（盒子参数顺序是 [xmin,xmax,ymin,ymax,...]，
    最容易把 x 和 y 的上下界接错，而接错了照样"看起来在工作"）。
    """
    xmin, xmax, ymin, ymax, zmin, zmax = zone
    return xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax


def slot_xy(slots_flat):
    return list(zip(slots_flat[0::2], slots_flat[1::2]))


def columns_of(slots, classes, tol=1e-6):
    """按 y 分列 → {y: [类别名, ...]}（y 取整到 tol 以吸收浮点噪声）。"""
    cols = {}
    for (x, y), c in zip(slots, classes):
        key = round(y / tol) * tol
        cols.setdefault(key, []).append(c)
    return cols


def problems(slots_flat, classes, pick_zone, place_z, stroke_mm=95.0):
    """返回「这张表有哪些毛病」的列表；空列表 = 没问题。

    每条毛病是一句人话 —— 上机前看一眼就知道该改哪。
    """
    out = []
    if len(slots_flat) % 2:
        out.append('place_slots 有 %d 个数（奇数）—— 必须成对 [x0,y0,x1,y1,...]'
                   % len(slots_flat))
        return out
    slots = slot_xy(slots_flat)
    n = len(slots)

    if len(classes) != n:
        out.append('place_slot_classes 有 %d 项，但 place_slots 有 %d 个槽位 —— '
                   '两者必须等长' % (len(classes), n))
        return out

    bad = [c for c in classes if c not in block_classes.DEFAULT_CLASSES]
    if bad:
        out.append('类别名不在类别表里：%s（合法名：%s）'
                   % (bad, block_classes.DEFAULT_CLASSES))

    # ---- 同列同色 + 列↔类 一一对应 ----
    cols = columns_of(slots, classes)
    for y, cs in sorted(cols.items()):
        if len(set(cs)) != 1:
            out.append('y=%.3f 这一列里混了多个类别 %s —— 违反「同列同色」'
                       % (y, cs))
    seen = {}
    for y, cs in sorted(cols.items()):
        c = cs[0]
        if c in seen:
            out.append('类别 %s 同时出现在 y=%.3f 与 y=%.3f 两列 —— '
                       '「每类一列」被破了' % (c, seen[c], y))
        seen[c] = y

    # ---- 间距 ----
    for i in range(n):
        for j in range(i + 1, n):
            dx = slots[i][0] - slots[j][0]
            dy = slots[i][1] - slots[j][1]
            d = (dx * dx + dy * dy) ** 0.5 * 1000.0
            if d < stroke_mm:
                out.append('槽位 %d 与 %d 间距 %.1f mm < 夹爪行程 %.0f mm'
                           % (i, j, d, stroke_mm))

    # ---- 必须在抓取区之外（否则放下去又被抓起来 = 死循环）----
    for i, (x, y) in enumerate(slots):
        if in_box(pick_zone, x, y, place_z):
            out.append('槽位 %d (%.2f,%.2f) 落在 pick_zone 里 —— '
                       '放下去会被重新抓起来' % (i, x, y))
    return out


# ----------------------------------------------------------------------

def load_site():
    with open(SITE_YAML, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    p = doc['grasp_planner']['ros__parameters']
    return (list(p['place_slots']), list(p['place_slot_classes']),
            list(p['pick_zone']), float(p['place_z']),
            float(p.get('ag95_stroke_mm', 95.0)))


def section_shape():
    print('\n[1] 现场 yaml 的形状：8 格、4 类各 2 格')
    slots, classes, pz, pz_h, stroke = load_site()
    check('place_slots 是 8 个槽位（16 个数）', len(slots) == 16,
          '实际 %d 个数' % len(slots))
    check('place_slot_classes 与 place_slots 等长（各 8 项）',
          len(classes) == len(slots) // 2,
          '%d vs %d' % (len(classes), len(slots) // 2))
    hist = {}
    for c in classes:
        hist[c] = hist.get(c, 0) + 1
    check('4 个类别、每类恰好 2 格',
          len(hist) == 4 and set(hist.values()) == {2}, str(hist))
    check('这 4 个名字都在类别表里',
          set(hist) <= set(block_classes.DEFAULT_CLASSES), str(sorted(hist)))
    check('整张表**没有**毛病（同列同色 / 间距 / 在抓取区外）',
          problems(slots, classes, pz, pz_h, stroke) == [],
          '；'.join(problems(slots, classes, pz, pz_h, stroke)))


def section_yellowgreen_deliberately_absent():
    print('\n[2] yellowgreen 是**故意不给槽位**的，不是漏了')
    # 实测：那一块黄方块同时被框成 `yellow 0.56` 与 `yellowgreen 0.65`，
    # 而候选按「同 y 时高分在前」排 ⇒ 黄绿候选排在前面。没有黄绿槽位时它被
    # 判「无槽位」跳过、循环落到同一块的 yellow 候选（结果是对的）；
    # 一旦有了黄绿槽位，这块黄方块就会被送进黄绿槽。
    slots, classes, pz, pz_h, stroke = load_site()
    check('表里没有 yellowgreen 槽位（有的话黄方块会被错分进去）',
          'yellowgreen' not in classes, str(classes))
    check('每个槽位都是具名类（没有空串通配 —— 通配会让谁都能占这一格）',
          all(c for c in classes), str(classes))


def section_in_box_selftest():
    print('\n[3] in_box 自己的方向先验一遍（x/y 上下界最容易接反）')
    zone = [0.4, 0.6, 0.0, 0.45, 0.0, 0.0485]
    check('盒内一点判为真', in_box(zone, 0.5, 0.2, 0.02))
    check('y 低于下限判为假（把 y 上界接到 y 下界就会错这一条）',
          not in_box(zone, 0.5, -0.2, 0.02))
    check('x 超出上限判为假', not in_box(zone, 0.7, 0.2, 0.02))
    check('z 超出上限判为假', not in_box(zone, 0.5, 0.2, 0.2))


def section_mutation():
    print('\n[4] 变异：把配置改坏，上面的判据必须真的变红')
    slots, classes, pz, pz_h, stroke = load_site()
    base = problems(slots, classes, pz, pz_h, stroke)
    check('未变异时 problems() 是空的（否则下面的变异证明不了什么）',
          base == [], str(base))

    # (a) 对调两个类别的名字 → 「同列同色 / 每类一列」必须破
    m = list(classes)
    m[0], m[2] = m[2], m[0]
    p = problems(slots, m, pz, pz_h, stroke)
    check('变异「对调两类」→ 被检出', p != [],
          '对调了 red 与 orange 却没被检出 —— 第 1 节的判据测不出类别错分')

    # (b) 少写一格 → 长度必须破
    p = problems(slots, classes[:-1], pz, pz_h, stroke)
    check('变异「少写一格」→ 被检出', p != [], '长度不等没被检出')

    # (c) 把第一个槽位挪进抓取区 → 「必须在 pick_zone 之外」必须破
    s = list(slots)
    s[0], s[1] = 0.50, 0.20          # y=0.20 在 pick_zone 的 y∈[0,0.45] 内
    p = problems(s, classes, pz, pz_h, stroke)
    check('变异「槽位挪进抓取区」→ 被检出', p != [],
          '挪进 pick_zone 没被检出 —— 会导致放下又被抓起来的死循环')

    # (d) 把间距压到行程以下 → 间距判据必须破
    s = list(slots)
    s[2] = s[0] + 0.05               # 同一 x 行、y 不变 ⇒ 与槽位 0 只差 50 mm
    p = problems(s, classes, pz, pz_h, stroke)
    check('变异「间距压到 50 mm」→ 被检出', p != [],
          '间距过近没被检出 —— 张着 95 mm 的爪子会压到邻格')

    # (e) 把某一列整列换成同一类（4 列变 3 类）→ 一一对应必须破
    m = list(classes)
    m[2] = m[3] = classes[0]
    p = problems(slots, m, pz, pz_h, stroke)
    check('变异「两列合成一类」→ 被检出（每类一列被破）', p != [],
          '两列同类没被检出')


def main():
    print('=' * 68)
    # yaml 缺失也要给可读的退出码，不是 traceback
    if not os.path.exists(SITE_YAML):
        print('SKIP  找不到 %s（退出码 2）' % SITE_YAML)
        return 2
    section_shape()
    section_yellowgreen_deliberately_absent()
    section_in_box_selftest()
    section_mutation()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：\n  - ' + '\n  - '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""``live_infer_record.py`` 的交付前关卡。

（沿用本仓库 ``scripts/test_*.py`` 的约定：``check()`` + PASS/FAIL 计数，
退出码 0 = 通过，2 = 环境没准备好。）

为什么要有它
============
这个脚本要在**真机台面旁边**跑，人在移动方块。它出错的方式和别的脚本不一样：

  · **它不该发任何消息** —— 只订阅。起了完整栈会拉起 UR 驱动、占住
    29999/30001/30004，有把示教器上的程序打成 STOPPED 的前科。所以
    「它只读」这件事值得用 **AST 结构断言**钉死，而不是靠注释里写一句。
  · **类别名来自权重**，与 `block_classes.py` 不一致时画面上的名字整体错位，
    而**没有任何报错** —— 录出来的视频看着"有框有色"，名字全是错的。
    所以启动时必须把权重里的类别表打印出来给人核对（第 3 节钉这一行）。
  · **框的颜色取自调色板**，若两类同色，图例就失去意义（人审视频时靠颜色
    分辨"这是哪一类"）。

第 4 节把这几处各改坏一次，对应断言必须失效。
"""

import ast
import os
import sys
import importlib.util
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import block_classes                    # noqa: E402

try:
    import cv2
except ImportError:
    cv2 = None
try:
    import numpy as np
except ImportError:
    np = None

N_PASS = N_FAIL = N_SKIP = 0
FAILS = []

SRC = os.path.join(HERE, 'live_infer_record.py')


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


def skip(name, why):
    global N_SKIP
    N_SKIP += 1
    print('  SKIP  %s   ← %s' % (name, why))


def load_module(text=None, tag='live'):
    if text is None:
        spec = importlib.util.spec_from_file_location('live_infer_record', SRC)
    else:
        d = tempfile.mkdtemp(prefix='live_')
        p = os.path.join(d, 'live_%s.py' % tag)
        with open(p, 'w') as f:
            f.write(text)
        spec = importlib.util.spec_from_file_location('live_%s' % tag, p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ----------------------------------------------------------------------
def section_readonly():
    """结构断言：这个脚本在**源码层面**就不可能发布消息。"""
    print('\n[1] 只读：AST 里不该存在任何 create_publisher')
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    pubs = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            if isinstance(f, ast.Attribute) and f.attr in (
                    'create_publisher', 'create_service', 'create_client',
                    'create_action_client'):
                pubs.append((f.attr, getattr(node, 'lineno', '?')))
    check('没有 create_publisher / create_service / create_client',
          not pubs, str(pubs))
    # 反向确认：订阅**确实**存在（否则上一条会因为在别的文件上跑而空过）
    subs = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == 'create_subscription']
    check('确实有 create_subscription（证明上一条不是"扫了个空文件"）',
          len(subs) == 1, '找到 %d 处' % len(subs))

    # 也不该 import 任何会驱动机械臂的东西
    src = open(SRC, encoding='utf-8').read()
    banned = [m for m in ('ur_robot_driver', 'dashboard_client', 'urcl',
                          'moveit', 'rtde') if m in src]
    check('没有 import 任何机械臂/控制器相关模块', not banned, str(banned))


def section_colors():
    print('\n[2] 框色取自调色板，且两两不同（图例才有意义）')
    m = load_module()
    seen = {}
    for nm in block_classes.DEFAULT_CLASSES:
        c = m.bgr_of(nm)
        check('%-13s → BGR %s' % (nm, c),
              all(0 <= v <= 255 for v in c), str(c))
        seen[nm] = c
    dup = {}
    for nm, c in seen.items():
        dup.setdefault(c, []).append(nm)
    same = {c: v for c, v in dup.items() if len(v) > 1}
    check('7 个类的框色两两不同', not same, str(same))
    # 调色板外的名字**不报错**，给中性灰 —— 线上真有可能遇到
    try:
        g = m.bgr_of('notacolor')
        ok = True
    except Exception as e:
        ok, g = False, repr(e)
    check('调色板外的类别名不抛异常，给中性灰', ok and g == (127, 127, 127), str(g))


def section_draw():
    print('\n[3] draw()：框色 = 该类颜色；越界的类别号不炸')
    if cv2 is None or np is None:
        skip('draw() 全部用例', '没装 opencv')
        return
    m = load_module()
    names = list(block_classes.DEFAULT_CLASSES)
    # ⚠️ 画布要够大、框要放在图例**下方**：7 行图例大约占掉顶部 160 px，
    #    小画布上两者会重叠，取样点会取到图例的颜色而不是框线的颜色
    #    （第一版就是栽在这里，报的是 yellowgreen 的色）。
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    boxes = [((300, 300, 420, 420), 5, 0.9)]         # 5 = cyan
    m.draw(frame, boxes, names, {'cyan': 3}, 'x')
    want = m.bgr_of('cyan')
    got = tuple(int(v) for v in frame[360, 300])     # 左边框线上一点
    check('框线用的是该类颜色', got == want, '%s vs %s' % (got, want))
    check('框下方内部没被涂色（只画框，不填充）',
          tuple(int(v) for v in frame[360, 360]) == (0, 0, 0),
          str(tuple(int(v) for v in frame[360, 360])))

    # 图例那几行必须真的画进去了（人审视频靠它数数）。只看顶部那片，
    # 别把框本身算进来 —— 否则这条断言在"图例压根没画"时也会绿。
    legend = frame[0:180, 0:300]
    painted = {tuple(int(v) for v in legend[y, x])
               for y in range(0, 180, 3) for x in range(0, 300, 3)}
    painted.discard((0, 0, 0))
    check('图例区域画了多种颜色（7 行各自的底色）',
          len(painted) >= 5, '只有 %d 种非黑颜色' % len(painted))

    # 类别号越界：权重与类别表不一致时会出现，**不能崩**（崩了就白录）
    try:
        frame2 = np.zeros((240, 320, 3), dtype=np.uint8)
        m.draw(frame2, [((40, 60, 140, 160), 99, 0.9)], names, {}, 'x')
        ok = True
    except Exception as e:
        ok = False
    check('越界的类别号不抛异常（画成 ?99）', ok)


def section_model_lookup():
    print('\n[4] 找权重：找不到就返回 None，**绝不联网下载**')
    m = load_module()
    check('不存在的权重 → None',
          m.resolve_model('definitely_missing_weights_12345.pt') is None)
    src = open(SRC, encoding='utf-8').read()
    # ultralytics 的 YOLO("yolov8n.pt") 在文件缺失时会**自动下载** COCO 权重 ——
    # 那会让「检测不到方块」看起来像模型坏了，而不是权重没送到。
    check('源码里没有把裸文件名交给 YOLO 去下载的写法',
          'YOLO("' not in src and "YOLO('" not in src,
          '发现字面量 YOLO(...) 调用 —— 那会触发联网下载')


def section_mutation():
    print('\n[5] 变异：改坏一处，上面的断言必须失效')
    src = open(SRC, encoding='utf-8').read()

    # (a) 让 bgr_of 对所有类返回同一个颜色 → 第 2 节的"两两不同"必须红
    old = '    r, g, b = PALETTE.get(name, (0.5, 0.5, 0.5))'
    assert old in src, '变异锚点没找到'
    mm = load_module(src.replace(old, '    r, g, b = (0.5, 0.5, 0.5)'), 'same')
    cols = {c: mm.bgr_of(c) for c in block_classes.DEFAULT_CLASSES}
    check('变异「所有类同色」→ 色值不再两两不同',
          len(set(cols.values())) == 1,
          '改坏了却不是同一个颜色 —— 第 2 节那条断言测不出东西')

    # (b) 加一个 create_publisher → 第 1 节的结构断言必须红
    mm = load_module(src.replace(
        '        self.create_subscription(Image, topic, self._on_img, 1)',
        '        self.create_subscription(Image, topic, self._on_img, 1)\n'
        '        self.create_publisher(Image, "/oops", 1)'), 'pub')
    tree = ast.parse(open(os.path.join(
        os.path.dirname(mm.__file__), 'live_pub.py'), encoding='utf-8').read())
    pubs = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == 'create_publisher']
    check('变异「加一个 create_publisher」→ 第 1 节会红', len(pubs) == 1,
          '改坏了却没被扫出来 —— 第 1 节的结构断言测不出东西')


def _rclpy_call(node, attr):
    """``node`` 是不是 ``rclpy.<attr>(...)`` 这个调用。"""
    return (isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == attr
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == 'rclpy')


def _stmts_call(stmts, attr):
    """这一串语句里（含嵌套）有没有 ``rclpy.<attr>(...)`` 调用。

    ⚠️ 必须对每条语句再 ``ast.walk`` 一层：``If.body`` 里装的是**语句**
    （``ast.Expr``），调用表达式在它的 ``.value`` 上。直接拿 body 的元素去比
    ``ast.Call`` 永远不成立 —— 于是断言恒红（或恒绿），而看起来还挺像回事。
    第一版就是这么写的，被自己的断言逮住。
    """
    for s in stmts:
        for n in ast.walk(s):
            if _rclpy_call(n, attr):
                return True
    return False


def _shutdown_guarded(tree):
    """源码里那处 ``rclpy.shutdown()`` 是否被 ``if rclpy.ok():`` 守卫着。"""
    for n in ast.walk(tree):
        if isinstance(n, ast.If) and _rclpy_call(n.test, 'ok'):
            if _stmts_call(n.body, 'shutdown'):
                return True
    return False


def section_shutdown():
    print('\n[6] 收工不能崩：rclpy.shutdown() 必须判 ok()')
    # 为什么值得一条断言：Ctrl-C 时 rclpy 自带的信号处理器**已经**关过一次上下文，
    # 无守卫的第二次 shutdown 会抛 RCLError；异常从 finally 里逃出去会**跳过
    # 紧随其后的整段汇总**。而 Ctrl-C 正是本脚本**正常**的收工方式 ⇒
    # 「逐类累计」和「实际处理 fps」这两条判据平时一条都打不出来。
    # 2026-10-05 实测踩到：录了 51 MB，汇总全丢、只剩一句 traceback。
    src = open(SRC, encoding='utf-8').read()
    tree = ast.parse(src)
    n_shut = sum(1 for n in ast.walk(tree) if _rclpy_call(n, 'shutdown'))
    check('源码里恰好有一处 rclpy.shutdown()', n_shut == 1,
          '找到 %d 处' % n_shut)
    check('它被 `if rclpy.ok():` 守卫着', _shutdown_guarded(tree),
          '没有守卫 ⇒ Ctrl-C 收工会二次 shutdown 抛异常，'
          '把「逐类累计 / 实际处理 fps」两条判据整段吞掉')

    old = '        if rclpy.ok():\n            rclpy.shutdown()'
    assert old in src, '变异锚点没找到 —— 断言锚点漂了，本条会空转'
    check('变异「去掉守卫」→ 上一条会红',
          not _shutdown_guarded(ast.parse(src.replace(old, '        rclpy.shutdown()'))),
          '去掉守卫却没被检出 —— 上一条断言测不出东西')


def main():
    print('=' * 68)
    section_readonly()
    section_colors()
    section_draw()
    section_model_lookup()
    section_shutdown()
    section_mutation()
    print('\n%d PASS  %d FAIL  %d SKIP' % (N_PASS, N_FAIL, N_SKIP))
    if FAILS:
        print('未通过：\n  - ' + '\n  - '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

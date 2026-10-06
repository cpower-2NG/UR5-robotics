#!/usr/bin/env python3
"""放置落点复核**接线**的自检。需要 ROS（rclpy），**不需要 MoveIt / 仿真 / 真机**。

（沿用本仓库 ``scripts/test_*.py`` 的约定：退出码 0 = 通过，2 = 环境没准备好。）

与 ``test_place_verify.py`` 的分工
==================================
那个测**判据本身**（纯函数）；这个测**判据有没有真的接进 planner_node**。
两者缺一不可 —— 判据对但没接线，真机上照样一次复核都不做，而纯函数测试全绿。

这里不 spin 执行器（不起 MoveIt、不连臂），而是**直接调方法喂合成消息**。
这样测的三件事都是纯函数测试看不到的：

  A. **参数通路**：7 个参数从声明 → 解析 → `_place_roi`；容差护栏真的硬拒。
  B. **时间戳闸门与旁路**：回 HOME 那一帧之前拍的帧必须被丢掉（臂还挡着），
     而闸门生效的前提是复核期间的帧真的走到了 `_place_verify_consume`
     —— 也就是 `_targets_cb` 的旁路必须排在「忙就早退」**之前**
     （复核期间 `_busy` 恰好是 True，排错位置就永远等不到帧）。
  C. **三种「什么都没做」的文案**与收工台账格式（现场靠 grep 日志验收）。

末尾三段变异把这三处的机制各拆掉一个，对应断言必须失效 —— 不做这一步的话，
「节点会复核」只是我读代码得出的印象。
"""
import argparse
import contextlib
import os
import sys

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PLANNER_SRC = os.path.join(REPO, "ros2_ws/src/grasp_planner")
WS_INSTALL = os.path.join(REPO, "ros2_ws/install")
INSTALL = os.path.join(WS_INSTALL, "grasp_planner")

N_PASS = N_FAIL = 0
FAILS = []

# 与 run 时不同的域，免得撞上正在跑的真节点
DOMAIN = "97"

# 真机 A2 的 8 个槽位（与 docs/现场yaml草稿-真机.yaml 同）：
# 最近同类槽距 0.10 m ⇒ place_verify_tol_xy 必须 < 0.05
SLOTS = [0.50, -0.10, 0.60, -0.10, 0.50, -0.20, 0.60, -0.20,
         0.50, -0.30, 0.60, -0.30, 0.50, -0.40, 0.60, -0.40]
CLASSES = 'red,red,orange,orange,yellow,yellow,cyan,cyan'
PLACE_Z = 0.0285


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


class LogRec:
    """把节点的日志抓下来 —— 现场验收就是 grep 这些行，所以它们**就是接口**。"""

    def __init__(self):
        self.lines = []

    def _rec(self, msg, *a, **k):
        self.lines.append(str(msg))

    info = _rec
    warn = _rec
    error = _rec
    debug = _rec

    def text(self):
        return '\n'.join(self.lines)


@contextlib.contextmanager
def planner(extra=None, carry=True, patch=None):
    """造一个真的 GraspPlanner，用完就收 —— **一次只活一个**。

    为什么必须是上下文管理器而不是 `make_node()`：``rclpy.init()`` 每个进程
    只能调一次，而**参数覆盖是在 init 时定的**（节点自己不解析这些 `-p`）。
    所以「换个参数造第二个节点」只能靠 init → 用 → destroy → shutdown → 再 init
    这个次序走；把两个节点同时活着（第一版就是这么写的）会在第二个 init 上
    抛 `Context.init() must only be called once`。

    ``patch``：在造节点**之前**改 planner_node 模块里的名字（变异用），
    退出时**还原** —— 免得一个变异漏到后面几段里去（那会让后面的断言
    在无意中被变异掉，红得莫名其妙）。
    """
    import rclpy
    from grasp_planner import planner_node
    rclpy.init(args=['--ros-args'] + params(extra, carry))
    saved = {}
    if patch:
        for k, v in patch.items():
            saved[k] = getattr(planner_node, k)
            setattr(planner_node, k, v)
    try:
        node = planner_node.GraspPlanner()
    except BaseException:
        for k, v in saved.items():
            setattr(planner_node, k, v)
        rclpy.shutdown()
        raise
    # 日志换成记录器：既避免 rosout 噪音，也让断言直接对着**那些行**。
    rec = LogRec()
    node.get_logger = lambda: rec
    try:
        yield node, rec
    finally:
        # 复核的死线 Timer 是真的线程，不取消的话它会在节点销毁之后开火。
        t = getattr(node, '_verify_timer', None)
        if t is not None:
            t.cancel()
        for k, v in saved.items():
            setattr(planner_node, k, v)
        node.destroy_node()
        rclpy.shutdown()


def params(extra=None, carry=True):
    # ⚠️ `gripper_backend:=none` 与 `carry_objects:=true` 是**被启动期拒绝**的
    #    组合（planner_node:379：不驱动夹爪就谈不上夹取）。复核的上下文只在
    #    `carry_objects=true` 时才建，所以这里必须用 sim（纯仿真后端，
    #    不连任何硬件，测试也从不真的驱动它）。
    p = ['-p', 'gripper_backend:=sim',
         '-p', 'carry_objects:=%s' % ('true' if carry else 'false'),
         '-p', 'place_slots:=[%s]' % ",".join(str(v) for v in SLOTS),
         '-p', 'place_slot_classes:=[%s]' % CLASSES,
         '-p', 'place_z:=%s' % PLACE_Z]
    # 复核死线调到很长：测试里不希望那个真 Timer 抢在断言之前烧掉。
    p += ['-p', 'place_verify_timeout:=60.0', '-p', 'place_verify_settle:=0.01']
    return p + (extra or [])



# ----------------------------------------------------------------------
# 合成消息：只填复核真正读的三个字段
#   results[0].hypothesis.class_id / .score / .pose.pose.position
# —— 与 `_judge_current_frame` 和抓取段 `_select_and_approach` 读的**同一组**。
# ----------------------------------------------------------------------
def make_msg(items, stamp=None):
    from vision_msgs.msg import Detection3DArray, Detection3D
    from vision_msgs.msg import ObjectHypothesisWithPose
    msg = Detection3DArray()
    if stamp is not None:
        msg.header.stamp = stamp
    for cls, score, x, y, z in items:
        d = Detection3D()
        oh = ObjectHypothesisWithPose()
        oh.hypothesis.class_id = cls
        oh.hypothesis.score = score
        oh.pose.pose.position.x = float(x)
        oh.pose.pose.position.y = float(y)
        oh.pose.pose.position.z = float(z)
        d.results.append(oh)
        msg.detections.append(d)
    return msg


def red_at_slot0(score=0.9):
    """槽位[0] = (0.50,-0.10) 上躺着一块 red —— 一次成功的复核。"""
    return [('red', score, 0.499, -0.101, PLACE_Z)]


def now_stamp(node, offset_s=0.0):
    """当前时钟（可选偏移）转成消息戳。"""
    from rclpy.duration import Duration
    t = node.get_clock().now()
    if offset_s:
        t = t - Duration(seconds=abs(offset_s)) if offset_s < 0 \
            else t + Duration(seconds=offset_s)
    return t.to_msg()


def arm_verify(node, index=0):
    """把节点摆到「刚放完第 index 块、已开始看帧」的状态（不碰执行器）。

    用的是真方法（`_snapshot_place_verification` → `_begin_place_verification`），
    只手工填 `after_stamp`（真机上由 `_on_home_done` 在回 HOME 那一刻填）。
    """
    node._place_index = index
    node._place_target = (SLOTS[2 * index], SLOTS[2 * index + 1])
    node._holding_class = CLASSES.split(',')[index]
    node._snapshot_place_verification()
    ctx = node._verify_ctx
    ctx['after_stamp'] = node.get_clock().now()
    # 走**真**入口（而不是手工置 `_verify_active`）：帧计数就是在这里清的，
    # 手工置位会让计数跨槽泄漏，测出来的超时文案与实际不符。
    node._begin_place_verification()
    return ctx


# ----------------------------------------------------------------------
# [A] 参数通路与启动期护栏
# ----------------------------------------------------------------------
def part_a_params():
    print('\n[A] 7 个参数的通路 + 容差护栏（硬错误，不是告警）')
    with planner() as (node, _):
        want = {'place_verify_enabled': True, 'place_verify_timeout': 60.0,
                'place_verify_settle': 0.01, 'place_verify_tol_xy': 0.025,
                'place_verify_z_tol': 0.03, 'place_verify_min_score': 0.30,
                'place_verify_roi_margin': 0.05}
        for k, v in want.items():
            got = getattr(node, k)
            check('A1 %s = %r' % (k, v), got == v, '实际 %r' % (got,))

        # ROI = 全部槽位包围盒外扩 margin：x∈[0.50,0.60] → [0.45,0.65]；
        # y∈[-0.40,-0.10] → [-0.45,-0.05]
        check('A2 _place_roi 是槽位包围盒外扩 50 mm',
              tuple(round(v, 4) for v in node._place_roi)
              == (0.45, 0.65, -0.45, -0.05), str(node._place_roi))

    # 容差盖到邻槽 ⇒ 必须**拒绝启动**（否则每格都通过、Δ 全是邻槽的）
    try:
        with planner(['-p', 'place_verify_tol_xy:=0.05']):
            check('A3 tol_xy = 最近槽距一半(0.05) 时必须拒绝启动', False,
                  '居然起来了')
    except ValueError as e:
        check('A3 tol_xy = 最近槽距一半(0.05) 时必须拒绝启动', True)
        check('  错误信息点出「会盖到邻槽」以及该怎么改',
              '邻槽' in str(e) and 'tol_xy' in str(e), str(e)[:120])
    except Exception as e:                      # noqa: BLE001
        check('A3 tol_xy = 最近槽距一半(0.05) 时必须拒绝启动', False,
              '抛的是 %s：%s' % (type(e).__name__, e))

    # 关掉复核时那条护栏不参与 —— 否则老配置会被无谓拦下
    with planner(['-p', 'place_verify_enabled:=false',
                  '-p', 'place_verify_tol_xy:=0.5']):
        check('A4 place_verify_enabled=false 时护栏不参与（旁路彻底）', True)

    # 只走位（carry_objects=false）时**不建上下文** —— 那正是「不适用」的来源
    with planner(carry=False) as (node, _):
        node._place_index = 0
        node._place_target = (0.50, -0.10)
        node._holding_class = 'red'
        node._snapshot_place_verification()
        check('A5 carry_objects=false 时不建复核上下文（没真放下东西）',
              node._verify_ctx is None and node._verify_order == [],
              str(node._verify_ctx))
        node._report_place_verification()


# ----------------------------------------------------------------------
# [B] 时间戳闸门 + `_targets_cb` 的旁路
# ----------------------------------------------------------------------
def part_b_gate_and_bypass():
    print('\n[B] 时间戳闸门（臂还挡着的那帧必须丢）+ 忙早退之前的旁路')

    # --- B1 快照：必须是**拷贝**，且发生在 _clear_holding 之前
    with planner() as (node, _):
        node._place_index = 0
        node._place_target = (0.50, -0.10)
        node._holding_class = 'red'
        node._snapshot_place_verification()
        ctx = node._verify_ctx
        check('B1 快照抓到期望坐标 = 实际下发的槽位坐标',
              ctx['expected_xy'] == (0.50, -0.10), str(ctx['expected_xy']))
        check('B1b 快照抓到期望类别 red（那一刻 _holding_class 还在）',
              ctx['expected_class'] == 'red', repr(ctx['expected_class']))
        check('B1c 快照时 after_stamp 还是空的（回 HOME 才填）',
              ctx['after_stamp'] is None)
        check('B1d 快照进了台账顺序表', node._verify_order == [0],
              str(node._verify_order))
        node._clear_holding()
        check('B1e 清完 holding 之后上下文仍在（是拷贝，不是引用）',
              node._verify_ctx['expected_class'] == 'red')
        # 复核是异步的：下一块会改写 _place_target，上下文不能被它带走
        node._place_target = (0.60, -0.40)
        node._holding_class = 'cyan'
        check('B1f 下一块改写了 _place_target，已记下的期望值不受影响',
              node._verify_ctx['expected_xy'] == (0.50, -0.10)
              and node._verify_ctx['expected_class'] == 'red')

    # --- B2 旁路：复核期间的帧走到复核侧，**不**落到「缓存最新一批」那条路
    with planner() as (node, _):
        arm_verify(node, 0)
        node._pending_msg = None
        node._targets_cb(make_msg(red_at_slot0(), now_stamp(node, 1.0)))
        check('B2 复核期间 /targets_3d 走到了复核侧（帧计数 +1）',
              node._verify_frames_total == 1, str(node._verify_frames_total))
        check('B2b 而且**没有**落到正常路径（_pending_msg 仍空）',
              node._pending_msg is None)
        check('B2c 判出来了 → 立刻收线，不等超时',
              node._verify_ctx is None and node._verify_active is False)
        check('B2d 落账了（槽位 0 有结果）', 0 in node._verify_results)
        res = node._verify_results[0][1]
        check('B2e 判决是「通过」', res.verdict == 'OK', res.verdict)
        check('B2f 且带着 Δ 明细', 'Δ=' in res.detail, res.detail)

    # --- B3 闸门：回 HOME 之前拍的帧**不算证据**
    with planner() as (node, _):
        arm_verify(node, index=2)                # 槽位[2] = orange
        node._targets_cb(make_msg([('orange', 0.9, 0.50, -0.20, PLACE_Z)],
                                  now_stamp(node, -5.0)))
        check('B3 旧帧（比回 HOME 早 5 s）被丢掉',
              node._verify_frames_stale == 1, str(node._verify_frames_stale))
        check('B3b 丢掉之后**没有**落账（否则会把臂判成方块）',
              node._verify_results == {}, str(node._verify_results))
        check('B3c 上下文还在，继续等', node._verify_ctx is not None)
        node._targets_cb(make_msg([('orange', 0.9, 0.50, -0.20, PLACE_Z)],
                                  now_stamp(node, 1.0)))
        check('B3d 新帧到了就判出来了', node._verify_results.get(2) is not None,
              str(list(node._verify_results)))
        check('B3e 记的是槽位[2] 的期望坐标',
              node._verify_results[2][0]['expected_xy'] == (0.50, -0.20))
        check('B3f 判决是通过', node._verify_results[2][1].verdict == 'OK')

    # --- B4 无戳帧：不能默认当新帧（戳缺失时混进旧帧 = 静默判错）
    with planner() as (node, _):
        arm_verify(node, 0)
        node._targets_cb(make_msg(red_at_slot0(), None))
        check('B4 无戳帧单独计数（不冒充新帧）',
              node._verify_frames_nostamp == 1
              and node._verify_results == {}, str(node._verify_frames_nostamp))
        check('B4b 也没落到正常路径', node._pending_msg is None)

    # --- B5 settle 之后才开始看：`_begin_place_verification` 是那个分岔点
    with planner() as (node, _):
        node._place_index = 0
        node._place_target = (0.50, -0.10)
        node._holding_class = 'red'
        node._snapshot_place_verification()
        check('B5 快照之后还没在看帧（臂还在放置区上方）',
              node._verify_active is False)
        # 这一帧「放下之后、回 HOME 之前」到达 —— 必须**不**被消费
        node._targets_cb(make_msg(red_at_slot0(), now_stamp(node, 1.0)))
        check('B5b 还没开始看帧时，帧走正常路径（复核侧一帧未收）',
              node._verify_frames_total == 0 and node._pending_msg is not None)
        node._verify_ctx['after_stamp'] = node.get_clock().now()
        node._begin_place_verification()
        check('B5c _begin_place_verification 之后才开始看',
              node._verify_active is True and node._verify_timer is not None)


# ----------------------------------------------------------------------
# [C] 超时三分支 + 收工台账
# ----------------------------------------------------------------------
def part_c_timeout_and_report():
    print('\n[C] 超时三分支的文案 + 收工台账（现场靠 grep 日志验收）')

    # C1：一帧都没来（投影器从不发空数组 ⇒ 这才是「相机看不见」的形态）
    with planner() as (node, rec):
        arm_verify(node, 0)
        node._on_place_verify_timeout()
        t1 = rec.text()
        check('C1 一帧都没来 → 文案指向「/targets_3d 一条新消息都没有」',
              '/targets_3d' in t1 and '一条新消息都没有' in t1, t1[-200:])
        check('C1b 落账为「超时未复核」',
              node._verify_results[0][1].verdict == 'TIMEOUT',
              str(node._verify_results[0][1].verdict))
        check('C1c 收线了（_verify_ctx 清空，不会卡住状态机）',
              node._verify_ctx is None and node._verify_active is False)

    # C2：来了 N 帧但都不新（戳冻结 / 时钟不同域）—— 原因与 C1 完全不同
    with planner() as (node, rec):
        arm_verify(node, 0)
        for _ in range(3):
            node._targets_cb(make_msg(red_at_slot0(), now_stamp(node, -3.0)))
        node._on_place_verify_timeout()
        t2 = rec.text()
        check('C2 「来了 3 帧但都不新」的文案与 C1 不同', t2 != t1, t2[-200:])
        check('C2b 并点名戳的可能原因（冻结 / 不同域 / 时钟）',
              '冻结' in t2 or '不同域' in t2 or '时钟' in t2, t2[-260:])
        check('C2c 帧计数与「新帧数」分开报（3 条全是旧的）',
              '3' in t2, t2[-260:])

    # C3：来了新帧但放置区里一个检测都没有 ⇒ UNAVAILABLE（**不是**失败）
    with planner() as (node, rec):
        arm_verify(node, 0)
        node._targets_cb(make_msg([('red', 0.9, 0.50, 0.35, PLACE_Z)],
                                  now_stamp(node, 1.0)))   # 全在抓取区，区外
        node._on_place_verify_timeout()
        t3 = rec.text()
        verdict = node._verify_results[0][1].verdict
        check('C3 新帧在但放置区空 → 判 UNAVAILABLE 而不是失败',
              verdict == 'UNAVAILABLE', verdict)
        check('C3b 文案把「判不了」与「放失败」分开',
              '不可用' in t3 or '取景' in t3 or '看不见' in t3, t3[-260:])

    # C4：收工台账的三种「什么都没做」各有各的话
    with planner(carry=False) as (node, rec):
        node._report_place_verification()
        check('C4 carry_objects=false → 报「不适用」而不是「0/8 通过」',
              '不适用' in rec.text(), rec.text())
        check('  并且**没有**打出汇总行（那会造出一次假的失败复盘）',
              '落点复核汇总' not in rec.text(), rec.text())
    with planner(['-p', 'place_verify_enabled:=false']) as (node, rec):
        node._report_place_verification()
        check('C4b place_verify_enabled=false → 报「关闭」',
              '关闭' in rec.text(), rec.text())
        check('  同样没有汇总行', '落点复核汇总' not in rec.text())
    with planner() as (node, rec):
        node._report_place_verification()
        check('C4c 一次都没放成 → 「无可复核」', '无可复核' in rec.text(),
              rec.text())

    # C5：完整台账：一行通过 + 一行放错类别 + 一行超时，外加汇总
    with planner() as (node, rec):
        arm_verify(node, 0)                       # 槽位[0] red → 通过
        node._targets_cb(make_msg([('red', 0.92, 0.498, -0.103, PLACE_Z)],
                                  now_stamp(node, 1.0)))
        arm_verify(node, 1)                       # 槽位[1] 期望 red，实际 cyan
        node._targets_cb(make_msg([('cyan', 0.88, 0.601, -0.099, PLACE_Z)],
                                  now_stamp(node, 1.0)))
        arm_verify(node, 6)                       # 槽位[6] 超时
        node._on_place_verify_timeout()
        node._report_place_verification()
        txt = rec.text()
        check('C5 台账里有明细标题', '落点复核（Δ = 实测 − 期望）' in txt)
        check('C5b 三行都在（按槽位下标，不是按完成顺序）',
              '槽位[0]' in txt and '槽位[1]' in txt and '槽位[6]' in txt)
        check('C5c 汇总行五类各归各位',
              all(k in txt for k in ('通过 1', '失败 1', '不可用 0',
                                     '超时未复核 1', '未复核 0')), txt[-320:])
        check('C5d 槽位[1] 报的是「放进了别的类别」而不是「没找到」',
              '别的类别' in txt, txt[-320:])
        # ⚠️ 取**台账**那一行（以「槽位[」开头），不是 `_begin` 的日志行
        #    （「落点复核 槽位[0]（类别 red） 期望… 等一帧」）—— 两者都含「槽位[0]」。
        line0 = [l for l in rec.lines if l.strip().startswith('槽位[0]')][0]
        check('C5e 通过行带着实测 Δ', 'Δ=' in line0 and '通过' in line0, line0)
        # ⚠️ 帧计数**不能跨槽泄漏**：槽位[6] 一帧都没来过，它的超时文案里
        #    不该出现「看了 2 帧」这种前两格攒下的数字（本测试抓到过这个 bug）。
        line6 = [l for l in rec.lines if l.strip().startswith('槽位[6]')][0]
        check('C5f 槽位[6] 的超时文案说「一帧都没来」，不带前几格的帧数',
              '一条新消息都没有' in line6, line6)


# ----------------------------------------------------------------------
# [E] 收线的唯一保证：每条路都要回到 _reset_busy（否则是静默死锁）
# ----------------------------------------------------------------------
def part_e_no_deadlock():
    print('\n[E] 收线保证：无论帧来不来，_busy 一定会被放下')
    # `_busy` 若停在 True，节点从此不再接受任何目标 —— 而且**一条日志都没有**。
    # 这正是本项目最怕的收尾方式，所以每条收线路径都要单独钉一次。

    # E1：正常判出来
    with planner() as (node, _):
        arm_verify(node, 0)
        node._busy = True
        node._targets_cb(make_msg(red_at_slot0(), now_stamp(node, 1.0)))
        check('E1 判出来之后 _busy 被放下 + 复核关掉',
              node._busy is False and node._verify_active is False,
              'busy=%r active=%r' % (node._busy, node._verify_active))

    # E2/E3/E4：超时的三条分支
    for label, prep in (
            ('一帧都没来', lambda n: None),
            ('来了但都不新', lambda n: [n._targets_cb(
                make_msg(red_at_slot0(), now_stamp(n, -3.0))) for _ in range(2)]),
            ('来了新帧但区里空', lambda n: n._targets_cb(
                make_msg([('red', 0.9, 0.50, 0.35, PLACE_Z)],
                         now_stamp(n, 1.0))))):
        with planner() as (node, _):
            arm_verify(node, 0)
            node._busy = True
            prep(node)
            node._on_place_verify_timeout()
            check('E2 超时（%s）之后 _busy 也被放下' % label,
                  node._busy is False and node._verify_active is False,
                  'busy=%r active=%r' % (node._busy, node._verify_active))

    # E5：`_drop_place_verification` **故意**不复位 —— 复位归调用方
    #     （`_open_and_reset` / `_finish`）。这条契约写反了会让恢复流程走不下去，
    #     所以把「故意」这两个字钉在测试里。
    with planner() as (node, _):
        arm_verify(node, 0)
        node._busy = True
        node._drop_place_verification('测试：模拟失败恢复')
        check('E5 _drop 只落账「未复核」、**不**碰 _busy（复位归调用方）',
              node._busy is True and node._verify_active is False
              and node._verify_ctx is None, 'busy=%r' % node._busy)
        check('  且落账的原因是调用方给的那句话（台账里要能看出为什么没复核）',
              node._verify_results[0][1].detail == '测试：模拟失败恢复',
              node._verify_results[0][1].detail)


# ----------------------------------------------------------------------
# [D] 变异：把三处机制各拆掉一个，对应断言必须失效
# ----------------------------------------------------------------------
def part_d_mutation():
    print('\n[D] 变异：机制拆掉之后，前面的断言必须失效')

    # M1：快照挪到 _clear_holding **之后** —— 期望类别丢了。
    #     这正是「复核时现读 _holding_class」那个坑：类别判据失效之后，
    #     放错格（红块进黄格）也会报通过，而日志看起来完全正常。
    print('  --- M1：快照排在 _clear_holding 之后 ---')
    with planner() as (node, _):
        node._place_index = 0
        node._place_target = (0.50, -0.10)
        node._holding_class = 'red'
        node._clear_holding()                  # 顺序故意反
        node._snapshot_place_verification()
        got = node._verify_ctx['expected_class']
        check('M1 顺序反了 → 期望类别塌成空串（所以 B1b 不是空转）',
              got == '', repr(got))
        check('  空期望类别下，放错格也会判「通过」—— 这正是要防的静默错',
              _judge_with(node, expected_class='',
                          items=[('cyan', 0.9, 0.50, -0.10, PLACE_Z)]) == 'OK')

    # M2：拆掉时间戳闸门 —— 臂还挡着的那一帧会被当成证据，
    #     而它会判成「不可用」，现场读起来像「相机没取景到」。
    print('  --- M2：拆掉时间戳闸门 ---')
    with planner() as (node, rec):
        arm_verify(node, 0)
        node._verify_ctx['after_stamp'] = None         # 等价于闸门不存在
        node._targets_cb(make_msg([('red', 0.9, 0.50, 0.35, PLACE_Z)],
                                  now_stamp(node, -30.0)))
        check('M2 闸门拆掉后，旧帧被当成证据收下（所以 B3 不是空转）',
              node._verify_frames_total == 1 and node._verify_frames_stale == 0,
              '总 %d 旧 %d' % (node._verify_frames_total,
                               node._verify_frames_stale))
        node._on_place_verify_timeout()
        check('  于是超时文案改口说「看了 1 帧」（闸门生效时是「一条新消息都没有」）'
              ' —— 现场会据此去查相机，而真正的问题是臂还挡着',
              '看了 1 帧' in rec.text(), rec.text()[-220:])
        check('  判决也跟着错：那一帧判成「不可用」落账',
              node._verify_results[0][1].verdict == 'UNAVAILABLE',
              str(node._verify_results[0][1].verdict))

    # M3：复核期间不动 `_verify_active`（等价于旁路不存在）—— 帧被
    #     「忙就早退」吃掉，复核只能超时。这就是「每次都超时，像相机坏了」。
    print('  --- M3：复核期间不走旁路 ---')
    with planner() as (node, _):
        arm_verify(node, 0)
        node._verify_active = False            # 等价于旁路不存在
        node._pending_msg = None
        node._targets_cb(make_msg(red_at_slot0(), now_stamp(node, 1.0)))
        check('M3 旁路拆掉 → 复核侧一帧都没收到（所以 B2 不是空转）',
              node._verify_frames_total == 0, str(node._verify_frames_total))
        check('  帧落到了正常路径（_pending_msg 被写了）',
              node._pending_msg is not None)
        check('  于是复核只能等超时 —— 正是要防的那种「像相机坏了」',
              node._verify_ctx is not None and node._verify_results == {})

    # M4：把容差护栏拆掉 —— 容差盖到邻槽时照样启动，于是每格都「通过」，
    #     而 Δ 报的是**邻槽**的偏移量（绿色但无意义）。
    #     ⚠️ 拆法是把 `_validate_zones` 换成空实现，**不是**只传个坏参数 ——
    #        后者正是被护栏拦住的那条路，那样写等于什么都没变异。
    print('  --- M4：拆掉容差护栏 ---')
    from grasp_planner import planner_node
    saved = planner_node.GraspPlanner._validate_zones
    planner_node.GraspPlanner._validate_zones = lambda self, *a, **k: None
    try:
        with planner(['-p', 'place_verify_tol_xy:=0.05']) as (node, _):
            check('M4 护栏拆掉后 tol_xy=0.05 也能起来（所以 A3 不是空转）', True)
            # 一块偏出 45 mm 的红 —— 已经超出方块自身 40 mm 的脚印、
            # 半个身子压在邻格上了（离槽位[1] 只剩 55 mm）。
            off = _judge_with(node, expected_class='red',
                              items=[('red', 0.9, 0.545, -0.10, PLACE_Z)])
            check('  于是偏出 45 mm 的方块照样报「通过」（Δ 看上去完全正常）',
                  off == 'OK', off)
            node.place_verify_tol_xy = 0.025
            off2 = _judge_with(node, expected_class='red',
                               items=[('red', 0.9, 0.545, -0.10, PLACE_Z)])
            check('  对照：容差收到 25 mm（上游默认值）时它判 NOT_FOUND',
                  off2 == 'NOT_FOUND', off2)
    finally:
        planner_node.GraspPlanner._validate_zones = saved


def _judge_with(node, expected_class, items):
    """直接调节点的 `_judge_current_frame`（判据全在 place_verify 里）。"""
    ctx = {'expected_xy': (0.50, -0.10), 'expected_class': expected_class,
           'expected_z': PLACE_Z, 'after_stamp': None}
    return node._judge_current_frame(make_msg(items), ctx).verdict


def main():
    ap = argparse.ArgumentParser(description='规划器落点复核接线自检')
    ap.add_argument('--only', choices=['A', 'B', 'C', 'D', 'E'], default=None)
    args = ap.parse_args()

    if not os.environ.get('ROS_DISTRO'):
        print('先 source /opt/ros/humble/setup.bash 与 ros2_ws/install/setup.bash'
              '（退 2）')
        return 2
    sys.path.insert(0, PLANNER_SRC)
    sys.path.insert(0, os.path.join(INSTALL, 'lib/python3.10/site-packages'))
    os.environ.setdefault('ROS_DOMAIN_ID', DOMAIN)

    try:
        import rclpy  # noqa: F401
    except ImportError as e:
        print('导入 rclpy 失败：%s（退 2）' % e)
        return 2

    if args.only in (None, 'A'):
        part_a_params()
    if args.only in (None, 'B'):
        part_b_gate_and_bypass()
    if args.only in (None, 'C'):
        part_c_timeout_and_report()
    if args.only in (None, 'E'):
        part_e_no_deadlock()
    if args.only in (None, 'D'):
        part_d_mutation()

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

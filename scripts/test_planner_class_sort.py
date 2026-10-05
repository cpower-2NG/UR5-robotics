#!/usr/bin/env python3
"""规划器「按类别分拣」的集成自检。**需要 ROS，但不需要 MoveIt / 仿真 / 真机**。

（沿用本仓库 ``scripts/test_*.py`` 的约定：退出码 0 = 通过，2 = 环境没准备好。）

和 ``test_slot_routing.py`` 的分工
==================================
那个测的是**判据本身**（纯函数，秒级，不需要 ROS）；这个测的是
**判据有没有真的接进规划器**。两者缺一不可：

  · 判据对但没接线 —— 规划器照样把黄块放进红格，而纯函数测试全绿；
  · 接线对但判据错 —— 同上。

所以这里不重测 ``pick_slot`` 的语义，只测三件纯函数测试看不到的事：
  A. **参数通路**：``place_slot_classes`` 从声明 → 类型 → 解析 → 启动日志；
     （空数组的类型是 rclpy 推断出来的，曾经推成 BYTE_ARRAY —— 见 D 段。）
  B. **节点里的消费顺序**：槽位占用是真的逐格标记，不是每次重算；
  C. **接线**：节点确实调 ``slot_routing.pick_slot``，没有被旁路。

D 段是变异：把 ``pick_slot`` 换成「不看类别」的版本，B 段的断言必须失效。
没有这一步的话，「节点调了 pick_slot」只是我读代码得出的印象。
"""
import argparse
import os
import shlex
import subprocess
import sys

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PLANNER_SRC = os.path.join(REPO, "ros2_ws/src/grasp_planner")
# ⚠️ 两个路径不一样：setup.bash 在工作区级的 install/ 下，
#    包自己的东西（lib/share）在 install/<包名>/ 下。混起来会得到一个
#    「明明编译过了却报没编译」的假失败。
WS_INSTALL = os.path.join(REPO, "ros2_ws/install")
INSTALL = os.path.join(WS_INSTALL, "grasp_planner")

N_PASS = N_FAIL = 0
FAILS = []

# 与 run 时同一个域，免得撞上正在跑的真节点
DOMAIN = "96"


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


# 5 个槽位：red red yellow yellow 通配
# ⚠️ 全部槽位都必须在 pick_zone 之外（y 不在 [-0.18, 0.16] 里）—— 否则
# 节点启动就被 _validate_zones 拒掉。第一版把通配槽位写在 (0.80, 0.00)，
# 正好落在抓取区里，于是这一步直接失败：**护栏本身是对的**。
SLOTS_FLAT = [0.44, 0.24, 0.60, 0.24, 0.44, -0.24, 0.60, -0.24, 0.80, -0.24]
# ⚠️ 通配槽位必须写成带引号的 ""（实测）：
#   · `[red,red,yellow,yellow,]` 末尾的裸空项会被 ROS 的解析器**静默丢掉**，
#     变成 4 项 —— 看着像「4 类 + 1 个通配」，实际是「只有 4 个槽位」；
#   · `[red,,yellow]` 中间裸空项更狠：rclpy.init 直接抛 RCLError 起不来。
#   这一条必须钉在测试里，因为它是**唯一**会被写错还没人发现的地方。
ROUTED = 'red,red,yellow,yellow,""'


def make_node(extra_params, patch=None):
    """在当前 rclpy 上下文里造一个真的 GraspPlanner。

    ``patch``：在造节点**之前**改 planner_node 模块里的名字（变异用）。
    必须在构造前打补丁 —— 节点的 __init__ 里就会读这些参数。
    """
    import rclpy
    rclpy.init(args=['--ros-args'] + extra_params)
    from grasp_planner import planner_node
    if patch:
        for k, v in patch.items():
            setattr(planner_node, k, v)
    node = planner_node.GraspPlanner()
    return node


def shutdown(node):
    import rclpy
    node.destroy_node()
    rclpy.shutdown()


def base_params(slot_classes=None):
    p = ['-p', 'gripper_backend:=none', '-p', 'carry_objects:=false',
         '-p', 'place_slots:=[%s]' % ",".join(str(v) for v in SLOTS_FLAT)]
    if slot_classes is not None:
        p += ['-p', 'place_slot_classes:=[%s]' % slot_classes]
    return p


def next_idx(node, cls):
    """只取下标 —— 坐标不参与路由判据。"""
    got = node._next_place_slot(cls)
    return None if got is None else got[0]


# ----------------------------------------------------------------------
def part_b_routing():
    """B. 节点里的消费顺序（路由开着）"""
    print('\n[B] 节点里的按类别消费顺序（red red yellow yellow 通配）')
    node = make_node(base_params(ROUTED))
    try:
        check('路由表解析正确', node.slot_classes == ['red', 'red', 'yellow',
                                                      'yellow', ''],
              str(node.slot_classes))
        check('初始占用表全空', node._place_used == [False] * 5,
              str(node._place_used))

        check('red → 0 号', next_idx(node, 'red') == 0)
        check('yellow → 2 号（**不是** 0 号）', next_idx(node, 'yellow') == 2)
        check('cyan 没有专属格 → 退到通配 4 号',
              next_idx(node, 'cyan') == 4)

        # 查两次不能改变占用（否则「查一下有没有空位」会吃掉槽位）
        node._next_place_slot('red')
        node._next_place_slot('red')
        check('反复查询不消费槽位', node._place_used == [False] * 5,
              str(node._place_used))

        # 真的用掉 red 的两格
        node._place_used[0] = True
        check('用掉 0 号后 red → 1 号', next_idx(node, 'red') == 1)
        node._place_used[1] = True
        check('red 专属格用完 → 退到通配 4 号', next_idx(node, 'red') == 4)
        node._place_used[4] = True
        check('通配也用了 → red 无处可放', next_idx(node, 'red') is None)

        check('yellow 不受 red 占用影响，仍是 2 号',
              next_idx(node, 'yellow') == 2)
        node._place_used[2] = True
        check('yellow 用掉 2 号 → 3 号', next_idx(node, 'yellow') == 3)
        node._place_used[3] = True
        check('yellow 也满了 → None', next_idx(node, 'yellow') is None)

        check('_slots_left_for 与实际一致（red 剩 0）',
              node._slots_left_for('red') == 0)
        check('_slots_left_for 与实际一致（未用过的类剩 0）',
              node._slots_left_for('magenta') == 0)

        # 收工原因要能区分「没配这个类」与「配了但占满」
        r_missing = node._place_exhausted_reason('magenta')
        check('没配过的类别：收工原因点出「一个槽位都没有」',
              '一个槽位都没有' in r_missing, r_missing)
        check('  并提示去比对权重里的类别名（拼写）',
              '权重' in r_missing, r_missing)
        r_full = node._place_exhausted_reason('red')
        check('配过但占满：收工原因报专属格数',
              '专属 2 个' in r_full, r_full)
    finally:
        shutdown(node)


def part_b2_unrouted():
    """B2. 不路由时必须与 A1 逐字相同"""
    print('\n[B2] 不路由（默认）时退回 A1 行为')
    node = make_node(base_params())          # 不给 place_slot_classes
    try:
        # 不路由在内部就是「全通配」（长度 = 槽位数），见 validate_slot_classes
        check('不路由 → 归一化成全通配（长度仍等于槽位数）',
              node.slot_classes == [''] * (len(SLOTS_FLAT) // 2),
              str(node.slot_classes))
        # A1 的只增游标序列：0,1,2,3,4
        seq = []
        for cls in ('red', 'yellow', 'cyan', 'anything', ''):
            i = next_idx(node, cls)
            seq.append(i)
            node._place_used[i] = True
        check('任意类别都按 0,1,2,3,4 顺序拿槽位（与 A1 相同）',
              seq == [0, 1, 2, 3, 4], str(seq))
        check('用满后返回 None', next_idx(node, 'red') is None)
        # ⚠️ A1 的验收判据就是下面这句话，**逐字**。
        reason = node._place_exhausted_reason('red')
        check('**收工原因逐字等于 A1 那句**（A1 验收判据）',
              reason == f"放置槽位已用尽（{len(SLOTS_FLAT)//2} 个）"
                        f"—— 没有地方可放了",
              repr(reason))
    finally:
        shutdown(node)


def part_d_mutation():
    """D. 变异：把 pick_slot 换成「不看类别」的版本，B 段断言必须失效"""
    print('\n[D] 变异：节点真的走 slot_routing.pick_slot 吗')
    from grasp_planner import slot_routing

    def blind_pick(used, slot_classes, class_name):
        """把类别判断去掉 —— 等价于「路由表写了但没生效」。"""
        for i, _want in enumerate(slot_classes):
            if not used[i]:
                return i
        return None

    node = make_node(base_params(ROUTED), patch={'pick_slot': blind_pick})
    try:
        # 未占用时 yellow 本应拿 2 号；不看类别就会拿 0 号
        check('变异后 yellow 拿到的**不是** 2 号（说明接线没被旁路）',
              next_idx(node, 'yellow') != 2,
              '仍然是 2 号 —— 说明节点没在用 pick_slot，B 段测的另有其物')
        check('变异后 yellow 拿到 0 号（red 的格子）—— 正是要防的那种错',
              next_idx(node, 'yellow') == 0)
    finally:
        shutdown(node)
    # 补一句：确认「真」pick_slot 在同一输入下给的是 2 号，两次对照成立
    check('对照：真 pick_slot 在同一输入下给 2 号',
          slot_routing.pick_slot([False] * 5,
                                 ['red', 'red', 'yellow', 'yellow', ''],
                                 'yellow') == 2)


def part_a_startup_log():
    """A. 参数通路：命令行 → 声明 → 解析 → 启动日志（子进程）"""
    print('\n[A] 启动日志里的路由表（子进程，走 ros2 run）')
    if not os.path.exists(os.path.join(WS_INSTALL, "setup.bash")):
        check('grasp_planner 已构建', False,
              '缺 %s/setup.bash —— 先 colcon build --packages-select grasp_planner' % WS_INSTALL)
        return
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = DOMAIN
    src = ("source /opt/ros/humble/setup.bash && "
           f"source {os.path.join(WS_INSTALL, 'setup.bash')} && ")

    def run(*params, params_file=None):
        # shlex.quote：路由表里带引号的 "" 必须原样送到节点，
        # 不能被 shell 吃掉（吃了就退化成「末尾裸空项被静默丢掉」那个坑）。
        pf = " --params-file " + shlex.quote(params_file) if params_file else ""
        cmd = src + ("timeout 12 ros2 run grasp_planner planner_node "
                     "--ros-args" + pf + " -p gripper_backend:=none "
                     "-p carry_objects:=false "
                     + " ".join(shlex.quote(p) for p in params))
        r = subprocess.run(['bash', '-lc', cmd], env=env, text=True,
                           capture_output=True, timeout=60)
        return r.stdout + r.stderr

    out = run('-p', 'place_slots:=[%s]' % ",".join(map(str, SLOTS_FLAT)),
              '-p', 'place_slot_classes:=[%s]' % ROUTED)
    check('路由表逐槽打进启动日志',
          '0:(0.44,0.24) red' in out and '3:(0.60,-0.24) yellow' in out,
          [l for l in out.splitlines() if '类别路由' in l])

    out2 = run('-p', 'place_slots:=[%s]' % ",".join(map(str, SLOTS_FLAT)))
    check('不传路由时日志明说「关闭」，不留模糊',
          '类别路由：**关闭**' in out2,
          [l for l in out2.splitlines() if '类别路由' in l])

    out3 = run('-p', 'place_slots:=[%s]' % ",".join(map(str, SLOTS_FLAT)),
               '-p', 'place_slot_classes:=[red,yellow]')
    check('长度不符 → 拒绝启动（不是截断）',
          '必须等长' in out3, out3.splitlines()[-3:])

    # ⚠️ 这一条钉的是**写法陷阱本身**：末尾裸逗号会被 ROS 静默丢掉，
    # 于是「4 类 + 1 个通配」被读成「只有 4 个槽位」——长度不符会挡住它（上面
    # 那条），但如果槽位数正好是 4，它就会静默地变成另一种配置。
    out4 = run('-p', 'place_slots:=[%s]' % ",".join(map(str, SLOTS_FLAT)),
               '-p', 'place_slot_classes:=[red,red,yellow,yellow,]')
    check('末尾裸逗号被静默丢掉（所以必须写带引号的 ""）',
          '必须等长' in out4,
          '居然通过了 —— 说明解析器行为变了，那条注释要重写：'
          + str([l for l in out4.splitlines() if '类别路由' in l]))

    # ⚠️ 这一条走的是 launch **真正**走的路径：`--params-file grasp_planner.yaml`
    #    （见 pipeline.launch.py 的 planner_params）。yaml 里的
    #    `place_slot_classes: []` 与命令行那个空数组是**两条不同的解析路径**，
    #    上面四条命令行用例一条都覆盖不到它。
    yaml_path = os.path.join(REPO, "ros2_ws/src/grasp_planner/config",
                             "grasp_planner.yaml")
    if os.path.exists(yaml_path):
        out5 = run(params_file=yaml_path)
        check('随仓发布的 grasp_planner.yaml 能加载（空数组没撞类型）',
              '类别路由' in out5,
              '没打出路由那一行 —— 多半是 place_slot_classes: [] 的类型推断'
              '把节点弄死了：' + str(out5.splitlines()[-4:]))
        check('  且默认是「关闭」（A1 行为，老配置不用改）',
              '类别路由：**关闭**' in out5,
              [l for l in out5.splitlines() if '类别路由' in l])
        check('  yaml 里 place_slots 是 6 格',
              '放置槽位 6 个' in out5,
              [l for l in out5.splitlines() if '放置槽位' in l])
    else:
        check('找到随仓发布的 grasp_planner.yaml', False, yaml_path)


def main():
    ap = argparse.ArgumentParser(description='规划器按类别分拣自检')
    ap.add_argument('--only', choices=['A', 'B', 'D'], default=None)
    args = ap.parse_args()

    if not os.environ.get('ROS_DISTRO'):
        print('先 source /opt/ros/humble/setup.bash 与 ros2_ws/install/setup.bash'
              '（退 2）')
        return 2
    for p in (PLANNER_SRC, os.path.join(INSTALL, 'lib/python3.10/site-packages')):
        sys.path.insert(0, p)

    try:
        import rclpy  # noqa: F401
    except ImportError as e:
        print('导入 rclpy 失败：%s（退 2）' % e)
        return 2

    if args.only in (None, 'B'):
        part_b_routing()
        part_b2_unrouted()
    if args.only in (None, 'D'):
        part_d_mutation()
    if args.only in (None, 'A'):
        part_a_startup_log()

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

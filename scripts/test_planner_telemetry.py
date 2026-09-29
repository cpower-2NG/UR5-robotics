#!/usr/bin/env python3
"""遥测通道自检 —— **不需要仿真器、不需要 MoveIt、不需要 colcon build**。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么要有这个测试
==================
遥测是**只读**的，加错了不会让机械臂出问题 —— 恰恰因为如此，它错了也没人发现。
最典型的三种错法各自对应下面一段：

  A. **模块级**：直接建一个 ``PlannerTelemetry``，检查三个话题的类型与 QoS。
     重点是 **transient_local 真的生效了吗** —— 「晚加入的 RViz 也能看到最后
     一条」这个性质全靠它，而 QoS 写错时**不会有任何报错**，只会静默地收不到。

  B. **节点级**：起一个**真的** ``planner_node``（只走位配置，不碰夹爪硬件），
     检查三个话题真的挂到图上了 —— 证明 20 处接线没有漏（漏一处的症状是
     「RViz 里这个显示一直是空的」，很容易被当成 RViz 配置问题查半天）。

  C. **时序**：连发多次 ``set_state``，确认**状态没变时不重复发布**。
     ``_reset_busy`` 在闲着时会被 10 Hz 的检测流每批调一次；没有变化检测的话
     这个话题会变成每秒几十条的水龙头，把 bag 里真正的状态时间线彻底淹掉。

用法
----
    python3 scripts/test_planner_telemetry.py           # 全部三段
    python3 scripts/test_planner_telemetry.py --only A  # 只跑一段
"""

import argparse
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 三个遥测话题。改这里的话，telemetry.py 的模块文档表格也要跟着改。
TOPICS = {
    "target": ("/grasp_planner/current_target", "geometry_msgs/msg/PoseStamped"),
    "slots": ("/grasp_planner/place_slots", "visualization_msgs/msg/MarkerArray"),
    "telemetry": ("/grasp_planner/telemetry",
                  "diagnostic_msgs/msg/DiagnosticArray"),
}

_failures = []


def check(cond, label, detail=""):
    if cond:
        print(f"  ✓ {label}")
    else:
        print(f"  ✗ {label}" + (f"\n      {detail}" if detail else ""))
        _failures.append(label)
    return cond


def spin_for(executor, seconds):
    """在 ``seconds`` 内尽量多地转执行器（没消息就立刻返回，所以是真等满）。"""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


# ----------------------------------------------------------------------
# A. 模块级：类型 + QoS
# ----------------------------------------------------------------------
def part_a():
    print("\n===== A. 模块级：三个话题的类型与 QoS =====")
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSDurabilityPolicy,
                           QoSHistoryPolicy, QoSProfile, ReliabilityPolicy)

    from grasp_planner.telemetry import SLOT_SKIPPED, SLOT_USED, PlannerTelemetry

    rclpy.init()
    host = Node("telemetry_test_host")
    telem = PlannerTelemetry(
        host,
        base_frame="base_link",
        place_slots=[(0.4, 0.3), (0.5, 0.3), (0.6, 0.3)],
        place_z=-0.35,
        snapshot=lambda: {"holding": False, "attempts_since_place": 0},
    )
    ex = SingleThreadedExecutor()
    ex.add_node(host)
    spin_for(ex, 1.0)

    ok = True
    for key, (topic, want_type) in TOPICS.items():
        infos = host.get_publishers_info_by_topic(topic)
        # 断言的是「本段真的建出了发布者」；多出来的发布者属于**环境不干净**
        # （多半是上次没清掉的 planner_node），所以只提示，不算失败 ——
        # 把环境问题报成代码问题会让人去改根本没坏的代码。
        if not check(len(infos) >= 1, f"{topic} 有发布者", "一个都没有"):
            ok = False
            continue
        if len(infos) > 1:
            others = [i.node_name for i in infos if i.node_name != host.get_name()]
            print(f"      ⚠️ 该话题还有 {len(infos) - 1} 个别的发布者 {others} ——"
                  " 多半是残留的 planner_node（`pkill -f [p]lanner_node`）")
        info = infos[0]
        check(info.topic_type == want_type,
              f"{topic} 类型是 {want_type}", f"实际 {info.topic_type}")
        qos = info.qos_profile
        # transient_local：晚加入的订阅者也能拿到最后一条。这是「RViz 随时
        # 打开都有东西看」的全部依据，写错时**不会报错**，只是收不到。
        check(qos.durability == DurabilityPolicy.TRANSIENT_LOCAL,
              f"{topic} durability = TRANSIENT_LOCAL", f"实际 {qos.durability}")
        check(qos.reliability == ReliabilityPolicy.RELIABLE,
              f"{topic} reliability = RELIABLE", f"实际 {qos.reliability}")

    # ⚠️ **不**去断言 `qos.depth == 1`：本机（Fast DDS）的
    #    `get_publishers_info_by_topic()` 把 history 报成 UNKNOWN(3)、depth 报成
    #    0，即使实际配置就是 KEEP_LAST(1)/depth=1 —— 实测确认过，这是**自省字段
    #    的假象，不是配置错**。断言它只会得到一个假的失败。
    #    改测**行为**，那才是我们真正依赖的性质：晚加入的订阅者**只**拿到最后
    #    一条（「RViz 晚开 30 秒也只看得到当前状态」，而不是收到一长串历史）。
    # 顺序很关键：**先把三条发完并推动执行器**，**再**建订阅者。
    # ⚠️ 反过来写会得到一个假的通过/失败：订阅者若在发送时已经匹配上，那三条是
    #    作为**实时消息**投递给它的，进的是**订阅者自己**的 depth 队列（这里 10），
    #    跟发布者的 depth=1 无关。发布者的 depth 只约束「留给晚到者的历史」。
    telem.set_target(1.0, 1.0, 1.0)
    telem.set_target(2.0, 2.0, 2.0)
    telem.set_target(3.0, 3.0, 3.0)
    spin_for(ex, 0.5)

    got_t = []
    late = Node("telemetry_test_late")
    # ⚠️ 订阅方**必须**也声明 TRANSIENT_LOCAL，否则拿不到锁存的那条 ——
    #    这是本测试第一版踩到的坑，也是**所有消费者（含 RViz）都要注意的**：
    #    发布方 latched 只说明「它留着一条」，订阅方 VOLATILE 就只收实时消息，
    #    于是 RViz 里那个显示会一直空着，看起来像话题没发（见 telemetry.py）。
    late.create_subscription(
        PoseStamped, TOPICS["target"][0], lambda m: got_t.append(m),
        QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                   reliability=ReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    ex.add_node(late)
    # 等发现完成 —— 是**轮询**而不是固定等 1.5 s：发现延迟随机器负载变化，
    # 拍一个死值迟早会在慢机器上变成一个假的失败（本机就要 2 s 以上）。
    deadline = time.monotonic() + 8.0
    while not got_t and time.monotonic() < deadline:
        spin_for(ex, 0.2)
    spin_for(ex, 0.5)      # 再多转一会儿，好把「多给了几条」也暴露出来
    check(len(got_t) == 1 and abs(got_t[0].pose.position.x - 3.0) < 1e-6,
          "晚加入的订阅者只拿到最后一条目标（latched depth=1 的实际语义）",
          f"收到 {len(got_t)} 条，x={[round(m.pose.position.x, 2) for m in got_t]}")
    late.destroy_node()

    # 槽位标记：3 个槽 → 1 条 DELETEALL + 3 块板 + 3 个序号 = 7 条
    from visualization_msgs.msg import MarkerArray
    got = []
    host.create_subscription(MarkerArray, TOPICS["slots"][0],
                             lambda m: got.append(m), 10)
    telem.publish_slots()
    spin_for(ex, 0.5)
    if check(len(got) >= 1, "place_slots 能收到 MarkerArray"):
        arr = got[-1]
        check(len(arr.markers) == 7,
              "槽位标记数 = 1(DELETEALL) + 3 板 + 3 序号 = 7",
              f"实际 {len(arr.markers)}")

    # mark_slot 的越界保护：画图不该把状态机带崩
    telem.mark_slot(99, SLOT_USED)
    check(True, "mark_slot 越界不抛异常（只告警）")

    host.destroy_node()
    rclpy.shutdown()
    return ok


# ----------------------------------------------------------------------
# C. 时序：状态变化检测
# ----------------------------------------------------------------------
def part_c():
    print("\n===== C. 时序：状态没变时不重复发布 =====")
    before = len(_failures)      # ⚠️ `_failures` 是模块级累积的，不记基线的话
    import rclpy                 #    本段会把 A 段的失败算成自己的失败
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    from grasp_planner.telemetry import PlannerTelemetry

    rclpy.init()
    host = Node("telemetry_test_seq")
    telem = PlannerTelemetry(
        host, base_frame="base_link", place_slots=[(0.4, 0.3)], place_z=-0.35,
        snapshot=lambda: {"holding": False},
    )
    got = []
    host.create_subscription(DiagnosticArray, TOPICS["telemetry"][0],
                             lambda m: got.append(m), 10)
    ex = SingleThreadedExecutor()
    ex.add_node(host)
    spin_for(ex, 0.5)
    got.clear()

    # 模拟 _reset_busy 在闲时被 10 Hz 检测流每批调一次
    for _ in range(20):
        telem.set_state("IDLE")
    spin_for(ex, 0.5)
    check(len(got) == 1,
          "重复 set_state('IDLE') 20 次只发 1 条（变化检测生效）",
          f"实际发了 {len(got)} 条 —— 没有变化检测就会是 20 条")

    got.clear()
    telem.set_state("APPROACH")
    telem.set_state("DESCEND")
    spin_for(ex, 0.5)
    check(len(got) == 2, "状态真的变了就发（APPROACH→DESCEND = 2 条）",
          f"实际 {len(got)} 条")

    # 收工：原因必须进消息体，级别必须能区分正常/异常
    from diagnostic_msgs.msg import DiagnosticStatus
    got.clear()
    telem.note_finished("槽位已用尽", ok=False)
    spin_for(ex, 0.5)
    if check(len(got) == 1, "note_finished 发了一条"):
        st = got[-1].status[0]
        vals = {kv.key: kv.value for kv in st.values}
        check(vals.get("finish_reason") == "槽位已用尽",
              "finish_reason 进了 values（事后不必翻 /rosout 文本）",
              f"实际 {vals.get('finish_reason')!r}")
        check(st.level == DiagnosticStatus.ERROR,
              "ok=False → level=ERROR", f"实际 {st.level}")

    host.destroy_node()
    rclpy.shutdown()
    return len(_failures) == before


# ----------------------------------------------------------------------
# B. 节点级：真的 planner_node 挂上三个话题
# ----------------------------------------------------------------------
def part_b():
    print("\n===== B. 节点级：起真的 planner_node，看三个话题在不在图上 =====")
    before = len(_failures)
    # 只走位配置：不碰夹爪后端，也不需要 MoveIt 服务在（客户端只是连不上而已）。
    # 这样这个自检在**任何**机器上都能跑，不依赖仿真器。
    env = dict(os.environ)
    for setup in ("/opt/ros/humble/setup.bash",
                  os.path.join(REPO, "ros2_ws/install/setup.bash")):
        if not os.path.exists(setup):
            print(f"  ! 缺 {setup}")
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"source {REPO}/ros2_ws/install/setup.bash && "
        "ros2 run grasp_planner planner_node --ros-args "
        "-p gripper_backend:=none -p carry_objects:=false"
    )
    proc = subprocess.Popen(["bash", "-lc", cmd], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        rclpy.init()
        watch = Node("telemetry_test_watch")
        ex = SingleThreadedExecutor()
        ex.add_node(watch)

        # 等节点起来并挂上发布者（最多 20 s）
        deadline = time.monotonic() + 20.0
        seen = set()
        while time.monotonic() < deadline:
            spin_for(ex, 0.4)
            # get_topic_names_and_types() 返回的是 (名字, 类型列表) 的**元组**列表
            seen = {name for name, _types in watch.get_topic_names_and_types()
                    if name.startswith("/grasp_planner/")}
            if len(seen) >= 3:
                break
        for _, (topic, _t) in TOPICS.items():
            check(topic in seen, f"{topic} 出现在 ROS 图上")

        watch.destroy_node()
        rclpy.shutdown()
    finally:
        import signal
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=15)
        except Exception:                          # noqa: BLE001
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:                      # noqa: BLE001
                pass
    return len(_failures) == before


def main():
    ap = argparse.ArgumentParser(description="遥测通道自检")
    ap.add_argument("--only", choices=["A", "B", "C"], default=None)
    args = ap.parse_args()

    if not os.environ.get("ROS_DISTRO"):
        print("先 source /opt/ros/humble/setup.bash 与 "
              "ros2_ws/install/setup.bash（退 2）")
        return 2
    if args.only in (None, "A", "B"):
        sys.path.insert(0, os.path.join(REPO, "ros2_ws/src/grasp_planner"))
        sys.path.insert(0, os.path.join(REPO, "ros2_ws/install/grasp_planner/lib/python3.10/site-packages"))

    ran = []
    if args.only in (None, "A"):
        ran.append(("A", part_a()))
    if args.only in (None, "C"):
        ran.append(("C", part_c()))
    if args.only in (None, "B"):
        ran.append(("B", part_b()))

    print("\n===== 结果 =====")
    for name, ok in ran:
        print(f"  {name}: {'通过' if ok else '**失败**'}")
    if _failures:
        print(f"\n失败 {len(_failures)} 项：")
        for f in _failures:
            print(f"  · {f}")
        return 1
    print("全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

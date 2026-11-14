#!/usr/bin/env python3
"""执行计划 JSON v2：**每份计划起手查一次闸门**，夹爪动作与臂动作分开记日志。

与 ``run_grasp.py`` 的关系
-------------------------
闸门、两点式 JTC、结果同时查 status 与 error_code —— 全部沿用，一字没改。
新增的只有两件事：

1. **夹爪动作**（``before`` / ``after``）穿插在航点之间，用
   ``grasp_planner.gripper.Ag95Gripper``（公开接口 open/close/current_position/
   is_empty）。写寄存器前它会自己查 ``0x0200 == 1``（初始化完成），没初始化就拒绝。
2. **JSONL 日志**：每个航点、每次夹爪动作、每次闸门都落一行，验收报告直接读它。
   日志与计划放同一个目录（仓库外），不污染仓库。

默认**预演**：不加 --go 一条轨迹都不发。

闸门（任一不过就原地停，不发这一份计划；2026-11-12 起每份计划只查一次）
--------------------------------------------------------------------
  · robot_mode == 7 (RUNNING)、safety_mode == 1 (NORMAL)
  · 示教器程序 programState 以 PLAYING 开头
    —— STOPPED 时 external control 的 URScript 不读 servoJ，JTC 照收目标但臂不动
  · scaled_joint_trajectory_controller == active
    —— controller_stopper 在程序停止时会把它停掉；action server 存在 ≠ 已激活
  · 当前关节角与**闸门基准**（计划起点 / 上一份计划的末航点）最大差 < 0.05 rad
  · 结果 status == SUCCEEDED **且** error_code == 0

用法：
  python3 precision_exec.py --plan pick.json --list                 # 只打印
  python3 precision_exec.py --plan pick.json --check                # 只查闸门
  python3 precision_exec.py --plan pick.json --go --upto lift       # 真动
"""
import argparse
import glob
import json
import math
import os
import sys
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripper_action as ga                                       # noqa: E402
import real_motion_authorize as rma                               # noqa: E402


def resolve_port(explicit=None):
    """串口：显式参数 > 环境变量 ``AG95_PORT`` > ``/dev/serial/by-id`` 自动选。

    不写死 by-id 名字：那串是转接器的**硬件序列号**，公开仓不许出现
    （AGENTS §1 红线 3），换一根转接线也会失效。``/dev/ttyUSB*`` 插拔会变号，
    ``by-id`` 名字不会。
    """
    if explicit:
        return explicit
    env = os.environ.get("AG95_PORT")
    if env:
        return env
    hits = (sorted(glob.glob("/dev/serial/by-id/usb-FTDI_*-if00-port0"))
            or sorted(glob.glob("/dev/serial/by-id/usb-FTDI_*")))
    if len(hits) == 1:
        return hits[0]
    if not hits:
        raise SystemExit("找不到 FTDI 串口（/dev/serial/by-id/usb-FTDI_* 无匹配）"
                         "—— 插好转接器，或用 AG95_PORT / --gripper-port 指定")
    raise SystemExit("有多根 FTDI 串口，无法自动选：%s —— 用 AG95_PORT / --gripper-port 指定"
                     % "  ".join(hits))

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TRAJ_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
ROBOT_MODE_RUNNING = 7
SAFETY_MODE_NORMAL = 1
JOINT_DRIFT_RAD = 0.05
# 空闭合实测回读 0.0，真夹到是 24.9–26.2 mm（2026-10-25 三轮），5 mm 容差分得开。
# 注意回读**不是**爪距：空载全开 95.0 / 全闭 0.0（两端都对），但夹住 31.5–32.3 mm
# 的块稳定读 24.9–25.3 mm —— 偏差只长在接触段，只能当**相对判据**（没滑）用。
AG95_EMPTY_TOL_MM = 5.0

# ---------------------------------------------------------------- 闸门的等待预算
#: 闸门里每个 ROS 服务**等服务出现**的上限（秒）。本机/DDS 正常时是毫秒级。
GATE_SERVICE_WAIT_S = 2.0
#: 闸门里等**应答**的上限（秒）。Dashboard 偶尔整次不应答（2026-10-27 实测一次
#: 8 s），所以保留重试；但单次上限要短 —— 旧值「5 s 等出现 + 8 s 等应答 ×3 次」
#: 让一次闸门最坏 42 s，而它原先还是**每个航点**都跑一遍（147.7 s 那次事故）。
GATE_REPLY_WAIT_S = 3.0
#: 每个查询最多试几次。2 次 = 第一次可能撞上瞬断，第二次拿不到就是真红。
GATE_ATTEMPTS = 2
#: 两次尝试之间的间隔（秒）。
GATE_RETRY_SLEEP_S = 0.5


def grasp_verdict(pos):
    """闭合后的回读 → ``gripped`` / ``empty`` / ``unknown``。

    0 mm 附近 = 手指一路合到头（指间没东西）；读不到 = 判不了。**两种都不许
    再抬升/搬运**：2026-11-09 s3 空夹后照样把空手搬完 lift/to_slot/place 一整圈。
    """
    if pos is None:
        return "unknown"
    if pos <= AG95_EMPTY_TOL_MM:
        return "empty"
    return "gripped"


def close_reaction(pos):
    """闭合回读 → ``(verdict, 人话, 是否就地停)``。

    「就地停」这一条单独成函数，是为了能被离线自检真正覆盖到 —— 只测
    ``grasp_verdict`` 的阈值方向，测不到「空夹之后还接着搬运」那个行为。
    """
    verdict = grasp_verdict(pos)
    if verdict == "unknown":
        return verdict, "闭合后读不到位置 —— 判不了夹没夹住，就地停", True
    if verdict == "empty":
        return (verdict,
                "闭合后回读 %.1f mm ≈ 全闭 —— **空夹**，不再抬升/搬运" % pos,
                True)
    return verdict, "", False


GRIP_OPEN_MM = 95.0
GRIP_CLOSE_MM = 0.0


def log_line(path, **rec):
    rec["t"] = round(time.time(), 3)
    with open(path, "a") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def _r2(v):
    return None if v is None else round(float(v), 2)


def _action_name(spec):
    """夹爪动作名（``open``/``close``/``readback``/``open_to``）。"""
    if isinstance(spec, dict):
        return spec.get("gripper")
    return spec


def _moves_fingers(spec):
    """这个夹爪动作会不会真的动手指（``readback`` 只读，不算）。"""
    return _action_name(spec) in ("open", "close", "open_to")


def validate_plan_doc(plan):
    """计划 JSON v2 的**结构**校验（离线可测；发轨迹前的最后一道线）。

    2026-11-14 审计：原来不查 version、不查 q 长度 —— v1 残留或截断的计划
    会静默进 ``send()``，靠 JTC 拒 goal 兜底（运行时才失败，且已经过了
    闸门/自检）。计划期就该拒。查的不变量：version=2、q_start 非空数字表、
    航点名唯一、每点 q 长度与 q_start 一致、duration 为正数。
    """
    if not isinstance(plan, dict):
        raise ValueError("计划不是 JSON 对象")
    if plan.get("version") != 2:
        raise ValueError("version=%r ≠ 2 —— 只认计划 JSON v2"
                         % (plan.get("version"),))
    q_start = plan.get("q_start")
    if not isinstance(q_start, list) or not q_start \
            or not all(isinstance(v, (int, float)) for v in q_start):
        raise ValueError("q_start 必须是非空数字列表：%r" % (q_start,))
    wps = plan.get("waypoints")
    if not isinstance(wps, list) or not wps:
        raise ValueError("waypoints 为空/缺失")
    seen = set()
    for w in wps:
        if not isinstance(w, dict):
            raise ValueError("航点不是对象：%r" % (w,))
        name = w.get("name")
        if not name or name in seen:
            raise ValueError("航点名缺失或重复：%r" % (name,))
        seen.add(name)
        q = w.get("q")
        if not isinstance(q, list) or len(q) != len(q_start):
            got = ("缺" if q is None else
                   (len(q) if isinstance(q, list) else "非列表"))
            raise ValueError("航点 %s 的 q 长度 %s ≠ q_start 长度 %d"
                             % (name, got, len(q_start)))
        dur = w.get("duration")
        if not isinstance(dur, (int, float)) or isinstance(dur, bool) \
                or dur <= 0:
            raise ValueError("航点 %s 的 duration 非法：%r" % (name, dur))


def _action_str(spec):
    """把夹爪动作 spec（字符串或 ``{"gripper":..,"mm":..,"speed":..}``）变成一行字。"""
    if isinstance(spec, dict):
        mm = spec.get("mm")
        sp = spec.get("speed")
        return str(spec.get("gripper")) + (
            "" if mm is None else "(%.1f)" % float(mm)) + (
            "" if sp is None else " @%d%%" % int(sp))
    return str(spec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True, nargs="+",
                    help="计划 JSON，可给**多份**：按顺序在**同一个进程**里跑完"
                         "（一个 rclpy 节点、一根夹爪串口、一次自检）。"
                         "任何一份失败就停，后面的不跑（fail-closed）")
    ap.add_argument("--go", action="store_true", help="**真的发轨迹**")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--check", action="store_true", help="只查闸门，不发轨迹")
    ap.add_argument("--only", default=None, help="只跑这一条航点")
    ap.add_argument("--upto", default=None, help="跑到这一条为止（含）")
    ap.add_argument("--from", dest="from_", default=None,
                    help="从这一条开始跑（含）。配合 --upto 可把一段循环切开，"
                         "中间插一次抓帧（例如抓住块之后、放下之前看它夹在哪）")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="**带动手指**的航点完成后等几秒（抓/放前的静定："
                         "指尖与块之间不能再有残余摆动）")
    ap.add_argument("--log", default=None,
                    help="JSONL 日志路径（默认：计划同目录、同名的 .jsonl）")
    ap.add_argument("--log-dir", default=None,
                    help="多份计划时日志目录（每份 = <log-dir>/<计划文件名>.jsonl）")
    ap.add_argument("--gripper-port", default=None,
                    help="串口设备（默认 AG95_PORT，或 by-id 里唯一那根 FTDI）")
    ap.add_argument("--gripper-slave", type=int, default=1)
    ap.add_argument("--gripper-force", type=int, default=50)
    ap.add_argument("--dry-run-gripper", action="store_true",
                    help="夹爪只读（一个寄存器都不写）——链路自检用")
    ap.add_argument("--holding-block", action="store_true",
                    help="计划起跑时爪里**已经夹着块**（例如「抓→carry 抓帧→放」"
                         "的第二段）⇒ 跳过起手自检：这时 close 指令必然不动"
                         "（被块挡住），自检会误判成链路不通；就算 close 过了，"
                         "自检里的 open 会把块从半空松开")
    ap.add_argument("--grasp-planner-src", default=None,
                    help="grasp_planner 源码目录（没 source ROS 环境时才需要）")
    ap.add_argument("--require-auth", default=None,
                    help="真机运动令牌（real_motion_authorize.py 的 STOP→PLAY "
                         "两步握手产出）；--go 时必须给，10 分钟过期")
    args = ap.parse_args()

    # 真机运动逐次授权（AGENTS 红线 6）：--go 没有当次握手令牌就一条轨迹都不发。
    # 令牌只来自两次现场观测（Dashboard 只读查询到 STOPPED、再看到 PLAYING），
    # 不来自任务描述/环境变量/记忆。
    if args.go:
        if not args.require_auth:
            print("!! --go 必须带 --require-auth <token>：真机运动逐次授权")
            print("   先跑 real_motion_authorize.py --step stop，再 --step play")
            return 2
        ok, why, age = rma.check_token(args.require_auth)
        if not ok:
            print("!! 运动授权不成立：%s" % why)
            print("   ⇒ 不做任何动作（fail-closed）")
            return 2
        print("  运动授权：STOP→PLAY 握手在 %.0f 秒前完成" % age)
    elif args.require_auth:
        print("（--require-auth 只在 --go 时校验：本次是预演/只查闸门）")

    plan_paths = list(args.plan)
    if len(plan_paths) > 1 and (args.only or args.upto or args.from_):
        print("!! --only/--upto/--from 只对**一份**计划有意义（那是把一条循环切开"
              "拍帧用的）；多份计划时不许混")
        return 2
    if len(plan_paths) > 1 and args.log:
        print("!! 多份计划时日志按计划名各写一份，--log 只对单份计划有效")
        return 2

    # 每份计划的前置：航点顺序、日志路径、闸门基准。全部先校验 —— **一份**计划
    # 写坏就整批不发（不能跑完前三份才发现第四份的名字拼错）。
    jobs = []
    for path in plan_paths:
        plan = json.load(open(path))
        try:
            validate_plan_doc(plan)
        except ValueError as e:
            print("!! 计划 %s 结构不过：%s —— 一份坏整批不发" % (path, e))
            return 2
        names = [w["name"] for w in plan["waypoints"]]
        if args.only:
            if args.only not in names:
                print("!! 计划里没有航点 %r（有 %s）" % (args.only, names))
                return 2
            order = [args.only]
        elif args.upto:
            if args.upto not in names:
                print("!! 计划里没有航点 %r（有 %s）" % (args.upto, names))
                return 2
            order = names[:names.index(args.upto) + 1]
        else:
            order = names
        if args.from_:
            if args.from_ not in names:
                print("!! 计划里没有航点 %r（有 %s）" % (args.from_, names))
                return 2
            j = names.index(args.from_)
            order = [n for n in order if names.index(n) >= j]
            if not order:
                print("!! --from %s 在 --upto %s 之后，没有航点可跑"
                      % (args.from_, args.upto))
                return 2
        if args.log_dir:
            # 与单计划时代的 ``--log <dir>/<name>.jsonl`` 同名（name 自带 .json），
            # 编排器按同一个名字回拉证据。
            log = os.path.join(args.log_dir, os.path.basename(path) + ".jsonl")
        else:
            log = args.log or os.path.splitext(path)[0] + ".jsonl"
        jobs.append({
            "path": path,
            "plan": plan,
            "names": names,
            "order": order,
            "wr": {w["name"]: w for w in plan["waypoints"]},
            "meta": plan.get("meta", {}),
            "log": log,
        })

    for i, job in enumerate(jobs):
        plan, names, order, wr, meta = (job["plan"], job["names"],
                                        job["order"], job["wr"], job["meta"])
        print("=" * 78)
        kind = meta.get("kind", "?")
        extra = ""
        if kind == "pick":
            extra = (" 块 #%s %s  抓点 (%.1f, %.1f) mm  槽 %s"
                     % (meta.get("block_idx"), meta.get("class"),
                        *[v * 1000 for v in meta.get("grasp_point", [0, 0])],
                        meta.get("slot")))
        print("计划 %d/%d %s  kind=%s%s"
              % (i + 1, len(jobs), job["path"], kind, extra))
        for n in names:
            w = wr[n]
            act = []
            if w.get("before"):
                act.append("前:" + _action_str(w["before"]))
            if w.get("after"):
                act.append("后:" + _action_str(w["after"]))
            print("  %-15s q(deg) [%s]  %4.1fs  %s"
                  % (n, " ".join("%6.1f" % math.degrees(v) for v in w["q"]),
                     w["duration"], " ".join(act)))
        print("本次要跑：%s" % order)
    print("=" * 78)
    if args.list or (not args.go and not args.check):
        print("⇒ 预演：一条轨迹都没发。真动加 --go。")
        return 0

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode
    from ur_dashboard_msgs.srv import RawRequest
    from controller_manager_msgs.srv import ListControllers
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from action_msgs.msg import GoalStatus

    rclpy.init()
    node = Node("precision_exec")
    got = {}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    # 三个状态话题是 TRANSIENT_LOCAL：只在变化时发，靠 latched 补发给后加入者。
    qos_l = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(JointState, "/joint_states",
                             lambda m: got.__setitem__("js", m), qos)
    node.create_subscription(RobotMode, "/io_and_status_controller/robot_mode",
                             lambda m: got.__setitem__("rm", m), qos_l)
    node.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                             lambda m: got.__setitem__("sm", m), qos_l)
    node.create_subscription(Bool, "/io_and_status_controller/robot_program_running",
                             lambda m: got.__setitem__("pr", m), qos_l)
    t0 = time.time()
    # robot_program_running 只在**状态变化时**发，latch 缓存可能是空的 —— 别为它空等
    # 满 20 s（run_grasp.py 就是这么等的，白等）。核心三件到齐就往下走。
    # 2026-11-12：拿不到状态的等待上限 20 s→5 s。三件都是 TRANSIENT_LOCAL，
    # 驱动在就是在（毫秒级），不在就是不在 —— 多等 15 s 不产生信息，只让
    # 「栈没起来」这个失败晚 15 s 才报出来。
    while any(k not in got for k in ("js", "rm", "sm")) and time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.2)
    for _ in range(5):                      # 再给 pr 半秒机会，仅作附加信息
        rclpy.spin_once(node, timeout_sec=0.1)
    miss = [k for k in ("js", "rm", "sm") if k not in got]
    if miss:
        print("!! 5 s 没收齐状态 %s —— 驱动起了吗？" % miss)
        return 2

    def joints():
        d = dict(zip(got["js"].name, got["js"].position))
        return [float(d[j]) for j in ARM]

    ctl = node.create_client(ListControllers, "/controller_manager/list_controllers")
    raw = node.create_client(RawRequest, "/dashboard_client/raw_request")

    def controller_state(name):
        """查控制器状态。**一次不应答不等于控制器没激活** —— 重试几次再判。

        2026-10-31 实测：示教器刚按 PLAY 的那几秒，`controller_stopper` 正在做
        STRICT 切换，`/controller_manager/list_controllers` 有 8 s 不应答，
        闸门拿到 None 就把整轮拒了（再查 JTC 是 active）。重试**不放宽**判据：
        都拿不到，仍然返回 None、仍然判红（fail-closed）。

        2026-11-12：单次等待从 3+5 s 压到 ``GATE_*`` 预算 —— 重试次数 3→2，
        最坏耗时 27 s→11 s。多等的那几次对「真断」没有信息量，只是让整轮更慢。
        """
        for _ in range(GATE_ATTEMPTS):
            if not ctl.wait_for_service(timeout_sec=GATE_SERVICE_WAIT_S):
                time.sleep(GATE_RETRY_SLEEP_S)
                continue
            f = ctl.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, f, timeout_sec=GATE_REPLY_WAIT_S)
            r = f.result()
            if r is not None:
                state = next((c.state for c in r.controller if c.name == name), None)
                if state is not None:
                    return state
            time.sleep(GATE_RETRY_SLEEP_S)
        return None

    def program_state():
        """查示教器程序状态。**一次不应答不等于没在跑** —— 重试几次再判。

        2026-10-27 round-08 实测：`/dashboard_client/raw_request` 有一次 8 s
        没回（Dashboard 服务端偶发不应答），闸门拿到 `None` 就把整轮打成
        「程序=None → 拒发」，而示教器上程序一直是 PLAYING。重试**不放宽**判据：
        都拿不到，仍然返回 None、仍然判红（fail-closed）。

        2026-11-12：单次等待从 5+8 s 压到 ``GATE_*`` 预算（原因同上）。
        """
        for _ in range(GATE_ATTEMPTS):
            if not raw.wait_for_service(timeout_sec=GATE_SERVICE_WAIT_S):
                return None
            req = RawRequest.Request()
            req.query = "programState"
            f = raw.call_async(req)
            rclpy.spin_until_future_complete(node, f, timeout_sec=GATE_REPLY_WAIT_S)
            r = f.result()
            if r is not None and str(r.answer).strip():
                return r.answer
            time.sleep(GATE_RETRY_SLEEP_S)
        return None

    # 逐计划可变的运行态（多份计划共用一个进程，但日志/闸门基准/自检各算各的）。
    # 这里用 dict 而不是 ``nonlocal`` 变量：下面那些闭包（gate/send/do_gripper）
    # 只**读**它，改值的地方只有 ``run_plan`` 一处。
    st = {"log": None, "expected": [], "selftest_done": False}

    def gate(where):
        cur = joints()
        rm, sm = int(got["rm"].mode), int(got["sm"].mode)
        drift = max(abs(a - b) for a, b in zip(cur, st["expected"]))
        cstate = controller_state("scaled_joint_trajectory_controller")
        pstate = program_state()
        playing = bool(pstate) and pstate.strip().upper().startswith("PLAYING")
        ok = (rm == ROBOT_MODE_RUNNING and sm == SAFETY_MODE_NORMAL
              and drift < JOINT_DRIFT_RAD and cstate == "active" and playing)
        print("  [%s] robot_mode=%d safety_mode=%d 程序=%r JTC=%s 最大差 %.2f° → %s"
              % (where, rm, sm, pstate, cstate, math.degrees(drift),
                 "OK" if ok else "**拒绝**"))
        if not playing:
            print("    ↑ 示教器上程序没在跑：external control 的 URScript 不读 servoJ，"
                  "发轨迹也不会动")
        if cstate != "active":
            print("    ↑ JTC 没激活：controller_stopper 在程序停止时把它停掉了")
        log_line(st["log"], event="gate", where=where, robot_mode=rm, safety_mode=sm,
                 program=pstate, jtc=cstate, drift_deg=round(math.degrees(drift), 3),
                 ok=bool(ok))
        return ok, cur

    # 起手快速闸门：连 ActionClient 之前先看一眼（基准 = 第一份计划）。
    st["log"] = jobs[0]["log"]
    _n0 = jobs[0]["names"]
    _i0 = _n0.index(jobs[0]["order"][0])
    st["expected"] = list(jobs[0]["plan"]["q_start"] if _i0 == 0
                          else jobs[0]["wr"][_n0[_i0 - 1]]["q"])
    ok, cur = gate("发轨迹前")
    if not ok:
        print("!! 闸门不过，一条轨迹都不发")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    ac = ActionClient(node, FollowJointTrajectory, TRAJ_ACTION)
    if args.check:
        print("  JTC action 在" if ac.wait_for_server(timeout_sec=5.0)
              else "  !! JTC action 不在")
        node.destroy_node()
        rclpy.shutdown()
        return 0
    if not ac.wait_for_server(timeout_sec=10.0):
        print("!! %s 不在" % TRAJ_ACTION)
        return 2

    # ---------------------------------------------------------------- 夹爪
    grip = {"obj": None}

    def gripper():
        if grip["obj"] is not None:
            return grip["obj"]
        if args.grasp_planner_src:
            sys.path.insert(0, os.path.abspath(args.grasp_planner_src))
        try:
            from grasp_planner.gripper import Ag95Gripper
        except ImportError as e:
            print("!! 导不进 grasp_planner.gripper（%s）—— source 过 ROS 环境吗？"
                  " 也可以给 --grasp-planner-src" % e)
            raise
        g = Ag95Gripper(node, port=resolve_port(args.gripper_port), baudrate=115200,
                        slave_id=args.gripper_slave, force=args.gripper_force,
                        stroke_mm=GRIP_OPEN_MM, open_mm=GRIP_OPEN_MM,
                        close_mm=GRIP_CLOSE_MM,
                        dry_run=bool(args.dry_run_gripper), open_is_high=True)
        print("  夹爪：%s" % g.describe())
        # 手指被块挡住时「走到目标」永远不成立 —— 打开卡住提前返回。
        # 安全理由：本脚本每个夹爪动作后都接 ``wait_stable()`` 复核最终位置
        # （``do_gripper`` 的 ``check_moved``），早返回不会让「没夹住」被判成
        # 「夹住」；规划器状态机不打开这个开关（它依赖「返回时手指已停」）。
        g.set_stall_detect(True)
        log_line(st["log"], event="gripper_init", desc=g.describe(),
                 dry_run=bool(args.dry_run_gripper))
        grip["obj"] = g
        return g

    def wait_stable(g, timeout=8.0, stable=0.6):
        """轮询回读，直到连续 stable 秒不变（或超时）。返回最后读数（毫米）。"""
        t0 = time.time()
        last, last_change = None, time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.0)
            p = g.current_position()
            now = time.time()
            if p is not None and (last is None or abs(p - last) > 0.5):
                last_change = now
            if p is not None:
                last = p
            if last is not None and now - last_change >= stable:
                return last
            time.sleep(0.1)
        return last

    def do_gripper(spec, where):
        """发一个夹爪动作并**证明它真的发生了**。

        2026-10-26 的教训：``Ag95Gripper.close()`` 不给 ``done_cb`` 是**空操作**
        （驱动 ``_async()`` 第一节就 return），而回读照样是 95.0 mm，日志看起来
        和成功一样。所以这里两件事缺一不可：走 ``ga.call_sync``（带回调），
        并且写完以后比对回读（``ga.check_moved``）。任一条不成立就抛
        :class:`ga.GripperActionError`，让循环停在原地。

        ``spec`` 可以是字符串 ``open/close/readback``，也可以是
        ``{"gripper":"open_to", "mm":50}``（阶段 2 半开）。
        """
        if isinstance(spec, dict):
            kind = spec.get("gripper")
            open_mm = spec.get("mm")
            speed = spec.get("speed")
        else:
            kind, open_mm, speed = spec, None, None
        g = gripper()
        t0 = time.time()
        before = g.current_position()
        # 速度是**独立于动作**的一个寄存器（0x0104），写它本身不动手指，
        # 但决定这次动多快。所以先设速度，再发动作。
        if speed is not None:
            try:
                g.set_speed(speed)
            except (RuntimeError, ValueError) as exc:
                print("  [%s] 夹爪设速度 %r **失败**：%s" % (where, speed, exc))
                log_line(st["log"], event="gripper_fail", where=where,
                         action="set_speed", speed=speed, error=str(exc))
                raise ga.GripperActionError("%s.set_speed：%s" % (where, exc))
        if kind in ("open", "close"):
            try:
                ga.call_sync(g, kind)
            except ga.GripperActionError as exc:
                print("  [%s] 夹爪 %s **失败**：%s" % (where, kind, exc))
                log_line(st["log"], event="gripper_fail", where=where, action=kind,
                         error=str(exc), before_mm=_r2(before))
                raise
        elif kind == "open_to":
            if open_mm is None:
                raise ga.GripperActionError("%s：open_to 缺 mm" % where)
            try:
                ga.call_move_sync(g, float(open_mm))
            except ga.GripperActionError as exc:
                print("  [%s] 夹爪 open_to(%.1f) **失败**：%s"
                      % (where, float(open_mm), exc))
                log_line(st["log"], event="gripper_fail", where=where,
                         action=kind, target_mm=float(open_mm),
                         error=str(exc), before_mm=_r2(before))
                raise
        elif kind != "readback":
            # 2026-11-14 审计补洞：原来只打印 + return None ⇒ 调用方把 None
            # 当「没做夹爪动作」继续发轨迹 —— after 拼错（如 "cloes"）就跳过
            # 空夹检测，空手继续 lift/to_slot/place（2026-11-09 s3 空夹事故的
            # 另一条入口）。改判红停轮：动作名只认 open/close/open_to/readback。
            log_line(st["log"], event="gripper_fail", where=where,
                     action=str(kind), error="unknown gripper action")
            raise ga.GripperActionError(
                "%s：不认识的夹爪动作 %r（只认 open/close/open_to/readback）"
                % (where, kind))
        pos = wait_stable(g)
        dt = time.time() - t0
        target = (GRIP_OPEN_MM if kind == "open" else
                  GRIP_CLOSE_MM if kind == "close" else
                  float(open_mm) if open_mm is not None else None)
        moved_ok, note = (True, "只读")
        if kind in ("open", "close"):
            moved_ok, note = ga.check_moved(before, pos, kind, target)
        elif kind == "open_to":
            moved_ok, note = ga.check_moved_to(before, pos, target)
        print("  [%s] 夹爪 %s%s → 回读 %s mm（%.1f s）%s"
              % (where, kind,
                 "" if open_mm is None else "(%.1f)" % float(open_mm),
                 "None" if pos is None else "%.1f" % pos, dt,
                 "  ⚠️ 读不到" if pos is None else "  " + note
                 + ("" if moved_ok else "  **判红**")))
        log_line(st["log"], event="gripper", where=where, action=kind,
                 position_mm=_r2(pos), before_mm=_r2(before),
                 target_mm=target if kind in ("open", "close", "open_to") else None,
                 moved=bool(moved_ok), note=note, duration_s=round(dt, 2))
        if not moved_ok:
            raise ga.GripperActionError("%s.%s：%s" % (where, kind, note))
        return pos

    def gripper_selftest():
        """动臂**之前**先证明夹爪链路是通的：空夹闭一次、再张开。

        为什么值得多花两秒：2026-10-26 第一个循环里，夹爪四段动作全是空操作，
        臂却完整走了一趟（下探→抬→槽位→张开→撤退），白跑 21 s 还差点把
        「夹爪没夹」记成「抓取失败」。空夹闭合在巡检位是安全的（指尖离台面
        还有 160 mm，两指之间没有东西），代价只有一次全行程。
        """
        print("  夹爪自检：空夹闭合 → 张开（指尖此时悬空）")
        do_gripper("close", "selftest.close")
        do_gripper("open", "selftest.open")
        print("  夹爪自检通过：写下去的位置指令确实让手指走了。")
        log_line(st["log"], event="gripper_selftest", ok=True)

    # ---------------------------------------------------------------- 轨迹
    def send(target, dur, what, settle_s=None):
        """发一条轨迹并等它完成；``settle_s`` 只在该航点要动手指时 > 0。

        2026-11-12：原来每条轨迹后都 ``sleep(--settle)`` —— 一份抓放计划 8 个
        航点 ⇒ 12 s 纯睡，5 个目标 60 s。静定的意义是**指尖动之前**臂不能还在
        余振，所以只有带 ``open``/``close``/``open_to`` 的航点需要等；纯位移
        航点直接读关节（``readback`` 也不必等，``wait_stable`` 自己会稳住）。
        到位误差照旧每段记，只是不再靠 sleep 换它。
        """
        if settle_s is None:
            settle_s = args.settle
        goal = FollowJointTrajectory.Goal()
        tj = goal.trajectory
        tj.joint_names = list(ARM)
        now = joints()
        # t=0 起点：共享实现（traj_util，D3 —— 与 sim_bridge 同源，2026-11-14）
        import traj_util
        tj.points.append(traj_util.start_point(now))
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in target]
        pt.velocities = [0.0] * len(ARM)
        pt.time_from_start = Duration(sec=int(dur),
                                      nanosec=int((dur - int(dur)) * 1e9))
        tj.points.append(pt)
        print("  发 %s（%.1fs，起点=当前实测）…" % (what, dur))
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("   !! 目标被拒")
            log_line(st["log"], event="traj", waypoint=what, ok=False,
                     error="goal rejected")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=dur + 30)
        res = rf.result()
        if res is None:
            print("   !! 没等到结果")
            log_line(st["log"], event="traj", waypoint=what, ok=False,
                     error="no result")
            return False
        gst, ec = int(res.status), int(res.result.error_code)
        ok = (gst == GoalStatus.STATUS_SUCCEEDED and ec == 0)
        print("   %s status=%d error_code=%d"
              % ("OK" if ok else "**失败**", gst, ec))
        log_line(st["log"], event="traj", waypoint=what, ok=bool(ok),
                 status=gst, error_code=ec)
        if not ok:
            return False
        time.sleep(settle_s)
        rclpy.spin_once(node, timeout_sec=0.05 if settle_s < 0.3 else 0.3)
        fin = joints()
        err = max(abs(a - b) for a, b in zip(fin, target))
        print("   到位误差（实测 vs 目标）最大 %.2f°" % math.degrees(err))
        log_line(st["log"], event="arrive", waypoint=what,
                 err_deg=round(math.degrees(err), 3))
        return True

    def run_plan(i, job):
        """跑一份计划（共用节点/夹爪/ActionClient）。返回退出码，0 = 全绿。

        1 / 2 与单计划时代同义：1 = 轨迹或夹爪判红，2 = 闸门/认证不过。
        """
        plan, order, wr = job["plan"], job["order"], job["wr"]
        st["log"] = job["log"]
        i0 = job["names"].index(order[0])
        st["expected"] = list(plan["q_start"] if i0 == 0
                              else wr[job["names"][i0 - 1]]["q"])
        print("=" * 78)
        print("计划 %d/%d %s" % (i + 1, len(jobs), job["path"]))
        print("  日志 %s" % st["log"])
        print("  闸门基准 = %s" % ("计划起点 q_start" if i0 == 0
                                  else "上一条航点 %s" % job["names"][i0 - 1]))

        ok_all = True
        closed_at = None       # 闭合后的回读，抬升后拿它比
        lifted_at = None
        # 动臂之前先证明夹爪链路通（i0==0 = 从计划起点开始，指尖一定在巡检位悬空）。
        # 例外：--holding-block（爪里已经夹着块，例如 eih 的「carry → 放」第二段）。
        # 只对**真的要用夹爪**的计划做：巡检计划没有夹爪动作，多一次全行程纯属噪声
        # （2026-10-26 第一版就在 survey_pick 上也跑了一遍，多花 3 秒）。
        #
        # 2026-11-12：**一个进程跑完全部计划**后，自检每轮只做一次
        # （``st["selftest_done"]``）—— 同一根串口、同一双手，后面几份计划再做
        # 一遍空夹全行程纯属白等 3.2 s。
        needs_grip = any(w.get("before") or w.get("after")
                         for w in plan["waypoints"])
        if (args.go and i0 == 0 and needs_grip and not args.dry_run_gripper
                and not args.holding_block and not st["selftest_done"]):
            try:
                gripper_selftest()
                st["selftest_done"] = True
            except ga.GripperActionError as exc:
                print("!! 夹爪自检判红：%s —— 一条轨迹都不发" % exc)
                return 1
        # 闸门**每份计划一次**（2026-11-12）。原先每个航点查一次：一份 8 航点
        # 的抓放计划要查 8 遍，每遍两次服务调用，正常时白花 ~1 s、Dashboard
        # 一卡就是几十秒；而两次航点之间**没有任何东西会改变**这些状态 ——
        # 程序停了、JTC 掉了，下一条轨迹会直接以 status/error_code 判红
        # （`send` 两个都查），不会静默动臂。改成计划边界 + 起手各一次。
        ok, _ = gate("计划 %d/%d 起手" % (i + 1, len(jobs)))
        if not ok:
            print("!! 闸门不过，这份计划一条轨迹都不发")
            return 2
        for name in order:
            w = wr[name]
            try:
                if w.get("before"):
                    # 传**整个 spec**：`open_to` 的开口在 `mm` 里，只取
                    # `["gripper"]` 会把它丢掉（2026-10-27 B2 首块就栽在这里）。
                    do_gripper(w["before"], name + ".before")
                if w.get("q") is not None:
                    settle_s = (args.settle
                                if _moves_fingers(w.get("before"))
                                or _moves_fingers(w.get("after")) else 0.0)
                    if not send(w["q"], w["duration"], name,
                                settle_s=settle_s):
                        print("!! %s 失败，后续一条都不发" % name)
                        return 1
                    st["expected"][:] = w["q"]
                if w.get("after"):
                    pos = do_gripper(w["after"], name + ".after")
                    # 手指被物体挡住 ⇒ 回读停在中间；全闭 ⇒ 空夹。
                    # 中间值**不等于**开口（见 AG95_EMPTY_TOL_MM），只判「夹到没」。
                    if _action_name(w["after"]) == "close":
                        closed_at = pos
                        # 空夹/读不到 ⇒ **就地停**：再抬升、再去槽位、再松手都是
                        # 白动，还会把空手带到别的块上面（2026-11-09 s3：空夹后
                        # 照样跑完 lift/to_slot/place 全程）。臂停在抓取位。
                        verdict, msg, stop = close_reaction(pos)
                        if stop:
                            print("   ⚠️ " + msg)
                            ok_all = False
                            break
                    elif name == "lift":
                        lifted_at = pos
                dwell = w.get("dwell_s")
                if dwell:
                    # 松手后**停在原地**等块落稳再撤。撤得太快时，手指还在块侧面
                    # 上蹭，块会跟着弹/滑（2026-10-27 用户目视）。计划给的秒数，
                    # 不改任何几何。
                    print("   松手后停 %.2f s 再撤（等块落稳）" % float(dwell))
                    log_line(st["log"], event="dwell", waypoint=name,
                             seconds=round(float(dwell), 3))
                    time.sleep(float(dwell))
            except ga.GripperActionError as exc:
                # 夹爪没做成 ⇒ 停在这里。继续往下（尤其继续抬升/去槽位）没有意义。
                print("!! 夹爪动作判红，停在「%s」：%s" % (name, exc))
                return 1

        # 抬升后复读：夹住的块如果滑了，回读会往闭合位缩。
        if closed_at is not None and lifted_at is not None:
            d = lifted_at - closed_at
            print("  抬升后回读 %.1f mm（闭合时 %.1f，差 %+.1f mm）→ %s"
                  % (lifted_at, closed_at, d,
                     "没滑" if abs(d) <= 1.0 else "**滑了**"))
            log_line(st["log"], event="grasp_check",
                     closed_mm=round(closed_at, 2),
                     lifted_mm=round(lifted_at, 2), delta_mm=round(d, 2),
                     slipped=bool(abs(d) > 1.0))
            if abs(d) > 1.0:
                ok_all = False
        print("  这份计划完成。" if ok_all else "  这份计划判红（见上），停轮。")
        return 0 if ok_all else 1

    rc = 0
    for i, job in enumerate(jobs):
        if i:
            # 令牌逐计划核验（单计划时代靠每份计划的独立进程查一次）。执行中
            # 过期是**本轮 void**，不是抓取失败 —— 这句字符串是编排器的唯一判据。
            if args.go and args.require_auth:
                ok, why, age = rma.check_token(args.require_auth)
                if not ok:
                    print("!! 运动授权不成立（aborted_token）：%s" % why)
                    print("   本轮 void（aborted_token），不算抓取失败；"
                          "后面的计划一条都不发")
                    # 结构化进执行日志：编排器此前只能靠 stdout 字符串匹配
                    # 判 void；round_report 直接读 jsonl 时也能认出这条
                    # （2026-11-14 审计：原来这条只活在 stdout）。
                    log_line(jobs[i]["log"], event="aborted_token",
                             reason=str(why), plan_index=i + 1,
                             plans_total=len(jobs))
                    rc = 3
                    break
                print("  运动授权仍在：握手在 %.0f 秒前完成" % age)
            time.sleep(1.0)
        rc = run_plan(i, job)
        if rc != 0:
            print("!! 计划 %d/%d 失败（rc=%d）—— 后面的计划不跑"
                  % (i + 1, len(jobs), rc))
            break

    node.destroy_node()
    rclpy.shutdown()
    if rc == 0:
        print("全部完成（%d 份计划）。" % len(jobs))
    return rc


if __name__ == "__main__":
    sys.exit(main())

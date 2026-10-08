#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""board_lib.py 的自检 —— **纯离线**，不起进程、不开串口、不碰臂。

分八节：
  1. 站点配置：合法性逐条说得对，缺项**不静默回落**
  2. 外参时效：fail-closed（「不知道」不许当成「新鲜」），且与 state_check.py 同线
  3. 残留进程：自我排除按**血缘/入口名**，不按函数住在哪个文件里
  4. 变异检查：把判据真的改坏，断言**改前 ≠ 改后**，并钉住红的是**哪一条**
  5. 源码钉子：顶层不许 import ROS
  6. 命令行行为：模板**必须**被判不合法（一份看着像真的配置比空文件更危险）
  7. 控制柜回复的**线上格式**：带标签 / 带程序名后缀，判定不许拿原文比
  8. 按 session 收栈 + 「AI 转达」：收栈记录、session 门槛、信号、两步握手

     python3 scripts/test_board_lib.py

退出码：0 = 全过；1 = 有断言失败。
"""

import sys

sys.dont_write_bytecode = True   # 防 .pyc 假绿（仓库约定）

import datetime as dt            # noqa: E402
import importlib.util            # noqa: E402
import os                        # noqa: E402
import re                        # noqa: E402
import subprocess                # noqa: E402
import tempfile                  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import board_lib                 # noqa: E402

_failed = []
_n = 0


def check(name, cond, detail=""):
    global _n
    _n += 1
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}"
          + (f"  {detail}" if detail else ""))
    if not cond:
        _failed.append(name)


def mentions(text, *keys):
    return all(k in text for k in keys)


# --------------------------------------------------------------------- 夹具
# ⚠️ 夹具里**一律用占位符**，不写真 IP / 真序列号 —— 本文件是被提交的，
#    而 scripts/test_deidentified.py 会扫它（红线 3：真实值放仓库外）。
TODAY = dt.date(2026, 10, 6)

PORT_PH = "/dev/serial/by-id/usb-<厂商>_<型号>_<序列号>-if00-port0"


def site_raw(**over):
    """一份**合法**的站点配置，再按需覆盖。"""
    raw = {
        "robot_ip": "<ur-controller-ip>",
        "ros_domain_id": 99,
        "planner_params": "~/site_real.yaml",
        "gripper": {"port": PORT_PH},
        "topcam": {
            "verified": "2026-10-05",
            "ttl_days": 1,
            "extrinsics": {k: "0.1" for k in board_lib.CAM_KEYS},
        },
    }
    for k, v in over.items():
        if v is None:
            raw.pop(k, None)
        elif isinstance(v, dict) and isinstance(raw.get(k), dict):
            raw[k].update(v)
        else:
            raw[k] = v
    return raw


def probs(raw, env=None, **over):
    return board_lib.validate_site(board_lib.parse_site(site_raw(**over)), env=env or {})


def one_problem(name, raw_over, *keys, **kw):
    p = probs(None, **raw_over)
    hit = [x for x in p if mentions(x, *keys)]
    check(name, len(hit) == 1 and len(p) == 1,
          f"命中 {len(hit)} 条 / 共 {len(p)} 条：{p}")


# ------------------------------------------------------------ 1. 站点配置
def section_site():
    print("\n[1] 站点配置：缺项要说得出缺哪一项、会出什么症状")
    check("合法配置 ⇒ 没有毛病", probs(None) == [], str(probs(None)))

    p = probs(None, env={"ROS_DOMAIN_ID": "7"})
    check("域号与环境下一致时才算过：99 vs 7 ⇒ 报不一致",
          len(p) == 1 and mentions(p[0], "99", "7", "不一致"), str(p))

    one_problem("缺 robot_ip ⇒ 单独一条", {"robot_ip": None}, "robot_ip")

    s = board_lib.parse_site(site_raw(robot_ip=None))
    old = os.environ.get("UR_ROBOT_IP")
    os.environ["UR_ROBOT_IP"] = "<ur-controller-ip>"
    try:
        check("robot_ip 缺失但环境变量给了 ⇒ 不算毛病（现场不想写进文件是合理的）",
              board_lib.validate_site(s, env={}) == [],
              str(board_lib.validate_site(s, env={})))
    finally:
        if old is None:
            os.environ.pop("UR_ROBOT_IP", None)
        else:
            os.environ["UR_ROBOT_IP"] = old

    one_problem("缺 planner_params ⇒ 要说清症状是「静默用仿真几何」",
                {"planner_params": None}, "planner_params", "仿真几何")

    one_problem("缺 gripper.port ⇒ 单独一条",
                {"gripper": {"port": None}}, "gripper.port")

    p = probs(None, topcam={"extrinsics": {k: "0.1" for k in board_lib.CAM_KEYS[:-1]}})
    check("外参缺一个键 ⇒ 点名是哪把键",
          len(p) == 1 and mentions(p[0], board_lib.CAM_KEYS[-1]), str(p))

    p = probs(None, topcam={"extrinsics": dict(
        {k: "0.1" for k in board_lib.CAM_KEYS}, camera_qw="<四元数>")})
    check("外参不是数（模板占位符）⇒ 报出来，不静默当成 0",
          len(p) == 1 and mentions(p[0], "camera_qw", "不是数"), str(p))

    p = probs(None, topcam={"verified": None})
    check("缺 verified ⇒ 说出来它导致「永远不会被判过期」",
          len(p) == 1 and mentions(p[0], "verified", "过期"), str(p))

    p = probs(None, topcam={"ttl_days": None})
    check("缺 ttl_days ⇒ 单独一条",
          len(p) == 1 and mentions(p[0], "ttl_days"), str(p))

    # 一次列全，别抛第一个就停 —— 现场来回改五行比来回改一行省事
    p = probs(None, robot_ip=None, planner_params=None, gripper={"port": None})
    check("三处都缺 ⇒ 一次列三条（不提前返回）", len(p) == 3, str(p))

    check("Site.extrinsics() 的值一律是字符串（launch 参数就是字符串）",
          all(isinstance(v, str) for v in
              board_lib.parse_site(site_raw()).extrinsics().values()))
    check("Site.missing_keys() 按 CAM_KEYS 顺序点名",
          board_lib.parse_site(site_raw(topcam={"extrinsics": {}})).missing_keys()
          == list(board_lib.CAM_KEYS))
    check("顶层不是映射 ⇒ SiteError（不是静默当空配置）",
          _raises_site_error(lambda: board_lib.parse_site(["不是", "映射"])))


def _raises_site_error(fn):
    try:
        fn()
    except board_lib.SiteError:
        return True
    except Exception:                             # noqa: BLE001
        return False
    return False


# ------------------------------------------------------------ 2. 外参时效
def section_freshness():
    print("\n[2] 外参时效：fail-closed，「不知道」不许当成「新鲜」")
    j = board_lib.judge_freshness

    st, det = j("2026-10-05", 1, TODAY)
    check("刚好满 TTL（age == 保质期）当天仍算数 —— 与 state_check.py 同一条线",
          st == board_lib.PASS and mentions(det, "剩 0 天"), f"{st}: {det}")

    st, det = j("2026-10-04", 1, TODAY)
    check("超一天 ⇒ FAIL，且说明里给出下一步命令",
          st == board_lib.FAIL and mentions(det, "已超期", "topcam_drift_check"), f"{st}: {det}")

    st, det = j(dt.date(2026, 10, 6), 0, TODAY)
    check("accepts date 对象；ttl=0 表示「只认当天」", st == board_lib.PASS, f"{st}: {det}")

    st, det = j(dt.datetime(2026, 10, 5, 23, 59), 1, TODAY)
    check("accepts datetime 对象（YAML 带时分会解析成它）", st == board_lib.PASS, f"{st}: {det}")

    for name, args, want in (
            ("verified 缺失", (None, 1, TODAY), "verified"),
            ("verified 解析不出日期", ("去年某天", 1, TODAY), "verified"),
            ("ttl_days 缺失", ("2026-10-05", None, TODAY), "ttl_days"),
            ("ttl_days 不是整数", ("2026-10-05", "abc", TODAY), "ttl_days"),
            ("ttl_days 是负数", ("2026-10-05", -1, TODAY), "负数"),
    ):
        st, det = j(*args)
        check(f"{name} ⇒ FAIL 且点名（不是当成新鲜）",
              st == board_lib.FAIL and want in det, f"{st}: {det}")

    st, det = board_lib.freshness_gate(board_lib.parse_site(site_raw()), today=TODAY)
    check("闸门：默认走 judge_freshness", st == board_lib.PASS, f"{st}: {det}")

    st, det = board_lib.freshness_gate(board_lib.parse_site(site_raw()), today=TODAY,
                                       override="相机刚重新标定过，值班人张三")
    check("破例：理由被原样记下来（报告里要能看见谁破的例）",
          st == board_lib.PASS and mentions(det, "⚠️", "重新标定过"), f"{st}: {det}")

    for blank in ("", "   "):
        st, det = board_lib.freshness_gate(board_lib.parse_site(site_raw()), today=TODAY,
                                           override=blank)
        check(f"破例但理由是空的（{blank!r}）⇒ FAIL：破例必须写下为什么",
              st == board_lib.FAIL and "空理由" in det, f"{st}: {det}")


# ------------------------------------------------------------ 3. 残留进程
def section_leftovers():
    print("\n[3] 残留进程：排除的是「我们这一族」，不是「名字恰好像我们的那一个」")

    line_stack = "  5150 ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense"
    line_move = "  5151 /opt/ros/humble/lib/move_group/move_group --ros-args -r __node:=move_group"
    line_quiet = "  7000 python3 scripts/something_else.py --verbose"
    # 调用方自己那条 `ps` 记录：**含一个残留模式**（日志名里带 planner_node）
    line_me = ("  4242 python3 scripts/test_board_lib.py "
               "--log ~/planner_node.log")
    # 经 ssh 跑时的那一层 shell：命令行里带着同样的模式（ssh 直接 255 的那次）
    line_ssh = ("  6000 bash -c cd ~ && pkill -f planner_node; "
                "python3 scripts/board_run.py")
    ps = [line_stack, line_move, line_quiet, line_me, line_ssh]

    got = [s.strip() for s in board_lib.leftovers(ps_out=ps, own=set())]
    check("只报真正的栈进程：不相干的、ssh 那层、以及**调用方自己**都排除掉",
          got == [line_stack.strip(), line_move.strip()], str(got))

    # 这一条是本次修复的**形状本身**：me 换成函数所在文件的名字（搬家后发生的事），
    # 调用方那条记录立刻被误报成「有残留」⇒ 前置检查拒绝起栈。
    # ⚠️ 这个夹具里**不能**出现 `board_lib` —— 否则 me="board_lib.py" 会靠子串
    #    碰巧匹配上（本文件名 test_board_lib.py 就含它），演示不出失效。
    line_caller = "  4242 python3 scripts/board_run.py --log ~/planner_node.log"
    got2 = [s.strip() for s in board_lib.leftovers(ps_out=[line_caller], me="board_lib.py",
                                                   own=set())]
    check("me 用「函数所在文件」的名字（搬家后的旧形状）⇒ 误报调用方",
          got2 == [line_caller.strip()], str(got2))
    got2b = [s.strip() for s in board_lib.leftovers(ps_out=[line_caller],
                                                    me="board_run.py", own=set())]
    check("  同一行、me 换成**入口名** ⇒ 排除掉了（这就是修好的样子）",
          got2b == [], str(got2b))

    got3 = [s.strip() for s in board_lib.leftovers(ps_out=[line_stack], me="x.py",
                                                   own={5150})]
    check("血缘里的 pid（自己/祖先）⇒ 排除", got3 == [], str(got3))

    got4 = [s.strip() for s in board_lib.leftovers(ps_out=[line_stack], me="", own=set())]
    check("me 是空串 ⇒ **不许**变成「全部排除」（空名字会关掉整条检查）",
          got4 == [line_stack.strip()], str(got4))

    got5 = [s.strip() for s in board_lib.leftovers(ps_out=[line_stack], me="py",
                                                   own=set())]
    check("me 太短（当成子串会误伤）⇒ 不参与排除", got5 == [line_stack.strip()], str(got5))

    own = board_lib.own_pids()
    check("own_pids() 至少含自己，且真的在往上追",
          os.getpid() in own and len(own) >= 2, str(own))
    check("own_pids() 追到 init 就停（不返回 0）", 0 not in own and 1 not in own, str(own))

    check("默认 me 就是本入口的名字 —— 由 sys.argv[0] 得来，与函数所在文件无关",
          board_lib._me_default() == os.path.basename(sys.argv[0]),
          f"{board_lib._me_default()!r} vs {os.path.basename(sys.argv[0])!r}")

    check("is_own_process 是纯函数，能单独判",
          board_lib.is_own_process("bash -c foo", "x.py", set()) is True
          and board_lib.is_own_process("ros2 launch yolo_bringup", "x.py", set()) is False)

    # ---- pattern 参数：分阶段脚本各有各的「什么算残留」----------------------
    # 直接钉住「不许把两个模式合并成一个」：合并**两个方向都会静默出错**，
    # 而两个方向都不报错，只表现成「检查说没事」。
    stage_pat = re.compile(r"fake_joint_states|move_group")
    # ⚠️ 这一行**只许**含 `fake_joint_states` 一个模式词。第一版写的是
    #    `.../lib/robot_state_publisher/fake_joint_states` —— 于是默认模式靠路径里的
    #    `robot_state_publisher` **顺带**匹配上了，「默认认不出它」那条断言当场翻车。
    #    夹具里混进别的模式词，测出来的就是那个词，不是被测的这条判据。
    #    形状照 no_arm_stack.py:240 真实 spawn 的那条命令来。
    line_fake = "  5300 python3 /opt/ur5vg/scripts/fake_joint_states.py"
    line_drv = "  5301 /opt/ros/humble/lib/ur_robot_driver/dashboard_client"

    check("默认 pattern=None 就是 LEFTOVER_RE（不是「另一个恰好相同的对象」）",
          board_lib.leftovers(ps_out=[], pattern=None, own=set()) == []
          and board_lib.LEFTOVER_RE.search("ur_ros2_control_node") is not None)

    got_stage = [s.strip() for s in
                 board_lib.leftovers(ps_out=[line_fake], pattern=stage_pat, own=set())]
    check("给了阶段 pattern ⇒ 认得本阶段的 fake_joint_states",
          got_stage == [line_fake.strip()], str(got_stage))
    got_drv_stage = [s.strip() for s in
                     board_lib.leftovers(ps_out=[line_drv], pattern=stage_pat, own=set())]
    check("  ⚠️ 同一个阶段 pattern 认不出真驱动 —— 这正是**不能合并**的原因"
          "（合并会让阶段检查少报一类残留）",
          got_drv_stage == [], str(got_drv_stage))

    got_fake_default = [s.strip() for s in
                        board_lib.leftovers(ps_out=[line_fake], own=set())]
    check("  ⚠️ 反过来也一样：默认（整车）pattern 认不出 fake_joint_states"
          "（合并会让整车检查少报一类残留）",
          got_fake_default == [], str(got_fake_default))

    me_line = "  4242 python3 scripts/no_arm_stack.py --log ~/fake_joint_states.log"
    got_me = [s.strip() for s in board_lib.leftovers(
        ps_out=[me_line], me="no_arm_stack.py", pattern=stage_pat, own=set())]
    check("换了 pattern，**自我排除照旧生效**（两件事互不牵连）",
          got_me == [], str(got_me))
    got_pid = [s.strip() for s in board_lib.leftovers(
        ps_out=[line_fake], pattern=stage_pat, own={5300})]
    check("  血缘排除在自定义 pattern 下也生效", got_pid == [], str(got_pid))


# ------------------------------------------------------------ 4. 变异检查
def load_mutant(src_text, tag):
    d = tempfile.mkdtemp(prefix=f"libmut_{tag}_")
    path = os.path.join(d, "board_lib_mutant.py")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(src_text)
    spec = importlib.util.spec_from_file_location(f"board_lib_mutant_{tag}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _mutate(src, old, new, tag):
    """找不到要改的那一行就**当场记 FAIL** —— 否则「变异没生效」会被读成
    「断言很结实」（改前改后都没变，于是 `base != got` 那类断言就悄悄失真）。"""
    if old not in src:
        check(f"{tag} 找得到要改的那一行", False, old[:60])
        return None
    return load_mutant(src.replace(old, new, 1), tag)


def section_mutation():
    print("\n[4] 变异检查：把判据改坏，结论必须**真的翻面**")
    src = open(os.path.join(HERE, "board_lib.py"), encoding="utf-8").read()

    # M4a：`>` 改 `>=`。刚好满保质期那一天由此翻面。
    m = _mutate(src, "    if age > ttl:", "    if age >= ttl:  # MUT", "m4a")
    if m is not None:
        base, _ = board_lib.judge_freshness("2026-10-05", 1, TODAY)
        got, gdet = m.judge_freshness("2026-10-05", 1, TODAY)
        check("M4a 边界从 `>` 变 `>=` ⇒ 刚好当天被判超期（与改前不同）",
              base == board_lib.PASS and got == board_lib.FAIL,
              f"改前={base} 改后={got} 理由={gdet}")

    # M4b：`verified` 缺失 ⇒ 当作「今天刚核对过」。fail-closed 被反过来。
    #      ⚠️ 变异点选在赋值那行、而不是下面那个 `if d is None:` —— 否则 d 仍是
    #      None，`today - None` 会抛异常，变异体是**崩了**而不是**翻面**，
    #      这样测出来的是「改坏了会炸」，不是「判据被拆掉了」。
    m = _mutate(src, "    d = as_date(verified)",
                "    d = as_date(verified) or today  # MUT", "m4b")
    if m is not None:
        base, bdet = board_lib.judge_freshness(None, 1, TODAY)
        got, gdet = m.judge_freshness(None, 1, TODAY)
        check("M4b verified 缺失当成「今天核对的」⇒「不知道」变成了「绿」（与改前不同）",
              base == board_lib.FAIL and got == board_lib.PASS,
              f"改前={base} 改后={got} 理由={gdet}")
        check("  改前的红是因为 verified 缺失", "verified" in bdet, bdet)
        check("  同一变异不许顺手把「真的超期」也放过（不是靠少报换来的绿）",
              m.judge_freshness("2026-10-04", 1, TODAY)[0] == board_lib.FAIL)

    # M4c：ttl_days 缺失 ⇒ 不再拦。
    m = _mutate(src, "        return FAIL, MISSING_TTL",
                '        return PASS, "MUT"', "m4c")
    if m is not None:
        base, _ = board_lib.judge_freshness("2026-10-05", None, TODAY)
        got, _ = m.judge_freshness("2026-10-05", None, TODAY)
        check("M4c ttl_days 缺失不再拦 ⇒ 一行永远无法被判过期（与改前不同）",
              base == board_lib.FAIL and got == board_lib.PASS,
              f"改前={base} 改后={got}")

    # M7：血缘/入口名排除改回 `basename(__file__)` —— 搬家后静默失效的那个判据。
    #     夹具是**真命令行形状**（日志名里带 planner_node），并且 me 走默认值
    #     （正是 sys.argv[0] 那条路），所以这条变异真的会被走到。
    m = _mutate(src, '    return os.path.basename(sys.argv[0] or "")',
                '    return os.path.basename(__file__)  # MUT', "m7")
    if m is not None:
        line_me = ("  4242 python3 scripts/test_board_lib.py "
                   "--log ~/planner_node.log")
        base = [s.strip() for s in board_lib.leftovers(ps_out=[line_me], own=set())]
        got = [s.strip() for s in m.leftovers(ps_out=[line_me], own=set())]
        check("M7 改回 basename(__file__) ⇒ 调用方被误报成残留进程（与改前不同）",
              base == [] and got == [line_me.strip()], f"改前={base} 改后={got}")
        check("  改前之所以为空，是因为排除的是**入口名**而不是文件名",
              board_lib._me_default() in line_me)
        # 反向：这条变异**不许**把「别人起的栈」也漏掉
        line_stack = "  5150 ros2 launch yolo_bringup bringup.launch.py"
        check("  同一变异不会把真残留一起漏掉（不是靠少报换来的绿）",
              [s.strip() for s in m.leftovers(ps_out=[line_stack], own=set())]
              == [line_stack.strip()])

    # M8：空 me 也参与排除 —— 一条空名字就把整条检查关掉，且关得无声无息。
    m = _mutate(src, "    if me and len(me) >= 4 and me in args:",
                "    if me in args:  # MUT", "m8")
    if m is not None:
        line_stack = "  5150 ros2 launch yolo_bringup bringup.launch.py"
        base = [s.strip() for s in board_lib.leftovers(ps_out=[line_stack], me="",
                                                       own=set())]
        got = [s.strip() for s in m.leftovers(ps_out=[line_stack], me="", own=set())]
        check("M8 空名字参与子串匹配 ⇒ 一条残留都报不出来（与改前不同）",
              base == [line_stack.strip()] and got == [], f"改前={base} 改后={got}")

    # M12：`pattern` 收了不用（永远走默认）—— 于是分阶段脚本**以为自己**在按
    #      本阶段的模式查，实际查的是整车模式：fake_joint_states 那类残留**静默漏报**。
    #      ⚠️ 这条与「模式该不该合并」是两件事：合并会错，收了不用**更**错
    #      （前者是设计取舍，后者是参数根本没接上线）。
    m = _mutate(src, "    pat = LEFTOVER_RE if pattern is None else pattern",
                "    pat = LEFTOVER_RE  # MUT", "m12")
    if m is not None:
        stage_pat = re.compile(r"fake_joint_states")
        line_fake = "  5300 /opt/ros/humble/lib/fake_joint_states"
        base = [s.strip() for s in board_lib.leftovers(
            ps_out=[line_fake], pattern=stage_pat, own=set())]
        got = [s.strip() for s in m.leftovers(
            ps_out=[line_fake], pattern=stage_pat, own=set())]
        check("M12 收了 pattern 却不用 ⇒ 本阶段残留静默漏报（与改前不同）",
              base == [line_fake.strip()] and got == [], f"改前={base} 改后={got}")
        # 反向：默认路径**不许**受影响 —— 否则这个变异是靠「把默认也改坏」换来的
        line_drv = "  5150 ros2 launch yolo_bringup bringup.launch.py"
        check("  同一变异没动默认路径（整车模式照旧）",
              [s.strip() for s in m.leftovers(ps_out=[line_drv], own=set())]
              == [line_drv.strip()])


# ------------------------------------------------------------ 5. 源码钉子
def toplevel_imports(path):
    """列出**顶层**的 import 名字（不递归进函数体）。

    为什么用 AST 而不是 `"import rclpy" not in src`：本文件的说明里就得写
    「顶层不 import rclpy」这句话，字符串判据会把**说明**当成**代码**，
    于是钉子要么误报、要么为了不误报而写成更松的东西。
    AST 问的是「真的有没有这条 import」，与注释怎么写无关。
    """
    import ast
    tree = ast.parse(open(path, encoding="utf-8").read())
    out = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            out += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            out.append(node.module or "")
    return out


def section_source():
    print("\n[5] 源码钉子：顶层不 import ROS（离线可测的唯一前提）")
    src = open(os.path.join(HERE, "board_lib.py"), encoding="utf-8").read()
    tops = toplevel_imports(os.path.join(HERE, "board_lib.py"))
    bad = [m for m in tops if m.split(".")[0] in ("rclpy", "launch")]
    check("顶层没有 rclpy / launch（连 `import rclpy.x` 也不算数）",
          bad == [], f"顶层 import：{tops}")
    check("  懒加载确实在函数里（yaml 只在 load_site 里 import，顶层没有）",
          "import yaml" in src and "yaml" not in tops, f"顶层 import：{tops}")
    check("自我排除用的是 sys.argv[0]（入口），不是 __file__（函数住哪）",
          'return os.path.basename(sys.argv[0] or "")' in src)
    check("空/过短的 me 不参与排除（防一条空名字关掉整条检查）",
          "len(me) >= 4" in src)
    check("本文件不写设备：连 switch/deactivate 这类词都没有",
          "deactivate" not in src and "switch_controller" not in src)
    check("本文件里的串口路径只以占位符形式出现（没有真序列号）",
          "usb-<" in src)


# ------------------------------------------------------------ 6. 命令行行为
def _run(*args, env=None):
    e = {k: v for k, v in os.environ.items()
         if k not in ("ROS_DOMAIN_ID", "UR5VG_SITE", "UR_ROBOT_IP")}
    e.update(env or {})
    return subprocess.run([sys.executable, os.path.join(HERE, "board_lib.py"), *args],
                          capture_output=True, text=True, env=e)


def section_cli():
    print("\n[6] 命令行行为：模板必须被判**不合法**")
    r = _run("--site", "/nonexistent/site.yaml")
    check("站点文件不存在 ⇒ 退 2（环境不满足），并告诉人怎么建",
          r.returncode == 2 and "site.example.yaml" in r.stdout,
          f"rc={r.returncode} {r.stdout.strip()[:120]}")

    tpl = os.path.join(REPO, "config", "site.example.yaml")
    check("仓库里那份模板确实在", os.path.exists(tpl))
    r = _run("--site", tpl, "--today", "2026-10-06")
    check("模板 ⇒ 退 1（**绝不能退 0**）：占位符必须过不了校验",
          r.returncode == 1 and "毛病" in r.stdout,
          f"rc={r.returncode} {r.stdout.strip()[:160]}")
    check("  且点出外参不是数 / 日期解析不了",
          mentions(r.stdout, "camera_x", "不是数", "verified"), r.stdout.strip()[:300])

    r = _run("--site", tpl, "--today", "昨天")
    check("--today 给个解析不了的值 ⇒ 退 2，不当成今天", r.returncode == 2,
          f"rc={r.returncode}")

    # 反过来：一份**填好了**的配置必须能退 0，否则上面那条「退 1」可能只是
    # 「这脚本永远退 1」——绿得没有理由的反面同样要排除。
    d = tempfile.mkdtemp(prefix="libsite_")
    good = os.path.join(d, "site.yaml")
    with open(good, "w", encoding="utf-8") as fh:
        fh.write(f"robot_ip: <ur-controller-ip>\n"
                 f"ros_domain_id: 99\n"
                 f"planner_params: ~/site_real.yaml\n"
                 f"gripper:\n  port: {PORT_PH}\n"
                 f"topcam:\n  verified: 2026-10-06\n  ttl_days: 1\n"
                 f"  extrinsics: {{"
                 + ", ".join(f"{k}: 0.1" for k in board_lib.CAM_KEYS) + "}\n")
    r = _run("--site", good, "--today", "2026-10-06")
    check("填好的配置 ⇒ 退 0（证明上一条的 1 是有理由的红，不是恒红）",
          r.returncode == 0 and "可用" in r.stdout,
          f"rc={r.returncode} {r.stdout.strip()[:160]}")
    r = _run("--site", good, "--today", "2026-10-09")
    check("同一份配置，三天后再跑 ⇒ 退 1（外参超期）",
          r.returncode == 1 and "已超期" in r.stdout, f"rc={r.returncode}")


# --------------------------------------------------------------------- main
def section_dash():
    """控制柜回复的**线上格式**（2026-10-06 实测原文）。

    这一节的存在理由：夹具里曾经写的是**猜的**格式（`{'robotmode': 'RUNNING'}`），
    猜错了，于是真机上 `P1.控制柜` 恒 FAIL、`P3` 恒等不到播放，而离线全绿。
    所以这里的素材一律是**线上原文**，逐字抄，不整理。
    """
    print("\n[7] 控制柜回复带标签/后缀：判定不许拿原文比（2026-10-06 实测）")
    RAW_RUN = {"robotmode": "Robotmode: RUNNING",
               "programState": "STOPPED ur5ros.urp",
               "safetystatus": "Safetystatus: NORMAL"}
    check("robotmode 剥掉 'Robotmode: ' 标签 ⇒ RUNNING",
          board_lib.dash_mode(RAW_RUN) == "RUNNING")
    check("safetystatus 同样带标签 ⇒ NORMAL",
          board_lib.dash_field(RAW_RUN["safetystatus"]) == "NORMAL")
    check("programState 无标签、缀着程序名 ⇒ 取首词 STOPPED（实测原文）",
          board_lib.dash_program_state(RAW_RUN) == "STOPPED")
    check("  播放那行 ⇒ PLAYING（线上**未实测**，由 STOPPED 那行同构推来；"
          "判据取首词，不依赖它）",
          board_lib.dash_program_state(
              {"programState": "PLAYING ur5ros.urp"}) == "PLAYING")
    check("标签白名单：值里带冒号不被切坏（'PLAYING C:\\prog.urp'）",
          board_lib.dash_program_state(
              {"programState": "PLAYING C:\\prog.urp"}) == "PLAYING")
    check("  未知冒号前缀**不**当标签剥（'Whatever: PLAYING' 原样返回）",
          board_lib.dash_field("Whatever: PLAYING") == "Whatever: PLAYING")
    check("空 / None / 连不上 ⇒ 空串（不抛，也不编一个状态出来）",
          board_lib.dash_program_state(None) == ""
          and board_lib.dash_mode({}) == ""
          and board_lib.dash_program_state({"programState": "   "}) == "")
    check("is_playing 只认**首词** PLAYING：NOT_PLAYING / 程序名里含 PLAYING 都不算",
          board_lib.is_playing({"programState": "PLAYING ur5ros.urp"})
          and not board_lib.is_playing({"programState": "NOT_PLAYING x.urp"})
          and not board_lib.is_playing({"programState": "STOPPED myPLAYING.urp"})
          and not board_lib.is_playing(None))


def section_session_and_ai():
    """按 session 收栈 + 「AI 转达」那一套（2026-10-08 修「挂残留 / 孤儿进程」与
    「不支持 AI 驱动」时新加的）。

    ⚠️ 这一节**绝不**真的调 `sweep()` / `teardown()`：它们会去杀**真的**进程
    （本机此刻就可能有别的残留）。判据里能离线验的部分全是纯的（记录读写、
    session 门槛、握手循环），这些才是这里要钉的。
    """
    print("\n[8] 按 session 收栈 + AI 转达：判据离线可钉的部分")

    # ---- 残留名单：漏一个名字的代价是**假绿**（不是红）------------------
    # 2026-10-08 实测：`trajectory_until_node` 与 `urscript_interface` 原来不在
    # 名单里 ⇒ `--stop` 报「已收干净」是假的，P1.残留进程 跟着一起假绿。
    for name in ("ur_robot_driver/trajectory_until_node",
                 "ur_robot_driver/urscript_interface",
                 "ur_robot_driver/controller_stopper_node",
                 "controller_manager/ros2_control_node",
                 "controller_manager/spawner"):
        check(f"残留名单认得 {name}（漏了它 ⇒ 收栈报「干净」是假的）",
              board_lib.LEFTOVER_RE.search(name) is not None)
    # 反过来：本仓**不起**的东西不许进名单 —— 进了就会去收别人的界面/仿真节点。
    for name in ("rviz2", "servo_node_main"):
        check(f"  名单**不认** {name}（本仓 launch_rviz/launch_servo 都关着，"
              f"认了会去杀别人的进程）",
              board_lib.LEFTOVER_RE.search(name) is None)

    # ---- session：不随 re-parent 改变的那条线索 -------------------------
    state, sid = board_lib.proc_stat(os.getpid())
    check("proc_stat(自己) 读得到 (state, session)，session 就是自己的 session",
          state in ("R", "S", "D") and isinstance(sid, int) and sid > 0,
          f"state={state} sid={sid}")
    check("sid_of 就是 proc_stat 的第 2 个值（同一个切法，不另抄一遍）",
          board_lib.sid_of(os.getpid()) == sid)
    check("读不到的 pid ⇒ (None, None)，不抛（`/proc` 里进程随时会消失）",
          board_lib.proc_stat(2 ** 22) == (None, None)
          and board_lib.sid_of(2 ** 22) is None)

    check("session 门槛：sid == 自己 pid 才算「它自己开的 session」",
          board_lib.is_session_leader(1234, 1234) is True
          and board_lib.is_session_leader(1234, 5678) is False)
    check("  自己这一条读的是**同一个判据**："
          "is_session_leader(getpid()) == (sid_of(getpid()) == getpid())",
          board_lib.is_session_leader(os.getpid())
          == (board_lib.sid_of(os.getpid()) == os.getpid()))
    got = board_lib.session_pids(9, entries=[(1, 9), (2, 9), (3, 8), (4, 7)])
    check("session_pids 按 session 挑，且**排除本进程**",
          got == [1, 2] and os.getpid() not in got, str(got))
    check("★ 真扫 `/proc` 时**僵尸不算活口**（它已经死了、只是还没被回收；"
          "算进去的话收栈会永远报「没收干净」——e2e 里就是这么翻车的）",
          "state != \"Z\"" in open(os.path.join(HERE, "board_lib.py"),
                                  encoding="utf-8").read())

    # ---- 记录下来的 (pid, sid)：`--stop` 在启动器被打死之后靠它 -------------
    d = tempfile.mkdtemp(prefix="libstack_")
    rec_path = os.path.join(d, "last_stack.json")
    check("没有记录文件 ⇒ read_stack_record 给 {}（**不抛**）",
          board_lib.read_stack_record(rec_path) == {})
    check("  ⚠️ 空记录**不许**被当成「sid 就是 leader」（第一版栽在这儿："
          "`{}` 的 .get 全是 None，`None == None` 让它去收一个不存在的 session，"
          "`--stop` 当场 KeyError 退 1）",
          board_lib.owned_session_pids({}, entries=[]) == []
          and board_lib.owned_session_pids({"pid": 5}, entries=[]) == [])
    p = board_lib.write_stack_record(4242, 4242, path=rec_path,
                                     extra={"task": "a1", "log": "~/x.log"})
    check("写记录（pid == sid ⇒ 它当年自己开的 session）",
          p == rec_path and board_lib.read_stack_record(rec_path)["pid"] == 4242)
    check("  按记录取 session 活口：只挑那个 session 的，别的 session 不碰",
          board_lib.owned_session_pids(
              board_lib.read_stack_record(rec_path),
              entries=[(4242, 4242), (4243, 4242), (999, 1)]) == [4242, 4243])
    check("  ⚠️ 记录里 sid != pid ⇒ **一条都不收**（那是别人 shell 的 session，"
          "按 session 收会把人自己的终端一起杀掉）",
          board_lib.owned_session_pids({"pid": 4242, "sid": 1},
                                       entries=[(4242, 1), (4243, 1)]) == [])
    with open(rec_path, "w", encoding="utf-8") as fh:
        fh.write("{这不是 json")
    check("记录文件坏了 ⇒ {}（**宁可少一条判据**，也不拿半截记录去杀进程）",
          board_lib.read_stack_record(rec_path) == {})
    board_lib.write_stack_record(4242, 4242, path=rec_path)
    board_lib.clear_stack_record(rec_path)
    check("clear 之后读不到（收干净就该销记录，否则下次 `--stop` 拿着旧 sid 乱收）",
          not os.path.exists(rec_path))
    check(f"记录路径可用 ${board_lib.STACK_RECORD_ENV} 覆盖"
          f"（离线测试不许动真人 ~/.config 里那一份）",
          board_lib.stack_record_path(rec_path) == rec_path)

    # ---- 信号：别把栈留成孤儿 ------------------------------------------
    import signal as _signal
    calls = []
    old = _signal.getsignal(_signal.SIGUSR1)
    restore = board_lib.install_stop_handlers(lambda name: calls.append(name),
                                              signals=(_signal.SIGUSR1,))
    check("install_stop_handlers 真的换了处置",
          _signal.getsignal(_signal.SIGUSR1) is not old)
    try:
        _signal.getsignal(_signal.SIGUSR1)(_signal.SIGUSR1, None)
        raised = None
    except SystemExit as exc:
        raised = exc.code
    check("  收到信号 ⇒ **先回调（收栈）、再抛 SystemExit**（默认处置是当场死，"
          "栈就成了孤儿）",
          calls == ["SIGUSR1"] and raised == 128 + _signal.SIGUSR1,
          f"calls={calls} raised={raised}")
    restore()
    check("restore() 把处置还原（测试不留全局副作用）",
          _signal.getsignal(_signal.SIGUSR1) is old)
    _lib_src = open(os.path.join(HERE, "board_lib.py"), encoding="utf-8").read()
    check("  默认管的是 SIGTERM / SIGHUP（`timeout`/CI/关终端那两种死法）",
          "(signal.SIGTERM, signal.SIGHUP) if signals is None else signals"
          in _lib_src)

    # ---- 「AI 转达」的可转达性 ------------------------------------------
    lines = []
    board_lib.human_action("去示教器按**播放**", step=1, total=2,
                           why="本启动器不发 play", out=lines.append)
    check("human_action 第一行以固定前缀开头、**动词在前**、独立成行",
          lines[0].startswith(board_lib.HUMAN_MARK)
          and "（第 1/2 步）" in lines[0] and "按**播放**" in lines[0],
          lines[0])
    check("  第二行是「为什么」的注脚，**不**带前缀（转达第一行就够）",
          len(lines) == 2 and not lines[1].startswith(board_lib.HUMAN_MARK)
          and "本启动器不发 play" in lines[1])
    lines2 = []
    board_lib.relay_notice(out=lines2.append, timeout=300.0, doing="启动器")
    check("relay_notice 说清三件事：那些行是给**人**的、要**原样**念、每步等多久",
          board_lib.HUMAN_MARK in lines2[0] and "转达" in lines2[0]
          and "原样" in lines2[1] and "300s" in lines2[2], str(lines2[:1]))

    # ---- 「人在环」两步握手：判据与文案都离线钉住 ------------------------
    class _Clock:
        def __init__(self):
            self.t = 0.0

        def __call__(self):
            return self.t

        def sleep(self, s):
            self.t += s

    def _handshake(replies, ip="203.0.113.7", timeout=5.0):
        it = iter(replies)
        last = {}

        def _dash(_ip, timeout=5.0):
            try:
                last.clear()
                last.update(next(it))
            except StopIteration:
                pass
            return dict(last)

        c = _Clock()
        said = []
        ok, why = board_lib.ensure_in_the_loop(
            ip, timeout, poll=1.0, dash=_dash, clock=c, sleep=c.sleep,
            prompt=said.append, why="这次写夹爪寄存器的授权",
            consequence="一个寄存器都不写", doing="写夹爪", step=(1, 2))
        return ok, why, said

    ok, why, said = _handshake([{"programState": "STOPPED ur5ros.urp"},
                                {"programState": "PLAYING ur5ros.urp"}])
    check("★ 两步都看到 ⇒ 过（这是「没有终端」时唯一能拿到的当次授权证明）",
          ok and "两步**都看到了**" in why, why)
    check("  两步的提示都是**可转达**的独立行（STOP 一条、PLAY 一条）",
          sum(1 for ln in said if ln.startswith(board_lib.HUMAN_MARK)) == 2
          and any("停止（STOP）" in ln for ln in said)
          and any("播放（PLAY）" in ln for ln in said), str(said[:2]))
    ok, why, _ = _handshake([{"programState": "PLAYING x.urp"}] * 40)
    check("起栈前就是 PLAYING、一直没离开 ⇒ **不过**（残状态不算「人按过」）",
          not ok and "没看到它离开 PLAYING" in why, why)
    ok, why, _ = _handshake([{"programState": "STOPPED x.urp"}] * 40)
    check("  离开了却没回来 ⇒ **不过**（「按过」这件事必须看到，不许猜）",
          not ok and "没等到 PLAYING" in why, why)
    ok, why, _ = _handshake([{}], ip=None)
    check("读不到控制柜 IP ⇒ **核验不了** ⇒ 不过，而且说清后果是「不写」",
          not ok and "核验不了" in why and "不写" in why, why)


def main():
    print("board_lib 自检（纯离线：不起进程、不开串口、不碰臂）")
    section_site()
    section_freshness()
    section_leftovers()
    section_mutation()
    section_source()
    section_cli()
    section_dash()
    section_session_and_ai()
    print(f"\n{_n} 条断言，{len(_failed)} 条失败")
    if _failed:
        for n in _failed:
            print(f"  ✗ {n}")
        return 1
    print("结果：✅ board_lib.py 全过")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""board_gate.py 的自检 —— **纯离线**，不碰 ROS、不碰臂、不碰串口。

为什么这里能离线测
------------------
`board_gate.py` 顶层**不 import rclpy**，判定逻辑全是纯函数（只吃 `dict`/`list`
之类），rclpy 只在几个 `_item_*` 的 I/O 里懒加载。这不是风格洁癖：本机（WSL）的
`/opt/ros/humble` 下**根本没装 rclpy**（只有纯 Python 的 ament_* 工具），
顶层一 import，这个文件连 import 都过不去。而离线可测正是这套东西唯一能被验证的
部分 —— 板端那一层我跑不了（没有板子访问权，且每次运动要授权）。

所以本文件测的是**闸门会不会放行坏的东西**，不是「板子现在通不通」。

变异检查（本仓的约定：削掉闸门必须立刻变红）
--------------------------------------------
第 3 节把 `board_gate.py` 的源码**真的改坏**，重新导入，断言行为确实翻面。
只断言「改坏了会红」是自说自话 —— 这里断言的是**改坏前后判定结果不同**，
从而证明上面那几条断言是**吃劲的**，不是恰好为真。
每个变异还额外钉住**它为什么红**（detail 里的关键字），免得「红」是撞上了
另一条判据。

用法:
    python3 scripts/test_board_gate.py
退出码 0 = 全通过，1 = 有失败项，2 = 环境不满足。
"""

import sys

sys.dont_write_bytecode = True   # 防 .pyc 假绿（仓库约定）

import builtins                  # noqa: E402
import contextlib                # noqa: E402
import importlib.util            # noqa: E402
import io                        # noqa: E402
import os                        # noqa: E402
import tempfile                  # noqa: E402
import types                     # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board_gate                # noqa: E402

_failed = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}"
          + (f"  {detail}" if detail else ''))
    if not cond:
        _failed.append(name)


def mentions(text, *keys):
    return all(k in text for k in keys)


# --------------------------------------------------------------------- 夹具
# 时间轴夹具一律写成 `(秒, {控制器名: state})`，与 judge_controllers 的签名同形。

C = "scaled_joint_trajectory_controller"

# 十个采样、全程 inactive —— 拿来打「子串判定」那处变异：
# 只看最后一瞬的话，`"act" in "inactive"` 会说它是 active。
FIX_ALL_INACTIVE = [(t, {C: "inactive"}) for t in (0, 2, 4, 6, 8, 10)]

# 先 active 了 3 秒、被 STRICT 切换打回 inactive、又回 active（第九十轮的形状）
FIX_FLAP = [(0, {C: "active"}), (3, {C: "inactive"}), (4, {C: "active"})]

# `pick_zone` 的**默认值**，与 `grasp_planner.yaml:101` 一致。这套数是**仿真
# 几何**（桌面顶面 -0.39、方块高 0.04）。⚠️ 现场那份 `site_real.yaml` 里的
# pick_zone 与它**不是同一套**（真机 target_plane_z=0.0285，比这里的 z 上界还高）
# —— 所以这里刻意不拿真机的 (x,y) 去配这个 zone，那只会得到「全在区外」的假红。
ZONE = [0.30, 0.90, -0.18, 0.16, -0.45, -0.30]

# 4 个候选全在区内：用的是 `grasp_planner.yaml` 注释里写明的仿真方块位置
# （x∈{0.44,0.60,0.76}、y∈{-0.12,0,0.12}、z=-0.37）。
FIX_TARGETS_OK = [((0.44, -0.12, -0.37), "block"),
                  ((0.60, 0.00, -0.37), "block"),
                  ((0.76, 0.12, -0.37), "block"),
                  ((0.44, 0.12, -0.37), "block")]

# 前 3 个在区内、第 4 个 y=1.4 —— 1.4 不是随手编的：那是外参 `q` 填错时**实测**
# 投出来的 y（`docs/待执行命令.md` 的三组 A/B 表：y≈1.38~1.57，1.5 m 外臂够不到）。
FIX_TARGETS_ONE_OUT = FIX_TARGETS_OK[:3] + [((0.50, 1.40, -0.37), "block")]

# 「空闲 20s 收工」那一类：收工了，但一个槽位都没占
FIX_TEL_FINISHED = {"state": "FINISHED", "slots_free": "4", "slots_total": "4",
                    "finish_reason": "空闲 20s 没有发起过任何一次抓取尝试"}
FIX_TEL_READY = {"state": "READY", "slots_free": "4", "slots_total": "4"}


# ------------------------------------------------------------ 1. 判定函数
def section_judge():
    print("\n[1] 判定函数：三态分得开，且各自的**理由**说得对")

    # 控制器：all-active 满 10 s
    st, det = board_gate.judge_controllers(
        [(t, {C: "active"}) for t in (0, 2, 4, 6, 8, 10)], [C], 8.0)
    check("全 active 满窗口 ⇒ PASS", st == board_gate.PASS, f"{st}: {det}")

    # 只有一个采样点：稳了 0 秒，绝不许判 PASS
    st, det = board_gate.judge_controllers([(0, {C: "active"})], [C], 8.0)
    check("只采到一个点（稳定 0s）⇒ FAIL，不是 PASS", st == board_gate.FAIL,
          f"{st}: {det}")
    check("  理由说的是「保持时长不够」", mentions(det, "只保持了"))

    # inactive：不能靠子串判成 active
    st, det = board_gate.judge_controllers(FIX_ALL_INACTIVE, [C], 8.0)
    check("全程 inactive ⇒ FAIL", st == board_gate.FAIL, f"{st}: {det}")
    check("  理由里点名了 inactive", mentions(det, "inactive"))

    # 控制器**不在列表里**：与 inactive 分开报
    st, det = board_gate.judge_controllers([(0, {}), (2, {})], [C], 1.0)
    check("控制器不在列表里 ⇒ FAIL 且说「不在控制器列表里」",
          st == board_gate.FAIL and mentions(det, "不在控制器列表里"),
          f"{st}: {det}")

    st, det = board_gate.judge_controllers([], [C], 8.0)
    check("一次采样都没有 ⇒ SKIP（没看成，不是失败）", st == board_gate.SKIP)

    # 相机
    check("相机 22 Hz ⇒ PASS", board_gate.judge_camera(22.3, 5.0)[0]
          == board_gate.PASS)
    check("相机 0 Hz ⇒ FAIL（在图上却不出帧）",
          board_gate.judge_camera(0.0, 5.0)[0] == board_gate.FAIL)
    check("相机没测到 ⇒ SKIP", board_gate.judge_camera(None, 5.0)[0]
          == board_gate.SKIP)

    # 候选
    st, det = board_gate.judge_targets([p for p, _ in FIX_TARGETS_OK],
                                       [l for _, l in FIX_TARGETS_OK],
                                       ZONE, 4)
    check("4 个候选全在区内 ⇒ PASS", st == board_gate.PASS, f"{st}: {det}")
    st, det = board_gate.judge_targets([p for p, _ in FIX_TARGETS_ONE_OUT],
                                       [l for _, l in FIX_TARGETS_ONE_OUT],
                                       ZONE, 4)
    check("4 个候选里 1 个在区外 ⇒ FAIL", st == board_gate.FAIL, f"{st}: {det}")
    check("  理由说的是「落在 pick_zone 外」而不是个数不够",
          mentions(det, "落在 pick_zone 外") and "只有" not in det)
    check("  把越界那个点的坐标打出来了（现场要照着看）",
          mentions(det, "+1.400"))
    st, det = board_gate.judge_targets([p for p, _ in FIX_TARGETS_OK][:3],
                                       ["block"] * 3, ZONE, 4)
    check("3 个候选（都在区内）⇒ FAIL，理由是个数", st == board_gate.FAIL
          and mentions(det, "只有 3 个"))
    check("0 个候选 ⇒ FAIL", board_gate.judge_targets([], [], ZONE, 4)[0]
          == board_gate.FAIL)

    # 边界：区间两端算在区内（闭区间）
    check("正好落在 zone 边界上算在区内（闭区间）",
          board_gate.in_zone((0.30, -0.18, -0.30), ZONE) is True)
    check("z 超出下界不算在区内（2D 矩形挡不住扎地的方块）",
          board_gate.in_zone((0.50, 0.00, -0.60), ZONE) is False)

    # 遥测
    check("就绪 + 槽位全空 ⇒ PASS",
          board_gate.judge_telemetry(FIX_TEL_READY, 4)[0] == board_gate.PASS)
    st, det = board_gate.judge_telemetry(FIX_TEL_FINISHED, 4)
    check("state=FINISHED ⇒ FAIL", st == board_gate.FAIL, f"{st}: {det}")
    check("  理由说的是「已经收工」并带出收工原因",
          mentions(det, "已经收工", "空闲 20s"))
    st, det = board_gate.judge_telemetry(
        {"state": "READY", "slots_free": "3", "slots_total": "4"}, 4)
    check("槽位不是全空 ⇒ FAIL 且说出占了几个", st == board_gate.FAIL
          and mentions(det, "1/4"))
    st, det = board_gate.judge_telemetry(FIX_TEL_READY, 8)
    check("槽位总数与任务期望不符 ⇒ FAIL，并点名 planner_params_file",
          st == board_gate.FAIL and mentions(det, "planner_params_file"))
    check("没有 state 这一项 ⇒ FAIL（报文形状不对）",
          board_gate.judge_telemetry({"slots_free": "4"}, None)[0]
          == board_gate.FAIL)
    check("没收到遥测 ⇒ SKIP",
          board_gate.judge_telemetry(None, None)[0] == board_gate.SKIP)

    # 检测器
    check("model 有、/detections 在发 ⇒ PASS",
          board_gate.judge_detector("block_best.pt", 21.9)[0] == board_gate.PASS)
    check("model 空串 ⇒ FAIL（起不来或起错了）",
          board_gate.judge_detector("", 21.9)[0] == board_gate.FAIL)
    check("model 有但一条都没发 ⇒ FAIL",
          board_gate.judge_detector("block_best.pt", 0.0)[0] == board_gate.FAIL)
    check("读不到参数 ⇒ SKIP", board_gate.judge_detector(None, None)[0]
          == board_gate.SKIP)

    # 参数回读
    TA = board_gate.DEFAULT_TRAJECTORY_ACTION
    st, det = board_gate.judge_params(
        {"trajectory_action": TA, "carry_objects": True,
         "gripper_backend": "ag95"}, {"carry_objects": "true"}, TA)
    check("轨迹 action 一致 + 期望参数相符 ⇒ PASS", st == board_gate.PASS,
          f"{st}: {det}")
    check("  报告里打出了 5 个死键的有效值", mentions(det, "carry_objects"))
    st, det = board_gate.judge_params(
        {"trajectory_action": "/joint_trajectory_controller/follow_joint_trajectory"},
        {}, TA)
    check("规划器用的 action 与闸门盯的不是同一个 ⇒ FAIL",
          st == board_gate.FAIL and mentions(det, "闸门看的是别人"))
    st, det = board_gate.judge_params({"trajectory_action": TA}, {"carry_objects": "true"}, TA)
    check("期望核对的参数不在回读里 ⇒ FAIL", st == board_gate.FAIL
          and mentions(det, "carry_objects"))
    st, det = board_gate.judge_params({"trajectory_action": TA, "carry_objects": False},
                                      {"carry_objects": "true"}, TA)
    check("参数值不符 ⇒ FAIL", st == board_gate.FAIL)
    check("读不到参数 ⇒ SKIP", board_gate.judge_params(None, {}, TA)[0]
          == board_gate.SKIP)

    # bool 归一：命令行只能给字符串，回读回来的是 bool
    check("carry_objects 的 True 与 'true' 判等（否则判据自己错）",
          board_gate._norm(True) == board_gate._norm("true") == "true")
    check("浮点归一去掉多余的 0（0.0285 不会写成 0.028500）",
          board_gate._norm(0.0285) == "0.0285")

    # 从 action 推控制器名
    check("控制器名从轨迹 action 推导（一份真值，不另抄常量）",
          board_gate.controller_from_action(TA) == C)
    check("  action 形状不对时返回 None，不猜",
          board_gate.controller_from_action("nope") is None)


# ------------------------------------------------ 2. 三态：SKIP 不许是绿
def section_three_states():
    print("\n[2] 三态语义：SKIP 是「没验到」，与 PASS 分得开")

    buf = io.StringIO()
    it = board_gate.Item("camera", board_gate.SKIP, "相机不在图上")
    with contextlib.redirect_stdout(buf):
        board_gate._emit(it, [])
    out = buf.getvalue()
    check("SKIP 打的是 [SKIP]，不是 [PASS]", "[SKIP]" in out and "[PASS]" not in out)
    check("SKIP 一定附一句「没验到」", mentions(out, "没验到"))

    # --allow-skip：只有被点名的项才转 SKIP，且 controllers 永不许被点名
    it = board_gate.Item("telemetry", board_gate.FAIL, "规划器没起")
    check("--allow-skip 点名的项转 SKIP",
          board_gate._is_skipped_ok(it, ["telemetry"]) is True)
    check("--allow-skip 没点名的项不动",
          board_gate._is_skipped_ok(it, ["camera"]) is False)
    it = board_gate.Item("controllers", board_gate.FAIL, "控制器没起来")
    check("controllers 不许被 --allow-skip 放过（它是这脚本存在的理由）",
          board_gate._is_skipped_ok(it, ["controllers"]) is False)


# ------------------------------------------------------- 3. 真·变异检查
def load_mutant(src_text, tag):
    d = tempfile.mkdtemp(prefix=f"gatemut_{tag}_")
    path = os.path.join(d, "board_gate_mutant.py")
    with open(path, "w") as fh:
        fh.write(src_text)
    spec = importlib.util.spec_from_file_location(f"board_gate_mutant_{tag}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _mutate(src, old, new, tag):
    """改一处源码、重新导入，把变异体交给断言。

    找不到要改的那一行就**当场记 FAIL** —— 否则「变异没生效」会被读成
    「断言很结实」（改前改后都没变，于是 `base != got` 那类断言就悄悄失真）。
    """
    if old not in src:
        check(f"{tag} 找得到要改的那一行", False, old[:60])
        return None
    return load_mutant(src.replace(old, new, 1), tag)


def section_mutation():
    print("\n[3] 变异检查：把闸门改坏，结论必须**真的翻面**")
    src_path = os.path.join(HERE, "board_gate.py")
    src = open(src_path, encoding="utf-8").read()

    # M1：精确相等 → 子串判定。`inactive` 里连着含 `act`，于是坏的被报成好的。
    m = _mutate(src, '    return state == "active"',
                '    return "act" in str(state or "").lower()',
                "m1")
    if m is not None:
        base, _ = board_gate.judge_controllers(FIX_ALL_INACTIVE, [C], 8.0)
        got, gdet = m.judge_controllers(FIX_ALL_INACTIVE, [C], 8.0)
        check("M1 子串判定 ⇒ 全程 inactive 被判成 active（与改前不同）",
              base == board_gate.FAIL and got == board_gate.PASS,
              f"改前={base} 改后={got} 理由={gdet}")

    # M2：去掉「连续稳定」窗口，只看最后一瞬。
    m = _mutate(src, "    if held + 1e-9 < stable_for:",
                "    if False:  # MUT", "m2")
    if m is not None:
        base, bdet = board_gate.judge_controllers(FIX_FLAP, [C], 8.0)
        got, _ = m.judge_controllers(FIX_FLAP, [C], 8.0)
        check("M2 去掉稳定窗口 ⇒ 被打回又回来的那一瞬被判 PASS（与改前不同）",
              base == board_gate.FAIL and got == board_gate.PASS,
              f"改前={base} 改后={got}")
        check("  改前的红是因为「只保持了 0.0s」",
              mentions(bdet, "只保持了"))

    # M3：候选的「落在 pick_zone 内」降级成「只数个数」。
    m = _mutate(src, "    if outside:", "    if False:  # MUT",
                "m3")
    if m is not None:
        args = ([p for p, _ in FIX_TARGETS_ONE_OUT],
                [l for _, l in FIX_TARGETS_ONE_OUT], ZONE, 4)
        base, _ = board_gate.judge_targets(*args)
        got, _ = m.judge_targets(*args)
        check("M3 只数个数 ⇒ 有个点飞到 y=1.4（臂够不到）也判 PASS（与改前不同）",
              base == board_gate.FAIL and got == board_gate.PASS,
              f"改前={base} 改后={got}")

    # M5：遥测里去掉「已经收工」这一条。
    m = _mutate(src, '    if state == "FINISHED":', "    if False:  # MUT",
                "m5")
    if m is not None:
        base, _ = board_gate.judge_telemetry(FIX_TEL_FINISHED, 4)
        got, _ = m.judge_telemetry(FIX_TEL_FINISHED, 4)
        check("M5 去掉「已经收工」⇒ 收工了的规划器被判 PASS（与改前不同）",
              base == board_gate.FAIL and got == board_gate.PASS,
              f"改前={base} 改后={got}")

    # M4（`judge_freshness` 的 `>` / `>=` 与「verified 缺失当作新鲜」）不在这里：
    # 外参时效属于站点配置那一层（board_lib.Site），随第 2 步落地时在
    # test_board_lib.py 里钉。写在这里会变成钉一个还不存在的函数。


# ------------------------------------------- 4. 源码层面的几条硬钉子
def section_source_pins():
    """这一节测的是**源码形状**，不是行为 —— 因为对应的行为要么需要 rclpy
    （这台机器上没有），要么需要板子。**它们是钉子，不是测试**：作用是让「谁把
    这几行删了」立刻变红，而不是证明现在跑得对。标签里写清楚，免得后来人
    把这里的绿当成验证过了。"""
    print("\n[4] 源码钉子（不是行为测试 —— 行为要 rclpy/板子）")
    src = open(os.path.join(HERE, "board_gate.py"), encoding="utf-8").read()

    top_imports = [ln for ln in src.splitlines()
                   if ln.startswith(("import rclpy", "from rclpy"))]
    check("顶层不 import rclpy（否则本文件在没装 rclpy 的机器上 import 就炸）",
          not top_imports, f"命中 {top_imports}")

    check("遥测订阅用的是 latched_qos()（TRANSIENT_LOCAL，漏了只会永远空着）",
          "qos=latched_qos()" in src)
    check("latched_qos() 里确实是 TRANSIENT_LOCAL",
          "durability=QoSDurabilityPolicy.TRANSIENT_LOCAL" in src)

    check("switch_controller 的 deactivate_controllers 恒为空",
          "req.deactivate_controllers = []" in src
          and "deactivate_controllers = list" not in src)
    check("strictness 用 BEST_EFFORT（STRICT 会整批 Aborting）",
          "req.strictness = SwitchController.Request.BEST_EFFORT" in src)

    # 「先冻结图快照、再建订阅」—— 顺序反了就是自证式判据
    i_freeze = src.find("_freeze_graph(probe")
    i_first_item = src.find("_item_controllers(a, probe)")
    check("先冻结图快照、再建订阅/客户端（顺序反了判据会自证）",
          0 <= i_freeze < i_first_item,
          f"freeze@{i_freeze} 首项@{i_first_item}")


# --------------------------------- 4b. 写路径的当次授权：两条路都不许空过
def section_repair_authorization():
    """`_run_repair` 里「要当次授权」那一段（2026-10-08：非交互终端不再一律拒）。

    这条能离线测的原因：授权那一段**只**依赖 `isatty()` 与注入的 `loop`；探测对象
    是假的 probe —— 不碰 ROS、不碰控制器、不碰串口。发出去的 `switch_controllers`
    记在假 probe 上，于是「核验不过时**一个字节都没发**」是**看得见**的，
    而不是靠读源码相信它。
    """
    print("\n[4b] 写路径的当次授权：终端敲 yes / 示教器 STOP→PLAY，两档都不许空过")

    class _Probe:
        """假 probe：switch 过就报 active，否则 inactive。"""

        def __init__(self):
            self.switched = []

        def switch_controllers(self, req):
            self.switched.append(list(req))

        def list_controllers(self):
            return {C: "active"} if self.switched else {C: "inactive"}

    def _args(**over):
        d = {"repair_attempts": 1, "dashboard_host": "203.0.113.7",
             "play_timeout": 5.0}
        d.update(over)
        return types.SimpleNamespace(**d)

    def quiet(*a):
        return None

    seen = []

    def _loop_ok(ip, timeout, **kw):
        seen.append((ip, timeout, kw.get("consequence")))
        return True, "STOP → PLAY 两步**都看到了**"

    def _loop_no(ip, timeout, **kw):
        seen.append((ip, timeout, kw.get("consequence")))
        return False, "读不到控制柜 IP ⇒ **核验不了**"

    p = _Probe()
    ok = board_gate._run_repair(_args(), p, [C], isatty=lambda: False,
                                loop=_loop_ok, out=quiet)
    check("★ 没有交互终端（AI 驱动）⇒ 走示教器两步握手，过了才发 switch_controller",
          ok is True and p.switched == [[C]], f"ok={ok} switched={p.switched}")
    check("  而且真把控制柜 IP 与「不写的后果」递进了握手（不是「返回 True 就算过」）",
          bool(seen) and seen[-1][0] == "203.0.113.7"
          and seen[-1][2] == "不发 switch_controller", str(seen[-1:]))

    p2 = _Probe()
    ok2 = board_gate._run_repair(_args(), p2, [C], isatty=lambda: False,
                                 loop=_loop_no, out=quiet)
    check("  核验不过 ⇒ **一个字节都不发**（fail-closed，不是「那就当确认了」）",
          ok2 is False and p2.switched == [], f"ok={ok2} switched={p2.switched}")

    # 站点没配好、没给 --dashboard-host ⇒ 核验不了 ⇒ 也不发（**不许**替它猜个 IP）
    p2b = _Probe()
    ok2b = board_gate._run_repair(
        _args(dashboard_host=None), p2b, [C], isatty=lambda: False,
        loop=lambda ip, timeout, **kw: (ip is not None,
                                        "过了" if ip else "没给控制柜 IP"),
        out=quiet)
    check("  没给 `--dashboard-host` ⇒ 核验不了 ⇒ 也不发（不许替它猜一个 IP）",
          ok2b is False and p2b.switched == [])

    # 有终端、却读不出确认（EOF）⇒ 拒。input 必须先堵死，否则真终端上会**卡住**。
    real_input = builtins.input
    p3 = _Probe()
    builtins.input = lambda *a, **k: (_ for _ in ()).throw(EOFError())
    try:
        ok3 = board_gate._run_repair(_args(), p3, [C], isatty=lambda: True,
                                     loop=_loop_ok, out=quiet)
    finally:
        builtins.input = real_input
    check("  说有终端、却读不出确认（EOF）⇒ 拒，且**一次都没发**",
          ok3 is False and p3.switched == [], f"ok={ok3} switched={p3.switched}")

    # 变异：把「没有终端就走示教器」这一支短路掉，AI 驱动那条路必须**当场消失**。
    src = open(os.path.join(HERE, "board_gate.py"), encoding="utf-8").read()
    m = _mutate(src, "    if not isatty():", "    if False:  # MUT", "m6")
    if m is not None:
        pm = _Probe()
        builtins.input = lambda *a, **k: (_ for _ in ()).throw(EOFError())
        try:
            got = m._run_repair(_args(), pm, [C], isatty=lambda: False,
                                loop=_loop_ok, out=quiet)
        finally:
            builtins.input = real_input
        check("M6 短路掉示教器那一支 ⇒ AI 驱动只能去敲键盘、于是发不出去（与改前不同）",
              ok is True and got is False and pm.switched == [],
              f"改前={ok} 改后={got} switched={pm.switched}")

    check("  源码里**不再**有「非终端就直接拒」的旧文案（它把 AI 驱动整条路堵死）",
          "本脚本**没有**无人值守模式" not in src)
    check("  两档走的是**同一个**握手件（board_lib.ensure_in_the_loop），不另抄一套",
          "board_lib.ensure_in_the_loop" in src)
    check("  默认判据就是 sys.stdin.isatty（谁改了默认值这里立刻红）",
          "isatty = sys.stdin.isatty if isatty is None else isatty" in src)
    check("  --dashboard-host / --play-timeout 是命令行参数（AI 得能把站点 IP 递进来）",
          '"--dashboard-host"' in src and '"--play-timeout"' in src)


# ------------------------------------------------- 5. 离线跑一遍 main()
def section_cli():
    print("\n[5] 命令行行为（离线）")

    def run(argv):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = board_gate.main(argv)
        return rc, buf.getvalue()

    rc, out = run(["--list-items"])
    check("--list-items 退 0 并列出六个项名",
          rc == 0 and all(n in out for n in board_gate.ITEM_NAMES))

    rc, out = run(["--allow-skip", "controllers"])
    check("--allow-skip controllers 被拒（退 2）", rc == 2, f"rc={rc}")

    rc, out = run(["--allow-skip", "tamlost"])
    check("--allow-skip 写错项名被拒（退 2，且提示可用项）",
          rc == 2 and board_gate.ITEM_NAMES[0] in out, f"rc={rc}")

    rc, out = run(["--pick-zone", "1,2,3"])
    check("--pick-zone 个数不对被拒（退 2）", rc == 2, f"rc={rc}")

    # 离线整跑：这台机器没有 rclpy ⇒ 必须退 2（fail-closed），**绝不能是 0**
    fast = ["--discover-timeout", "0.2", "--controller-timeout", "0.2",
            "--camera-window", "0.2", "--targets-timeout", "0.2",
            "--telemetry-timeout", "0.2"]
    rc, out = run(fast)
    # 只探「装没装」，**不 import** —— 这台机器上 import rclpy 会失败，而我们要的是
    # 一个布尔值，不是一次失败。
    has_ros = importlib.util.find_spec("rclpy") is not None
    if has_ros:
        check("有 ROS 时整跑退 0/1/2 之一（这台机器有 ROS，栈不一定在）",
              rc in (0, 1, 2), f"rc={rc}")
    else:
        check("离线整跑退 2（fail-closed）——**绝不能退 0**", rc == 2, f"rc={rc}")
        check("  并说清楚是 import 不了 rclpy",
              mentions(out, "rclpy"))
    check("  ROS_DOMAIN_ID 被打印出来（漏配会看到空图，最容易误判）",
          "ROS_DOMAIN_ID" in out)


def main():
    print("=" * 70)
    print("  board_gate 自检（纯离线：不碰 ROS / 臂 / 串口）")
    print("=" * 70)
    section_judge()
    section_three_states()
    section_mutation()
    section_source_pins()
    section_repair_authorization()
    section_cli()
    print("\n" + "=" * 70)
    if _failed:
        print(f"  FAIL：{len(_failed)} 项没过")
        for f in _failed:
            print("    ✗ " + f)
        return 1
    print("  全通过：三态分得开、每条判据的理由说得对，")
    print("  且四处变异都让结论真的翻面（断言是吃劲的，不是恰好为真）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""整轮编排器（``stage4_round_run.py``）的自检：**不连板子、不动臂、不做 IK**。

这一份测的是**编排与闸门**，不是几何：
· 几何（机位、抓放、动作表）由 ``stage4_clear_run`` 自己的测试把守；
· 这里把 ``plan_views`` / ``plan_sequence`` / ``actions_from_attempt`` 换成
  写假计划文件的桩，板子换成 ``FakeBoard``，于是「哪一阶段先跑、令牌不够在
  哪停、失败了会不会偷偷重试、b̂ 符号朝哪边」这些**只有编排器负责**的事，
  可以逐条钉死。

钉住的事：

A 纯函数：估时、R1/R2 边界、b̂ 种子三种来源、解垫文档校验、预检全绿/逐条红、
  facts 不伪造安全数字、关节排序；
B 布局表与 ``gen_scene`` 逐块一致（「布局即真值」的前提）；
C 阶段编排：
  1) 干跑只到阶段 4，不执行；
  2) 全绿一趟：八个阶段全落、run.sh 只跑**一次**、令牌守卫线在脚本里；
  3) R1 不足 ⇒ 停在 compute（rc 3），一条执行都没发；
  4) R1 够、R2 不足 ⇒ 同样不发执行（rc 3）；
  5) 预检红（夹爪未初始化）⇒ rc 2，连授权都不进入；
  6) 缺 b̂ 种子 ⇒ rc 2，不发执行；
  7) 执行失败 ⇒ rc 5，**不自动重试**（run.sh 只调一次）；
  8) 执行中令牌过期 ⇒ rc 3 且本轮 ``void=true``（aborted_token，不记抓取失败）；
  9) 判分不完整 ⇒ rc 6；
 10) 探针报清障 / 格位不符 ⇒ rc 4，不执行；
 11) ``--from/--to`` 只跑指定阶段。

跑法：``python3 scripts/test_stage4_round_run.py``；退出码 0 = 全通过。
"""

import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
import types

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_clear_model as cm                                 # noqa: E402
import stage4_clear_run as cr                                   # noqa: E402
import stage4_layouts as lay                                    # noqa: E402
import stage4_round_run as r                                    # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=""):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print("  PASS  %s" % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print("  FAIL  %s%s" % (name, ("\n        " + detail) if detail else ""))


def quiet(fn, *a, **kw):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = fn(*a, **kw)
    return rc, buf.getvalue()


# =====================================================================
print("--- A. 纯函数 ---")

with tempfile.TemporaryDirectory() as td:
    good = os.path.join(td, "a.json")
    with open(good, "w") as f:
        json.dump({"waypoints": [{"duration": 10.0}, {"duration": 5.0}]}, f)
    view = os.path.join(td, "b.json")
    with open(view, "w") as f:
        json.dump({"waypoints": [{"duration": 2.0}]}, f)
    est = r.exec_estimate([
        {"path": good, "kind": "target"},
        {"path": view, "kind": "view", "view": "pick"}])
    # 15 + (30+1) + 2 + (30+1) + 1×6 = 85（每份计划 30 s：停稳/夹爪/进程启动，
    # 2026-11-12 实测值；旧值 5 s 会把执行估低一半，闸门形同虚设）
    check("exec_estimate：Σduration + 每份开销 + burst 开销", abs(est - 85.0) < 1e-6,
          repr(est))
    miss = os.path.join(td, "nope.json")
    try:
        r.exec_estimate([{"path": miss, "kind": "target"}])
        check("exec_estimate：读不了计划 ⇒ 抛（不许当 0 用）", False)
    except OSError:
        check("exec_estimate：读不了计划 ⇒ 抛（不许当 0 用）", True)

check("r1_need = 计算 + 执行 + 余量", abs(r.r1_need(200, 300) - 560.0) < 1e-6)
ok, msg = r.r1_gate(560, 200, 300)
check("R1 恰好够 ⇒ 通过", ok, msg)
ok, msg = r.r1_gate(559.9, 200, 300)
check("R1 差 0.1 秒 ⇒ 不过（边界真的在判）", not ok, msg)
ok, msg = r.r1_gate(None, 200, 300)
check("R1 没有令牌 ⇒ 不过（fail-closed）", not ok and "STOP" in msg, msg)

check("R2 余量 = max(45, 15%)", r.r2_margin(100) == 45.0 and
      abs(r.r2_margin(400) - 60.0) < 1e-9)
ok, _ = r.r2_gate(160, 100)
check("R2 100 执行 + 45 余量 = 145，剩 160 ⇒ 通过", ok)
ok, _ = r.r2_gate(144, 100)
check("R2 剩 144 < 145 ⇒ 不过", not ok)
ok, _ = r.r2_gate(None, 100)
check("R2 没有令牌 ⇒ 不过", not ok)

b = r.bias_setup("fixed", "-2.791,-0.248", None)
check("b̂ fixed 显式种子：命令偏置 = 种子",
      b["mode"] == "fixed" and b["applied_mm"] == [-2.791, -0.248], repr(b))
b0 = r.bias_setup("none", "-2.791,-0.248", None)
check("b̂ none：显式给种子也不补（[0,0] + 标未补偿）",
      b0["mode"] == "none" and b0["applied_mm"] == [0.0, 0.0]
      and "none" in b0["note"], repr(b0))
try:
    r.bias_setup("fixed", None, "/no/such/estimate.json")
    check("b̂ 缺种子且非 none ⇒ 停轮（RoundError 2）", False)
except r.RoundError as e:
    check("b̂ 缺种子且非 none ⇒ 停轮（RoundError 2）", e.code == r.EXIT_MISSING)
try:
    r.bias_setup("adaptive", "1,1", None)
    check("b̂ 不收 adaptive（与整轮一次成脚本冲突）", False)
except ValueError:
    check("b̂ 不收 adaptive（与整轮一次成脚本冲突）", True)

with tempfile.TemporaryDirectory() as td:
    est_path = os.path.join(td, "bias_estimate.json")
    with open(est_path, "w") as f:
        json.dump({"version": 3, "model": "constant_pre_nudge",
                   "bias_mm": [-2.5, -0.1], "n": 40}, f)
    b2 = r.bias_setup("fixed", None, est_path)
    check("b̂ 缺 --bias-seed 时读 bias_estimate.json",
          b2["applied_mm"] == [-2.5, -0.1] and b2["source"] == est_path, repr(b2))
    with open(est_path, "w") as f:
        json.dump({"version": 1, "bias_mm": [-2.5, -0.1], "n": 40}, f)
    try:
        r.bias_setup("fixed", None, est_path)
        check("b̂ 版本不符的估计当没有 ⇒ 停轮", False)
    except r.RoundError:
        check("b̂ 版本不符的估计当没有 ⇒ 停轮", True)

pad_ok = {"kind": "stage2-grid", "n_frames_ok": 20, "n_frames_total": 20,
          "spread_xy_mm": {"max": 1.0}, "spread_yaw_deg": {"max": 0.2},
          "grid_center_base_mm": [424.2, -112.24],
          "grid_yaw_base_deg": -179.778}
check("解垫文档合格 ⇒ 无问题", r.validate_pad_doc(pad_ok) == [])
check("解垫：帧数不足 ⇒ 红",
      any("帧" in x for x in
          r.validate_pad_doc(dict(pad_ok, n_frames_ok=10))))
check("解垫：xy 极差超限 ⇒ 红",
      any("xy" in x for x in
          r.validate_pad_doc(dict(pad_ok, spread_xy_mm={"max": 5.0}))))
check("解垫：kind 不对 ⇒ 红",
      any("kind" in x for x in r.validate_pad_doc(dict(pad_ok, kind="x"))))

EXPECT = {"serial": "SN-1", "tree_sha256": "abc123", "tree_files": 7,
          "stack": {"ur_robot_driver": True}}
GOT = {"serial": "SN-1", "tree": {"sha256": "abc123", "files": 7},
       "gripper": {"ok": True, "init_state": 1},
       "realsense": [{"dev": "1-1", "speed": "5000"}],
       "dashboard": {"ok": True, "state": "PLAYING"},
       "stack": {"ur_robot_driver": True}}
check("预检全绿 ⇒ 无问题", r.evaluate_preflight(GOT, EXPECT) == [])
check("预检缺 --expect ⇒ 红（不知道是哪块板）",
      bool(r.evaluate_preflight(GOT, None)))
check("预检：序列号不符 ⇒ 红",
      bool(r.evaluate_preflight(dict(GOT, serial="SN-2"), EXPECT)))
check("预检：代码树哈希不符 ⇒ 红",
      bool(r.evaluate_preflight(
          dict(GOT, tree={"sha256": "ffff", "files": 7}), EXPECT)))
check("预检：夹爪 0x0200=0 ⇒ 红（未初始化，不得自动初始化）",
      bool(r.evaluate_preflight(
          dict(GOT, gripper={"ok": True, "init_state": 0}), EXPECT)))
check("预检：腕部相机在 USB2 ⇒ 红",
      bool(r.evaluate_preflight(
          dict(GOT, realsense=[{"dev": "1-1", "speed": "480"}]), EXPECT)))
check("预检：栈里没驱动 ⇒ 红",
      bool(r.evaluate_preflight(dict(GOT, stack={}), EXPECT)))
# dashboard 死在令牌窗口里是纯浪费（2026-11-12 实测白等 147 s）⇒ 预检必须拦。
check("预检：dashboard 没响应 ⇒ 红（不许把卡死带进令牌窗口）",
      any("dashboard" in x for x in r.evaluate_preflight(
          dict(GOT, dashboard={"ok": False, "state": "unknown"}), EXPECT)))
check("预检：没查 dashboard ⇒ 红（旧版预检脚本不算绿）",
      any("dashboard" in x for x in r.evaluate_preflight(
          {k: v for k, v in GOT.items() if k != "dashboard"}, EXPECT)))
# 序列号真值不许出现在问题文本/日志里（AGENTS.md 红线 3）
_msg = "；".join(r.evaluate_preflight(dict(GOT, serial="SN-2"), EXPECT))
check("预检：序列号不符只打指纹（真值不进文本）",
      "SN-2" not in _msg and "SN-1" not in _msg and
      r.serial_fp("SN-2") in _msg, _msg)
check("预检：空树（路径对不上）⇒ 红，不靠期望值蒙过去",
      any("0 个文件" in x for x in r.evaluate_preflight(
          dict(GOT, tree={"sha256": "e3b0c442", "files": 0}),
          {"serial": "SN-1", "stack": {"ur_robot_driver": True}})))

# 树哈希只覆盖源码：板子上跑一次 import 就多个 .pyc、跑一次构建就变一堆产物，
# 这些都不该让「这棵树是哪棵」变味（2026-11-11 实测：`--help` 跑一次，
# 文件数 1129→1130、哈希全变 ⇒ 预检假红，把「部署变了」和「有人 import 过」混淆）
def _run_preflight(repo):
    # 只跑共享的树哈希代码（`PREFLIGHT_TREE_CODE` 与板上预检同一份）；跑整份
    # PREFLIGHT_PY 会去开夹爪串口 / 查 dashboard，本机白等 ~28 s。
    src = ("import hashlib\nimport json\nimport os\nimport sys\n"
           + r.PREFLIGHT_TREE_CODE
           + "\nprint(json.dumps(scan_tree(sys.argv[1])))\n")
    p = subprocess.run([sys.executable, "-", repo], input=src,
                       capture_output=True, text=True)
    return {"tree": json.loads(p.stdout)}


with tempfile.TemporaryDirectory() as _tt:
    os.makedirs(os.path.join(_tt, "scripts"))
    with open(os.path.join(_tt, "scripts", "a.py"), "w") as _f:
        _f.write("x = 1\n")
    _p1 = _run_preflight(_tt)
    os.makedirs(os.path.join(_tt, "scripts", "__pycache__"))
    with open(os.path.join(_tt, "scripts", "__pycache__", "a.pyc"), "wb") as _f:
        _f.write(b"\x00junk")
    os.makedirs(os.path.join(_tt, "ros2_ws", "build", "pkg"))
    with open(os.path.join(_tt, "ros2_ws", "build", "pkg", "o.o"), "wb") as _f:
        _f.write(b"obj")
    os.makedirs(os.path.join(_tt, "scripts.bak-20260101"))
    with open(os.path.join(_tt, "scripts.bak-20260101", "old.py"), "w") as _f:
        _f.write("y = 2\n")
    _p2 = _run_preflight(_tt)
    check("树哈希只数源码：.pyc / 构建产物 / .bak 目录都不进（否则假红）",
          _p1["tree"] == _p2["tree"], repr((_p1["tree"], _p2["tree"])))
    check("树哈希：源码 1 个文件就数 1", _p1["tree"]["files"] == 1,
          repr(_p1["tree"]))

facts = r.build_facts("S0-1-r1", "S0", 1, lay.layout_blocks("S0", 1),
                      {"mode": "none", "applied_mm": [0, 0]})
check("facts：人没给安全数字 ⇒ 不伪造（不写 safety 键）",
      "safety" not in facts and facts["auto_top1"] is None)
facts2 = r.build_facts("S0-1-r1", "S0", 1, lay.layout_blocks("S0", 1),
                       {"mode": "none", "applied_mm": [0, 0]},
                       {"safety": {"collisions": 0}, "interventions":
                        {"count": 1}, "human_review": {"required": False}})
check("facts：人给了就原样并入", facts2["safety"]["collisions"] == 0
      and facts2["interventions"]["count"] == 1)
check("facts：specs 按块 id 记颜色/边长",
      facts["specs"]["0"]["color"] == "yellowgreen" and
      abs(facts["specs"]["0"]["size_m"] - cm.BLOCK_M) < 1e-12)

names = ["b", "10", "a", "2"]
check("_sorted_ids：数字按数值、其余按串",
      r._sorted_ids(names) == ["2", "10", "a", "b"], repr(r._sorted_ids(names)))
cells = cm.parse_cells("8,6,4,2,0", 5)
amap = cm.assign_cells([4, 0, 3, 1, 2], cells)
check("assign_cells：id 升序 → 8,6,4,2,0",
      amap == {"0": 8, "1": 6, "2": 4, "3": 2, "4": 0}, repr(amap))

chain = types.SimpleNamespace(move_joints=["j2", "j1"])
doc = {"name": ["j1", "j2"], "position": [0.2, 0.1]}
check("_q_from_joints 按链的关节顺序重排",
      r._q_from_joints(doc, chain) == [0.1, 0.2])
try:
    r._q_from_joints({"name": ["j1"], "position": [0.2]}, chain)
    check("_q_from_joints 缺关节 ⇒ 停（不猜 0）", False)
except r.RoundError:
    check("_q_from_joints 缺关节 ⇒ 停（不猜 0）", True)

# =====================================================================
print("--- B. 布局表 = gen_scene（布局即真值的唯一出处） ---")
check("布局表与 gen_scene 逐块一致", lay.compare_with_gen_scene("S0") == [],
      repr(lay.compare_with_gen_scene("S0")))
errs = []
for k in sorted(lay.BATCHES["S0"]):
    errs += lay.validate_layout("S0", k)
check("三个布局自检（区内 + 间距 ≥60 mm）", errs == [], repr(errs))

# =====================================================================
print("--- C. 阶段编排（FakeBoard + 打桩规划） ---")

PLAN_CALLS = []
DEVIATION_CALLS = []


def _write_plan(path, duration=10.0):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump({"waypoints": [{"duration": duration},
                                 {"duration": duration}]}, f)


def fake_plan_views(chain_, q, views, out_dir, start=0):
    plans = []
    os.makedirs(out_dir, exist_ok=True)
    for i, v in enumerate(views, start=int(start) + 1):
        name = "%02d-view-%s.json" % (i, v["name"])
        path = os.path.join(out_dir, name)
        _write_plan(path)
        plans.append({"name": name, "path": path, "kind": "view",
                      "view": v["name"], "slot": [0.5, 0.3],
                      "n_waypoints": 2})
    return plans, list(q)


def _act_rec(a, blocks):
    """一条动作 + 它在**传入布局**里的块位置（抓点是否用实测只看这里）。

    桩动作表的块键是 ``block``（真实现）或 ``id``（本文件 ``fake_actions_*``），
    两种都认。
    """
    bid = a.get("block", a.get("id"))
    blk = cr.find_block(blocks, bid) if bid is not None else None
    return {"kind": a.get("kind"), "block": bid,
            "block_xy": (None if blk is None else
                         [float(blk["x"]), float(blk["y"])]),
            "from_xy": a.get("from_xy")}


def fake_plan_sequence(chain_, q, actions, blocks, out_dir,
                       survey_tool0=None, place_clearance=None,
                       wrist_survey=True, views=None, place_yaw_equiv=False,
                       start=0, min_block_gap=0.0, gap_check="probe_verified",
                       tail_targets=None):
    PLAN_CALLS.append({"start": start, "wrist_survey": wrist_survey,
                       "gap_check": gap_check, "min_block_gap": min_block_gap,
                       "n_actions": len(actions),
                       # 末块的收尾支：编排器必须把机位目标交给**最后一个**
                       # 动作，否则收尾 IK 支可能与机位链分属不同折圈族。
                       "tail_targets": tail_targets,
                       # 每个动作抓的那块在**传入布局**里的位置 —— 抓点是否
                       # 用实测，只能从这里看出来（plan 桩不跑几何）。
                       "acts": [_act_rec(a, blocks) for a in actions]})
    plans = []
    os.makedirs(out_dir, exist_ok=True)
    for i, a in enumerate(actions):
        name = "%02d-%s-block-%s.json" % (int(start) + i, a["kind"], a["id"])
        path = os.path.join(out_dir, name)
        _write_plan(path)
        plans.append({"name": name, "path": path, "kind": a["kind"],
                      "slot": a.get("slot"), "n_waypoints": 2})
    new_blocks = [dict(b) for b in blocks]
    for a in actions:
        if a["kind"] == "target":
            for b in new_blocks:
                if str(b["id"]) == str(a["id"]):
                    b["x"], b["y"] = a.get("commanded_center_mm", [0, 0])
                    b["x"] /= 1000.0
                    b["y"] /= 1000.0
    return plans, new_blocks, list(q)


def fake_actions_from_attempt(truth, attempt, pad_thick, bias_mm=None):
    bias = bias_mm or [0.0, 0.0]
    cell = int(attempt["cell"])
    cx, cy = cm.cell_center(cell)
    desired = [cx * 1000.0, cy * 1000.0]
    cmd = [desired[0] - bias[0], desired[1] - bias[1]]
    blk = cr.find_block(truth["blocks"], attempt["id"])
    act = {"kind": "target", "id": attempt["id"],
           # 与真实现同约定：动作自带抓点 ``from_xy``（实测/探针/fold 后）。
           "from_xy": [float(blk["x"]), float(blk["y"])],
           "slot": [cmd[0] / 1000.0,
                    cmd[1] / 1000.0],
           "desired_center_mm": [round(v, 3) for v in desired],
           "commanded_center_mm": [round(v, 3) for v in cmd],
           "applied_bias_mm": [round(v, 3) for v in bias]}
    return [act], truth["blocks"]


class Rc:
    def __init__(self, code=0, out="", err=""):
        self.returncode = code
        self.stdout = out
        self.stderr = err


# pull_dir 必须走 rsync：``scp -r host:path/. local`` 在新版 OpenSSH（SFTP 后端）
# 上直接报 ``error: unexpected filename: .`` 却返回 0 ⇒ 静默 0 帧。
# 2026-11-11 Round 0 实测踩到：板上 20 帧齐全，拉回 0 帧，烧掉一次真机授权。
_saved_run = r.subprocess.run
try:
    _calls = []
    r.subprocess.run = lambda cmd, **kw: (_calls.append(cmd), Rc())[1]
    _rd = object.__new__(r.RoundBoard)
    _rd.alias = "fake"
    with tempfile.TemporaryDirectory() as _td:
        _rd.pull_dir("~/x/y", _td)
        check("pull_dir 用 rsync 且远端路径以 / 结尾",
              _calls[0][0] == "rsync" and _calls[0][-1].endswith("/"),
              repr(_calls[0]))
        r.subprocess.run = lambda cmd, **kw: Rc(23, err="boom")
        try:
            _rd.pull_dir("~/x/y", _td)
            check("pull_dir 拉失败 ⇒ 抛错（不静默 0 帧）", False)
        except r.RoundError as e:
            check("pull_dir 拉失败 ⇒ 抛错（不静默 0 帧）",
                  e.code == r.EXIT_MISSING)
finally:
    r.subprocess.run = _saved_run


class FakeBoard:
    def __init__(self):
        self.calls = []
        self.pulls = []
        self.files = {}
        self.pushed = []
        self.preflight_doc = {
            "serial": "SN-1", "tree": {"sha256": "abc123", "files": 7},
            "gripper": {"ok": True, "init_state": 1},
            "realsense": [{"dev": "1-1", "speed": "5000"}],
            "dashboard": {"ok": True, "state": "PLAYING"},
            "stack": {"ur_robot_driver": True}}
        self.tokens = [(("t" * 32), 800.0, "")]
        self._last_token = (None, None, "没有令牌")
        self.pre_results = [(0, "PRE DONE")]
        self.run_results = [(0, "ALL DONE")]

    def _next(self, seq, default):
        return seq.pop(0) if seq else default

    def ssh(self, cmd, quiet=True, check=False):
        # 编排器只用 ssh 做 chmod 这类收尾动作；记录即可（真机路径由 RoundBoard 管）
        self.calls.append(("ssh", cmd))
        return Rc(0)

    def preflight(self, repo):
        self.calls.append(("preflight", repo))
        return self.preflight_doc

    def mkdir(self, remote):
        self.calls.append(("mkdir", remote))

    def clear(self, remote):
        self.calls.append(("clear", remote))

    def joints(self, dest):
        self.calls.append(("joints", dest))
        os.makedirs(dest, exist_ok=True)
        doc = {"name": ["j1", "j2"], "position": [0.1, 0.2]}
        with open(os.path.join(dest, "joint_states.json"), "w") as f:
            json.dump(doc, f)
        return doc

    def token_state(self):
        # 队列用完就**重复最后一条**（真实令牌在 600 s 内一直有效，不是一次性的）
        if self.tokens:
            self._last_token = self.tokens.pop(0)
        tok, rem, why = self._last_token
        self.calls.append(("token_state", rem))
        return tok, rem, why

    def wait_token(self, timeout_s, poll_s, now=None, sleep=None, out=None,
                   fresh_min=None):
        self.calls.append(("wait_token", timeout_s, fresh_min))
        return ("t" * 32, 780.0)

    def push(self, local, remote, label=None):
        self.pushed.append(remote)

    def push_many(self, pairs):
        for local, remote in pairs:
            self.push(local, remote)

    def pull_file(self, remote, local):
        self.pulls.append(("file", remote))
        self.calls.append(("pull_file", remote))
        os.makedirs(os.path.dirname(os.path.abspath(local)) or ".",
                    exist_ok=True)
        val = self.files.get(remote)
        with open(local, "w", encoding="utf-8") as f:
            if isinstance(val, (dict, list)):
                json.dump(val, f)
            elif val is not None:
                f.write(str(val))
            else:
                f.write("")

    def pull_dir(self, remote, local):
        self.pulls.append(("dir", remote))
        self.calls.append(("pull_dir", remote))
        os.makedirs(local, exist_ok=True)
        with open(os.path.join(local, "color_000.png"), "wb") as f:
            f.write(b"\x89PNG")

    def probe(self, repo, cells, group, truth_remote, grid_remote,
              report_remote):
        self.calls.append(("probe", cells))
        self.files[report_remote] = self.probe_report()
        return Rc(0)

    def probe_report(self):
        blocks = lay.layout_blocks("S0", 1)
        ids = r._sorted_ids([b["id"] for b in blocks])
        cmap = cm.assign_cells(ids, cm.parse_cells("8,6,4,2,0"))
        attempts = [{"id": int(i), "ok": True, "cell": cmap[i],
                     "clearing_steps": [], "reason": None,
                     "terminal": {"ok": True}}
                    for i in ids]
        return {"scenes": [{"attempts": attempts}], "group": "ur_manipulator",
                "scene_cleanup": {"ok": True, "world": [], "attached": []}}

    def run_script(self, remote, token):
        self.calls.append(("run_script", remote))
        if remote.endswith("pre.sh"):
            return self._next(self.pre_results, (0, "PRE DONE"))
        return self._next(self.run_results, (0, "ALL DONE"))


def make_args(td, **over):
    layout = over.pop("layout", 1)
    bias_seed = over.pop("bias_seed", "-2.791,-0.248")
    argv = ["--batch", "S0", "--layout", str(layout),
            "--run", "1",
            "--out-dir", os.path.join(td, "run", "s0-1-r1"),
            "--urdf", os.path.join(td, "robot.urdf"),
            "--board", "fake", "--board-repo", "/repo",
            "--board-dir", "/b",
            "--expect", os.path.join(td, "expect.json")]
    if bias_seed is not False:
        argv += ["--bias-seed", str(bias_seed)]
    for key, val in over.items():
        if val is False:
            continue
        # ``from_`` 是 Python 侧的名字，命令行旗标叫 ``--from``；照着
        # ``key.replace("_", "-")`` 机械转换会造出 ``--from-``（argparse 直接
        # SystemExit(2)，连汇总行都不打 —— 2026-11-11 踩过）。
        flag = "--from" if key == "from_" else "--" + key.replace("_", "-")
        if val is True:
            argv.append(flag)
        else:
            argv += [flag, str(val)]
    ap = r.build_parser()
    args = r.prepare(ap, r.parse_cli(ap, argv))
    return args


def write_expect(td):
    with open(os.path.join(td, "expect.json"), "w") as f:
        json.dump(EXPECT, f)


# 板上镜像目录缺省值：Round 与 main() 必须同源（2026-11-11 不给 --board-dir
# 时 main() 曾把 None 交给 RoundBoard，崩在 None.rstrip）
_td_bb = tempfile.mkdtemp()
_out_bb = os.path.join(_td_bb, "s0-1-r0")
os.makedirs(_out_bb, exist_ok=True)
_dflt = r.default_board_dir(_out_bb)
check("default_board_dir = ~/precision/e9/<run>/<round>",
      _dflt == "~/precision/e9/%s/s0-1-r0" % os.path.basename(_td_bb), repr(_dflt))
write_expect(_td_bb)
_a_bb = make_args(_td_bb, run=0, out_dir=_out_bb)
_a_bb.board_dir = ""            # 真实情形：命令行没给 --board-dir
_rd_bb = r.Round(_a_bb, types.SimpleNamespace(alias="x", exp_dir=_td_bb))
check("Round 解析的 board_dir 与 default_board_dir 一致",
      _rd_bb.board_dir == _dflt, repr(_rd_bb.board_dir))
try:
    r.RoundBoard("x", "", _td_bb)
    check("RoundBoard 空目录 ⇒ 明确报错（不是 None.rstrip）", False)
except r.RoundError:
    check("RoundBoard 空目录 ⇒ 明确报错（不是 None.rstrip）", True)

# 令牌新鲜度：没过期 ≠ 这一次授权（红线 6 逐次授权；2026-11-11 Round 0 实测
# 沿用上一道令牌 479 s，人根本没按 STOP/PLAY）
_rb = object.__new__(r.RoundBoard)
_seq = [("a" * 32, 479.0, ""), ("b" * 32, 598.0, "")]


def _tok_state():
    return _seq.pop(0) if _seq else ("b" * 32, 598.0, "")


_rb.token_state = _tok_state
_clock = [0.0]
_rc, _out = quiet(_rb.wait_token, 30, 1, now=lambda: _clock[0],
                  sleep=lambda s: _clock.__setitem__(0, _clock[0] + s),
                  fresh_min=540.0)
check("等待授权：旧的未过期令牌不算，等到新签发的才成立",
      _rc == ("b" * 32, 598.0), repr(_rc))
check("等待授权：说明写着「不是这一次签发的」",
      "不是这一次签发" in _out, _out[-200:])

_rb2 = object.__new__(r.RoundBoard)
_rb2.token_state = lambda: ("a" * 32, 479.0, "")
_c2 = [0.0]


def _now2():
    _c2[0] += 1.0
    return _c2[0]


try:
    quiet(_rb2.wait_token, 3, 1, now=_now2, sleep=lambda s: None,
          fresh_min=540.0)
    check("等待授权：只有旧令牌 ⇒ 超时停（RoundError 3）", False)
except r.RoundError as e:
    check("等待授权：只有旧令牌 ⇒ 超时停（RoundError 3）",
          e.code == r.EXIT_GATE)


# 板上探针命令：``~/precision/run.sh`` 自己就是 ``exec python3 "$@"``，
# 后面再写一个 python3 会被当成「要打开的脚本名」（2026-11-11 Round 0 实测
# rc=2、`can't open file '<repo>/python3'`，白烧一次真机授权）
class _SshLog:
    def __init__(self):
        self.cmds = []

    def ssh(self, cmd, quiet=True, check=False):
        self.cmds.append(cmd)
        return types.SimpleNamespace(returncode=0, stdout="", stderr="")


def _probe_cmd(repo="~/yolo_ros_original"):
    rb = object.__new__(r.RoundBoard)
    rb._b = _SshLog()
    rb.probe(repo, "8,6,4,2,0", "ur_manipulator",
             "~/precision/e9/x/round0/truth/layout.json",
             "~/precision/e9/x/round0/grid-pre.json",
             "~/precision/e9/x/round0/probe.json")
    return rb._b.cmds[-1]


_pcmd = _probe_cmd()
check("探针命令：&& 之后直接是 run.sh + 脚本（不再传 python3）",
      _pcmd.split("&&", 1)[1].strip().startswith(
          "~/precision/run.sh scripts/stage4_clear_probe.py"), _pcmd)
check("探针命令：仓库路径只出现在 cd 里",
      _pcmd.count("~/yolo_ros_original") == 1 and
      _pcmd.startswith("cd ~/yolo_ros_original && "), _pcmd)
check("探针命令：~ 路径没被单引号焊死（否则板端不展开）",
      "'~" not in _pcmd and '"~' not in _pcmd, _pcmd)
try:
    _probe_cmd(repo="~/repo with space")
    check("探针命令：路径含空格 ⇒ 明确报错（不静默拼错命令）", False)
except r.RoundError as e:
    check("探针命令：路径含空格 ⇒ 明确报错（不静默拼错命令）",
          e.code == r.EXIT_MISSING)


# 板上分析命令（解垫 / 真值 / 彩色）：与探针同一条 ``~/precision/run.sh`` 通路，
# 只回传 JSON。2026-11-12 用户定调：本机不承担运行时任务。
class _SshRc(_SshLog):
    def __init__(self, rc=0):
        super().__init__()
        self.rc = rc

    def ssh(self, cmd, quiet=True, check=False):
        self.cmds.append(cmd)
        return types.SimpleNamespace(returncode=self.rc, stdout="", stderr="x")


def _analysis_cmds(kind="truth", opts=None, repo="~/stage4-entry", rc=0,
                   pull_log=None):
    rb = object.__new__(r.RoundBoard)
    rb._b = _SshRc(rc)
    rb.pull_file = (lambda remote, local: pull_log.append(remote)) \
        if pull_log is not None else (lambda remote, local: None)
    out = "~/precision/e9/x/r/analysis/%s-a.json" % kind
    rb.analyze(repo, kind,
               list(opts or ["--burst", "~/precision/e9/x/r/pre/wrist-pick"]),
               out, "/tmp/unused.json")
    return rb._b


_acmds = _analysis_cmds().cmds
_acmd = _acmds[-1]
check("板上分析命令：先建板上目录再跑", _acmds[0].startswith("mkdir -p "),
      repr(_acmds))
check("板上分析命令：&& 之后是 run.sh + 板上脚本",
      _acmd.split("&&", 1)[1].strip().startswith(
          "~/precision/run.sh scripts/stage4_board_analysis.py truth"), _acmd)
check("板上分析命令：仓库路径只出现在 cd 里、~ 没被引号焊死",
      _acmd.startswith("cd ~/stage4-entry && ") and "'~" not in _acmd, _acmd)
check("板上分析命令：--out 是板上路径（回传后本地才有产物）",
      "--out ~/precision/e9/x/r/analysis/truth-a.json" in _acmd, _acmd)
try:
    _analysis_cmds(opts=["--burst", "~/a b"])
    check("板上分析：参数含空格 ⇒ 明确报错", False)
except r.RoundError as e:
    check("板上分析：参数含空格 ⇒ 明确报错", e.code == r.EXIT_MISSING)
_pulled = []
_rb_rc = _analysis_cmds(rc=7, pull_log=_pulled)
check("板上分析：rc≠0 不拉产物、也不抛（由阶段决定怎么处置）",
      _pulled == [] and _rb_rc.rc == 7,
      repr((_pulled, _rb_rc.rc)))


def patched(deps):
    """装上规划桩与 deps；返回 (undo, deps)。"""
    saves = {}

    def setm(obj, name, val):
        saves.setdefault((id(obj), name), (obj, name, getattr(obj, name)))
        setattr(obj, name, val)

    setm(r.urkin, "Chain", lambda urdf: types.SimpleNamespace(
        move_joints=["j1", "j2"]))
    setm(r.cr, "plan_views", fake_plan_views)
    setm(r.cr, "plan_sequence", fake_plan_sequence)
    setm(r.cr, "actions_from_attempt", fake_actions_from_attempt)
    setm(r.cr, "compare_layout",
         lambda man, post, **kw: {"ok": True, "n_blocks": len(post)})

    def undo():
        for (_, _), (obj, name, val) in saves.items():
            setattr(obj, name, val)

    return undo


def fake_solve_pad_writer():
    def f(burst, out, min_frames=20, max_spread_mm=2.0, max_spread_deg=0.5):
        with open(out, "w") as fh:
            json.dump(pad_ok, fh)
        return 0
    return f


TRUTH_CALLS = []


def fake_truth(burst, out, allow_empty=False):
    TRUTH_CALLS.append((burst, allow_empty))
    # 跑前 pick burst 现在喂**实测抓点**（``--truth-source measured``）：
    # 返回与布局表一一对应的块（偏移 0），否则计算段会按「量不出块」停轮。
    blocks = []
    if not allow_empty:
        blocks = [{"id": b["id"], "color": b["color"], "x": b["x"],
                   "y": b["y"], "yaw": b["yaw"], "size": b["size"],
                   "top": b["top"]} for b in lay.layout_blocks("S0", 1)]
    with open(out, "w") as f:
        json.dump({"blocks": blocks}, f)
    return 0


def fake_truth_offset(dx_mm, dy_mm):
    """跑前 pick burst 的实测块位姿 = 布局表 + 固定偏移（手摆偏差的替身）。"""
    def f(burst, out, allow_empty=False):
        TRUTH_CALLS.append((burst, allow_empty))
        blocks = []
        if not allow_empty:
            blocks = [{"id": b["id"], "color": b["color"],
                       "x": b["x"] + dx_mm / 1000.0,
                       "y": b["y"] + dy_mm / 1000.0,
                       "yaw": b["yaw"], "size": b["size"], "top": b["top"]}
                      for b in lay.layout_blocks("S0", 1)]
        with open(out, "w") as f:
            json.dump({"blocks": blocks}, f)
        return 0
    return f


def fake_colour(offset=(1.0, 0.5), ok=True):
    def f(burst, xy_mm, top_z, expect_class, hue_deg, search_mm):
        return {"ok": ok, "xy_mm": [xy_mm[0] + offset[0], xy_mm[1] + offset[1]],
                "hue_deg": 30.0, "n_frames": 8, "n_missed": 0}
    return f


def fake_deviation(argv):
    DEVIATION_CALLS.append(list(argv))
    return 0


def _opt_map(opts):
    """``["--burst", "x", "--allow-empty"]`` → ``{"--burst": "x", "--allow-empty": True}``。"""
    out, i = {}, 0
    while i < len(opts):
        if opts[i].startswith("--"):
            if i + 1 < len(opts) and not opts[i + 1].startswith("--"):
                out[opts[i]] = opts[i + 1]
                i += 2
                continue
            out[opts[i]] = True
        i += 1
    return out


def fake_analyze(d):
    """板上分析桩：把三个本地 fake 包成 ``analyze(kind, opts, ...)`` 接口。

    闭包引用 ``d`` **本体** ⇒ 测试后面覆盖 ``solve_pad`` / ``measure_colour``
    时，调用点拿到的是覆盖后的版本（``base_deps`` 先建 dict 再 update）。
    """
    def f(kind, opts, out_remote, out_local):
        kv = _opt_map(opts)
        os.makedirs(os.path.dirname(os.path.abspath(out_local)) or ".",
                    exist_ok=True)
        if kind == "pad":
            return d["solve_pad"](kv.get("--burst", ""), out_local,
                                  int(kv.get("--min-frames", 20)),
                                  float(kv.get("--max-spread-mm", 2.0)),
                                  float(kv.get("--max-spread-deg", 0.5)))
        if kind == "truth":
            return d["truth_from_view"](
                kv.get("--burst", ""), out_local,
                allow_empty=bool(kv.get("--allow-empty")))
        if kind == "colour":
            xy = [float(v) for v in kv["--xy"].split(",")]
            doc = d["measure_colour"](
                kv.get("--burst", ""), xy, float(kv["--top-z"]),
                kv.get("--expect-class"),
                float(kv["--hue"]) if kv.get("--hue") else None,
                float(kv.get("--search-mm", 25.0)))
            with open(out_local, "w") as fh:
                json.dump(doc, fh)
            return 0
        raise AssertionError("未知分析类型 %r" % kind)
    return f


def base_deps(**over):
    d = {"solve_pad": fake_solve_pad_writer(),
         "truth_from_view": fake_truth,
         "measure_colour": fake_colour(),
         "round_report": lambda argv: 0,
         "placement_deviation": fake_deviation,
         "score": lambda argv: 0,
         "now": (lambda: 1000.0),
         "sleep": (lambda s: None)}
    d["analyze"] = fake_analyze(d)
    d.update(over)
    return d


def run(td, board, deps=None, **over):
    write_expect(td)
    global PLAN_CALLS, DEVIATION_CALLS, TRUTH_CALLS
    PLAN_CALLS = []
    DEVIATION_CALLS = []
    TRUTH_CALLS = []
    undo = patched(deps)
    try:
        args = make_args(td, **over)
        rd = r.Round(args, board, deps=base_deps(**(deps or {})))
        rc, out = quiet(rd.run)
        if rc != 0 and os.environ.get("ROUND_TEST_DEBUG"):
            print(out)
        return rc, out, rd
    finally:
        undo()
        cm.install_model_pad_defaults()   # C1：回到模型基线（显式，不再靠隐式默认）


def phases_of(rd):
    return [p["phase"] for p in rd.phases]


# ---- C1 干跑
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, dry_run=True)
    check("干跑：rc 0 且停在 compute", rc == 0 and
          phases_of(rd) == ["preflight", "views", "authorize", "observe",
                            "compute"], repr(phases_of(rd)))
    check("干跑：run.sh 已写出但不执行",
          os.path.exists(rd.p("run.sh")) and
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls), repr([c for c in b.calls
                                           if c[0] == "run_script"]))
    runsh = open(rd.p("run.sh")).read()
    check("干跑：run.sh 带令牌守卫线",
          "real_motion_authorize.py --check-token" in runsh and
          "--min-remaining" in runsh)
    presh = open(rd.p("pre.sh")).read()
    check("跑前只拍 pick/pad 两机位",
          "01-view-pick" in presh and "02-view-pad" in presh and
          "view-free" not in presh, presh[:400])
    check("跑后三机位只追加一次（06/07/08，次序 pick→pad→free）",
          all(("%02d-view-%s" % (i, n)) in runsh
              for i, n in ((6, "pick"), (7, "pad"), (8, "free"))),
          runsh[-600:])
    check("板上分析：2D 标定（pad-mat）随轮推一次、md5 走 push",
          [p for p in b.pushed if p.endswith("analysis/stage2-mat.json")] ==
          ["/b/analysis/stage2-mat.json"], repr(b.pushed))

# ---- C1c 抓点：实测块位姿必须进 plan_sequence（不是标称坐标）
# 回归 2026-11-12 S0:1 r1 attempt5：跑前 burst 量出实测（写进 e9.from_xy），
# 编排器却把布局表标称坐标喂给 plan_sequence ⇒ IK 目标差 25 mm ⇒ 空夹
# （夹爪回读 0.0 mm）。计划里的 from_xy 与真正抓的位置必须是同一份。
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, dry_run=True,
                      deps={"truth_from_view": fake_truth_offset(20.0, -15.0)})
    want = {str(bl["id"]): [bl["x"] + 0.020, bl["y"] - 0.015]
            for bl in lay.layout_blocks("S0", 1)}
    got = {}
    for p in PLAN_CALLS:
        for a in p.get("acts") or []:
            got.setdefault(str(a["block"]), a["block_xy"])
    check("抓点：plan_sequence 吃实测合并后的块位姿（不是标称）",
          rc == 0 and got and all(
              got.get(k) is not None
              and abs(got[k][0] - want[k][0]) < 1e-6
              and abs(got[k][1] - want[k][1]) < 1e-6 for k in want),
          repr({k: got.get(k) for k in want}))

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, dry_run=True,
                      deps={"truth_from_view": fake_truth_offset(20.0, -15.0)})
    acts = [a for p in PLAN_CALLS for a in (p.get("acts") or [])]
    check("抓点：from_xy 与传进规划的那块位置一致（量了就用同一份）",
          bool(acts) and all(
              a["from_xy"] is not None and a["block_xy"] is not None
              and abs(a["from_xy"][0] - a["block_xy"][0]) < 1e-6
              and abs(a["from_xy"][1] - a["block_xy"][1]) < 1e-6
              for a in acts), repr(acts[:2]))

# ---- C1b 分析段在哪算：正式轮必须在板上（本机不承担运行时任务）
with tempfile.TemporaryDirectory() as td:
    try:
        make_args(td, analysis="local")
        check("--analysis local 不带 --dry-run ⇒ 直接拒（正式轮必须在板上算）",
              False)
    except SystemExit:
        check("--analysis local 不带 --dry-run ⇒ 直接拒（正式轮必须在板上算）",
              True)

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, dry_run=True, analysis="local")
    dirs = [p for k, p in b.pulls if k == "dir"]
    check("--analysis local（离线排练）：burst 拉回本机",
          any(p.endswith("wrist-pre/pad") for p in dirs) and
          any(p.endswith("wrist-pre/pick") for p in dirs), repr(dirs))
    check("--analysis local：真值/解垫用本机路径（不是板上路径）",
          TRUTH_CALLS and all("/pre/" in c[0] for c in TRUTH_CALLS),
          repr(TRUTH_CALLS))

# ---- C2 全绿
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b)
    check("全绿：rc 0、八个阶段齐全",
          rc == 0 and phases_of(rd) == list(r.PHASES), repr(phases_of(rd)))
    runs = [c for c in b.calls if c[0] == "run_script"]
    check("全绿：pre.sh 与 run.sh 各一次、顺序对",
          [os.path.basename(c[1]) for c in runs] == ["pre.sh", "run.sh"],
          repr(runs))
    check("全绿：run.sh 输出含 ALL DONE 才认", "ALL DONE" in out)
    # 板上执行日志/机位帧只追加不清理：重跑前必须清，否则判分读进上一次尝试。
    cl = [i for i, c in enumerate(b.calls) if c[0] == "clear"]
    run_i = [i for i, c in enumerate(b.calls) if c[0] == "run_script"]
    paths = [c[1] for c in b.calls if c[0] == "clear"]
    check("干净起跑：pre/exec 与三个机位在跑前被清",
          len(cl) >= 5 and any(p.endswith("/pre/exec") for p in paths) and
          sum("wrist-" in p for p in paths) >= 4, repr(paths))
    check("干净起跑：每次清都在对应 run_script 之前",
          all(any(i < j for j in run_i) for i in cl), repr((cl, run_i)))
    # 2026-11-12 用户定调：本机不承担运行时任务 ⇒ 轮内（观察/计算/执行/判分）
    # 一帧图像都不拉；分析在板上算、只回传 JSON，五组 burst 全在**归档段**
    # （整轮结束、令牌窗口之外）才归档。
    run_i = [i for i, c in enumerate(b.calls)
             if c[0] == "run_script" and c[1].endswith("run.sh")]
    dir_i = [i for i, c in enumerate(b.calls)
             if c[0] == "pull_dir" and "wrist-" in c[1]]
    check("轮内不拉任何 burst（图像不占令牌窗口）",
          bool(run_i) and dir_i and min(dir_i) > max(run_i),
          repr((run_i[-1:], dir_i)))
    check("归档段把五组 burst 都拉回（pre pick/pad + post 三机位）",
          sum(1 for c in b.calls if c[0] == "pull_dir"
              and "wrist-" in c[1]) == 5,
          repr([c[1] for c in b.calls if c[0] == "pull_dir"]))
    check("干净起跑：execute 前也清（exec + 三个跑后机位）",
          any(p.endswith("/exec") for p in paths) and
          sum(p.endswith(("wrist-post/pick", "wrist-post/free",
                          "wrist-post/pad")) for p in paths) == 3,
          repr(paths))
    man = json.load(open(rd.p("plan", "manifest-0.json")))
    cmd = [a for a in man["actions"] if a["kind"] == "target"][0]
    check("manifest：命令 = 期望 − b̂，且期望仍是格心",
          abs(cmd["commanded_center_mm"][0]
              - (cmd["desired_center_mm"][0] + 2.791)) < 1e-6 and
          abs(cmd["commanded_center_mm"][1]
              - (cmd["desired_center_mm"][1] + 0.248)) < 1e-6,
          repr(cmd))
    check("manifest：applied_bias 与 --bias-seed 同值（整轮 fixed）",
          man["bias_mm"] == [-2.791, -0.248], repr(man.get("bias_mm")))
    check("manifest：gap_check 记 layout_gap，不是 probe_verified",
          man["gap_check"] == "layout_gap")
    check("plan_sequence 逐目标 wrist_survey=False",
          PLAN_CALLS and all(p["wrist_survey"] is False for p in PLAN_CALLS),
          repr(PLAN_CALLS))
    check("plan_sequence 的 start 逐段递增（计划名不撞）",
          [p["start"] for p in PLAN_CALLS] == [0, 1, 2, 3, 4], repr(PLAN_CALLS))
    check("收尾支只交给最后一个目标，且就是三机位目标",
          [p["tail_targets"] is None for p in PLAN_CALLS]
          == [True, True, True, True, False]
          and len(PLAN_CALLS[-1]["tail_targets"]) == 6,
          repr([None if p["tail_targets"] is None
                else len(p["tail_targets"]) for p in PLAN_CALLS]))
    bias = json.load(open(rd.p("bias.json")))
    # 样本 = 彩色质心 − 命令 = (期望 + 彩偏) − (期望 − 种子) = 彩偏 + 种子
    want = [1.0 + (-2.791), 0.5 + (-0.248)]
    check("判分：逐块 b̂ = 彩色质心 − 命令 = 彩偏 + 种子",
          bias["samples_mm"] and all(
              abs(s["bias_mm"][0] - want[0]) < 5e-3 and
              abs(s["bias_mm"][1] - want[1]) < 5e-3
              for s in bias["samples_mm"]), repr(bias.get("samples_mm")))
    est = os.path.join(td, "run", "bias_estimate.json")
    check("跑完写回 <run>/bias_estimate.json（下一轮种子）",
          os.path.exists(est) and
          abs(json.load(open(est))["bias_mm"][0] - want[0]) < 5e-3)
    timing = json.load(open(rd.p("timing.json")))
    check("timing：R1 / R2 / exec_est 都落盘",
          timing["r1"]["ok"] and timing["r2"]["ok"] and
          timing["exec_est_s"] > 0, repr(timing))
    check("摆位偏差：用跑前 pick burst 出 record-only 表（不缺数就不跳过）",
          len(DEVIATION_CALLS) == 1
          and "--truth" in DEVIATION_CALLS[0]
          and DEVIATION_CALLS[0][DEVIATION_CALLS[0].index("--out") + 1]
          .endswith("placement-deviation.json"), repr(DEVIATION_CALLS))
    # 2026-11-12 S0:1 r1：free 机位本来就没块，真值工具零块 ⇒ rc=2 ⇒ 该机位
    # 被当「缺证据」剔出溯源，「全桌终态」只剩 pad 一只眼。跑后三机位必须允许空；
    # 跑前 pick 不允许空（那里必须有块，零块就是分割/相机坏了）。
    # 分析缺省在板上算（``--analysis board``）⇒ burst 是**板上**路径。
    post = [c for c in TRUTH_CALLS if c[1] is True]
    pre = [c for c in TRUTH_CALLS if c[1] is False]
    check("跑后三机位真值都带 allow_empty（干净的机位也是证据）",
          len(post) == 3 and all("wrist-post" in c[0] for c in post),
          repr(TRUTH_CALLS))
    check("跑前 pick 真值不允许空（零块就是坏，不许静默）",
          len(pre) == 1 and "wrist-pre" in pre[0][0], repr(TRUTH_CALLS))

# ---- C3 R1 不足
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.tokens = [(("t" * 32), 100.0, "")]
    rc, out, rd = run(td, b)
    check("R1 不足 ⇒ rc 3、停在 compute", rc == r.EXIT_GATE and
          phases_of(rd)[-1] == "compute", repr((rc, phases_of(rd))))
    check("R1 不足 ⇒ 一条执行都没发",
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls))
    check("R1 不足 ⇒ 写 timing.json 供重标",
          json.load(open(rd.p("timing.json")))["r1"]["ok"] is False)

# ---- C3b 干跑：R1 不预留执行时间（否则用**还没测出来的** T_exec_est 拦死
# 量 T_probe/T_plan 的那一步；缺省 210+450+60=720 s > 600 s TTL，永远过不去）
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.tokens = [(("t" * 32), 450.0, "")]
    rc, out, rd = run(td, b, dry_run=True)
    timing = json.load(open(rd.p("timing.json")))
    check("干跑：令牌 450 s 也能过 R1（不用未测的执行估时拦自己）",
          rc == 0 and timing["r1"]["ok"] and
          timing["r1"]["gate_exec_est_s"] == 0.0, repr(timing.get("r1")))
    check("干跑：exec_est 仍按先验记下来（供标定，不是丢弃）",
          timing["r1"]["exec_est_s"] > 0, repr(timing.get("r1")))

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.tokens = [(("t" * 32), 450.0, "")]
    rc, out, rd = run(td, b)
    check("同一条令牌 450 s：正式轮 R1 不过（干跑的豁免不泄漏）",
          rc == r.EXIT_GATE, repr(rc))

# ---- C4 R2 不足
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.tokens = [(("t" * 32), 750.0, ""), (("t" * 32), 100.0, "")]
    rc, out, rd = run(td, b)
    check("R2 不足 ⇒ rc 3、停在 compute",
          rc == r.EXIT_GATE and phases_of(rd)[-1] == "compute",
          repr((rc, phases_of(rd))))
    check("R2 不足 ⇒ 不发执行",
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls))
    t = json.load(open(rd.p("timing.json")))
    check("timing 记 R1 通过、R2 不过", t["r1"]["ok"] and
          t["r2"]["ok"] is False, repr(t))

# ---- C5 预检红
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.preflight_doc = dict(b.preflight_doc,
                           gripper={"ok": True, "init_state": 0})
    rc, out, rd = run(td, b)
    check("预检红（夹爪未初始化）⇒ rc 2、停在 preflight",
          rc == r.EXIT_MISSING and phases_of(rd) == ["preflight"],
          repr((rc, phases_of(rd))))
    check("预检红 ⇒ 不进入授权、不跑任何脚本",
          not any(c[0] in ("wait_token", "run_script") for c in b.calls))

# ---- C6 缺 b̂ 种子
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, bias_seed=False)
    check("缺 b̂ 种子且未显式 none ⇒ rc 2、停在 compute",
          rc == r.EXIT_MISSING and phases_of(rd)[-1] == "compute",
          repr((rc, phases_of(rd))))
    check("缺 b̂ 种子 ⇒ 不发执行",
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls))

# ---- C7 执行失败不重试
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.run_results = [(3, "boom")]
    rc, out, rd = run(td, b)
    runs = [c for c in b.calls if c[0] == "run_script"
            and c[1].endswith("run.sh")]
    check("执行失败 ⇒ rc 5、停在 execute", rc == r.EXIT_EXEC and
          phases_of(rd)[-1] == "execute", repr((rc, phases_of(rd))))
    check("执行失败 ⇒ run.sh 只调一次（不自动重试）", len(runs) == 1,
          repr(runs))
    check("执行失败 ⇒ 仍回拉 jsonl 证据（KB 级）",
          any(k == "file" and p.endswith(".jsonl") for k, p in b.pulls),
          repr(b.pulls))
    check("执行失败 ⇒ 不进归档段（burst 留在板上，随时补拉）",
          not any(k == "dir" and "wrist-" in p for k, p in b.pulls),
          repr(b.pulls))

# ---- C8 执行中令牌过期 ⇒ void
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.run_results = [(9, "!!!!! STOP: 令牌剩余不足 20 秒（rc=1）—— 先重做 STOP→PLAY；"
                         "本轮 void（aborted_token），不算抓取失败")]
    rc, out, rd = run(td, b)
    row = [p for p in rd.phases if p["phase"] == "execute"][0]
    check("执行中令牌过期 ⇒ rc 3（不是 5）", rc == r.EXIT_GATE, repr(rc))
    check("且本轮标 void=true（aborted_token，不记抓取失败）",
          row.get("void") is True, repr(row))
    check("aborted_token 的提示写明不算抓取失败", "不算抓取失败" in out)

# ---- C8b 观察段缺目录之类的普通失败 ≠ 令牌问题（2026-11-11 Round 0 误诊）
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.pre_results = [(9, "===== 令牌剩余时间守卫（要求 ≥ 30 s）\n"
                         "Traceback (most recent call last):\n"
                         "FileNotFoundError: [Errno 2] No such file or dir")]
    rc, out, rd = run(td, b)
    check("观察段普通失败（输出里只是守卫说明）⇒ rc 5，不误报令牌",
          rc == r.EXIT_EXEC, repr(rc))
    check("观察段普通失败不写 aborted_token/void",
          "void" not in out and "aborted_token" not in out, out[-200:])

# ---- C8c 板上只写不推的目录必须显式建（pre/exec、exec）
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    run(td, b)
    made = [c[1] for c in b.calls if c[0] == "mkdir"]
    check("观察段建了 pre/exec（precision_exec 写执行日志要用）",
          any(p.endswith("/pre/exec") for p in made), repr(made))
    check("执行段建了 exec", any(p.endswith("/exec") and
                                 not p.endswith("/pre/exec") for p in made),
          repr(made))

# ---- C9 判分不完整
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, deps={"round_report": lambda argv: 2})
    check("判分不完整 ⇒ rc 6、停在 score", rc == r.EXIT_SCORE and
          phases_of(rd)[-1] == "score", repr((rc, phases_of(rd))))

# ---- C9b --facts 给了但文件不在 ⇒ 明说缺、不许当「都没发生」
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, facts=os.path.join(td, "nope.json"))
    check("facts 缺失 ⇒ rc 6 且报「facts 输入不存在」（不默认成空字典）",
          rc == r.EXIT_SCORE and "facts 输入不存在" in out,
          repr((rc, out[-300:])))

# ---- C10 探针问题


def patch_probe(board, mutate):
    """换掉 ``board.probe_report``，返回一份改过的报告。

    **必须先抓住原方法再替换**：在补丁函数里直接写 ``b.probe_report()`` 会调到
    它自己。2026-11-11 实测：RecursionError 被 compute 的兜底 ``except`` 接住，
    照样以 rc 4 停在 compute ⇒「要求清障 / 格不符 / 幽灵块」三条**因为错的原因
    变绿**（只有断言了报文文本的那条才暴露）。
    """
    orig = board.probe_report

    def patched():
        doc = orig()
        mutate(doc)
        return doc
    board.probe_report = patched


with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()

    def _clearing(doc):
        doc["scenes"][0]["attempts"][2]["clearing_steps"] = [{"block": 0}]
    patch_probe(b, _clearing)
    rc, out, rd = run(td, b)
    check("探针要求清障 ⇒ rc 4（累积清障未实现，不假装能跑）",
          rc == r.EXIT_PLAN and phases_of(rd)[-1] == "compute"
          and "清障" in out, repr((rc, out[-300:])))
    check("探针报清障 ⇒ 不发执行",
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls))

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()

    def _bad_cell(doc):
        doc["scenes"][0]["attempts"][0]["cell"] = 7
    patch_probe(b, _bad_cell)
    rc, out, rd = run(td, b)
    check("探针格与本轮格不符 ⇒ rc 4",
          rc == r.EXIT_PLAN and "探针格" in out, repr((rc, out[-300:])))

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()

    def _dirty(doc):
        doc["scene_cleanup"] = {"ok": False, "world": ["block-0"],
                                "attached": []}
    patch_probe(b, _dirty)
    rc, out, rd = run(td, b)
    check("探针留下幽灵块 ⇒ rc 4（先清场景）",
          rc == r.EXIT_PLAN and "幽灵块" in out, repr((rc, out[-300:])))

# 规划组写错**不报错**，只是每个目标 0.4 s 秒失败（2026-11-11 Round 0：缺省是
# 仿真组 ur5_manipulator，真机上白烧一次授权）
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()

    def _sim_group(doc):
        doc["group"] = "ur5_manipulator"
    patch_probe(b, _sim_group)
    rc, out, rd = run(td, b)
    check("探针用了仿真规划组 ⇒ rc 4（不是当成场景无解）",
          rc == r.EXIT_PLAN and "仿真" in out, repr((rc, out[-300:])))

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()

    def _other_group(doc):
        doc["group"] = "some_other_group"
    patch_probe(b, _other_group)
    rc, out, rd = run(td, b)
    check("探针规划组与 --probe-group 不符 ⇒ rc 4",
          rc == r.EXIT_PLAN and "规划组" in out, repr((rc, out[-300:])))

# --probe-report：复用一份已跑过的报告 ⇒ 本轮不碰板端探针（只读、不占令牌）
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    reuse = os.path.join(td, "probe-reuse.json")
    with open(reuse, "w") as f:
        json.dump(b.probe_report(), f)
    rc, out, rd = run(td, b, probe_report=reuse)
    check("--probe-report 复用 ⇒ 不调板端探针",
          not any(c[0] == "probe" for c in b.calls), repr(b.calls[:6]))
    check("--probe-report 复用 ⇒ 照常出计划并跑完干跑", rc == 0, repr(rc))

# ---- C10c 干跑不要求令牌（2026-11-11 用户定调：先离线把计算链跑通）
# 干跑只到 compute、从不发轨迹 ⇒ R1/R2 没有保护对象。反过来，要求令牌就会逼人
# 为一次纯计算按 STOP→PLAY，而且机械臂关机时连代码都验不了。
def _seed_then(td, **over):
    """先整跑一遍干跑把 ``pre/`` 产物铺好，再清空调用记录跑 ``--from compute``。

    只想验 compute 段，但 compute 要读观察段落的 ``q_after_observe.json`` 与
    burst 目录 —— 这是 ``--from compute`` 的真实前置，不能靠桩假装。
    """
    b = FakeBoard()
    run(td, b, dry_run=True)
    reuse = os.path.join(td, "probe-reuse.json")
    with open(reuse, "w") as f:
        json.dump(b.probe_report(), f)
    b.calls = []
    b.tokens = [(None, None, "没有令牌（机械臂关机）")]
    rc, out, rd = run(td, b, dry_run=True, from_="compute", to="compute",
                      probe_report=reuse, **over)
    return b, rc, out, rd, reuse


with tempfile.TemporaryDirectory() as td:
    b, rc, out, rd, _reuse = _seed_then(td)
    check("干跑 + 没有令牌 ⇒ 仍跑到 compute 收尾（rc 0）", rc == 0, repr(rc))
    tj = json.load(open(rd.p("timing.json")))
    check("干跑 + 没有令牌 ⇒ timing 记 skipped=dry_run（不是假装通过）",
          tj["r1"].get("skipped") == "dry_run"
          and tj["r1"].get("ok") is True, repr(tj.get("r1")))

# 干跑 + 复用探针报告 ⇒ **纯离线**：一次板子调用都不许有（机械臂关机也能验）
with tempfile.TemporaryDirectory() as td:
    b, rc, out, rd, _reuse = _seed_then(td)
    check("干跑 + 复用探针 ⇒ 完全离线（无任何板子调用）",
          rc == 0 and not b.calls, repr((rc, b.calls[:6])))

# 非干跑同一条没有令牌 ⇒ 仍然 fail-closed（干跑的豁免不许泄漏）
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    run(td, b, dry_run=True)
    reuse = os.path.join(td, "probe-reuse.json")
    with open(reuse, "w") as f:
        json.dump(b.probe_report(), f)
    b.calls = []
    b.tokens = [(None, None, "没有令牌（机械臂关机）")]
    rc, out, rd = run(td, b, from_="compute", to="compute", probe_report=reuse)
    check("非干跑 + 没有令牌 ⇒ rc 3（跳过的只有干跑）",
          rc == r.EXIT_GATE, repr((rc, out[-200:])))

# --from 重跑单段 ⇒ 前几段的证据**不许被覆盖**（2026-11-11 Round 0 实测丢过）
with tempfile.TemporaryDirectory() as td:
    b, rc, out, rd, _reuse = _seed_then(td)
    names = [p["phase"] for p in rd.phases]
    check("--from compute：phases.json 保留前四段与新 compute 段",
          names == ["preflight", "views", "authorize", "observe", "compute"],
          repr(names))
    tj = json.load(open(rd.p("timing.json")))
    check("--from compute：timing 保留观察段秒数（不丢证据）",
          tj.get("observe_s") is not None, repr(tj))

# ---- C11 --from/--to
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, to="observe")
    check("--to observe：只跑前四段",
          [p["phase"] for p in rd.phases] == list(r.PHASES[:4]),
          repr([p["phase"] for p in rd.phases]))
    check("--to observe：compute 没跑 ⇒ 没有 run.sh",
          not os.path.exists(rd.p("run.sh")))

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, dry_run=True)
    args = rd.args
    args.from_ = "archive"
    args.to = "archive"
    args.dry_run = False
    undo = patched(None)
    try:
        rd2 = r.Round(args, b, deps=base_deps())
        rc2, out2 = quiet(rd2.run)
    finally:
        undo()
        cm.install_model_pad_defaults()   # C1：回到模型基线（显式，不再靠隐式默认）
    before = [p["phase"] for p in rd.phases]
    check("--from/--to archive：只跑归档一段（前几段保留不重跑）",
          [p["phase"] for p in rd2.phases] == before + ["archive"] and rc2 == 0,
          repr((rc2, [p["phase"] for p in rd2.phases])))

# ---- C12 |b̂|>15 mm 记失败、不当偏置
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, deps={"measure_colour": fake_colour(offset=(20.0, 0.0))})
    bias = json.load(open(rd.p("bias.json")))
    check("|b̂|>15 mm ⇒ 记 failures，不进 samples",
          bias["samples_mm"] == [] and len(bias["failures_mm"]) == 5,
          repr((bias["samples_mm"], bias["failures_mm"])))
    check("|b̂|>15 mm ⇒ 不写 bias_estimate",
          not os.path.exists(os.path.join(td, "run", "bias_estimate.json")))
    check("|b̂|>15 mm 仍能完成判分（失败分类交给 round_report）", rc == 0,
          repr(rc))

# ---- C13 --from score 单步重跑：判分必须装**同一份**实测垫（不是模型常量）
# 2026-11-12 真机 S0:1 r1：执行段炸在最后一张 pad 照片上，用 --from score 重跑
# 判分时进程没跑过 compute，cm 还在模型常量（与印刷垫差 87.5 mm / 88.7°）——
# 彩色量按模型格心开窗，5 块里 3 块「窗内没看见彩色块」、2 块量到邻块。
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc_a, _, rd_a = run(td, b)
    # 把本轮 grid-pre.json 换成一份**与模型常量不同**的实测垫（真机就是这样：
    # 2026-11-12 实测中心 471.94, −157.6 mm / −91.623°，模型是 424.2, −112.24
    # mm / −179.778°）。pad_ok 是测试夹具，恰好等于模型常量，所以必须换。
    alt = dict(pad_ok, grid_center_base_mm=[471.94, -157.6],
               grid_yaw_base_deg=-91.623)
    gp = rd_a.p("grid-pre.json")
    with open(gp, "w") as f:
        json.dump(alt, f)
    model = {c: [v * 1000.0 for v in cm.cell_center(c)] for c in (8, 6, 4, 2, 0)}
    cm.load_grid(gp)
    want = {c: [v * 1000.0 for v in cm.cell_center(c)] for c in (8, 6, 4, 2, 0)}
    cm.install_model_pad_defaults()       # C1：回到模型基线（显式）
    seen_score = []

    def rec_score(burst, xy_mm, top_z, expect_class, hue_deg, search_mm):
        seen_score.append(list(xy_mm))
        return {"ok": True, "xy_mm": [xy_mm[0] + 1.0, xy_mm[1] + 0.5],
                "hue_deg": 30.0, "n_frames": 8, "n_missed": 0}

    rc_b, out_b, rd_b = run(td, b, deps={"measure_colour": rec_score},
                            from_="score", to="score")
    check("--from score：rc 0（判分可单步重跑）", rc_b == 0, repr((rc_b, out_b[-300:])))
    check("--from score：开窗 = grid-pre.json 的格心（同一份实测垫）",
          len(seen_score) == 5 and all(
              abs(s[0] - want[c][0]) < 1e-6 and abs(s[1] - want[c][1]) < 1e-6
              for s, c in zip(seen_score, (8, 6, 4, 2, 0))),
          repr((seen_score, want)))
    check("--from score：开窗**不是**模型常量格心（回归：曾整盘错 87 mm）",
          all(any(abs(s[j] - model[c][j]) > 20.0 for j in (0, 1))
              for s, c in zip(seen_score, (8, 6, 4, 2, 0))),
          repr((seen_score, model)))

# ---- C14 round_report 的 --manifest 必须逐份带开关（action="append"）
# 2026-11-12 真机 S0:1 r1：写成 ["--manifest", p1, p2, ...] ⇒ argparse 只认
# 第一个，其余全落 unrecognized，判分整段 rc=2。
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    seen_argv = []

    def rec_report(argv):
        seen_argv.append(list(argv))
        return 0

    rc, out, rd = run(td, b, deps={"round_report": rec_report})
    argv = seen_argv[0] if seen_argv else []
    n_flag = argv.count("--manifest")
    man_paths = [argv[i + 1] for i, a in enumerate(argv[:-1])
                 if a == "--manifest"]
    check("round_report：5 份 manifest 各带一次 --manifest",
          n_flag == 5 and all(os.path.exists(p) for p in man_paths),
          repr((n_flag, man_paths)))
    check("round_report：--manifest 后不跟别人的参数（旧写法会吞 4 份）",
          rc == 0 and len(set(man_paths)) == 5, repr(rc))

# ---- C15 真值保留区按机位取（不喂整张台）
# 2026-11-12 真机 S0:1 r1：自由区机位把示教器当成一块 42 mm 高的青色块
# （785, 509 mm），旧大区（x ≤ 0.80）放它进来 ⇒ 全桌终态假红。
z_free = r.truth_zone_for("/x/post/wrist-free")
z_pick = r.truth_zone_for("/x/pre/wrist-pick")
check("保留区：free = 本机位 zone 外扩 25 mm",
      z_free == "0.255,0.775,0.435,0.645", z_free)
check("保留区：pick = 本机位 zone 外扩 25 mm",
      z_pick == "0.225,0.600,0.145,0.445", z_pick)
check("保留区：示教器幻影点 (785, 509) mm 在 free 区外",
      not (0.255 <= 0.7847 <= 0.775 and 0.435 <= 0.5094 <= 0.645))
check("保留区：认不出机位 ⇒ 兜底大区（不猜）",
      r.truth_zone_for("/x/nope") == r.TRUTH_ZONE)

_tf_calls = []
_orig_tf_main = r.tf.main
r.tf.main = lambda argv: _tf_calls.append(list(argv)) or 0
try:
    r.default_truth_from_view("/x/post/wrist-pad", "/tmp/o.json")
    r.default_truth_from_view("/x/post/wrist-pad", "/tmp/o.json",
                              allow_empty=True)
    r.default_truth_from_view("/x/pre/wrist-pick", "/tmp/o.json")
finally:
    r.tf.main = _orig_tf_main
_z = (_tf_calls[0][_tf_calls[0].index("--zone") + 1]
      if _tf_calls and "--zone" in _tf_calls[0] else None)
check("default_truth_from_view 把机位区传给真值工具",
      _z == r.truth_zone_for("/x/post/wrist-pad"), repr((_z, _tf_calls)))
check("allow_empty 才加 --allow-empty（默认不加）",
      "--allow-empty" not in _tf_calls[0]
      and "--allow-empty" in _tf_calls[1]
      and "--allow-empty" not in _tf_calls[2], repr(_tf_calls))

# =====================================================================
print()
print("通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(0 if N_FAIL == 0 else 1)
